#!/usr/bin/env python3
"""Measure chassis motion in a freshly launched tomato field.

Use an isolated ROS_DOMAIN_ID and GZ_PARTITION. By default this only checks
readiness; --execute explicitly enables /cmd_vel publication. Gazebo model
poses are acceptance-test truth, never an input to the robot's algorithms.
The limits below are simulation regression limits, not hardware calibration.
"""

import argparse
from collections import deque
import csv
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import threading
import time


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_progress(angles):
    """Accumulate signed rotation; equal wrapped endpoints cannot prove a turn."""
    return sum(wrap_angle(b - a) for a, b in zip(angles, angles[1:]))


def rotation_passed(angles, target, tolerance=math.radians(3)):
    return bool(len(angles) >= 2 and abs(yaw_progress(angles) - target) <= tolerance)


def quaternion_angles(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                      1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


@dataclass(frozen=True)
class Pose:
    stamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


def align_odometry(pose, initial_odom, initial_truth):
    """Apply the initial odom-to-world rigid transform to an odometry pose."""
    angle = initial_truth.yaw - initial_odom.yaw
    dx, dy = pose.x - initial_odom.x, pose.y - initial_odom.y
    return Pose(pose.stamp,
                initial_truth.x + math.cos(angle) * dx - math.sin(angle) * dy,
                initial_truth.y + math.sin(angle) * dx + math.cos(angle) * dy,
                initial_truth.z + pose.z - initial_odom.z,
                pose.roll, pose.pitch, wrap_angle(pose.yaw + angle))


def expected_endpoint(start, velocity, yaw_rate, duration):
    turn = yaw_rate * duration
    if abs(yaw_rate) < 1e-9:
        dx = velocity * duration * math.cos(start.yaw)
        dy = velocity * duration * math.sin(start.yaw)
    else:
        radius = velocity / yaw_rate
        dx = radius * (math.sin(start.yaw + turn) - math.sin(start.yaw))
        dy = radius * (math.cos(start.yaw) - math.cos(start.yaw + turn))
    return Pose(start.stamp + duration, start.x + dx, start.y + dy, start.z,
                start.roll, start.pitch, wrap_angle(start.yaw + turn))


def stop_metrics(poses, minimum_duration=0.5):
    """Check sustained rest using consecutive truth samples, not odometry alone."""
    if len(poses) < 2:
        return {'passed': False, 'reason': 'too few ground-truth samples'}
    linear, angular = [], []
    for a, b in zip(poses, poses[1:]):
        dt = b.stamp - a.stamp
        if dt > 0:
            linear.append(math.hypot(b.x - a.x, b.y - a.y) / dt)
            angular.append(abs(wrap_angle(b.yaw - a.yaw)) / dt)
    span = poses[-1].stamp - poses[0].stamp
    max_linear = max(linear, default=math.inf)
    max_angular = max(angular, default=math.inf)
    max_gap = max((b.stamp - a.stamp for a, b in zip(poses, poses[1:])), default=math.inf)
    return {
        'passed': span >= minimum_duration - 1e-6
                  and max_gap <= 0.1 + 1e-6 and max_linear < 0.01 and max_angular < 0.02,
        'observed_duration_s': span,
        'max_truth_speed_m_s': max_linear,
        'max_truth_yaw_rate_rad_s': max_angular,
        'maximum_truth_sample_gap_s': max_gap,
        'truth_drift_m': math.hypot(poses[-1].x - poses[0].x,
                                   poses[-1].y - poses[0].y),
    }


def command_timeout_metrics(before, after, last_command_stamp, deadline=0.6):
    """Verify that a moving robot reaches sustained rest by the watchdog deadline."""
    before_duration = before[-1].stamp - before[0].stamp if len(before) >= 2 else 0.0
    prestop_speed = (math.hypot(before[-1].x - before[0].x, before[-1].y - before[0].y)
                     / before_duration if before_duration > 0 else 0.0)
    rest_start, sustained_rest_start = None, None
    before_gap = max((b.stamp - a.stamp for a, b in zip(before, before[1:])), default=math.inf)
    after_gap = max((b.stamp - a.stamp for a, b in zip(after, after[1:])), default=math.inf)
    for a, b in zip(after, after[1:]):
        dt = b.stamp - a.stamp
        if dt <= 0 or dt > 0.1 + 1e-6:
            rest_start = None
            continue
        linear = math.hypot(b.x - a.x, b.y - a.y) / dt
        angular = abs(wrap_angle(b.yaw - a.yaw)) / dt
        if linear < 0.01 and angular < 0.02:
            if rest_start is None:
                rest_start = a.stamp
            if b.stamp - rest_start >= 0.5 - 1e-6:
                sustained_rest_start = rest_start
                break
        else:
            rest_start = None
    delay = (sustained_rest_start - last_command_stamp
             if sustained_rest_start is not None else None)
    return {
        'passed': prestop_speed > 0.05 and delay is not None and 0 <= delay <= deadline
                  and max(before_gap, after_gap) <= 0.1 + 1e-6,
        'prestop_truth_speed_m_s': prestop_speed,
        'last_command_sim_stamp_s': last_command_stamp,
        'sustained_rest_start_sim_stamp_s': sustained_rest_start,
        'watchdog_stop_delay_s': delay,
        'watchdog_setting_s': 0.5,
        'dynamics_and_sampling_tolerance_s': deadline - 0.5,
        'watchdog_stop_deadline_s': deadline,
        'maximum_truth_sample_gap_s': max(before_gap, after_gap),
    }


class PreconditionsError(RuntimeError):
    pass


class MotionFailure(RuntimeError):
    pass


class Observer:
    def __init__(self, execute, progress_timeout, wall_timeout):
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GazeboNode
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState

        self.rclpy, self.message_type = rclpy, TwistStamped
        self.ros = Node('check_chassis_motion', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.lock = threading.Lock()
        self.truth, self.odom, self.joints = deque(maxlen=200000), deque(maxlen=200000), deque(maxlen=200000)
        self.arms = deque(maxlen=200000)
        self.initial_arm_positions = None
        self.nonincreasing_truth_messages = 0
        self.regressing_truth_messages = 0
        self.invalid_pose_messages = 0
        self.initial_truth = None
        self.last_progress = time.monotonic()
        self.progress_timeout = progress_timeout
        self.wall_deadline = time.monotonic() + wall_timeout
        self.max_odom_stamp_difference = 0.0
        self.command, self.publish_enabled = (0.0, 0.0), False
        self.last_command_stamp = None
        self.publisher = self.ros.create_publisher(TwistStamped, '/cmd_vel', 10) if execute else None
        self.subscriptions = [
            self.ros.create_subscription(Odometry, '/wheel/odom', self.on_odom, qos_profile_sensor_data),
            self.ros.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data),
        ]
        self.timer = self.ros.create_timer(0.05, self.publish_command)
        self.gazebo = GazeboNode()
        self.gazebo.subscribe(Pose_V, '/world/field/dynamic_pose/info', self.on_truth)

    def on_truth(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
        for pose in message.pose:
            if pose.name == 'agri_robot':
                angles = quaternion_angles(pose.orientation)
                sample = Pose(stamp, pose.position.x, pose.position.y, pose.position.z, *angles)
                with self.lock:
                    if not all(math.isfinite(value) for value in asdict(sample).values()):
                        self.invalid_pose_messages += 1
                        break
                    if not self.truth or stamp > self.truth[-1].stamp:
                        self.truth.append(sample)
                        self.last_progress = time.monotonic()
                    else:
                        self.nonincreasing_truth_messages += 1
                        if stamp < self.truth[-1].stamp:
                            self.regressing_truth_messages += 1
                break

    def on_odom(self, message):
        p = message.pose.pose
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        sample = Pose(stamp, p.position.x, p.position.y, p.position.z,
                      *quaternion_angles(p.orientation))
        if not all(math.isfinite(value) for value in asdict(sample).values()):
            self.invalid_pose_messages += 1
            return
        self.odom.append(sample)

    def on_joints(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        wheels = {name: abs(velocity) for name, velocity in zip(message.name, message.velocity)
                  if name in ('f1_wheel_joint', 'f2_wheel_joint', 'r1_wheel_joint', 'r2_wheel_joint')}
        self.joints.append((stamp, wheels))
        arms = {name: position for name, position in zip(message.name, message.position)
                if name in ('arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'cr5_joint6')}
        self.arms.append((stamp, arms))

    def publish_command(self):
        if self.publisher is None or not self.publish_enabled:
            return
        message = self.message_type()
        message.header.stamp = self.ros.get_clock().now().to_msg()
        message.header.frame_id = 'base_footprint'
        message.twist.linear.x, message.twist.angular.z = self.command
        self.publisher.publish(message)
        self.last_command_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def poll(self):
        self.rclpy.spin_once(self.ros, timeout_sec=0.02)
        if time.monotonic() > self.wall_deadline:
            raise PreconditionsError('Total wall-clock timeout exceeded')
        if time.monotonic() - self.last_progress > self.progress_timeout:
            raise PreconditionsError('No advancing field ground truth; check Gazebo playback and GZ_PARTITION')
        if self.regressing_truth_messages:
            raise PreconditionsError('Gazebo truth timestamp moved backwards; the world may have been reset')
        if self.invalid_pose_messages:
            raise PreconditionsError('Gazebo truth or odometry contained a nonfinite pose')
        if getattr(self, 'motion_authorized', False):
            pose = self.latest_truth()
            if (abs(pose.x) > 0.65 or not -6.75 <= pose.y <= 6.5
                    or abs(pose.roll) > math.radians(10) or abs(pose.pitch) > math.radians(10)):
                raise MotionFailure('Robot left the central test corridor or tilted more than 10 degrees')
            if abs(pose.z - self.initial_truth.z) > 0.03:
                raise MotionFailure('Robot height changed more than 0.03 m from its grounded starting pose')
            if self.initial_arm_positions and self.arms:
                if any(abs(position - self.initial_arm_positions[name]) > 0.02
                       for name, position in self.arms[-1][1].items()):
                    raise MotionFailure('An arm joint moved more than 0.02 rad from its initial held position')

    def latest_truth(self):
        with self.lock:
            return self.truth[-1] if self.truth else None

    def truth_between(self, start, end):
        with self.lock:
            return [pose for pose in self.truth if start <= pose.stamp <= end]

    def odometry_at(self, stamp):
        if not self.odom:
            raise PreconditionsError('No /wheel/odom received; enable use_kinematics')
        pose = min(self.odom, key=lambda p: abs(p.stamp - stamp))
        difference = abs(pose.stamp - stamp)
        self.max_odom_stamp_difference = max(self.max_odom_stamp_difference, difference)
        if difference > 0.05:
            raise PreconditionsError('Odometry and Gazebo ground truth are not synchronized')
        return pose

    def ensure_no_other_publishers(self):
        others = [info.node_name for info in self.ros.get_publishers_info_by_topic('/cmd_vel')
                  if info.node_name != self.ros.get_name() or info.node_namespace != self.ros.get_namespace()]
        if others:
            raise PreconditionsError('Stop other /cmd_vel publishers before executing: ' + ', '.join(others))

    def wait_ready(self):
        started = time.monotonic()
        while True:
            self.poll()
            if time.monotonic() - started > 30:
                raise PreconditionsError('ROS odometry/joint states and advancing field truth not ready within 30 s')
            truth = self.latest_truth()
            if truth is not None and self.odom and self.joints and time.monotonic() - started >= 2:
                if abs(self.odom[-1].stamp - truth.stamp) <= 0.15:
                    break
        self.ensure_no_other_publishers()
        return truth

    def collect(self, duration, velocity=0.0, yaw_rate=0.0, publish=True):
        self.ensure_no_other_publishers()
        self.command, self.publish_enabled = (velocity, yaw_rate), publish and self.publisher is not None
        self.publish_command()
        start = self.latest_truth()
        while self.latest_truth().stamp - start.stamp < duration:
            self.poll()
        end = self.latest_truth()
        samples = self.truth_between(start.stamp, end.stamp)
        return start, end, samples

    def shutdown_stop(self):
        self.command, self.publish_enabled = (0.0, 0.0), self.publisher is not None
        for _ in range(5):
            self.publish_command()
            self.rclpy.spin_once(self.ros, timeout_sec=0.02)


def run_suite(observer, options):
    observer.wait_ready()
    observer.collect(2.0, publish=False)  # Let the spawn settle without sending commands.
    initial = observer.latest_truth()
    if math.hypot(initial.x, initial.y + 6.0) > 0.25 or abs(wrap_angle(initial.yaw - math.pi / 2)) > 0.1:
        raise PreconditionsError('Start a fresh tomato_field.launch.py at x=0, y=-6, yaw=1.5708')
    if abs(initial.roll) > 0.1 or abs(initial.pitch) > 0.1:
        raise PreconditionsError('Robot has not settled upright on the field ground')
    if abs(initial.z - 0.36) > 0.03:
        raise PreconditionsError('Expected settled CAD model-origin height z=0.36 +/-0.03 m')
    observer.initial_truth = initial
    initial_odom = observer.odometry_at(initial.stamp)
    initial_arms = observer.arms[-1][1] if observer.arms else {}
    if len(initial_arms) != 6:
        raise PreconditionsError('All six arm joint positions must be available for the held-arm check')
    observer.initial_arm_positions = dict(initial_arms)
    readiness = {
        'world': 'field', 'robot': 'agri_robot',
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
        'gz_partition': os.environ['GZ_PARTITION'],
        'initial_world_pose': asdict(initial),
        'initial_odom_pose': asdict(initial_odom),
        'initial_arm_joint_positions_rad': initial_arms,
        'passed': True, 'executed': False,
    }
    if not options.execute:
        return readiness
    observer.motion_authorized = True
    results = []
    observer.results = results
    motion_start = initial.stamp

    def stop(name, publish=True):
        last_command_stamp = observer.last_command_stamp
        before_end = observer.latest_truth().stamp
        prestop_samples = observer.truth_between(before_end - 0.5, before_end)
        start, end, samples = observer.collect(2.0, publish=publish)
        # Include the sample just before the final half-second for a full interval.
        rest_prefix = [p for p in samples if p.stamp <= end.stamp - 0.5]
        first = rest_prefix[-1].stamp if rest_prefix else start.stamp
        metrics = stop_metrics([p for p in samples if p.stamp >= first])
        wheel_messages = [wheels for stamp, wheels in observer.joints
                          if first <= stamp <= end.stamp]
        complete = bool(wheel_messages) and all(len(wheels) == 4 for wheels in wheel_messages)
        wheel_max = max((max(wheels.values(), default=math.inf) for wheels in wheel_messages),
                        default=math.inf)
        metrics.update({
            'name': name, 'kind': 'stop' if publish else 'command_timeout',
            'start_world_pose': asdict(start), 'end_world_pose': asdict(end),
            'stop_total_drift_m': math.hypot(end.x - start.x, end.y - start.y),
            'stop_total_yaw_drift_deg': math.degrees(abs(yaw_progress([p.yaw for p in samples]))),
            'max_wheel_speed_rad_s': wheel_max, 'all_four_wheels_observed': complete,
        })
        metrics['passed'] = metrics['passed'] and complete and wheel_max < 0.08
        if not publish:
            watchdog = command_timeout_metrics(prestop_samples, samples, last_command_stamp)
            metrics['watchdog'] = watchdog
            metrics['passed'] = metrics['passed'] and watchdog['passed']
        if publish and results and results[-1]['kind'] in ('motion', 'full_turn'):
            previous_motion = results[-1]
            target = previous_motion['target_world_pose']
            settled_position_error = math.hypot(end.x - target['x'], end.y - target['y'])
            settled_heading_error = abs(wrap_angle(end.yaw - target['yaw']))
            settled_passed = settled_position_error <= previous_motion['position_error_limit_m'] \
                             and settled_heading_error <= math.radians(3)
            previous_motion.update({
                'settled_end_world_pose': asdict(end),
                'settled_position_error_m': settled_position_error,
                'settled_heading_error_deg': math.degrees(settled_heading_error),
            })
            if previous_motion['kind'] == 'full_turn':
                settled_samples = observer.truth_between(previous_motion['start_world_pose']['stamp'], end.stamp)
                settled_progress = yaw_progress([pose.yaw for pose in settled_samples])
                settled_gap = max((b.stamp - a.stamp for a, b in zip(settled_samples, settled_samples[1:])),
                                  default=math.inf)
                previous_motion['settled_continuous_yaw_progress_deg'] = math.degrees(settled_progress)
                settled_passed = settled_passed and abs(settled_progress - 2 * math.pi) <= math.radians(3) \
                                 and settled_gap <= 0.1 + 1e-6
            previous_motion['passed'] = previous_motion['passed'] and settled_passed
            print(f"{previous_motion['name']} settled: {'PASS' if settled_passed else 'FAIL'} "
                  f"position error={settled_position_error:.3f} m, "
                  f"heading error={math.degrees(settled_heading_error):.2f} deg", flush=True)
        results.append(metrics)
        print(f"{name}: {'PASS' if metrics['passed'] else 'FAIL'} "
              f"rest speed={metrics.get('max_truth_speed_m_s', math.inf):.4f} m/s", flush=True)

    def motion(name, velocity, yaw_rate, duration, full_turn=False, stop_after=True):
        start, end, samples = observer.collect(duration, velocity, yaw_rate)
        # Immediately stop refreshing the previous motion command between stages.
        if stop_after:
            observer.command = (0.0, 0.0)
            observer.publish_command()
        actual_duration = end.stamp - start.stamp
        target = expected_endpoint(start, velocity, yaw_rate, duration)
        position_error = math.hypot(end.x - target.x, end.y - target.y)
        heading_error = abs(wrap_angle(end.yaw - target.yaw))
        angular_progress = yaw_progress([p.yaw for p in samples])
        endpoint_odom = observer.odometry_at(end.stamp)
        aligned = align_odometry(endpoint_odom, initial_odom, initial)
        odom_error = math.hypot(aligned.x - end.x, aligned.y - end.y)
        odom_heading_error = abs(wrap_angle(aligned.yaw - end.yaw))
        position_limit = 0.05 if not yaw_rate or full_turn else 0.08
        maximum_truth_gap = max((b.stamp - a.stamp for a, b in zip(samples, samples[1:])),
                                default=math.inf)
        passed = maximum_truth_gap <= 0.1 + 1e-6 and position_error <= position_limit \
                 and heading_error <= math.radians(3) and odom_error <= 0.15 \
                 and odom_heading_error <= math.radians(5)
        if full_turn:
            passed = passed and rotation_passed([p.yaw for p in samples], 2 * math.pi)
        else:
            if yaw_rate:
                passed = passed and angular_progress * yaw_rate > 0
        result = {
            'name': name, 'kind': 'full_turn' if full_turn else 'motion', 'passed': passed,
            'command_vx_m_s': velocity, 'command_wz_rad_s': yaw_rate,
            'command_duration_s': duration,
            'actual_observed_duration_s': actual_duration,
            'duration_overshoot_s': actual_duration - duration,
            'start_world_pose': asdict(start), 'end_world_pose': asdict(end),
            'target_world_pose': asdict(target), 'position_error_m': position_error,
            'position_error_limit_m': position_limit,
            'heading_error_deg': math.degrees(heading_error),
            'continuous_yaw_progress_deg': math.degrees(angular_progress),
            'maximum_truth_sample_gap_s': maximum_truth_gap,
            'endpoint_odom_stamp_difference_s': abs(endpoint_odom.stamp - end.stamp),
            'odometry_vs_truth_position_error_m': odom_error,
            'odometry_vs_truth_heading_error_deg': math.degrees(odom_heading_error),
        }
        results.append(result)
        print(f"{name}: {'PASS' if passed else 'FAIL'} position error={position_error:.3f} m, "
              f"heading error={math.degrees(heading_error):.2f} deg", flush=True)

    stop('initial_stop')
    for repeat in range(1, options.repeats + 1):
        round_trip_start = observer.latest_truth()
        motion(f'forward_{repeat}', 0.1, 0.0, 10.0)
        stop(f'forward_stop_{repeat}')
        motion(f'reverse_{repeat}', -0.1, 0.0, 10.0)
        stop(f'reverse_stop_{repeat}')
        round_trip_end = observer.latest_truth()
        return_error = math.hypot(round_trip_end.x - round_trip_start.x,
                                  round_trip_end.y - round_trip_start.y)
        results.append({'name': f'round_trip_{repeat}', 'kind': 'round_trip',
                        'passed': return_error <= 0.05, 'return_error_m': return_error,
                        'return_error_limit_m': 0.05})
    if options.full_turns:
        for repeat in range(1, options.repeats + 1):
            motion(f'full_turn_{repeat}', 0.0, 0.2, 2 * math.pi / 0.2, full_turn=True)
            stop(f'full_turn_stop_{repeat}')
    motion('left_arc', 0.1, 0.15, 3.0)
    stop('left_arc_stop')
    motion('right_arc', 0.1, -0.15, 3.0)
    stop('right_arc_stop')
    # Refresh a real motion command before disconnecting; a stopped command does
    # not demonstrate that the kinematics node's stale-command watchdog works.
    motion('before_command_timeout', 0.1, 0.0, 2.0, stop_after=False)
    observer.command = (0.1, 0.0)
    observer.publish_command()
    stop('command_timeout', publish=False)
    stop('final_stop')
    samples = observer.truth_between(motion_start, observer.latest_truth().stamp)
    envelope = {
        'x_min_m': min(p.x for p in samples), 'x_max_m': max(p.x for p in samples),
        'y_min_m': min(p.y for p in samples), 'y_max_m': max(p.y for p in samples),
        'z_min_m': min(p.z for p in samples), 'z_max_m': max(p.z for p in samples),
        'max_initial_z_deviation_m': max(abs(p.z - initial.z) for p in samples),
        'max_abs_roll_deg': math.degrees(max(abs(p.roll) for p in samples)),
        'max_abs_pitch_deg': math.degrees(max(abs(p.pitch) for p in samples)),
    }
    corridor_passed = all(abs(p.x) <= 0.65 and -6.75 <= p.y <= 6.5
                          and abs(p.roll) <= math.radians(10)
                          and abs(p.pitch) <= math.radians(10)
                          and abs(p.z - initial.z) <= 0.03 for p in samples)
    arm_deviation = {name: max((abs(positions[name] - initial_arms[name])
                               for stamp, positions in observer.arms
                               if stamp >= motion_start and name in positions), default=math.inf)
                     for name in initial_arms}
    readiness.update({
        'executed': True, 'repeats': options.repeats, 'full_turns': options.full_turns,
        'sim_duration_s': samples[-1].stamp - samples[0].stamp,
        'maximum_odom_stamp_difference_s': observer.max_odom_stamp_difference,
        'steps': results, 'world_pose_envelope': envelope,
        'central_corridor_and_upright_passed': corridor_passed,
        'arm_max_initial_deviation_rad': arm_deviation,
        'truth_nonincreasing_message_count': observer.nonincreasing_truth_messages,
        'passed': all(result['passed'] for result in results) and corridor_passed
                  and all(value <= 0.02 for value in arm_deviation.values())
                  and observer.nonincreasing_truth_messages == 0,
        'limits': {'heading_error_deg': 3, 'odom_position_error_m': 0.15,
                   'odom_heading_error_deg': 5, 'rest_speed_m_s': 0.01,
                   'rest_yaw_rate_rad_s': 0.02, 'rest_duration_s': 0.5},
        'scope': 'Flat-ground simulation regression; no contact-force or fruit/leaf collision certification. '
                 'Full turns are conditional simulation capability; physical steering limits remain unconfirmed.',
    })
    return readiness


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Explicitly publish chassis motion commands')
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--full-turns', action='store_true', help='Also test one full rotation per repeat')
    parser.add_argument('--progress-timeout', type=float, default=15.0,
                        help='Wall seconds without advancing Gazebo ground truth before aborting')
    parser.add_argument('--timeout', '--wall-timeout', dest='wall_timeout', type=float, default=600.0,
                        help='Total wall seconds allowed for readiness and the complete suite')
    parser.add_argument('--output', type=Path)
    options = parser.parse_args()
    if options.repeats < 1 or options.progress_timeout <= 0 or options.wall_timeout <= 0:
        parser.error('--repeats and timeout values must be positive')
    observer, initialized = None, False
    try:
        if not os.environ.get('GZ_PARTITION'):
            raise PreconditionsError('Set an isolated nonempty GZ_PARTITION before running this test')
        import rclpy
        rclpy.init(args=[])
        initialized = True
        observer = Observer(options.execute, options.progress_timeout, options.wall_timeout)
        report = run_suite(observer, options)
        code = 0 if report['passed'] else 1
    except (PreconditionsError, MotionFailure, ImportError, KeyboardInterrupt) as error:
        report = {'passed': False, 'error': str(error) or 'interrupted',
                  'executed': bool(observer and getattr(observer, 'motion_authorized', False))}
        code = 1 if isinstance(error, MotionFailure) else 2
    finally:
        if observer is not None:
            if getattr(observer, 'motion_authorized', False):
                try:
                    observer.shutdown_stop()
                except RuntimeError:
                    pass  # ROS may already have shut down after Ctrl-C; the command watchdog remains active.
            observer.ros.destroy_node()
        if initialized:
            rclpy.shutdown()
    def finite_json(value):
        if isinstance(value, float) and not math.isfinite(value):
            return None
        if isinstance(value, dict):
            return {key: finite_json(item) for key, item in value.items()}
        if isinstance(value, list):
            return [finite_json(item) for item in value]
        return value

    if observer is not None:
        report['truth_nonincreasing_message_count'] = observer.nonincreasing_truth_messages
        if 'steps' not in report and getattr(observer, 'results', None):
            report['partial_steps'] = observer.results
        if options.output:
            csv_path = options.output.with_suffix('.truth.csv')
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            with observer.lock:
                samples = list(observer.truth)
            continuous_yaw = samples[0].yaw if samples else 0.0
            previous_yaw = continuous_yaw
            with csv_path.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['sim_time_s', 'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad',
                                 'yaw_rad', 'continuous_yaw_rad'])
                for sample in samples:
                    continuous_yaw += wrap_angle(sample.yaw - previous_yaw)
                    previous_yaw = sample.yaw
                    writer.writerow([sample.stamp, sample.x, sample.y, sample.z,
                                     sample.roll, sample.pitch, sample.yaw, continuous_yaw])
            report['truth_trajectory_csv'] = str(csv_path)
            report['truth_trajectory_sample_count'] = len(samples)
    payload = json.dumps(finite_json(report), indent=2, allow_nan=False) + '\n'
    print(payload, end='')
    if options.output:
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(payload)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
