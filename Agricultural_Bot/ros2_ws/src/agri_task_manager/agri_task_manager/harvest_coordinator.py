"""Select a fresh mature target only with a live parked-station permit."""

from __future__ import annotations

import copy
import json
import math
import os
import time

from action_msgs.msg import GoalStatus
from agri_harvest_interfaces.action import HarvestWork
from agri_harvest_interfaces.msg import TomatoTargetArray
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .harvest_safety import fresh_stamp, stamp_ns


def transform_position(position, transform):
    """Apply a validated rigid TF to one point, with no CAD-axis assumptions."""
    q = transform.transform.rotation
    values = (q.x, q.y, q.z, q.w)
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid TF quaternion')
    x, y, z, w = (value / norm for value in values)
    px, py, pz = position.x, position.y, position.z
    translation = transform.transform.translation
    result = (
        (1 - 2 * (y*y + z*z))*px + 2*(x*y-z*w)*py + 2*(x*z+y*w)*pz + translation.x,
        2*(x*y+z*w)*px + (1 - 2*(x*x+z*z))*py + 2*(y*z-x*w)*pz + translation.y,
        2*(x*z-y*w)*px + 2*(y*z+x*w)*py + (1 - 2*(x*x+y*y))*pz + translation.z,
    )
    if not all(math.isfinite(value) for value in result):
        raise ValueError('nonfinite target TF result')
    return result


def target_quality(target, minimum_confidence=0.5, require_segmentation=True) -> bool:
    """Reject a plausible position with missing or poor observation quality."""
    return bool(
        target.target_id and target.mature and target.class_name == 'tomato_R' and
        (target.segmentation_used or not require_segmentation) and
        math.isfinite(target.confidence) and minimum_confidence <= target.confidence <= 1.0 and
        target.temporally_stable and target.stable_frame_count >= 3 and
        target.depth_sample_count >= 15 and
        math.isfinite(target.depth_valid_fraction) and
        0.6 <= target.depth_valid_fraction <= 1.0 and
        math.isfinite(target.depth_mad_m) and 0.0 <= target.depth_mad_m <= 0.03 and
        all(math.isfinite(value) for value in (
            target.position.x, target.position.y, target.position.z)))


def side_and_window(position, side, longitudinal_offset=-0.17998,
                    longitudinal_window=0.30, maximum_distance=1.25,
                    minimum_height=0.2, maximum_height=1.5) -> bool:
    """Restrict task selection; this envelope is not an IK reach certificate."""
    try:
        x, y, z = position
        if (not all(math.isfinite(value) for value in
                    (x, y, z, minimum_height, maximum_height)) or
                not 0 <= minimum_height < maximum_height):
            return False
    except (TypeError, ValueError):
        return False
    correct_side = ((side == 'left' and y > 0.1) or
                    (side == 'right' and y < -0.1))
    return bool(correct_side and abs(x - longitudinal_offset) <= longitudinal_window and
                0.15 <= math.hypot(x - longitudinal_offset, y) <= maximum_distance and
                minimum_height <= z <= maximum_height)


class HarvestCoordinator(Node):
    """Bind visual observations to one nonce-tagged station and worker action."""

    def __init__(self):
        super().__init__('harvest_coordinator')
        self.declare_parameter('ideal_simulation', os.getenv('AGRI_IDEAL_SIMULATION') == '1')
        self._ideal_simulation = (bool(self.get_parameter('ideal_simulation').value) and
                                  bool(self.get_parameter('use_sim_time').value))
        self.declare_parameter('route_status_topic', '/field_route_executor/status')
        self.declare_parameter('station_topic', '/field_route_executor/current_harvest_point')
        self.declare_parameter('work_report_topic', '/field_route_executor/work_report')
        self.declare_parameter('targets_topic', '/agri_vision/tomato_targets_base')
        self.declare_parameter('work_action', '/harvest_work')
        self.declare_parameter('worker_select_targets_after_scan', False)
        self.declare_parameter('selection_frame', 'base_footprint')
        self.declare_parameter('target_max_age_s', 0.5)
        self.declare_parameter('minimum_confidence', 0.5)
        self.declare_parameter('require_segmentation', True)
        self.declare_parameter('scan_timeout_s', 20.0)
        self.declare_parameter('goal_response_timeout_s', 3.0)
        self.declare_parameter('action_timeout_s', 30.0)
        self.declare_parameter('longitudinal_window_m', 0.30)
        self.declare_parameter('maximum_target_distance_m', 1.25)
        for name in ('target_max_age_s', 'scan_timeout_s', 'goal_response_timeout_s',
                     'action_timeout_s', 'longitudinal_window_m', 'maximum_target_distance_m'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        confidence = float(self.get_parameter('minimum_confidence').value)
        if not math.isfinite(confidence) or not 0.0 < confidence <= 1.0:
            raise ValueError('minimum_confidence must be in (0,1]')
        self._route = None
        self._route_received = None
        self._station = None
        self._station_received = None
        self._targets = None
        self._targets_received = None
        self._work_context = None
        self._handle = None
        self._send_future = None
        self._pending_goal = None
        self._request_started = None
        self._result_future = None
        self._cancel_requested = False
        self._handled_contexts = set()
        self._target_history = {}
        self._reported_ns = 0
        self._phase = 'WAITING_FOR_STATION'
        self._blocking_reason = 'no fresh guarded station'
        self._selected_target_id = ''
        self._tf = Buffer(node=self)
        self._listener = TransformListener(self._tf, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._status_pub = self.create_publisher(String, '~/status', qos)
        self._report_pub = self.create_publisher(
            String, str(self.get_parameter('work_report_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('route_status_topic').value),
            self._route_callback, qos)
        self.create_subscription(
            String, str(self.get_parameter('station_topic').value),
            self._station_callback, qos)
        self.create_subscription(
            TomatoTargetArray, str(self.get_parameter('targets_topic').value),
            self._targets_callback, 10)
        self._client = ActionClient(
            self, HarvestWork, str(self.get_parameter('work_action').value))
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self._tick, clock=self._steady_clock)

    def _route_callback(self, message):
        try:
            self._route = json.loads(message.data)
            self._route_received = time.monotonic()
        except (ValueError, TypeError):
            self._route = None

    def _station_callback(self, message):
        try:
            self._station = json.loads(message.data)
            self._station_received = time.monotonic()
        except (ValueError, TypeError):
            self._station = None

    def _targets_callback(self, message):
        self._targets = message
        self._targets_received = time.monotonic()
        # Track use independently of the detector's session-scoped IDs so a
        # target observed at the previous plant cannot be reused after a stop.
        for target in message.targets:
            self._target_history.setdefault(target.target_id, set())

    def _station_context(self, require_permission=True):
        route, station = self._route, self._station
        if (not isinstance(route, dict) or not isinstance(station, dict) or
                not isinstance(route.get('current'), dict)):
            return None
        try:
            status_stamp = int(route.get('status_stamp_ns', 0))
            station_stamp = int(station.get('station_started_ns', 0))
        except (ValueError, TypeError):
            return None
        if (station_stamp <= 0 or station.get('work_side') not in {'left', 'right'} or
                any(not isinstance(station.get(name), str) or not station.get(name)
                    for name in ('run_id', 'waypoint_id', 'work_id', 'plant_id'))):
            return None
        now = self.get_clock().now().nanoseconds
        if ((not getattr(self, '_ideal_simulation', False) and
             (self._route_received is None or
              time.monotonic() - self._route_received > 0.5 or
              not fresh_stamp(status_stamp, now, 0.5))) or
                not route.get('require_harvest_handshake') or
                route.get('state') != 'AT_STATION' or not station.get('active') or
                route.get('run_id') != station.get('run_id') or
                route.get('work_id') != station.get('work_id') or
                route.get('current', {}).get('waypoint_id') != station.get('waypoint_id') or
                not station.get('work_id') or
                (require_permission and not route.get('harvest_work_ready'))):
            return None
        return (station['run_id'], station['waypoint_id'], station['work_id'])

    def _select_target(self):
        array = self._targets
        now_ns = self.get_clock().now().nanoseconds
        age = float(self.get_parameter('target_max_age_s').value)
        if (array is None or self._targets_received is None or
                time.monotonic() - self._targets_received > age or
                not array.observation_valid or
                not fresh_stamp(stamp_ns(array.header.stamp), now_ns, age) or
                not fresh_stamp(stamp_ns(array.depth_stamp), now_ns, age) or
                abs(stamp_ns(array.header.stamp) - stamp_ns(array.depth_stamp)) > 50_000_000 or
                stamp_ns(array.header.stamp) < self._station['station_started_ns'] or
                stamp_ns(array.depth_stamp) < self._station['station_started_ns']):
            self._blocking_reason = 'no fresh post-arrival RGB/depth observation'
            return None
        frame = str(self.get_parameter('selection_frame').value)
        try:
            transform = (None if array.header.frame_id == frame else
                         self._tf.lookup_transform(
                             frame, array.header.frame_id,
                             Time.from_msg(array.header.stamp)))
        except Exception as exc:
            self._blocking_reason = f'target TF unavailable at image time: {exc}'
            return None
        candidates = []
        for target in array.targets:
            confidence = float(self.get_parameter('minimum_confidence').value)
            require_mask = bool(self.get_parameter('require_segmentation').value)
            if (not target_quality(target, confidence, require_mask) or
                    self._target_history.get(target.target_id, set()) - {self._work_context[:2]}):
                continue
            try:
                xyz = ((target.position.x, target.position.y, target.position.z)
                       if transform is None else transform_position(target.position, transform))
            except ValueError:
                continue
            if not side_and_window(
                    xyz, self._station['work_side'],
                    longitudinal_window=float(self.get_parameter('longitudinal_window_m').value),
                    maximum_distance=float(self.get_parameter('maximum_target_distance_m').value)):
                continue
            output = copy.deepcopy(target)
            output.position.x, output.position.y, output.position.z = xyz
            candidates.append(output)
        if not candidates:
            self._blocking_reason = 'no mature stable target inside current side/window'
            return None
        candidates.sort(key=lambda target: (
            -target.confidence,
            math.hypot(target.position.x + 0.17998, target.position.y)))
        return candidates[0]

    def _report(self, phase, **details):
        context = self._work_context
        if context is None:
            return
        stamp = max(self.get_clock().now().nanoseconds, self._reported_ns + 1)
        self._reported_ns = stamp
        payload = {'run_id': context[0], 'waypoint_id': context[1],
                   'work_id': context[2], 'phase': phase, 'stamp_ns': stamp,
                   'plant_identity_verified': False, 'actual_grasp_verified': False,
                   **details}
        self._report_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_status(self):
        self._status_pub.publish(String(data=json.dumps({
            'ideal_simulation': getattr(self, '_ideal_simulation', False),
            'phase': self._phase, 'blocking_reason': self._blocking_reason,
            'context': self._work_context, 'selected_target_id': self._selected_target_id,
            'plant_identity_verified': False, 'actual_grasp_verified': False,
            'stamp_ns': self.get_clock().now().nanoseconds}, sort_keys=True)))

    def _tick(self):
        current = self._station_context(require_permission=False)
        if self._pending_goal is not None:
            if current != self._work_context:
                self._report('FAILED', terminal_confirmed=True, action_accepted=False,
                             message='station permission revoked before worker goal was sent')
                self._handled_contexts.add(self._work_context)
                self._pending_goal = None
                self._request_started = None
            elif (self._route.get('work_phase') == 'WORKING' and
                  self._route.get('work_terminal_confirmed') is False):
                # A publish followed immediately by an action request races
                # independent DDS streams. Wait for the executor's explicit
                # nonce acknowledgement before the worker can leave STOWED.
                goal = self._pending_goal
                self._pending_goal = None
                self._send_future = self._client.send_goal_async(
                    goal, feedback_callback=self._feedback)
                self._send_future.add_done_callback(self._accepted)
            elif time.monotonic() - self._request_started > float(
                    self.get_parameter('goal_response_timeout_s').value):
                self._report('FAILED', terminal_confirmed=True, action_accepted=False,
                             message='executor did not acknowledge work nonce')
                self._handled_contexts.add(self._work_context)
                self._pending_goal = None
                self._request_started = None
            self._publish_status()
            return
        if self._work_context is not None and self._send_future is not None:
            if current != self._work_context and self._handle is not None:
                self._cancel_action()
            if (self._request_started is not None and
                    time.monotonic() - self._request_started >
                    float(self.get_parameter('action_timeout_s').value)):
                self._cancel_action()
                self._blocking_reason = 'work action timed out; awaiting terminal/stow'
            if (self._handle is None and not self._send_future.done() and
                    self._request_started is not None and
                    time.monotonic() - self._request_started >
                    float(self.get_parameter('goal_response_timeout_s').value)):
                self._phase = 'FAILED'
                self._report('FAILED', terminal_confirmed=False,
                             message='worker goal response is unconfirmed')
                self._handled_contexts.add(self._work_context)
                self._request_started = None
            self._publish_status()
            return
        permitted = self._station_context()
        if permitted is None or permitted in self._handled_contexts:
            self._request_started = None
            self._phase = 'WAITING_FOR_STATION'
            self._blocking_reason = 'waiting for fresh locked, stopped station permit'
            self._publish_status()
            return
        if permitted != self._work_context:
            self._request_started = None
        self._work_context = permitted
        self._phase = 'SCANNING'
        started_ns = int(self._station.get('station_started_ns', 0))
        # Wall time is captured on the first permitted scan, not derived from
        # simulated time: paused /clock must not create an infinite wait.
        if self._request_started is None:
            self._request_started = time.monotonic()
        worker_scan = bool(self.get_parameter('worker_select_targets_after_scan').value)
        target = None if worker_scan else self._select_target()
        if ((worker_scan or target is not None) and
                self._client.server_is_ready() and started_ns > 0):
            goal = HarvestWork.Goal()
            goal.header.frame_id = str(self.get_parameter('selection_frame').value)
            goal.header.stamp = (self.get_clock().now().to_msg() if worker_scan
                                 else self._targets.header.stamp)
            goal.run_id, goal.waypoint_id, goal.work_id = self._work_context
            goal.plant_id = self._station['plant_id']
            goal.work_side = self._station['work_side']
            goal.scan_before_selection = worker_scan
            if target is not None:
                goal.target = target
                self._selected_target_id = target.target_id
                self._target_history.setdefault(target.target_id, set()).add(
                    self._work_context[:2])
            else:
                self._selected_target_id = ''
            self._phase = 'WAITING_FOR_EXECUTOR_ACK'
            self._blocking_reason = ''
            self._report('STARTED', target_id=self._selected_target_id,
                         scan_before_selection=worker_scan)
            self._request_started = time.monotonic()
            self._pending_goal = goal
        elif time.monotonic() - self._request_started > float(
                self.get_parameter('scan_timeout_s').value):
            self._report('FAILED', terminal_confirmed=True,
                         message='no acceptable mature target or worker within scan timeout')
            self._handled_contexts.add(self._work_context)
            self._request_started = None
            self._phase = 'FAILED'
        self._publish_status()

    def _accepted(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._report('FAILED', terminal_confirmed=False, message=str(exc))
            self._handled_contexts.add(self._work_context)
            self._send_future = None
            self._request_started = None
            self._phase = 'FAILED'
            return
        if not handle.accepted:
            self._report('FAILED', terminal_confirmed=True, action_accepted=False,
                         message='worker rejected goal')
            self._handled_contexts.add(self._work_context)
            self._send_future = None
            self._request_started = None
            return
        self._handle = handle
        self._phase = 'WORKING'
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(self._result)
        if self._station_context(require_permission=False) != self._work_context:
            self._cancel_action()

    def _feedback(self, message):
        self._phase = message.feedback.phase

    def _cancel_action(self):
        if self._handle is not None and not self._cancel_requested:
            self._handle.cancel_goal_async()
            self._cancel_requested = True
            self._phase = 'CANCELING_WORK'

    def _result(self, future):
        try:
            wrapped = future.result()
            result = wrapped.result
            if result.target_id:
                self._selected_target_id = result.target_id
                self._target_history.setdefault(result.target_id, set()).add(
                    self._work_context[:2])
            ideal = getattr(self, '_ideal_simulation', False)
            succeeded = (wrapped.status == GoalStatus.STATUS_SUCCEEDED and
                         result.success and (result.sim_fruit_verified if ideal else result.arm_stowed) and
                         self._station_context(require_permission=False) == self._work_context)
            self._report('SUCCESS' if succeeded else 'FAILED',
                         success=bool(succeeded), arm_stowed=bool(result.arm_stowed),
                         target_id=result.target_id,
                         sim_fruit_verified=bool(result.sim_fruit_verified if ideal else
                                                 succeeded and result.sim_fruit_verified),
                         terminal_confirmed=True, message=result.message)
        except Exception as exc:
            self._report('FAILED', terminal_confirmed=False, message=str(exc))
        self._handled_contexts.add(self._work_context)
        self._handle = None
        self._send_future = None
        self._request_started = None
        self._cancel_requested = False
        self._phase = ('WAITING_FOR_STATION' if getattr(self, '_ideal_simulation', False)
                       else 'WAITING_FOR_STOW_ACK')


def main(args=None):
    """Run the coordinator; no mechanical-arm joint commands are published."""
    rclpy.init(args=args)
    node = HarvestCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel_action()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
