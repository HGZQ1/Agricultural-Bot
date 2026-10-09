"""
Validate the stage-three SLAM Toolbox topic and TF contract.

The checker observes only public ROS interfaces.  It does not publish a
velocity command, read Gazebo truth, or alter the SLAM node.  The map
subscription uses transient-local QoS so a map published before the checker
starts is still received.  A volatile subscription is also installed as a
compatibility path for publishers that do not offer transient-local
durability; duplicate map deliveries are de-duplicated by their timestamp and
content signature.
"""

import argparse
import hashlib
import json
import math
import sys
import time

from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .check_scan import scan_statistics, stamp_nanoseconds


RESOLUTION_TOLERANCE_M = 1e-6
MAP_VALUE_MIN = -1
MAP_VALUE_MAX = 100
TF_QUATERNION_TOLERANCE = 1e-3


def quaternion_norm(rotation):
    """Return the Euclidean norm of a geometry quaternion."""
    return math.sqrt(sum(value * value for value in (
        rotation.x, rotation.y, rotation.z, rotation.w)))


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def validate_map(message, *, expected_frame='map', expected_resolution=0.05,
                 allow_unknown=False):
    """Validate one ``nav_msgs/OccupancyGrid`` and return map statistics."""
    if message.header.frame_id != expected_frame:
        raise ValueError(
            f'expected map frame {expected_frame!r}, got '
            f'{message.header.frame_id!r}')
    stamp = stamp_nanoseconds(message.header.stamp)
    info = message.info
    if info.width <= 0 or info.height <= 0:
        raise ValueError('map width and height must be positive')
    if not math.isfinite(float(info.resolution)) or info.resolution <= 0.0:
        raise ValueError('map resolution must be positive and finite')
    if not math.isclose(
            info.resolution, expected_resolution,
            rel_tol=0.0, abs_tol=RESOLUTION_TOLERANCE_M):
        raise ValueError(
            f'expected map resolution {expected_resolution:g}, '
            f'got {info.resolution:g}')
    origin = info.origin
    origin_values = (
        origin.position.x, origin.position.y, origin.position.z,
        origin.orientation.x, origin.orientation.y,
        origin.orientation.z, origin.orientation.w,
    )
    if not _finite(origin_values):
        raise ValueError('map origin contains non-finite values')
    if abs(quaternion_norm(origin.orientation) - 1.0) > TF_QUATERNION_TOLERANCE:
        raise ValueError('map origin quaternion is not normalized')
    expected_cells = int(info.width) * int(info.height)
    if len(message.data) != expected_cells:
        raise ValueError(
            f'map data has {len(message.data)} cells; expected {expected_cells}')
    values = [int(value) for value in message.data]
    if any(value < MAP_VALUE_MIN or value > MAP_VALUE_MAX for value in values):
        raise ValueError('map occupancy values must be -1..100')
    known = sum(value >= 0 for value in values)
    if not allow_unknown and known == 0:
        raise ValueError('map contains no known occupancy cells')
    return {
        'frame_id': message.header.frame_id,
        'stamp_ns': stamp,
        'width': int(info.width),
        'height': int(info.height),
        'resolution_m': float(info.resolution),
        'origin_xyz_m': [
            float(origin.position.x), float(origin.position.y),
            float(origin.position.z),
        ],
        'known_cells': known,
        'unknown_cells': expected_cells - known,
        'free_cells': sum(value == 0 for value in values),
        'occupied_cells': sum(value > 0 for value in values),
    }


def validate_odom(message, *, expected_odom_frame='odom',
                  expected_base_frame='base_footprint'):
    """Validate one navigation-facing ``nav_msgs/Odometry`` message."""
    if message.header.frame_id != expected_odom_frame:
        raise ValueError(
            f'expected odometry frame {expected_odom_frame!r}, got '
            f'{message.header.frame_id!r}')
    if message.child_frame_id != expected_base_frame:
        raise ValueError(
            f'expected odometry child frame {expected_base_frame!r}, got '
            f'{message.child_frame_id!r}')
    stamp = stamp_nanoseconds(message.header.stamp)
    pose = message.pose.pose
    twist = message.twist.twist
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
        twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z,
        *message.pose.covariance, *message.twist.covariance,
    )
    if not _finite(values):
        raise ValueError('odometry contains non-finite values')
    if abs(quaternion_norm(pose.orientation) - 1.0) > TF_QUATERNION_TOLERANCE:
        raise ValueError('odometry pose quaternion is not normalized')
    return {
        'stamp_ns': stamp,
        'frame_id': message.header.frame_id,
        'child_frame_id': message.child_frame_id,
        'position_xyz_m': [
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z),
        ],
        'linear_speed_mps': math.sqrt(
            float(twist.linear.x) ** 2 + float(twist.linear.y) ** 2 +
            float(twist.linear.z) ** 2),
        'angular_speed_rps': math.sqrt(
            float(twist.angular.x) ** 2 + float(twist.angular.y) ** 2 +
            float(twist.angular.z) ** 2),
    }


def validate_transform(transform, *, parent, child, allow_zero_stamp=False):
    """
    Validate a TF transform and return a compact JSON-safe record.

    Static transforms published on ``/tf_static`` conventionally carry a
    zero timestamp.  Dynamic SLAM and odometry transforms must still carry a
    positive timestamp, so accepting zero is explicit at the call site.
    """
    if transform.header.frame_id != parent or transform.child_frame_id != child:
        raise ValueError(
            f'expected TF {parent}->{child}, got '
            f'{transform.header.frame_id}->{transform.child_frame_id}')
    raw_stamp = transform.header.stamp
    if allow_zero_stamp and raw_stamp.sec == 0 and raw_stamp.nanosec == 0:
        stamp = 0
    else:
        stamp = stamp_nanoseconds(raw_stamp)
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    values = [
        translation.x, translation.y, translation.z,
        rotation.x, rotation.y, rotation.z, rotation.w,
    ]
    if not _finite(values):
        raise ValueError(f'TF {parent}->{child} contains non-finite values')
    if abs(quaternion_norm(rotation) - 1.0) > TF_QUATERNION_TOLERANCE:
        raise ValueError(f'TF {parent}->{child} quaternion is not normalized')
    return {
        'parent': parent,
        'child': child,
        'stamp_ns': stamp,
        'xyz_m': [float(translation.x), float(translation.y),
                  float(translation.z)],
        'quaternion_xyzw': [float(rotation.x), float(rotation.y),
                            float(rotation.z), float(rotation.w)],
    }


class _Stream:
    """Track message count and simulation/wall time for one stream."""

    def __init__(self):
        self.messages = 0
        self.increasing = 0
        self.first_stamp = self.last_stamp = None
        self.first_received = self.last_received = None

    def observe(self, stamp, received, *, allow_equal=False):
        self.messages += 1
        previous_stamp = self.last_stamp
        if self.last_stamp is not None:
            if allow_equal and stamp < self.last_stamp:
                raise ValueError('timestamps are decreasing')
            if not allow_equal and stamp <= self.last_stamp:
                raise ValueError('timestamps are not strictly increasing')
        if self.first_stamp is None:
            self.first_stamp = stamp
            self.first_received = received
        self.last_stamp = stamp
        self.last_received = received
        if previous_stamp is None or stamp > previous_stamp:
            self.increasing += 1

    @property
    def duration(self):
        if self.first_stamp is None or self.last_stamp is None:
            return 0.0
        return (self.last_stamp - self.first_stamp) / 1e9

    def report(self):
        wall_span = (
            self.last_received - self.first_received
            if self.last_received is not None and self.first_received is not None
            else 0.0)
        return {
            'messages': self.messages,
            'messages_with_increasing_stamp': self.increasing,
            'sim_duration_s': self.duration,
            'sim_hz': ((self.increasing - 1) / self.duration
                       if self.duration > 0.0 else None),
            'wall_hz': ((self.increasing - 1) / wall_span
                        if wall_span > 0.0 else None),
        }


class MappingCheck(Node):
    """Collect and validate the live SLAM mapping contract."""

    def __init__(self, options):
        super().__init__(
            'check_mapping', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.options = options
        self.errors = []
        self.scan_stream = _Stream()
        self.odom_stream = _Stream()
        self.map_stream = _Stream()
        self.map_messages = 0
        self.map_stats = None
        self.map_digest = None
        self.last_scan_stats = None
        self.last_odom_stats = None
        self.tf_records = {}
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.scan_subscription = self.create_subscription(
            LaserScan, options.scan_topic, self._on_scan, qos_profile_sensor_data)
        odom_qos = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.odom_subscription = self.create_subscription(
            Odometry, options.odom_topic, self._on_odom, odom_qos)
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        volatile_map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        # Keep both subscriptions: map_server/SLAM Toolbox normally uses
        # transient-local, while simple test publishers often use volatile.
        self.map_subscription = self.create_subscription(
            OccupancyGrid, options.map_topic, self._on_map, map_qos)
        self.volatile_map_subscription = self.create_subscription(
            OccupancyGrid, options.map_topic, self._on_map, volatile_map_qos)

    def fail(self, reason):
        if reason not in self.errors and len(self.errors) < 24:
            self.errors.append(reason)

    @property
    def complete(self):
        return (self.scan_stream.duration >= self.options.duration and
                self.odom_stream.duration >= self.options.duration and
                self.map_stats is not None and bool(self.tf_records))

    def _on_scan(self, message):
        try:
            stamp = stamp_nanoseconds(message.header.stamp)
            self.scan_stream.observe(stamp, time.monotonic())
            self.last_scan_stats = scan_statistics(
                message, expected_frame=self.options.scan_frame,
                expected_min_range=self.options.expected_min_range,
                expected_max_range=self.options.expected_max_range,
                require_finite=not self.options.allow_empty_scan)
        except (ValueError, TypeError, OverflowError) as exc:
            self.fail('invalid /scan: ' + str(exc))

    def _on_odom(self, message):
        try:
            stats = validate_odom(
                message, expected_odom_frame=self.options.odom_frame,
                expected_base_frame=self.options.base_frame)
            self.odom_stream.observe(stats['stamp_ns'], time.monotonic())
            self.last_odom_stats = stats
        except (ValueError, TypeError, OverflowError) as exc:
            self.fail('invalid /odom: ' + str(exc))

    def _on_map(self, message):
        try:
            # A newly activated SLAM node may publish an all-unknown grid before
            # the first scan.  Validate its shape now and decide whether a
            # known cell is required only in the final report.
            stats = validate_map(
                message, expected_frame=self.options.map_frame,
                expected_resolution=self.options.expected_resolution,
                allow_unknown=True)
            digest = hashlib.sha256(
                bytes(int(value) & 0xff for value in message.data)).hexdigest()
            signature = (stats['stamp_ns'], stats['width'], stats['height'], digest)
            if signature == self.map_digest:
                return
            self.map_digest = signature
            self.map_messages += 1
            self.map_stream.observe(
                stats['stamp_ns'], time.monotonic(), allow_equal=True)
            self.map_stats = stats
        except (ValueError, TypeError, OverflowError) as exc:
            self.fail('invalid /map: ' + str(exc))

    def refresh_tf(self):
        pairs = (
            (self.options.map_frame, self.options.odom_frame),
            (self.options.odom_frame, self.options.base_frame),
            (self.options.base_frame, self.options.scan_frame),
        )
        for parent, child in pairs:
            try:
                message = self.buffer.lookup_transform(parent, child, Time())
                self.tf_records[f'{parent}->{child}'] = validate_transform(
                    message, parent=parent, child=child,
                    allow_zero_stamp=(child == self.options.scan_frame))
            except (TransformException, ValueError, TypeError, OverflowError) as exc:
                if not isinstance(exc, TransformException):
                    self.fail(f'invalid TF {parent}->{child}: {exc}')

    @staticmethod
    def _rate_ok(report, minimum, maximum):
        rate = report['sim_hz']
        return rate is not None and minimum <= rate <= maximum

    def report(self, elapsed, interrupted=False):
        if interrupted:
            self.fail('check interrupted before acceptance completed')
        if self.scan_stream.duration < self.options.duration:
            self.fail('wall timeout before collecting requested /scan duration')
        if self.odom_stream.duration < self.options.duration:
            self.fail('wall timeout before collecting requested /odom duration')
        if self.map_stats is None:
            self.fail('no valid /map message received')
        elif not self.options.allow_unknown_map and self.map_stats['known_cells'] == 0:
            self.fail('latest /map contains no known occupancy cells')
        scan_report = self.scan_stream.report()
        odom_report = self.odom_stream.report()
        if not self._rate_ok(scan_report, self.options.min_scan_hz,
                             self.options.max_scan_hz):
            self.fail(
                f'/scan simulation rate must be {self.options.min_scan_hz:g}..'
                f'{self.options.max_scan_hz:g} Hz')
        if not self._rate_ok(odom_report, self.options.min_odom_hz,
                             self.options.max_odom_hz):
            self.fail(
                f'/odom simulation rate must be {self.options.min_odom_hz:g}..'
                f'{self.options.max_odom_hz:g} Hz')
        for pair in (
                f'{self.options.map_frame}->{self.options.odom_frame}',
                f'{self.options.odom_frame}->{self.options.base_frame}',
                f'{self.options.base_frame}->{self.options.scan_frame}'):
            if pair not in self.tf_records:
                self.fail(f'TF unavailable: {pair}')
        return {
            'passed': not self.errors,
            'errors': self.errors,
            'map_topic': self.options.map_topic,
            'scan_topic': self.options.scan_topic,
            'odom_topic': self.options.odom_topic,
            'map_messages': self.map_messages,
            'map': self.map_stats,
            'scan': scan_report,
            'last_scan': self.last_scan_stats,
            'odom': odom_report,
            'last_odom': self.last_odom_stats,
            'tf': self.tf_records,
            'wall_elapsed_s': elapsed,
            'duration_required_s': self.options.duration,
            'expected_resolution_m': self.options.expected_resolution,
        }


def parse_options(args):
    """Parse checker options and reject unsafe numeric combinations."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--scan-frame', default='mid360_scan_frame')
    parser.add_argument('--expected-resolution', type=float, default=0.05)
    parser.add_argument('--expected-min-range', type=float, default=0.10)
    parser.add_argument('--expected-max-range', type=float, default=40.0)
    parser.add_argument('--min-scan-hz', type=float, default=9.0)
    parser.add_argument('--max-scan-hz', type=float, default=11.0)
    parser.add_argument('--min-odom-hz', type=float, default=5.0)
    parser.add_argument('--max-odom-hz', type=float, default=100.0)
    parser.add_argument('--allow-unknown-map', action='store_true')
    parser.add_argument('--allow-empty-scan', action='store_true')
    options = parser.parse_args(args)
    values = (
        options.duration, options.timeout, options.expected_resolution,
        options.expected_min_range, options.expected_max_range,
        options.min_scan_hz, options.max_scan_hz,
        options.min_odom_hz, options.max_odom_hz,
    )
    if not all(math.isfinite(value) for value in values):
        parser.error('numeric options must be finite')
    if options.duration <= 0.0 or options.timeout <= 0.0:
        parser.error('duration and timeout must be positive')
    if options.expected_resolution <= 0.0:
        parser.error('expected-resolution must be positive')
    if not 0 <= options.expected_min_range < options.expected_max_range:
        parser.error('require 0 <= expected-min-range < expected-max-range')
    if not 0 < options.min_scan_hz <= options.max_scan_hz:
        parser.error('require 0 < min-scan-hz <= max-scan-hz')
    if not 0 < options.min_odom_hz <= options.max_odom_hz:
        parser.error('require 0 < min-odom-hz <= max-odom-hz')
    return options


def main(args=None):
    argv = sys.argv if args is None else [sys.argv[0], *args]
    options = parse_options(remove_ros_args(args=argv)[1:])
    rclpy.init(args=argv)
    node = MappingCheck(options)
    started = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() - started < options.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.refresh_tf()
            if node.complete:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
    finally:
        report = node.report(time.monotonic() - started, interrupted=interrupted)
        print(json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
