"""Validate the navigation LaserScan contract produced from MID-360 points.

This checker deliberately observes the public ``sensor_msgs/LaserScan`` and TF
interfaces only.  It does not consume Gazebo entities or publish commands.  A
separate offline reference projector is provided for unit tests so that height
and range filtering have explicit, reviewable boundary cases.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


RANGE_TOLERANCE_M = 1e-3
ANGLE_TOLERANCE_RAD = 1e-6
FRAME_ORIGIN_TOLERANCE_M = 0.01
HORIZONTAL_TOLERANCE_RAD = math.radians(1.0)


def stamp_nanoseconds(stamp):
    """Return a positive ROS stamp, rejecting invalid or zero values."""
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('invalid LaserScan timestamp')
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    if value <= 0:
        raise ValueError('LaserScan timestamp is zero')
    return value


def _expected_sample_count(angle_min, angle_max, angle_increment):
    """Compute the inclusive number of angular bins with float tolerance."""
    quotient = (angle_max - angle_min) / angle_increment
    rounded = round(quotient)
    if abs(quotient - rounded) > ANGLE_TOLERANCE_RAD / angle_increment:
        raise ValueError('angle range is not an integer number of increments')
    return int(rounded) + 1


def scan_statistics(scan, expected_frame=None, expected_min_range=None,
                    expected_max_range=None, require_finite=False):
    """Validate one LaserScan and return finite/empty beam statistics.

    A missing return is represented by positive infinity, matching the
    ``pointcloud_to_laserscan`` ``use_inf=true`` contract.  NaN and negative
    infinity are rejected because they make costmap ray clearing ambiguous.
    ``expected_*_range`` are optional configuration checks; the scan's own
    metadata is always checked against every finite beam.
    """
    if expected_frame is not None and scan.header.frame_id != expected_frame:
        raise ValueError(
            f'expected frame_id {expected_frame}, got {scan.header.frame_id!r}')
    stamp_nanoseconds(scan.header.stamp)
    values = (
        scan.angle_min, scan.angle_max, scan.angle_increment,
        scan.time_increment, scan.scan_time, scan.range_min, scan.range_max,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('LaserScan metadata contains a non-finite value')
    if scan.angle_increment <= 0 or scan.angle_max < scan.angle_min:
        raise ValueError('LaserScan angle limits/increment are invalid')
    if scan.range_min < 0 or scan.range_max <= scan.range_min:
        raise ValueError('LaserScan range limits are invalid')
    if scan.time_increment < 0 or scan.scan_time < 0:
        raise ValueError('LaserScan timing metadata is invalid')
    expected_count = _expected_sample_count(
        scan.angle_min, scan.angle_max, scan.angle_increment)
    # pointcloud_to_laserscan follows the common LaserScan convention of
    # treating angle_max as an exclusive upper edge, so a 360-degree request
    # at 1-degree resolution contains 360 bins although the inclusive oracle
    # above returns 361.  Accept both conventions and reject larger errors.
    if len(scan.ranges) not in (expected_count, expected_count - 1):
        raise ValueError(
            f'expected {expected_count} (inclusive) or {expected_count - 1} '
            f'(exclusive) range bins, got {len(scan.ranges)}')
    if expected_min_range is not None and not math.isclose(
            scan.range_min, expected_min_range, abs_tol=RANGE_TOLERANCE_M):
        raise ValueError(
            f'expected range_min {expected_min_range:g}, got {scan.range_min:g}')
    if expected_max_range is not None and not math.isclose(
            scan.range_max, expected_max_range, abs_tol=RANGE_TOLERANCE_M):
        raise ValueError(
            f'expected range_max {expected_max_range:g}, got {scan.range_max:g}')

    finite = []
    empty = 0
    for index, value in enumerate(scan.ranges):
        if math.isinf(value):
            if value < 0:
                raise ValueError(f'range bin {index} is negative infinity')
            empty += 1
            continue
        if not math.isfinite(value):
            raise ValueError(f'range bin {index} is NaN')
        if value < scan.range_min - RANGE_TOLERANCE_M:
            raise ValueError(f'range bin {index} is below range_min')
        if value > scan.range_max + RANGE_TOLERANCE_M:
            raise ValueError(f'range bin {index} is above range_max')
        finite.append(float(value))
    if require_finite and not finite:
        raise ValueError('scan contains no finite obstacle returns')
    return {
        'bins': len(scan.ranges),
        'finite_bins': len(finite),
        'empty_bins': empty,
        'min_range_m': min(finite) if finite else None,
        'max_range_m': max(finite) if finite else None,
        'angle_min_rad': float(scan.angle_min),
        'angle_max_rad': float(scan.angle_max),
        'angle_increment_rad': float(scan.angle_increment),
        'range_min_m': float(scan.range_min),
        'range_max_m': float(scan.range_max),
        'scan_time_s': float(scan.scan_time),
        'time_increment_s': float(scan.time_increment),
    }


def reference_project_points(points, *, min_height, max_height, angle_min,
                             angle_max, angle_increment, range_min, range_max):
    """Project XYZ points into nearest-return scan bins for offline tests.

    This is a small contract oracle, not the runtime projection node.  It
    makes explicit that height filtering happens before radial range filtering,
    that non-finite points are ignored, and that each bin keeps the nearest
    return.  Empty bins are positive infinity.
    """
    values = (
        min_height, max_height, angle_min, angle_max, angle_increment,
        range_min, range_max,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('projection bounds must be finite')
    if max_height <= min_height or angle_max <= angle_min or angle_increment <= 0:
        raise ValueError('projection bounds are invalid')
    if range_min < 0 or range_max <= range_min:
        raise ValueError('projection range is invalid')
    count = _expected_sample_count(angle_min, angle_max, angle_increment)
    ranges = [math.inf] * count
    for point in points:
        if len(point) < 3:
            raise ValueError('point must contain x, y and z')
        x, y, z = (float(point[0]), float(point[1]), float(point[2]))
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        if not min_height <= z <= max_height:
            continue
        distance = math.hypot(x, y)
        if not range_min <= distance <= range_max:
            continue
        angle = math.atan2(y, x)
        if angle < angle_min or angle > angle_max:
            continue
        index = int(round((angle - angle_min) / angle_increment))
        if 0 <= index < count and distance < ranges[index]:
            ranges[index] = distance
    return ranges


def _quaternion_norm(rotation):
    return math.sqrt(sum(value * value for value in (
        rotation.x, rotation.y, rotation.z, rotation.w)))


def _z_axis_in_parent(rotation):
    """Return the source frame's +Z axis expressed in the parent frame."""
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    return (2 * (x * z + y * w), 2 * (y * z - x * w),
            1 - 2 * (x * x + y * y))


class ScanCheck(Node):
    """Collect and validate a live navigation scan stream."""

    def __init__(self, options):
        super().__init__(
            'check_scan', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.options = options
        self.errors = []
        self.messages = 0
        self.messages_with_increasing_stamp = 0
        self.first_stamp = self.last_stamp = None
        self.first_received = self.last_received = None
        self.actual_frame = None
        self.last_stats = None
        self.total_finite = self.total_empty = 0
        self.min_range_seen = self.max_range_seen = None
        self.scan_transform = None
        self.sensor_transform = None
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.subscription = self.create_subscription(
            LaserScan, options.topic, self.on_scan, qos_profile_sensor_data)

    def fail(self, reason):
        if reason not in self.errors and len(self.errors) < 16:
            self.errors.append(reason)

    @property
    def sim_duration(self):
        if self.first_stamp is None or self.last_stamp is None:
            return 0.0
        return (self.last_stamp - self.first_stamp) / 1e9

    @property
    def complete(self):
        return self.sim_duration >= self.options.duration

    def on_scan(self, scan):
        if self.complete:
            return
        received = time.monotonic()
        self.messages += 1
        self.actual_frame = scan.header.frame_id
        try:
            stamp = stamp_nanoseconds(scan.header.stamp)
            if self.last_stamp is not None and stamp <= self.last_stamp:
                self.fail('LaserScan timestamps are not strictly increasing')
            else:
                if self.first_stamp is None:
                    self.first_stamp = stamp
                    self.first_received = received
                self.last_stamp = stamp
                self.last_received = received
                self.messages_with_increasing_stamp += 1
        except ValueError as exc:
            self.fail(str(exc))
        try:
            stats = scan_statistics(
                scan, expected_frame=self.options.frame,
                expected_min_range=self.options.expected_min_range,
                expected_max_range=self.options.expected_max_range,
                require_finite=self.options.require_finite,
            )
            self.last_stats = stats
            self.total_finite += stats['finite_bins']
            self.total_empty += stats['empty_bins']
            if stats['min_range_m'] is not None:
                self.min_range_seen = (
                    stats['min_range_m'] if self.min_range_seen is None
                    else min(self.min_range_seen, stats['min_range_m']))
                self.max_range_seen = (
                    stats['max_range_m'] if self.max_range_seen is None
                    else max(self.max_range_seen, stats['max_range_m']))
        except (ValueError, TypeError, OverflowError) as exc:
            self.fail('invalid LaserScan: ' + str(exc))

    def refresh_transforms(self):
        try:
            scan = self.buffer.lookup_transform(
                self.options.base_frame, self.options.frame, Time()).transform
            sensor = self.buffer.lookup_transform(
                self.options.base_frame, self.options.sensor_frame, Time()).transform
        except TransformException:
            return
        for name, transform in (('scan', scan), ('sensor', sensor)):
            values = [transform.translation.x, transform.translation.y,
                      transform.translation.z, transform.rotation.x,
                      transform.rotation.y, transform.rotation.z,
                      transform.rotation.w]
            if not all(math.isfinite(value) for value in values):
                self.fail(f'{name} TF contains non-finite values')
                return
            if abs(_quaternion_norm(transform.rotation) - 1.0) > 1e-3:
                self.fail(f'{name} TF quaternion is not normalized')
                return
        normal = _z_axis_in_parent(scan.rotation)
        if math.hypot(normal[0], normal[1]) > math.sin(HORIZONTAL_TOLERANCE_RAD):
            self.fail('scan frame is not horizontal in the base frame')
        delta = math.sqrt(sum((a - b) ** 2 for a, b in zip(
            (scan.translation.x, scan.translation.y, scan.translation.z),
            (sensor.translation.x, sensor.translation.y, sensor.translation.z))))
        if delta > FRAME_ORIGIN_TOLERANCE_M:
            self.fail(
                f'scan origin differs from sensor origin by {delta:.4f} m')
        self.scan_transform = {
            'parent': self.options.base_frame, 'child': self.options.frame,
            'xyz_m': [scan.translation.x, scan.translation.y, scan.translation.z],
        }
        self.sensor_transform = {
            'parent': self.options.base_frame, 'child': self.options.sensor_frame,
            'xyz_m': [sensor.translation.x, sensor.translation.y, sensor.translation.z],
        }

    def report(self, elapsed, interrupted=False):
        if interrupted:
            self.fail('check interrupted before acceptance completed')
        if not self.complete:
            self.fail(
                f'wall timeout before collecting {self.options.duration:g} s of scans')
        if self.scan_transform is None or self.sensor_transform is None:
            self.fail(
                f'TF unavailable: {self.options.base_frame} -> '
                f'{self.options.frame}/{self.options.sensor_frame}')
        sim_hz = (
            (self.messages_with_increasing_stamp - 1) / self.sim_duration
            if self.messages_with_increasing_stamp > 1 and self.sim_duration > 0
            else None)
        wall_span = (
            self.last_received - self.first_received
            if self.last_received is not None and self.first_received is not None else 0.0)
        wall_hz = (
            (self.messages_with_increasing_stamp - 1) / wall_span
            if self.messages_with_increasing_stamp > 1 and wall_span > 0 else None)
        if sim_hz is None or not self.options.min_hz <= sim_hz <= self.options.max_hz:
            self.fail(
                f'simulation rate must be {self.options.min_hz:g}..'
                f'{self.options.max_hz:g} Hz')
        return {
            'passed': not self.errors,
            'errors': self.errors,
            'topic': self.options.topic,
            'frame_id': self.actual_frame,
            'messages': self.messages,
            'messages_with_increasing_stamp': self.messages_with_increasing_stamp,
            'sim_duration_s': round(self.sim_duration, 6),
            'wall_elapsed_s': round(elapsed, 6),
            'sim_hz': round(sim_hz, 6) if sim_hz is not None else None,
            'wall_hz': round(wall_hz, 6) if wall_hz is not None else None,
            'finite_bins': self.total_finite,
            'empty_bins': self.total_empty,
            'range_m': [self.min_range_seen, self.max_range_seen],
            'last_scan': self.last_stats,
            'scan_tf': self.scan_transform,
            'sensor_tf': self.sensor_transform,
            'origin_tolerance_m': FRAME_ORIGIN_TOLERANCE_M,
            'horizontal_tolerance_deg': math.degrees(HORIZONTAL_TOLERANCE_RAD),
        }


def parse_options(args):
    """Parse checker options and reject unsafe numeric combinations."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--topic', default='/scan')
    parser.add_argument('--frame', default='mid360_scan_frame')
    parser.add_argument('--sensor-frame', default='mid360_sensor_frame')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--expected-min-range', type=float, default=0.10)
    parser.add_argument('--expected-max-range', type=float, default=40.0)
    parser.add_argument('--min-hz', type=float, default=9.0)
    parser.add_argument('--max-hz', type=float, default=11.0)
    parser.add_argument('--allow-empty', action='store_true',
                        help='allow scans with no finite obstacle returns')
    options = parser.parse_args(args)
    values = (options.duration, options.timeout, options.expected_min_range,
              options.expected_max_range, options.min_hz, options.max_hz)
    if not all(math.isfinite(value) for value in values):
        parser.error('numeric options must be finite')
    if options.duration <= 0 or options.timeout <= 0:
        parser.error('duration and timeout must be positive')
    if not 0 <= options.expected_min_range < options.expected_max_range:
        parser.error('require 0 <= expected-min-range < expected-max-range')
    if not 0 < options.min_hz <= options.max_hz:
        parser.error('require 0 < min-hz <= max-hz')
    options.require_finite = not options.allow_empty
    return options


def main(args=None):
    argv = sys.argv if args is None else [sys.argv[0], *args]
    options = parse_options(remove_ros_args(args=argv)[1:])
    rclpy.init(args=argv)
    node = ScanCheck(options)
    started = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() - started < options.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.refresh_transforms()
            if node.complete and node.scan_transform is not None:
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
