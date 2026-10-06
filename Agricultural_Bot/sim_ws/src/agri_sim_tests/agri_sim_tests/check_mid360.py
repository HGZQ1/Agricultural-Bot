"""Check a live MID-360 point-cloud interface without moving the robot."""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


REQUIRED_FIELDS = ('x', 'y', 'z', 'intensity')
RANGE_TOLERANCE_M = 0.001
VERTICAL_TOLERANCE_RAD = 1e-6


def cloud_statistics(cloud, min_range, max_range,
                     vertical_min=math.radians(-7.0), vertical_max=math.radians(52.0)):
    """Decode numeric fields, including row padding and either byte order.

    NaN/Inf XYZ values are missing returns. Finite XYZ returns must lie inside
    the declared lidar range and have finite intensity. Distances are measured
    from the sensor origin; no world-frame coordinates enter this check.
    Vertical bounds are radians in this local frame. FOV violations are
    returned for the caller to reject while retaining angle diagnostics.
    """
    names = [field.name for field in cloud.fields]
    if len(set(names)) != len(names):
        raise ValueError('duplicate point field names')
    by_name = {field.name: field for field in cloud.fields}
    for name in REQUIRED_FIELDS:
        field = by_name.get(name)
        if field is None:
            raise ValueError('missing point field: ' + name)
        if field.count != 1 or field.datatype not in (
                PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError('point field must be one floating-point value: ' + name)
    if cloud.width <= 0 or cloud.height <= 0 or cloud.point_step <= 0:
        raise ValueError('empty cloud or invalid point_step')
    if cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError('row_step is shorter than the point row')
    if len(cloud.data) != cloud.row_step * cloud.height:
        raise ValueError('data length does not match row_step * height')

    dtype = point_cloud2.dtype_from_fields(cloud.fields, point_step=cloud.point_step)
    dtype = dtype.newbyteorder('>' if cloud.is_bigendian else '<')
    points = np.ndarray(
        shape=(cloud.height, cloud.width), dtype=dtype,
        buffer=memoryview(cloud.data), strides=(cloud.row_step, cloud.point_step),
    )
    xyz = [points[name].astype(np.float64, copy=False) for name in ('x', 'y', 'z')]
    finite = np.isfinite(xyz[0]) & np.isfinite(xyz[1]) & np.isfinite(xyz[2])
    valid_count = int(np.count_nonzero(finite))
    total_count = int(cloud.width * cloud.height)
    if not valid_count:
        raise ValueError('cloud has no finite XYZ returns')
    if not np.all(np.isfinite(points['intensity'][finite])):
        raise ValueError('finite XYZ return has non-finite intensity')
    distances = np.hypot(np.hypot(xyz[0][finite], xyz[1][finite]), xyz[2][finite])
    lower = float(np.min(distances))
    upper = float(np.max(distances))
    if lower < min_range - RANGE_TOLERANCE_M or upper > max_range + RANGE_TOLERANCE_M:
        raise ValueError(
            f'finite return range [{lower:.6f}, {upper:.6f}] m '
            f'is outside [{min_range}, {max_range}] m')
    elevations = np.arctan2(
        xyz[2][finite], np.hypot(xyz[0][finite], xyz[1][finite]))
    violations = int(np.count_nonzero(
        (elevations < vertical_min - VERTICAL_TOLERANCE_RAD)
        | (elevations > vertical_max + VERTICAL_TOLERANCE_RAD)))
    return {
        'total': total_count, 'valid': valid_count,
        'missing': total_count - valid_count,
        'min_range_m': lower, 'max_range_m': upper,
        'min_vertical_rad': float(np.min(elevations)),
        'max_vertical_rad': float(np.max(elevations)),
        'vertical_violations': violations,
    }


def stamp_nanoseconds(stamp):
    """Reject zero/invalid timestamps before comparing sensor updates."""
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('invalid point-cloud timestamp')
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    if value <= 0:
        raise ValueError('point-cloud timestamp is zero')
    return value


class Mid360Check(Node):
    def __init__(self, options):
        super().__init__(
            'check_mid360',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.options = options
        self.errors = []
        self.messages = 0
        self.messages_with_increasing_stamp = 0
        self.first_stamp = self.last_stamp = None
        self.first_received = self.last_received = None
        self.fields = []
        self.actual_frame = None
        self.total_points = self.valid_points = self.missing_points = 0
        self.min_valid_per_cloud = None
        self.max_valid_per_cloud = 0
        self.min_range_seen = self.max_range_seen = None
        self.min_vertical_seen = self.max_vertical_seen = None
        self.vertical_violations = 0
        self.transform = None
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.subscription = self.create_subscription(
            PointCloud2, options.topic, self.on_cloud, qos_profile_sensor_data,
        )

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

    def on_cloud(self, cloud):
        if self.complete:
            return
        received = time.monotonic()
        self.messages += 1
        self.actual_frame = cloud.header.frame_id
        self.fields = [field.name for field in cloud.fields]
        if cloud.header.frame_id != self.options.frame:
            self.fail(
                f'expected frame_id {self.options.frame}, got {cloud.header.frame_id!r}')
        try:
            stamp = stamp_nanoseconds(cloud.header.stamp)
            if self.last_stamp is not None and stamp <= self.last_stamp:
                self.fail('point-cloud timestamps are not strictly increasing')
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
            stats = cloud_statistics(
                cloud, self.options.min_range, self.options.max_range,
                self.options.vertical_min, self.options.vertical_max)
            self.total_points += stats['total']
            self.valid_points += stats['valid']
            self.missing_points += stats['missing']
            self.min_valid_per_cloud = (
                stats['valid'] if self.min_valid_per_cloud is None
                else min(self.min_valid_per_cloud, stats['valid']))
            self.max_valid_per_cloud = max(self.max_valid_per_cloud, stats['valid'])
            self.min_range_seen = (
                stats['min_range_m'] if self.min_range_seen is None
                else min(self.min_range_seen, stats['min_range_m']))
            self.max_range_seen = (
                stats['max_range_m'] if self.max_range_seen is None
                else max(self.max_range_seen, stats['max_range_m']))
            self.min_vertical_seen = (
                stats['min_vertical_rad'] if self.min_vertical_seen is None
                else min(self.min_vertical_seen, stats['min_vertical_rad']))
            self.max_vertical_seen = (
                stats['max_vertical_rad'] if self.max_vertical_seen is None
                else max(self.max_vertical_seen, stats['max_vertical_rad']))
            self.vertical_violations += stats['vertical_violations']
            if stats['vertical_violations']:
                self.fail(
                    'finite returns outside sensor-local vertical FOV '
                    f'[{self.options.vertical_min_deg:g}, '
                    f'{self.options.vertical_max_deg:g}] degrees')
        except (ValueError, TypeError, AssertionError, BufferError, OverflowError) as exc:
            self.fail('invalid point cloud: ' + str(exc))

    def refresh_transform(self):
        try:
            transform = self.buffer.lookup_transform(
                self.options.base_frame, self.options.frame, Time(),
            ).transform
        except TransformException:
            return
        xyz = [transform.translation.x, transform.translation.y, transform.translation.z]
        quaternion = [
            transform.rotation.x, transform.rotation.y,
            transform.rotation.z, transform.rotation.w,
        ]
        if not all(math.isfinite(value) for value in xyz + quaternion):
            self.fail('sensor TF has non-finite values')
            return
        norm = math.sqrt(sum(value * value for value in quaternion))
        if abs(norm - 1.0) > 1e-3:
            self.fail('sensor TF quaternion is not normalized')
            return
        self.transform = {
            'parent': self.options.base_frame, 'child': self.options.frame,
            'xyz_m': xyz, 'quaternion_xyzw': quaternion,
        }

    def report(self, elapsed, interrupted=False):
        if interrupted:
            self.fail('check interrupted before acceptance completed')
        if not self.complete:
            self.fail(
                f'wall timeout before collecting {self.options.duration:g} s of sensor data')
        if self.transform is None:
            self.fail(
                f'TF unavailable: {self.options.base_frame} -> {self.options.frame}')
        sim_hz = (
            (self.messages_with_increasing_stamp - 1) / self.sim_duration
            if self.messages_with_increasing_stamp > 1 and self.sim_duration > 0 else None)
        wall_span = (
            self.last_received - self.first_received
            if self.last_received is not None and self.first_received is not None else 0.0)
        wall_hz = (
            (self.messages_with_increasing_stamp - 1) / wall_span
            if self.messages_with_increasing_stamp > 1 and wall_span > 0 else None)
        if sim_hz is None or not self.options.min_hz <= sim_hz <= self.options.max_hz:
            self.fail(
                f'simulation rate must be {self.options.min_hz:g}..{self.options.max_hz:g} Hz')
        return {
            'passed': not self.errors,
            'errors': self.errors,
            'topic': self.options.topic,
            'frame_id': self.actual_frame,
            'fields': self.fields,
            'messages': self.messages,
            'messages_with_increasing_stamp': self.messages_with_increasing_stamp,
            'sim_duration_s': round(self.sim_duration, 6),
            'wall_elapsed_s': round(elapsed, 6),
            'sim_hz': round(sim_hz, 6) if sim_hz is not None else None,
            'wall_hz': round(wall_hz, 6) if wall_hz is not None else None,
            'valid_points': {
                'total': self.valid_points,
                'min_per_cloud': self.min_valid_per_cloud,
                'max_per_cloud': self.max_valid_per_cloud,
                'missing_returns': self.missing_points,
                'decoded_points': self.total_points,
            },
            'range_m': [self.min_range_seen, self.max_range_seen],
            'range_tolerance_m': RANGE_TOLERANCE_M,
            'vertical_fov': {
                'expected_min_deg': self.options.vertical_min_deg,
                'expected_max_deg': self.options.vertical_max_deg,
                'actual_min_deg': (
                    math.degrees(self.min_vertical_seen)
                    if self.min_vertical_seen is not None else None),
                'actual_max_deg': (
                    math.degrees(self.max_vertical_seen)
                    if self.max_vertical_seen is not None else None),
                'violations': self.vertical_violations,
                'tolerance_rad': VERTICAL_TOLERANCE_RAD,
            },
            'tf': self.transform,
        }


def parse_options(args):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=5.0, help='Sensor timestamp span in seconds')
    parser.add_argument('--timeout', type=float, default=90.0, help='Wall-clock deadline in seconds')
    parser.add_argument('--topic', default='/mid360/points')
    parser.add_argument('--frame', default='mid360_sensor_frame')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--min-range', type=float, default=0.1)
    parser.add_argument('--max-range', '--maxrange', type=float, default=40.0)
    parser.add_argument('--vertical-min-deg', type=float, default=-7.0,
                        help='Minimum sensor-local elevation in degrees (default: -7)')
    parser.add_argument('--vertical-max-deg', type=float, default=52.0,
                        help='Maximum sensor-local elevation in degrees (default: 52)')
    parser.add_argument('--min-hz', type=float, default=9.0)
    parser.add_argument('--max-hz', type=float, default=11.0)
    options = parser.parse_args(args)
    values = (
        options.duration, options.timeout, options.min_range, options.max_range,
        options.min_hz, options.max_hz,
        options.vertical_min_deg, options.vertical_max_deg,
    )
    if not all(math.isfinite(value) for value in values):
        parser.error('numeric options must be finite')
    if options.duration <= 0 or options.timeout <= 0:
        parser.error('duration and timeout must be positive')
    if not 0 <= options.min_range < options.max_range:
        parser.error('require 0 <= min-range < max-range')
    if not 0 < options.min_hz <= options.max_hz:
        parser.error('require 0 < min-hz <= max-hz')
    if not -90 <= options.vertical_min_deg < options.vertical_max_deg <= 90:
        parser.error('require -90 <= vertical-min-deg < vertical-max-deg <= 90')
    options.vertical_min = math.radians(options.vertical_min_deg)
    options.vertical_max = math.radians(options.vertical_max_deg)
    return options


def main(args=None):
    argv = sys.argv if args is None else [sys.argv[0], *args]
    options = parse_options(remove_ros_args(args=argv)[1:])
    rclpy.init(args=argv)
    node = Mid360Check(options)
    started = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() - started < options.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.refresh_transform()
            if node.complete and node.transform is not None:
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
