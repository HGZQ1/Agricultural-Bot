"""Check live D405 RGB-D streams, synchronization and image-time TF."""

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.clock import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


STREAMS = ('color', 'depth', 'info')
DEPTH_TOLERANCE_M = 1e-6


def image_view(image, dtype, channels=1):
    """View image rows without changing message storage or reading padding."""
    dtype = np.dtype(dtype).newbyteorder('>' if image.is_bigendian else '<')
    if image.width <= 0 or image.height <= 0:
        raise ValueError('image dimensions must be positive')
    pixel_step = dtype.itemsize * channels
    if image.step < image.width * pixel_step:
        raise ValueError('image step is shorter than its pixel row')
    if len(image.data) != image.height * image.step:
        raise ValueError('image data length does not match height * step')
    if image.is_bigendian not in (0, 1):
        raise ValueError('image is_bigendian must be zero or one')
    if channels == 1:
        shape = (image.height, image.width)
        strides = (image.step, pixel_step)
    else:
        shape = (image.height, image.width, channels)
        strides = (image.step, pixel_step, dtype.itemsize)
    return np.ndarray(shape=shape, dtype=dtype, buffer=memoryview(image.data), strides=strides)


def depth_values(image):
    """Decode either supported depth encoding into independent float32 metres."""
    if image.encoding == '32FC1':
        return image_view(image, np.float32).astype(np.float32, copy=True)
    if image.encoding == '16UC1':
        return image_view(image, np.uint16).astype(np.float32) * np.float32(0.001)
    raise ValueError('depth encoding must be 32FC1 (metres) or 16UC1 (millimetres)')


def depth_metres(image):
    """Normalize invalid 0/negative/NaN/Inf depth to NaN without Python pixel loops."""
    depth = depth_values(image)
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    return depth


def depth_statistics(image, min_depth=0.07, max_depth=0.5):
    depth = depth_values(image)
    finite = np.isfinite(depth)
    valid = finite & (depth > 0)
    values = depth[valid]
    violations = int(np.count_nonzero(
        (values < min_depth - DEPTH_TOLERANCE_M)
        | (values > max_depth + DEPTH_TOLERANCE_M)))
    negative = int(np.count_nonzero(finite & (depth < 0)))
    count = int(values.size)
    return {
        'total': int(depth.size), 'valid': count, 'invalid': int(depth.size) - count,
        'min_depth_m': float(np.min(values)) if count else None,
        'max_depth_m': float(np.max(values)) if count else None,
        'range_violations': violations, 'negative_values': negative,
    }


def validate_color(image, width=848, height=480):
    if (image.width, image.height) != (width, height):
        raise ValueError(f'color resolution must be {width}x{height}')
    if image.encoding not in ('rgb8', 'bgr8'):
        raise ValueError('color encoding must be rgb8 or bgr8')
    image_view(image, np.uint8, channels=3)


def validate_camera_info(info, width=848, height=480):
    if (info.width, info.height) != (width, height):
        raise ValueError(f'CameraInfo resolution must be {width}x{height}')
    k, p, r, d = (np.asarray(values, dtype=np.float64) for values in (
        info.k, info.p, info.r, info.d))
    if k.size != 9 or p.size != 12 or r.size != 9:
        raise ValueError('CameraInfo K/P/R lengths must be 9/12/9')
    if not all(np.all(np.isfinite(values)) for values in (k, p, r, d)):
        raise ValueError('CameraInfo K/P/R/D must be finite')
    if not all(value > 0 for value in (k[0], k[4], p[0], p[5])):
        raise ValueError('CameraInfo focal lengths must be positive')
    if not np.allclose(k[6:9], [0, 0, 1], atol=1e-6, rtol=0):
        raise ValueError('CameraInfo K homogeneous row is invalid')
    if not np.allclose(p[8:12], [0, 0, 1, 0], atol=1e-6, rtol=0):
        raise ValueError('CameraInfo P homogeneous row is invalid')
    if not (0 <= k[2] < width and 0 <= k[5] < height
            and 0 <= p[2] < width and 0 <= p[6] < height):
        raise ValueError('CameraInfo principal point lies outside the image')
    rotation = r.reshape(3, 3)
    if (not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-5, rtol=0)
            or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5)):
        raise ValueError('CameraInfo R is not a proper rotation')
    distortion_lengths = {'': 0, 'plumb_bob': 5, 'rational_polynomial': 8, 'equidistant': 4}
    if (info.distortion_model not in distortion_lengths
            or d.size != distortion_lengths[info.distortion_model]):
        raise ValueError('CameraInfo distortion model and D length are inconsistent')
    if (np.all(d == 0) and np.allclose(rotation, np.eye(3), atol=1e-6, rtol=0)
            and np.allclose(p[[3, 7]], 0, atol=1e-6, rtol=0)
            and not np.allclose(k.reshape(3, 3), p.reshape(3, 4)[:, :3],
                                atol=1e-6, rtol=0)):
        raise ValueError('ideal aligned CameraInfo must use the same K and P intrinsics')
    return {
        'width': width, 'height': height, 'distortion_model': info.distortion_model,
        'k': k.tolist(), 'p': p.tolist(), 'r': r.tolist(), 'd': d.tolist(),
    }


def stamp_nanoseconds(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('invalid sensor timestamp')
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    if value <= 0:
        raise ValueError('sensor timestamp is zero')
    return value


class StreamStats:
    def __init__(self):
        self.messages = self.increasing = 0
        self.first_stamp = self.last_stamp = None
        self.first_received = self.last_received = None

    def observe(self, stamp_ns, received):
        self.messages += 1
        if stamp_ns <= 0 or (self.last_stamp is not None and stamp_ns <= self.last_stamp):
            raise ValueError('sensor timestamps must be nonzero and strictly increasing')
        if self.first_stamp is None:
            self.first_stamp, self.first_received = stamp_ns, received
        self.last_stamp, self.last_received = stamp_ns, received
        self.increasing += 1

    @property
    def duration(self):
        if self.first_stamp is None:
            return 0.0
        return (self.last_stamp - self.first_stamp) / 1e9

    def report(self):
        span = (self.last_received - self.first_received) if self.increasing else 0.0
        return {
            'messages': self.messages, 'messages_with_increasing_stamp': self.increasing,
            'sim_duration_s': self.duration,
            'sim_hz': (self.increasing - 1) / self.duration if self.duration > 0 else None,
            'wall_hz': (self.increasing - 1) / span if span > 0 else None,
        }


@dataclass(frozen=True)
class StampRecord:
    stamp_ns: int
    received: float


class StampSynchronizer:
    """Consume monotone stream records once, dropping provably stale heads."""

    def __init__(self, max_delta_ns=33_000_000, queue_size=120):
        self.max_delta_ns = max_delta_ns
        self.queue_size = queue_size
        self.queues = {name: deque() for name in STREAMS}
        self.drops = {name: 0 for name in STREAMS}

    def add(self, stream, record):
        queue = self.queues[stream]
        queue.append(record)
        if len(queue) > self.queue_size:
            queue.popleft()
            self.drops[stream] += 1
        groups = []
        while all(self.queues[name] for name in STREAMS):
            heads = tuple(self.queues[name][0] for name in STREAMS)
            stamps = [head.stamp_ns for head in heads]
            if max(stamps) - min(stamps) <= self.max_delta_ns:
                groups.append(tuple(self.queues[name].popleft() for name in STREAMS))
            else:
                oldest = STREAMS[stamps.index(min(stamps))]
                self.queues[oldest].popleft()
                self.drops[oldest] += 1
        return groups


class TFWindow:
    def __init__(self, started, timeout=0.1):
        self.started = started
        self.timeout = timeout
        self.finished = self.timely = False
        self.wait_s = None

    def complete(self, now, available):
        if self.finished:
            return self.timely
        elapsed = max(0.0, now - self.started)
        if available and elapsed <= self.timeout + 1e-9:
            self.finished = self.timely = True
            self.wait_s = elapsed
        elif elapsed >= self.timeout:
            self.finished = True
            self.wait_s = elapsed
        return self.timely


class D405Check(Node):
    def __init__(self, options):
        super().__init__('check_d405', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.options = options
        self.errors = []
        self.streams = {name: StreamStats() for name in STREAMS}
        self.synchronizer = StampSynchronizer(int(options.sync_ms * 1e6), options.queue_size)
        self.encodings = {'color': set(), 'depth': set()}
        self.frames = {name: set() for name in STREAMS}
        self.intrinsics = None
        self.depth = {
            'total': 0, 'valid': 0, 'invalid': 0, 'range_violations': 0,
            'negative_values': 0, 'frames_with_valid_depth': 0, 'all_invalid_frames': 0,
            'min_depth_m': None, 'max_depth_m': None,
        }
        self.groups = self.tf_timely = self.tf_unavailable = 0
        self.max_sync_s = 0.0
        self.max_tf_wait_s = 0.0
        self.pending = deque()
        self.transform = None
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.subscriptions_held = [
            self.create_subscription(
                Image, options.color_topic, lambda msg: self.on_message('color', msg),
                qos_profile_sensor_data),
            self.create_subscription(
                Image, options.depth_topic, lambda msg: self.on_message('depth', msg),
                qos_profile_sensor_data),
            self.create_subscription(
                CameraInfo, options.info_topic, lambda msg: self.on_message('info', msg),
                qos_profile_sensor_data),
        ]

    def fail(self, reason):
        if reason not in self.errors and len(self.errors) < 24:
            self.errors.append(reason)

    @property
    def complete(self):
        return all(stats.duration >= self.options.duration for stats in self.streams.values())

    def on_message(self, stream, message):
        if self.complete:
            return
        received = time.monotonic()
        self.frames[stream].add(message.header.frame_id)
        if message.header.frame_id != self.options.frame:
            self.fail(f'{stream}: expected frame {self.options.frame}, got {message.header.frame_id!r}')
        try:
            if stream == 'color':
                self.encodings[stream].add(message.encoding)
                validate_color(message, self.options.width, self.options.height)
            elif stream == 'depth':
                self.encodings[stream].add(message.encoding)
                if (message.width, message.height) != (self.options.width, self.options.height):
                    raise ValueError('depth resolution does not match the configured image size')
                stats = depth_statistics(message, self.options.min_depth, self.options.max_depth)
                for name in ('total', 'valid', 'invalid', 'range_violations', 'negative_values'):
                    self.depth[name] += stats[name]
                self.depth['frames_with_valid_depth' if stats['valid'] else 'all_invalid_frames'] += 1
                for name, reducer in (('min_depth_m', min), ('max_depth_m', max)):
                    if stats[name] is not None:
                        self.depth[name] = (stats[name] if self.depth[name] is None
                                            else reducer(self.depth[name], stats[name]))
                if stats['range_violations'] or stats['negative_values']:
                    self.fail('depth contains negative or out-of-range finite values')
            else:
                self.intrinsics = validate_camera_info(
                    message, self.options.width, self.options.height)
        except (ValueError, TypeError) as exc:
            self.fail(f'{stream}: {exc}')
        try:
            stamp = stamp_nanoseconds(message.header.stamp)
        except ValueError as exc:
            self.streams[stream].messages += 1
            self.fail(f'{stream}: {exc}')
            return
        try:
            self.streams[stream].observe(stamp, received)
        except ValueError as exc:
            self.fail(f'{stream}: {exc}')
            return
        for group in self.synchronizer.add(stream, StampRecord(stamp, received)):
            self.groups += 1
            stamps = [record.stamp_ns for record in group]
            self.max_sync_s = max(self.max_sync_s, (max(stamps) - min(stamps)) / 1e9)
            self.pending.append((group[0].stamp_ns, TFWindow(time.monotonic(), self.options.tf_ms / 1000)))

    def refresh_transforms(self):
        remaining = deque()
        for stamp, window in self.pending:
            transform = None
            try:
                transform = self.buffer.lookup_transform(
                    self.options.base_frame, self.options.frame,
                    Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME),
                ).transform
                xyz = [transform.translation.x, transform.translation.y, transform.translation.z]
                quaternion = [transform.rotation.x, transform.rotation.y,
                              transform.rotation.z, transform.rotation.w]
                if (not all(math.isfinite(value) for value in xyz + quaternion)
                        or abs(math.sqrt(sum(value * value for value in quaternion)) - 1) > 1e-3):
                    self.fail('image-time TF is non-finite or has a non-unit quaternion')
                    transform = None
            except TransformException:
                pass
            window.complete(time.monotonic(), transform is not None)
            if window.finished:
                self.max_tf_wait_s = max(self.max_tf_wait_s, window.wait_s)
                if window.timely:
                    self.tf_timely += 1
                    self.transform = {
                        'parent': self.options.base_frame, 'child': self.options.frame,
                        'image_stamp_ns': stamp, 'xyz_m': xyz, 'quaternion_xyzw': quaternion,
                    }
                else:
                    self.tf_unavailable += 1
            else:
                remaining.append((stamp, window))
        self.pending = remaining

    def report(self, elapsed, interrupted=False):
        if interrupted:
            self.fail('check interrupted before acceptance completed')
        if not self.complete:
            self.fail('wall timeout before collecting the requested sensor timestamp duration')
        stream_reports = {name: stats.report() for name, stats in self.streams.items()}
        for name, stats in stream_reports.items():
            hz = stats['sim_hz']
            if hz is None or not self.options.min_hz <= hz <= self.options.max_hz:
                self.fail(f'{name}: simulation frequency must be {self.options.min_hz}..{self.options.max_hz} Hz')
        denominator = max((stats.increasing for stats in self.streams.values()), default=0)
        sync_fraction = self.groups / denominator if denominator else 0.0
        tf_fraction = self.tf_timely / self.groups if self.groups else 0.0
        joint_fraction = self.tf_timely / denominator if denominator else 0.0
        if joint_fraction < self.options.min_coverage:
            self.fail('fewer than the required fraction of image groups satisfy synchronization and timely TF')
        if self.pending:
            self.fail('image-time TF checks remain pending at the wall deadline')
        if self.intrinsics is None:
            self.fail('no valid CameraInfo received')
        if self.options.require_valid_depth and not self.depth['valid']:
            self.fail('test scene requires at least one valid depth sample')
        return {
            'passed': not self.errors, 'errors': self.errors,
            'wall_elapsed_s': elapsed, 'required_sim_duration_s': self.options.duration,
            'frame': self.options.frame,
            'topics': {'color': self.options.color_topic, 'depth': self.options.depth_topic,
                       'info': self.options.info_topic},
            'streams': stream_reports,
            'frames': {name: sorted(values) for name, values in self.frames.items()},
            'encodings': {name: sorted(values) for name, values in self.encodings.items()},
            'synchronization': {
                'matched_groups': self.groups, 'eligible_groups': denominator,
                'matched_fraction': sync_fraction, 'max_delta_s': self.max_sync_s,
                'limit_s': self.options.sync_ms / 1000,
                'discarded': self.synchronizer.drops,
                'unmatched_buffered': {name: len(queue) for name, queue in self.synchronizer.queues.items()},
            },
            'tf': {
                'timely_groups': self.tf_timely, 'unavailable_groups': self.tf_unavailable,
                'pending_groups': len(self.pending), 'timely_fraction_of_matched': tf_fraction,
                'synchronized_and_timely_fraction': joint_fraction,
                'required_fraction': self.options.min_coverage,
                'max_wait_s': self.max_tf_wait_s, 'limit_s': self.options.tf_ms / 1000,
                'last_transform': self.transform,
            },
            'camera_info': self.intrinsics, 'depth': self.depth,
            'depth_contract_m': [self.options.min_depth, self.options.max_depth],
            'depth_tolerance_m': DEPTH_TOLERANCE_M,
            'nominal_d405_algorithm_working_range_m': [0.10, 0.50],
            'extended_simulation_range': (
                self.options.min_depth < 0.07 or self.options.max_depth > 0.50),
        }


def parse_options(args):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.0, help='Sensor stamp duration in seconds')
    parser.add_argument('--timeout', type=float, default=120.0, help='Wall-clock deadline in seconds')
    parser.add_argument('--color-topic', default='/d405/color/image_raw')
    parser.add_argument('--depth-topic', default='/d405/aligned_depth_to_color/image_raw')
    parser.add_argument('--info-topic', default='/d405/color/camera_info')
    parser.add_argument('--frame', default='camera_optical_frame')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--width', type=int, default=848)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--min-depth', type=float, default=0.07)
    parser.add_argument('--max-depth', type=float, default=0.50)
    parser.add_argument('--min-hz', type=float, default=27.0)
    parser.add_argument('--max-hz', type=float, default=33.0)
    parser.add_argument('--sync-ms', type=float, default=33.0)
    parser.add_argument('--tf-ms', type=float, default=100.0)
    parser.add_argument('--min-coverage', type=float, default=0.99)
    parser.add_argument('--queue-size', type=int, default=120)
    parser.add_argument('--require-valid-depth', action='store_true')
    options = parser.parse_args(args)
    values = (options.duration, options.timeout, options.min_depth, options.max_depth,
              options.min_hz, options.max_hz, options.sync_ms, options.tf_ms, options.min_coverage)
    if not all(math.isfinite(value) for value in values):
        parser.error('numeric options must be finite')
    if options.duration <= 0 or options.timeout <= 0:
        parser.error('duration and timeout must be positive')
    if options.width <= 0 or options.height <= 0 or options.queue_size <= 0:
        parser.error('resolution and queue size must be positive')
    if not 0 < options.min_depth < options.max_depth:
        parser.error('require 0 < min-depth < max-depth')
    if not 0 < options.min_hz <= options.max_hz:
        parser.error('require 0 < min-hz <= max-hz')
    if options.sync_ms < 0 or options.tf_ms <= 0 or not 0 < options.min_coverage <= 1:
        parser.error('sync-ms must be nonnegative, tf-ms positive, and coverage in (0, 1]')
    if not all(value for value in (options.color_topic, options.depth_topic, options.info_topic,
                                   options.frame, options.base_frame)):
        parser.error('topics and frames must be nonempty')
    return options


def main(args=None):
    argv = sys.argv if args is None else [sys.argv[0], *args]
    options = parse_options(remove_ros_args(args=argv)[1:])
    rclpy.init(args=argv)
    node = D405Check(options)
    started = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() - started < options.timeout:
            rclpy.spin_once(node, timeout_sec=0.005)
            node.refresh_transforms()
            if node.complete and not node.pending:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
    finally:
        report = node.report(time.monotonic() - started, interrupted)
        print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
