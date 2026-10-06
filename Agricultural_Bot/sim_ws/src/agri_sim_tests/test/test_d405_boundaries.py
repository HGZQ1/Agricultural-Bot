"""Exercise RGB-D storage, timestamp matching, and acceptance deadlines offline."""

from array import array
from collections import deque
import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException

from agri_sim_tests.check_d405 import (
    D405Check, StampRecord, StampSynchronizer, StreamStats, TFWindow,
    depth_metres, depth_statistics, parse_options, stamp_nanoseconds,
    validate_camera_info, validate_color,
)


def depth_image(rows, encoding='32FC1', bigendian=False, padding=0):
    dtype = np.dtype('>f4' if bigendian else '<f4')
    if encoding == '16UC1':
        dtype = np.dtype('>u2' if bigendian else '<u2')
    data = b''.join(np.array(row, dtype=dtype).tobytes() + b'\xff' * padding for row in rows)
    return Image(
        height=len(rows), width=len(rows[0]), encoding=encoding,
        is_bigendian=int(bigendian), step=len(rows[0]) * dtype.itemsize + padding,
        data=array('B', data),
    )


def camera_info(width=848, height=480):
    fx, fy, cx, cy = 400.0, 401.0, width / 2, height / 2
    return CameraInfo(
        width=width, height=height, distortion_model='plumb_bob', d=[0.0] * 5,
        k=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        p=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    )


@pytest.mark.parametrize('bigendian', [False, True])
@pytest.mark.parametrize('encoding', ['32FC1', '16UC1'])
def test_depth_units_byte_order_and_padding(bigendian, encoding):
    rows = [[0.07, 0.2], [0.3, 0.5]] if encoding == '32FC1' else [[70, 200], [300, 500]]
    image = depth_image(rows, encoding, bigendian, padding=7)
    original = image.data.tobytes()
    decoded = depth_metres(image)
    assert decoded.dtype == np.float32
    np.testing.assert_allclose(decoded, [[0.07, 0.2], [0.3, 0.5]], atol=1e-7)
    assert depth_statistics(image)['range_violations'] == 0
    assert image.data.tobytes() == original


def test_depth_encodings_produce_equivalent_nan_mask_and_metres():
    metres = depth_metres(depth_image([[0, 0.07, 0.5]], '32FC1'))
    millimetres = depth_metres(depth_image([[0, 70, 500]], '16UC1'))
    np.testing.assert_allclose(metres, millimetres, equal_nan=True, atol=1e-7)


def test_invalid_depth_values_are_nan_without_rejecting_an_empty_view():
    image = depth_image([[0, float('nan'), float('inf'), -0.1]])
    assert np.all(np.isnan(depth_metres(image)))
    stats = depth_statistics(image)
    assert stats['valid'] == 0
    assert stats['invalid'] == 4
    assert stats['negative_values'] == 1
    assert stats['min_depth_m'] is None
    assert stats['max_depth_m'] is None


def test_sensor_range_includes_values_below_algorithm_working_range():
    stats = depth_statistics(depth_image([[0.07, 0.09, 0.5, 0.069, 0.501]]))
    assert stats['valid'] == 5
    assert stats['range_violations'] == 2


@pytest.mark.parametrize('corruption', ['truncated', 'short_step', 'unsupported_encoding'])
def test_invalid_image_storage_fails_before_reading(corruption):
    image = depth_image([[0.2, 0.3]])
    if corruption == 'truncated':
        image.data.pop()
    elif corruption == 'short_step':
        image.step = 7
    else:
        image.encoding = 'mono16'
    with pytest.raises(ValueError):
        depth_metres(image)


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8'])
def test_color_padding_and_supported_encoding(encoding):
    image = Image(height=2, width=2, encoding=encoding, step=8, data=array('B', bytes(16)))
    validate_color(image, 2, 2)
    image.step = 5
    with pytest.raises(ValueError, match='step'):
        validate_color(image, 2, 2)


@pytest.mark.parametrize('corruption', [
    'focal_length', 'nan_distortion', 'wrong_resolution', 'singular_rotation',
    'wrong_homogeneous_row', 'distortion_count', 'principal_point', 'projection_mismatch',
])
def test_malformed_intrinsics_are_rejected(corruption):
    info = camera_info()
    if corruption == 'focal_length':
        info.k[0] = 0
    elif corruption == 'nan_distortion':
        info.d[0] = math.nan
    elif corruption == 'wrong_resolution':
        info.width = 640
    elif corruption == 'singular_rotation':
        info.r = [0.0] * 9
    elif corruption == 'wrong_homogeneous_row':
        info.p[10] = 0
    elif corruption == 'distortion_count':
        info.d = [0.0]
    elif corruption == 'projection_mismatch':
        info.p[5] = info.k[0]
    else:
        info.k[2] = 848
    with pytest.raises(ValueError):
        validate_camera_info(info)


def test_calibration_retains_distortion_and_projection_values():
    info = camera_info()
    info.d[0] = 0.01
    result = validate_camera_info(info)
    assert result['k'][0] == 400
    assert result['d'][0] == 0.01


def test_synchronization_accepts_boundary_and_consumes_records_once():
    sync = StampSynchronizer()
    assert sync.add('info', StampRecord(1_033_000_000, 0.01)) == []
    assert sync.add('depth', StampRecord(1_010_000_000, 0.02)) == []
    groups = sync.add('color', StampRecord(1_000_000_000, 0.03))
    assert len(groups) == 1
    assert [record.stamp_ns for record in groups[0]] == [1_000_000_000, 1_010_000_000, 1_033_000_000]
    assert sync.add('color', StampRecord(1_040_000_000, 0.04)) == []


def test_missed_frame_does_not_match_a_full_30hz_period_away():
    sync = StampSynchronizer()
    sync.add('color', StampRecord(1_000_000_000, 0))
    sync.add('depth', StampRecord(1_033_333_333, 0))
    assert sync.add('info', StampRecord(1_033_333_333, 0)) == []
    assert sync.drops['color'] == 1
    groups = sync.add('color', StampRecord(1_033_333_333, 0))
    assert len(groups) == 1
    assert len({record.stamp_ns for record in groups[0]}) == 1


def test_group_span_rejects_three_pairwise_near_anchor_records():
    sync = StampSynchronizer()
    sync.add('color', StampRecord(1_000_000_000, 0))
    sync.add('depth', StampRecord(980_000_000, 0))
    assert sync.add('info', StampRecord(1_020_000_000, 0)) == []
    assert sync.drops['depth'] == 1


def test_silent_stream_bounds_cache_memory_and_records_drops():
    sync = StampSynchronizer(queue_size=3)
    for i in range(10):
        sync.add('color', StampRecord(i + 1, float(i)))
    assert len(sync.queues['color']) == 3
    assert sync.drops['color'] == 7


@pytest.mark.parametrize('bad_stamp', [0, 1_000_000_000, 999_000_000])
def test_bad_timestamps_fail_without_inflating_frequency(bad_stamp):
    stats = StreamStats()
    stats.observe(1_000_000_000, 0)
    with pytest.raises(ValueError):
        stats.observe(bad_stamp, 0.001)
    stats.observe(1_033_333_333, 0.1)
    result = stats.report()
    assert result['messages'] == 3
    assert result['messages_with_increasing_stamp'] == 2
    assert result['sim_hz'] == pytest.approx(30.0)
    assert result['wall_hz'] == pytest.approx(10.0)


@pytest.mark.parametrize('stamp', [Time(), Time(sec=-1), Time(nanosec=1_000_000_000)])
def test_invalid_ros_stamps_are_rejected(stamp):
    with pytest.raises(ValueError):
        stamp_nanoseconds(stamp)


def test_tf_available_at_deadline_is_timely_but_late_tf_cannot_repair_failure():
    exact = TFWindow(10.0)
    assert exact.complete(10.1, True)
    late = TFWindow(10.0)
    assert not late.complete(10.101, True)
    assert late.finished
    assert not late.complete(10.102, True)
    absent = TFWindow(10.0)
    assert not absent.complete(10.05, False)
    assert not absent.finished
    assert absent.complete(10.06, True)


def test_extended_depth_contract_accepts_far_samples_and_rejects_out_of_range():
    image = depth_image([[0.07, 0.7, 1.6, 2.0, 2.001, float('inf')]])
    nominal = depth_statistics(image)
    extended = depth_statistics(image, max_depth=2.0)
    assert nominal['range_violations'] == 4
    assert extended['range_violations'] == 1
    assert extended['valid'] == 5
    assert extended['invalid'] == 1


class CheckHarness:
    """Use real acceptance/report code while replacing DDS with controlled records."""

    fail = D405Check.fail
    report = D405Check.report
    refresh_transforms = D405Check.refresh_transforms
    complete = D405Check.complete

    def __init__(self, require_valid=False):
        args = ['--duration', '1'] + (['--require-valid-depth'] if require_valid else [])
        self.options = parse_options(args)
        self.errors = []
        self.streams = {name: StreamStats() for name in ('color', 'depth', 'info')}
        self.synchronizer = StampSynchronizer()
        self.encodings = {'color': {'rgb8'}, 'depth': {'32FC1'}}
        self.frames = {name: {'camera_optical_frame'} for name in self.streams}
        self.intrinsics = validate_camera_info(camera_info())
        self.depth = {'valid': 0}
        self.groups = self.tf_timely = 31
        self.tf_unavailable = 0
        self.max_sync_s = self.max_tf_wait_s = 0.0
        self.pending = deque()
        self.transform = None

    def collect(self):
        for stats in self.streams.values():
            for frame in range(31):
                stats.observe(1_000_000_000 + round(frame * 1e9 / 30), frame / 15)


def test_slow_wall_rate_does_not_fail_simulation_frequency():
    checker = CheckHarness()
    checker.collect()
    report = checker.report(2.1)
    assert report['passed']
    assert report['streams']['depth']['sim_hz'] == 30
    assert report['streams']['depth']['wall_hz'] == 15


def test_timeout_cannot_pass_a_short_or_silent_stream():
    checker = CheckHarness()
    checker.collect()
    checker.streams['info'] = StreamStats()
    report = checker.report(120)
    assert not report['passed']
    assert any('wall timeout' in error for error in report['errors'])


def test_joint_sync_tf_coverage_counts_input_drops_and_pending_tf():
    checker = CheckHarness()
    checker.collect()
    checker.groups = checker.tf_timely = 30
    report = checker.report(2.1)
    assert not report['passed']
    assert report['tf']['synchronized_and_timely_fraction'] == pytest.approx(30 / 31)
    checker = CheckHarness()
    checker.collect()
    checker.pending.append((2_000_000_000, TFWindow(0)))
    assert not checker.report(120)['passed']


def test_all_invalid_depth_is_allowed_until_scene_requires_a_return():
    checker = CheckHarness()
    checker.collect()
    assert checker.report(2.1)['passed']
    checker.options.require_valid_depth = True
    assert not checker.report(2.1)['passed']


def test_tf_queries_image_stamp_and_retries_without_blocking(monkeypatch):
    checker = CheckHarness()
    stamp = 1_234_567_890
    checker.pending.append((stamp, TFWindow(10.0)))
    checker.tf_timely = 0
    now = [10.02]
    calls = []

    def lookup(parent, child, query_time):
        calls.append((parent, child, query_time.nanoseconds))
        if len(calls) == 1:
            raise TransformException('future transform not received yet')
        result = TransformStamped()
        result.transform.rotation.w = 1.0
        return result

    checker.buffer = SimpleNamespace(lookup_transform=lookup)
    monkeypatch.setattr('agri_sim_tests.check_d405.time.monotonic', lambda: now[0])
    checker.refresh_transforms()
    assert len(checker.pending) == 1
    now[0] = 10.04
    checker.refresh_transforms()
    assert not checker.pending
    assert checker.tf_timely == 1
    assert calls == [('base_footprint', 'camera_optical_frame', stamp)] * 2
    assert checker.transform['image_stamp_ns'] == stamp


@pytest.mark.parametrize('args', [
    ['--duration', 'nan'], ['--timeout', '0'], ['--width', '0'],
    ['--min-depth', '0.6'], ['--sync-ms', '-1'], ['--min-coverage', '1.1'],
])
def test_invalid_acceptance_options_are_rejected(args):
    with pytest.raises(SystemExit):
        parse_options(args)
