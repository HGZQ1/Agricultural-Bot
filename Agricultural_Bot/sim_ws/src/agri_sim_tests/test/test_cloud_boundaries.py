"""Exercise PointCloud2 storage and invalid-return boundaries without ROS nodes."""

from array import array
import math
import struct

from builtin_interfaces.msg import Time
import pytest
from sensor_msgs.msg import PointCloud2, PointField

from agri_sim_tests.check_mid360 import (
    Mid360Check, VERTICAL_TOLERANCE_RAD, cloud_statistics, parse_options,
    stamp_nanoseconds,
)


def make_cloud(rows, bigendian=False, padding=0):
    width = len(rows[0])
    byte_order = '>' if bigendian else '<'
    data = b''.join(
        b''.join(struct.pack(byte_order + 'ffff', *point) for point in row)
        + b'\xff' * padding
        for row in rows
    )
    return PointCloud2(
        width=width, height=len(rows), point_step=16,
        row_step=16 * width + padding,
        is_bigendian=bigendian,
        fields=[
            PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(('x', 'y', 'z', 'intensity'))
        ],
        data=array('B', data), is_dense=False,
    )


@pytest.mark.parametrize('bigendian', [False, True])
def test_row_padding_and_byte_order_are_preserved(bigendian):
    cloud = make_cloud(
        [[(1.0, 2.0, 2.0, 10.0)], [(0.0, 3.2, 2.4, 20.0)]],
        bigendian=bigendian, padding=8,
    )
    original_bytes = cloud.data.tobytes()
    result = cloud_statistics(cloud, 0.1, 40.0)
    assert result['valid'] == 2
    assert result['min_range_m'] == pytest.approx(3.0)
    assert result['max_range_m'] == pytest.approx(4.0)
    assert cloud.data.tobytes() == original_bytes


def test_missing_echoes_do_not_fail_a_cloud_with_real_returns():
    cloud = make_cloud([[
        (float('nan'), 0.0, 0.0, float('nan')),
        (float('inf'), 0.0, 0.0, float('inf')),
        (3.0, 4.0, 0.0, 1.0),
    ]])
    result = cloud_statistics(cloud, 0.1, 40.0)
    assert result['valid'] == 1
    assert result['missing'] == 2
    assert result['max_range_m'] == pytest.approx(5.0)


def test_all_missing_echoes_fail():
    cloud = make_cloud([[(float('nan'), 0.0, 0.0, 1.0)]])
    with pytest.raises(ValueError, match='no finite'):
        cloud_statistics(cloud, 0.1, 40.0)


@pytest.mark.parametrize('point', [
    (0.0, 0.0, 0.0, 1.0),
    (30.0, 30.0, 0.0, 1.0),
])
def test_range_uses_sensor_norm_and_rejects_outside_limits(point):
    cloud = make_cloud([[point]])
    with pytest.raises(ValueError, match='outside'):
        cloud_statistics(cloud, 0.1, 40.0)


def test_invalid_intensity_for_a_real_return_fails():
    cloud = make_cloud([[(1.0, 0.0, 0.0, float('nan'))]])
    with pytest.raises(ValueError, match='intensity'):
        cloud_statistics(cloud, 0.1, 40.0)


def test_truncated_cloud_fails_before_decoding():
    cloud = make_cloud([[(1.0, 0.0, 0.0, 1.0)]])
    cloud.data.pop()
    with pytest.raises(ValueError, match='data length'):
        cloud_statistics(cloud, 0.1, 40.0)


@pytest.mark.parametrize('stamp', [
    Time(), Time(sec=-1), Time(nanosec=1_000_000_000),
])
def test_invalid_or_zero_timestamps_fail(stamp):
    with pytest.raises(ValueError):
        stamp_nanoseconds(stamp)


def test_nonzero_timestamp_conversion():
    assert stamp_nanoseconds(Time(sec=2, nanosec=3)) == 2_000_000_003


def elevation_return(angle, azimuth=0.0):
    return (
        3 * math.cos(angle) * math.cos(azimuth),
        3 * math.cos(angle) * math.sin(azimuth),
        3 * math.sin(angle), 1.0,
    )


@pytest.mark.parametrize('angle,violations', [
    (math.radians(-7), 0),
    (math.radians(52), 0),
    (math.radians(-7) - 0.5 * VERTICAL_TOLERANCE_RAD, 0),
    (math.radians(52) + 0.5 * VERTICAL_TOLERANCE_RAD, 0),
    (math.radians(-7) - 2 * VERTICAL_TOLERANCE_RAD, 1),
    (math.radians(52) + 2 * VERTICAL_TOLERANCE_RAD, 1),
])
def test_elevation_limits_allow_roundoff_but_count_outside_returns(angle, violations):
    result = cloud_statistics(make_cloud([[elevation_return(angle)]]), 0.1, 40.0)
    assert result['vertical_violations'] == violations
    assert result['min_vertical_rad'] == pytest.approx(angle, abs=5e-8)
    assert result['max_vertical_rad'] == pytest.approx(angle, abs=5e-8)


def test_vertical_poles_are_counted_and_azimuth_does_not_change_elevation():
    cloud = make_cloud([[
        (0.0, 0.0, 2.0, 1.0), (0.0, 0.0, -2.0, 1.0),
        elevation_return(math.radians(36), math.pi),
        elevation_return(math.radians(36), math.pi / 2),
        (float('nan'), 0.0, -1.0, float('nan')),
    ]])
    result = cloud_statistics(cloud, 0.1, 40.0)
    assert result['vertical_violations'] == 2
    assert result['valid'] == 4
    assert result['missing'] == 1
    assert result['min_vertical_rad'] == pytest.approx(-math.pi / 2)
    assert result['max_vertical_rad'] == pytest.approx(math.pi / 2)


def test_custom_cli_elevation_bounds_use_degrees():
    options = parse_options(['--vertical-min-deg', '-30', '--vertical-max-deg', '60'])
    cloud = make_cloud([[
        elevation_return(math.radians(-25)), elevation_return(math.radians(55)),
    ]])
    assert cloud_statistics(cloud, 0.1, 40.0)['vertical_violations'] == 2
    result = cloud_statistics(cloud, 0.1, 40.0, options.vertical_min, options.vertical_max)
    assert result['vertical_violations'] == 0
    assert options.vertical_min == pytest.approx(-math.pi / 6)
    assert options.vertical_max == pytest.approx(math.pi / 3)


@pytest.mark.parametrize('args', [
    ['--vertical-min-deg', 'nan'],
    ['--vertical-min-deg', '-91'],
    ['--vertical-max-deg', '91'],
    ['--vertical-min-deg', '52'],
])
def test_invalid_elevation_options_fail(args):
    with pytest.raises(SystemExit):
        parse_options(args)


class CheckHarness:
    """Exercise callback/report behavior without creating DDS or TF objects."""

    fail = Mid360Check.fail
    on_cloud = Mid360Check.on_cloud
    report = Mid360Check.report
    complete = Mid360Check.complete
    sim_duration = Mid360Check.sim_duration

    def __init__(self):
        self.options = parse_options(['--duration', '0.2'])
        self.errors = []
        self.messages = self.messages_with_increasing_stamp = 0
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
        self.transform = {'parent': 'base_footprint', 'child': self.options.frame}


@pytest.mark.parametrize('bad_stamp', [Time(), Time(sec=1), Time(nanosec=990_000_000)])
def test_invalid_or_nonincreasing_stamps_do_not_inflate_rate(monkeypatch, bad_stamp):
    times = iter([0.0, 0.04, 0.1, 0.2])
    monkeypatch.setattr('agri_sim_tests.check_mid360.time.monotonic', lambda: next(times))
    checker = CheckHarness()
    for stamp in [Time(sec=1), bad_stamp, Time(sec=1, nanosec=100_000_000),
                  Time(sec=1, nanosec=200_000_000)]:
        cloud = make_cloud([[(1.0, 0.0, 0.0, 1.0)]])
        cloud.header.frame_id = checker.options.frame
        cloud.header.stamp = stamp
        checker.on_cloud(cloud)
    report = checker.report(0.2)
    assert report['passed'] is False
    assert report['messages'] == 4
    assert report['messages_with_increasing_stamp'] == 3
    assert report['sim_hz'] == pytest.approx(10.0)
    assert report['wall_hz'] == pytest.approx(10.0)


def test_fov_failure_retains_angle_diagnostics_in_json_report():
    checker = CheckHarness()
    for stamp in [Time(sec=1), Time(sec=1, nanosec=200_000_000)]:
        cloud = make_cloud([[(0.0, 0.0, 3.0, 1.0), (1.0, 0.0, 0.0, 1.0)]])
        cloud.header.frame_id = checker.options.frame
        cloud.header.stamp = stamp
        checker.on_cloud(cloud)
    report = checker.report(0.2)
    assert report['passed'] is False
    assert any('sensor-local vertical FOV' in error for error in report['errors'])
    assert report['vertical_fov']['violations'] == 2
    assert report['vertical_fov']['actual_min_deg'] == pytest.approx(0)
    assert report['vertical_fov']['actual_max_deg'] == pytest.approx(90)
