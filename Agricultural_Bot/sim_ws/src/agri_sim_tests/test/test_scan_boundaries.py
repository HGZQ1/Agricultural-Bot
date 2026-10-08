"""Offline boundary tests for the stage-two LaserScan contract."""

import math

import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan

from agri_sim_tests.check_scan import (
    _expected_sample_count,
    parse_options,
    reference_project_points,
    scan_statistics,
    stamp_nanoseconds,
)


def make_scan(ranges, *, frame='mid360_scan_frame', stamp=1,
              angle_min=-math.pi, angle_increment=math.pi / 2,
              range_min=0.1, range_max=40.0):
    message = LaserScan()
    message.header.frame_id = frame
    message.header.stamp = Time(sec=stamp)
    message.angle_min = angle_min
    message.angle_increment = angle_increment
    message.angle_max = angle_min + angle_increment * (len(ranges) - 1)
    message.time_increment = 0.1
    message.scan_time = 0.1
    message.range_min = range_min
    message.range_max = range_max
    message.ranges = list(ranges)
    return message


def test_valid_scan_accepts_positive_infinity_for_empty_beams():
    scan = make_scan([math.inf, 3.0, math.inf, 1.0, math.inf])
    result = scan_statistics(
        scan, expected_frame='mid360_scan_frame',
        expected_min_range=0.1, expected_max_range=40.0,
        require_finite=True)
    assert result['bins'] == 5
    assert result['finite_bins'] == 2
    assert result['empty_bins'] == 3
    assert result['min_range_m'] == pytest.approx(1.0)
    assert result['max_range_m'] == pytest.approx(3.0)


@pytest.mark.parametrize('bad_value', [math.nan, -math.inf])
def test_nan_and_negative_infinity_are_not_valid_empty_returns(bad_value):
    scan = make_scan([bad_value, 1.0, math.inf, math.inf, math.inf])
    with pytest.raises(ValueError, match='range bin'):
        scan_statistics(scan)


@pytest.mark.parametrize('bad_range', [0.05, 40.01])
def test_finite_beam_must_stay_inside_scan_range(bad_range):
    scan = make_scan([bad_range, math.inf, math.inf, math.inf, math.inf])
    with pytest.raises(ValueError, match='range bin'):
        scan_statistics(scan)


def test_frame_and_configured_range_are_checked():
    scan = make_scan([1.0, math.inf, math.inf, math.inf, math.inf], frame='wrong')
    with pytest.raises(ValueError, match='frame_id'):
        scan_statistics(scan, expected_frame='mid360_scan_frame')
    scan.header.frame_id = 'mid360_scan_frame'
    with pytest.raises(ValueError, match='range_max'):
        scan_statistics(scan, expected_max_range=20.0)


@pytest.mark.parametrize('field,value', [
    ('angle_increment', 0.0), ('scan_time', -0.1), ('range_max', 0.0),
])
def test_invalid_scan_metadata_is_rejected(field, value):
    scan = make_scan([1.0, math.inf, math.inf, math.inf, math.inf])
    setattr(scan, field, value)
    with pytest.raises(ValueError):
        scan_statistics(scan)


def test_range_bin_count_must_match_angle_geometry():
    scan = make_scan([1.0, math.inf])
    scan.angle_max += 2 * scan.angle_increment
    with pytest.raises(ValueError, match='range bins'):
        scan_statistics(scan)


def test_non_integral_angle_span_is_rejected():
    with pytest.raises(ValueError, match='integer number'):
        _expected_sample_count(0.0, 1.0, 0.3)


def test_empty_scan_is_allowed_only_when_explicitly_requested():
    scan = make_scan([math.inf] * 5)
    with pytest.raises(ValueError, match='no finite'):
        scan_statistics(scan, require_finite=True)
    result = scan_statistics(scan, require_finite=False)
    assert result['finite_bins'] == 0
    assert result['empty_bins'] == 5


def test_height_filter_drops_ground_and_overhead_points_and_keeps_nearest():
    ranges = reference_project_points(
        [
            (2.0, 0.0, -0.01),       # ground, reject
            (3.0, 0.0, 0.25),        # retained, farther
            (1.5, 0.0, 0.50),        # retained, nearest in same bin
            (1.0, 0.0, 1.01),        # overhead, reject
            (0.5, 0.0, math.nan),     # invalid, reject
            (0.4, 0.0, 0.25),        # below range_min, reject
            (21.0, 0.0, 0.25),        # beyond range_max, reject
        ],
        min_height=0.05, max_height=0.90,
        angle_min=-math.pi, angle_max=math.pi,
        angle_increment=math.pi / 2,
        range_min=0.5, range_max=20.0)
    assert ranges[2] == pytest.approx(1.5)
    assert ranges[0] == math.inf
    assert all(value == math.inf for value in ranges[3:])


def test_height_filter_keeps_boundary_heights_and_separates_angles():
    ranges = reference_project_points(
        [
            (2.0, 0.0, 0.05),
            (0.0, 2.0, 0.90),
            (0.0, -2.0, 0.91),
        ],
        min_height=0.05, max_height=0.90,
        angle_min=-math.pi, angle_max=math.pi,
        angle_increment=math.pi / 2,
        range_min=0.1, range_max=20.0)
    assert ranges[2] == pytest.approx(2.0)
    assert ranges[3] == pytest.approx(2.0)
    assert ranges[1] == math.inf


@pytest.mark.parametrize('point', [(1.0, 2.0)])
def test_reference_projector_requires_xyz(point):
    with pytest.raises(ValueError, match='x, y and z'):
        reference_project_points(
            [point], min_height=0.0, max_height=1.0,
            angle_min=-math.pi, angle_max=math.pi,
            angle_increment=math.pi / 2, range_min=0.1, range_max=20.0)


def test_reference_projector_ignores_extra_point_fields():
    ranges = reference_project_points(
        [(1.0, 0.0, 0.2, 99.0)], min_height=0.0, max_height=1.0,
        angle_min=-math.pi, angle_max=math.pi,
        angle_increment=math.pi / 2, range_min=0.1, range_max=20.0)
    assert ranges[2] == pytest.approx(1.0)


def test_timestamp_helper_rejects_zero_and_accepts_positive():
    with pytest.raises(ValueError):
        stamp_nanoseconds(Time())
    assert stamp_nanoseconds(Time(sec=2, nanosec=3)) == 2_000_000_003


def test_cli_defaults_make_navigation_contract_explicit():
    options = parse_options([])
    assert options.topic == '/scan'
    assert options.frame == 'mid360_scan_frame'
    assert options.sensor_frame == 'mid360_sensor_frame'
    assert options.expected_min_range == pytest.approx(0.1)
    assert options.expected_max_range == pytest.approx(40.0)
    assert options.require_finite is True


@pytest.mark.parametrize('args', [
    ['--duration', '0'], ['--expected-min-range', '20', '--expected-max-range', '1'],
    ['--min-hz', '0'], ['--max-hz', 'nan'],
])
def test_invalid_cli_values_fail(args):
    with pytest.raises(SystemExit):
        parse_options(args)
