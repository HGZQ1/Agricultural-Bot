"""Offline contract tests for the stage-three mapping baseline."""

import math

from agri_sim_tests.check_mapping import (
    parse_options,
    validate_map,
    validate_odom,
    validate_transform,
)
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
import pytest


def make_map(*, stamp=1, width=3, height=2, resolution=0.05,
             data=None, frame='map'):
    message = OccupancyGrid()
    message.header.frame_id = frame
    message.header.stamp = Time(sec=stamp)
    message.info = MapMetaData(
        map_load_time=Time(sec=stamp), resolution=resolution,
        width=width, height=height)
    message.info.origin.orientation.w = 1.0
    message.data = list(data if data is not None else [0, 100, -1, 0, 0, 0])
    return message


def make_odom(*, stamp=1, frame='odom', child='base_footprint'):
    message = Odometry()
    message.header.frame_id = frame
    message.header.stamp = Time(sec=stamp)
    message.child_frame_id = child
    message.pose.pose.orientation.w = 1.0
    return message


def test_valid_map_reports_known_free_and_occupied_cells():
    result = validate_map(make_map())
    assert result['width'] == 3
    assert result['height'] == 2
    assert result['known_cells'] == 5
    assert result['unknown_cells'] == 1
    assert result['free_cells'] == 4
    assert result['occupied_cells'] == 1


def test_unknown_only_map_is_rejected_by_default_but_can_be_allowed():
    message = make_map(data=[-1] * 6)
    with pytest.raises(ValueError, match='no known'):
        validate_map(message)
    assert validate_map(message, allow_unknown=True)['known_cells'] == 0


@pytest.mark.parametrize('mutator,pattern', [
    (lambda message: setattr(message.info, 'resolution', 0.1), 'resolution'),
    (lambda message: setattr(message, 'data', [0]), 'data has'),
    (lambda message: setattr(message, 'data', [101] * 6), 'occupancy'),
    (lambda message: setattr(message.header, 'frame_id', 'odom'), 'map frame'),
])
def test_invalid_map_contract_is_rejected(mutator, pattern):
    message = make_map()
    mutator(message)
    with pytest.raises(ValueError, match=pattern):
        validate_map(message)


def test_map_origin_quaternion_must_be_normalized():
    message = make_map()
    message.info.origin.orientation = Quaternion(w=2.0)
    with pytest.raises(ValueError, match='normalized'):
        validate_map(message)


def test_valid_odom_reports_navigation_frames_and_speed():
    message = make_odom()
    message.twist.twist.linear.x = 0.2
    result = validate_odom(message)
    assert result['frame_id'] == 'odom'
    assert result['child_frame_id'] == 'base_footprint'
    assert result['linear_speed_mps'] == pytest.approx(0.2)


@pytest.mark.parametrize('frame,child,pattern', [
    ('map', 'base_footprint', 'odometry frame'),
    ('odom', 'base_link', 'child frame'),
])
def test_odom_frame_pair_is_checked(frame, child, pattern):
    with pytest.raises(ValueError, match=pattern):
        validate_odom(make_odom(frame=frame, child=child))


def test_odom_rejects_nonfinite_pose_and_bad_quaternion():
    message = make_odom()
    message.pose.pose.position.x = math.nan
    with pytest.raises(ValueError, match='non-finite'):
        validate_odom(message)
    message = make_odom()
    message.pose.pose.orientation.w = 2.0
    with pytest.raises(ValueError, match='normalized'):
        validate_odom(message)


def test_static_transform_may_use_zero_stamp_explicitly():
    message = TransformStamped()
    message.header.frame_id = 'base_footprint'
    message.child_frame_id = 'mid360_scan_frame'
    message.transform.rotation.w = 1.0
    result = validate_transform(
        message, parent='base_footprint', child='mid360_scan_frame',
        allow_zero_stamp=True)
    assert result['stamp_ns'] == 0
    with pytest.raises(ValueError, match='timestamp'):
        validate_transform(
            message, parent='base_footprint', child='mid360_scan_frame')


def test_mapping_cli_defaults_match_stage_three_contract():
    options = parse_options([])
    assert options.map_topic == '/map'
    assert options.scan_topic == '/scan'
    assert options.odom_topic == '/odom'
    assert options.map_frame == 'map'
    assert options.odom_frame == 'odom'
    assert options.base_frame == 'base_footprint'
    assert options.scan_frame == 'mid360_scan_frame'
    assert options.expected_resolution == pytest.approx(0.05)
    assert options.allow_unknown_map is False


@pytest.mark.parametrize('args', [
    ['--duration', '0'],
    ['--expected-resolution', '0'],
    ['--min-scan-hz', '20', '--max-scan-hz', '1'],
    ['--min-odom-hz', 'nan'],
])
def test_mapping_cli_rejects_invalid_values(args):
    with pytest.raises(SystemExit):
        parse_options(args)
