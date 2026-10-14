"""Optional smoothing must preserve the downstream safety chain and assets."""

from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from agri_navigation.navigation_parameters import configure_navigation_parameters
from agri_navigation.navigation_smoothing import (
    RAW_COMMAND_TOPIC, SMOOTH_COMMAND_TOPIC, VELOCITY_SMOOTHER_REMAPPINGS,
    configure_velocity_smoothing, with_smoothing_lifecycle)


def navigation_parameters():
    path = Path(__file__).resolve().parents[1] / 'config/nav2_navigation.yaml'
    return yaml.safe_load(path.read_text())


def test_disabled_smoothing_preserves_old_configuration_and_lifecycle():
    original = navigation_parameters()
    assert configure_velocity_smoothing(original) == original
    nodes = ['controller_server', 'planner_server', 'behavior_server',
             'bt_navigator', 'collision_monitor']
    assert with_smoothing_lifecycle(nodes) == nodes
    assert 'velocity_smoother' not in original


def test_enabled_smoothing_changes_only_monitor_input_and_adds_stamped_profile():
    original = navigation_parameters()
    before = deepcopy(original)
    result = configure_velocity_smoothing(original, True)
    assert original == before
    expected = deepcopy(original)
    expected['collision_monitor']['ros__parameters']['cmd_vel_in_topic'] = SMOOTH_COMMAND_TOPIC
    assert {name: value for name, value in result.items()
            if name != 'velocity_smoother'} == expected
    monitor = result['collision_monitor']['ros__parameters']
    assert monitor['cmd_vel_out_topic'] == '/cmd_vel_nav'
    assert monitor['RobotStop'] == original['collision_monitor']['ros__parameters']['RobotStop']
    assert VELOCITY_SMOOTHER_REMAPPINGS == (
        ('cmd_vel', RAW_COMMAND_TOPIC), ('cmd_vel_smoothed', SMOOTH_COMMAND_TOPIC))
    profile = result['velocity_smoother']['ros__parameters']
    assert profile['enable_stamped_cmd_vel'] is True
    assert profile['stamp_smoothed_velocity_with_smoothing_time'] is False
    assert profile['feedback'] == 'OPEN_LOOP'
    assert profile['scale_velocities'] is True
    assert profile['max_accel'] == [.01, 0., .02]
    assert profile['max_decel'] == [-.01, 0., -.02]
    assert profile['min_velocity'][0] < 0.  # Reverse navigation is retained.
    assert profile['velocity_timeout'] < monitor['source_timeout']
    nodes = ['controller_server', 'planner_server', 'behavior_server',
             'bt_navigator', 'collision_monitor']
    managed = with_smoothing_lifecycle(nodes, True)
    assert managed == nodes[:-1] + ['velocity_smoother', 'collision_monitor']
    assert with_smoothing_lifecycle(managed, True) == managed


def test_smoothing_composes_with_dual_maps_without_changing_amcl_or_safety():
    original = navigation_parameters()
    dual = configure_navigation_parameters(original, '/scan_obstacles', '/navigation_map')
    result = configure_velocity_smoothing(dual, True)
    for name in ('local_costmap', 'global_costmap'):
        assert result[name] == dual[name]
    for name in ('amcl', 'map_server'):
        assert result.get(name) == original.get(name)
    assert result['collision_monitor']['ros__parameters']['scan']['topic'] == '/scan_obstacles'
    assert result['collision_monitor']['ros__parameters']['RobotStop'] == (
        original['collision_monitor']['ros__parameters']['RobotStop'])


@pytest.mark.parametrize('node', ['controller_server', 'behavior_server', 'collision_monitor'])
def test_mixed_twist_types_are_rejected(node):
    parameters = navigation_parameters()
    parameters[node]['ros__parameters']['enable_stamped_cmd_vel'] = False
    with pytest.raises(ValueError, match='must use stamped'):
        configure_velocity_smoothing(parameters, True)


@pytest.mark.parametrize('change', [
    {'linear_acceleration': 0.}, {'angular_acceleration': -1.},
    {'velocity_timeout': float('nan')}, {'smoothing_frequency': 1001.},
])
def test_invalid_motion_limits_are_rejected(change):
    with pytest.raises(ValueError):
        configure_velocity_smoothing(navigation_parameters(), True, **change)


def test_smoother_cannot_route_collision_stop_output_back_into_ramp():
    parameters = navigation_parameters()
    parameters['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] = RAW_COMMAND_TOPIC
    with pytest.raises(ValueError, match='remain downstream'):
        configure_velocity_smoothing(parameters, True)
    with pytest.raises(ValueError, match='managed collision monitor'):
        with_smoothing_lifecycle(['controller_server'], True)
