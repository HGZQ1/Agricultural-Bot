"""Omni smoothing must reach the real launch without weakening stop gates."""
from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from agri_navigation.navigation_smoothing import configure_velocity_smoothing
from test_navigation_smoothing_launch import launch_case, native_string_array


CONFIG = Path(__file__).resolve().parents[1] / 'config'
V8 = CONFIG / 'nav2_close_harvest_omni_v8.yaml'


def read(path):
    return yaml.safe_load(path.read_text())


def test_v8_only_adds_smoother_to_v7():
    original = read(CONFIG / 'nav2_close_harvest_omni_v7.yaml')
    candidate = read(V8)
    profile = candidate.pop('velocity_smoother')['ros__parameters']
    assert candidate == original
    assert profile['max_velocity'] == [.12, .08, .05]
    assert profile['min_velocity'] == [-.12, -.08, -.05]
    assert profile['max_accel'] == [.006, .006, .02]
    assert profile['max_decel'] == [-.006, -.006, -.02]
    assert profile['velocity_timeout'] == .25


def test_explicit_lateral_api_retains_reverse_and_downstream_stop():
    old = read(CONFIG / 'nav2_navigation.yaml')
    candidate = configure_velocity_smoothing(
        old, True, linear_velocity=.12, lateral_velocity=.08, angular_velocity=.05,
        linear_acceleration=.006, lateral_acceleration=.006)
    profile = candidate['velocity_smoother']['ros__parameters']
    assert profile['max_velocity'] == [.12, .08, .05]
    assert profile['max_accel'] == [.006, .006, .02]
    assert profile['min_velocity'] == [-.12, -.08, -.05]
    assert candidate['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] == '/cmd_vel_nav'
    assert old == read(CONFIG / 'nav2_navigation.yaml')


@pytest.mark.parametrize('enabled', [False, True])
def test_real_launch_keeps_omni_profile_and_collision_ownership(launch_case, enabled):
    configure, expand = launch_case
    context, nodes = configure(
        params_file=str(V8), amcl_params_file=str(V8),
        navigation_map='/tmp/collision map.yaml', obstacle_scan='/scan_obstacles',
        velocity_smoothing=str(enabled).lower())
    assert nodes['velocity_smoother'].condition.evaluate(context) is enabled
    _, monitor = expand('collision_monitor')
    _, smoother = expand('velocity_smoother')
    assert smoother == read(V8)['velocity_smoother']['ros__parameters']
    assert monitor['cmd_vel_in_topic'] == (
        '/cmd_vel_nav_smoothed' if enabled else '/cmd_vel_nav_raw')
    assert monitor['cmd_vel_out_topic'] == '/cmd_vel_nav'
    assert monitor['RobotStop'] == read(V8)['collision_monitor']['ros__parameters']['RobotStop']
    assert monitor['scan']['topic'] == '/scan_obstacles'
    _, controller = expand('controller_server')
    assert controller == read(V8)['controller_server']['ros__parameters']
    _, amcl = expand('amcl')
    assert amcl['robot_model_type'] == 'nav2_amcl::OmniMotionModel'
    assert amcl['scan_topic'] == '/scan'
    manager, managed = expand('lifecycle_manager_navigation')
    assert ('velocity_smoother' in managed['node_names']) is enabled
    if enabled:
        assert managed['node_names'][-2:] == ['velocity_smoother', 'collision_monitor']
    native = Path(manager._Node__expanded_parameter_arguments[0][0])
    assert native_string_array(native, '/**', 'node_names') == managed['node_names']


@pytest.mark.parametrize('name,value', [
    ('feedback', 'CLOSED_LOOP'), ('enable_stamped_cmd_vel', False),
    ('scale_velocities', False), ('stamp_smoothed_velocity_with_smoothing_time', True),
    ('max_velocity', [.12, float('nan'), .05]), ('max_velocity', [.12, .08]),
    ('min_velocity', [-.12, .08, -.05]), ('max_accel', [.006, 0., .02]),
    ('max_decel', [-.006, .006, -.02]), ('velocity_timeout', 0.),
    ('smoothing_frequency', 1001.), ('unknown_setting', 1),
])
def test_malformed_native_profile_fails_before_launch(name, value):
    candidate = read(V8)
    before = deepcopy(candidate)
    candidate['velocity_smoother']['ros__parameters'][name] = value
    with pytest.raises(ValueError):
        configure_velocity_smoothing(candidate, True)
    assert before == read(V8)
    assert candidate['collision_monitor'] == before['collision_monitor']


@pytest.mark.parametrize('value', [-.08, float('nan'), float('inf')])
def test_invalid_lateral_velocity_fails_before_launch(value):
    with pytest.raises(ValueError):
        configure_velocity_smoothing(read(V8), True, lateral_velocity=value)
