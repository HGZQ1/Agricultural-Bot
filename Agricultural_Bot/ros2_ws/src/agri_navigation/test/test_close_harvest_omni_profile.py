"""Offline checks of the opt-in DWB/4WS close-harvest candidate.

Exercise real launch parameter expansion, installed plugin registration,
the actual velocity gate and measured-feedback kinematics; no nodes start.
"""

from math import hypot
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

from test_navigation_smoothing_launch import (
    launch_case, load_parameter_file, native_string_array,
)


ROOT = Path(__file__).resolve().parents[2]
NAV = ROOT / 'agri_navigation/config/nav2_close_harvest_omni.yaml'
BASE = ROOT / 'agri_base_adapter/config/base_adapter_close_harvest_omni.yaml'


def read(path):
    return yaml.safe_load(path.read_text())


def test_candidate_preserves_all_safety_and_changes_only_requested_controls():
    original = read(NAV.with_name('nav2_navigation.yaml'))
    candidate = read(NAV)
    original_controller = original['controller_server']['ros__parameters']
    candidate_controller = candidate['controller_server']['ros__parameters']
    original_controller.pop('FollowPath')
    original_controller.pop('.position_checker')
    candidate_controller.pop('FollowPath')
    assert candidate == original
    base = read(BASE)
    assert base['base_velocity_gate']['ros__parameters']['max_linear_y'] == .08
    base['base_velocity_gate']['ros__parameters']['max_linear_y'] = 0.
    assert base == read(BASE.with_name('base_adapter.yaml'))


def test_installed_plugins_and_native_ros_parameter_arrays_are_available():
    controller = read(NAV)['controller_server']['ros__parameters']
    settings = controller['FollowPath']
    classes = set()
    for package, filename in (
            ('dwb_core', 'local_planner_plugin.xml'),
            ('dwb_plugins', 'plugins.xml'),
            ('dwb_critics', 'default_critics.xml')):
        tree = ET.parse(Path(get_package_share_directory(package)) / filename)
        classes.update(entry.attrib['type'] for entry in tree.iter('class'))
    assert settings['plugin'] in classes
    assert settings['trajectory_generator_name'] in classes
    assert all(f'dwb_critics::{name}Critic' in classes for name in settings['critics'])
    assert native_string_array(NAV, 'controller_server', 'FollowPath.critics') == (
        settings['critics'])
    assert settings['critics'][0] == 'ObstacleFootprint'
    assert 'BaseObstacle' not in settings['critics']
    assert not {'PathAlign', 'GoalAlign', 'PreferForward'} & set(settings['critics'])
    assert 'Twirling' in settings['critics'] and 'RotateToGoal' in settings['critics']
    assert settings['sim_period'] == pytest.approx(1. / controller['controller_frequency'])
    assert hypot(settings['max_vel_x'], settings['max_vel_y']) <= .15
    assert settings['xy_goal_tolerance'] == controller['general_goal_checker']['xy_goal_tolerance']


def test_candidate_expands_through_main_launch_without_smoother(launch_case):
    configure, expand = launch_case
    context, nodes = configure(
        params_file=str(NAV), velocity_smoothing='false',
        navigation_map='/tmp/navigation collision.yaml', obstacle_scan='/scan_obstacles')
    assert nodes['velocity_smoother'].condition.evaluate(context) is False
    node, controller = expand('controller_server')
    assert controller['FollowPath']['plugin'] == 'dwb_core::DWBLocalPlanner'
    _, monitor = expand('collision_monitor')
    assert monitor['cmd_vel_in_topic'] == '/cmd_vel_nav_raw'
    assert monitor['cmd_vel_out_topic'] == '/cmd_vel_nav'
    parameters = load_parameter_file(Path(node._Node__expanded_parameter_arguments[0][0]))
    for name in ('local_costmap', 'global_costmap'):
        costmap = parameters[name][name]['ros__parameters']
        assert costmap['static_layer']['map_topic'] == '/navigation_map'
        assert costmap['obstacle_layer']['scan']['topic'] == '/scan_obstacles'
        assert costmap['inflation_layer']['inflation_radius'] == .40
        assert costmap['footprint_padding'] == .035
        assert costmap['keepout_filter']['enabled'] is True
    _, managed = expand('lifecycle_manager_navigation')
    assert 'velocity_smoother' not in managed['node_names']


@pytest.mark.parametrize('twist', [(0., .06, 0.), (.09, .09 * .75 / 1.18, 0.),
                                   (-.09, -.09 * .75 / 1.18, 0.)])
def test_gate_and_calibrated_four_wheel_feedback_preserve_lateral_command(twist):
    # Import the real neighboring packages from their source trees, without
    # depending on whether root has rebuilt their installed copies yet.
    for package in ('agri_base_adapter', 'agri_base_kinematics'):
        sys.path.insert(0, str(ROOT / package))
    from agri_base_adapter.velocity_gate import VelocityGateCore
    from agri_base_kinematics.kinematics import (
        FourWheelSteeringKinematics, calibrated_steering_limits,
        integrate_planar_pose,
    )
    limits = read(BASE)['base_velocity_gate']['ros__parameters']
    gate = VelocityGateCore(['navigation'], {'navigation': .35},
                            tuple(limits[name] for name in (
                                'max_linear_x', 'max_linear_y', 'max_angular_z')))
    assert gate.update('navigation', twist, 1.)
    assert gate.select(1.1)[0] == pytest.approx(twist)
    parameters = read(ROOT / 'agri_base_kinematics/config/four_wheel_steering.yaml')[
        'four_wheel_steering_node']['ros__parameters']
    names = parameters['wheel_names']
    coordinates = parameters['wheel_positions']
    positions = {name: coordinates[2 * index:2 * index + 2]
                 for index, name in enumerate(names)}
    steering_limits = calibrated_steering_limits(
        parameters['steering_limits'], parameters['steering_joint_limits'],
        parameters['steering_joint_signs'], parameters['steering_joint_offsets'])
    model = FourWheelSteeringKinematics(
        positions, parameters['wheel_radius'], parameters['max_wheel_speed'],
        tuple(parameters['steering_limits']), dict(zip(names, steering_limits)))
    commands = model.inverse(*twist, current_steering={name: 0. for name in names})
    measured = model.body_velocity(
        {name: command.steering for name, command in commands.items()},
        {name: command.velocity for name, command in commands.items()})
    assert measured == pytest.approx(twist)
    assert integrate_planar_pose(0., 0., 0., *measured, 1.) == pytest.approx(twist)
    gate.set_locked(True)
    assert gate.select(1.2)[0] == (0., 0., 0.)
