"""Offline contracts for saved-map navigation and field keepout generation."""

import importlib.util
import math
from pathlib import Path
import sys

import pytest
import yaml


PROJECT = Path(__file__).resolve().parents[4]
KEEPOUT_SCRIPT = (
    PROJECT / 'sim_ws' / 'src' / 'agri_greenhouse_worlds' / 'scripts'
    / 'generate_keepout_mask.py'
)
NAV_CONFIG = (
    PROJECT / 'ros2_ws' / 'src' / 'agri_navigation' / 'config'
    / 'nav2_navigation.yaml'
)
SPEC = importlib.util.spec_from_file_location(
    'generate_keepout_mask', KEEPOUT_SCRIPT)
keepout = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = keepout
SPEC.loader.exec_module(keepout)


def test_robot_start_pose_derives_world_to_map_transform():
    transform = keepout.transform_from_robot_start((0.0, -6.0, math.pi / 2))
    assert transform == pytest.approx((6.0, 0.0, -math.pi / 2))
    assert keepout.world_to_map((0.0, -6.0), transform) == pytest.approx(
        (0.0, 0.0))


def test_world_to_map_transform_preserves_robot_relative_axes():
    transform = keepout.transform_from_robot_start((2.0, 3.0, math.pi / 2))
    assert keepout.world_to_map((2.0, 4.0), transform) == pytest.approx(
        (1.0, 0.0))
    assert keepout.world_to_map((1.0, 3.0), transform) == pytest.approx(
        (0.0, 1.0))


def test_nav2_stage_four_plugins_and_frames_are_explicit():
    config = yaml.safe_load(NAV_CONFIG.read_text(encoding='utf-8'))
    controller = config['controller_server']['ros__parameters']
    planner = config['planner_server']['ros__parameters']
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = config['global_costmap']['global_costmap'][
        'ros__parameters']

    assert controller['enable_stamped_cmd_vel'] is True
    assert controller['progress_checker']['plugin'].endswith(
        'PoseProgressChecker')
    assert controller['FollowPath']['plugin'].endswith(
        'RegulatedPurePursuitController')
    assert planner['GridBased']['plugin'].endswith('NavfnPlanner')
    assert planner['GridBased']['allow_unknown'] is True
    assert local['robot_base_frame'] == 'base_footprint'
    assert global_costmap['robot_base_frame'] == 'base_footprint'
    assert local['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert global_costmap['plugins'] == [
        'static_layer', 'obstacle_layer', 'inflation_layer']
    assert global_costmap['static_layer']['footprint_clearing_enabled'] is True


def test_collision_monitor_connects_nav2_to_base_gate():
    config = yaml.safe_load(NAV_CONFIG.read_text(encoding='utf-8'))
    monitor = config['collision_monitor']['ros__parameters']
    assert monitor['enable_stamped_cmd_vel'] is True
    assert monitor['cmd_vel_in_topic'] == '/cmd_vel_nav_raw'
    assert monitor['cmd_vel_out_topic'] == '/cmd_vel_nav'
    assert yaml.safe_load(monitor['RobotStop']['points']) == [
        [0.50, 0.40], [0.50, -0.40], [-0.54, -0.40], [-0.54, 0.40],
    ]
