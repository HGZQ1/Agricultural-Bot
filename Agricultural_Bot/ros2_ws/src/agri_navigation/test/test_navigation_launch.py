"""Contract tests for the saved-map navigation launch description."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.substitution import Substitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
PARAMS_FILE = PACKAGE_ROOT / 'config' / 'nav2_navigation.yaml'


def _load_navigation_launch():
    spec = importlib.util.spec_from_file_location(
        'navigation_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _resolve_substitutions(context, value):
    if (isinstance(value, tuple)
            and all(isinstance(item, Substitution) for item in value)):
        return yaml.safe_load(perform_substitutions(context, list(value)))
    return value


def _literal_parameters(node, context):
    """Resolve the inline parameter dictionary of a launch Node action."""
    result = {}
    for source in node._Node__parameters:
        if not isinstance(source, dict):
            continue
        for key, value in source.items():
            result[perform_substitutions(context, list(key))] = (
                _resolve_substitutions(context, value))
    return result


def test_keepout_topics_are_absolute_and_shared(monkeypatch, tmp_path):
    """Both namespaced costmaps must receive one root-level keepout mask."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    module = _load_navigation_launch()
    monkeypatch.setattr(
        module, 'get_package_share_directory', lambda _name: str(PACKAGE_ROOT))

    nodes = {
        entity._Node__node_name: entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, Node)
    }
    context = LaunchContext()
    context.launch_configurations['keepout_mask'] = '/tmp/keepout.yaml'
    mask_parameters = _literal_parameters(
        nodes['filter_mask_server'], context)
    info_parameters = _literal_parameters(
        nodes['costmap_filter_info_server'], context)

    assert mask_parameters['topic_name'] == '/keepout_filter_mask'
    assert info_parameters['filter_info_topic'] == '/costmap_filter_info'
    assert info_parameters['mask_topic'] == mask_parameters['topic_name']


def test_controller_closes_station_yaw_without_breaking_reverse_return():
    """Keep terminal-yaw correction and same-heading reverse compatible."""
    parameters = yaml.safe_load(PARAMS_FILE.read_text(encoding='utf-8'))
    controller = parameters['controller_server']['ros__parameters']
    follow_path = controller['FollowPath']
    planner = parameters['planner_server']['ros__parameters']['GridBased']

    assert follow_path['plugin'] == (
        'nav2_rotation_shim_controller::RotationShimController')
    assert follow_path['primary_controller'] == (
        'nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController')
    assert follow_path['rotate_to_goal_heading'] is True
    assert follow_path['rotate_to_heading_once'] is True
    assert follow_path['use_path_orientations'] is True
    assert follow_path['closed_loop'] is False
    assert follow_path['use_rotate_to_heading'] is False
    assert follow_path['allow_reversing'] is True
    assert planner['use_final_approach_orientation'] is False
    # A first XY hit cannot remain latched while four-wheel steering changes
    # position during the final heading correction.
    assert controller['general_goal_checker']['stateful'] is False
    assert controller['.position_checker']['stateful'] is False
    assert controller['general_goal_checker']['yaw_goal_tolerance'] <= (
        follow_path['angular_disengage_threshold'])

