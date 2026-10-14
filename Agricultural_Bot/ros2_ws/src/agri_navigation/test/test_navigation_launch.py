"""Contract tests for the saved-map navigation launch description."""

import importlib.util
from pathlib import Path

from agri_navigation.navigation_parameters import configure_navigation_parameters
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
    if (isinstance(value, (tuple, list)) and value
            and all(isinstance(item, Substitution) for item in value)):
        return yaml.safe_load(perform_substitutions(context, list(value)))
    if isinstance(value, (tuple, list)):
        return [_resolve_substitutions(context, item) for item in value]
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


def test_default_inputs_preserve_the_existing_navigation_profile():
    """Omitting the optional collision map retains the shipped plugin setup."""
    original = yaml.safe_load(PARAMS_FILE.read_text(encoding='utf-8'))
    configured = configure_navigation_parameters(original, '/scan')

    assert configured == original
    assert configured is not original
    assert 'static_layer' not in configured['local_costmap'][
        'local_costmap']['ros__parameters']['plugins']


def test_separate_obstacles_keep_localization_and_safety_parameters_intact():
    """Only navigation inputs change, including the hard-stop scan input."""
    original = yaml.safe_load(PARAMS_FILE.read_text(encoding='utf-8'))
    original['amcl'] = {'ros__parameters': {'scan_topic': '/scan'}}
    configured = configure_navigation_parameters(
        original, '/scan_obstacles', '/navigation_map')

    assert configured['amcl'] == original['amcl']
    for name in ('global_costmap', 'local_costmap'):
        costmap = configured[name][name]['ros__parameters']
        before = original[name][name]['ros__parameters']
        assert costmap['obstacle_layer']['scan']['topic'] == '/scan_obstacles'
        assert costmap['static_layer']['map_topic'] == '/navigation_map'
        assert costmap['static_layer']['map_subscribe_transient_local'] is True
        assert costmap['static_layer']['enabled'] is True
        assert costmap['plugins'].count('static_layer') == 1
        for unchanged in ('footprint', 'footprint_padding', 'inflation_layer',
                          'keepout_filter'):
            assert costmap[unchanged] == before[unchanged]
    monitor = configured['collision_monitor']['ros__parameters']
    assert monitor['scan']['topic'] == '/scan_obstacles'
    assert monitor['RobotStop'] == original['collision_monitor'][
        'ros__parameters']['RobotStop']
    assert original['local_costmap']['local_costmap'][
        'ros__parameters']['obstacle_layer']['scan']['topic'] == '/scan'


def test_navigation_map_server_is_opt_in_and_uses_a_separate_lifecycle(
        monkeypatch, tmp_path):
    """An optional navigation map does not replace AMCL's original map server."""
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
    context.launch_configurations.update({
        'map': '/tmp/localization.yaml',
        'navigation_map': '',
    })
    server = nodes['navigation_map_server']
    manager = nodes['lifecycle_manager_navigation_map']
    assert server.condition.evaluate(context) is False
    assert manager.condition.evaluate(context) is False

    context.launch_configurations['navigation_map'] = '/tmp/collision map.yaml'
    assert server.condition.evaluate(context) is True
    assert manager.condition.evaluate(context) is True
    assert _literal_parameters(server, context)['topic_name'] == '/navigation_map'
    assert _literal_parameters(server, context)['frame_id'] == 'map'
    assert _literal_parameters(server, context)['yaml_filename'] == (
        '/tmp/collision map.yaml')
    assert _literal_parameters(nodes['map_server'], context)['yaml_filename'] == (
        '/tmp/localization.yaml')
    assert _literal_parameters(manager, context)['node_names'] == [
        'navigation_map_server']
    managed = _literal_parameters(
        nodes['lifecycle_manager_navigation'], context)['node_names']
    assert 'map_server' in managed and 'amcl' in managed
    assert 'navigation_map_server' not in managed


def test_parameter_file_rewrite_keeps_child_costmap_namespaces(monkeypatch,
                                                             tmp_path):
    """The actual launch substitution loads overrides in the child nodes."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    module = _load_navigation_launch()
    context = LaunchContext()
    context.launch_configurations.update({
        'params_file': str(PARAMS_FILE),
        'navigation_map': '/tmp/navigation map.yaml',
        'obstacle_scan': '/scan_obstacles',
    })
    rewritten = module._ConfiguredNavigationParameters()
    filename = rewritten.perform(context)
    try:
        data = yaml.safe_load(Path(filename).read_text(encoding='utf-8'))
        assert data['global_costmap']['global_costmap']['ros__parameters'][
            'static_layer']['map_topic'] == '/navigation_map'
        assert data['local_costmap']['local_costmap']['ros__parameters'][
            'obstacle_layer']['scan']['topic'] == '/scan_obstacles'
        assert rewritten.perform(context) == filename
    finally:
        Path(filename).unlink()

