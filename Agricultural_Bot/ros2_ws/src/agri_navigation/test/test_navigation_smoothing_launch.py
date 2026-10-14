"""Exercise optional smoothing through real launch_ros parameter evaluation.

Only Node's pre-execution substitution phase is invoked.  No ROS processes
are launched; lifecycle arrays are inspected in the actual generated YAML
that launch_ros would pass to the lifecycle manager executable.
"""

import ctypes
import importlib.util
from pathlib import Path

import pytest
import yaml

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from ament_index_python.packages import get_package_prefix


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch/navigation.launch.py'
RAW_TOPIC = '/cmd_vel_nav_raw'
SMOOTH_TOPIC = '/cmd_vel_nav_smoothed'
PROTECTED_STOP_TOPIC = '/cmd_vel_nav'
OLD_LIFECYCLE = ['map_server', 'amcl', 'filter_mask_server',
                 'costmap_filter_info_server', 'controller_server',
                 'planner_server', 'behavior_server', 'bt_navigator',
                 'collision_monitor']


class _RosYamlLoader(yaml.SafeLoader):
    """Handle launch_ros's native tuple-array serialization as a sequence."""


_RosYamlLoader.add_constructor(
    'tag:yaml.org,2002:python/tuple',
    lambda loader, node: loader.construct_sequence(node))


def load_parameter_file(path):
    # launch_ros converts evaluated arrays to tuples and yaml.dump emits a
    # tuple tag.  The ROS C parser handles this as a sequence.  Do not use
    # unrestricted yaml.load; only add that one sequence constructor.
    return yaml.load(path.read_text(), Loader=_RosYamlLoader)


def native_string_array(path, node_name, parameter_name):
    """Read a real string-array parameter using ROS's native C YAML parser.

    This calls no ROS context, node or DDS machinery.  The ctypes layouts
    below follow rcutils/allocator.h, rcutils/types/string_array.h and
    rcl_yaml_param_parser/types.h in the installed ROS distribution.
    """
    class Allocator(ctypes.Structure):
        _fields_ = [(name, ctypes.c_void_p) for name in (
            'allocate', 'deallocate', 'reallocate', 'zero_allocate', 'state')]

    class StringArray(ctypes.Structure):
        _fields_ = [('size', ctypes.c_size_t),
                    ('data', ctypes.POINTER(ctypes.c_char_p)),
                    ('allocator', Allocator)]

    class Variant(ctypes.Structure):
        _fields_ = [(name, ctypes.c_void_p) for name in (
            'bool_value', 'integer_value', 'double_value', 'string_value',
            'byte_array_value', 'bool_array_value', 'integer_array_value',
            'double_array_value')] + [
                ('string_array_value', ctypes.POINTER(StringArray))]

    parser = ctypes.CDLL(str(Path(get_package_prefix('rcl_yaml_param_parser')) /
                             'lib/librcl_yaml_param_parser.so'))
    rcutils = ctypes.CDLL(str(Path(get_package_prefix('rcutils')) / 'lib/librcutils.so'))
    rcutils.rcutils_get_default_allocator.restype = Allocator
    parser.rcl_yaml_node_struct_init.argtypes = [Allocator]
    parser.rcl_yaml_node_struct_init.restype = ctypes.c_void_p
    parser.rcl_parse_yaml_file.argtypes = [ctypes.c_char_p, ctypes.c_void_p]
    parser.rcl_parse_yaml_file.restype = ctypes.c_bool
    parser.rcl_yaml_node_struct_get.argtypes = [ctypes.c_char_p, ctypes.c_char_p,
                                               ctypes.c_void_p]
    parser.rcl_yaml_node_struct_get.restype = ctypes.POINTER(Variant)
    parser.rcl_yaml_node_struct_fini.argtypes = [ctypes.c_void_p]
    params = parser.rcl_yaml_node_struct_init(rcutils.rcutils_get_default_allocator())
    assert params
    try:
        assert parser.rcl_parse_yaml_file(str(path).encode(), params), (
            f'Native ROS parameter parser rejected {path}')
        value = parser.rcl_yaml_node_struct_get(
            node_name.encode(), parameter_name.encode(), params)
        assert value and value.contents.string_array_value, (
            f'{parameter_name} must be a native ROS string array, not a YAML string')
        strings = value.contents.string_array_value.contents
        return [strings.data[index].decode() for index in range(strings.size)]
    finally:
        parser.rcl_yaml_node_struct_fini(params)


@pytest.fixture
def launch_case(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    spec = importlib.util.spec_from_file_location('smoothing_launch_case', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda _name: str(PACKAGE_ROOT))
    description = module.generate_launch_description()
    nodes = {entity._Node__node_name: entity for entity in description.entities
             if isinstance(entity, Node)}
    context = LaunchContext()
    created = set()

    def configure(**arguments):
        context.launch_configurations.update(arguments)
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        return context, nodes

    def expand(name):
        node = nodes[name]
        # This is the native evaluator invoked by Node's actual launch code,
        # not a custom test implementation of substitution/YAML conversion.
        evaluated = evaluate_parameters(context, node._Node__parameters)
        created.update(value for value in evaluated
                       if isinstance(value, Path) and value.name.startswith('agri_navigation_'))
        node._perform_substitutions(context)
        parameters = {}
        for filename, is_file in node._Node__expanded_parameter_arguments:
            assert is_file, 'This launch uses YAML files for its ROS parameters.'
            path = Path(filename)
            if path.name.startswith(('agri_navigation_', 'launch_params_')):
                created.add(path)
            tree = load_parameter_file(path)
            for key in ('/**', name, f'/{name}'):
                if key in tree:
                    parameters.update(tree[key]['ros__parameters'])
        return node, parameters

    yield configure, expand
    for path in created:
        path.unlink(missing_ok=True)


@pytest.mark.parametrize('argument,enabled', [(None, False), ('false', False), ('true', True)])
def test_launch_conditions_routing_and_actual_lifecycle_yaml_agree(launch_case, argument, enabled):
    configure, expand = launch_case
    context, nodes = configure(**({} if argument is None else {'velocity_smoothing': argument}))
    assert context.launch_configurations['velocity_smoothing'] == ('true' if enabled else 'false')
    smoother = nodes['velocity_smoother']
    assert smoother.condition.evaluate(context) is enabled
    manager, managed = expand('lifecycle_manager_navigation')
    names = managed['node_names']
    assert isinstance(names, list)
    assert all(isinstance(name, str) for name in names)
    assert names == (OLD_LIFECYCLE[:-1] + ['velocity_smoother', 'collision_monitor']
                     if enabled else OLD_LIFECYCLE)
    assert names.count('velocity_smoother') == int(enabled)

    # Verify the resulting wildcard ROS parameter YAML itself, so a string
    # accidentally containing a list cannot pass by being reparsed in tests.
    manager_file = Path(manager._Node__expanded_parameter_arguments[0][0])
    manager_yaml = load_parameter_file(manager_file)
    assert manager_yaml['/**']['ros__parameters']['node_names'] == names
    assert not isinstance(manager_yaml['/**']['ros__parameters']['node_names'], str)
    assert native_string_array(manager_file, '/**', 'node_names') == names

    controller_node, controller = expand('controller_server')
    behavior_node, behavior = expand('behavior_server')
    monitor_node, monitor = expand('collision_monitor')
    assert ('cmd_vel', RAW_TOPIC) in controller_node.expanded_remapping_rules
    assert ('cmd_vel', RAW_TOPIC) in behavior_node.expanded_remapping_rules
    assert controller['enable_stamped_cmd_vel'] is True
    assert behavior['enable_stamped_cmd_vel'] is True
    assert monitor['enable_stamped_cmd_vel'] is True
    assert monitor['cmd_vel_in_topic'] == (SMOOTH_TOPIC if enabled else RAW_TOPIC)
    assert monitor['cmd_vel_out_topic'] == PROTECTED_STOP_TOPIC
    assert monitor['RobotStop']['points'] == (
        '[[0.50, 0.40], [0.50, -0.40], [-0.54, -0.40], [-0.54, 0.40]]')
    assert monitor['RobotStop']['action_type'] == 'stop'
    configured_path = Path(controller_node._Node__expanded_parameter_arguments[0][0])
    tree = yaml.safe_load(configured_path.read_text())
    assert ('velocity_smoother' in tree) is enabled
    # Every node must receive the same cached configured parameter file.
    assert monitor_node._Node__expanded_parameter_arguments[0][0] == str(configured_path)
    assert behavior_node._Node__expanded_parameter_arguments[0][0] == str(configured_path)
    if enabled:
        smoother_node, parameters = expand('velocity_smoother')
        assert smoother_node._Node__expanded_parameter_arguments[0][0] == str(configured_path)
        assert ('cmd_vel', RAW_TOPIC) in smoother_node.expanded_remapping_rules
        assert ('cmd_vel_smoothed', monitor['cmd_vel_in_topic']) in smoother_node.expanded_remapping_rules
        assert parameters['enable_stamped_cmd_vel'] is True
        assert parameters['stamp_smoothed_velocity_with_smoothing_time'] is False
        assert parameters['max_accel'] == [.01, 0., .02]
        assert parameters['max_decel'] == [-.01, 0., -.02]


def test_enabled_smoothing_resolves_dual_map_scan_and_typed_clock_together(launch_case, tmp_path):
    configure, expand = launch_case
    # Spaces in the source path exercise native ParameterFile substitution.
    parameter_source = tmp_path / 'navigation params with spaces.yaml'
    parameter_source.write_bytes((PACKAGE_ROOT / 'config/nav2_navigation.yaml').read_bytes())
    context, nodes = configure(
        velocity_smoothing='true', params_file=str(parameter_source),
        map='/tmp/localization map.yaml', navigation_map='/tmp/collision map.yaml',
        obstacle_scan='/scan_obstacles', use_sim_time='false')
    assert nodes['velocity_smoother'].condition.evaluate(context) is True
    assert nodes['navigation_map_server'].condition.evaluate(context) is True
    assert nodes['lifecycle_manager_navigation_map'].condition.evaluate(context) is True
    controller, _ = expand('controller_server')
    tree = yaml.safe_load(Path(controller._Node__expanded_parameter_arguments[0][0]).read_text())
    for name in ('global_costmap', 'local_costmap'):
        parameters = tree[name][name]['ros__parameters']
        assert parameters['static_layer']['map_topic'] == '/navigation_map'
        assert parameters['obstacle_layer']['scan']['topic'] == '/scan_obstacles'
        assert parameters['footprint_padding'] == .035
        assert parameters['inflation_layer']['inflation_radius'] == .40
        assert parameters['keepout_filter']['enabled'] is True
    _, monitor = expand('collision_monitor')
    _, smoother = expand('velocity_smoother')
    assert monitor['scan']['topic'] == '/scan_obstacles'
    assert monitor['cmd_vel_in_topic'] == SMOOTH_TOPIC
    assert smoother['use_sim_time'] is False
    assert monitor['use_sim_time'] is False
    _, localization_server = expand('map_server')
    _, navigation_server = expand('navigation_map_server')
    _, amcl = expand('amcl')
    assert localization_server['yaml_filename'] == '/tmp/localization map.yaml'
    assert navigation_server['yaml_filename'] == '/tmp/collision map.yaml'
    assert navigation_server['topic_name'] == '/navigation_map'
    assert amcl['scan_topic'] == '/scan'
    _, manager = expand('lifecycle_manager_navigation')
    _, navigation_map_manager = expand('lifecycle_manager_navigation_map')
    assert manager['use_sim_time'] is False
    assert navigation_map_manager['node_names'] == ['navigation_map_server']
    assert 'navigation_map_server' not in manager['node_names']
    map_manager_file = Path(nodes['lifecycle_manager_navigation_map'].
                            _Node__expanded_parameter_arguments[0][0])
    assert native_string_array(map_manager_file, '/**', 'node_names') == [
        'navigation_map_server']


def test_invalid_smoothing_flag_fails_real_parameter_evaluation(launch_case):
    configure, expand = launch_case
    configure(velocity_smoothing='yes')
    with pytest.raises(ValueError, match='velocity_smoothing must be true or false'):
        expand('controller_server')
    with pytest.raises(ValueError, match='velocity_smoothing must be true or false'):
        expand('lifecycle_manager_navigation')
