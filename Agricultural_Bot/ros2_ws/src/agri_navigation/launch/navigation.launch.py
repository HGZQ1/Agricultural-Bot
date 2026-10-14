"""Start saved-map AMCL and the single-goal Nav2 navigation baseline."""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from agri_navigation.navigation_parameters import configure_navigation_parameters
from agri_navigation.navigation_smoothing import (
    configure_velocity_smoothing, VELOCITY_SMOOTHER_REMAPPINGS,
    with_smoothing_lifecycle,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration, NotEqualsSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float_parameter(name: str) -> ParameterValue:
    """Return a launch argument converted to a ROS double parameter."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _smoothing_enabled(context):
    value = LaunchConfiguration('velocity_smoothing', default='false').perform(context).lower()
    if value not in {'true', 'false'}:
        raise ValueError('velocity_smoothing must be true or false')
    return value == 'true'


class _NavigationLifecycleNodes(Substitution):
    def perform(self, context):
        names = ['map_server', 'amcl', 'filter_mask_server',
                 'costmap_filter_info_server', 'controller_server', 'planner_server',
                 'behavior_server', 'bt_navigator', 'collision_monitor']
        return yaml.safe_dump(with_smoothing_lifecycle(names, _smoothing_enabled(context)),
                              default_flow_style=True).strip()


class _ConfiguredNavigationParameters(Substitution):
    """Resolve the launch inputs once into a correctly namespaced YAML file."""

    def __init__(self):
        super().__init__()
        self._resolved = {}

    def describe(self) -> str:
        return 'Nav2 parameters with separate localization/obstacle inputs'

    def perform(self, context) -> str:
        source = LaunchConfiguration('params_file').perform(context)
        obstacle_scan = LaunchConfiguration('obstacle_scan').perform(context)
        navigation_map = LaunchConfiguration('navigation_map').perform(context)
        smoothing = _smoothing_enabled(context)
        key = (source, obstacle_scan, navigation_map, smoothing)
        if key not in self._resolved:
            with open(source, encoding='utf-8') as stream:
                configured = configure_navigation_parameters(
                    yaml.safe_load(stream), obstacle_scan,
                    '/navigation_map' if navigation_map else None,
                )
                configured = configure_velocity_smoothing(configured, smoothing)
            with tempfile.NamedTemporaryFile(
                    mode='w', prefix='agri_navigation_', suffix='.yaml',
                    encoding='utf-8', delete=False) as stream:
                yaml.safe_dump(configured, stream, sort_keys=False)
                self._resolved[key] = stream.name
        return self._resolved[key]


def generate_launch_description() -> LaunchDescription:
    """Build localization, planner, controller, recovery, and BT nodes."""
    share_dir = get_package_share_directory('agri_navigation')
    # The packaged default is generated from the current default tomato field.
    # Keep ``stage3_baseline.yaml`` available as an explicit legacy fixture.
    default_map = os.path.join(share_dir, 'maps', 'tomato_field_default.yaml')
    default_keepout = os.path.join(
        share_dir, 'maps', 'tomato_field_default_keepout.yaml')
    default_params = os.path.join(share_dir, 'config', 'nav2_navigation.yaml')
    default_amcl_params = os.path.join(share_dir, 'config', 'amcl.yaml')

    map_file = LaunchConfiguration('map')
    navigation_map = LaunchConfiguration('navigation_map')
    keepout_mask = LaunchConfiguration('keepout_mask')
    params_file = _ConfiguredNavigationParameters()
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    launch_args = [
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'navigation_map', default_value='',
            description='Optional collision map YAML; AMCL keeps the map argument.'),
        DeclareLaunchArgument(
            'obstacle_scan', default_value='/scan',
            description='LaserScan for costmaps and collision monitor; AMCL keeps /scan.'),
        DeclareLaunchArgument('keepout_mask', default_value=default_keepout),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'amcl_params_file', default_value=default_amcl_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'velocity_smoothing', default_value='false',
            description='Optional conservative command smoothing before Collision Monitor.'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
    ]
    use_clock = ParameterValue(use_sim_time, value_type=bool)
    nav_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[
            amcl_params_file,
            {'use_sim_time': use_clock, 'yaml_filename': map_file},
        ],
        remappings=nav_remappings,
    )
    navigation_map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='navigation_map_server', output='screen',
        condition=IfCondition(NotEqualsSubstitution(navigation_map, '')),
        parameters=[{
            'use_sim_time': use_clock,
            'yaml_filename': navigation_map,
            'topic_name': '/navigation_map',
            'frame_id': 'map',
        }],
        remappings=nav_remappings,
    )
    navigation_map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation_map', output='screen',
        condition=IfCondition(NotEqualsSubstitution(navigation_map, '')),
        parameters=[{
            'use_sim_time': use_clock,
            'autostart': ParameterValue(autostart, value_type=bool),
            'node_names': ['navigation_map_server'],
        }],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[
            amcl_params_file,
            {
                'use_sim_time': use_clock,
                'set_initial_pose': True,
                'initial_pose.x': _float_parameter('initial_pose_x'),
                'initial_pose.y': _float_parameter('initial_pose_y'),
                'initial_pose.yaw': _float_parameter('initial_pose_yaw'),
            },
        ],
        remappings=nav_remappings,
    )
    filter_mask_server = Node(
        package='nav2_map_server', executable='map_server',
        name='filter_mask_server', output='screen',
        parameters=[{
            'use_sim_time': use_clock,
            'yaml_filename': keepout_mask,
            # Costmaps live in /global_costmap and /local_costmap namespaces;
            # publish the shared mask at the root so both filters consume the
            # same map rather than resolving separate relative topic names.
            'topic_name': '/keepout_filter_mask',
            'frame_id': 'map',
        }],
        remappings=nav_remappings,
    )
    filter_info_server = Node(
        package='nav2_map_server', executable='costmap_filter_info_server',
        name='costmap_filter_info_server', output='screen',
        parameters=[{
            'use_sim_time': use_clock,
            'type': 0,
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )
    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[params_file, {'use_sim_time': use_clock}],
        remappings=nav_remappings + [('cmd_vel', '/cmd_vel_nav_raw')],
    )
    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[params_file, {'use_sim_time': use_clock}],
        remappings=nav_remappings,
    )
    behavior = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[params_file, {'use_sim_time': use_clock}],
        remappings=nav_remappings + [('cmd_vel', '/cmd_vel_nav_raw')],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[params_file, {'use_sim_time': use_clock}],
        remappings=nav_remappings,
    )
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[params_file, {'use_sim_time': use_clock}],
        remappings=nav_remappings,
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        condition=IfCondition(LaunchConfiguration('velocity_smoothing', default='false')),
        parameters=[params_file, {'use_sim_time': use_clock}],
        remappings=nav_remappings + list(VELOCITY_SMOOTHER_REMAPPINGS),
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[
            {
                'use_sim_time': use_clock,
                'autostart': ParameterValue(autostart, value_type=bool),
                'node_names': _NavigationLifecycleNodes(),
            },
        ],
    )

    return LaunchDescription(
        launch_args + [
            map_server, navigation_map_server, navigation_map_lifecycle_manager,
            amcl, filter_mask_server, filter_info_server,
            controller, planner, behavior, bt_navigator, velocity_smoother, collision_monitor,
            lifecycle_manager,
        ])
