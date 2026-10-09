"""Start saved-map AMCL and the single-goal Nav2 navigation baseline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float_parameter(name: str) -> ParameterValue:
    """Return a launch argument converted to a ROS double parameter."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description() -> LaunchDescription:
    """Build localization, planner, controller, recovery, and BT nodes."""
    share_dir = get_package_share_directory('agri_navigation')
    default_map = os.path.join(share_dir, 'maps', 'stage3_baseline.yaml')
    default_params = os.path.join(share_dir, 'config', 'nav2_navigation.yaml')
    default_amcl_params = os.path.join(share_dir, 'config', 'amcl.yaml')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    launch_args = [
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'amcl_params_file', default_value=default_amcl_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
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
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[
            {
                'use_sim_time': use_clock,
                'autostart': ParameterValue(autostart, value_type=bool),
                'node_names': [
                    'map_server', 'amcl',
                    'controller_server', 'planner_server', 'behavior_server',
                    'bt_navigator', 'collision_monitor',
                ],
            },
        ],
    )

    return LaunchDescription(
        launch_args + [
            map_server, amcl, controller, planner, behavior, bt_navigator,
            collision_monitor, lifecycle_manager,
        ])
