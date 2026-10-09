"""Start the saved-map localization baseline (map_server and AMCL).

This launch file is deliberately separate from mapping.launch.py.  SLAM
Toolbox and AMCL must never publish ``map -> odom`` at the same time.
"""

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
    """Build map server, AMCL, and their lifecycle manager."""
    share_dir = get_package_share_directory('agri_navigation')
    default_map = os.path.join(share_dir, 'maps', 'stage3_baseline.yaml')
    default_params = os.path.join(share_dir, 'config', 'amcl.yaml')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    common_args = [
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Saved occupancy-grid YAML file.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='AMCL parameter YAML file.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically activate localization lifecycle nodes.'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
    ]

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
             'yaml_filename': map_file},
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'set_initial_pose': True,
                'initial_pose.x': _float_parameter('initial_pose_x'),
                'initial_pose.y': _float_parameter('initial_pose_y'),
                'initial_pose.yaw': _float_parameter('initial_pose_yaw'),
            },
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': ParameterValue(autostart, value_type=bool),
                'node_names': ['map_server', 'amcl'],
            },
        ],
    )
    return LaunchDescription(common_args + [map_server, amcl, lifecycle_manager])

