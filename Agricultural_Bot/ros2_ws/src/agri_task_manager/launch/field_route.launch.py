"""Launch the version-bound single-corridor route executor."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start the route executor with strict map and mask checks."""
    share = Path(get_package_share_directory('agri_task_manager'))
    default_mission = (
        share / 'config' / 'missions'
        / 'tomato_field_default_single_corridor.yaml')
    arguments = [
        DeclareLaunchArgument(
            'mission_file', default_value=str(default_mission)),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('keepout_yaml', default_value=''),
        DeclareLaunchArgument('field_metadata', default_value=''),
        DeclareLaunchArgument('field_sdf', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('require_motion_lock', default_value='true'),
        DeclareLaunchArgument('require_station_ack', default_value='false'),
        DeclareLaunchArgument('require_live_map', default_value='true'),
        DeclareLaunchArgument('require_live_keepout', default_value='true'),
    ]
    executor = Node(
        package='agri_task_manager',
        executable='field_route_executor',
        name='field_route_executor',
        output='screen',
        parameters=[{
            'mission_file': LaunchConfiguration('mission_file'),
            'map_yaml': LaunchConfiguration('map_yaml'),
            'keepout_yaml': LaunchConfiguration('keepout_yaml'),
            'field_metadata': LaunchConfiguration('field_metadata'),
            'field_sdf': LaunchConfiguration('field_sdf'),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
            'autostart': ParameterValue(
                LaunchConfiguration('autostart'), value_type=bool),
            'require_motion_lock': ParameterValue(
                LaunchConfiguration('require_motion_lock'), value_type=bool),
            'require_station_ack': ParameterValue(
                LaunchConfiguration('require_station_ack'), value_type=bool),
            'require_live_map': ParameterValue(
                LaunchConfiguration('require_live_map'), value_type=bool),
            'require_live_keepout': ParameterValue(
                LaunchConfiguration('require_live_keepout'), value_type=bool),
        }],
    )
    return LaunchDescription(arguments + [executor])
