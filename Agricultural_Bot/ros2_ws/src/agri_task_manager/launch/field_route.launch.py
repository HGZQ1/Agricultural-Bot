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
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument(
            'keepout_topic', default_value='/keepout_filter_mask'),
        DeclareLaunchArgument(
            'navigate_action', default_value='/navigate_to_pose'),
        DeclareLaunchArgument(
            'navigate_status_topic',
            default_value='/navigate_to_pose/_action/status'),
        DeclareLaunchArgument(
            'motion_lock_service', default_value='/base_motion/set_lock'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument(
            'require_idle_navigator_on_start', default_value='true'),
        DeclareLaunchArgument('require_motion_lock', default_value='true'),
        DeclareLaunchArgument(
            'motion_lock_state_topic', default_value='/base_motion/locked'),
        DeclareLaunchArgument('require_station_ack', default_value='false'),
        DeclareLaunchArgument('require_live_map', default_value='true'),
        DeclareLaunchArgument('require_live_keepout', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('parking_settle_s', default_value='0.5'),
        DeclareLaunchArgument('parking_timeout_s', default_value='5.0'),
        DeclareLaunchArgument('parking_tf_max_age_s', default_value='0.35'),
        DeclareLaunchArgument('parking_position_tolerance_m', default_value='0.10'),
        DeclareLaunchArgument('parking_yaw_tolerance_rad', default_value='0.08726646259971647'),
        DeclareLaunchArgument('parking_max_retries', default_value='3'),
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
            'map_topic': LaunchConfiguration('map_topic'),
            'keepout_topic': LaunchConfiguration('keepout_topic'),
            'navigate_action': LaunchConfiguration('navigate_action'),
            'navigate_status_topic': LaunchConfiguration(
                'navigate_status_topic'),
            'motion_lock_service': LaunchConfiguration(
                'motion_lock_service'),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
            'autostart': ParameterValue(
                LaunchConfiguration('autostart'), value_type=bool),
            'require_idle_navigator_on_start': ParameterValue(
                LaunchConfiguration('require_idle_navigator_on_start'),
                value_type=bool),
            'require_motion_lock': ParameterValue(
                LaunchConfiguration('require_motion_lock'), value_type=bool),
            'motion_lock_state_topic': LaunchConfiguration(
                'motion_lock_state_topic'),
            'require_station_ack': ParameterValue(
                LaunchConfiguration('require_station_ack'), value_type=bool),
            'require_live_map': ParameterValue(
                LaunchConfiguration('require_live_map'), value_type=bool),
            'require_live_keepout': ParameterValue(
                LaunchConfiguration('require_live_keepout'), value_type=bool),
            'base_frame': LaunchConfiguration('base_frame'),
            **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
               for name in (
                   'parking_settle_s', 'parking_timeout_s', 'parking_tf_max_age_s',
                   'parking_position_tolerance_m', 'parking_yaw_tolerance_rad')},
            'parking_max_retries': ParameterValue(
                LaunchConfiguration('parking_max_retries'), value_type=int),
        }],
    )
    return LaunchDescription(arguments + [executor])
