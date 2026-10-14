from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    metadata = Path(get_package_share_directory('agri_greenhouse_worlds')) / 'worlds/tomato_field_default.json'
    return LaunchDescription([
        DeclareLaunchArgument('metadata', default_value=str(metadata)),
        DeclareLaunchArgument('world_name', default_value='field'),
        DeclareLaunchArgument('gz_partition', default_value='agricultural_bot_tomato_field'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_request_timeout_s', default_value='5.0'),
        DeclareLaunchArgument('moveit_ack_timeout_s', default_value='3.0'),
        DeclareLaunchArgument('command_lock_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('allow_cli_fallback', default_value='false'),
        DeclareLaunchArgument('fruit_arm_pass_through', default_value='false'),
        Node(package='agri_harvest_sim', executable='harvest_scene.py', output='screen', parameters=[{
            'metadata': LaunchConfiguration('metadata'),
            'world_name': LaunchConfiguration('world_name'),
            'gz_partition': LaunchConfiguration('gz_partition'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gazebo_request_timeout_s': LaunchConfiguration('gazebo_request_timeout_s'),
            'moveit_ack_timeout_s': LaunchConfiguration('moveit_ack_timeout_s'),
            'command_lock_timeout_s': LaunchConfiguration('command_lock_timeout_s'),
            'allow_cli_fallback': LaunchConfiguration('allow_cli_fallback'),
            'fruit_arm_pass_through': LaunchConfiguration('fruit_arm_pass_through'),
        }]),
    ])
