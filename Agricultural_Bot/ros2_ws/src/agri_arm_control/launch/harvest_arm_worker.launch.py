"""Launch only the worker, reusing the already running MoveIt/Gazebo backend."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('agri_arm_control'),
                          'config', 'harvest_arm_worker.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('backend_mode', default_value='gazebo'),
        DeclareLaunchArgument('targets_topic', default_value='/agri_vision/tomato_targets_base'),
        Node(package='agri_arm_control', executable='harvest_arm_worker', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'backend_mode': LaunchConfiguration('backend_mode'),
                 'targets_topic': LaunchConfiguration('targets_topic')}]),
    ])
