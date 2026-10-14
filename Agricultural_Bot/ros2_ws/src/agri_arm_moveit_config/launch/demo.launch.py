"""Compatibility entry for the standalone mock; never launch beside Gazebo."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    source = os.path.join(get_package_share_directory("agri_arm_moveit_config"), "launch", "mock_arm.launch.py")
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(source))])
