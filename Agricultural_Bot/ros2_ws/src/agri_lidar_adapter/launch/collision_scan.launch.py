"""Project the optional simulation collision lidar alongside the localization scan."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory('agri_lidar_adapter'))
    config = yaml.safe_load((share / 'config/lidar_to_scan.yaml').read_text())
    # Read the two existing parameter dictionaries explicitly so renamed nodes
    # retain the calibrated self-filter and the horizontal scan-frame limits.
    filter_params = config['mid360_navigation_cloud_filter']['ros__parameters']
    scan_params = config['mid360_pointcloud_to_laserscan']['ros__parameters']
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    filter_params.update({
        'use_sim_time': clock,
        'input_topic': '/hard_obstacles/points',
        'output_topic': '/hard_obstacles/navigation_points',
        'max_range': 8.0,
    })
    scan_params.update({'use_sim_time': clock, 'range_max': 8.0})
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='agri_lidar_adapter', executable='mid360_cloud_filter',
             name='hard_obstacle_cloud_filter', output='screen',
             parameters=[filter_params]),
        Node(package='pointcloud_to_laserscan',
             executable='pointcloud_to_laserscan_node',
             name='hard_obstacle_pointcloud_to_laserscan', output='screen',
             remappings=[('cloud_in', '/hard_obstacles/navigation_points'),
                         ('scan', '/scan_obstacles')],
             parameters=[scan_params]),
    ])
