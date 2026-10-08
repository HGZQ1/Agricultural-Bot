from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    share = Path(get_package_share_directory('agri_lidar_adapter'))
    default_config = str(share / 'config' / 'lidar_to_scan.yaml')

    config = LaunchConfiguration('config')
    input_topic = LaunchConfiguration('input_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    target_frame = LaunchConfiguration('target_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

    filter_node = Node(
        package='agri_lidar_adapter',
        executable='mid360_cloud_filter',
        name='mid360_navigation_cloud_filter',
        output='screen',
        parameters=[config, {
            'use_sim_time': use_sim_time,
            'input_topic': input_topic,
            'output_topic': cloud_topic,
            'target_frame': target_frame,
            'min_range': _float('range_min'),
            'max_range': _float('range_max'),
            'min_height': _float('min_height'),
            'max_height': _float('max_height'),
            'voxel_size': _float('voxel_size'),
        }],
    )
    scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='mid360_pointcloud_to_laserscan',
        output='screen',
        remappings=[('cloud_in', cloud_topic), ('scan', scan_topic)],
        parameters=[config, {
            'use_sim_time': use_sim_time,
            'target_frame': target_frame,
            'min_height': _float('min_height'),
            'max_height': _float('max_height'),
            'range_min': _float('range_min'),
            'range_max': _float('range_max'),
            'angle_increment': _float('angle_increment'),
            'scan_time': 0.10,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('input_topic', default_value='/mid360/points'),
        DeclareLaunchArgument('cloud_topic', default_value='/mid360/navigation_points'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('target_frame', default_value='mid360_scan_frame'),
        DeclareLaunchArgument('min_height', default_value='-0.40'),
        DeclareLaunchArgument('max_height', default_value='0.40'),
        DeclareLaunchArgument('range_min', default_value='0.10'),
        DeclareLaunchArgument('range_max', default_value='40.0'),
        DeclareLaunchArgument('angle_increment', default_value='0.017453292519943295'),
        DeclareLaunchArgument('voxel_size', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        filter_node,
        scan_node,
    ])
