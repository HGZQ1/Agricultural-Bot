"""Start guarded harvest routing with an optional isolated mock worker."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start guarded routing, target coordination and an optional mock worker."""
    share = Path(get_package_share_directory('agri_task_manager'))
    values = {
        'use_sim_time': 'true', 'autostart': 'false',
        'mission_file': str(share / 'config/missions/tomato_field_default_single_corridor.yaml'),
        'field_metadata': '', 'field_sdf': '',
        'map_yaml': '', 'map_topic': '/map', 'keepout_yaml': '',
        'require_live_map': 'true', 'require_live_keepout': 'true',
        'enable_mock_worker': 'true', 'failure_mode': 'none',
        'work_duration_s': '1.0', 'retract_duration_s': '0.5',
        'scan_timeout_s': '20.0', 'action_timeout_s': '30.0',
        'work_timeout_s': '45.0',
        'parking_timeout_s': '5.0',
        'targets_topic': '/agri_vision/tomato_targets_base',
        'require_segmentation': 'true',
        'worker_select_targets_after_scan': 'false',
    }
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, value in values.items()]
    route = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/field_route.launch.py')),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name in (
                'use_sim_time', 'autostart', 'mission_file', 'field_metadata',
                'field_sdf', 'map_yaml', 'map_topic', 'keepout_yaml',
                'require_live_map', 'require_live_keepout', 'work_timeout_s',
                'parking_timeout_s')},
            'require_motion_lock': 'true', 'require_station_ack': 'true',
            'require_harvest_handshake': 'true',
        }.items())
    coordinator = Node(
        package='agri_task_manager', executable='harvest_coordinator', output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'targets_topic': LaunchConfiguration('targets_topic'),
            'worker_select_targets_after_scan': ParameterValue(
                LaunchConfiguration('worker_select_targets_after_scan'), value_type=bool),
            'require_segmentation': ParameterValue(
                LaunchConfiguration('require_segmentation'), value_type=bool),
            'scan_timeout_s': ParameterValue(
                LaunchConfiguration('scan_timeout_s'), value_type=float),
            'action_timeout_s': ParameterValue(
                LaunchConfiguration('action_timeout_s'), value_type=float),
        }])
    worker = Node(
        package='agri_task_manager', executable='mock_harvest_worker', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_mock_worker')),
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'failure_mode': LaunchConfiguration('failure_mode'),
            'require_segmentation': ParameterValue(
                LaunchConfiguration('require_segmentation'), value_type=bool),
            'work_duration_s': ParameterValue(
                LaunchConfiguration('work_duration_s'), value_type=float),
            'retract_duration_s': ParameterValue(
                LaunchConfiguration('retract_duration_s'), value_type=float),
        }])
    return LaunchDescription(arguments + [route, coordinator, worker])
