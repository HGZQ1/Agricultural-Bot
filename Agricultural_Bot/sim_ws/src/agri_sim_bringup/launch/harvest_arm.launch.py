"""Connect guarded arm work to an existing Gazebo and navigation session."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _include(package, filename, values):
    path = Path(get_package_share_directory(package)) / 'launch' / filename
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=values.items())


def _validate_assets(context):
    """Fail before arm nodes start if the selected world differs from the mission."""
    from agri_task_manager.waypoint_schema import load_mission, sha256_file

    mission_path = LaunchConfiguration('mission_file').perform(context)
    metadata = LaunchConfiguration('field_metadata').perform(context)
    world = LaunchConfiguration('field_sdf').perform(context)
    mission = load_mission(mission_path)
    if (sha256_file(metadata) != mission.field.metadata_sha256 or
            sha256_file(world) != mission.field.sdf_sha256):
        raise ValueError('harvest arm world/metadata do not match the mission binding')
    return []


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('mission_file', description='Prepared harvestable mission YAML'),
        DeclareLaunchArgument('field_metadata', description='Matching field metadata JSON'),
        DeclareLaunchArgument('field_sdf', description='Exact running harvestable world SDF'),
        DeclareLaunchArgument('gz_partition', description='Same partition as the running Gazebo'),
        DeclareLaunchArgument('world_name', default_value='field'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('fruit_arm_pass_through', default_value='false',
                              description='Simulation demo: allow fruit through arm links only'),
        DeclareLaunchArgument('ideal_gripper',
                              default_value='true' if os.environ.get('AGRI_IDEAL_SIMULATION') == '1' else 'false',
                              description='Simulation demo: directly follow commanded finger positions'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('keepout_yaml', default_value=''),
        DeclareLaunchArgument('worker_params_file', default_value=str(
            Path(get_package_share_directory('agri_arm_control')) /
            'config' / 'harvest_arm_worker.yaml')),
        DeclareLaunchArgument('targets_topic', default_value='/agri_vision/tomato_targets_base'),
        DeclareLaunchArgument('action_timeout_s', default_value='420.0'),
        DeclareLaunchArgument('work_timeout_s', default_value='440.0'),
        DeclareLaunchArgument('parking_timeout_s', default_value='15.0',
                              description='Bounded wall-clock budget for heavy simulation parking'),
    ]
    clock = LaunchConfiguration('use_sim_time')
    moveit = _include('agri_arm_moveit_config', 'moveit.launch.py', {
        'use_sim_time': clock, 'use_rviz': LaunchConfiguration('use_rviz'),
    })
    worker = _include('agri_arm_control', 'harvest_arm_worker.launch.py', {
        'use_sim_time': clock, 'params_file': LaunchConfiguration('worker_params_file'),
        'backend_mode': 'gazebo',
        'targets_topic': LaunchConfiguration('targets_topic'),
    })
    scene = _include('agri_harvest_sim', 'harvest_scene.launch.py', {
        'use_sim_time': clock, 'metadata': LaunchConfiguration('field_metadata'),
        'gz_partition': LaunchConfiguration('gz_partition'),
        'world_name': LaunchConfiguration('world_name'),
        'fruit_arm_pass_through': LaunchConfiguration('fruit_arm_pass_through'),
    })
    native_fruit_filter = Node(
        package='agri_harvest_sim', executable='fruit_arm_pass_through.py',
        condition=IfCondition(LaunchConfiguration('fruit_arm_pass_through')),
        arguments=['--enable', '--gz-partition', LaunchConfiguration('gz_partition'),
                   '--world-name', LaunchConfiguration('world_name')], output='screen')
    ideal_gripper = Node(
        package='agri_harvest_sim', executable='ideal_gripper.py',
        condition=IfCondition(LaunchConfiguration('ideal_gripper')),
        arguments=['--gz-partition', LaunchConfiguration('gz_partition'),
                   '--world-name', LaunchConfiguration('world_name'),
                   '--ideal-simulation', '--load-system'], output='screen')
    guarded_route = _include('agri_task_manager', 'harvest_preflight.launch.py', {
        'use_sim_time': clock, 'autostart': LaunchConfiguration('autostart'),
        'mission_file': LaunchConfiguration('mission_file'),
        'field_metadata': LaunchConfiguration('field_metadata'),
        'field_sdf': LaunchConfiguration('field_sdf'),
        'map_yaml': LaunchConfiguration('map_yaml'),
        'map_topic': LaunchConfiguration('map_topic'),
        'keepout_yaml': LaunchConfiguration('keepout_yaml'),
        'enable_mock_worker': 'false', 'worker_select_targets_after_scan': 'true',
        'require_segmentation': 'true', 'targets_topic': LaunchConfiguration('targets_topic'),
        'action_timeout_s': LaunchConfiguration('action_timeout_s'),
        'work_timeout_s': LaunchConfiguration('work_timeout_s'),
        'parking_timeout_s': LaunchConfiguration('parking_timeout_s'),
    })
    return LaunchDescription(arguments + [OpaqueFunction(function=_validate_assets),
                                         moveit, native_fruit_filter, ideal_gripper,
                                         scene, worker, guarded_route])
