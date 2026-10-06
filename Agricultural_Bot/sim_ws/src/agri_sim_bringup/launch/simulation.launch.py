from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = Path(get_package_share_directory("agri_sim_description"))
    simulation = sim_share / "launch" / "simulation.launch.py"
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="empty_greenhouse.sdf"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("headless_rendering", default_value="true"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("world_name", default_value=""),
            DeclareLaunchArgument("gz_partition", default_value="agri_sim"),
            DeclareLaunchArgument("resource_path", default_value=""),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.40"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_control", default_value="true"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("camera_config", default_value=""),
            DeclareLaunchArgument("lidar_mode", default_value="gpu_lidar"),
            DeclareLaunchArgument("lidar_config", default_value=""),
            DeclareLaunchArgument("rgl_install_prefix", default_value=""),
            DeclareLaunchArgument("rgl_patterns_dir", default_value=""),
            DeclareLaunchArgument(
                "use_kinematics",
                default_value=LaunchConfiguration("use_control"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(simulation)),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "headless_rendering": LaunchConfiguration("headless_rendering"),
                    "paused": LaunchConfiguration("paused"),
                    "world_name": LaunchConfiguration("world_name"),
                    "gz_partition": LaunchConfiguration("gz_partition"),
                    "resource_path": LaunchConfiguration("resource_path"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                    "use_control": LaunchConfiguration("use_control"),
                    "use_lidar": LaunchConfiguration("use_lidar"),
                    "use_camera": LaunchConfiguration("use_camera"),
                    "camera_config": LaunchConfiguration("camera_config"),
                    "lidar_mode": LaunchConfiguration("lidar_mode"),
                    "lidar_config": LaunchConfiguration("lidar_config"),
                    "rgl_install_prefix": LaunchConfiguration("rgl_install_prefix"),
                    "rgl_patterns_dir": LaunchConfiguration("rgl_patterns_dir"),
                    "use_kinematics": LaunchConfiguration("use_kinematics"),
                }.items(),
            )
        ]
    )
