from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = Path(get_package_share_directory("agri_sim_description"))
    worlds_share = Path(get_package_share_directory("agri_greenhouse_worlds"))
    simulation = sim_share / "launch" / "simulation.launch.py"
    world = worlds_share / "worlds" / "tomato_field_22x14.sdf"

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("paused", default_value="true"),
            DeclareLaunchArgument(
                "gz_partition",
                default_value="agricultural_bot_tomato_field",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="-6.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.40"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.5708"),
            DeclareLaunchArgument("use_control", default_value="false"),
            DeclareLaunchArgument(
                "use_kinematics",
                default_value=LaunchConfiguration("use_control"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(simulation)),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "paused": LaunchConfiguration("paused"),
                    "world": str(world),
                    "world_name": "field",
                    "resource_path": str(worlds_share / "models"),
                    "gz_partition": LaunchConfiguration("gz_partition"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                    "use_control": LaunchConfiguration("use_control"),
                    "use_kinematics": LaunchConfiguration("use_kinematics"),
                }.items(),
            ),
        ]
    )
