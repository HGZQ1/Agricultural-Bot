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
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("spawn_z", default_value="0.40"),
            DeclareLaunchArgument("use_control", default_value="true"),
            DeclareLaunchArgument(
                "use_kinematics",
                default_value=LaunchConfiguration("use_control"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(simulation)),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "paused": LaunchConfiguration("paused"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "use_control": LaunchConfiguration("use_control"),
                    "use_kinematics": LaunchConfiguration("use_kinematics"),
                }.items(),
            )
        ]
    )
