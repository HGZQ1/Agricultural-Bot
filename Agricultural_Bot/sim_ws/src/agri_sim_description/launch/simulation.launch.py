from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _make_gazebo(context, world, *args, **kwargs):
    gui = LaunchConfiguration("gui").perform(context).lower()
    paused = LaunchConfiguration("paused").perform(context).lower()
    run_flag = "" if paused in {"1", "true", "yes", "on"} else " -r"
    server_only = " -s" if gui in {"0", "false", "no", "off"} else ""
    gz_share = Path(get_package_share_directory("ros_gz_sim"))
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_share / "launch" / "gz_sim.launch.py")),
        # ros_gz_sim uses a shell command internally; quote paths because this
        # workspace may live under a directory containing spaces.
        launch_arguments={"gz_args": f'{run_flag}{server_only} "{world}"'}.items(),
    )
    return [gz_launch]


def generate_launch_description():
    sim_share = Path(get_package_share_directory("agri_sim_description"))
    description_share = Path(
        get_package_share_directory("agri_robot_description")
    )
    world = sim_share / "worlds" / "empty_greenhouse.sdf"
    urdf = description_share / "urdf" / "robot_pick_robot11.urdf"
    resource_root = description_share.parent
    spawn_z = LaunchConfiguration("spawn_z")

    resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(resource_root),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": urdf.read_text(),
                "use_sim_time": True,
            }
        ],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_agri_robot",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "agri_robot",
            "-z",
            spawn_z,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start Gazebo with a GUI; set false for headless tests.",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.40",
                description="Initial model height above the Gazebo ground plane (m).",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Start Gazebo paused so the initial model pose can be inspected.",
            ),
            resource_path,
            OpaqueFunction(function=lambda context: _make_gazebo(context, world)),
            robot_state_publisher,
            spawn,
        ]
    )
