from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro


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


def _make_robot(context, model_xacro, controller_config, *args, **kwargs):
    use_control = LaunchConfiguration("use_control").perform(context).lower()
    control_enabled = use_control in {"1", "true", "yes", "on"}
    robot_description = xacro.process_file(
        str(model_xacro),
        mappings={
            "use_gazebo": "true",
            "use_ros2_control": "true" if control_enabled else "false",
            "use_control": "true" if control_enabled else "false",
            "controller_config": str(controller_config),
        },
    ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
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
            LaunchConfiguration("spawn_z"),
        ],
    )
    return [robot_state_publisher, spawn]


def generate_launch_description():
    sim_share = Path(get_package_share_directory("agri_sim_description"))
    description_share = Path(get_package_share_directory("agri_robot_description"))
    control_share = Path(get_package_share_directory("agri_sim_control"))
    world = sim_share / "worlds" / "empty_greenhouse.sdf"
    model_xacro = sim_share / "urdf" / "agri_robot.gazebo.urdf.xacro"
    controller_config = control_share / "config" / "controllers.yaml"
    resource_root = description_share.parent
    use_control = LaunchConfiguration("use_control")

    resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(resource_root),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("agri_sim_control"))
                / "launch"
                / "control.launch.py"
            )
        ),
        condition=IfCondition(use_control),
    )

    kinematics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("agri_base_kinematics"))
                / "launch"
                / "four_wheel_steering.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(LaunchConfiguration("use_kinematics")),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
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
            DeclareLaunchArgument(
                "use_control",
                default_value="true",
                description=(
                    "Start ros2_control spawners; set false for model-only smoke tests."
                ),
            ),
            DeclareLaunchArgument(
                "use_kinematics",
                default_value=use_control,
                description=(
                    "Start the Python four-wheel-steering command and odometry "
                    "node. Defaults to the value of use_control."
                ),
            ),
            resource_path,
            OpaqueFunction(function=lambda context: _make_gazebo(context, world)),
            clock_bridge,
            OpaqueFunction(
                function=lambda context: _make_robot(
                    context, model_xacro, controller_config
                )
            ),
            control_launch,
            kinematics_launch,
        ]
    )
