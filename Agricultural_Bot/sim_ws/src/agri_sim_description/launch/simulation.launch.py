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
from launch.substitutions import EnvironmentVariable
import xacro


def _make_gazebo(context, *args, **kwargs):
    gui = LaunchConfiguration("gui").perform(context).lower()
    paused = LaunchConfiguration("paused").perform(context).lower()
    world = LaunchConfiguration("world").perform(context)
    run_flag = "" if paused in {"1", "true", "yes", "on"} else " -r"
    headless = gui in {"0", "false", "no", "off"}
    server_only = " -s" if headless else ""
    headless_rendering = " --headless-rendering" if headless else ""
    gz_share = Path(get_package_share_directory("ros_gz_sim"))
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_share / "launch" / "gz_sim.launch.py")),
        # ros_gz_sim uses a shell command internally; quote paths because this
        # workspace may live under a directory containing spaces.
        launch_arguments={
            "gz_args": f'{run_flag}{server_only}{headless_rendering} "{world}"'
        }.items(),
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
            "-world",
            LaunchConfiguration("world_name"),
            "-topic",
            "robot_description",
            "-name",
            "agri_robot",
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-z",
            LaunchConfiguration("spawn_z"),
            "-Y",
            LaunchConfiguration("spawn_yaw"),
        ],
    )
    return [robot_state_publisher, spawn]


def generate_launch_description():
    sim_share = Path(get_package_share_directory("agri_sim_description"))
    description_share = Path(get_package_share_directory("agri_robot_description"))
    control_share = Path(get_package_share_directory("agri_sim_control"))
    default_world = sim_share / "worlds" / "empty_greenhouse.sdf"
    model_xacro = sim_share / "urdf" / "agri_robot.gazebo.urdf.xacro"
    controller_config = control_share / "config" / "controllers.yaml"
    resource_root = description_share.parent
    use_control = LaunchConfiguration("use_control")

    resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            LaunchConfiguration("resource_path"),
            ":",
            str(resource_root),
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    # Gazebo Transport discovers worlds through a partition.  Keeping this
    # launch in its own partition prevents an older Gazebo server (for example
    # the upstream camera demo) from answering this launch's spawn request.
    transport_partition = SetEnvironmentVariable(
        name="GZ_PARTITION",
        value=LaunchConfiguration("gz_partition"),
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
                "world",
                default_value=str(default_world),
                description="Absolute path to the SDF world file.",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="empty_greenhouse",
                description="Name of the <world> element used by the spawn service.",
            ),
            DeclareLaunchArgument(
                "resource_path",
                default_value="",
                description="Additional Gazebo model-resource directory.",
            ),
            DeclareLaunchArgument(
                "gz_partition",
                default_value="agri_sim",
                description=(
                    "Gazebo Transport partition used to isolate this simulation "
                    "from other Gazebo sessions."
                ),
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="0.0",
                description="Initial robot X position in the world (m).",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="0.0",
                description="Initial robot Y position in the world (m).",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.40",
                description="Initial model height above the Gazebo ground plane (m).",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0.0",
                description="Initial robot yaw about world Z (rad).",
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
            transport_partition,
            OpaqueFunction(function=_make_gazebo),
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
