from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context, *args, **kwargs):
    model_path = Path(LaunchConfiguration("model").perform(context))
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = {"robot_description": model_path.read_text()}
    package_share = Path(get_package_share_directory("agri_robot_description"))
    rviz_config = package_share / "rviz" / "robot_description.rviz"
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(rviz_config)],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("agri_robot_description")
    )
    default_model = package_share / "urdf" / "robot_pick_robot11.urdf"

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=str(default_model),
        description="Path to the robot URDF file",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz2; set false for headless description checks.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use ROS simulation time; keep false for standalone RViz.",
    )

    return LaunchDescription(
        [
            model_arg,
            rviz_arg,
            use_sim_time_arg,
            OpaqueFunction(function=_make_nodes),
        ]
    )
