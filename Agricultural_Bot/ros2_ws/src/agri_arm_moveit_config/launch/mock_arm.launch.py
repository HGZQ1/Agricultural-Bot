"""Standalone mock only; run in a separate ROS domain from Gazebo/hardware."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context):
    config_share = get_package_share_directory("agri_arm_moveit_config")
    model = os.path.join(config_share, "urdf", "mock_robot.urdf.xacro")
    config = (
        MoveItConfigsBuilder("agri_robot", package_name="agri_arm_moveit_config")
        .robot_description(file_path=model, mappings={"use_ros2_control": "true"})
        .robot_description_semantic(file_path="config/agri_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    control_config = os.path.join(config_share, "config", "ros2_controllers.yaml")
    # All joints are supplied by the mock system. No joint_state_publisher
    # merges placeholder wheels into real state.
    return [
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[control_config, config.robot_description, {"use_sim_time": False}],
             remappings=[("~/robot_description", "/robot_description")]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[config.robot_description, {"use_sim_time": False}]),
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller",
                        "--controller-manager", "/controller_manager", "--activate-as-group"]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[config.to_dict(), {"use_sim_time": False}]),
        Node(package="rviz2", executable="rviz2", output="log",
             arguments=["-d", os.path.join(config_share, "config", "moveit.rviz")],
             condition=IfCondition(LaunchConfiguration("use_rviz")),
             parameters=[config.to_dict(), {"use_sim_time": False}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
