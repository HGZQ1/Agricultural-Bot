"""MoveIt for an already running Gazebo robot.

This launch owns no hardware, controllers, joint-state publisher or TF publisher.
The canonical description is expanded with independent finger joints, matching
the Gazebo robot. Use mock_arm.launch.py for a separate standalone test backend.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context):
    description_share = get_package_share_directory("agri_robot_description")
    config_share = get_package_share_directory("agri_arm_moveit_config")
    model = os.path.join(description_share, "urdf", "agri_robot.urdf.xacro")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    config = (
        MoveItConfigsBuilder("agri_robot", package_name="agri_arm_moveit_config")
        .robot_description(file_path=model, mappings={"use_ros2_control": "true"})
        .robot_description_semantic(file_path="config/agri_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=False,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    common = {"use_sim_time": use_sim_time}
    return [
        Node(
            package="moveit_ros_move_group", executable="move_group", output="screen",
            parameters=[config.to_dict(), common, {
                "trajectory_execution.allowed_execution_duration_scaling": 1.5,
                "trajectory_execution.allowed_goal_duration_margin": 1.0,
                "trajectory_execution.allowed_start_tolerance": 0.01,
            }],
            remappings=[("joint_states", LaunchConfiguration("joint_states_topic"))],
        ),
        Node(
            package="rviz2", executable="rviz2", output="log",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", os.path.join(config_share, "config", "moveit.rviz")],
            parameters=[config.robot_description, config.robot_description_semantic,
                        config.robot_description_kinematics, config.planning_pipelines,
                        config.joint_limits, common],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        OpaqueFunction(function=_launch_setup),
    ])
