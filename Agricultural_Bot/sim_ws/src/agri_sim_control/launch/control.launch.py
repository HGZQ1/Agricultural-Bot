"""Spawn the ros2_control controllers provided by gz_ros2_control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _spawner(name, config_file):
    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawn_{name}",
        output="screen",
        arguments=[
            name,
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            config_file,
        ],
    )


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("agri_sim_control"), "config", "controllers.yaml"]
    )
    delay = LaunchConfiguration("controller_start_delay")

    # The Gazebo plugin creates /controller_manager. Delay spawners briefly so
    # the manager has time to load after the entity is inserted into Gazebo.
    controllers = [
        _spawner("joint_state_broadcaster", config_file),
        _spawner("steering_controller", config_file),
        _spawner("wheel_controller", config_file),
        _spawner("arm_controller", config_file),
        _spawner("gripper_controller", config_file),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_start_delay",
                default_value="3.0",
                description="Seconds to wait for gz_ros2_control's controller manager.",
            ),
            TimerAction(period=delay, actions=controllers),
        ]
    )
