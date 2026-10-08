from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('agri_base_adapter'), 'config', 'base_adapter.yaml'])
    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='agri_base_adapter',
            executable='velocity_gate',
            name='base_velocity_gate',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='agri_base_adapter',
            executable='odom_adapter',
            name='base_odom_adapter',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
    ])
