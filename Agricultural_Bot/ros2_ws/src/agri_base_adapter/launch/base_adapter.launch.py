from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('agri_base_adapter'), 'config', 'base_adapter.yaml'])
    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('require_harvest_interlock', default_value='false'),
        DeclareLaunchArgument('harvest_worker_state_topic', default_value='/harvest_worker/state'),
        DeclareLaunchArgument('harvest_worker_timeout_s', default_value='0.75'),
        DeclareLaunchArgument('harvest_worker_max_age_s', default_value='0.5'),
        DeclareLaunchArgument('harvest_worker_future_skew_s', default_value='0.05'),
        Node(
            package='agri_base_adapter',
            executable='velocity_gate',
            name='base_velocity_gate',
            output='screen',
            parameters=[config, {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'require_harvest_interlock': ParameterValue(
                    LaunchConfiguration('require_harvest_interlock'), value_type=bool),
                'harvest_worker_state_topic': LaunchConfiguration('harvest_worker_state_topic'),
                **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                   for name in ('harvest_worker_timeout_s', 'harvest_worker_max_age_s',
                                'harvest_worker_future_skew_s')},
            }],
        ),
        Node(
            package='agri_base_adapter',
            executable='odom_adapter',
            name='base_odom_adapter',
            output='screen',
            parameters=[config, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        ),
    ])
