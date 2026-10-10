"""Launch the YOLO tomato detector on the simulated D405 camera."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=os.environ.get('AGRI_TOMATO_MODEL', ''),
            description='Absolute path to the trained Ultralytics best.pt',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/d405/color/image_raw',
        ),
        DeclareLaunchArgument(
            'annotated_image_topic',
            default_value='/agri_vision/annotated_image',
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/agri_vision/detections',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/d405/aligned_depth_to_color/image_raw',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/d405/color/camera_info',
        ),
        DeclareLaunchArgument(
            'positions_topic',
            default_value='/agri_vision/tomato_positions_camera',
        ),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'base_positions_topic',
            default_value='/agri_vision/tomato_positions_base',
        ),
        DeclareLaunchArgument('confidence_threshold', default_value='0.25'),
        DeclareLaunchArgument('image_size', default_value='640'),
        DeclareLaunchArgument('device', default_value='0'),
        Node(
            package='agri_perception',
            executable='tomato_detector',
            name='tomato_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'model_path': LaunchConfiguration('model_path'),
                'image_topic': LaunchConfiguration('image_topic'),
                'annotated_image_topic': LaunchConfiguration(
                    'annotated_image_topic'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'positions_topic': LaunchConfiguration('positions_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
                'base_positions_topic': LaunchConfiguration(
                    'base_positions_topic'),
                'confidence_threshold': LaunchConfiguration(
                    'confidence_threshold'),
                'image_size': LaunchConfiguration('image_size'),
                'device': LaunchConfiguration('device'),
            }],
        ),
    ])
