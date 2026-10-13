"""Launch sampled RGB-D tomato observations with capture-time quality checks."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'use_sim_time': True,
        'model_path': os.environ.get('AGRI_TOMATO_MODEL', ''),
        'image_topic': '/d405/color/image_raw',
        'depth_topic': '/d405/aligned_depth_to_color/image_raw',
        'camera_info_topic': '/d405/color/camera_info',
        'annotated_image_topic': '/agri_vision/annotated_image',
        'detections_topic': '/agri_vision/detections',
        'positions_topic': '/agri_vision/tomato_positions_camera',
        'base_positions_topic': '/agri_vision/tomato_positions_base',
        'targets_topic': '/agri_vision/tomato_targets_camera',
        'base_targets_topic': '/agri_vision/tomato_targets_base',
        'target_frame': 'base_link', 'confidence_threshold': 0.25,
        'image_size': 640, 'mature_only': True, 'mature_class': 'tomato_R',
        'depth_sample_count': 81, 'depth_sample_radius_px': 8.0,
        'depth_bbox_radius_fraction': 0.25, 'depth_sampling_seed': 42,
        'depth_min_m': 0.07, 'depth_max_m': 2.0,
        'depth_min_valid_samples': 15, 'depth_min_valid_fraction': 0.6,
        'depth_max_mad_m': 0.03, 'require_segmentation': False,
        'segmentation_erode_px': 1,
        'max_rgb_depth_skew_s': 0.05, 'max_observation_age_s': 0.5,
        'image_pair_queue_size': 10, 'stream_timeout_wall_s': 1.0,
        'stability_min_frames': 3, 'stability_window_s': 0.5,
        'stability_min_span_s': 0.1, 'stability_max_spread_m': 0.025,
        'track_match_distance_m': 0.08,
    }
    arguments, parameters = [], {}
    for name, value in defaults.items():
        default = str(value).lower() if isinstance(value, bool) else str(value)
        arguments.append(DeclareLaunchArgument(name, default_value=default))
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=type(value))
    # Preserve the existing numeric GPU or string 'cpu' device convention.
    arguments.append(DeclareLaunchArgument('device', default_value='0'))
    parameters['device'] = LaunchConfiguration('device')
    return LaunchDescription(arguments + [Node(
        package='agri_perception', executable='tomato_detector', name='tomato_detector',
        output='screen', parameters=[parameters],
    )])
