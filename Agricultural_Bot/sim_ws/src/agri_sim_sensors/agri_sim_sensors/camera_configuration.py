"""Validate the D405 contract and configure Gazebo's aligned RGB-D camera."""

from __future__ import annotations

import math
from numbers import Real
from pathlib import Path
import re

import yaml


_TOPIC = re.compile(r'/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*')
_REQUIRED = (
    'sensor_frame', 'optical_frame', 'gz_topic', 'color_topic', 'depth_topic',
    'info_topic', 'width', 'height', 'update_rate', 'hfov_deg', 'vfov_deg',
    'color_near', 'color_far', 'depth_near', 'depth_far',
)


def load_camera_config(path: Path) -> dict:
    """Read and validate a configuration before spawning the camera sensor."""
    with path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not isinstance(document.get('d405'), dict):
        raise ValueError('D405 configuration must contain a d405 mapping')
    config = document['d405']
    config.setdefault('pixel_center_offset', 0.5)
    for key in _REQUIRED:
        if key not in config:
            raise ValueError(f'D405 configuration is missing {key}')
    if config['sensor_frame'] != 'd405_sensor_frame':
        raise ValueError('D405 sensor_frame must match the canonical d405_sensor_frame link')
    if config['optical_frame'] != 'camera_optical_frame':
        raise ValueError('D405 optical_frame must match the canonical camera_optical_frame link')
    for key in ('width', 'height'):
        if isinstance(config[key], bool) or not isinstance(config[key], int) or config[key] < 2:
            raise ValueError(f'D405 {key} must be an integer >= 2')
    for key in (
        'update_rate', 'hfov_deg', 'vfov_deg', 'color_near', 'color_far',
        'depth_near', 'depth_far',
        'pixel_center_offset',
    ):
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
            raise ValueError(f'D405 {key} must be a finite number')
    if not 0 <= config['pixel_center_offset'] <= 1:
        raise ValueError('D405 pixel_center_offset must lie between 0 and 1 pixels')
    if config['update_rate'] <= 0:
        raise ValueError('D405 update_rate must be positive')
    for key in ('hfov_deg', 'vfov_deg'):
        if not 0 < config[key] < 180:
            raise ValueError(f'D405 {key} must lie strictly between 0 and 180 degrees')
    if not (
        0 < config['color_near'] <= config['depth_near']
        < config['depth_far'] <= config['color_far']
    ):
        raise ValueError(
            'D405 requires 0 < color_near <= depth_near < depth_far <= color_far'
        )
    topics = [config[key] for key in ('gz_topic', 'color_topic', 'depth_topic', 'info_topic')]
    if any(not isinstance(topic, str) or not _TOPIC.fullmatch(topic) for topic in topics):
        raise ValueError('D405 topics must be absolute names with valid, nonempty tokens')
    if len(set(topics)) != len(topics):
        raise ValueError('D405 gz_topic, color_topic, depth_topic, and info_topic must differ')
    return config


def camera_xacro_mappings(config: dict) -> dict[str, str]:
    """Use independent focal lengths to preserve both nominal fields of view."""
    names = {
        'd405_frame': 'sensor_frame', 'd405_optical_frame': 'optical_frame',
        'd405_topic': 'gz_topic', 'd405_width': 'width', 'd405_height': 'height',
        'd405_rate': 'update_rate', 'd405_color_near': 'color_near',
        'd405_color_far': 'color_far', 'd405_depth_near': 'depth_near',
        'd405_depth_far': 'depth_far',
    }
    mappings = {name: str(config[key]) for name, key in names.items()}
    hfov = math.radians(config['hfov_deg'])
    vfov = math.radians(config['vfov_deg'])
    mappings.update({
        'd405_hfov': str(hfov),
        'd405_fx': str(config['width'] / (2 * math.tan(hfov / 2))),
        'd405_fy': str(config['height'] / (2 * math.tan(vfov / 2))),
        'd405_cx': str(config['width'] / 2),
        'd405_cy': str(config['height'] / 2),
    })
    return mappings


def write_camera_bridge(config: dict, destination: Path, include_images: bool = True,
                        info_topic_override: str | None = None) -> None:
    """Bridge only RGB, aligned depth, and their shared camera calibration."""
    topics = (
        ('color_topic', 'image', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
        ('depth_topic', 'depth_image', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
        ('info_topic', 'camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
    )
    bridge = [{
        'ros_topic_name': info_topic_override if key == 'info_topic' and info_topic_override else config[key],
        'gz_topic_name': config['gz_topic'] + '/' + suffix,
        'ros_type_name': ros_type,
        'gz_type_name': gz_type,
        'direction': 'GZ_TO_ROS',
        'qos_profile': 'SENSOR_DATA',
        'frame_id': config['optical_frame'],
        'lazy': False,
    } for key, suffix, ros_type, gz_type in topics if include_images or key == 'info_topic']
    destination.write_text(yaml.safe_dump(bridge, sort_keys=False), encoding='utf-8')


def write_camera_rviz(config: dict, template: Path, destination: Path) -> None:
    """Match the image topics and float-depth display range to the sensor."""
    document = yaml.safe_load(template.read_text(encoding='utf-8'))
    displays = {display['Name']: display
                for display in document['Visualization Manager']['Displays']}
    color, depth = displays['D405 Color'], displays['D405 Depth']
    color['Topic']['Value'] = config['color_topic']
    depth['Topic']['Value'] = config['depth_topic']
    depth['Normalize Range'] = False
    depth['Min Value'] = config['depth_near']
    depth['Max Value'] = config['depth_far']
    destination.write_text(yaml.safe_dump(document, sort_keys=False), encoding='utf-8')


def image_bridge_environment(profile: Path, environment: dict) -> dict[str, str]:
    """Give Fast DDS enough SHM for images, respecting the user's DDS profiles."""
    rmw = environment.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    if rmw not in ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp'):
        return {}
    if any(environment.get(key) for key in (
        'FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE',
    )):
        return {}
    # Jazzy ships Fast DDS 2.x, whose loader uses this legacy variable name.
    return {'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile)}
