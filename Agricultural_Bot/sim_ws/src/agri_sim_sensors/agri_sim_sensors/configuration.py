"""Validate the sensor contract and prepare an isolated Gazebo world."""

from __future__ import annotations

import ctypes
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


def load_config(path: Path) -> dict:
    with path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)['mid360']
    required = (
        'frame_id', 'gz_topic', 'points_topic', 'update_rate', 'min_range',
        'max_range', 'horizontal_samples', 'vertical_samples',
        'horizontal_min_angle', 'horizontal_max_angle',
        'vertical_min_angle', 'vertical_max_angle', 'range_resolution',
        'noise_stddev', 'visualize', 'render_engine', 'rgl',
    )
    for key in required:
        if key not in config:
            raise ValueError(f'MID-360 configuration is missing {key}')
    if config['frame_id'] != 'mid360_sensor_frame':
        raise ValueError('MID-360 frame_id must match the canonical mid360_sensor_frame link')
    if config['render_engine'] != 'ogre2':
        raise ValueError('MID-360 requires the ogre2 render engine')
    for key in required[3:14]:
        if not math.isfinite(float(config[key])):
            raise ValueError(f'MID-360 {key} must be finite')
    if not 0 < config['min_range'] < config['max_range']:
        raise ValueError('MID-360 requires 0 < min_range < max_range')
    if config['update_rate'] <= 0 or config['range_resolution'] <= 0:
        raise ValueError('MID-360 rate and range resolution must be positive')
    for key in ('horizontal_samples', 'vertical_samples'):
        value = config[key]
        if isinstance(value, bool) or int(value) != value or value < 2:
            raise ValueError(f'MID-360 {key} must be an integer >= 2')
    for axis in ('horizontal', 'vertical'):
        if config[f'{axis}_min_angle'] >= config[f'{axis}_max_angle']:
            raise ValueError(f'MID-360 {axis} angle bounds are reversed')
    if config['noise_stddev'] < 0:
        raise ValueError('MID-360 noise cannot be negative')
    if config['points_topic'] != config['gz_topic'].rstrip('/') + '/points':
        raise ValueError('points_topic must be gz_topic + /points for both backends')
    if not isinstance(config['visualize'], bool):
        raise ValueError('MID-360 visualize must be a YAML boolean')
    if config['rgl']['pattern_preset'] != 'Livox Mid360':
        raise ValueError('Use the official Livox Mid360 named RGL preset')
    return config


def find_workspace(package_share: Path) -> Path:
    for parent in (package_share, *package_share.parents):
        if (parent / 'ros2_ws').is_dir() and (parent / 'sim_ws').is_dir():
            return parent
    return Path.cwd()


def resolve_backend(mode: str, install_prefix: Path, patterns_dir: Path) -> tuple[str, str]:
    if mode not in ('gpu_lidar', 'rgl', 'auto'):
        raise ValueError('lidar_mode must be gpu_lidar, rgl, or auto')
    if mode == 'gpu_lidar':
        return mode, 'MID-360: Gazebo gpu_lidar functional-equivalent scan'
    plugin_dir = install_prefix / 'RGLServerPlugin'
    required = [
        plugin_dir / 'libRGLServerPluginManager.so',
        plugin_dir / 'libRGLServerPluginInstance.so',
        plugin_dir / 'libRobotecGPULidar.so',
        patterns_dir / 'LivoxMid360.mat3x4f',
    ]
    reason = ''
    if missing := [str(path) for path in required if not path.is_file()]:
        reason = 'missing RGL files: ' + ', '.join(missing)
    else:
        try:
            gpu = subprocess.run(
                ['nvidia-smi', '-L'], capture_output=True, text=True,
                timeout=10, check=False,
            )
            if gpu.returncode != 0 or 'GPU' not in gpu.stdout:
                reason = 'NVIDIA driver/device is unavailable to this process'
            else:
                # Check gz-sim/vendor ABI dependencies as well as the core .so.
                for path in required[:3]:
                    ctypes.CDLL(str(path), mode=os.RTLD_LOCAL)
        except (OSError, subprocess.TimeoutExpired) as error:
            reason = f'RGL dependency check failed: {error}'
    if reason:
        if mode == 'rgl':
            raise RuntimeError(
                f'{reason}. Run python3 scripts/setup_mid360_rgl.py after '
                'sourcing /opt/ros/jazzy/setup.bash, or select lidar_mode:=gpu_lidar.'
            )
        return 'gpu_lidar', f'MID-360 auto falls back to gpu_lidar: {reason}'
    return 'rgl', 'MID-360: RGL official Livox Mid360 preset, 40 scan segments'


def xacro_mappings(config: dict, mode: str) -> dict[str, str]:
    names = {
        'mid360_frame': 'frame_id', 'mid360_topic': 'gz_topic',
        'mid360_rate': 'update_rate', 'mid360_min_range': 'min_range',
        'mid360_max_range': 'max_range', 'mid360_h_samples': 'horizontal_samples',
        'mid360_v_samples': 'vertical_samples',
        'mid360_h_min': 'horizontal_min_angle', 'mid360_h_max': 'horizontal_max_angle',
        'mid360_v_min': 'vertical_min_angle', 'mid360_v_max': 'vertical_max_angle',
        'mid360_resolution': 'range_resolution', 'mid360_noise': 'noise_stddev',
        'mid360_visualize': 'visualize',
    }
    result = {name: str(config[key]).lower() if isinstance(config[key], bool)
              else str(config[key]) for name, key in names.items()}
    result['lidar_mode'] = mode
    result['mid360_preset'] = config['rgl']['pattern_preset']
    return result


def write_bridge(config: dict, destination: Path, mode: str = 'gpu_lidar') -> None:
    bridge = [{
        'ros_topic_name': config['points_topic'] if mode != 'rgl' else
                          config['gz_topic'].rstrip('/') + '/rgl_raw',
        'gz_topic_name': config['points_topic'],
        'ros_type_name': 'sensor_msgs/msg/PointCloud2',
        'gz_type_name': 'gz.msgs.PointCloudPacked',
        'direction': 'GZ_TO_ROS', 'qos_profile': 'SENSOR_DATA',
        'frame_id': config['frame_id'], 'lazy': False,
    }]
    destination.write_text(yaml.safe_dump(bridge, sort_keys=False), encoding='utf-8')


def write_world(source: Path, destination: Path, config: dict, mode: str) -> None:
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError(f'{source} has no SDF world')
    sensors = world.find("plugin[@name='gz::sim::systems::Sensors']")
    if sensors is None:
        sensors = ET.SubElement(world, 'plugin', {
            'filename': 'gz-sim-sensors-system', 'name': 'gz::sim::systems::Sensors',
        })
    engine = sensors.find('render_engine')
    if engine is None:
        engine = ET.SubElement(sensors, 'render_engine')
    engine.text = config['render_engine']
    for plugin in list(world.findall("plugin[@name='rgl::RGLServerPluginManager']")):
        world.remove(plugin)
    if mode == 'rgl':
        manager = ET.SubElement(world, 'plugin', {
            'filename': 'RGLServerPluginManager', 'name': 'rgl::RGLServerPluginManager',
        })
        ET.SubElement(manager, 'do_ignore_entities_in_lidar_link').text = 'true'
    ET.indent(tree, space='  ')
    tree.write(destination, encoding='utf-8', xml_declaration=True)
