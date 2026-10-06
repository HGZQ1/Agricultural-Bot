#!/usr/bin/env python3
"""Place visual-only RGB-D calibration targets in d405_validation only.

Gazebo poses are used solely to construct and evaluate this test fixture.
This script does not publish truth to robot algorithms or command the robot.
Targets follow the camera forward axis, with up independently set by world +Z.
"""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


WORLD = 'd405_validation'
MODEL = 'd405_calibration_targets'
ROBOT = 'agri_robot'
DEFAULT_DIRECTORY = Path(__file__).resolve().parents[1] / 'artifacts/d405_scene_validation'
FIXTURES = [
    {'name': 'red_sphere', 'shape': 'sphere', 'center': [-0.055, -0.025, 0.30],
     'radius': 0.025, 'rgb': [1, 0, 0]},
    {'name': 'green_sphere', 'shape': 'sphere', 'center': [0.07, 0.04, 0.40],
     'radius': 0.025, 'rgb': [0, 1, 0]},
    {'name': 'yellow_near', 'shape': 'box', 'center': [0.025, 0.018, 0.04],
     'size': [0.010, 0.008, 0.010], 'rgb': [1, 1, 0]},
    {'name': 'blue_far', 'shape': 'box', 'center': [-0.26, 0.05, 0.70],
     'size': [0.16, 0.10, 0.01], 'rgb': [0, 0, 1]},
    {'name': 'grey_background', 'shape': 'box', 'center': [0, 0, 1.0],
     'size': [2.4, 1.5, 0.01], 'rgb': [0.25, 0.25, 0.25]},
]


def build_fixtures(max_depth=0.5):
    """Keep the baseline targets and scale far references to the chosen range."""
    if isinstance(max_depth, bool) or not isinstance(max_depth, (int, float)) \
            or not math.isfinite(max_depth) or max_depth < 0.5:
        raise ValueError('Fixture max-depth must be finite and at least 0.5 m')
    fixtures = deepcopy(FIXTURES)
    far_depth = max_depth + max(0.2, 0.1 * max_depth)
    scale = far_depth / 0.70
    far = next(fixture for fixture in fixtures if fixture['name'] == 'blue_far')
    far['center'] = [-0.26 * scale, 0.05 * scale, far_depth]
    far['size'] = [0.16 * scale, 0.10 * scale, 0.01]
    background = next(fixture for fixture in fixtures if fixture['name'] == 'grey_background')
    background_depth = max(1.0, far_depth + 0.30)
    background['center'][2] = background_depth
    background['size'] = [2.4 * background_depth, 1.5 * background_depth, 0.01]
    if max_depth >= 1.0:
        # Constant angular size gives >500 interior pixels even at 4 m.
        # Its view direction avoids the original near targets and gripper.
        distance = 0.8 * max_depth
        fixtures.append({
            'name': 'magenta_sphere', 'shape': 'sphere',
            'center': [0.225 * distance, -0.175 * distance, distance],
            'radius': 0.04 * distance, 'rgb': [1, 0, 1],
        })
    if not all(math.isfinite(value) for fixture in fixtures
               for value in [*fixture['center'], *fixture.get('size', []),
                             fixture.get('radius', 0)]):
        raise ValueError('Fixture max-depth is too large to produce finite geometry')
    return fixtures


def rpy_matrix(roll, pitch, yaw):
    sr, cr, sp, cp, sy, cy = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch), math.sin(yaw), math.cos(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def matrix_rpy(matrix):
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1, 1)))
    if abs(math.cos(pitch)) > 1e-8:
        return [math.atan2(matrix[2, 1], matrix[2, 2]), pitch,
                math.atan2(matrix[1, 0], matrix[0, 0])]
    return [0.0, pitch, math.atan2(-matrix[0, 1], matrix[1, 1])]


def quaternion_matrix(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        raise ValueError('TF quaternion contains non-finite values')
    if abs(math.sqrt(x*x+y*y+z*z+w*w) - 1) > 1e-3:
        raise ValueError('TF quaternion is not normalized')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform_pose(transform):
    position = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    if not np.isfinite(position).all():
        raise ValueError('TF translation contains non-finite values')
    return position, quaternion_matrix(transform.rotation)


def world_up_fixture_rotation(forward):
    """Place asymmetric targets upright independently of the camera's roll."""
    forward = np.asarray(forward, dtype=float)
    if forward.shape != (3,) or not np.isfinite(forward).all() or np.linalg.norm(forward) < 1e-8:
        raise ValueError('Fixture forward must be a finite nonzero 3-vector')
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    if np.linalg.norm(right) < 1e-6:
        raise ValueError('World-up fixture requires a camera axis away from vertical')
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return np.column_stack((right, down, forward))


def model_pose(name, required=True):
    result = subprocess.run(['gz', 'model', '-m', name, '-p'],
                            capture_output=True, text=True, timeout=15)
    if f'world [{WORLD}]' not in result.stdout:
        if not required and 'world [' not in result.stdout:
            return None
        raise RuntimeError(f'Only world [{WORLD}] is authorized for this fixture: {result.stdout} {result.stderr}')
    vectors = re.findall(r'^\s*\[([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)\]',
                         result.stdout, re.MULTILINE)
    if len(vectors) != 2:
        if not required:
            return None
        raise RuntimeError('Cannot read Gazebo model pose: ' + result.stdout)
    position, angles = (np.array(values, dtype=float) for values in vectors)
    if not np.isfinite(position).all() or not np.isfinite(angles).all():
        raise RuntimeError('Gazebo model pose is not finite')
    return position, rpy_matrix(*angles)


def service(path, reqtype, request):
    result = subprocess.run([
        'gz', 'service', '--service', path, '--reqtype', reqtype,
        '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', request,
    ], capture_output=True, text=True, timeout=15)
    if result.returncode or not re.search(r'\bdata:\s*true\b', result.stdout):
        raise RuntimeError(f'Gazebo service failed: {path}: {result.stdout} {result.stderr}')


def write_fixture(path, position, orientation, fixtures=None):
    root = ET.Element('sdf', {'version': '1.10'})
    model = ET.SubElement(root, 'model', {'name': MODEL})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = ' '.join(map(str, [*position, *matrix_rpy(orientation)]))
    link = ET.SubElement(model, 'link', {'name': 'targets'})
    for fixture in FIXTURES if fixtures is None else fixtures:
        visual = ET.SubElement(link, 'visual', {'name': fixture['name']})
        ET.SubElement(visual, 'pose').text = ' '.join(map(str, [*fixture['center'], 0, 0, 0]))
        ET.SubElement(visual, 'cast_shadows').text = 'false'
        geometry = ET.SubElement(visual, 'geometry')
        shape = ET.SubElement(geometry, fixture['shape'])
        if fixture['shape'] == 'sphere':
            ET.SubElement(shape, 'radius').text = str(fixture['radius'])
        else:
            ET.SubElement(shape, 'size').text = ' '.join(map(str, fixture['size']))
        material = ET.SubElement(visual, 'material')
        rgba = ' '.join(map(str, [*fixture['rgb'], 1]))
        for tag in ('ambient', 'diffuse', 'emissive'):
            ET.SubElement(material, tag).text = rgba
        ET.SubElement(material, 'specular').text = '0 0 0 1'
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_DIRECTORY)
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--stable-sim-seconds', type=float, default=0.5)
    parser.add_argument('--max-depth', type=float, default=0.5,
                        help='Sensor depth upper limit in metres (default: 0.5)')
    options = parser.parse_args()
    if not math.isfinite(options.timeout) or not math.isfinite(options.stable_sim_seconds) \
            or options.timeout <= 0 or options.stable_sim_seconds < 0.5:
        parser.error('finite timeout must be positive and stable-sim-seconds at least 0.5')
    try:
        fixtures = build_fixtures(options.max_depth)
    except ValueError as exc:
        parser.error(str(exc))
    model_pose(ROBOT)  # Guard the world before any mutation.
    rclpy.init(args=[])
    node = Node('setup_d405_validation_scene', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node, spin_thread=False)
    started = time.monotonic()
    stable_since = None
    stable_position = stable_rotation = None
    last_query = 0.0
    world_position = world_rotation = base_position = base_rotation = None
    try:
        while time.monotonic() - started < options.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - last_query < 0.25:
                continue
            try:
                transform = buffer.lookup_transform('base_footprint', 'camera_optical_frame', Time()).transform
            except TransformException:
                continue
            base_position, base_rotation = transform_pose(transform)
            robot_position, robot_rotation = model_pose(ROBOT)
            last_query = time.monotonic()
            # Flush clock updates accumulated during the read-only Gazebo query.
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0)
            sim_time = node.get_clock().now().nanoseconds / 1e9
            if sim_time <= 0:
                continue
            world_position = robot_position + robot_rotation @ base_position
            world_rotation = robot_rotation @ base_rotation
            movement = float('inf') if stable_position is None else np.linalg.norm(world_position - stable_position)
            rotation_movement = float('inf') if stable_rotation is None else np.linalg.norm(world_rotation - stable_rotation)
            if movement >= 0.001 or rotation_movement >= 0.003:
                stable_since = sim_time
                stable_position, stable_rotation = world_position.copy(), world_rotation.copy()
            elif sim_time - stable_since >= options.stable_sim_seconds:
                break
        else:
            raise RuntimeError('Camera world pose did not remain stable for 0.5 simulation seconds')
    finally:
        assert listener is not None
        node.destroy_node()
        rclpy.shutdown()
    directory = options.output_dir.expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    fixture_file = directory / (MODEL + '.sdf')
    # The old fixture copied optical X/Y and could rotate along with an
    # inverted camera. Only its forward follows the camera; up is world +Z.
    fixture_rotation = world_up_fixture_rotation(world_rotation[:, 2])
    write_fixture(fixture_file, world_position, fixture_rotation, fixtures)
    if model_pose(MODEL, required=False) is not None:
        service(f'/world/{WORLD}/remove', 'gz.msgs.Entity', f'name: "{MODEL}" type: MODEL')
    service(f'/world/{WORLD}/create', 'gz.msgs.EntityFactory',
            'sdf_filename: ' + json.dumps(str(fixture_file)) + ' allow_renaming: false')
    metadata = {
        'world': WORLD, 'fixture_model': MODEL, 'robot_model': ROBOT,
        'optical_frame': 'camera_optical_frame', 'base_frame': 'base_footprint',
        'fixture_world_xyz_m': world_position.tolist(),
        'fixture_world_rpy_rad': matrix_rpy(fixture_rotation),
        'fixture_world_rotation': fixture_rotation.tolist(),
        'fixture_orientation_reference': 'world_up',
        'optical_xyz_in_base_m': base_position.tolist(),
        'optical_rotation_in_base': base_rotation.tolist(),
        'stable_sim_seconds': options.stable_sim_seconds, 'fixtures': fixtures,
        'depth_contract_m': [0.07, options.max_depth],
        'sdf_file': str(fixture_file), 'truth_scope': 'simulation acceptance only',
    }
    metadata_file = directory / 'scene.json'
    metadata_file.write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps({'created': True, 'world': WORLD, 'model': MODEL,
                      'fixture_count': len(fixtures), 'scene': str(metadata_file),
                      'depth_contract_m': metadata['depth_contract_m'],
                      'world_xyz_m': world_position.tolist()}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
