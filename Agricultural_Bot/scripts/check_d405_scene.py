#!/usr/bin/env python3
"""Evaluate public RGB-D streams against the d405_validation world-up fixture.

Gazebo truth is used only here for acceptance, never sent to robot algorithms.
Sphere residuals test metric depth and registration; colored out-of-range
targets test that RGB sees objects whose depth is clipped.
At the arm zero pose, --require-upright also checks the independent world-up
and world-left target directions against the image and its optical TF.
"""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from agri_sim_tests.check_d405 import (
    DEPTH_TOLERANCE_M, depth_metres, depth_statistics, image_view, validate_camera_info,
)
from setup_d405_validation_scene import (
    DEFAULT_DIRECTORY, MODEL, ROBOT, WORLD, model_pose, transform_pose,
)


def erode(mask):
    padded = np.pad(mask, 1, constant_values=False)
    return np.logical_and.reduce([
        padded[y:y+mask.shape[0], x:x+mask.shape[1]] for y in range(3) for x in range(3)])


def dilate(mask):
    padded = np.pad(mask, 1, constant_values=False)
    return np.logical_or.reduce([
        padded[y:y+mask.shape[0], x:x+mask.shape[1]] for y in range(3) for x in range(3)])


def color_masks(rgb):
    red, green, blue = (rgb[..., index].astype(np.float32) for index in range(3))
    return {
        'red_sphere': (red > 100) & (green < red*0.35) & (blue < red*0.35),
        'green_sphere': (green > 100) & (red < green*0.35) & (blue < green*0.35),
        'blue_far': (blue > 100) & (red < blue*0.35) & (green < blue*0.35),
        'yellow_near': ((red > 100) & (green > 100) & (blue < np.minimum(red, green)*0.35)
                        & (green > red*0.8) & (red > green*0.8)),
        'magenta_sphere': ((red > 100) & (blue > 100) & (green < np.minimum(red, blue)*0.35)
                           & (blue > red*0.8) & (red > blue*0.8)),
    }


def scene_depth_range(metadata, requested_max_depth=None):
    """Use recorded range, with an explicit override checked rather than assumed."""
    interval = metadata.get('depth_contract_m', [0.07, 0.5])
    if not isinstance(interval, (list, tuple)) or len(interval) != 2 \
            or any(isinstance(value, bool) or not isinstance(value, (int, float))
                   or not math.isfinite(value) for value in interval) \
            or not 0 < interval[0] < interval[1]:
        raise ValueError('Scene depth_contract_m must contain finite 0 < min < max')
    if requested_max_depth is not None:
        if isinstance(requested_max_depth, bool) \
                or not isinstance(requested_max_depth, (int, float)) \
                or not math.isfinite(requested_max_depth) \
                or requested_max_depth <= interval[0]:
            raise ValueError('Requested max-depth must be finite and above min-depth')
        if not math.isclose(requested_max_depth, interval[1], rel_tol=0, abs_tol=1e-9):
            raise ValueError('Requested max-depth does not match the generated scene range')
    return tuple(interval)


def scene_fixtures(metadata, depth_range):
    """Reject duplicate, missing or incorrectly placed metric reference targets."""
    fixtures = metadata['fixtures']
    if not isinstance(fixtures, list) or not all(isinstance(item, dict) for item in fixtures):
        raise ValueError('Scene fixtures must be a list of mappings')
    names = [item.get('name') for item in fixtures]
    if not all(isinstance(name, str) for name in names) or len(set(names)) != len(names):
        raise ValueError('Scene fixture names must be unique strings')
    indexed = {item['name']: item for item in fixtures}
    required = {'red_sphere': 'sphere', 'green_sphere': 'sphere',
                'yellow_near': 'box', 'blue_far': 'box'}
    if depth_range[1] >= 1.0:
        required['magenta_sphere'] = 'sphere'
    for name, shape in required.items():
        if name not in indexed or indexed[name].get('shape') != shape:
            raise ValueError(f'Scene requires {name} with shape {shape}')
        fixture = indexed[name]
        center = fixture.get('center')
        if not isinstance(center, list) or len(center) != 3 \
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in center):
            raise ValueError(f'{name}: center must contain three finite coordinates')
        if shape == 'sphere':
            radius = fixture.get('radius')
            if isinstance(radius, bool) or not isinstance(radius, (int, float)) \
                    or not math.isfinite(radius) or radius <= 0:
                raise ValueError(f'{name}: radius must be finite and positive')
            if not depth_range[0] < center[2] - radius < center[2] + radius < depth_range[1]:
                raise ValueError(f'{name}: entire sphere must be inside the recorded depth range')
            if name == 'magenta_sphere' and center[2] - radius <= 0.5:
                raise ValueError('magenta_sphere must lie beyond the original 0.5 m limit')
        else:
            size = fixture.get('size')
            if not isinstance(size, list) or len(size) != 3 \
                    or any(isinstance(value, bool) or not isinstance(value, (int, float))
                           or not math.isfinite(value) or value <= 0 for value in size):
                raise ValueError(f'{name}: size must contain three finite positive lengths')
            if name == 'yellow_near' and center[2] + size[2]/2 >= depth_range[0]:
                raise ValueError('yellow_near must lie entirely below min-depth')
            if name == 'blue_far' and center[2] - size[2]/2 <= depth_range[1]:
                raise ValueError('blue_far must lie entirely above max-depth')
    return indexed


def boundary_fraction(color, surface):
    color_edge = color & ~erode(color)
    surface_edge = surface & ~erode(surface)
    fractions = []
    for edge, other in ((color_edge, surface_edge), (surface_edge, color_edge)):
        count = int(np.count_nonzero(edge))
        fractions.append(float(np.count_nonzero(edge & dilate(other))) / count if count else 0.0)
    return min(fractions)


def assess_upright(masks, world_optical_rotation, principal):
    """Check zero-pose up/right using world gravity and asymmetric targets."""
    result = {'image_up_dot_world_up': float(-world_optical_rotation[2, 1]),
              'centroids_uv': {}, 'errors': []}
    if result['image_up_dot_world_up'] < 0.95:
        result['errors'].append('Zero-pose image-up must point toward world +Z')
    for name in ('red_sphere', 'green_sphere'):
        rows, columns = np.nonzero(masks[name])
        if not rows.size:
            result['errors'].append(f'{name}: missing upright reference target')
        else:
            result['centroids_uv'][name] = [float(np.mean(columns)), float(np.mean(rows))]
    if len(result['centroids_uv']) == 2:
        red, green = (result['centroids_uv'][name] for name in ('red_sphere', 'green_sphere'))
        if not red[1] < principal[1] < green[1]:
            result['errors'].append('World-upper red target must be above center and lower green below')
        if not red[0] < principal[0] < green[0]:
            result['errors'].append('World-left red target must appear left and right green right')
    result['passed'] = not result['errors']
    return result


def evaluate(group, optical_tf, robot_pose, fixture_pose, fixtures, require_upright=False,
             depth_range=(0.07, 0.5)):
    color, depth_message, info = group
    if {message.header.frame_id for message in group} != {'camera_optical_frame'}:
        raise ValueError('RGB, depth and CameraInfo must all use camera_optical_frame')
    if color.encoding not in ('rgb8', 'bgr8'):
        raise ValueError('Color encoding must be rgb8 or bgr8')
    intrinsics = validate_camera_info(info, color.width, color.height)
    if (depth_message.width, depth_message.height) != (color.width, color.height):
        raise ValueError('Color and depth sizes differ')
    rgb = image_view(color, np.uint8, channels=3)
    if color.encoding == 'bgr8':
        rgb = rgb[..., ::-1]
    rgb = rgb.copy()
    depth = depth_metres(depth_message)
    valid = np.isfinite(depth)
    k = np.array(info.k).reshape(3, 3)
    rows, columns = np.indices(depth.shape)
    # K supports a possible skew; z is axial optical depth, not ray length.
    rays = np.stack((columns, rows, np.ones_like(columns)), axis=-1) @ np.linalg.inv(k).T
    optical = rays * depth[..., None]
    base_position, base_rotation = transform_pose(optical_tf)
    robot_position, robot_rotation = robot_pose
    fixture_position, fixture_rotation = fixture_pose
    world = (optical @ base_rotation.T + base_position) @ robot_rotation.T + robot_position
    points = (world - fixture_position) @ fixture_rotation
    masks = color_masks(rgb)
    result = {'stamp_ns': color.header.stamp.sec*1_000_000_000 + color.header.stamp.nanosec,
              'intrinsics': intrinsics, 'targets': {}, 'errors': []}
    result['depth'] = depth_statistics(depth_message, *depth_range)
    result['depth_contract_m'] = list(depth_range)
    if result['depth']['range_violations']:
        result['errors'].append('Finite positive depth lies outside the recorded sensor range')
    if result['depth']['negative_values']:
        result['errors'].append('Depth contains finite negative values')
    if require_upright:
        result['upright'] = assess_upright(masks, robot_rotation @ base_rotation, (k[0, 2], k[1, 2]))
        result['errors'].extend(result['upright']['errors'])
    target_names = ['red_sphere', 'green_sphere', 'yellow_near', 'blue_far']
    if 'magenta_sphere' in fixtures:
        target_names.append('magenta_sphere')
    for name in target_names:
        mask = masks[name]
        core = erode(mask)
        pixels = int(np.count_nonzero(core))
        finite_count = int(np.count_nonzero(core & valid))
        fraction = finite_count/pixels if pixels else 0.0
        target = {'core_pixels': pixels, 'valid_depth_pixels': finite_count,
                  'valid_depth_fraction': fraction, 'invalid_depth_fraction': 1-fraction}
        if fixtures[name]['shape'] == 'sphere':
            measured = depth[core & valid]
            target['min_depth_m'] = float(np.min(measured)) if measured.size else None
            target['max_depth_m'] = float(np.max(measured)) if measured.size else None
            target['median_depth_m'] = float(np.median(measured)) if measured.size else None
            if name == 'magenta_sphere' and (
                    not measured.size or target['min_depth_m'] <= 0.5 + DEPTH_TOLERANCE_M):
                result['errors'].append('magenta_sphere: valid axial Z must exceed 0.5 m')
            residual = np.abs(np.linalg.norm(points - np.array(fixtures[name]['center']), axis=-1)
                              - fixtures[name]['radius'])
            errors = residual[core & valid]
            target['median_radial_error_m'] = float(np.median(errors)) if errors.size else None
            target['p95_radial_error_m'] = float(np.percentile(errors, 95)) if errors.size else None
            surface = valid & (residual < 0.002)
            target['boundary_match_within_1px_fraction'] = boundary_fraction(mask, surface)
            if pixels < 500:
                result['errors'].append(f'{name}: fewer than 500 interior color pixels')
            if fraction < 0.98:
                result['errors'].append(f'{name}: less than 98% valid registered depth')
            if not errors.size or target['p95_radial_error_m'] >= 0.002:
                result['errors'].append(f'{name}: sphere surface p95 error is not below 2 mm')
            if target['boundary_match_within_1px_fraction'] < 0.95:
                result['errors'].append(f'{name}: RGB/depth boundaries do not match within 1 pixel')
        else:
            if pixels < 50:
                result['errors'].append(f'{name}: fewer than 50 interior color pixels')
            if pixels and fraction > 0.01:
                result['errors'].append(f'{name}: less than 99% invalid clipped depth')
        result['targets'][name] = target
    result['passed'] = not result['errors']
    return result, rgb, depth


def save_previews(directory, rgb, depth, depth_range=(0.07, 0.5)):
    try:
        os.environ.setdefault('MPLCONFIGDIR', '/tmp/agri_d405_matplotlib')
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return {}
    directory.mkdir(parents=True, exist_ok=True)
    color_path, depth_path = directory/'color.png', directory/'depth.png'
    plt.imsave(color_path, rgb)
    palette = plt.get_cmap('viridis').copy()
    palette.set_bad('black')
    plt.imsave(depth_path, np.ma.masked_invalid(depth), cmap=palette,
               vmin=depth_range[0], vmax=depth_range[1])
    return {'color': str(color_path), 'depth': str(depth_path)}


def collect(options):
    rclpy.init(args=[])
    node = Node('check_d405_scene', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node, spin_thread=False)
    queues = {name: deque(maxlen=12) for name in ('color', 'depth', 'info')}
    pending = deque(maxlen=12)
    samples = []

    def received(name, message):
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        if stamp <= 0:
            return
        queues[name].append((stamp, message))
        while all(queues.values()):
            names = ('color', 'depth', 'info')
            stamps = [queues[key][0][0] for key in names]
            # Native RGB-D renders all three streams with an identical stamp.
            if max(stamps) == min(stamps):
                pending.append(tuple(queues[key].popleft()[1] for key in names))
            else:
                queues[names[stamps.index(min(stamps))]].popleft()

    subscriptions = [
        node.create_subscription(Image, '/d405/color/image_raw', lambda msg: received('color', msg), qos_profile_sensor_data),
        node.create_subscription(Image, '/d405/aligned_depth_to_color/image_raw', lambda msg: received('depth', msg), qos_profile_sensor_data),
        node.create_subscription(CameraInfo, '/d405/color/camera_info', lambda msg: received('info', msg), qos_profile_sensor_data),
    ]
    started = time.monotonic()
    try:
        while time.monotonic()-started < options.timeout and len(samples) < options.frames:
            rclpy.spin_once(node, timeout_sec=0.02)
            for _ in range(len(pending)):
                group = pending.popleft()
                stamp = group[0].header.stamp
                try:
                    transform = buffer.lookup_transform('base_footprint', 'camera_optical_frame',
                                                        Time.from_msg(stamp, clock_type=ClockType.ROS_TIME)).transform
                except TransformException:
                    pending.append(group)
                    continue
                samples.append((group, transform))
                if len(samples) >= options.frames:
                    break
    finally:
        assert listener is not None and subscriptions
        node.destroy_node()
        rclpy.shutdown()
    if len(samples) < options.frames:
        raise RuntimeError(f'Collected only {len(samples)}/{options.frames} synchronized RGB-D frames with image-time TF')
    return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, default=DEFAULT_DIRECTORY/'scene.json')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--frames', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--require-upright', action='store_true',
                        help='Require world-up image orientation at the arm zero pose')
    parser.add_argument('--max-depth', type=float,
                        help='Assert upper limit matches scene metadata (legacy default: 0.5 m)')
    options = parser.parse_args()
    if options.frames < 1 or options.timeout <= 0 or not math.isfinite(options.timeout):
        parser.error('frames and finite timeout must be positive')
    output = options.output or options.scene.parent/'scene_report.json'
    report = {'passed': False, 'errors': [], 'truth_scope': 'simulation acceptance only'}
    try:
        metadata = json.loads(options.scene.read_text())
        if not isinstance(metadata, dict):
            raise ValueError('Scene metadata must be a mapping')
        if metadata['world'] != WORLD or metadata['fixture_model'] != MODEL:
            raise ValueError('Scene metadata must describe d405_validation/d405_calibration_targets')
        if options.require_upright and metadata.get('fixture_orientation_reference') != 'world_up':
            raise ValueError('Upright acceptance requires a freshly generated world-up fixture')
        depth_range = scene_depth_range(metadata, options.max_depth)
        fixtures = scene_fixtures(metadata, depth_range)
        fixture_pose = model_pose(MODEL)
        initial_robot_pose = model_pose(ROBOT)
        samples = collect(options)
        robot_pose = model_pose(ROBOT)
        movement = float(np.linalg.norm(robot_pose[0]-initial_robot_pose[0]))
        rotation_movement = float(np.linalg.norm(robot_pose[1]-initial_robot_pose[1]))
        if movement > 0.001 or rotation_movement > 0.003:
            report['errors'].append('Robot moved during static fixture acceptance; run setup after settling')
        frame_reports = []
        previews = {}
        for group, transform in samples:
            evaluated, rgb, depth = evaluate(group, transform, robot_pose, fixture_pose, fixtures,
                                            require_upright=options.require_upright,
                                            depth_range=depth_range)
            frame_reports.append(evaluated)
            report['errors'].extend(evaluated['errors'])
            if not previews:
                previews = save_previews(output.parent, rgb, depth, depth_range)
        report.update({
            'world': WORLD, 'fixture_model': MODEL, 'frames_checked': len(frame_reports),
            'depth_contract_m': list(depth_range),
            'frames': frame_reports, 'previews': previews,
            'robot_world_translation_drift_m': movement,
            'fixture_world_xyz_m': fixture_pose[0].tolist(),
            'fixture_world_rotation': fixture_pose[1].tolist(),
            'robot_world_xyz_m': robot_pose[0].tolist(),
        })
        report['passed'] = not report['errors']
    except (OSError, ValueError, RuntimeError, KeyError, subprocess.TimeoutExpired) as exc:
        report['errors'].append(str(exc))
    payload = json.dumps(report, indent=2, allow_nan=False)+'\n'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(payload)
    print(payload, end='')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
