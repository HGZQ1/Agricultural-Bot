#!/usr/bin/env python3
"""Check MID-360 rays against the known mid360_validation Gazebo world.

Gazebo model pose is read only by this simulation acceptance script. It is
never supplied to an algorithm node. The four wall inner faces are +/-2.95 m;
the ground plane is z=0. Near robot returns are excluded from this test.
"""

import argparse
import json
from math import cos, sin
from pathlib import Path
import re
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = cos(roll), sin(roll), cos(pitch), sin(pitch), cos(yaw), sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ])


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--backend', default='unspecified')
    options = parser.parse_args()
    result = subprocess.run(
        ['gz', 'model', '-m', 'agri_robot', '-p'], capture_output=True, text=True,
        timeout=15, check=True,
    )
    if 'world [mid360_validation]' not in result.stdout:
        raise RuntimeError('This check requires world:=mid360_validation.sdf')
    vectors = re.findall(r'^\s*\[([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)\]',
                         result.stdout, re.MULTILINE)
    if len(vectors) != 2:
        raise RuntimeError('Cannot read Gazebo model world pose: ' + result.stdout)
    translation, angles = (np.array(vector, dtype=float) for vector in vectors)
    model_rotation = rpy_matrix(*angles)

    rclpy.init()
    node = Node('check_mid360_scene')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node, spin_thread=False)
    clouds = []
    subscription = node.create_subscription(
        PointCloud2, '/mid360/points', clouds.append, qos_profile_sensor_data,
    )
    started = time.monotonic()
    transform = None
    while time.monotonic() - started < 30:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform(
                'base_footprint', 'mid360_sensor_frame', Time()).transform
        except TransformException:
            pass
        if transform is not None and len(clouds) >= 3:
            break
    if transform is None or not clouds:
        raise RuntimeError('No sensor TF or point cloud within 30 s')
    # Hold references to listener/subscription throughout collection.
    assert listener is not None and subscription is not None
    errors = []
    origin = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    rotation = quaternion_matrix(transform.rotation)
    for cloud in clouds[-3:]:
        points = point_cloud2.read_points_numpy(
            cloud, field_names=['x', 'y', 'z'], skip_nans=False).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        points = points[np.linalg.norm(points, axis=1) > 1.5]
        world = (points @ rotation.T + origin) @ model_rotation.T + translation
        distances = np.minimum.reduce([
            np.abs(np.abs(world[:, 0])-2.95),
            np.abs(np.abs(world[:, 1])-2.95), np.abs(world[:, 2]),
        ])
        errors.extend(distances.tolist())
    node.destroy_node()
    rclpy.shutdown()
    if len(errors) < 1000:
        raise RuntimeError('Too few environment returns for wall/ground validation')
    errors = np.asarray(errors)
    report = {
        'backend': options.backend, 'passed': bool(np.mean(errors < 0.03) >= 0.98),
        'environment_returns': len(errors), 'within_30mm_fraction': float(np.mean(errors < 0.03)),
        'median_surface_error_m': float(np.median(errors)),
        'p95_surface_error_m': float(np.percentile(errors, 95)),
        'gazebo_model_xyz_m': translation.tolist(),
        'sensor_xyz_in_base_m': origin.tolist(),
    }
    payload = json.dumps(report, indent=2) + '\n'
    print(payload, end='')
    if options.output:
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(payload)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
