#!/usr/bin/env python3
"""Check MID-360 emission geometry against the CAD mesh, optionally through SDF.

Run after sourcing ROS and ros2_ws/install/local_setup.bash:
  python3 test/check_mid360_geometry.py urdf/agri_robot.urdf.xacro --sdf

Only the Python standard library is required. The optical-dome sphere check
guards against reintroducing the chassis/COM origin as the emission origin.
"""

from __future__ import annotations

import argparse
import math
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path


def matrix_product(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3))
             for j in range(3)] for i in range(3)]


def rotate(rotation, vector):
    return [sum(row[k] * vector[k] for k in range(3)) for row in rotation]


def rotation(rpy):
    roll, pitch, yaw = rpy
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def origin(element):
    node = element.find("origin")
    xyz = [float(v) for v in node.attrib.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in node.attrib.get("rpy", "0 0 0").split()]
    return xyz, rotation(rpy)


def close(actual, expected, tolerance, label):
    error = max(abs(a - b) for a, b in zip(actual, expected))
    if error > tolerance:
        raise AssertionError(f"{label}: error {error:g}; {actual} != {expected}")


def solve(matrix, rhs):
    """Small pivoted linear solve for a least-squares sphere fit."""
    rows = [list(row) + [value] for row, value in zip(matrix, rhs)]
    for column in range(len(rows)):
        pivot = max(range(column, len(rows)), key=lambda i: abs(rows[i][column]))
        rows[column], rows[pivot] = rows[pivot], rows[column]
        scale = rows[column][column]
        if abs(scale) < 1e-15:
            raise AssertionError("Optical dome has a degenerate sphere fit")
        rows[column] = [value / scale for value in rows[column]]
        for i in range(len(rows)):
            if i != column:
                scale = rows[i][column]
                rows[i] = [a - scale * b for a, b in zip(rows[i], rows[column])]
    return [row[-1] for row in rows]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("--sdf", action="store_true", help="also check fixed-joint sensor lumping")
    args = parser.parse_args()
    if args.model.name.endswith(".xacro"):
        xml = subprocess.run(["xacro", str(args.model)], check=True,
                             capture_output=True, text=True).stdout
        root = ET.fromstring(xml)
    else:
        root = ET.parse(args.model).getroot()

    body_position, body_rotation = origin(root.find("joint[@name='mid360_joint']"))
    scan_position, scan_rotation = origin(root.find("joint[@name='mid360_sensor_frame_joint']"))
    horizontal_position, horizontal_rotation = origin(
        root.find("joint[@name='mid360_scan_frame_joint']"))
    base_position, base_rotation = origin(root.find("joint[@name='base_footprint_joint']"))
    optical_cad = [a + b for a, b in zip(body_position, rotate(body_rotation, scan_position))]
    optical_ros = [a + b for a, b in zip(base_position, rotate(base_rotation, optical_cad))]
    scan_ros = matrix_product(base_rotation, matrix_product(body_rotation, scan_rotation))
    horizontal_ros = matrix_product(
        scan_ros, horizontal_rotation)
    close(optical_ros, [-0.401439121228018, 0.00343775880066549, 0.459044570728433],
          1e-9, "CAD optical-window center in base_footprint")
    expected = rotation([0, -math.pi / 12, 0])
    for actual_row, expected_row in zip(scan_ros, expected):
        close(actual_row, expected_row, 1e-12, "forward/left/dome-up scanning axes")
    horizontal_optical_ros = [
        a + b for a, b in zip(
            optical_ros,
            rotate(scan_ros, horizontal_position),
        )
    ]
    close(horizontal_optical_ros, optical_ros, 1e-12,
          "horizontal navigation frame origin")
    for actual_row, expected_row in zip(horizontal_ros, rotation([0, 0, 0])):
        close(actual_row, expected_row, 1e-12,
              "horizontal navigation frame axes")

    link = root.find("link[@name='mid360_link']")
    for tag in ["visual", "collision"]:
        mesh_position, mesh_rotation = origin(link.find(tag))
        close([a + b for a, b in zip(body_position, rotate(body_rotation, mesh_position))],
              [0, 0, 0], 1e-12, f"unchanged assembled {tag} position")
        for i, row in enumerate(matrix_product(body_rotation, mesh_rotation)):
            close(row, [float(j == i) for j in range(3)], 1e-12,
                  f"unchanged assembled {tag} rotation")

    mesh = Path(__file__).resolve().parents[1] / "meshes/visual/mid360_link.STL"
    payload = mesh.read_bytes()
    count = struct.unpack_from("<I", payload, 80)[0]
    if len(payload) != 84 + count * 50:
        raise AssertionError("Unexpected binary STL length")
    cad_scan_rotation = matrix_product(body_rotation, scan_rotation)
    inverse = list(zip(*cad_scan_rotation))
    dome_points = set()
    normal_areas = defaultdict(float)
    for face in struct.iter_unpack("<12fH", payload[84:]):
        vertices = [face[3 + 3 * i:6 + 3 * i] for i in range(3)]
        for vertex in vertices:
            local = rotate(inverse, [a - b for a, b in zip(vertex, optical_cad)])
            if local[2] > 0.001:
                dome_points.add(tuple(local))
        u = [a - b for a, b in zip(vertices[1], vertices[0])]
        v = [a - b for a, b in zip(vertices[2], vertices[0])]
        cross = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                 u[0] * v[1] - u[1] * v[0]]
        length = math.sqrt(sum(value * value for value in cross))
        if length < 1e-12:
            continue
        normal = [value / length for value in cross]
        if normal[1] < 0:
            normal = [-value for value in normal]
        normal_areas[tuple(round(value, 3) for value in normal)] += length / 2
    dominant = max(normal_areas, key=normal_areas.get)
    dome_axis = [row[2] for row in cad_scan_rotation]
    close(dome_axis, dominant, 0.003, "scanning Z follows the tilted CAD dome normal")
    if len(dome_points) < 500:
        raise AssertionError("Too few optical-dome vertices")
    rows = [[2 * x, 2 * y, 2 * z, 1] for x, y, z in dome_points]
    rhs = [sum(value * value for value in point) for point in dome_points]
    normal_matrix = [[sum(row[i] * row[j] for row in rows) for j in range(4)]
                     for i in range(4)]
    normal_rhs = [sum(row[i] * value for row, value in zip(rows, rhs)) for i in range(4)]
    fit = solve(normal_matrix, normal_rhs)
    center = fit[:3]
    radius = math.sqrt(fit[3] + sum(value * value for value in center))
    residual = math.sqrt(sum((math.dist(point, center) - radius) ** 2
                             for point in dome_points) / len(dome_points))
    close(center, [0, 0, 0], 0.0001, "emission origin near fitted optical-window center")
    close([radius], [0.022], 0.0001, "optical-dome radius")
    if residual > 0.0001:
        raise AssertionError(f"Optical-dome fit RMS {residual:g} exceeds 0.1 mm")

    if args.sdf:
        gazebo = ET.SubElement(root, "gazebo", {"reference": "mid360_sensor_frame"})
        sensor = ET.SubElement(gazebo, "sensor", {"name": "mid360_geometry_probe", "type": "gpu_lidar"})
        ET.SubElement(sensor, "pose").text = "0 0 0 0 0 0"
        with tempfile.TemporaryDirectory(prefix="mid360_geometry_") as directory:
            path = Path(directory) / "probe.urdf"
            ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
            result = subprocess.run(["gz", "sdf", "-p", str(path)], check=True,
                                    capture_output=True, text=True)
        sdf = ET.fromstring(result.stdout)
        emitted = sdf.find(".//link[@name='base_footprint']/sensor[@name='mid360_geometry_probe']")
        if emitted is None:
            raise AssertionError("Sensor was lost during fixed-joint lumping")
        pose = [float(value) for value in emitted.findtext("pose").split()]
        close(pose[:3], optical_ros, 5e-6, "SDF emission position after fixed-joint lumping")
        for actual_row, expected_row in zip(rotation(pose[3:]), expected):
            close(actual_row, expected_row, 5e-6, "SDF emission axes after fixed-joint lumping")
        print("SDF fixed-joint sensor position and axes are preserved.")
    print(f"MID-360 geometry OK: optical center {optical_ros}; pitch -15 degrees; "
          f"dome radius {radius * 1000:.4f} mm, RMS {residual * 1000:.4f} mm.")


if __name__ == "__main__":
    main()
