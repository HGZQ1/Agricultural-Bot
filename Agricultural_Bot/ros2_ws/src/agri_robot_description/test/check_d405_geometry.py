#!/usr/bin/env python3
"""Validate the wrist RGB-D origin, CAD front face, and standard optical axes.

  python3 test/check_d405_geometry.py urdf/agri_robot.urdf.xacro --sdf

The CAD contains the housing and rear mounting holes, not the two imager
centers. The simulated aligned RGB-D pinhole uses the front-glass center,
3.7 mm inward, rather than claiming physical left-imager calibration.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from check_mid360_geometry import close, matrix_product, origin, rotate, rotation


def frame_transform(root, frame, relative_to="base_footprint"):
    """Return the model transform with all movable joints at zero position."""
    if frame == relative_to:
        return [0, 0, 0], rotation([0, 0, 0])
    joint = next(node for node in root.findall("joint")
                 if node.find("child").attrib["link"] == frame)
    parent_position, parent_rotation = frame_transform(
        root, joint.find("parent").attrib["link"], relative_to)
    local_position, local_rotation = origin(joint)
    position = [a + b for a, b in zip(parent_position, rotate(parent_rotation, local_position))]
    return position, matrix_product(parent_rotation, local_rotation)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("--sdf", action="store_true")
    args = parser.parse_args()
    if args.model.name.endswith(".xacro"):
        result = subprocess.run(["xacro", str(args.model)], check=True,
                                capture_output=True, text=True)
        root = ET.fromstring(result.stdout)
    else:
        root = ET.parse(args.model).getroot()

    body = root.find("link[@name='camera_lens_link']")
    assert body is not None, "The D405 CAD housing needs its own physical link"
    assert body.find("inertial/mass").attrib["value"] == "0.0387408552979231"
    close(origin(body.find("inertial"))[0],
          [-8.27600293560327e-05, -0.000518418102922302, 0.0113997621610378],
          1e-12, "preserved CAD camera-body COM")
    for tag in ["visual", "collision"]:
        component = body.find(tag)
        assert component.find("geometry/mesh").attrib["filename"].endswith(
            "/meshes/visual/camera_optical_frame.STL")
        position, orientation = origin(component)
        close(position, [0, 0, 0], 1e-12, f"preserved body {tag} position")
        for row, expected in zip(orientation, rotation([0, 0, 0])):
            close(row, expected, 1e-12, f"preserved body {tag} orientation")
    for name in ["d405_sensor_frame", "camera_optical_frame"]:
        frame = root.find(f"link[@name='{name}']")
        assert frame is not None and len(frame) == 0, f"{name} must be a massless TF frame"
    for name, expected_position, expected_rpy in [
        ("camera_joint", [-0.0400000000000365, 0.0485000000000264, -0.0225],
         [-1.5707963267949, 0, 0]),
        ("camera_option_joint", [-0.017221, -0.0195, 0.037803], [-1.5898, 0, -1.5708]),
    ]:
        position, orientation = origin(root.find(f"joint[@name='{name}']"))
        close(position, expected_position, 1e-12, f"preserved {name} position")
        for row, expected in zip(orientation, rotation(expected_rpy)):
            close(row, expected, 1e-12, f"preserved {name} orientation")

    scan_position, scan_rotation = origin(root.find("joint[@name='d405_sensor_frame_joint']"))
    optical_position, optical_rotation = origin(root.find("joint[@name='camera_optical_joint']"))
    close(optical_position, [0, 0, 0], 1e-12, "coincident RGB-D and optical origins")
    expected_optical = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    for row, expected in zip(optical_rotation, expected_optical):
        close(row, expected, 1e-12, "optical X right, Y down, Z forward")

    payload = (Path(__file__).resolve().parents[1]
               / "meshes/visual/camera_optical_frame.STL").read_bytes()
    count = struct.unpack_from("<I", payload, 80)[0]
    assert len(payload) == 84 + 50 * count, "Unexpected binary STL length"
    vertices = []
    front_triangles = []
    weighted_normal = [0.0, 0.0, 0.0]
    front_area = 0.0
    for face in struct.iter_unpack("<12fH", payload[84:]):
        triangle = [face[3 + 3 * i:6 + 3 * i] for i in range(3)]
        vertices.extend(triangle)
        u = [a - b for a, b in zip(triangle[1], triangle[0])]
        v = [a - b for a, b in zip(triangle[2], triangle[0])]
        cross = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                 u[0] * v[1] - u[1] * v[0]]
        length = math.sqrt(sum(value * value for value in cross))
        if length < 1e-12:
            continue
        normal = [value / length for value in cross]
        if normal[2] < -0.998 and 0.037 < normal[1] < 0.039:
            front_area += length / 2
            weighted_normal = [a + b * length / 2 for a, b in zip(weighted_normal, normal)]
            front_triangles.append((triangle, length / 2))
    assert front_area > 0.0015, "Cannot identify the CAD front face"
    norm = math.sqrt(sum(value * value for value in weighted_normal))
    front_normal = [value / norm for value in weighted_normal]
    close([row[0] for row in scan_rotation], front_normal, 1e-7,
          "Gazebo +X points through the front glass, away from rear mounting holes")
    theta = math.atan2(front_normal[1], -front_normal[2])
    untilt = list(zip(*rotation([theta, 0, 0])))
    aligned = [rotate(untilt, vertex) for vertex in vertices]
    dimensions = [max(v[i] for v in aligned) - min(v[i] for v in aligned) for i in range(3)]
    close(dimensions, [0.04209, 0.042, 0.023], 1e-7, "CAD housing envelope")
    plane_areas = defaultdict(float)
    for triangle, area in front_triangles:
        centroid = [sum(vertex[i] for vertex in triangle) / 3 for i in range(3)]
        plane_areas[round(rotate(untilt, centroid)[2], 6)] += area
    glass_plane = max(plane_areas, key=plane_areas.get)
    close(rotate(untilt, scan_position), [0, 0, glass_plane + 0.0037], 1e-7,
          "central virtual pinhole 3.7 mm behind the front glass")

    lens_optical = matrix_product(scan_rotation, optical_rotation)
    for point, expected in [([0.2, 0, 0], [0, 0, 0.2]),
                            ([0.2, -0.01, -0.02], [0.01, 0.02, 0.2])]:
        body_point = [a + b for a, b in zip(scan_position, rotate(scan_rotation, point))]
        optical_point = rotate(list(zip(*lens_optical)),
                               [a - b for a, b in zip(body_point, scan_position)])
        close(optical_point, expected, 1e-12, "front points project to positive optical depth")
    gripper_position, gripper_rotation = frame_transform(root, "d405_sensor_frame", "gripper_base_link")
    assert gripper_rotation[0][0] < -0.999, "D405 must look toward the gripper fingertips (-X)"
    assert abs(gripper_rotation[2][0]) < 1e-4, "Unexpected wrist camera sidewards direction"
    # An unchanged forward normal cannot detect a 180-degree image roll.
    # Use the independent gripper assembly and zero-pose base up directions.
    assert gripper_rotation[1][2] > 0.999, "Camera image-up must follow the bracket's +Y"
    base_position, base_rotation = frame_transform(root, "d405_sensor_frame")
    base_optical_rotation = frame_transform(root, "camera_optical_frame")[1]
    assert base_rotation[2][2] > 0.999, "Zero-pose camera image-up must point toward base +Z"
    assert base_optical_rotation[2][1] < -0.999, "Zero-pose optical +Y must point down"
    assert base_optical_rotation[0][0] > 0.999, "Zero-pose optical +X must point toward base +X"

    if args.sdf:
        gazebo = ET.SubElement(root, "gazebo", {"reference": "d405_sensor_frame"})
        sensor = ET.SubElement(gazebo, "sensor", {"name": "d405_geometry_probe", "type": "rgbd_camera"})
        ET.SubElement(sensor, "pose").text = "0 0 0 0 0 0"
        with tempfile.TemporaryDirectory(prefix="d405_geometry_") as directory:
            path = Path(directory) / "probe.urdf"
            ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
            result = subprocess.run(["gz", "sdf", "-p", str(path), "--precision", "17"], check=True,
                                    capture_output=True, text=True)
        sdf = ET.fromstring(result.stdout)
        emitted = sdf.find(".//link[@name='gripper_base_link']/sensor[@name='d405_geometry_probe']")
        assert emitted is not None, "RGB-D sensor was lost during fixed-joint lumping"
        pose = [float(value) for value in emitted.findtext("pose").split()]
        # URDF-to-SDF serializes the embedded sensor pose with six significant
        # digits before --precision applies; combined RPY rounding can exceed
        # 5e-6 per matrix component. Canonical axes above are checked exactly.
        close(pose[:3], gripper_position, 5e-6, "SDF optical origin at the wrist")
        for row, expected in zip(rotation(pose[3:]), gripper_rotation):
            close(row, expected, 1e-5, "SDF wrist RGB-D viewing axes")
        print("SDF preserves the RGB-D camera origin and axes after fixed-joint lumping.")
    print(json.dumps({
        "passed": True, "reference": "aligned RGB-D central virtual pinhole",
        "glass_inward_offset_m": 0.0037, "cad_tilt_deg": math.degrees(theta),
        "lens_to_sensor_xyz_m": scan_position,
        "sensor_in_gripper_xyz_m": gripper_position,
        "sensor_in_gripper_rotation": gripper_rotation,
        "sensor_in_base_zero_xyz_m": base_position,
        "sensor_in_base_zero_rotation": base_rotation,
        "zero_pose_image_up_dot_base_up": base_rotation[2][2],
        "zero_pose_optical_down_dot_base_up": base_optical_rotation[2][1],
    }, indent=2))


if __name__ == "__main__":
    main()
