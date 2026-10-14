#!/usr/bin/env python3
"""Offline check of optional analytic tire collisions against original CAD STL.

Source both workspaces, then run this file. No simulator or robot is launched.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import struct
import subprocess
import xml.etree.ElementTree as ET

from check_mid360_geometry import origin, rotate


REPO = Path(__file__).resolve().parents[4]
DESCRIPTION = REPO / "ros2_ws/src/agri_robot_description"
WHEELS = ("f1", "f2", "r1", "r2")


def expanded(model, mode=None):
    command = ["xacro", str(model)]
    if mode is not None:
        command.append(f"wheel_collision_geometry:={mode}")
    if "agri_sim_description" in str(model):
        command += ["use_lidar:=false", "use_camera:=false", "use_control:=false"]
    result = subprocess.run(command, capture_output=True, text=True)
    return result, ET.fromstring(result.stdout) if result.returncode == 0 else None


def structure(node):
    # XML layout/comments are immaterial; every element and attribute matters.
    return (node.tag, tuple(sorted(node.attrib.items())), (node.text or "").strip(),
            tuple(structure(child) for child in node))


def without_tire_collisions(root):
    root = copy.deepcopy(root)
    for wheel in WHEELS:
        link = root.find(f"link[@name='{wheel}_wheel_link']")
        for collision in link.findall("collision"):
            link.remove(collision)
    return structure(root)


def cad_vertices(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + 50 * count, "Expected complete binary STL"
    for triangle in range(count):
        for vertex in range(3):
            yield struct.unpack_from("<3f", data, 84 + triangle * 50 + 12 + vertex * 12)


def main():
    details = []
    models = [DESCRIPTION / "urdf/agri_robot.urdf.xacro",
              REPO / "sim_ws/src/agri_sim_description/urdf/agri_robot.gazebo.urdf.xacro"]
    for model in models:
        result, default = expanded(model)
        assert result.returncode == 0, result.stderr
        result, mesh = expanded(model, "mesh")
        assert result.returncode == 0, result.stderr
        result, cylinder = expanded(model, "cylinder")
        assert result.returncode == 0, result.stderr
        assert structure(default) == structure(mesh), "Default must stay exactly mesh"
        assert without_tire_collisions(mesh) == without_tire_collisions(cylinder), \
            "Only the four tire collisions may change (including all visual/inertia/joint/control data)"
        bad, _ = expanded(model, "invalid_geometry")
        assert bad.returncode != 0, "Unsupported geometry must fail instead of dropping collisions"
        for wheel in WHEELS:
            old = mesh.find(f"link[@name='{wheel}_wheel_link']/collision")
            new = cylinder.find(f"link[@name='{wheel}_wheel_link']/collision")
            assert old.find("origin").attrib == {"xyz": "0 -0.05 0", "rpy": "0 0 0"}
            assert old.find("geometry/mesh").attrib["filename"].endswith(f"/{wheel}_wheel_link.STL")
            assert len(cylinder.find(f"link[@name='{wheel}_wheel_link']").findall("collision")) == 1
            geometry = new.find("geometry/cylinder")
            radius, width = float(geometry.attrib["radius"]), float(geometry.attrib["length"])
            assert abs(radius - 0.127) < 1e-12 and abs(width - 0.100) < 1e-12
            center, rotation = origin(new)
            assert center == [0.0, 0.0, 0.0]
            axis = rotate(rotation, [0, 0, 1])
            assert abs(axis[0]) < 1e-12 and abs(abs(axis[1]) - 1) < 1e-12 and abs(axis[2]) < 1e-12
            vertices = list(cad_vertices(DESCRIPTION / f"meshes/visual/{wheel}_wheel_link.STL"))
            radial_max = max(math.hypot(v[0], v[2]) for v in vertices)
            lower = [min(v[i] for v in vertices) for i in range(3)]
            upper = [max(v[i] for v in vertices) for i in range(3)]
            cad_center = [(a + b) / 2 for a, b in zip(lower, upper)]
            assert max(abs(v) for v in (cad_center[0], cad_center[1] - 0.05, cad_center[2])) < 1e-8
            assert abs(radial_max - radius) < 1e-8 and abs(upper[1] - lower[1] - width) < 1e-8
            # CAD float32 noise is <5 nm; every shifted CAD point fits the cylinder
            # within 10 nm. This checks the whole mesh, not a handpicked vertex.
            assert all(math.hypot(v[0], v[2]) <= radius + 1e-8 and
                       abs(v[1] - 0.05) <= width / 2 + 1e-8 for v in vertices)
            assert new.find("geometry/mesh") is None
            details.append({"model": model.name, "wheel": wheel, "radius_m": radius,
                            "width_m": width, "cad_radius_max_m": radial_max,
                            "cad_bounds_m": [lower, upper], "cylinder_axis": axis,
                            "max_local_x_support_increase_m": radius - upper[0],
                            "chamfer_note": "cylinder encloses CAD end chamfers; corner contacts change"})
    print(json.dumps({"passed": True, "models": len(models), "wheel_checks": len(details),
                      "cad_envelope_tolerance_m": 1e-8, "details": details}, indent=2))


if __name__ == "__main__":
    main()
