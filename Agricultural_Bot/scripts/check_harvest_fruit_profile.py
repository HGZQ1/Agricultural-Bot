#!/usr/bin/env python3
"""Independently audit a generated harvest-demo field, without ROS or IK.

Gazebo ground truth is used only for offline scene acceptance. Passing means
that recorded proxy geometry, per-station ripe coverage and preserved scan
geometry agree. It does not establish IK, full-arm collision or camera FOV.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


TOLERANCE = 1e-9
SUPPORT_RADIUS_M = 0.004
PARAMETERS = {
    "shoulder_world_z_m": "harvest_arm_reference_height",
    "shoulder_xy_uncertainty_m": "harvest_shoulder_lateral_offset",
    "tcp_offset_m": "harvest_tcp_offset",
    "pregrasp_distance_m": "harvest_pregrasp_distance",
    "flange_distance_budget_m": "harvest_flange_radius",
    "parking_xy_allowance_m": "harvest_parking_xy_allowance",
    "parking_yaw_allowance_rad": "harvest_parking_yaw_allowance",
    "preserved_scan_max_z_m": "harvest_preserve_scan_max_z",
}


def number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) \
            or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def indexed(items, name):
    if not isinstance(items, list) or not all(isinstance(item, dict) for item in items):
        raise ValueError(f"{name} must be a list of mappings")
    result = {}
    for item in items:
        identifier = item.get("id")
        if not isinstance(identifier, str) or not identifier or identifier in result:
            raise ValueError(f"{name} contains a missing or duplicate id: {identifier!r}")
        result[identifier] = item
    return result


def vector(text, size, name):
    try:
        values = [float(value) for value in text.split()]
    except (ValueError, AttributeError) as error:
        raise ValueError(f"{name} must contain {size} finite numbers") from error
    if len(values) != size or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return values


def matches(actual, expected):
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(
            matches(a, e) for a, e in zip(actual, expected)
        )
    return isinstance(actual, (int, float)) and not isinstance(actual, bool) \
        and math.isfinite(actual) and abs(actual - expected) <= TOLERANCE


def sphere(fruit):
    values = [number(fruit.get(key), f"{fruit.get('id')}.{key}")
              for key in ("x", "y", "z", "radius")]
    if values[3] <= 0 or not isinstance(fruit.get("ripe"), bool):
        raise ValueError(f"{fruit.get('id')}: positive radius and boolean ripe required")
    return values


def audit(field, mission, baseline, field_sdf=None):
    errors = []
    profile = field.get("harvest_profile")
    if not isinstance(profile, dict):
        raise ValueError("field metadata has no harvest_profile")
    params = field.get("parameters", {})
    assumptions = profile.get("assumptions", {})
    if not isinstance(params, dict) or not isinstance(assumptions, dict):
        raise ValueError("parameters and harvest_profile.assumptions must be mappings")
    # Newer metadata may put numeric assumptions directly in the profile. The
    # current generator records them as harvest_* parameters; never substitute
    # a different arm model or an unrecorded reach radius here.
    geometry = {
        name: number(assumptions.get(name, params.get(parameter)), parameter)
        for name, parameter in PARAMETERS.items()
    }
    for name, value in geometry.items():
        if value < 0:
            raise ValueError(f"{name} must be nonnegative")
    if geometry["flange_distance_budget_m"] <= 0:
        raise ValueError("flange_distance_budget_m must be positive")
    if geometry["parking_yaw_allowance_rad"] > math.pi:
        raise ValueError("parking_yaw_allowance_rad must not exceed pi")
    offset = number(mission.get("robot", {}).get(
        "arm_base_longitudinal_offset_m",
        params.get("harvest_arm_reference_longitudinal_offset"),
    ), "robot.arm_base_longitudinal_offset_m")
    geometry["arm_base_longitudinal_offset_m"] = offset
    allowance = geometry["parking_xy_allowance_m"] + 2 * abs(offset) * math.sin(
        geometry["parking_yaw_allowance_rad"] / 2,
    )
    geometry["combined_parking_xy_allowance_m"] = allowance
    threshold = geometry["preserved_scan_max_z_m"]
    tx, ty, angle = [number(value, "alignment.world_to_map") for value in
                     mission["alignment"]["world_to_map"]]
    cosine, sine = math.cos(angle), math.sin(angle)
    plants = indexed(field.get("plants"), "plants")
    baseline_plants = indexed(baseline.get("plants"), "baseline plants")
    fruits = indexed(field.get("fruits"), "fruits")
    baseline_fruits = indexed(baseline.get("fruits"), "baseline fruits")
    if plants != baseline_plants:
        errors.append("plant geometry differs from baseline")
    for fruit in list(fruits.values()) + list(baseline_fruits.values()):
        sphere(fruit)
        if fruit.get("plant_id") not in plants:
            raise ValueError(f"{fruit['id']}: unknown plant_id")
    entries = mission["route"]["waypoints"]
    selected = {}
    for entry in entries:
        if not entry.get("enabled", True) or entry.get("task", "harvest") != "harvest":
            continue
        plant_id = entry["plant"]["id"]
        if plant_id not in plants or plant_id in selected:
            raise ValueError(f"missing or duplicate mission plant: {plant_id}")
        selected[plant_id] = entry
    if not selected:
        raise ValueError("mission has no enabled harvest stations")

    unselected_ids = {identifier for identifier, fruit in baseline_fruits.items()
                      if fruit["plant_id"] not in selected}
    current_unselected = {identifier for identifier, fruit in fruits.items()
                          if fruit["plant_id"] not in selected}
    if unselected_ids != current_unselected:
        errors.append("unselected plant fruit ids differ from baseline")
    changed_unselected = [identifier for identifier in sorted(unselected_ids)
                          if fruits.get(identifier) != baseline_fruits[identifier]]
    if changed_unselected:
        errors.append(f"unselected fruit records changed: {changed_unselected}")
    protected = {identifier: fruit for identifier, fruit in baseline_fruits.items()
                 if fruit["plant_id"] in selected and fruit["z"] - fruit["radius"] <= threshold}
    changed_protected = [identifier for identifier, fruit in protected.items()
                         if identifier not in fruits or not matches(sphere(fruits[identifier]), sphere(fruit))]
    if changed_protected:
        errors.append(f"protected scan spheres changed or removed: {changed_protected}")
    new_low = [identifier for identifier, fruit in fruits.items() if fruit["plant_id"] in selected
               and fruit["z"] - fruit["radius"] <= threshold and identifier not in protected]
    if new_low:
        errors.append(f"new selected fruit enters preserved scan band: {new_low}")

    models = None
    if field_sdf is not None:
        world = ET.parse(field_sdf).getroot().find("world")
        if world is None:
            raise ValueError("field SDF has no world")
        models = {model.get("name"): model for model in world.findall("model")}
        for identifier, fruit in fruits.items():
            model = models.get(identifier)
            if model is None:
                errors.append(f"{identifier}: missing SDF model")
            else:
                check_sdf_sphere(model, fruit, errors)
    stations, candidate_checks = [], []
    for plant_id, entry in selected.items():
        plant = plants[plant_id]
        expected_plant = entry["plant"]["pose_world"]
        for key in ("x", "y"):
            if not matches(plant[key], number(expected_plant[key], f"{plant_id}.pose_world.{key}")):
                errors.append(f"{plant_id}: mission and field plant geometry differ")
        pose = entry["base_pose"]
        mx, my, myaw = [number(pose[key], f"{plant_id}.base_pose.{key}")
                         for key in ("x", "y", "yaw")]
        bx = cosine * (mx - tx) + sine * (my - ty)
        by = -sine * (mx - tx) + cosine * (my - ty)
        yaw = myaw - angle
        reference = [bx + offset * math.cos(yaw), by + offset * math.sin(yaw),
                     geometry["shoulder_world_z_m"]]
        side = entry.get("work_side")
        lateral = -math.sin(yaw) * (plant["x"] - bx) + math.cos(yaw) * (plant["y"] - by)
        if side not in ("left", "right") or lateral * (1 if side == "left" else -1) <= 0:
            errors.append(f"{plant_id}: work_side conflicts with parking pose")
        members = [fruit for fruit in fruits.values() if fruit["plant_id"] == plant_id]
        candidates = [fruit for fruit in members if isinstance(fruit.get("harvest_profile"), dict)]
        valid_ids, ripe_ids = [], []
        for fruit in candidates:
            xyz = sphere(fruit)[:3]
            radial = math.hypot(xyz[0] - reference[0], xyz[1] - reference[1])
            if radial <= geometry["tcp_offset_m"] + geometry["pregrasp_distance_m"]:
                errors.append(f"{fruit['id']}: insufficient horizontal approach distance")
                continue
            direction = [(xyz[0] - reference[0]) / radial,
                         (xyz[1] - reference[1]) / radial, 0.0]
            flange = [xyz[i] - geometry["tcp_offset_m"] * direction[i] for i in range(3)]
            pregrasp = [flange[i] - geometry["pregrasp_distance_m"] * direction[i]
                        for i in range(3)]

            def distance(point):
                horizontal = math.hypot(point[0] - reference[0], point[1] - reference[1])
                return math.hypot(horizontal + geometry["shoulder_xy_uncertainty_m"] + allowance,
                                  point[2] - reference[2])

            grasp_distance, pregrasp_distance = distance(flange), distance(pregrasp)
            margin = geometry["flange_distance_budget_m"] - max(grasp_distance, pregrasp_distance)
            candidate_ok = margin >= -TOLERANCE
            if not candidate_ok:
                errors.append(f"{fruit['id']}: independent flange-distance screen failed")
            recorded = fruit["harvest_profile"]
            comparison = {
                "nominal_arm_reference_world_xyz_m": reference,
                "nominal_approach_unit_world": direction,
                "nominal_grasp_flange_world_xyz_m": flange,
                "nominal_pregrasp_flange_world_xyz_m": pregrasp,
                "grasp_flange_worst_distance_m": grasp_distance,
                "pregrasp_flange_worst_distance_m": pregrasp_distance,
                "flange_distance_budget_m": geometry["flange_distance_budget_m"],
                "remaining_distance_margin_m": margin,
            }
            for key, expected in comparison.items():
                if not matches(recorded.get(key), expected):
                    errors.append(f"{fruit['id']}: recorded {key} disagrees with independent calculation")
            if recorded.get("waypoint_id") != entry["id"] or recorded.get("work_side") != side:
                errors.append(f"{fruit['id']}: candidate waypoint/work_side mismatch")
            if any(recorded.get(key) != "pending" for key in
                   ("ik_status", "full_arm_collision_status", "fov_status")):
                errors.append(f"{fruit['id']}: candidate must retain pending IK/collision/FOV status")
            if fruit["z"] - fruit["radius"] <= threshold:
                errors.append(f"{fruit['id']}: near fruit enters preserved scan band")
            support_bottom = fruit["z"] - SUPPORT_RADIUS_M
            if support_bottom <= threshold:
                errors.append(f"{fruit['id']}: assumed support enters preserved scan band")
            if models is not None:
                model = models.get(fruit["id"])
                if model is not None:
                    check_sdf_fruit(model, fruit, plant, threshold, errors)
            if candidate_ok:
                valid_ids.append(fruit["id"])
                if fruit["ripe"]:
                    ripe_ids.append(fruit["id"])
            candidate_checks.append({"fruit_id": fruit["id"], "plant_id": plant_id,
                                     "ripe": fruit["ripe"], "distance_passed": candidate_ok,
                                     "grasp_flange_worst_distance_m": grasp_distance,
                                     "pregrasp_flange_worst_distance_m": pregrasp_distance,
                                     "remaining_distance_margin_m": margin,
                                     "fruit_bottom_world_z_m": fruit["z"] - fruit["radius"],
                                     "support_bottom_world_z_m": support_bottom})
        if not ripe_ids:
            errors.append(f"{plant_id}: no ripe near-side distance candidate")
        stations.append({"plant_id": plant_id, "waypoint_id": entry["id"], "work_side": side,
                         "fruit_count": len(members), "ripe_count": sum(f["ripe"] for f in members),
                         "near_candidate_count": len(candidates), "distance_candidate_count": len(valid_ids),
                         "ripe_distance_candidate_count": len(ripe_ids),
                         "distance_candidate_ids": valid_ids, "ripe_distance_candidate_ids": ripe_ids})
    all_scan_spheres = {identifier: fruit for identifier, fruit in baseline_fruits.items()
                        if fruit["z"] - fruit["radius"] <= threshold}
    return {"passed": not errors, "audit_type": "independent_offline_proxy_geometry_not_IK",
            "errors": errors, "assumed_geometry": geometry,
            "validation": {"ik": "pending", "full_arm_collision": "pending", "camera_fov": "pending",
                           "support_sdf": "checked" if models is not None else "assumed_from_metadata"},
            "counts": {"stations": len(stations), "plants": len(plants), "fruits": len(fruits),
                       "selected_fruits": sum(station["fruit_count"] for station in stations),
                       "selected_ripe_fruits": sum(station["ripe_count"] for station in stations),
                       "near_candidates": len(candidate_checks),
                       "ripe_near_candidates": sum(check["ripe"] for check in candidate_checks),
                       "stations_with_ripe_distance_candidate": sum(
                           station["ripe_distance_candidate_count"] > 0 for station in stations),
                       "protected_scan_spheres": len(protected),
                       "preserved_all_scan_spheres": sum(
                           identifier in fruits and matches(sphere(fruits[identifier]), sphere(fruit))
                           for identifier, fruit in all_scan_spheres.items()),
                       "unchanged_unselected_fruits": len(unselected_ids) - len(changed_unselected)},
            "stations": stations, "candidate_checks": candidate_checks}


def check_sdf_sphere(model, fruit, errors):
    identifier = fruit["id"]
    pose = vector(model.findtext("pose"), 6, f"{identifier}.SDF pose")
    if not matches(pose, [fruit["x"], fruit["y"], fruit["z"], 0.0, 0.0, 0.0]):
        errors.append(f"{identifier}: SDF pose differs from metadata")
    collisions = model.findall("link/collision")
    if len(collisions) != 1 or collisions[0].find("geometry/sphere") is None:
        errors.append(f"{identifier}: expected one fruit sphere collision and no support collision")
    elif not matches(float(collisions[0].findtext("geometry/sphere/radius")), fruit["radius"]):
        errors.append(f"{identifier}: SDF collision radius differs from metadata")
    elif not matches(vector(collisions[0].findtext("pose", "0 0 0 0 0 0"), 6,
                            f"{identifier}.collision pose"), [0.0] * 6):
        errors.append(f"{identifier}: SDF collision must be centered on recorded fruit")


def check_sdf_fruit(model, fruit, plant, threshold, errors):
    identifier = fruit["id"]
    pose = vector(model.findtext("pose"), 6, f"{identifier}.SDF pose")
    support = model.find("link/visual[@name='harvest_support_visual']")
    if support is None or support.find("geometry/cylinder") is None:
        errors.append(f"{identifier}: missing cylindrical support visual")
        return
    radius = float(support.findtext("geometry/cylinder/radius"))
    length = float(support.findtext("geometry/cylinder/length"))
    branch_pose = vector(support.findtext("pose"), 6, f"{identifier}.support pose")
    dx, dy = fruit["x"] - plant["x"], fruit["y"] - plant["y"]
    stem_distance = math.hypot(dx, dy)
    expected_length = stem_distance - fruit["radius"]
    expected_pose = [-dx + expected_length * dx / stem_distance / 2,
                     -dy + expected_length * dy / stem_distance / 2,
                     0.0, 0.0, math.pi / 2, math.atan2(dy, dx)]
    if not matches(radius, SUPPORT_RADIUS_M) or not matches(length, expected_length) \
            or not matches(branch_pose, expected_pose):
        errors.append(f"{identifier}: support geometry differs from recorded fruit/stem geometry")
    roll, pitch = branch_pose[3], branch_pose[4]
    vertical_axis = math.cos(pitch) * math.cos(roll)
    vertical_extent = abs(vertical_axis) * length / 2 + radius * math.sqrt(max(0, 1 - vertical_axis**2))
    if pose[2] + branch_pose[2] - vertical_extent <= threshold:
        errors.append(f"{identifier}: SDF support enters preserved scan band")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--field-metadata", type=Path, required=True)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--baseline-metadata", type=Path, required=True)
    parser.add_argument("--field-sdf", type=Path, help="Also verify generated support and sphere SDF geometry.")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        result = audit(json.loads(args.field_metadata.read_text(encoding="utf-8")),
                       yaml.safe_load(args.mission.read_text(encoding="utf-8")),
                       json.loads(args.baseline_metadata.read_text(encoding="utf-8")), args.field_sdf)
    except (OSError, ValueError, TypeError, KeyError, IndexError, ET.ParseError, yaml.YAMLError) as error:
        result = {"passed": False, "errors": [str(error)], "counts": {}, "stations": [],
                  "validation": {"ik": "pending", "full_arm_collision": "pending", "camera_fov": "pending"}}
    result["sources"] = {"field_metadata": str(args.field_metadata), "mission": str(args.mission),
                         "baseline_metadata": str(args.baseline_metadata),
                         "field_sdf": str(args.field_sdf) if args.field_sdf else None}
    rendered = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
