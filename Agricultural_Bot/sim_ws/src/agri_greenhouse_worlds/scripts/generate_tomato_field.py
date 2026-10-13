#!/usr/bin/env python3
"""Generate a lightweight, parameterized Gazebo tomato-field world."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import quoteattr


FRUIT_PLACEMENT_ATTEMPTS = 1000
HARVEST_SUPPORT_RADIUS = 0.004
STEM_HALF_WIDTH = 0.05
STEM_HEIGHT = 1.2416
# Branch1/Leaf1/Leaf2 in tomato_0/meshes/tomato.dae, meter=1, Z_UP.
# Height is the crown top above z=0; width is max(local X span, local Y span).
PLANT_HEIGHT = 1.298104
PLANT_WIDTH = 0.866695
PLANT_SUBMESHES = {"Branch1", "Leaf1", "Leaf2"}


def package_root() -> Path:
    """Locate resources for source, symlink and regular colcon installations."""
    script = Path(__file__).resolve()
    source_root = script.parents[1]
    if (source_root / "models").is_dir():
        return source_root
    return script.parents[2] / "share" / "agri_greenhouse_worlds"


def plant_geometry(args: argparse.Namespace) -> dict:
    """Resolve plant dimensions without scaling independent fruit targets."""
    height = PLANT_HEIGHT if args.plant_height is None else args.plant_height
    width = PLANT_WIDTH if args.plant_width is None else args.plant_width
    horizontal = width / PLANT_WIDTH
    vertical = height / PLANT_HEIGHT
    # The visual canopy width and the collision footprint are deliberately
    # independent.  A dense visual model can otherwise make the row spacing
    # look correct while its simplified stem box still blocks the chassis.
    collision_width = (
        2 * STEM_HALF_WIDTH * horizontal
        if args.plant_collision_width is None
        else args.plant_collision_width
    )
    collision_height = (
        STEM_HEIGHT * vertical
        if args.plant_collision_height is None
        else args.plant_collision_height
    )
    stem_size = [collision_width, collision_width, collision_height]
    stem_center_z = stem_size[2] / 2
    if (
        not args.randomize_fruits
        and args.plant_height is None
        and args.plant_width is None
        and args.plant_collision_width is None
        and args.plant_collision_height is None
    ):
        # Report the actual untouched legacy include, whose box is half buried.
        stem_size = [0.1, 0.1, 1.0]
        stem_center_z = 0.0
    return {
        "height": height,
        "width": width,
        "mesh_scale": [horizontal, horizontal, vertical],
        "stem_size": stem_size,
        "stem_center_z": stem_center_z,
        "collision_width": stem_size[0],
        "collision_height": stem_size[2],
    }


def validate_args(args: argparse.Namespace) -> None:
    """Reject invalid geometry before creating or overwriting output files."""
    float_names = (
        "row_spacing", "plant_spacing", "origin_x", "origin_y", "ground_x",
        "ground_y", "yaw_jitter", "fruit_height_min", "fruit_height_max",
        "fruit_diameter_min", "fruit_diameter_max", "fruit_radius_min",
        "fruit_radius_max", "ripe_ratio", "fruit_min_clearance",
    )
    for name in float_names:
        if not math.isfinite(getattr(args, name)):
            raise ValueError(f"{name.replace('_', '-')} must be finite")
    # These two options are optional so ``None`` means "derive the collision
    # box from the corresponding visual dimension".  Validate them separately
    # instead of passing None to math.isfinite().
    for name in ("plant_height", "plant_width",
                 "plant_collision_width", "plant_collision_height"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value <= 0):
            raise ValueError(f"{name.replace('_', '-')} must be finite and positive")
    geometry = plant_geometry(args)
    if not all(math.isfinite(value) and value > 0
               for value in geometry["mesh_scale"] + geometry["stem_size"]):
        raise ValueError("plant dimensions overflow or underflow; use practical sizes")
    if args.rows < 1 or args.plants_per_row < 1:
        raise ValueError("rows and plants-per-row must both be positive")
    if args.row_spacing <= 0 or args.plant_spacing <= 0:
        raise ValueError("row-spacing and plant-spacing must both be positive")
    if args.ground_x <= 0 or args.ground_y <= 0:
        raise ValueError("ground dimensions must both be positive")
    # ``origin_*`` are the first plant centers.  Check the complete row/column
    # span before writing anything so a larger row spacing cannot silently put
    # plants outside the ground collision.  The diagonal bound is conservative
    # for a yawed canopy and remains valid for the source model's asymmetric X/Y
    # spans.
    canopy_radius = geometry["width"] / math.sqrt(2)
    min_x = args.origin_x - canopy_radius
    max_x = args.origin_x + (args.rows - 1) * args.row_spacing + canopy_radius
    min_y = args.origin_y - canopy_radius
    max_y = args.origin_y + (args.plants_per_row - 1) * args.plant_spacing + canopy_radius
    if min_x < -args.ground_x / 2 or max_x > args.ground_x / 2:
        raise ValueError(
            "plant rows exceed ground-x; adjust rows/row-spacing/origin-x or enlarge ground-x"
        )
    if min_y < -args.ground_y / 2 or max_y > args.ground_y / 2:
        raise ValueError(
            "plant columns exceed ground-y; adjust plants-per-row/plant-spacing/origin-y "
            "or enlarge ground-y"
        )
    if args.yaw_jitter < 0:
        raise ValueError("yaw-jitter must be nonnegative")
    if args.fruit_count_min < 0 or args.fruit_count_min > args.fruit_count_max:
        raise ValueError("fruit counts must satisfy 0 <= min <= max")
    for name in ("height", "diameter", "radius"):
        lower = getattr(args, f"fruit_{name}_min")
        upper = getattr(args, f"fruit_{name}_max")
        if lower > upper:
            raise ValueError(f"fruit-{name}-min must not exceed fruit-{name}-max")
    if args.fruit_diameter_min <= 0:
        raise ValueError("fruit diameters must be positive")
    if args.fruit_radius_min < 0 or args.fruit_height_min < 0:
        raise ValueError("fruit heights and horizontal radii must be nonnegative")
    if args.fruit_min_clearance < 0:
        raise ValueError("fruit-min-clearance must be nonnegative")
    if not 0 <= args.ripe_ratio <= 1:
        raise ValueError("ripe-ratio must be between 0 and 1")
    if args.randomize_fruits:
        fruit_radius = args.fruit_diameter_max / 2
        if args.fruit_height_min < fruit_radius + args.fruit_min_clearance:
            raise ValueError(
                "fruit-height-min must be at least fruit-diameter-max / 2 + "
                "fruit-min-clearance to keep spheres above ground"
            )
        if args.fruit_radius_min < (
            geometry["stem_size"][0] / 2 + fruit_radius + args.fruit_min_clearance
        ):
            raise ValueError(
                "fruit-radius-min must be at least scaled stem half-width "
                f"({geometry['stem_size'][0] / 2:.6g} m) + "
                "fruit-diameter-max / 2 + fruit-min-clearance"
            )
    if args.harvest_mission is not None:
        if not args.randomize_fruits:
            raise ValueError("harvest-mission requires --randomize-fruits")
        for name, value in vars(args).items():
            if name.startswith("harvest_") and isinstance(value, float):
                if not math.isfinite(value):
                    raise ValueError(f"{name.replace('_', '-')} must be finite")
        if not 0 <= args.harvest_profile_ratio <= 1:
            raise ValueError("harvest-profile-ratio must be between 0 and 1")
        if not 0 <= args.harvest_sector_half_angle <= math.pi / 2:
            raise ValueError("harvest-sector-half-angle must be between 0 and pi / 2")
        for dimension in ("height", "radius", "diameter"):
            lower = getattr(args, f"harvest_{dimension}_min")
            upper = getattr(args, f"harvest_{dimension}_max")
            if lower <= 0 or lower > upper:
                raise ValueError(f"harvest-{dimension} bounds must satisfy 0 < min <= max")
        if args.harvest_arm_reference_height <= 0 or args.harvest_flange_radius <= 0:
            raise ValueError("harvest arm reference height and flange radius must be positive")
        if any(value < 0 for value in (
            args.harvest_shoulder_lateral_offset, args.harvest_tcp_offset,
            args.harvest_pregrasp_distance, args.harvest_parking_xy_allowance,
            args.harvest_parking_yaw_allowance,
        )):
            raise ValueError("harvest shoulder/TCP offsets and pregrasp distance must be nonnegative")
        if args.harvest_height_min < args.harvest_diameter_max / 2 + args.fruit_min_clearance:
            raise ValueError("harvest-height-min must keep fruit spheres above ground")
        if args.harvest_preserve_scan_max_z is not None and args.harvest_preserve_scan_max_z < 0:
            raise ValueError("harvest-preserve-scan-max-z must be nonnegative")
        if args.harvest_parking_yaw_allowance > math.pi:
            raise ValueError("harvest-parking-yaw-allowance must not exceed pi")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a Gazebo Harmonic SDF tomato field."
    )
    parser.add_argument("--rows", type=int, default=10)
    parser.add_argument("--plants-per-row", type=int, default=15)
    parser.add_argument("--row-spacing", type=float, default=2.0)
    parser.add_argument("--plant-spacing", type=float, default=0.7)
    parser.add_argument("--origin-x", type=float, default=-9.0)
    parser.add_argument("--origin-y", type=float, default=-5.0)
    parser.add_argument("--ground-x", type=float, default=22.0)
    parser.add_argument("--ground-y", type=float, default=14.0)
    parser.add_argument("--yaw-jitter", type=float, default=0.12)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--world-name", default="field")
    parser.add_argument(
        "--plant-height", type=float,
        help="Branch/leaf crown-top height above ground (meters). Only scales plant Z; "
             f"omitting this preserves the original {PLANT_HEIGHT:g} m height.",
    )
    parser.add_argument(
        "--plant-width", type=float,
        help="Maximum local X/Y branch/leaf canopy span before yaw (meters). "
             f"Scales plant X/Y independently of height; original {PLANT_WIDTH:g} m.",
    )
    parser.add_argument(
        "--plant-collision-width", type=float,
        help="Square X/Y width of each plant's simplified collision box (meters). "
             "Independent of the visual canopy width; omitted uses the scaled stem width.",
    )
    parser.add_argument(
        "--plant-collision-height", type=float,
        help="Height of each plant's simplified collision box (meters). "
             "Independent of the visual canopy height; omitted uses the scaled stem height.",
    )
    parser.add_argument(
        "--randomize-fruits", action="store_true",
        help="Use the fruit-free tomato_plant asset and add separate static fruit targets.",
    )
    parser.add_argument(
        "--fruit-visual", choices=("mesh", "sphere"), default="mesh",
        help="Fruit appearance: original textured fruit mesh (default), or plain spheres. "
             "Both use conservative sphere collision geometry.",
    )
    parser.add_argument("--fruit-count-min", type=int, default=2)
    parser.add_argument("--fruit-count-max", type=int, default=6)
    parser.add_argument(
        "--fruit-height-min", type=float, default=1.0,
        help="Minimum fruit-center world Z in meters above the ground plane.",
    )
    parser.add_argument(
        "--fruit-height-max", type=float, default=1.7,
        help="Maximum fruit-center world Z in meters above the ground plane.",
    )
    parser.add_argument("--fruit-diameter-min", type=float, default=0.06)
    parser.add_argument("--fruit-diameter-max", type=float, default=0.09)
    parser.add_argument(
        "--fruit-radius-min", type=float, default=0.12,
        help="Minimum horizontal distance from the stem center to a fruit center (meters).",
    )
    parser.add_argument(
        "--fruit-radius-max", type=float, default=0.25,
        help="Maximum horizontal distance from the stem center to a fruit center (meters).",
    )
    parser.add_argument("--ripe-ratio", type=float, default=0.7)
    parser.add_argument("--fruit-min-clearance", type=float, default=0.01)
    parser.add_argument(
        "--harvest-mission", type=Path,
        help="Offline mission YAML used to bias selected plants toward nominal parking poses. "
             "This generates distance candidates only; IK, collisions and FOV remain unvalidated.",
    )
    parser.add_argument("--harvest-profile-ratio", type=float, default=0.85,
                        help="Per-selected-plant biased quota, rounded to nearest count; at least one.")
    parser.add_argument("--harvest-sector-half-angle", type=float, default=0.25)
    parser.add_argument("--harvest-height-min", type=float, default=1.33)
    parser.add_argument("--harvest-height-max", type=float, default=1.42)
    parser.add_argument("--harvest-radius-min", type=float, default=0.68)
    parser.add_argument("--harvest-radius-max", type=float, default=0.74)
    parser.add_argument("--harvest-diameter-min", type=float, default=0.05)
    parser.add_argument("--harvest-diameter-max", type=float, default=0.065)
    parser.add_argument("--harvest-arm-reference-height", type=float, default=0.891556378129253,
                        help="Assumed world Z of J2 shoulder, not a calibrated TCP height.")
    parser.add_argument("--harvest-arm-reference-longitudinal-offset", type=float, default=-0.17998,
                        help="Fallback arm offset from base_footprint; mission robot value takes precedence.")
    parser.add_argument("--harvest-shoulder-lateral-offset", type=float, default=0.08,
                        help="Worst unknown shoulder horizontal offset radius around the J1 axis.")
    parser.add_argument("--harvest-tcp-offset", type=float, default=0.16,
                        help="Assumed flange-to-grasp-center length along the approach direction.")
    parser.add_argument("--harvest-flange-radius", type=float, default=0.86,
                        help="Conservative nominal shoulder-to-flange distance budget; not an IK result.")
    parser.add_argument("--harvest-pregrasp-distance", type=float, default=0.08)
    parser.add_argument("--harvest-parking-xy-allowance", type=float, default=0.10)
    parser.add_argument("--harvest-parking-yaw-allowance", type=float, default=0.08726646,
                        help="Parking yaw uncertainty in radians, propagated through the arm offset.")
    parser.add_argument("--harvest-preserve-scan-max-z", type=float,
                        help="Keep original fruit sphere geometry whose bottom is at/below this world Z. "
                             "New near-side fruit/support visuals must remain above the threshold.")
    parser.add_argument(
        "--metadata", type=Path,
        help="Optional JSON with parameters and world-frame poses of generated fruit targets.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=package_root() / "worlds" / "tomato_field_22x14.sdf",
    )
    args = parser.parse_args(argv)
    try:
        validate_args(args)
        if args.metadata is not None and args.metadata.resolve() == args.output.resolve():
            raise ValueError("metadata and output must be different files")
        if args.harvest_mission is not None and args.harvest_mission.resolve() in (
            args.output.resolve(), args.metadata.resolve() if args.metadata is not None else None,
        ):
            raise ValueError("harvest-mission must differ from output and metadata")
    except ValueError as error:
        parser.error(str(error))
    return args


def plant_poses(args: argparse.Namespace) -> list[dict]:
    """Keep the original plant order and seeded yaw sequence in both modes."""
    rng = random.Random(args.seed)
    plants = []
    for row in range(args.rows):
        x = args.origin_x + row * args.row_spacing
        for plant in range(args.plants_per_row):
            y = args.origin_y + plant * args.plant_spacing
            yaw = rng.uniform(-args.yaw_jitter, args.yaw_jitter)
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError("plant poses overflow; reduce field coordinates or spacing")
            plants.append({
                "id": f"tomato_{len(plants)}",
                "x": float(f"{x:.6f}"),
                "y": float(f"{y:.6f}"),
                "yaw": float(f"{yaw:.6f}"),
            })
    return plants


def plant_template(args: argparse.Namespace) -> ET.Element:
    """Inline a resized copy, keeping all shared model assets untouched."""
    asset = "tomato_plant" if args.randomize_fruits else "tomato_0"
    model = ET.parse(package_root() / "models" / asset / "model.sdf").getroot().find("model")
    geometry = plant_geometry(args)
    for visual in model.findall("link/visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is not None and mesh.findtext("submesh/name") in PLANT_SUBMESHES:
            scale = ET.SubElement(mesh, "scale")
            scale.text = " ".join(f"{value:.17g}" for value in geometry["mesh_scale"])
            ET.SubElement(mesh.find("submesh"), "center").text = "false"
        # The legacy model uses RGB rather than SDF's RGBA diffuse values.
        diffuse = visual.find("material/diffuse")
        if diffuse is not None and len(diffuse.text.split()) == 3:
            diffuse.text += " 1"
    collision = model.find("link/collision")
    collision.set("name", "stem")
    pose = collision.find("pose")
    if pose is None:
        pose = ET.SubElement(collision, "pose")
    pose.text = f"0 0 {geometry['stem_center_z']:.17g} 0 0 0"
    collision.find("geometry/box/size").text = " ".join(
        f"{value:.17g}" for value in geometry["stem_size"]
    )
    return model


def _sample_random_fruits(args: argparse.Namespace) -> list[dict]:
    """Sample collision-free static targets, independently of plant yaw draws."""
    validate_args(args)
    if not args.randomize_fruits:
        return []
    rng = random.Random(args.seed)
    plants = plant_poses(args)
    geometry = plant_geometry(args)
    stem_half_width = geometry["stem_size"][0] / 2
    stem_height = geometry["stem_size"][2]
    fruits: list[dict] = []
    cell_size = max(0.1, args.fruit_diameter_max + args.fruit_min_clearance)
    fruit_cells: dict[tuple[int, int, int], list[dict]] = {}
    stem_cells: dict[tuple[int, int], list[dict]] = {}

    def cell_index(value: float) -> int:
        scaled = value / cell_size
        if not math.isfinite(scaled):
            raise ValueError("fruit spatial index overflows; reduce coordinates or sizes")
        return math.floor(scaled)

    for plant in plants:
        key = (cell_index(plant["x"]), cell_index(plant["y"]))
        stem_cells.setdefault(key, []).append(plant)

    def clears_stems(x: float, y: float, z: float, radius: float) -> bool:
        reach = math.sqrt(2) * stem_half_width + radius + args.fruit_min_clearance
        for ix in range(cell_index(x - reach), cell_index(x + reach) + 1):
            for iy in range(cell_index(y - reach), cell_index(y + reach) + 1):
                for plant in stem_cells.get((ix, iy), ()):
                    dx, dy = x - plant["x"], y - plant["y"]
                    cosine, sine = math.cos(plant["yaw"]), math.sin(plant["yaw"])
                    local_x = cosine * dx + sine * dy
                    local_y = -sine * dx + cosine * dy
                    distance = math.hypot(
                        max(abs(local_x) - stem_half_width, 0),
                        max(abs(local_y) - stem_half_width, 0),
                        max(-z, z - stem_height, 0),
                    )
                    if distance < radius + args.fruit_min_clearance:
                        return False
        return True

    def clears_fruits(x: float, y: float, z: float, radius: float) -> bool:
        key = (cell_index(x), cell_index(y), cell_index(z))
        for ix in range(key[0] - 1, key[0] + 2):
            for iy in range(key[1] - 1, key[1] + 2):
                for iz in range(key[2] - 1, key[2] + 2):
                    for other in fruit_cells.get((ix, iy, iz), ()):
                        distance = math.dist((x, y, z), (other["x"], other["y"], other["z"]))
                        if distance < radius + other["radius"] + args.fruit_min_clearance:
                            return False
        return True

    for plant in plants:
        count = rng.randint(args.fruit_count_min, args.fruit_count_max)
        for index in range(count):
            for _ in range(FRUIT_PLACEMENT_ATTEMPTS):
                radius = rng.uniform(args.fruit_diameter_min, args.fruit_diameter_max) / 2
                z = rng.uniform(args.fruit_height_min, args.fruit_height_max)
                horizontal_radius = rng.uniform(args.fruit_radius_min, args.fruit_radius_max)
                angle = rng.uniform(-math.pi, math.pi) + plant["yaw"]
                x = plant["x"] + horizontal_radius * math.cos(angle)
                y = plant["y"] + horizontal_radius * math.sin(angle)
                if not all(math.isfinite(value) for value in (x, y, z, radius)):
                    raise ValueError("fruit poses overflow; reduce coordinates or sizes")
                if not clears_stems(x, y, z, radius) or not clears_fruits(x, y, z, radius):
                    continue
                fruit = {
                    "id": f"{plant['id']}_fruit_{index}",
                    "plant_id": plant["id"],
                    "x": x, "y": y, "z": z, "radius": radius,
                    "ripe": rng.random() < args.ripe_ratio,
                }
                fruits.append(fruit)
                key = (cell_index(x), cell_index(y), cell_index(z))
                fruit_cells.setdefault(key, []).append(fruit)
                break
            else:
                raise ValueError(
                    f"unable to place fruit {plant['id']}_fruit_{index} after "
                    f"{FRUIT_PLACEMENT_ATTEMPTS} attempts; reduce fruit count, diameter "
                    "or clearance, or widen the height/radius intervals and plant spacing"
                )
    return fruits


def load_harvest_stations(args: argparse.Namespace) -> list[dict]:
    """Read an offline waypoint snapshot without claiming live fruit sensing or IK."""
    import yaml

    try:
        mission = yaml.safe_load(args.harvest_mission.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"unable to read harvest-mission: {error}") from error
    if not isinstance(mission, dict):
        raise ValueError("harvest-mission must contain a YAML mapping")

    def number(value, name):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"harvest-mission {name} must be a finite number")
        return float(value)

    for section in ("alignment", "robot", "route"):
        if not isinstance(mission.get(section, {}), dict):
            raise ValueError(f"harvest-mission {section} must be a mapping")
    alignment = mission.get("alignment", {}).get("world_to_map")
    if not isinstance(alignment, list) or len(alignment) != 3:
        raise ValueError("harvest-mission requires alignment.world_to_map [x, y, yaw]")
    tx, ty, rotation = [number(value, "world_to_map") for value in alignment]
    cosine, sine = math.cos(rotation), math.sin(rotation)
    robot = mission.get("robot", {})
    arm_offset = number(robot.get(
        "arm_base_longitudinal_offset_m", args.harvest_arm_reference_longitudinal_offset,
    ), "robot.arm_base_longitudinal_offset_m")
    entries = mission.get("route", {}).get("waypoints")
    if not isinstance(entries, list):
        raise ValueError("harvest-mission requires route.waypoints")
    plants = {plant["id"]: plant for plant in plant_poses(args)}
    stations, selected = [], set()
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("harvest-mission waypoints must be mappings")
        if not isinstance(entry.get("enabled", True), bool):
            raise ValueError("harvest-mission waypoint enabled must be a boolean")
        if not entry.get("enabled", True) or entry.get("task", "harvest") != "harvest":
            continue
        plant_entry = entry.get("plant", {})
        if not isinstance(plant_entry, dict):
            raise ValueError("harvest-mission waypoint plant must be a mapping")
        plant_id = plant_entry.get("id")
        if plant_id not in plants or plant_id in selected:
            raise ValueError(f"harvest-mission has missing or duplicate plant {plant_id!r}")
        selected.add(plant_id)
        plant = plants[plant_id]
        expected = plant_entry.get("pose_world", {})
        if not isinstance(expected, dict) or any(
            abs(number(expected.get(key), f"{plant_id}.pose_world.{key}") - plant[key]) > 1e-5
            for key in ("x", "y")
        ):
            raise ValueError(f"harvest-mission plant {plant_id} does not match generated field")
        base_pose = entry.get("base_pose", {})
        if not isinstance(base_pose, dict):
            raise ValueError(f"harvest-mission {plant_id} requires base_pose")
        mx, my, map_yaw = [number(base_pose.get(key), f"{plant_id}.base_pose.{key}")
                           for key in ("x", "y", "yaw")]
        # Inverse of map = R(rotation) * world + translation.
        bx = cosine * (mx - tx) + sine * (my - ty)
        by = -sine * (mx - tx) + cosine * (my - ty)
        yaw = map_yaw - rotation
        side = entry.get("work_side")
        lateral = -math.sin(yaw) * (plant["x"] - bx) + math.cos(yaw) * (plant["y"] - by)
        if side not in ("left", "right") or lateral * (1 if side == "left" else -1) <= 0:
            raise ValueError(f"harvest-mission {plant_id} work_side conflicts with base_pose")
        ax, ay = bx + arm_offset * math.cos(yaw), by + arm_offset * math.sin(yaw)
        stem_distance = math.hypot(plant["x"] - ax, plant["y"] - ay)
        if stem_distance <= args.harvest_shoulder_lateral_offset:
            raise ValueError(f"harvest-mission {plant_id} nominal arm reference is too close to stem")
        parking_allowance = args.harvest_parking_xy_allowance + 2 * abs(arm_offset) * math.sin(
            args.harvest_parking_yaw_allowance / 2,
        )
        stations.append({
            "plant_id": plant_id, "waypoint_id": entry.get("id", plant_id), "work_side": side,
            "nominal_base_pose_map": {"x": mx, "y": my, "yaw": map_yaw},
            "nominal_base_pose_world": {"x": bx, "y": by, "yaw": yaw},
            "nominal_arm_reference_world_xyz_m": [ax, ay, args.harvest_arm_reference_height],
            "arm_reference_definition": "nominal_J1_axis_XY_with_assumed_J2_shoulder_world_Z",
            "shoulder_horizontal_uncertainty_radius_m": args.harvest_shoulder_lateral_offset,
            "parking_horizontal_allowance_m": parking_allowance,
            "near_side_sector_center_world_rad": math.atan2(ay - plant["y"], ax - plant["x"]),
        })
    if not stations:
        raise ValueError("harvest-mission has no enabled harvest waypoints")
    return stations


def harvest_distance_candidate(args: argparse.Namespace, station: dict, xyz: tuple) -> dict | None:
    """Bound flange distances for unknown shoulder XY and allowed parking error."""
    reference = station["nominal_arm_reference_world_xyz_m"]
    horizontal = math.hypot(xyz[0] - reference[0], xyz[1] - reference[1])
    if horizontal <= args.harvest_tcp_offset + args.harvest_pregrasp_distance:
        return None
    direction = [(xyz[0] - reference[0]) / horizontal,
                 (xyz[1] - reference[1]) / horizontal, 0.0]
    flange = [xyz[i] - args.harvest_tcp_offset * direction[i] for i in range(3)]
    pregrasp = [xyz[i] - args.harvest_pregrasp_distance * direction[i] for i in range(3)]
    pregrasp_flange = [flange[i] - args.harvest_pregrasp_distance * direction[i] for i in range(3)]

    def worst_distance(point):
        return math.hypot(math.hypot(point[0] - reference[0], point[1] - reference[1])
                          + station["shoulder_horizontal_uncertainty_radius_m"]
                          + station["parking_horizontal_allowance_m"], point[2] - reference[2])

    grasp_worst = worst_distance(flange)
    pregrasp_worst = worst_distance(pregrasp_flange)
    if max(grasp_worst, pregrasp_worst) > args.harvest_flange_radius:
        return None
    return {
        "status": "distance_candidate_pending_ik",
        "waypoint_id": station["waypoint_id"], "work_side": station["work_side"],
        "nominal_arm_reference_world_xyz_m": list(reference),
        "shoulder_horizontal_uncertainty_radius_m": station["shoulder_horizontal_uncertainty_radius_m"],
        "parking_horizontal_allowance_m": station["parking_horizontal_allowance_m"],
        "nominal_approach_unit_world": direction,
        "nominal_grasp_flange_world_xyz_m": flange,
        "nominal_pregrasp_tcp_world_xyz_m": pregrasp,
        "nominal_pregrasp_flange_world_xyz_m": pregrasp_flange,
        "arm_reference_to_fruit_center_m": math.dist(xyz, reference),
        "nominal_grasp_flange_distance_m": math.dist(flange, reference),
        "grasp_flange_worst_distance_m": grasp_worst,
        "pregrasp_flange_worst_distance_m": pregrasp_worst,
        "flange_distance_budget_m": args.harvest_flange_radius,
        "remaining_distance_margin_m": args.harvest_flange_radius - max(
            grasp_worst, pregrasp_worst,
        ),
        "ik_status": "pending", "full_arm_collision_status": "pending", "fov_status": "pending",
    }


def apply_harvest_profile(args: argparse.Namespace, fruits: list[dict]) -> list[dict]:
    """Replace a quota with near-side candidates while preserving untouched RNG output."""
    stations = load_harvest_stations(args)
    plants = plant_poses(args)
    plants_by_id = {plant["id"]: plant for plant in plants}
    replace_ids = set()
    for station in stations:
        members = [fruit for fruit in fruits if fruit["plant_id"] == station["plant_id"]]
        count = max(1, int(len(members) * args.harvest_profile_ratio + 0.5))
        protected = [fruit for fruit in members if args.harvest_preserve_scan_max_z is not None
                     and fruit["z"] - fruit["radius"] <= args.harvest_preserve_scan_max_z]
        protected_ids = {fruit["id"] for fruit in protected}
        replaceable = [fruit for fruit in members if fruit["id"] not in protected_ids]
        station["replace_ids"] = [fruit["id"] for fruit in replaceable[:count]]
        next_index = max((int(fruit["id"].rsplit("_", 1)[1]) for fruit in members), default=-1) + 1
        station["candidate_ids"] = list(station["replace_ids"]) + [
            f"{station['plant_id']}_fruit_{next_index + index}"
            for index in range(count - len(station["replace_ids"]))
        ]
        replace_ids.update(station["replace_ids"])
    selected_ids = {station["plant_id"] for station in stations}
    preserved_fruits = {}
    for fruit in fruits:
        if fruit["plant_id"] in selected_ids and fruit["id"] not in replace_ids:
            fruit = {**fruit, "ripe": False, "harvest_background": {
                "status": "background_unripe",
                "scan_geometry_preserved": args.harvest_preserve_scan_max_z is not None
                and fruit["z"] - fruit["radius"] <= args.harvest_preserve_scan_max_z,
            }}
        preserved_fruits[fruit["id"]] = fruit
    placed = [fruit for fruit in preserved_fruits.values() if fruit["id"] not in replace_ids]
    replacements = {}
    geometry = plant_geometry(args)
    stem_half_width, stem_height = geometry["stem_size"][0] / 2, geometry["stem_size"][2]

    def clears_geometry(xyz, radius):
        x, y, z = xyz
        if z - radius < args.fruit_min_clearance:
            return False
        if args.harvest_preserve_scan_max_z is not None and z - radius <= args.harvest_preserve_scan_max_z:
            return False
        for plant in plants:
            dx, dy = x - plant["x"], y - plant["y"]
            cosine, sine = math.cos(plant["yaw"]), math.sin(plant["yaw"])
            lx, ly = cosine * dx + sine * dy, -sine * dx + cosine * dy
            gap = math.hypot(max(abs(lx) - stem_half_width, 0),
                             max(abs(ly) - stem_half_width, 0), max(-z, z - stem_height, 0))
            if gap < radius + args.fruit_min_clearance:
                return False
        return all(math.dist(xyz, (other["x"], other["y"], other["z"])) >=
                   radius + other["radius"] + args.fruit_min_clearance for other in placed)

    for station in stations:
        plant = plants_by_id[station["plant_id"]]
        # Separate streams prevent profile changes from perturbing unselected plants.
        rng = random.Random(f"{args.seed}:{station['plant_id']}:harvest-profile-v1")
        original_placed_count = len(placed)
        for arrangement in range(30):
            # Retry the complete cluster if an early random placement blocks
            # later fruit. Never silently reduce the required candidate quota.
            del placed[original_placed_count:]
            local_replacements = {}
            for index, fruit_id in enumerate(station["candidate_ids"]):
                for _ in range(FRUIT_PLACEMENT_ATTEMPTS):
                    radius = rng.uniform(args.harvest_diameter_min, args.harvest_diameter_max) / 2
                    z = rng.uniform(args.harvest_height_min, args.harvest_height_max)
                    stem_radius = rng.uniform(args.harvest_radius_min, args.harvest_radius_max)
                    angle = station["near_side_sector_center_world_rad"] + rng.uniform(
                        -args.harvest_sector_half_angle, args.harvest_sector_half_angle,
                    )
                    xyz = (plant["x"] + stem_radius * math.cos(angle),
                           plant["y"] + stem_radius * math.sin(angle), z)
                    candidate = harvest_distance_candidate(args, station, xyz)
                    if candidate is None or not clears_geometry(xyz, radius):
                        continue
                    sampled_ripe = rng.random() < args.ripe_ratio
                    fruit = {"id": fruit_id, "plant_id": plant["id"],
                             "x": xyz[0], "y": xyz[1], "z": xyz[2], "radius": radius,
                             "ripe": True if index == 0 else sampled_ripe,
                             "harvest_profile": {**candidate, "forced_ripe": index == 0}}
                    placed.append(fruit)
                    local_replacements[fruit_id] = fruit
                    break
                else:
                    break
            else:
                replacements.update(local_replacements)
                break
        else:
            raise ValueError(
                f"unable to place harvest distance candidates for {plant['id']} after "
                f"30 cluster retries of {FRUIT_PLACEMENT_ATTEMPTS} attempts per fruit; "
                "revise the mission/near-side ranges or nominal geometry budget; "
                "IK remains unvalidated"
            )
    result = [replacements.get(fruit["id"], preserved_fruits[fruit["id"]]) for fruit in fruits]
    result.extend(replacements[fruit_id] for station in stations for fruit_id in station["candidate_ids"]
                  if fruit_id not in preserved_fruits)
    return result


def sample_fruits(args: argparse.Namespace) -> list[dict]:
    fruits = _sample_random_fruits(args)
    if args.harvest_mission is not None:
        return apply_harvest_profile(args, fruits)
    return fruits


def harvest_profile_metadata(args: argparse.Namespace, fruits: list[dict]) -> dict:
    stations = load_harvest_stations(args)
    legacy = _sample_random_fruits(args)
    for station in stations:
        candidates = [fruit for fruit in fruits if fruit["plant_id"] == station["plant_id"]
                      and "harvest_profile" in fruit]
        station["distance_candidate_ids"] = [fruit["id"] for fruit in candidates]
        station["ripe_distance_candidate_ids"] = [fruit["id"] for fruit in candidates if fruit["ripe"]]
        original_members = [fruit for fruit in legacy if fruit["plant_id"] == station["plant_id"]]
        current_members = [fruit for fruit in fruits if fruit["plant_id"] == station["plant_id"]]
        protected = [fruit for fruit in original_members if args.harvest_preserve_scan_max_z is not None
                     and fruit["z"] - fruit["radius"] <= args.harvest_preserve_scan_max_z]
        station.update({
            "original_fruit_count": len(original_members),
            "protected_fruit_ids": [fruit["id"] for fruit in protected],
            "protected_fruit_count": len(protected),
            "near_fruit_count": len(candidates), "total_fruit_count": len(current_members),
            "effective_near_fraction": len(candidates) / len(current_members),
            "quota_basis": "round(original_fruit_count * profile_ratio), at least one",
        })
    return {
        "status": "distance_candidate_pending_ik", "usage": "offline_scene_generation_only",
        "mission_sha256_at_generation": hashlib.sha256(args.harvest_mission.read_bytes()).hexdigest(),
        "assumptions": {
            "ideal_nominal_parking_pose": True,
            "shoulder_XY_worst_case_radius_around_J1_included": True,
            "parking_XY_and_yaw_error_allowance_included": True,
            "flange_to_TCP_aligned_with_horizontal_J1_to_fruit": True,
            "TCP_is_unvalidated_proxy": True,
            "IK_full_arm_collision_FOV_pending": True,
            "actual_parking_accuracy_pending_runtime": True,
        },
        "stations": stations,
    }


def build_world(args: argparse.Namespace, fruits: list[dict] | None = None) -> str:
    validate_args(args)
    if fruits is None:
        fruits = sample_fruits(args)
    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.10">',
        f'  <world name={quoteattr(args.world_name)}>',
        '    <gravity>0 0 -9.81</gravity>',
        '    <physics name="default_physics" type="ode">',
        '      <max_step_size>0.001</max_step_size>',
        '      <real_time_factor>1.0</real_time_factor>',
        '      <real_time_update_rate>1000</real_time_update_rate>',
        '    </physics>',
        '    <plugin filename="gz-sim-physics-system"',
        '            name="gz::sim::systems::Physics"/>',
        '    <plugin filename="gz-sim-user-commands-system"',
        '            name="gz::sim::systems::UserCommands"/>',
        '    <plugin filename="gz-sim-scene-broadcaster-system"',
        '            name="gz::sim::systems::SceneBroadcaster"/>',
        '    <light name="sun" type="directional">',
        '      <cast_shadows>true</cast_shadows>',
        '      <pose>0 0 10 0 0 0</pose>',
        '      <diffuse>0.8 0.8 0.8 1</diffuse>',
        '      <specular>0.2 0.2 0.2 1</specular>',
        '      <direction>-0.5 0.1 -0.9</direction>',
        '    </light>',
        '    <model name="ground_plane">',
        '      <static>true</static>',
        '      <pose>0 0 -0.025 0 0 0</pose>',
        '      <link name="ground_link">',
        '        <collision name="collision">',
        '          <geometry><box>',
        f'            <size>{args.ground_x:g} {args.ground_y:g} 0.05</size>',
        '          </box></geometry>',
        '        </collision>',
        '        <visual name="visual">',
        '          <geometry><box>',
        f'            <size>{args.ground_x:g} {args.ground_y:g} 0.05</size>',
        '          </box></geometry>',
        '          <material>',
        '            <ambient>0.45 0.29 0.07 1</ambient>',
        '            <diffuse>0.45 0.29 0.07 1</diffuse>',
        '          </material>',
        '        </visual>',
        '      </link>',
        '    </model>',
    ]

    plant_uri = "tomato_plant" if args.randomize_fruits else "tomato_0"
    template = (
        plant_template(args)
        if (
            args.plant_height is not None
            or args.plant_width is not None
            or args.plant_collision_width is not None
            or args.plant_collision_height is not None
        )
        else None
    )
    for plant in plant_poses(args):
        if template is not None:
            model = copy.deepcopy(template)
            model.set("name", plant["id"])
            ET.SubElement(model, "pose").text = (
                f'{plant["x"]:.6f} {plant["y"]:.6f} 0 0 0 {plant["yaw"]:.6f}'
            )
            ET.indent(model, space="  ", level=2)
            lines.extend(("    " + ET.tostring(model, encoding="unicode")).splitlines())
            continue
        lines.extend([
            '    <include>',
            f'      <uri>model://{plant_uri}</uri>',
            f'      <name>{plant["id"]}</name>',
            f'      <pose>{plant["x"]:.6f} {plant["y"]:.6f} 0 0 0 {plant["yaw"]:.6f}</pose>',
            '    </include>',
        ])

    plants_by_id = {plant["id"]: plant for plant in plant_poses(args)}
    for fruit in fruits:
        color = "0.8 0.04 0.025 1" if fruit["ripe"] else "0.12 0.5 0.06 1"
        radius = f'{fruit["radius"]:.17g}'
        pose = " ".join(f'{fruit[key]:.17g}' for key in ("x", "y", "z"))
        lines.extend([
            f'    <model name="{fruit["id"]}">',
            '      <static>true</static>',
            f'      <pose>{pose} 0 0 0</pose>',
            '      <link name="fruit_link">',
            '        <collision name="fruit_collision">',
            f'          <geometry><sphere><radius>{radius}</radius></sphere></geometry>',
            '        </collision>',
            '        <visual name="fruit_visual">',
        ])
        if args.fruit_visual == "mesh":
            # The extracted fruit is centered and bounded by a radius-0.5 sphere.
            # A uniform diameter scale preserves its original shape and guarantees
            # that the existing collision/placement sphere encloses its vertices.
            diameter = f'{2 * fruit["radius"]:.17g}'
            texture = "AG15frt1.png" if fruit["ripe"] else "AG15frt4.png"
            lines.extend([
                '          <geometry><mesh>',
                '            <uri>model://tomato_fruit/meshes/fruit.dae</uri>',
                f'            <scale>{diameter} {diameter} {diameter}</scale>',
                '          </mesh></geometry>',
                '          <material>',
                '            <ambient>1 1 1 1</ambient>',
                '            <diffuse>1 1 1 1</diffuse>',
                '            <pbr><metal>',
                f'              <albedo_map>model://tomato_0/materials/textures/{texture}</albedo_map>',
                '              <metalness>0</metalness>',
                '              <roughness>0.4</roughness>',
                '            </metal></pbr>',
                '          </material>',
            ])
        else:
            lines.extend([
                f'          <geometry><sphere><radius>{radius}</radius></sphere></geometry>',
                '          <material>',
                f'            <ambient>{color}</ambient>',
                f'            <diffuse>{color}</diffuse>',
                '          </material>',
            ])
        lines.append('        </visual>')
        if "harvest_profile" in fruit:
            plant = plants_by_id[fruit["plant_id"]]
            dx, dy = fruit["x"] - plant["x"], fruit["y"] - plant["y"]
            stem_distance = math.hypot(dx, dy)
            length = stem_distance - fruit["radius"]
            # Cylinder local Z points horizontally from the stem to the fruit
            # surface. The branch shares the fruit's height and has no collision.
            center_x = -dx + length * dx / stem_distance / 2
            center_y = -dy + length * dy / stem_distance / 2
            lines.extend([
                '        <visual name="harvest_support_visual">',
                f'          <pose>{center_x:.17g} {center_y:.17g} 0 0 {math.pi / 2:.17g} {math.atan2(dy, dx):.17g}</pose>',
                '          <geometry><cylinder>',
                f'            <radius>{HARVEST_SUPPORT_RADIUS:.17g}</radius>',
                f'            <length>{length:.17g}</length>',
                '          </cylinder></geometry>',
                '          <material>',
                '            <ambient>0.12 0.35 0.04 1</ambient>',
                '            <diffuse>0.12 0.35 0.04 1</diffuse>',
                '          </material>',
                '        </visual>',
            ])
        lines.extend(['      </link>', '    </model>'])

    lines.extend(['  </world>', '</sdf>', ''])
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    try:
        fruits = sample_fruits(args)
        world = build_world(args, fruits)
        metadata_data = {
            "schema_version": 2 if args.harvest_mission is not None else 1,
            "coordinate_frame": "world",
            "seed": args.seed,
            "parameters": {
                key: str(value) if isinstance(value, Path) else value
                for key, value in vars(args).items()
                if key not in ("output", "metadata", "seed")
                and (args.harvest_mission is not None or not key.startswith("harvest_"))
            },
            "plants": plant_poses(args),
            "plant_geometry": plant_geometry(args),
            "fruits": fruits,
        }
        if args.harvest_mission is not None:
            metadata_data["harvest_profile"] = harvest_profile_metadata(args, fruits)
        metadata = json.dumps(metadata_data, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2) from error
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(world, encoding="utf-8")
    if args.metadata is not None:
        args.metadata.parent.mkdir(parents=True, exist_ok=True)
        args.metadata.write_text(metadata, encoding="utf-8")
    print(
        f"Generated {args.rows * args.plants_per_row} plants in "
        f"{args.output.resolve()}"
    )
    if args.randomize_fruits:
        print(f"Generated {len(fruits)} static fruit targets with seed {args.seed}")


if __name__ == "__main__":
    main()
