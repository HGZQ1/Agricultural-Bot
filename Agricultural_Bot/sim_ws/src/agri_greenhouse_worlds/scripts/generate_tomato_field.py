#!/usr/bin/env python3
"""Generate a lightweight, parameterized Gazebo tomato-field world."""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import quoteattr


FRUIT_PLACEMENT_ATTEMPTS = 1000
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
        "--fruit-height-min", type=float, default=0.6,
        help="Minimum fruit-center world Z in meters above the ground plane.",
    )
    parser.add_argument(
        "--fruit-height-max", type=float, default=1.1,
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


def sample_fruits(args: argparse.Namespace) -> list[dict]:
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
        lines.extend([
            '        </visual>',
            '      </link>',
            '    </model>',
        ])

    lines.extend(['  </world>', '</sdf>', ''])
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    try:
        fruits = sample_fruits(args)
        world = build_world(args, fruits)
        metadata = json.dumps({
            "schema_version": 1,
            "coordinate_frame": "world",
            "seed": args.seed,
            "parameters": {
                key: value for key, value in vars(args).items()
                if key not in ("output", "metadata", "seed")
            },
            "plants": plant_poses(args),
            "plant_geometry": plant_geometry(args),
            "fruits": fruits,
        }, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
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
