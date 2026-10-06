#!/usr/bin/env python3
"""Generate a lightweight, parameterized Gazebo tomato-field world."""

from __future__ import annotations

import argparse
from pathlib import Path
import random


def parse_args() -> argparse.Namespace:
    package_root = Path(__file__).resolve().parents[1]
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
        "--output",
        type=Path,
        default=package_root / "worlds" / "tomato_field_22x14.sdf",
    )
    args = parser.parse_args()
    if args.rows < 1 or args.plants_per_row < 1:
        parser.error("rows and plants-per-row must both be positive")
    if args.row_spacing <= 0 or args.plant_spacing <= 0:
        parser.error("row-spacing and plant-spacing must both be positive")
    if args.ground_x <= 0 or args.ground_y <= 0:
        parser.error("ground dimensions must both be positive")
    return args


def build_world(args: argparse.Namespace) -> str:
    rng = random.Random(args.seed)
    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.10">',
        f'  <world name="{args.world_name}">',
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

    plant_index = 0
    for row in range(args.rows):
        x = args.origin_x + row * args.row_spacing
        for plant in range(args.plants_per_row):
            y = args.origin_y + plant * args.plant_spacing
            yaw = rng.uniform(-args.yaw_jitter, args.yaw_jitter)
            lines.extend(
                [
                    '    <include>',
                    '      <uri>model://tomato_0</uri>',
                    f'      <name>tomato_{plant_index}</name>',
                    f'      <pose>{x:.6f} {y:.6f} 0 0 0 {yaw:.6f}</pose>',
                    '    </include>',
                ]
            )
            plant_index += 1

    lines.extend(['  </world>', '</sdf>', ''])
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(build_world(args), encoding="utf-8")
    print(
        f"Generated {args.rows * args.plants_per_row} plants in "
        f"{args.output.resolve()}"
    )


if __name__ == "__main__":
    main()
