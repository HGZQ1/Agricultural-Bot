"""Generate a version-bound mission for the two rows beside one aisle."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any

import yaml

from .route_validation import MapGrid, verify_mission_map
from .waypoint_schema import (
    load_mission, normalized_angle, Pose2D, sha256_file)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Generate one logical harvest waypoint per plant for the two '
            'crop rows beside a mapped single corridor'))
    parser.add_argument('--field-id', required=True)
    parser.add_argument('--metadata', required=True, type=Path)
    parser.add_argument('--sdf', required=True, type=Path)
    parser.add_argument('--map-yaml', required=True, type=Path)
    parser.add_argument('--keepout-yaml', required=True, type=Path)
    parser.add_argument('--map-package', default='agri_navigation')
    parser.add_argument(
        '--map-relative-yaml', default='maps/tomato_field_default.yaml')
    parser.add_argument(
        '--keepout-relative-yaml',
        default='maps/tomato_field_default_keepout.yaml')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument(
        '--robot-start', nargs=3, type=float,
        metavar=('WORLD_X', 'WORLD_Y', 'WORLD_YAW'),
        default=(0.0, -6.0, math.pi / 2.0))
    parser.add_argument('--corridor-world-x', type=float, default=0.0)
    parser.add_argument('--home-x', type=float, default=0.5)
    parser.add_argument('--home-y', type=float, default=0.0)
    parser.add_argument('--front-m', type=float, default=0.495)
    parser.add_argument('--rear-m', type=float, default=0.535)
    parser.add_argument('--half-width-m', type=float, default=0.395)
    parser.add_argument('--lateral-search-m', type=float, default=0.40)
    parser.add_argument('--search-step-m', type=float, default=0.01)
    parser.add_argument('--max-unknown-cells', type=int, default=8)
    parser.add_argument('--goal-timeout-s', type=float, default=120.0)
    parser.add_argument('--station-dwell-s', type=float, default=1.0)
    parser.add_argument('--micro-longitudinal-m', type=float, default=0.05)
    parser.add_argument('--micro-lateral-m', type=float, default=0.025)
    parser.add_argument('--micro-yaw-rad', type=float, default=0.025)
    return parser


def world_to_map_transform(
    start: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return map<-world when mapping starts at the robot pose."""
    start_x, start_y, start_yaw = start
    yaw = -start_yaw
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        -(cosine * start_x - sine * start_y),
        -(sine * start_x + cosine * start_y),
        yaw,
    )


def transform_pose(
    x: float,
    y: float,
    yaw: float,
    transform: tuple[float, float, float],
) -> Pose2D:
    """Apply a planar rigid transform to a pose."""
    translation_x, translation_y, rotation = transform
    cosine = math.cos(rotation)
    sine = math.sin(rotation)
    return Pose2D(
        x=translation_x + cosine * x - sine * y,
        y=translation_y + sine * x + cosine * y,
        yaw=normalized_angle(yaw + rotation),
    )


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be numeric') from exc
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


def load_metadata(path: Path) -> dict:
    """Read generator metadata and validate its grid contract."""
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'unable to read metadata {path}: {exc}') from exc
    if (not isinstance(data, dict) or
            not isinstance(data.get('parameters'), dict)):
        raise ValueError('metadata must contain a parameters object')
    if not isinstance(data.get('plants'), list):
        raise ValueError('metadata must contain a plants list')
    parameters = data['parameters']
    rows = int(parameters.get('rows', 0))
    plants_per_row = int(parameters.get('plants_per_row', 0))
    if rows < 2 or plants_per_row < 1:
        raise ValueError(
            'metadata needs at least two rows and one plant per row')
    if len(data['plants']) != rows * plants_per_row:
        raise ValueError('plant count differs from rows*plants_per_row')
    clean = []
    plant_ids = set()
    for index, plant in enumerate(data['plants']):
        if not isinstance(plant, dict):
            raise ValueError(f'plant {index} must be an object')
        plant_id = str(plant.get('id', index))
        if not plant_id or plant_id in plant_ids:
            raise ValueError('plant IDs must be non-empty and unique')
        plant_ids.add(plant_id)
        clean.append({
            'id': plant_id,
            'x': _finite(plant.get('x'), f'plants[{index}].x'),
            'y': _finite(plant.get('y'), f'plants[{index}].y'),
            'yaw': _finite(plant.get('yaw', 0.0),
                           f'plants[{index}].yaw'),
        })
    data['plants'] = clean
    return data


def select_adjacent_rows(
    metadata: dict,
    corridor_world_x: float,
) -> tuple[int, int]:
    """Select the nearest crop row on each side of an aisle centreline."""
    parameters = metadata['parameters']
    count = int(parameters['plants_per_row'])
    rows = int(parameters['rows'])
    row_x = []
    for row in range(rows):
        values = [
            plant['x']
            for plant in metadata['plants'][row * count:(row + 1) * count]
        ]
        row_x.append((sum(values) / len(values), row))
    lower = [item for item in row_x if item[0] < corridor_world_x]
    upper = [item for item in row_x if item[0] > corridor_world_x]
    if not lower or not upper:
        raise ValueError('corridor must have one crop row on each side')
    return max(lower)[1], min(upper)[1]


def _best_stop_pose(
    grid: MapGrid,
    keepout_grid: MapGrid,
    longitudinal_x: float,
    nominal_y: float,
    footprint: tuple[float, float, float],
    lateral_search_m: float,
    search_step_m: float,
    max_unknown_cells: int,
) -> tuple[Pose2D, Any]:
    if lateral_search_m <= 0.0 or search_step_m <= 0.0:
        raise ValueError('lateral search and step must be positive')
    front, rear, half_width = footprint
    occupied = grid.occupied_centres()
    keepout_occupied = keepout_grid.occupied_centres()
    steps = int(math.floor(lateral_search_m / search_step_m))
    candidates = []
    for index in range(-steps, steps + 1):
        y = nominal_y + index * search_step_m
        pose = Pose2D(longitudinal_x, y, 0.0)
        report = grid.footprint_report(
            pose, front, rear, half_width, occupied)
        keepout_report = keepout_grid.footprint_report(
            pose, front, rear, half_width, keepout_occupied)
        if (not report.clear or
                report.unknown_cells > max_unknown_cells or
                report.nearest_occupied_m is None or
                not keepout_report.clear or
                keepout_report.unknown_cells):
            continue
        # Maximise raw obstacle clearance.  Exact ties favour fewer unknown
        # cells and a position closer to the ideal geometric centreline.
        score = (
            round(report.nearest_occupied_m, 9),
            -report.unknown_cells,
            -abs(y - nominal_y),
            -abs(y),
        )
        candidates.append((score, pose, report))
    if not candidates:
        raise ValueError(
            'no collision-free footprint candidate at map '
            f'x={longitudinal_x:.3f}')
    _, pose, report = max(candidates, key=lambda item: item[0])
    return Pose2D(
        round(pose.x, 3), round(pose.y, 3), round(pose.yaw, 6)), report


def build_mission(args: argparse.Namespace) -> dict:
    """Build a serializable mission from field metadata and a saved map."""
    metadata = load_metadata(args.metadata)
    parameters = metadata['parameters']
    grid = MapGrid.from_yaml(args.map_yaml)
    keepout_grid = MapGrid.from_yaml(args.keepout_yaml)
    if (grid.width != keepout_grid.width or
            grid.height != keepout_grid.height or
            not math.isclose(
                grid.resolution, keepout_grid.resolution,
                rel_tol=0.0, abs_tol=1e-12) or
            any(not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9)
                for actual, expected in zip(
                    grid.origin, keepout_grid.origin))):
        raise ValueError('keepout mask geometry does not match the saved map')
    transform = world_to_map_transform(tuple(args.robot_start))
    first_row, second_row = select_adjacent_rows(
        metadata, args.corridor_world_x)
    count = int(parameters['plants_per_row'])
    selected = [first_row, second_row]
    rows = {
        row: metadata['plants'][row * count:(row + 1) * count]
        for row in selected
    }
    mapped = {
        row: [transform_pose(p['x'], p['y'], p['yaw'], transform)
              for p in plants]
        for row, plants in rows.items()
    }
    footprint = (args.front_m, args.rear_m, args.half_width_m)
    stops = []
    for column in range(count):
        first = mapped[first_row][column]
        second = mapped[second_row][column]
        longitudinal_x = (first.x + second.x) / 2.0
        nominal_y = (first.y + second.y) / 2.0
        stops.append(_best_stop_pose(
            grid, keepout_grid, longitudinal_x, nominal_y, footprint,
            args.lateral_search_m, args.search_step_m,
            args.max_unknown_cells))

    waypoints = []
    sequence = 1
    for column, (stop_pose, report) in enumerate(stops):
        for row in selected:
            plant = rows[row][column]
            plant_map = mapped[row][column]
            side = 'left' if plant_map.y > stop_pose.y else 'right'
            waypoint_id = f'harvest_{plant["id"]}'
            waypoints.append({
                'id': waypoint_id,
                'sequence': sequence,
                'stop_id': f'stop_{column:02d}',
                'base_pose': {
                    'x': stop_pose.x, 'y': stop_pose.y,
                    'yaw': stop_pose.yaw,
                },
                'plant': {
                    'id': plant['id'],
                    'row_id': row,
                    'index_in_row': column,
                    'pose_world': {
                        'x': plant['x'], 'y': plant['y'],
                        'yaw': plant['yaw'],
                    },
                    'pose_map': {
                        'x': round(plant_map.x, 6),
                        'y': round(plant_map.y, 6),
                        'yaw': round(plant_map.yaw, 6),
                    },
                },
                'work_side': side,
                'task': 'harvest',
                'scan_pose_id': f'd405_{side}_scan',
                'arm_pose_id': f'cr5_{side}_harvest',
                'micro_adjust': {
                    'longitudinal_m': args.micro_longitudinal_m,
                    'lateral_m': args.micro_lateral_m,
                    'yaw_rad': args.micro_yaw_rad,
                },
                'goal_timeout_s': args.goal_timeout_s,
                'station_dwell_s': args.station_dwell_s,
                'enabled': True,
                'validation': {
                    'map_status': 'offline_footprint_clear',
                    'navigation_status': 'pending_runtime',
                    'ik_status': 'pending_stage7',
                    'fov_status': 'pending_stage7',
                    'unknown_cells': report.unknown_cells,
                    'base_origin_to_nearest_occupied_m': round(
                        report.nearest_occupied_m, 6),
                },
            })
            sequence += 1

    home = Pose2D(args.home_x, args.home_y, 0.0)
    home_report = grid.footprint_report(
        home, *footprint, grid.occupied_centres())
    if (not home_report.clear or
            home_report.unknown_cells > args.max_unknown_cells):
        raise ValueError('home pose is not footprint-clear in the saved map')
    all_y = [pose.y for pose, _ in stops] + [home.y]
    all_x = [pose.x for pose, _ in stops] + [home.x]
    geometry = metadata.get('plant_geometry', {})
    return {
        'schema_version': 1,
        'mission_id': f'{args.field_id}_single_corridor_harvest',
        'description': (
            'Central single-corridor mission; one logical harvest waypoint '
            'per adjacent plant and one shared base stop per left/right '
            'pair.'),
        'work_side_reference': 'base_footprint_at_goal',
        'field': {
            'id': args.field_id,
            'metadata_sha256': sha256_file(args.metadata),
            'sdf_sha256': sha256_file(args.sdf),
            'seed': int(metadata.get('seed', parameters.get('seed', 0))),
            'rows': int(parameters['rows']),
            'plants_per_row': count,
            'row_spacing_m': float(parameters['row_spacing']),
            'plant_spacing_m': float(parameters['plant_spacing']),
            'plant_width_m': float(
                parameters.get('plant_width', geometry.get('width', 1.0))),
            'plant_collision_width_m': float(
                parameters.get('plant_collision_width', 0.06)),
            'selected_rows': selected,
            'excluded_plant_count': len(metadata['plants']) - 2 * count,
        },
        'map': {
            'frame_id': 'map',
            'package': args.map_package,
            'yaml_relative_path': args.map_relative_yaml,
            'image_sha256': sha256_file(grid.image_path),
            'occupancy_sha256': grid.occupancy_sha256(),
            'keepout_yaml_relative_path': args.keepout_relative_yaml,
            'keepout_image_sha256': sha256_file(keepout_grid.image_path),
            'keepout_occupancy_sha256': keepout_grid.occupancy_sha256(),
            'resolution_m': grid.resolution,
            'origin': list(grid.origin),
            'width_px': grid.width,
            'height_px': grid.height,
            'mode': grid.mode,
            'negate': grid.negate,
            'occupied_thresh': grid.occupied_thresh,
            'free_thresh': grid.free_thresh,
        },
        'alignment': {
            'world_to_map': [round(value, 12) for value in transform],
            'mapping_start_world': {
                'x': args.robot_start[0],
                'y': args.robot_start[1],
                'yaw': args.robot_start[2],
            },
        },
        'robot': {
            'base_frame': 'base_footprint',
            'arm_base_longitudinal_offset_m': -0.17998,
            'note': (
                'Arm/scan pose IDs are stage-7 integration contracts; '
                'their IK and camera FOV remain to be validated.'),
        },
        'route': {
            'profile': 'single_corridor_forward_reverse',
            'home': {
                'id': 'home',
                'pose': {'x': home.x, 'y': home.y, 'yaw': home.yaw},
            },
            'return_home': True,
            'corridor': {
                'x_min': round(min(all_x) - 0.10, 3),
                'x_max': round(max(all_x) + 0.20, 3),
                'y_min': round(min(all_y) - 0.05, 3),
                'y_max': round(max(all_y) + 0.05, 3),
                'required_yaw': 0.0,
                'yaw_tolerance': 0.0873,
            },
            'footprint': {
                'front_m': args.front_m,
                'rear_m': args.rear_m,
                'half_width_m': args.half_width_m,
            },
            'max_unknown_cells': args.max_unknown_cells,
            'waypoints': waypoints,
        },
    }


def main(argv=None) -> int:
    """Generate a mission YAML and print its compact identity report."""
    args = _parser().parse_args(argv)
    temporary_path = None
    try:
        mission = build_mission(args)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        text = yaml.safe_dump(
            mission, allow_unicode=True, sort_keys=False, width=100)
        with tempfile.NamedTemporaryFile(
                mode='w', encoding='utf-8', dir=args.output.parent,
                prefix=f'.{args.output.name}.', suffix='.tmp',
                delete=False) as stream:
            stream.write(text)
            temporary_path = Path(stream.name)
        validated = load_mission(temporary_path)
        verify_mission_map(validated, args.map_yaml, args.keepout_yaml)
        os.replace(temporary_path, args.output)
        temporary_path = None
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
    print(json.dumps({
        'output': str(args.output),
        'mission_id': mission['mission_id'],
        'waypoint_count': len(mission['route']['waypoints']),
        'unique_stop_count': len({
            point['stop_id'] for point in mission['route']['waypoints']
        }),
        'selected_rows': mission['field']['selected_rows'],
    }, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
