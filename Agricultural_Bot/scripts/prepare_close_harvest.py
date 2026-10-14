#!/usr/bin/env python3
"""Bind close-side harvest missions to an independently prepared collision map."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare(directory, side_offset, first_station_only=True):
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / 'ros2_ws/src/agri_task_manager'))
    from agri_task_manager.route_validation import MapGrid, verify_mission_map
    from agri_task_manager.waypoint_schema import load_mission, Pose2D

    source = root / 'artifacts/harvest_demo_profile_20261013/previous_default'
    original = source / 'tomato_field_default_single_corridor.yaml'
    data = yaml.safe_load(original.read_text())
    for name, key in [('tomato_field_default.sdf', 'sdf_sha256'),
                      ('tomato_field_default.json', 'metadata_sha256')]:
        if digest(source / name) != data['field'][key]:
            raise ValueError('original mission does not bind the natural source scene')
    directory = directory.resolve()
    sdf = directory / 'tomato_field_collision_navigation.sdf'
    metadata = directory / 'tomato_field.json'
    nav_map = directory / 'navigation_collision.yaml'
    if digest(metadata) != data['field']['metadata_sha256']:
        raise ValueError('natural fruit metadata was changed during preparation')
    grid = MapGrid.from_yaml(nav_map)
    keepout = root / 'ros2_ws/src/agri_navigation' / data['map']['keepout_yaml_relative_path']
    keepout_grid = MapGrid.from_yaml(keepout)
    occupied, keepout_occupied = grid.occupied_centres(), keepout_grid.occupied_centres()
    data['mission_id'] += '_natural_close'
    data['description'] = (
        'Natural fruit geometry, independent close-side stops and simulation-only '
        'physical-collision navigation map. Runtime vision/IK/grip eligibility is pending.')
    data['field']['sdf_sha256'] = digest(sdf)
    data['map']['yaml_relative_path'] = str(nav_map)
    data['map']['image_sha256'] = digest(grid.image_path)
    data['map']['occupancy_sha256'] = grid.occupancy_sha256()
    data['robot']['note'] = (
        'Original fruit positions/sizes/maturity; individual perception, aperture, '
        'collision-aware IK and physical grip must pass. No per-plant success guarantee.')
    route = data['route']
    route['corridor']['y_min'] = -side_offset - .05
    route['corridor']['y_max'] = side_offset + .05
    route['generation']['side_offset_m'] = side_offset
    route['generation']['min_obstacle_clearance_m'] = None
    route['generation']['validation_basis'] = 'full padded footprint and unchanged keepout'
    footprint = route['footprint']
    dimensions = (footprint['front_m'], footprint['rear_m'], footprint['half_width_m'])
    if first_station_only:
        for index, point in enumerate(route['waypoints']):
            point['enabled'] = index == 0
        route['transits'] = []
        route['return_home'] = False
    for point in route['waypoints']:
        if not point['enabled']:
            point['validation'] = {'map_status': 'not_validated_in_close_profile',
                                   'navigation_status': 'disabled', 'ik_status': 'pending_runtime',
                                   'fov_status': 'pending_runtime'}
            continue
        sign = 1 if point['work_side'] == 'left' else -1
        adjustment = point['micro_adjust']
        # Known obstacles outside the exact foliage envelope remain in the map.
        # Retreat individual stops if those cells constrain their full envelope.
        for step in range(26):
            y = sign * round(side_offset - step * .01, 3)
            clear = True
            for dx in (-adjustment['longitudinal_m'], 0, adjustment['longitudinal_m']):
                for dy in (-adjustment['lateral_m'], 0, adjustment['lateral_m']):
                    for yaw in (-adjustment['yaw_rad'], 0, adjustment['yaw_rad']):
                        pose = Pose2D(point['base_pose']['x'] + dx, y + dy, yaw)
                        report = grid.footprint_report(pose, *dimensions, occupied)
                        mask_report = keepout_grid.footprint_report(pose, *dimensions, keepout_occupied)
                        if (not report.clear or report.unknown_cells > route['max_unknown_cells'] or
                                not mask_report.clear or mask_report.unknown_cells):
                            clear = False
            if clear:
                point['base_pose']['y'] = y
                break
        else:
            raise ValueError(f"no close stop with a clear micro-adjust envelope: {point['id']}")
        point['validation'] = {
            'map_status': 'pending_offline_footprint_check',
            'navigation_status': 'pending_runtime', 'ik_status': 'pending_runtime',
            'fov_status': 'pending_runtime',
        }
    for transit in route['transits']:
        if transit['pose']['y']:
            transit['pose']['y'] = side_offset if transit['pose']['y'] > 0 else -side_offset
    static_mission = directory / 'close_mission_static.yaml'
    if static_mission.is_symlink() or static_mission.resolve() == original.resolve():
        raise ValueError('mission output must be independent and not a symlink')
    # Failed validation must not replace a previously valid mission/manifest.
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', dir=directory,
                                     delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(yaml.safe_dump(data, sort_keys=False))
    try:
        mission = load_mission(temporary)
        _, reports = verify_mission_map(mission, nav_map, keepout)
    finally:
        temporary.unlink(missing_ok=True)
    # Preserve the validator's real evidence rather than old clearance values.
    for point in data['route']['waypoints']:
        if not point['enabled']:
            continue
        report = reports[point['stop_id']]
        point['validation'].update({
            'map_status': 'offline_footprint_clear',
            'unknown_cells': report.unknown_cells,
            'base_origin_to_nearest_occupied_m': report.nearest_occupied_m,
        })
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', dir=directory,
                                     delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(yaml.safe_dump(data, sort_keys=False))
    temporary.replace(static_mission)
    output = directory / 'harvestable'
    subprocess.run([
        sys.executable, str(root / 'scripts/prepare_harvest_demo.py'),
        '--output-dir', str(output), '--source-sdf', str(sdf),
        '--metadata', str(metadata), '--mission', str(static_mission),
    ], check=True, stdout=subprocess.DEVNULL)
    result = {
        'scope': 'candidate; original defaults and development-log dates untouched',
        'localization_map': str(root / 'ros2_ws/src/agri_navigation/maps/tomato_field_default.yaml'),
        'navigation_map': str(nav_map), 'route_map_topic': '/navigation_map',
        'side_offset_m': side_offset, 'stem_standoff_m': 1.5 - side_offset,
        'padded_footprint_and_micro_adjust_and_route_verified': True,
        'runtime_navigation_and_harvest_passed': False,
        'natural_metadata_sha256': digest(metadata),
        'harvest_directory': str(output), 'station_count': len(route['waypoints']),
        'enabled_station_count': sum(point['enabled'] for point in route['waypoints']),
        'full_route_validated': not first_station_only,
        'actual_side_offsets_m': [abs(point['base_pose']['y']) for point in route['waypoints']],
        'keepout_yaml': str(keepout),
    }
    (directory / 'close_mission_manifest.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    return result


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=root / 'artifacts/close_harvest_20261014')
    parser.add_argument('--side-offset-m', type=float, default=.75)
    parser.add_argument('--first-station-only', action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    if not 0 < args.side_offset_m <= .75:
        parser.error('side offset must be in (0, 0.75] m for the existing padded footprint/keepout')
    print(json.dumps(prepare(args.directory, args.side_offset_m, args.first_station_only),
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
