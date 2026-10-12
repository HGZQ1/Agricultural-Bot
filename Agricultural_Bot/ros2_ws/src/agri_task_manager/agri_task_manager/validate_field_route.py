"""Command-line validation for a mission and its bound occupancy map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory

from .route_validation import verify_mission_map
from .waypoint_schema import load_mission, MissionConfigError, sha256_file


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Validate a harvest mission against its saved map')
    parser.add_argument('mission', type=Path)
    parser.add_argument(
        '--map-yaml', type=Path,
        help='override the package-relative map YAML stored in the mission')
    parser.add_argument(
        '--keepout-yaml', type=Path,
        help=(
            'override the package-relative keepout YAML stored in the '
            'mission'))
    parser.add_argument(
        '--field-metadata', type=Path,
        help='optionally verify the field metadata SHA-256')
    parser.add_argument(
        '--field-sdf', type=Path,
        help='optionally verify the field SDF SHA-256')
    return parser


def validate(
    mission_path: Path,
    map_yaml: Path | None = None,
    keepout_yaml: Path | None = None,
    field_metadata: Path | None = None,
    field_sdf: Path | None = None,
) -> dict:
    """Return a JSON-serializable validation report."""
    mission = load_mission(mission_path)
    if map_yaml is None:
        share = Path(get_package_share_directory(mission.map_binding.package))
        map_yaml = share / mission.map_binding.yaml_relative_path
    if keepout_yaml is None:
        share = Path(get_package_share_directory(mission.map_binding.package))
        keepout_yaml = share / mission.map_binding.keepout_yaml_relative_path
    if ((field_metadata is None) != (field_sdf is None)):
        raise MissionConfigError(
            '--field-metadata and --field-sdf must be supplied together')
    if field_metadata is not None:
        try:
            if sha256_file(field_metadata) != mission.field.metadata_sha256:
                raise MissionConfigError(
                    'field metadata SHA-256 does not match mission binding')
            if sha256_file(field_sdf) != mission.field.sdf_sha256:
                raise MissionConfigError(
                    'field SDF SHA-256 does not match mission binding')
        except OSError as exc:
            raise MissionConfigError(
                f'unable to read bound field asset: {exc}') from exc
    grid, reports = verify_mission_map(mission, map_yaml, keepout_yaml)
    return {
        'passed': True,
        'mission_id': mission.mission_id,
        'field_id': mission.field.field_id,
        'map_yaml': str(grid.yaml_path),
        'map_image_sha256': mission.map_binding.image_sha256,
        'occupancy_sha256': mission.map_binding.occupancy_sha256,
        'keepout_image_sha256': mission.map_binding.keepout_image_sha256,
        'keepout_occupancy_sha256': (
            mission.map_binding.keepout_occupancy_sha256),
        'waypoint_count': len(mission.enabled_waypoints),
        'unique_stop_count': len({
            point.stop_id for point in mission.enabled_waypoints}),
        'transit_count': len(mission.route_transits),
        'selected_rows': list(mission.field.selected_rows),
        'excluded_plant_count': mission.field.excluded_plant_count,
        'field_assets_verified': field_metadata is not None,
        'stops': {
            stop_id: {
                'free_cells': report.free_cells,
                'unknown_cells': report.unknown_cells,
                'occupied_cells': report.occupied_cells,
                'outside_cells': report.outside_cells,
                'nearest_occupied_m': report.nearest_occupied_m,
            }
            for stop_id, report in reports.items()
        },
    }


def main(argv=None) -> int:
    """Validate CLI inputs and print one machine-readable report."""
    args = _parser().parse_args(argv)
    try:
        report = validate(
            args.mission,
            args.map_yaml,
            args.keepout_yaml,
            args.field_metadata,
            args.field_sdf,
        )
    except MissionConfigError as exc:
        print(json.dumps(
            {'passed': False, 'error': str(exc)}, ensure_ascii=False))
        return 1
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
