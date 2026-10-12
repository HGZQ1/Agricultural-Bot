"""Reproducibility tests for field-to-map waypoint generation."""

from argparse import Namespace
from pathlib import Path

from agri_task_manager.generate_single_corridor_mission import build_mission
from agri_task_manager.route_validation import MapGrid
from agri_task_manager.waypoint_schema import sha256_file

import pytest

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = Path(__file__).resolve().parents[4]
FIELD_ID = 'rows6_n10_row3p0_plant0p7_h2p4_w1p0_c0p06_fh1p0p1p7_seed42'
FIELD_ROOT = (
    PROJECT_ROOT / 'sim_ws' / 'src' / 'agri_greenhouse_worlds' / 'worlds')
MAP_FILE = (
    PROJECT_ROOT / 'ros2_ws' / 'src' / 'agri_navigation' / 'maps'
    / 'tomato_field_default.yaml')
KEEPOUT_FILE = (
    PROJECT_ROOT / 'ros2_ws' / 'src' / 'agri_navigation' / 'maps'
    / 'tomato_field_default_keepout.yaml')
CHECKED_IN = (
    PACKAGE_ROOT / 'config' / 'missions'
    / 'tomato_field_default_single_corridor.yaml')


def _arguments() -> Namespace:
    return Namespace(
        field_id=FIELD_ID,
        metadata=FIELD_ROOT / 'tomato_field_default.json',
        sdf=FIELD_ROOT / 'tomato_field_default.sdf',
        map_yaml=MAP_FILE,
        keepout_yaml=KEEPOUT_FILE,
        map_package='agri_navigation',
        map_relative_yaml='maps/tomato_field_default.yaml',
        keepout_relative_yaml='maps/tomato_field_default_keepout.yaml',
        output=Path('/unused'),
        robot_start=(0.0, -6.0, 1.5707963267948966),
        corridor_world_x=0.0,
        home_x=0.5,
        home_y=0.0,
        front_m=0.495,
        rear_m=0.535,
        half_width_m=0.395,
        lateral_search_m=0.40,
        layout='independent_side_stops',
        side_offset_m=0.40,
        side_search_m=0.25,
        min_obstacle_clearance_m=0.80,
        arm_longitudinal_offset_m=-0.17998,
        headland_forward_m=1.02,
        headland_cross_forward_m=0.50,
        search_step_m=0.01,
        max_unknown_cells=8,
        goal_timeout_s=120.0,
        station_dwell_s=1.0,
        micro_longitudinal_m=0.05,
        micro_lateral_m=0.025,
        micro_yaw_rad=0.025,
    )


def test_checked_in_route_is_reproducible_from_bound_assets():
    """Regenerate exactly the checked-in default mission."""
    generated = build_mission(_arguments())
    checked_in = yaml.safe_load(CHECKED_IN.read_text(encoding='utf-8'))
    assert generated == checked_in


def test_generator_records_world_to_map_alignment_and_version_hashes():
    """Bind the field, map, mask, and alignment identities."""
    route = build_mission(_arguments())
    assert route['alignment']['world_to_map'] == [
        6.0, 0.0, -1.570796326795,
    ]
    assert route['field']['metadata_sha256'].startswith('ab20a6cf')
    assert route['field']['sdf_sha256'].startswith('7823a30c')
    for source, image_key, occupancy_key in (
            (MAP_FILE, 'image_sha256', 'occupancy_sha256'),
            (KEEPOUT_FILE, 'keepout_image_sha256',
             'keepout_occupancy_sha256')):
        grid = MapGrid.from_yaml(source)
        assert route['map'][image_key] == sha256_file(grid.image_path)
        assert route['map'][occupancy_key] == grid.occupancy_sha256()


def test_side_stop_search_preserves_work_side_and_clearance():
    """Keep twenty distinct work stops aligned to their own plant."""
    route = build_mission(_arguments())
    points = route['route']['waypoints']
    assert len({point['stop_id'] for point in points}) == 20
    assert [p['work_side'] for p in points] == ['left'] * 10 + ['right'] * 10
    for point in points:
        pose = point['base_pose']
        assert pose['x'] == pytest.approx(point['plant']['pose_map']['x'] + .18)
        assert .15 <= abs(pose['y']) <= .40
        assert (pose['y'] > 0) == (point['work_side'] == 'left')
        assert point['validation'][
            'base_origin_to_nearest_occupied_m'] >= .80
    assert len(route['route']['transits']) == 3


def test_generator_refuses_side_stops_without_required_clearance():
    """Do not silently fall back to the middle when the request is unsafe."""
    args = _arguments()
    args.side_offset_m = 1.2
    with pytest.raises(ValueError, match='no collision-free footprint'):
        build_mission(args)
