"""Reproducibility tests for field-to-map waypoint generation."""

from argparse import Namespace
from pathlib import Path

from agri_task_manager.generate_single_corridor_mission import build_mission

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = Path(__file__).resolve().parents[4]
FIELD_ID = 'rows6_n10_row2p0_plant0p7_h2p4_w1p0_c0p06_fh1p0p1p7_seed42'
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
    assert route['field']['metadata_sha256'].startswith('c4190f9c')
    assert route['field']['sdf_sha256'].startswith('c5b6227a')
    assert route['map']['image_sha256'].startswith('0efabc59')
    assert route['map']['occupancy_sha256'].startswith('92367b58')
    assert route['map']['keepout_image_sha256'].startswith('b5c3dd2f')
    assert route['map']['keepout_occupancy_sha256'].startswith('536673b7')
