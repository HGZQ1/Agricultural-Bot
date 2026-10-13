"""Contracts for the checked-in single-corridor harvest mission."""

from pathlib import Path

from agri_task_manager.route_validation import verify_mission_map
from agri_task_manager.validate_field_route import validate
from agri_task_manager.waypoint_schema import load_mission, MissionConfigError

import pytest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = Path(__file__).resolve().parents[4]
MISSION_FILE = (
    PACKAGE_ROOT / 'config' / 'missions'
    / 'tomato_field_default_single_corridor.yaml')
MAP_FILE = (
    PROJECT_ROOT / 'ros2_ws' / 'src' / 'agri_navigation' / 'maps'
    / 'tomato_field_default.yaml')
KEEPOUT_FILE = (
    PROJECT_ROOT / 'ros2_ws' / 'src' / 'agri_navigation' / 'maps'
    / 'tomato_field_default_keepout.yaml')


def test_default_mission_has_one_task_per_adjacent_plant():
    """Cover every plant in the selected adjacent rows once."""
    mission = load_mission(MISSION_FILE)
    assert mission.field.selected_rows == (2, 3)
    assert mission.field.excluded_plant_count == 40
    assert len(mission.enabled_waypoints) == 20
    assert len({point.plant.plant_id for point in mission.waypoints}) == 20
    assert len({point.stop_id for point in mission.waypoints}) == 20
    assert mission.route_layout == 'independent_side_stops'
    assert {point.work_side for point in mission.waypoints} == {
        'left', 'right',
    }
    assert all(point.pose.yaw == 0.0 for point in mission.waypoints)
    assert mission.home == mission.home.__class__(0.5, 0.0, 0.0)


def test_each_plant_has_its_own_work_side_base_stop():
    """Visit the left row outbound and right row on the reverse return."""
    mission = load_mission(MISSION_FILE)
    assert [p.plant.index_in_row for p in mission.waypoints[:10]] == list(
        range(10))
    assert [p.plant.index_in_row for p in mission.waypoints[10:]] == list(
        reversed(range(10)))
    assert all(p.work_side == 'left' and p.pose.y > 0
               for p in mission.waypoints[:10])
    assert all(p.work_side == 'right' and p.pose.y < 0
               for p in mission.waypoints[10:])
    assert len(mission.route_transits) == 3


def test_cli_report_separates_harvest_stops_from_navigation_transits():
    """Do not count navigation-only anchors as extra harvesting stations."""
    report = validate(MISSION_FILE, MAP_FILE, KEEPOUT_FILE)
    assert report['unique_stop_count'] == report['waypoint_count'] == 20
    assert report['transit_count'] == 3
    assert len(report['stops']) == 23


def test_route_follows_saved_free_space_and_validates_against_map():
    """Follow the mapped corridor and keep each footprint clear."""
    mission = load_mission(MISSION_FILE)
    _, reports = verify_mission_map(mission, MAP_FILE, KEEPOUT_FILE)
    stop_poses = {}
    for point in mission.waypoints:
        stop_poses.setdefault(point.stop_id, point.pose)
    assert stop_poses['stop_00_left'].y == pytest.approx(0.26)
    assert stop_poses['stop_00_right'].y == pytest.approx(-0.23)
    assert all(report.occupied_cells == 0 for report in reports.values())
    assert all(report.outside_cells == 0 for report in reports.values())
    assert max(report.unknown_cells for report in reports.values()) <= 1


def _write_mutated(tmp_path, mutate):
    data = yaml.safe_load(MISSION_FILE.read_text(encoding='utf-8'))
    mutate(data)
    path = tmp_path / 'mission.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    return path


def test_map_hash_mismatch_fails_closed(tmp_path):
    """Reject a mission bound to a different saved-map image."""
    path = _write_mutated(
        tmp_path,
        lambda data: data['map'].__setitem__('image_sha256', '0' * 64),
    )
    mission = load_mission(path)
    with pytest.raises(MissionConfigError, match='SHA-256'):
        verify_mission_map(mission, MAP_FILE)


def test_keepout_semantics_mismatch_fails_closed(tmp_path):
    """Bind effective mask occupancy, including YAML negate/thresholds."""
    source = yaml.safe_load(KEEPOUT_FILE.read_text(encoding='utf-8'))
    source['image'] = str(
        (KEEPOUT_FILE.parent / source['image']).resolve())
    source['negate'] = 1
    changed = tmp_path / 'keepout.yaml'
    changed.write_text(yaml.safe_dump(source), encoding='utf-8')
    mission = load_mission(MISSION_FILE)
    with pytest.raises(MissionConfigError, match='keepout OccupancyGrid'):
        verify_mission_map(mission, MAP_FILE, changed)


def test_waypoint_file_order_must_match_sequence(tmp_path):
    """Prevent YAML order from silently changing route execution order."""
    path = _write_mutated(
        tmp_path,
        lambda data: data['route']['waypoints'].__setitem__(
            slice(0, 2), list(reversed(data['route']['waypoints'][:2]))),
    )
    with pytest.raises(MissionConfigError, match='ascending sequence order'):
        load_mission(path)


def test_each_shared_task_micro_adjustment_is_checked(tmp_path):
    """Validate the second plant task even when it reuses a base stop."""
    path = _write_mutated(
        tmp_path,
        lambda data: data['route']['waypoints'][1][
            'micro_adjust'].__setitem__('lateral_m', 2.0),
    )
    mission = load_mission(path)
    with pytest.raises(MissionConfigError, match='micro-adjust envelope'):
        verify_mission_map(mission, MAP_FILE, KEEPOUT_FILE)


@pytest.mark.parametrize(
    'mutate, message',
    [
        (
            lambda data: data['route']['waypoints'][0].__setitem__(
                'work_side', 'right'),
            'work_side disagrees',
        ),
        (
            lambda data: data['route']['waypoints'][0][
                'base_pose'].__setitem__('y', 2.0),
            'outside corridor',
        ),
        (
            lambda data: data['route']['waypoints'][1]['plant'].__setitem__(
                'id', data['route']['waypoints'][0]['plant']['id']),
            'each plant',
        ),
    ],
)
def test_unsafe_route_edits_are_rejected(tmp_path, mutate, message):
    """Reject invalid sides, corridor poses, and plant identities."""
    path = _write_mutated(tmp_path, mutate)
    with pytest.raises(MissionConfigError, match=message):
        load_mission(path)
