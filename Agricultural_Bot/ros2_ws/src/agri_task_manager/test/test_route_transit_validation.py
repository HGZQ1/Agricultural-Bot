"""Map geometry checks include transit anchors, their links, and return."""

from types import SimpleNamespace

from agri_task_manager.route_validation import MapGrid, verify_mission_map
from agri_task_manager.waypoint_schema import (
    MapBinding, MicroAdjust, MissionConfigError, Pose2D, sha256_file,
    TransitWaypoint)

import pytest

import yaml


def _grid(tmp_path, label, occupied):
    """Write a small map with designated bottom-left occupied cells."""
    width, height = 20, 12
    pixels = bytes(
        0 if (x, y) in occupied else 255
        for y in range(height - 1, -1, -1) for x in range(width))
    image = tmp_path / f'{label}.pgm'
    image.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + pixels)
    config = tmp_path / f'{label}.yaml'
    config.write_text(yaml.safe_dump({
        'image': image.name, 'mode': 'trinary', 'resolution': 0.1,
        'origin': [0.0, 0.0, 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }), encoding='utf-8')
    return MapGrid.from_yaml(config)


def _mission(tmp_path, occupied=(), keepout_cells=()):
    """Bind one detour to independently loaded map and keepout pixels."""
    grid = _grid(tmp_path, 'map', occupied)
    keepout = _grid(tmp_path, 'keepout', keepout_cells)
    binding = MapBinding(
        frame_id='map', package='agri_navigation',
        yaml_relative_path='maps/map.yaml',
        image_sha256=sha256_file(grid.image_path),
        occupancy_sha256=grid.occupancy_sha256(),
        keepout_yaml_relative_path='maps/keepout.yaml',
        keepout_image_sha256=sha256_file(keepout.image_path),
        keepout_occupancy_sha256=keepout.occupancy_sha256(),
        resolution_m=grid.resolution, origin=grid.origin,
        width_px=grid.width, height_px=grid.height, mode=grid.mode,
        negate=grid.negate, occupied_thresh=grid.occupied_thresh,
        free_thresh=grid.free_thresh)
    points = tuple(SimpleNamespace(
        sequence=index, waypoint_id=label, stop_id=label,
        pose=pose, micro_adjust=MicroAdjust(0.0, 0.0, 0.0))
        for index, (label, pose) in enumerate((
            ('left', Pose2D(0.35, 0.65, 0.0)),
            ('right', Pose2D(0.95, 0.25, 0.0)),
        ), start=1))
    mission = SimpleNamespace(
        map_binding=binding, footprint=(0.02, 0.02, 0.02),
        max_unknown_cells=0, home=Pose2D(0.25, 0.25, 0.0),
        enabled_waypoints=points,
        route_transits=(TransitWaypoint(
            'side_change', 'right', Pose2D(0.95, 0.65, 0.0)),))
    return mission, grid.yaml_path, keepout.yaml_path


def test_transit_has_its_own_footprint_report(tmp_path):
    """A valid detour is inspected once without a work micro-adjust box."""
    mission, map_yaml, keepout_yaml = _mission(tmp_path)
    _, reports = verify_mission_map(mission, map_yaml, keepout_yaml)
    assert set(reports) == {'left', 'transit:side_change', 'right'}
    assert all(report.clear for report in reports.values())


@pytest.mark.parametrize('mask', [False, True])
def test_transit_footprint_rejects_map_and_keepout_obstacles(tmp_path, mask):
    """A free harvest point cannot hide an occupied intermediate anchor."""
    kwargs = {'keepout_cells' if mask else 'occupied': {(9, 6)}}
    mission, map_yaml, keepout_yaml = _mission(tmp_path, **kwargs)
    with pytest.raises(MissionConfigError, match='transit:side_change'):
        verify_mission_map(mission, map_yaml, keepout_yaml)


@pytest.mark.parametrize('cell', [(6, 6), (9, 4)])
def test_transit_incoming_and_outgoing_segments_are_checked(tmp_path, cell):
    """Both anchors can be clear while a required transit link is blocked."""
    mission, map_yaml, keepout_yaml = _mission(tmp_path, occupied={cell})
    with pytest.raises(MissionConfigError, match='preset route segment'):
        verify_mission_map(mission, map_yaml, keepout_yaml)


def test_return_home_link_from_transit_is_checked(tmp_path):
    """An outbound detour does not prove an operator's direct return safe."""
    mission, map_yaml, keepout_yaml = _mission(
        tmp_path, occupied={(6, 4)})
    with pytest.raises(MissionConfigError, match='return-home route segment'):
        verify_mission_map(mission, map_yaml, keepout_yaml)
