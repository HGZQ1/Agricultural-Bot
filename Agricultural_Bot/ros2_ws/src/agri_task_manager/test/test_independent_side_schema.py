"""Contracts for independent side stations and legacy shared stops."""

from copy import deepcopy
from pathlib import Path

from agri_task_manager.waypoint_schema import load_mission, MissionConfigError

import pytest

import yaml


MISSION_FILE = (
    Path(__file__).resolve().parents[1] / 'config' / 'missions'
    / 'tomato_field_default_single_corridor.yaml')


def _route_data(layout):
    """Build both layouts from the versioned plant identity fixture."""
    data = yaml.safe_load(MISSION_FILE.read_text(encoding='utf-8'))
    count = data['field']['plants_per_row']
    points = deepcopy(data['route']['waypoints'])
    by_side = {
        side: sorted(
            (point for point in points if point['work_side'] == side),
            key=lambda point: point['plant']['index_in_row'])
        for side in ('left', 'right')
    }
    data['route']['corridor'].update({
        'y_min': -0.65, 'y_max': 0.65,
        'required_yaw': 0.0, 'yaw_tolerance': 0.20,
    })
    data['route']['home']['pose']['yaw'] = 0.0
    data['route']['layout'] = layout
    data['route'].pop('transits', None)
    if layout == 'shared_pairs':
        ordered = []
        for column in range(count):
            left = by_side['left'][column]
            right = by_side['right'][column]
            centre_y = (
                left['plant']['pose_map']['y'] +
                right['plant']['pose_map']['y']) / 2.0
            common_pose = deepcopy(left['base_pose'])
            common_pose.update({'y': centre_y, 'yaw': 0.0})
            for point in (left, right):
                point['stop_id'] = f'shared_{column:02d}'
                point['base_pose'] = deepcopy(common_pose)
                ordered.append(point)
    else:
        ordered = by_side['left'] + list(reversed(by_side['right']))
        for point in ordered:
            side = point['work_side']
            column = point['plant']['index_in_row']
            point['stop_id'] = f'{side}_{column:02d}'
            point['base_pose'].update({
                'y': 0.40 if side == 'left' else -0.40, 'yaw': 0.0,
            })
    for sequence, point in enumerate(ordered, start=1):
        point['sequence'] = sequence
    data['route']['waypoints'] = ordered
    return data


def _write(tmp_path, data):
    path = tmp_path / 'mission.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    return path


def test_independent_stations_cover_each_plant_on_two_ordered_legs(tmp_path):
    """Each plant gets a separate stop without alternating across the aisle."""
    data = _route_data('independent_side_stops')
    mission = load_mission(_write(tmp_path, data))
    count = mission.field.plants_per_row
    assert mission.route_layout == 'independent_side_stops'
    assert len({point.stop_id for point in mission.waypoints}) == 2 * count
    assert [point.work_side for point in mission.waypoints] == (
        ['left'] * count + ['right'] * count)
    assert [point.plant.index_in_row for point in mission.waypoints] == (
        list(range(count)) + list(reversed(range(count))))
    assert all(point.pose.y > 0.0 for point in mission.waypoints[:count])
    assert all(point.pose.y < 0.0 for point in mission.waypoints[count:])


@pytest.mark.parametrize('explicit', [True, False])
def test_legacy_shared_pairs_remain_supported(tmp_path, explicit):
    """Existing files keep their paired-stop rules when layout is omitted."""
    data = _route_data('shared_pairs')
    if not explicit:
        del data['route']['layout']
    mission = load_mission(_write(tmp_path, data))
    assert mission.route_layout == 'shared_pairs'
    assert len({point.stop_id for point in mission.waypoints}) == (
        mission.field.plants_per_row)


def test_legacy_layout_does_not_accept_singleton_stops(tmp_path):
    """Selecting a new layout is required before shared pairs may split."""
    data = _route_data('independent_side_stops')
    del data['route']['layout']
    with pytest.raises(MissionConfigError, match='one task per work side'):
        load_mission(_write(tmp_path, data))


def test_independent_stations_reject_shared_stop_ids(tmp_path):
    """The executor must navigate to every independently placed station."""
    data = _route_data('independent_side_stops')
    data['route']['waypoints'][1]['stop_id'] = (
        data['route']['waypoints'][0]['stop_id'])
    with pytest.raises(MissionConfigError, match='unique stop IDs'):
        load_mission(_write(tmp_path, data))


def test_independent_return_leg_must_reverse_column_order(tmp_path):
    """A correctly renumbered route still needs the intended return leg."""
    data = _route_data('independent_side_stops')
    count = data['field']['plants_per_row']
    data['route']['waypoints'][count:] = list(reversed(
        data['route']['waypoints'][count:]))
    for sequence, point in enumerate(data['route']['waypoints'], start=1):
        point['sequence'] = sequence
    with pytest.raises(MissionConfigError, match='descending right-side'):
        load_mission(_write(tmp_path, data))


def test_independent_route_rejects_alternating_sides(tmp_path):
    """Do not turn every station into a cross-aisle manoeuvre."""
    data = _route_data('independent_side_stops')
    count = data['field']['plants_per_row']
    points = data['route']['waypoints']
    points[1], points[count] = points[count], points[1]
    for sequence, point in enumerate(points, start=1):
        point['sequence'] = sequence
    with pytest.raises(MissionConfigError, match='ascending left-side'):
        load_mission(_write(tmp_path, data))


@pytest.mark.parametrize('target', ['station', 'home'])
def test_independent_route_requires_zero_yaw(tmp_path, target):
    """A broader corridor tolerance must not permit reversed base headings."""
    data = _route_data('independent_side_stops')
    if target == 'home':
        data['route']['home']['pose']['yaw'] = 0.10
    else:
        data['route']['waypoints'][0]['base_pose']['yaw'] = 0.10
    with pytest.raises(MissionConfigError, match='yaw=0'):
        load_mission(_write(tmp_path, data))


def test_independent_route_keeps_plant_side_contract(tmp_path):
    """A station marked left cannot be positioned beyond its target plant."""
    data = _route_data('independent_side_stops')
    data['route']['waypoints'][0]['work_side'] = 'right'
    with pytest.raises(MissionConfigError, match='work_side disagrees'):
        load_mission(_write(tmp_path, data))


def test_unknown_layout_fails_closed(tmp_path):
    """Misspelled layout names cannot silently weaken route validation."""
    data = _route_data('independent_side_stops')
    data['route']['layout'] = 'independent'
    with pytest.raises(MissionConfigError, match='route.layout must be'):
        load_mission(_write(tmp_path, data))


def _transit_route():
    data = _route_data('independent_side_stops')
    count = data['field']['plants_per_row']
    target = data['route']['waypoints'][count]['id']
    x = data['route']['waypoints'][count]['base_pose']['x']
    data['route']['corridor']['x_max'] = x + 1.50
    data['route']['transits'] = [
        {
            'id': 'exit_left_row', 'before_waypoint_id': target,
            'pose': {'x': x + 1.0, 'y': 0.40, 'yaw': 0.0},
        },
        {
            'id': 'cross_headland', 'before_waypoint_id': target,
            'pose': {'x': x + 1.0, 'y': -0.40, 'yaw': 0.0},
            'goal_timeout_s': 90.0,
        },
    ]
    return data


def test_transits_preserve_before_station_order_and_timeout(tmp_path):
    """Headland goals stay ordered and do not become harvest tasks."""
    mission = load_mission(_write(tmp_path, _transit_route()))
    assert [point.transit_id for point in mission.route_transits] == [
        'exit_left_row', 'cross_headland',
    ]
    assert len(mission.waypoints) == 2 * mission.field.plants_per_row
    assert mission.route_transits[0].goal_timeout_s == 120.0
    assert mission.route_transits[1].goal_timeout_s == 90.0
    assert (mission.route_transits[0].before_waypoint_id ==
            mission.route_transits[1].before_waypoint_id)


@pytest.mark.parametrize(
    'mutate, message',
    [
        (
            lambda data: data['route']['transits'][1].__setitem__(
                'id', data['route']['transits'][0]['id']),
            'transit IDs must be unique',
        ),
        (
            lambda data: data['route']['transits'][0].__setitem__(
                'id', data['route']['waypoints'][0]['id']),
            'transit IDs must differ',
        ),
        (
            lambda data: data['route']['transits'][0].__setitem__(
                'before_waypoint_id', 'unknown_plant'),
            'enabled harvest waypoint',
        ),
        (
            lambda data: data['route']['waypoints'][
                data['field']['plants_per_row']].__setitem__('enabled', False),
            'enabled harvest waypoint',
        ),
        (
            lambda data: data['route']['transits'][0]['pose'].__setitem__(
                'y', 2.0),
            'outside corridor',
        ),
        (
            lambda data: data['route']['transits'][0]['pose'].__setitem__(
                'yaw', 0.10),
            'transit poses require yaw=0',
        ),
        (
            lambda data: data['route']['transits'][0]['pose'].__setitem__(
                'x', float('nan')),
            'must be finite',
        ),
        (
            lambda data: data['route']['transits'][0].__setitem__(
                'goal_timeout_s', 0.0),
            'must be positive',
        ),
    ],
)
def test_unsafe_transit_contracts_fail_closed(tmp_path, mutate, message):
    """Reject unsafe transit references and poses before executing Nav2."""
    data = _transit_route()
    mutate(data)
    with pytest.raises(MissionConfigError, match=message):
        load_mission(_write(tmp_path, data))
