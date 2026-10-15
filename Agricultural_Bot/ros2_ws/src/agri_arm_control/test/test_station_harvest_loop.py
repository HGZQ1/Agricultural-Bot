"""Exercise repeated fruit transactions at one station without ROS or motion."""

from types import SimpleNamespace as NS
import threading

import pytest

from agri_arm_control.controller import MotionError
from agri_arm_control.harvest_worker import HarvestArmWorker

from test_worker_boundaries import chain_fixture


def station_fixture(monkeypatch, observations, fail_target=None):
    """Fake camera and physical basket outcomes, retaining the worker's counting."""
    request = NS(plant_id='tomato_20', work_side='left', work_id='same-station-work')
    observations = iter(observations)
    scans, basket, phases, stow = [], [], [], []
    worker = NS(
        p={'stow_joints': [0.0] * 6},
        arm=NS(_check=lambda: None, move_joints=lambda joints: stow.append(joints),
               stowed=lambda: True),
        _scan_height_m=None, _observation=None, _target_history={},
        _work_context=('run', 'waypoint', request.work_id),
        _harvested_target_ids=set(),
        _station_progress={'harvested_count': 0, 'scan_completed': False, 'fruits': []},
        _set_phase=phases.append, _to_planning=lambda *args: args,
        _scene=lambda *_args, **_kwargs: pytest.fail('fake basket owns scene results'),
        _record_grasp_diagnostics=lambda *_args: None,
        _record_contact_diagnostics=lambda *_args: None,
        get_logger=lambda: NS(info=lambda _message: None),
    )

    def select(current, *, start_height=None, allow_empty=False):
        assert current is request
        assert worker._observation is None  # Never reuse the previous fruit's image.
        scans.append((start_height, allow_empty))
        observation = next(observations)
        if observation is None:
            worker._scan_height_m = 0.9  # Camera completed its lowest valid empty level.
            return None
        target_id, height = observation
        worker._scan_height_m = height
        return NS(target_id=target_id, position=NS(x=-0.18, y=0.65, z=height),
                  class_name='tomato_R', confidence=0.9, mature=True), NS(
                      frame_id='base_footprint', stamp=NS(sec=2, nanosec=0))

    def verified_chain(*_args, **_kwargs):
        def execute(_position, side, *, retract=True):
            assert side == request.work_side
            assert retract is False  # Stow belongs to the whole station, not each fruit.
            target_id = worker._observation[0].target_id
            if target_id == fail_target:
                raise MotionError('second fruit basket verification failed')
            basket.append(target_id)
            return True
        return NS(execute=execute)

    monkeypatch.setattr('agri_arm_control.harvest_worker.GraspExecutor', verified_chain)
    worker._select_after_scan = select
    worker._harvest_selected = lambda current, **kwargs: HarvestArmWorker._harvest_selected(
        worker, current, **kwargs)
    return worker, request, scans, basket, phases, stow


def test_two_fruits_resume_saved_observation_heights_and_stow_once(monkeypatch):
    worker, request, scans, basket, phases, stow = station_fixture(
        monkeypatch, [('upper-ripe', 1.4), ('lower-ripe', 1.1), None])

    HarvestArmWorker._harvest_station(worker, request)

    assert scans == [(None, True), (1.4, True), (1.1, True)]
    assert basket == ['upper-ripe', 'lower-ripe']
    assert worker._station_progress['harvested_count'] == 2
    assert worker._station_progress['scan_completed']
    assert [fruit['target_id'] for fruit in worker._station_progress['fruits']] == basket
    assert worker._harvested_target_ids == set(basket)
    assert worker._observation is None
    assert phases == ['PLANT_SCAN_COMPLETE', 'RETRACTING']
    assert stow == [[0.0] * 6]


def test_multiple_fruits_at_same_height_are_taken_before_descending(monkeypatch):
    worker, request, scans, basket, _, stow = station_fixture(
        monkeypatch, [('ripe-a', 1.2), ('ripe-b', 1.2), ('ripe-c', 1.0), None])

    HarvestArmWorker._harvest_station(worker, request)

    assert scans == [(None, True), (1.2, True), (1.2, True), (1.0, True)]
    assert basket == ['ripe-a', 'ripe-b', 'ripe-c']
    assert worker._station_progress['harvested_count'] == 3
    assert worker._station_progress['scan_completed']
    assert len(stow) == 1


def test_valid_empty_plant_scan_completes_without_claiming_a_harvest(monkeypatch):
    worker, request, scans, basket, phases, stow = station_fixture(monkeypatch, [None])

    HarvestArmWorker._harvest_station(worker, request)

    assert scans == [(None, True)]
    assert not basket and not worker._harvested_target_ids
    assert worker._station_progress == {
        'harvested_count': 0, 'scan_completed': True, 'fruits': []}
    assert phases == ['PLANT_SCAN_COMPLETE', 'RETRACTING']
    assert stow == [[0.0] * 6]


def test_second_fruit_failure_preserves_first_count_without_finishing_station(monkeypatch):
    worker, request, scans, basket, phases, stow = station_fixture(
        monkeypatch, [('ripe-a', 1.3), ('ripe-b', 1.1), None], fail_target='ripe-b')

    with pytest.raises(MotionError, match='basket verification failed'):
        HarvestArmWorker._harvest_station(worker, request)

    assert scans == [(None, True), (1.3, True)]
    assert basket == ['ripe-a']
    assert worker._station_progress['harvested_count'] == 1
    assert worker._harvested_target_ids == {'ripe-a'}
    assert not worker._station_progress['scan_completed']
    assert phases == []  # Neither station-complete nor final retraction can be claimed.
    assert stow == []


def test_grasp_chain_can_verify_basket_and_clear_contact_without_per_fruit_stow():
    chain, stow = chain_fixture()
    operations, phases = [], []
    original_scene = chain.scene

    def scene(operation, **kwargs):
        operations.append(operation)
        return original_scene(operation, **kwargs)

    chain.scene, chain.phase = scene, phases.append
    assert chain.execute((-0.18, 0.65, 0.95), 'left', retract=False)
    assert operations[-2:] == ['wait_in_basket', 'clear']
    assert phases[-1] == 'CLEARING_COLLISION_TARGET'
    assert 'RETRACTING' not in phases
    assert stow == []


def execute_station_fixture(monkeypatch, observations):
    """Run the actual worker result conversion around its real station loop."""
    worker, request, scans, basket, phases, stow = station_fixture(monkeypatch, observations)
    actions, errors, heartbeats = [], [], []
    request.harvest_until_clear = True
    worker._ideal_simulation = True
    worker._mutex = threading.RLock()
    worker._busy, worker._started_wall = True, 1.0
    worker._fault = False
    worker.arm.ready = lambda: None
    worker.arm.cancel_and_confirm = lambda: True
    worker._environment_ready = lambda: None
    worker._harvest_station = lambda current: HarvestArmWorker._harvest_station(worker, current)
    worker._heartbeat = lambda: heartbeats.append(worker._busy)
    worker.get_logger = lambda: NS(info=lambda _message: None, error=errors.append)
    handle = NS(request=request, is_cancel_requested=False,
                succeed=lambda: actions.append('succeeded'),
                abort=lambda: actions.append('aborted'),
                canceled=lambda: actions.append('canceled'))
    result = HarvestArmWorker._execute(worker, handle)
    assert errors == []
    assert actions == ['succeeded']
    assert heartbeats == [False]
    assert not worker._busy and worker._handle is None
    assert phases[-1] == 'STOWED'
    assert len(stow) == 1
    return result, basket, scans


def test_execute_returns_successful_empty_scan_without_a_verified_fruit(monkeypatch):
    result, basket, scans = execute_station_fixture(monkeypatch, [None])
    assert result.success and result.arm_stowed
    assert result.scan_completed
    assert result.harvested_count == 0
    assert result.sim_fruit_verified is False
    assert result.target_id == ''
    assert basket == [] and scans == [(None, True)]


def test_execute_returns_two_verified_fruits_only_after_whole_plant_scan(monkeypatch):
    result, basket, scans = execute_station_fixture(
        monkeypatch, [('ripe-a', 1.3), ('ripe-b', 1.1), None])
    assert result.success and result.arm_stowed
    assert result.scan_completed
    assert result.harvested_count == 2
    assert result.sim_fruit_verified is True
    assert result.target_id == 'ripe-b'
    assert basket == ['ripe-a', 'ripe-b']
    assert scans == [(None, True), (1.3, True), (1.1, True)]
