"""Ideal simulation removes protective locks while preserving actual results."""

from concurrent.futures import Future
import json
from types import SimpleNamespace as NS

import pytest
from std_msgs.msg import Bool, String

from agri_task_manager.field_route_executor import FieldRouteExecutor
from agri_task_manager.harvest_coordinator import HarvestCoordinator


@pytest.mark.parametrize('velocity,expected', [
    (None, False), ((0., 0., 0., 0.), True), ((.009, 0., 0., .019), True),
    ((.01, 0., 0., 0.), False), ((0., 0., .011, 0.), False),
    ((0., 0., 0., .02), False), ((float('nan'), 0., 0., 0.), False),
])
def test_only_actual_finite_zero_velocity_allows_station_work(velocity, expected):
    assert FieldRouteExecutor._actual_stopped(NS(_base_velocity=velocity)) is expected


def test_ideal_movement_does_not_consult_worker_stow_but_keeps_active_work_ownership():
    node = NS(_ideal_simulation=True, _require_harvest=True, _work_terminal=True,
              _worker_lease=NS(stowed=lambda *_args: pytest.fail('stow lease must not gate transit')))
    assert FieldRouteExecutor._work_movement_error(node) is None
    node._work_terminal = False
    assert 'active or its terminal result' in FieldRouteExecutor._work_movement_error(node)


def test_ideal_work_completion_does_not_wait_for_stow_heartbeat_or_station_tf():
    completed, failed = [], []
    node = NS(_ideal_simulation=True, _require_harvest=True, _state='AT_STATION',
              _work_started=True, _actual_stopped=lambda: True, _work_terminal=True,
              _work_result_pending={'sim_fruit_verified': True}, _work_id='actual-work',
              _record=lambda *_args: None, _finish_station=lambda: completed.append(True),
              _fail=failed.append)
    FieldRouteExecutor._check_harvest(node, 1000.)
    assert completed == [True] and not failed
    assert node._work_result_pending is None and node._work_phase == 'COMPLETED'


def test_nonzero_actual_velocity_still_stops_work_in_ideal_mode():
    failed = []
    node = NS(_ideal_simulation=True, _require_harvest=True, _state='AT_STATION',
              _work_started=True, _actual_stopped=lambda: False, _fail=failed.append)
    FieldRouteExecutor._check_harvest(node, 1.)
    assert failed == ['actual base velocity is nonzero during harvest']


def test_ideal_task_failure_releases_program_stop_and_never_requests_protective_lock():
    commands, states = [], []
    node = NS(_ideal_simulation=True, _lock_generation=1, _goal_generation=2,
              _unsafe_active_goal=True, _goal_handle=None,
              _require_lock=True, _lock_recovery=True,
              _clear_lock_transition=lambda: None,
              _lock_client=NS(service_is_ready=lambda: True,
                              call_async=lambda request: commands.append(request.data)),
              get_logger=lambda: NS(error=lambda _message: None, warning=lambda _message: None),
              _publish_point_tombstone=lambda _message: None,
              _set_state=lambda state, reason: states.append((state, reason)))
    FieldRouteExecutor._fail(node, 'gripper actual result failed')
    assert commands == [False]
    assert states == [('FAILED', 'gripper actual result failed')]
    assert not node._lock_recovery and not node._work_permission
    assert node._goal_generation == 3 and not node._unsafe_active_goal


def test_ideal_old_nav_result_releases_owner_without_reviving_failed_mission():
    future = Future()
    future.set_result(NS(status=5))
    node = NS(_ideal_simulation=True, _goal_result_future=future,
              _goal_generation=2, _goal_handle=NS(),
              _lock_then=lambda *_args: pytest.fail('late result cannot park a new station'))
    FieldRouteExecutor._goal_result(node, future, 1, 'old-goal')
    assert node._goal_result_future is None and node._goal_handle is None


def test_ideal_start_does_not_keep_a_permanent_protective_fault_latch():
    node = NS(_ideal_simulation=True, _unsafe_active_goal=True, _state='FAILED',
              _shutting_down=False, _lock_pending=False, _work_movement_error=lambda: None,
              _service_reply=FieldRouteExecutor._service_reply)
    reply = FieldRouteExecutor._start_service(node, None, NS(success=False, message=''))
    assert reply.success and node._start_pending


def test_failed_state_does_not_automatically_relock_when_user_unlocks():
    node = NS(_ideal_simulation=True, _base_lock_observed=True, _lock_observation_count=0,
              _shutting_down=False, _require_lock=True, _state='FAILED', _lock_pending=False,
              _record=lambda *_args: None, _publish_status=lambda: None,
              _start_lock_recovery=lambda *_args: pytest.fail('fault cannot create protective lock'))
    FieldRouteExecutor._lock_state_callback(node, Bool(data=False))
    assert node._base_lock_observed is False


@pytest.mark.parametrize('command,lock_expected', [('pause', True), ('cancel', True),
                                                 ('timeout', False), ('binding_mismatch', False)])
def test_explicit_manual_stop_is_distinct_from_automatic_fault_lock(command, lock_expected):
    stopped = []
    node = NS(_ideal_simulation=True, _after_cancel=None, _require_lock=True,
              _goal_handle=None, _set_state=lambda *_args: None,
              _cancel_lock_ready=lambda: None,
              _lock_then=lambda _callback: stopped.append(True))
    FieldRouteExecutor._begin_cancel(node, command)
    assert bool(stopped) is lock_expected
    if not lock_expected:
        assert node._cancel_lock_confirmed


def coordinator_fixture(ideal):
    context = ('run', 'point', 'work')
    node = HarvestCoordinator.__new__(HarvestCoordinator)
    node._ideal_simulation = ideal
    node._route_received = None
    node._route = {'require_harvest_handshake': True, 'state': 'AT_STATION',
        'status_stamp_ns': 1_000_000_000, 'run_id': 'run', 'work_id': 'work',
        'current': {'waypoint_id': 'point'}, 'harvest_work_ready': True}
    node._station = {'active': True, 'station_started_ns': 1_000_000_000,
        'run_id': 'run', 'waypoint_id': 'point', 'work_id': 'work',
        'plant_id': 'plant', 'work_side': 'left'}
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000))
    return node, context


def test_old_route_heartbeat_is_allowed_only_in_explicit_ideal_mode():
    node, context = coordinator_fixture(True)
    assert node._station_context() == context
    node._ideal_simulation = False
    assert node._station_context() is None


def test_ideal_mode_still_rejects_changed_nonce_and_explicit_pause():
    node, _ = coordinator_fixture(True)
    node._route['work_id'] = 'another-work'
    assert node._station_context() is None
    node._route['work_id'] = 'work'
    node._route['state'] = 'PAUSED'
    assert node._station_context(require_permission=False) is None


@pytest.mark.parametrize('status,success,verified,phase', [
    (4, True, True, 'SUCCESS'), (6, True, True, 'FAILED'),
    (4, False, True, 'FAILED'), (4, True, False, 'FAILED'),
])
def test_ideal_success_requires_real_action_and_fruit_verification_not_stow(
        status, success, verified, phase):
    node, context = coordinator_fixture(True)
    node._work_context, node._handled_contexts = context, set()
    node._target_history, node._selected_target_id = {}, ''
    reports = []
    node._report = lambda result_phase, **details: reports.append((result_phase, details))
    future = Future()
    future.set_result(NS(status=status, result=NS(target_id='ripe-actual', success=success,
        arm_stowed=False, sim_fruit_verified=verified, message='actual simulation result')))
    node._result(future)
    assert reports[-1][0] == phase
    assert reports[-1][1]['arm_stowed'] is False
    assert reports[-1][1]['sim_fruit_verified'] is verified


def test_ideal_route_accepts_verified_fruit_result_without_stow_ack():
    node = FieldRouteExecutor.__new__(FieldRouteExecutor)
    node._ideal_simulation, node._require_harvest = True, True
    node._work_id, node._run_id = 'work', 'run'
    node._state, node._work_started = 'AT_STATION', True
    node._work_station_started_ns, node._work_last_report_ns = 1_000_000_000, 0
    node._current_point = lambda: NS(waypoint_id='point')
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000))
    node._record, node._publish_status = lambda *_args: None, lambda: None
    node._fail = lambda reason: pytest.fail(reason)
    node._work_report_callback(String(data=json.dumps({'run_id': 'run',
        'waypoint_id': 'point', 'work_id': 'work', 'stamp_ns': 2_000_000_000,
        'phase': 'SUCCESS', 'success': True, 'arm_stowed': False, 'sim_fruit_verified': True})))
    assert node._work_terminal and node._work_result_pending['sim_fruit_verified']
    assert node._work_phase == 'WORK_COMPLETED'
