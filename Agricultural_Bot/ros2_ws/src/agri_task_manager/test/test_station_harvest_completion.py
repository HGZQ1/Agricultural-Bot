"""Only finish a whole-plant station after the worker completes its scan."""

from concurrent.futures import Future
import json
from types import SimpleNamespace as NS

import pytest
from std_msgs.msg import String

from agri_task_manager.field_route_executor import FieldRouteExecutor
from agri_task_manager.harvest_coordinator import HarvestCoordinator
from agri_task_manager.harvest_safety import station_harvest_evidence


@pytest.mark.parametrize('count,scanned,verified,accepted', [
    (0, True, False, True), (1, True, True, True), (8, True, True, True),
    (0, True, True, False), (1, True, False, False),
    (1, False, True, False), (0, False, False, False),
    (None, True, False, False), (True, True, True, False),
    (-1, True, False, False), (1.0, True, True, False),
])
def test_whole_plant_result_distinguishes_empty_scan_from_verified_basket(
        count, scanned, verified, accepted):
    assert station_harvest_evidence(
        count, scanned, verified, harvest_until_clear=True, ideal=True) is accepted


def test_legacy_single_fruit_mode_still_requires_verification_only_in_ideal_mode():
    assert station_harvest_evidence(
        None, None, True, harvest_until_clear=False, ideal=True)
    assert not station_harvest_evidence(
        None, None, False, harvest_until_clear=False, ideal=True)
    assert station_harvest_evidence(
        None, None, False, harvest_until_clear=False, ideal=False)


def _coordinator():
    node = HarvestCoordinator.__new__(HarvestCoordinator)
    context = ('run', 'point', 'work')
    node._ideal_simulation = True
    node._harvest_until_clear = True
    node._work_context = context
    node._handled_contexts = set()
    node._target_history = {}
    node._selected_target_id = ''
    node._station_context = lambda require_permission=True: context
    reports = []
    node._report = lambda phase, **details: reports.append((phase, details))
    return node, reports


@pytest.mark.parametrize('count,scanned,verified,expected_phase', [
    (0, True, False, 'SUCCESS'), (4, True, True, 'SUCCESS'),
    (1, False, True, 'FAILED'), (0, False, False, 'FAILED'),
    (0, True, True, 'FAILED'), (2, True, False, 'FAILED'),
])
def test_coordinator_reports_whole_plant_completion_without_fabricating_fruit(
        count, scanned, verified, expected_phase):
    node, reports = _coordinator()
    future = Future()
    future.set_result(NS(status=4, result=NS(
        success=True, arm_stowed=False, target_id='', message='scan outcome',
        harvested_count=count, scan_completed=scanned, sim_fruit_verified=verified)))
    node._result(future)
    phase, result = reports[-1]
    assert phase == expected_phase
    assert result['harvested_count'] == count
    assert result['scan_completed'] is scanned
    assert result['sim_fruit_verified'] is verified


def _route():
    node = FieldRouteExecutor.__new__(FieldRouteExecutor)
    node._ideal_simulation, node._require_harvest = True, True
    node._work_id, node._run_id = 'work', 'run'
    node._state, node._work_started = 'AT_STATION', False
    node._work_station_started_ns, node._work_last_report_ns = 1_000_000_000, 0
    node._current_point = lambda: NS(waypoint_id='point')
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000))
    node._record, node._publish_status = lambda *_args: None, lambda: None
    node._harvest_work_ready = lambda: True
    node._work_context = None
    node._work_result_pending = None
    failures = []
    node._fail = failures.append
    return node, failures


def _report(stamp, phase, **details):
    return String(data=json.dumps({
        'run_id': 'run', 'waypoint_id': 'point', 'work_id': 'work',
        'stamp_ns': stamp, 'phase': phase, **details}))


@pytest.mark.parametrize('count,scanned,verified,accepted', [
    (0, True, False, True), (3, True, True, True),
    (1, False, True, False), (0, False, False, False),
    (0, True, True, False), (2, True, False, False),
])
def test_route_binds_whole_plant_requirement_to_started_transaction(
        count, scanned, verified, accepted):
    node, failures = _route()
    node._work_report_callback(_report(2_000_000_000, 'STARTED', harvest_until_clear=True))
    assert node._work_harvest_until_clear
    # A terminal payload cannot turn off the mode acknowledged at STARTED.
    node._work_report_callback(_report(
        3_000_000_000, 'SUCCESS', harvest_until_clear=False, success=True,
        arm_stowed=False, harvested_count=count, scan_completed=scanned,
        sim_fruit_verified=verified))
    assert bool(node._work_result_pending) is accepted
    assert bool(failures) is (not accepted)


def test_zero_action_timeout_keeps_same_work_transaction_without_canceling(monkeypatch):
    node, _ = _coordinator()
    node._pending_goal = None
    node._handle = object()
    node._send_future = object()
    node._request_started = 1.0
    node._cancel_action = lambda: pytest.fail('0 disables the work action timeout')
    node.get_parameter = lambda name: NS(value=0.0) if name == 'action_timeout_s' else None
    node._publish_status = lambda: None
    monkeypatch.setattr('agri_task_manager.harvest_coordinator.time.monotonic', lambda: 100000.)
    node._tick()


def test_zero_route_work_timeout_does_not_cut_off_repeated_fruit_cycles():
    node, failures = _route()
    node._work_started = True
    node._actual_stopped = lambda: True
    node._work_terminal = False
    node._work_station_started_wall = 1.0
    node._work_timeout = 0.0
    node._check_harvest(100000.)
    assert failures == []
    assert node._work_result_pending is None
