"""Real worker callbacks deduplicate clock samples without hiding unsafe state."""

from concurrent.futures import ThreadPoolExecutor
import json
import threading
from types import SimpleNamespace as NS

import pytest
from rclpy.time import Time

from agri_arm_control.harvest_worker import HarvestArmWorker


def _worker():
    clock = NS(ns=100_000_000_000, reads=0)

    def now():
        clock.reads += 1
        return Time(nanoseconds=clock.ns)

    states, statuses = [], []
    arm = NS(stowed=lambda: True, stow_report=lambda: {'passed': True},
             terminal_confirmed=True, execution_status={}, _navigation_stowed=True,
             reset_calls=[])
    def navigation_stowed():
        arm._navigation_stowed = arm.stowed()
        return arm._navigation_stowed
    def reset_navigation_stow(**_kwargs):
        arm.reset_calls.append(True)
        arm._navigation_stowed = False
    arm.navigation_stowed = navigation_stowed
    arm.navigation_stow_report = lambda: {
        'passed': arm._navigation_stowed,
        'reasons': [] if arm._navigation_stowed else ['strict_stow_latch_not_acquired']}
    arm.reset_navigation_stow = reset_navigation_stow
    worker = NS(
        _mutex=threading.RLock(), _last_heartbeat=None,
        _healthy=True, _fault=False, _busy=False, _phase='IDLE',
        _work_context=('', '', ''), _startup_checked=True,
        p={'task_frame': 'base_footprint'},
        arm=arm,
        _grasp_diagnostics=[], _contact_diagnostics={}, _scene_ready_diagnostics={},
        _scan_diagnostics={}, _selected_target={}, _last_error='', _last_health_error='',
        get_clock=lambda: NS(now=now), _pub=NS(publish=states.append),
        _status_pub=NS(publish=lambda message: statuses.append(json.loads(message.data))))
    return worker, clock, states, statuses


def _stamp(state):
    return state.header.stamp.sec * 1_000_000_000 + state.header.stamp.nanosec


def test_timer_and_finally_same_clock_publish_one_real_sample():
    worker, clock, states, statuses = _worker()
    HarvestArmWorker._heartbeat(worker)
    HarvestArmWorker._heartbeat(worker)
    assert len(states) == len(statuses) == 1
    assert states[0].arm_stowed
    assert _stamp(states[0]) == statuses[0]['stamp_ns'] == clock.ns
    # One clock sample per call, including the suppressed callback.
    assert clock.reads == 2
    clock.ns += 10_000_000
    HarvestArmWorker._heartbeat(worker)
    assert [_stamp(state) for state in states] == [100_000_000_000, clock.ns]


@pytest.mark.parametrize('change', ['busy', 'fault', 'health', 'phase', 'measured'])
def test_dangerous_change_at_same_clock_is_published_immediately(change):
    worker, clock, states, statuses = _worker()
    HarvestArmWorker._heartbeat(worker)
    if change == 'busy':
        worker._busy = True
    elif change == 'fault':
        worker._fault = True
    elif change == 'health':
        worker._healthy = False
    elif change == 'phase':
        worker._phase = 'DEPLOYING'
    else:
        worker.arm.stowed = lambda: False
    HarvestArmWorker._heartbeat(worker)
    assert len(states) == len(statuses) == 2
    assert states[0].arm_stowed and not states[1].arm_stowed
    assert _stamp(states[0]) == _stamp(states[1]) == clock.ns
    assert statuses[-1]['stamp_ns'] == clock.ns
    if change != 'measured':
        assert worker.arm.reset_calls
    assert statuses[-1]['stow_denial_reason']


@pytest.mark.parametrize('attribute,value', [
    ('_work_context', ('run', 'station', 'work')), ('_phase', 'STOWED'),
])
def test_changed_identity_or_phase_is_not_an_exact_duplicate(attribute, value):
    worker, clock, states, _statuses = _worker()
    HarvestArmWorker._heartbeat(worker)
    setattr(worker, attribute, value)
    HarvestArmWorker._heartbeat(worker)
    assert len(states) == 2
    assert _stamp(states[0]) == _stamp(states[1]) == clock.ns


def test_rollback_is_published_as_real_time_and_never_forged_forward():
    worker, clock, states, _statuses = _worker()
    HarvestArmWorker._heartbeat(worker)
    clock.ns = 20_000_000_000
    HarvestArmWorker._heartbeat(worker)
    HarvestArmWorker._heartbeat(worker)
    assert [_stamp(state) for state in states] == [100_000_000_000, 20_000_000_000]
    clock.ns += 1_000_000
    HarvestArmWorker._heartbeat(worker)
    assert _stamp(states[-1]) == 20_001_000_000


def test_concurrent_timer_finally_callbacks_are_serialized_and_deduplicated():
    worker, clock, states, statuses = _worker()
    barrier = threading.Barrier(8)

    def callback(_index):
        barrier.wait(timeout=2.)
        HarvestArmWorker._heartbeat(worker)

    with ThreadPoolExecutor(max_workers=8) as executor:
        list(executor.map(callback, range(8)))
    assert len(states) == len(statuses) == 1
    assert clock.reads == 8
    assert _stamp(states[0]) == clock.ns


def test_background_health_cannot_mutate_midway_through_a_heartbeat_snapshot():
    worker, clock, states, _statuses = _worker()
    checking, release, health_failed = (threading.Event() for _index in range(3))

    def stowed():
        checking.set()
        assert release.wait(timeout=2.)
        return True

    def not_ready():
        health_failed.set()
        raise RuntimeError('backend stopped')

    worker.arm.stowed, worker.arm.ready = stowed, not_ready
    worker._closing = threading.Event()
    worker.context = NS(ok=lambda: True)
    worker.p['health_interval_s'] = .001
    worker.get_logger = lambda: NS(warning=lambda _message: worker._closing.set())
    with ThreadPoolExecutor(max_workers=2) as executor:
        heartbeat = executor.submit(HarvestArmWorker._heartbeat, worker)
        assert checking.wait(timeout=2.)
        health = executor.submit(HarvestArmWorker._health, worker)
        assert health_failed.wait(timeout=2.)
        # Backend failure is known, but its state transition must wait for the
        # existing serial snapshot rather than changing the report mid-sample.
        assert worker._healthy
        release.set()
        heartbeat.result(timeout=2.)
        health.result(timeout=2.)
    assert not worker._healthy
    HarvestArmWorker._heartbeat(worker)
    assert [state.arm_stowed for state in states] == [True, False]
    assert [_stamp(state) for state in states] == [clock.ns, clock.ns]
    assert worker._last_health_error == 'backend stopped'


def test_idle_backend_health_does_not_refresh_scene_or_hide_last_failure():
    worker, _, _, _ = _worker()
    worker._healthy = False
    worker._last_health_error = 'previous backend timeout'
    worker._closing = threading.Event()
    worker.context = NS(ok=lambda: True)
    worker.p['health_interval_s'] = .001
    checked = []
    def ready():
        checked.append('backend')
        worker._closing.set()
    worker.arm.ready = ready
    worker._detect_startup_pose = lambda: checked.append('startup')
    worker._environment_ready = lambda **_kwargs: pytest.fail('idle health cannot refresh whole scene')
    HarvestArmWorker._health(worker)
    assert checked == ['backend', 'startup']
    assert worker._healthy and worker._last_health_error == ''


def test_health_failure_status_distinguishes_backend_from_measured_joint_stow():
    worker, _, states, statuses = _worker()
    worker._healthy = False
    worker._last_health_error = 'arm controller is not active'
    HarvestArmWorker._heartbeat(worker)
    assert not states[-1].arm_stowed
    assert statuses[-1]['stow_evidence']['passed']
    assert statuses[-1]['stow_denial_reason'] == 'backend_unhealthy'
    assert statuses[-1]['last_health_error'] == 'arm controller is not active'
