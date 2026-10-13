"""Fault-focused contracts for target selection and guarded station release."""

import json
from types import SimpleNamespace

from agri_harvest_interfaces.msg import HarvestWorkerState, TomatoTarget

from agri_task_manager.field_route_executor import FieldRouteExecutor
from agri_task_manager.harvest_coordinator import (
    HarvestCoordinator, side_and_window, target_quality, transform_position)
from agri_task_manager.harvest_safety import StoppedMotion, WorkerLease
from geometry_msgs.msg import TransformStamped
import pytest

from std_msgs.msg import String


def _worker(stamp, context=('', '', ''), *, stowed=True, phase='STOWED'):
    message = HarvestWorkerState()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    message.run_id, message.waypoint_id, message.work_id = context
    message.arm_stowed = stowed
    message.phase = phase
    return message


def test_stop_window_requires_measured_velocity_after_station_and_no_data_gaps():
    evidence = StoppedMotion()
    for offset in (0, 100, 200, 300, 400, 500):
        stamp = 1_000_000_000 + offset * 1_000_000
        assert evidence.update(stamp, stamp, offset / 1000, (0.0, 0.0, 0.0), 0.0)
    assert evidence.ready(1_500_000_000, 0.5, 1_000_000_000)
    assert not evidence.ready(1_500_000_000, 0.5, 1_300_000_000)
    # Fresh static TF/zero commands cannot rescue paused or lost wheel feedback.
    assert not evidence.ready(1_500_000_000, 1.0)
    evidence.update(2_000_000_000, 2_000_000_000, 1.0, (0.0, 0.0, 0.0), 0.0)
    assert not evidence.ready(2_000_000_000, 1.0)


@pytest.mark.parametrize('linear,angular', [
    ((0.0, 0.01, 0.0), 0.0), ((0.0, 0.0, 0.01), 0.0),
    ((0.0, 0.0, 0.0), 0.02), ((float('nan'), 0.0, 0.0), 0.0),
])
def test_any_measured_motion_or_invalid_velocity_revokes_stop_window(linear, angular):
    evidence = StoppedMotion()
    evidence.update(1_000_000_000, 1_000_000_000, 0.0, (0.0, 0.0, 0.0), 0.0)
    assert not evidence.update(1_600_000_000, 1_600_000_000, 0.6, linear, angular)
    assert not evidence.ready(1_600_000_000, 0.6)


def test_worker_stow_is_a_lease_with_matching_nonce_and_post_result_stamp():
    lease = WorkerLease()
    context = ('run', 'waypoint', 'work')
    assert lease.update(_worker(2_000_000_000, context), 2_000_000_000, 0.0)
    assert lease.stowed(2_000_000_000, 0.0, context)
    assert not lease.stowed(2_000_000_000, 0.0, ('run', 'waypoint', 'old_work'))
    assert not lease.stowed(2_000_000_000, 0.0, context, 2_100_000_000)
    assert not lease.stowed(2_000_000_000, 0.8, context)
    assert not lease.update(_worker(2_000_000_000, context), 2_000_000_000, 0.1)
    assert not lease.fresh(2_000_000_000, 0.1)


def _target():
    target = TomatoTarget()
    target.target_id = 'detector-session:42'
    target.class_name = 'tomato_R'
    target.mature = True
    target.segmentation_used = True
    target.confidence = 0.9
    target.temporally_stable = True
    target.stable_frame_count = 3
    target.depth_sample_count = 50
    target.depth_valid_fraction = 0.8
    target.depth_mad_m = 0.01
    target.position.x, target.position.y, target.position.z = -0.18, 0.65, 1.0
    return target


@pytest.mark.parametrize('attribute,value', [
    ('class_name', 'tomato_G'), ('mature', False), ('target_id', ''),
    ('confidence', 0.4), ('confidence', 1.01), ('confidence', float('nan')),
    ('temporally_stable', False), ('stable_frame_count', 2),
    ('depth_sample_count', 14), ('depth_valid_fraction', 0.59),
    ('depth_mad_m', 0.031), ('depth_mad_m', float('nan')),
    ('segmentation_used', False),
])
def test_target_filter_rejects_green_unstable_and_unreliable_observations(attribute, value):
    target = _target()
    assert target_quality(target)
    setattr(target, attribute, value)
    assert not target_quality(target)


def test_maskless_compatibility_requires_an_explicit_selection_override():
    """Keep background-contaminated box-only sampling opt-in for work goals."""
    target = _target()
    target.segmentation_used = False
    assert not target_quality(target)
    assert target_quality(target, require_segmentation=False)


def test_side_selection_uses_transformed_rep_axes_and_rejects_adjacent_plant_window():
    target = _target()
    transform = TransformStamped()
    # Example CAD-axis frame rotated into REP base axes; raw y is misleading.
    transform.transform.rotation.z = 2 ** -0.5
    transform.transform.rotation.w = 2 ** -0.5
    target.position.x, target.position.y = 0.65, 0.18
    xyz = transform_position(target.position, transform)
    assert side_and_window(xyz, 'left')
    assert not side_and_window(xyz, 'right')
    assert not side_and_window((0.52, 0.65, 1.0), 'left')


def _guarded_executor(monkeypatch):
    executor = object.__new__(FieldRouteExecutor)
    executor._require_harvest = True
    executor._work_terminal = False
    executor._work_context = ('run', 'waypoint', 'work')
    executor._work_previous_context = None
    executor._work_id = 'work'
    executor._run_id = 'run'
    executor._state = 'AT_STATION'
    executor._work_station_started_ns = 1_000_000_000
    executor._work_last_report_ns = 0
    executor._work_started = True
    executor._work_result_pending = None
    executor._work_active_received_wall = 0.0
    executor._work_station_started_wall = 0.0
    executor._work_timeout = 45.0
    executor._base_lock_observed = True
    executor._lock_received_wall_s = 0.0
    executor._parking_stability_anchor = None
    executor._work_permission = False
    executor._work_phase = 'WORKING'
    executor._worker_lease = WorkerLease()
    executor._motion_evidence = SimpleNamespace(ready=lambda *args: True)
    executor.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=2_000_000_000))
    executor._current_point = lambda: SimpleNamespace(waypoint_id='waypoint')
    executor._record = lambda *args: None
    executor._publish_status = lambda: None
    executor._parked_pose_status = lambda: ('valid', {'pose': (0.0, 0.0, 0.0)})
    executor.failures = []
    executor._fail = executor.failures.append
    executor.completions = []
    executor._finish_station = lambda: executor.completions.append(True)
    monkeypatch.setattr('agri_task_manager.field_route_executor.time.monotonic', lambda: 0.0)
    return executor


@pytest.mark.parametrize('service', [
    '_start_service', '_resume_service', '_skip_service', '_return_home_service',
    '_complete_current_service',
])
def test_legacy_service_cannot_bypass_active_work(service, monkeypatch):
    executor = _guarded_executor(monkeypatch)
    executor._worker_lease.update(
        _worker(2_000_000_000, executor._work_context), 2_000_000_000, 0.0)
    response = SimpleNamespace(success=None, message='')
    getattr(executor, service)(None, response)
    assert response.success is False
    assert executor.completions == []


def test_old_waypoint_result_cannot_complete_current_station(monkeypatch):
    executor = _guarded_executor(monkeypatch)
    executor._work_report_callback(String(data=json.dumps({
        'run_id': 'run', 'waypoint_id': 'previous', 'work_id': 'old',
        'stamp_ns': 2_000_000_000, 'phase': 'SUCCESS',
        'success': True, 'arm_stowed': True})))
    assert executor._work_result_pending is None
    assert not executor._work_terminal


def test_result_success_without_a_new_matching_stow_ack_keeps_base_locked(monkeypatch):
    executor = _guarded_executor(monkeypatch)
    executor._worker_lease.update(
        _worker(1_900_000_000, executor._work_context), 2_000_000_000, 0.0)
    executor._work_report_callback(String(data=json.dumps({
        'run_id': 'run', 'waypoint_id': 'waypoint', 'work_id': 'work',
        'stamp_ns': 2_000_000_000, 'phase': 'SUCCESS',
        'success': True, 'arm_stowed': True})))
    executor._check_harvest(0.0)
    assert executor.completions == []
    executor._worker_lease.update(
        _worker(2_010_000_000, executor._work_context), 2_010_000_000, 0.0)
    executor._check_harvest(0.0)
    assert executor.completions == [True]


def test_lost_worker_mid_work_fails_without_advancing(monkeypatch):
    executor = _guarded_executor(monkeypatch)
    executor._lock_received_wall_s = 1.0
    executor._check_harvest(1.0)
    assert executor.completions == []
    assert executor.failures == ['worker heartbeat lost or work identity changed']


def test_goal_transport_exception_does_not_wedge_coordinator_or_claim_terminal():
    """Clean its local pending future while leaving the executor fail-closed."""
    coordinator = object.__new__(HarvestCoordinator)
    coordinator._work_context = ('run', 'waypoint', 'work')
    coordinator._handled_contexts = set()
    coordinator._send_future = object()
    coordinator._request_started = 1.0
    reports = []
    coordinator._report = lambda phase, **details: reports.append((phase, details))

    def failed_result():
        raise RuntimeError('goal response transport lost')

    coordinator._accepted(SimpleNamespace(result=failed_result))
    assert reports[0][0] == 'FAILED'
    assert reports[0][1]['terminal_confirmed'] is False
    assert coordinator._send_future is None
    assert coordinator._request_started is None
    assert coordinator._work_context in coordinator._handled_contexts
