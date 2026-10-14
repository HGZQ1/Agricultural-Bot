"""Only observational readiness/grip queries permit bounded transport retries."""

from types import SimpleNamespace as NS

import pytest

from agri_arm_control.controller import MotionError
from agri_arm_control.harvest_worker import HarvestArmWorker
from agri_arm_control.safety import clamp_gripper_measurement


def fixture(monkeypatch, replies):
    import agri_arm_control.harvest_worker as module
    wall, calls, checks = [0.], [], []
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(module.time, 'sleep', lambda seconds: wall.__setitem__(0, wall[0]+seconds))
    def wait(_future, _timeout, operation, _check):
        calls.append(operation)
        reply = replies[len(calls)-1] if len(calls) <= len(replies) else replies[-1]
        if isinstance(reply, Exception):
            raise reply
        return reply
    client = NS(service_is_ready=lambda: True, call_async=lambda _request: NS())
    worker = NS(p={'scene_service_timeout_s': .6, 'scene_ready_attempt_timeout_s': .3,
                   'scene_ready_retry_interval_s': .2},
                _scene_clients={'contact': client, 'fruit': client},
                arm=NS(_check=lambda: checks.append(True), _wait=wait),
                _scene_request=lambda operation: operation,
                _transport_timeout=HarvestArmWorker._transport_timeout,
                _scene_ready_diagnostics={})
    return worker, calls, checks, wall


def busy():
    return NS(success=False, state='BUSY', message='scene transaction busy')


def ready():
    return NS(success=True, state='SCENE_READY', message='acknowledged')


@pytest.mark.parametrize('first', [busy(), MotionError('ready response timed out'),
    NS(success=False, state='FAILED', message='Gazebo harvest service unavailable: Service call timed out')])
def test_ready_retries_busy_or_transport_timeout_and_requires_new_ack(monkeypatch, first):
    worker, calls, checks, _wall = fixture(monkeypatch, [first, ready()])
    assert HarvestArmWorker._scene(worker, 'ready').state == 'SCENE_READY'
    assert calls == ['ready', 'ready'] and len(checks) > 2
    assert worker._scene_ready_diagnostics['attempts'] == 2


@pytest.mark.parametrize('operation,reply', [('select', busy()),
    ('attach', MotionError('attach response timed out')),
    ('release', MotionError('release response timed out'))])
def test_contact_and_physical_commands_never_blindly_retry(monkeypatch, operation, reply):
    worker, calls, _checks, _wall = fixture(monkeypatch, [reply, ready()])
    with pytest.raises(MotionError):
        HarvestArmWorker._scene(worker, operation)
    assert calls == [operation]


def test_busy_ready_remains_bounded_by_total_wall_deadline(monkeypatch):
    worker, calls, _checks, wall = fixture(monkeypatch, [busy()])
    with pytest.raises(MotionError, match='total scene deadline'):
        HarvestArmWorker._scene(worker, 'ready')
    assert 1 < len(calls) <= 4 and wall[0] <= .62


def test_ready_retry_stops_immediately_when_parking_permit_is_lost(monkeypatch):
    worker, calls, _checks, _wall = fixture(monkeypatch, [busy(), ready()])
    checks = [0]
    def permit():
        checks[0] += 1
        if checks[0] > 1:
            raise MotionError('fresh locked and stationary base permit lost')
    worker.arm._check = permit
    with pytest.raises(MotionError, match='permit lost'):
        HarvestArmWorker._scene(worker, 'ready')
    assert calls == ['ready']


def test_read_only_grip_query_retries_busy_within_its_short_explicit_deadline(monkeypatch):
    confirmed = NS(success=True, state='CONTACT_CONFIRMED', message='real contacts')
    worker, calls, checks, _wall = fixture(monkeypatch, [busy(), confirmed])
    assert HarvestArmWorker._scene(worker, 'check_grip', timeout_s=.5).state == 'CONTACT_CONFIRMED'
    assert calls == ['check_grip', 'check_grip'] and len(checks) > 2
    worker, calls, _checks, wall = fixture(monkeypatch, [busy()])
    with pytest.raises(MotionError, match='total scene deadline'):
        HarvestArmWorker._scene(worker, 'check_grip', timeout_s=.3)
    assert wall[0] <= .32 and len(calls) == 2


def test_gripper_hold_clamps_only_micrometre_numerical_limit_noise():
    assert clamp_gripper_measurement(-1.8e-18, .035) == 0.
    assert clamp_gripper_measurement(.035+5e-7, .035) == .035
    assert clamp_gripper_measurement(.012, .035) == .012
    for value in (-2e-6, .035+2e-6, float('nan')):
        with pytest.raises(ValueError):
            clamp_gripper_measurement(value, .035)
