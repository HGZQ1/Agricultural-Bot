"""Transit stow renews measured geometry, never a cached position or lease."""

from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState
import yaml

from agri_arm_control.controller import ArmController
from agri_arm_control.harvest_worker import HarvestArmWorker
from agri_arm_control.safety import JointEvidence


def fixture(monkeypatch):
    import agri_arm_control.controller as module
    state = NS(wall=0., now_ns=1_000_000_000)
    monkeypatch.setattr(module.time, 'monotonic', lambda: state.wall)
    arm = ArmController.__new__(ArmController)
    arm._mutex = threading.RLock()
    arm._uncertain_submission, arm._active_handle = False, None
    arm._navigation_stow_latched = False
    arm.joints = [f'joint_{index}' for index in range(6)]
    arm.p = {'stow_joints': [0.]*6, 'joint_position_tolerance_rad': .03,
             'joint_velocity_tolerance_rps': .03, 'joint_settle_s': .25,
             'navigation_stow_velocity_tolerance_rps': .15}
    arm.evidence = JointEvidence(.5)
    arm.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=state.now_ns)))
    def update(wall, position=0., velocity=0.):
        state.wall, state.now_ns = wall, round((1.+wall)*1e9)
        return arm.evidence.update(arm.joints, [position]+[0.]*5, [velocity]+[0.]*5,
                                   state.now_ns, state.now_ns, state.wall)
    update(0.)
    return arm, state, update


def acquire(arm, update):
    assert not arm.navigation_stowed()
    update(.3)
    assert arm.navigation_stowed()


def test_transit_velocity_cannot_acquire_an_initial_stow_latch(monkeypatch):
    arm, _, update = fixture(monkeypatch)
    update(.01, velocity=.1)
    assert not arm.navigation_stowed()
    report = arm.navigation_stow_report()
    assert report['envelope_valid'] and not report['strict_stow_latch_acquired']
    update(.1)
    assert not arm.navigation_stowed()
    update(.36)
    assert arm.navigation_stowed()


def test_already_verified_stow_survives_bounded_chassis_vibration(monkeypatch):
    arm, _, update = fixture(monkeypatch)
    acquire(arm, update)
    update(.4, position=.01, velocity=.1)
    assert arm.navigation_stowed()
    assert not arm.stowed()  # Strict work/recovery verification remains unchanged.
    assert 'velocity_above_stopped_tolerance' in arm.stow_report()['reasons']
    report = arm.navigation_stow_report()
    assert report['passed'] and report['velocity_tolerance_rps'] == .15
    assert report['position_tolerance_rad'] == .03


@pytest.mark.parametrize('position,velocity', [(.03001, 0.), (0., .15001)])
def test_envelope_violation_revokes_and_requires_new_strict_settlement(
        monkeypatch, position, velocity):
    arm, _, update = fixture(monkeypatch)
    acquire(arm, update)
    update(.4, position, velocity)
    assert not arm.navigation_stowed()
    assert not arm.navigation_stow_latched
    update(.5)
    assert not arm.navigation_stowed()
    update(.76)
    assert arm.navigation_stowed()


def test_stale_feedback_revokes_without_renewing_any_real_sample(monkeypatch):
    arm, state, update = fixture(monkeypatch)
    acquire(arm, update)
    receipt, stamp = arm.evidence.wall, arm.evidence.stamp
    state.wall, state.now_ns = .81, 1_810_000_000
    assert not arm.navigation_stowed()
    assert (arm.evidence.wall, arm.evidence.stamp) == (receipt, stamp)
    assert 'wall_feedback_stale' in arm.navigation_stow_report()['reasons']


def test_active_controller_goal_cannot_preserve_transit_stow(monkeypatch):
    arm, _, update = fixture(monkeypatch)
    acquire(arm, update)
    arm._active_handle = NS()
    assert not arm.navigation_stowed()
    assert 'controller_goal_not_confirmed_terminal' in arm.navigation_stow_report()['reasons']
    arm._active_handle = None
    update(.4)
    assert not arm.navigation_stowed()
    update(.7)
    assert arm.navigation_stowed()


def test_repeated_busy_resets_do_not_erase_actual_retract_settlement(monkeypatch):
    arm, _, update = fixture(monkeypatch)
    acquire(arm, update)
    arm.reset_navigation_stow()
    update(.4)
    assert not arm.stowed()
    arm.reset_navigation_stow()  # Busy heartbeat after the first revocation.
    update(.7)
    assert arm.stowed()
    assert arm.navigation_stowed()


def test_navigation_diagnostic_cannot_warm_or_reset_strict_settlement(monkeypatch):
    arm, _, _ = fixture(monkeypatch)
    original = dict(arm.evidence._settled), arm.evidence.wall, arm.evidence.stamp
    assert not arm.navigation_stow_report()['passed']
    assert (arm.evidence._settled, arm.evidence.wall, arm.evidence.stamp) == original


def test_invalid_joint_sample_between_heartbeats_revokes_latch_immediately(monkeypatch):
    arm, state, update = fixture(monkeypatch)
    acquire(arm, update)
    worker = NS(arm=arm, evidence=arm.evidence, get_clock=arm.node.get_clock)
    message = JointState(name=arm.joints, position=[.001]+[0.]*5, velocity=[0.]*6)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(state.now_ns, 1_000_000_000)
    state.wall = .31
    HarvestArmWorker._joint_callback(worker, message)  # Changed same-stamp data.
    assert not arm.navigation_stow_latched
    update(.32)
    assert not arm.navigation_stowed()
    update(.6)
    assert arm.navigation_stowed()


def test_exact_duplicate_joint_sample_neither_revokes_nor_renews_transit_evidence(monkeypatch):
    arm, state, update = fixture(monkeypatch)
    acquire(arm, update)
    worker = NS(arm=arm, evidence=arm.evidence, get_clock=arm.node.get_clock)
    message = JointState(name=arm.joints, position=[0.]*6, velocity=[0.]*6)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(state.now_ns, 1_000_000_000)
    state.wall = .4
    HarvestArmWorker._joint_callback(worker, message)
    assert arm.navigation_stow_latched and arm.navigation_stowed()
    assert arm.evidence.wall == .3


@pytest.mark.parametrize('limit', [.02, .25001, float('nan'), float('inf')])
def test_navigation_velocity_parameter_rejects_invalid_or_overbroad_envelope(limit):
    path = Path(__file__).parents[1] / 'config' / 'harvest_arm_worker.yaml'
    params = yaml.safe_load(path.read_text())['harvest_arm_worker']['ros__parameters']
    params['navigation_stow_velocity_tolerance_rps'] = limit
    with pytest.raises(ValueError, match='navigation stow velocity'):
        HarvestArmWorker._validate_parameters(NS(p=params))
