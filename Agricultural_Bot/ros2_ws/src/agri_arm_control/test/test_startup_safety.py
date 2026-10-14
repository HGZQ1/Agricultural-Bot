"""Restart and cold-start guards never infer a lease from a missing goal handle."""

from types import SimpleNamespace as NS
from concurrent.futures import Future
import threading

import pytest

from agri_arm_control.controller import ArmController, MotionError
from agri_arm_control.harvest_worker import HarvestArmWorker
from agri_arm_control.safety import ControllerGoalEvidence, JointEvidence


def test_both_real_status_snapshots_required_before_any_terminal_lease():
    evidence = ControllerGoalEvidence()
    assert not evidence.ready() and not evidence.terminal()
    evidence.update('arm', [])
    assert not evidence.ready() and not evidence.terminal()
    evidence.update('gripper', [(b'g'*16, 5)])
    assert evidence.ready() and evidence.terminal()


def test_foreign_active_or_canceling_goal_blocks_restart_even_if_robot_stopped():
    evidence = ControllerGoalEvidence()
    evidence.update('arm', [(b'a'*16, 3)])
    evidence.update('gripper', [])
    assert not evidence.ready() and not evidence.terminal()
    assert not evidence.idle_for_initialization()


def test_owned_active_goal_keeps_health_but_only_its_result_resolves_delayed_status():
    evidence = ControllerGoalEvidence()
    identifier = b'a'*16
    evidence.update('arm', [(identifier, 2)])
    evidence.update('gripper', [])
    assert evidence.ready({identifier})
    assert not evidence.terminal()
    assert evidence.terminal({identifier})
    evidence.update('gripper', [(b'external_id_00001', 2)])
    assert not evidence.terminal({identifier})


@pytest.mark.parametrize('stowed,terminal,moving', [(False, True, False),
                                                  (True, True, False),
                                                  (False, False, False),
                                                  (False, True, True)])
def test_startup_nonstow_fault_only_after_fresh_stopped_and_actual_terminal(
        monkeypatch, stowed, terminal, moving):
    wall = [0.]
    now = [1_000_000_000]
    import agri_arm_control.harvest_worker as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    worker = NS(_mutex=threading.RLock(), _busy=False, _fault=False, _healthy=True,
                _startup_checked=False, _phase='IDLE',
                p={'arm_joint_names': ['arm'], 'gripper_joint_names': ['gripper'],
                   'joint_velocity_tolerance_rps': .03, 'gripper_velocity_tolerance_mps': .003,
                   'joint_position_tolerance_rad': .03, 'stow_joints': [0.],
                   'joint_settle_s': .25}, evidence=JointEvidence(.5),
                arm=NS(terminal_confirmed=terminal, stowed=lambda: stowed),
                get_clock=lambda: NS(now=lambda: NS(nanoseconds=now[0])),
                get_logger=lambda: NS(warning=lambda _message: None))
    for index in range(4):
        wall[0], now[0] = index*.3, int((1.+index*.3)*1e9)
        worker.evidence.update(['arm', 'gripper'], [0. if stowed else 2., 0.],
                               [.1 if moving else 0., 0.],
                               now[0], now[0], wall[0])
        HarvestArmWorker._detect_startup_pose(worker)
    assert worker._fault == (not stowed and terminal and not moving)
    assert worker._startup_checked == (terminal and not moving)


def test_stale_or_incomplete_feedback_cannot_prove_stationary_pose():
    evidence = JointEvidence(.5)
    evidence.update(['arm'], [2.], [0.], 1_000_000_000, 1_000_000_000, 0.)
    assert not evidence.stationary(['arm', 'gripper'], 1_000_000_000, .3)
    assert not evidence.stationary(['arm'], 1_600_000_000, .6)


def test_cold_hold_at_stow_waits_for_pose_settlement_without_false_fault(monkeypatch):
    wall, now = [0.], [1_000_000_000]
    import agri_arm_control.harvest_worker as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    p = {'arm_joint_names': ['arm'], 'gripper_joint_names': ['gripper'],
         'joint_velocity_tolerance_rps': .03, 'gripper_velocity_tolerance_mps': .003,
         'joint_position_tolerance_rad': .03, 'stow_joints': [0.], 'joint_settle_s': .25}
    evidence = JointEvidence(.5)
    worker = NS(_mutex=threading.RLock(), _busy=False, _fault=False, _healthy=True,
                _startup_checked=False, _phase='IDLE', p=p, evidence=evidence,
                get_clock=lambda: NS(now=lambda: NS(nanoseconds=now[0])),
                get_logger=lambda: NS(warning=lambda _message: pytest.fail('pose is in stow')))
    worker.arm = NS(terminal_confirmed=True, stowed=lambda: evidence.at(
        ['arm'], [0.], now[0], wall[0], settle=.25))
    for index in range(5):
        wall[0], now[0] = index*.3, int((1.+index*.3)*1e9)
        evidence.update(['arm','gripper'], [.004,0.], [0.,0.], now[0], now[0], wall[0])
        HarvestArmWorker._detect_startup_pose(worker)
        assert not worker._fault
        if index == 2:  # velocity settled but at-pose key only just starts here
            assert not worker._startup_checked
    assert worker._startup_checked


@pytest.mark.parametrize('label,expected', [('arm', [2.4]), ('gripper', [.012])])
def test_initialization_hold_uses_current_feedback_and_never_stow(label, expected):
    controller = ArmController.__new__(ArmController)
    controller.joints, controller.gripper_joints = ['arm'], ['gripper']
    controller.p = {'gripper_max_m': .035}
    controller._check = lambda: None
    controller.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000)))
    controller.evidence = NS(snapshot=lambda *_args: {'arm': 2.4, 'gripper': .012})
    controller.arm_action, controller.gripper_action = object(), object()
    controller._initialization_idle_labels = {'arm', 'gripper'}
    sent = []
    controller._action = lambda _client, goal, *_args, **kwargs: sent.append((goal, kwargs))
    controller.hold_current_for_initialization(label)
    assert list(sent[0][0].trajectory.points[0].positions) == expected
    assert sent[0][1]['initialization_hold']


def test_cancel_ack_with_unknown_canceling_status_does_not_authorize_hold(monkeypatch):
    controller = ArmController.__new__(ArmController)
    controller._initialization_idle_labels = set()
    controller._uncertain_submission, controller._active_handle = False, None
    controller._controller_goals = ControllerGoalEvidence()
    controller._controller_goals.update('arm', [(b'a'*16, 3)])
    controller._terminal_goal_ids = set()
    controller._check = lambda: None
    controller.p = {'cancel_timeout_s': .1}
    controller._service = lambda *_args: NS(return_code=0, goals_canceling=[])
    elapsed = [0.]
    import agri_arm_control.controller as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: elapsed[0])
    monkeypatch.setattr(module.time, 'sleep', lambda _seconds: elapsed.__setitem__(0, elapsed[0]+.05))
    with pytest.raises(MotionError, match='terminal status'):
        controller.prepare_initialization_hold()
    assert not controller._initialization_idle_labels


def test_delayed_previous_terminal_callback_cannot_clear_a_new_goal():
    controller = ArmController.__new__(ArmController)
    controller._mutex = threading.RLock()
    controller._terminal_goal_ids = set()
    previous, current = Future(), Future()
    previous.set_result(NS(status=5))
    handle = NS(goal_id=NS(uuid=list(b'b'*16)))
    controller._active_result, controller._active_handle = current, handle
    controller._late_terminal(previous)
    assert controller._active_result is current and controller._active_handle is handle
    assert not controller._terminal_goal_ids
