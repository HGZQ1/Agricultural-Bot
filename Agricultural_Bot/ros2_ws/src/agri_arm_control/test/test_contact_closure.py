"""A single-finger contact or failed controller result never becomes a grasp."""

from types import SimpleNamespace as NS

import pytest

from agri_arm_control.contact_closure import ContactCloser, next_finger_goals
from agri_arm_control.controller import (
    ArmController, ControllerToleranceError, JointSettlementTimeout, MotionError)


def fixture(states, budget=5):
    positions = [.003, .001]
    sent, observed = [], []
    params = {'gripper_contact_timeout_s': 120., 'maximum_gripper_contact_steps': budget,
              'gripper_contact_step_m': .001, 'gripper_contact_step_s': .25,
              'gripper_max_m': .035}
    def move(goals, seconds, deadline_wall, position_tolerance_m=None):
        sent.append(list(goals))
        assert seconds == .25 and deadline_wall > 0
        if position_tolerance_m is not None:
            assert 0 < position_tolerance_m <= .00025
        positions[:] = goals
    arm = NS(_check=lambda: None, measured_gripper_positions=lambda: list(positions),
             gripper_positions=move)
    def scene(operation, timeout_s):
        assert operation == 'check_grip' and 0 < timeout_s <= 5.
        state = states.pop(0) if len(states) > 1 else states[0]
        observed.append(state)
        return NS(success=True, state=state, message='actual collision contacts')
    return ContactCloser(arm, params, scene), sent, observed


def test_contacted_finger_freezes_actual_position_while_other_closes():
    assert next_finger_goals([.011, .003], [True,False], .001, .035) == [.011,.004]
    assert next_finger_goals([.011, .003], [False,True], .001, .035) == [.012,.003]


@pytest.mark.parametrize('actual,step', [([.035,.035],.001), ([float('nan'),.003],.001),
                                        ([.011,.003],.003)])
def test_invalid_or_exhausted_travel_cannot_create_a_step(actual, step):
    with pytest.raises(MotionError):
        next_finger_goals(actual,[False,False],step,.035)


def test_both_real_contacts_and_successful_current_position_hold_required():
    closer, sent, observed = fixture([
        'NO_CONTACT','LEFT_CONTACT','CONTACT_SETTLING','CONTACT_SETTLING','CONTACT_CONFIRMED'])
    assert closer.close()
    assert sent == [[.004,.002],[.004,.003],[.004,.003]]
    assert observed[-1] == 'CONTACT_CONFIRMED'


def test_single_touch_has_bounded_steps_and_never_reports_success():
    closer, sent, _ = fixture(['LEFT_CONTACT'],budget=3)
    with pytest.raises(MotionError,match='step budget'):
        closer.close()
    assert len(sent) == 3 and all(goal[0] == .003 for goal in sent)


def test_untyped_controller_error_propagates_without_contact_recovery():
    closer, sent, observed = fixture(['NO_CONTACT'])
    def abort(*_args, **_kwargs):
        raise MotionError('gripper controller aborted -5')
    closer.arm.gripper_positions = abort
    with pytest.raises(MotionError,match='aborted -5'):
        closer.close()
    assert not sent and observed == ['NO_CONTACT']


def _contact_stall_fixture(states, error, *, terminal=True, budget=5):
    closer, sent, observed = fixture(states, budget=budget)
    move = closer.arm.gripper_positions
    attempted = []
    gates = []
    def close_step(goals, seconds, **kwargs):
        attempted.append((list(goals), kwargs.get('position_tolerance_m')))
        if len(attempted) == 1:
            raise error
        return move(goals, seconds, **kwargs)
    closer.arm.gripper_positions = close_step
    closer.arm.cancel_and_confirm = lambda: gates.append('terminal') or terminal
    closer.arm.wait_gripper_stopped = lambda _deadline: (
        gates.append('stopped') or closer.arm.measured_gripper_positions())
    return closer, sent, observed, attempted, gates


@pytest.mark.parametrize('error', [
    ControllerToleranceError('gripper', 6, -4),
    ControllerToleranceError('gripper', 6, -5),
    JointSettlementTimeout('measured joint position/velocity settlement timed out'),
])
def test_blocked_close_can_recover_only_after_terminal_stop_and_real_double_contact(error):
    closer, sent, observed, attempted, gates = _contact_stall_fixture(
        ['NO_CONTACT', 'CONTACT_SETTLING', 'CONTACT_CONFIRMED'], error)
    records = []
    closer.diagnostic = records.append
    assert closer.close()
    assert gates == ['terminal', 'stopped']
    assert attempted[0][1] == .00025
    assert attempted[1][1] is None  # measured-position hold uses normal tolerance
    assert sent == [[.003, .001]]
    assert observed[-1] == 'CONTACT_CONFIRMED'
    assert records[-1]['recovery_count'] == 1
    assert records[-1]['last_recovery']['controller_terminal_confirmed']
    assert records[-1]['last_recovery']['native_contact_message']


def test_controller_tolerance_failure_with_no_physical_contact_still_fails():
    error = ControllerToleranceError('gripper', 6, -5)
    closer, sent, observed, _, gates = _contact_stall_fixture(['NO_CONTACT'], error)
    with pytest.raises(ControllerToleranceError) as caught:
        closer.close()
    assert caught.value is error
    assert not sent and observed == ['NO_CONTACT', 'NO_CONTACT']
    assert gates == ['terminal', 'stopped']


def test_single_contact_recovery_holds_actual_then_closes_only_other_finger():
    closer, sent, observed, attempted, _ = _contact_stall_fixture(
        ['NO_CONTACT', 'LEFT_CONTACT', 'LEFT_CONTACT', 'CONTACT_CONFIRMED',
         'CONTACT_CONFIRMED'], ControllerToleranceError('gripper', 6, -5))
    assert closer.close()
    # Failed initial step never establishes grasp; successful hold resets the
    # command, then measured left stays fixed while right closes by 1 mm.
    assert sent == [[.003, .001], [.003, .002], [.003, .002]]
    assert attempted[2] == ([.003, .002], .00025)
    assert observed[-1] == 'CONTACT_CONFIRMED'


def test_single_contact_stall_recovery_never_bypasses_closure_step_budget():
    closer, sent, observed, _, _ = _contact_stall_fixture(
        ['NO_CONTACT', 'LEFT_CONTACT'], ControllerToleranceError('gripper', 6, -5), budget=1)
    with pytest.raises(MotionError, match='step budget'):
        closer.close()
    assert sent == [[.003, .001]]
    assert 'CONTACT_CONFIRMED' not in observed


def test_contact_stop_recovery_does_not_query_or_hold_without_confirmed_terminal():
    closer, sent, observed, _, gates = _contact_stall_fixture(
        ['NO_CONTACT', 'CONTACT_CONFIRMED'], ControllerToleranceError('gripper', 6, -5),
        terminal=False)
    with pytest.raises(MotionError, match='confirmed controller terminal'):
        closer.close()
    assert not sent and observed == ['NO_CONTACT'] and gates == ['terminal']


def test_contact_stop_recovery_requires_fresh_stopped_finger_feedback():
    closer, sent, observed, _, _ = _contact_stall_fixture(
        ['NO_CONTACT', 'CONTACT_CONFIRMED'], ControllerToleranceError('gripper', 6, -5))
    closer.arm.wait_gripper_stopped = lambda _deadline: (
        _ for _ in ()).throw(MotionError('fresh stopped finger feedback unavailable'))
    with pytest.raises(MotionError, match='fresh stopped finger'):
        closer.close()
    assert not sent and observed == ['NO_CONTACT']


def test_measured_hold_failure_after_real_double_contact_is_not_recoverable():
    closer, sent, observed, attempted, _ = _contact_stall_fixture(
        ['NO_CONTACT', 'CONTACT_CONFIRMED'], ControllerToleranceError('gripper', 6, -5))
    failed_step = closer.arm.gripper_positions
    def hold_fails(*args, **kwargs):
        if kwargs.get('position_tolerance_m') is None:
            raise ControllerToleranceError('gripper', 6, -5, 'actual hold aborted')
        return failed_step(*args, **kwargs)
    closer.arm.gripper_positions = hold_fails
    with pytest.raises(ControllerToleranceError, match='actual hold aborted'):
        closer.close()
    assert not sent and len(attempted) == 1
    assert observed == ['NO_CONTACT', 'CONTACT_CONFIRMED']


@pytest.mark.parametrize('error', [
    ControllerToleranceError('arm', 6, -5),
    ControllerToleranceError('gripper', 5, -5),
    ControllerToleranceError('gripper', 6, -3),
])
def test_other_controller_and_non_tolerance_terminal_failures_cannot_use_contact_recovery(error):
    closer, sent, observed, _, gates = _contact_stall_fixture(
        ['NO_CONTACT', 'CONTACT_CONFIRMED'], error)
    with pytest.raises(ControllerToleranceError) as caught:
        closer.close()
    assert caught.value is error
    assert not sent and observed == ['NO_CONTACT'] and not gates


def test_parking_lease_loss_after_blocked_close_cannot_start_contact_recovery():
    closer, sent, observed, _, gates = _contact_stall_fixture(
        ['NO_CONTACT', 'CONTACT_CONFIRMED'], ControllerToleranceError('gripper', 6, -5))
    original = closer.arm.gripper_positions
    def loses_permit(*args, **kwargs):
        closer.arm._check = lambda: (_ for _ in ()).throw(MotionError('parking permit lost'))
        return original(*args, **kwargs)
    closer.arm.gripper_positions = loses_permit
    with pytest.raises(MotionError, match='parking permit lost'):
        closer.close()
    assert not sent and observed == ['NO_CONTACT'] and not gates


def test_contact_loss_after_hold_is_failure():
    closer, sent, _ = fixture(['CONTACT_CONFIRMED','LEFT_CONTACT'])
    with pytest.raises(MotionError,match='contact lost'):
        closer.close()
    assert sent == [[.003,.001]]


@pytest.mark.parametrize('state,success', [('UNAVAILABLE',True), ('CONTACT_REJECTED',False)])
def test_invalid_contact_query_fails_before_joint_movement(state, success):
    closer, sent, _ = fixture(['NO_CONTACT'])
    closer.scene = lambda *_args, **_kwargs: NS(state=state,success=success)
    with pytest.raises(MotionError,match='invalid physical'):
        closer.close()
    assert not sent


def test_rejected_native_contact_query_preserves_physical_rejection_reason():
    closer, sent, _ = fixture(['NO_CONTACT'])
    closer.scene = lambda *_args, **_kwargs: NS(
        state='CONTACT_REJECTED', success=False, message='selected fruit touches palm collision')
    with pytest.raises(MotionError, match='touches palm collision'):
        closer.close()
    assert not sent


def test_settling_double_contact_holds_once_and_cannot_wait_indefinitely(monkeypatch):
    import agri_arm_control.contact_closure as module
    wall = [0.]
    monkeypatch.setattr(module.time,'monotonic',lambda: wall[0])
    monkeypatch.setattr(module.time,'sleep',lambda seconds: wall.__setitem__(0,wall[0]+seconds))
    closer, sent, _ = fixture(['CONTACT_SETTLING'])
    with pytest.raises(MotionError,match='wall deadline'):
        closer.close()
    assert sent == [[.003,.001]] and wall[0] <= 5.001


def test_lease_loss_cancels_closing_before_another_contact_query():
    closer, sent, observed = fixture(['NO_CONTACT'])
    closer.arm._check = lambda: (_ for _ in ()).throw(MotionError('parking permit lost'))
    with pytest.raises(MotionError,match='permit lost'):
        closer.close()
    assert not sent and not observed


def test_dual_finger_controller_goal_keeps_independent_positions_and_real_result_gate():
    arm = ArmController.__new__(ArmController)
    arm.p = {'gripper_max_m':.035,'gripper_move_s':1.,'gripper_timeout_s':4.,
             'gripper_wall_timeout_s':20.,'feedback_timeout_s':3.,
             'gripper_position_tolerance_m':.002,'gripper_velocity_tolerance_mps':.003}
    arm.gripper_joints = ['left','right']
    arm.gripper_action = object()
    sent, feedback, feedback_tolerances = [], [], []
    arm._action = lambda _client, goal, _timeout, _label, successful, **kwargs: (
        sent.append(goal) or successful(NS(error_code=0)))
    def measured(names, positions, *_args, **kwargs):
        feedback.append(list(positions))
        feedback_tolerances.append(kwargs['position_tolerance'])
    arm.wait_joints = measured
    arm.gripper_positions([.011,.003],.25)
    assert list(sent[0].trajectory.points[0].positions) == [.011,.003]
    assert feedback == [[.011,.003]]
    assert all(tolerance.position == .002 for tolerance in sent[0].goal_tolerance)
    arm.gripper_positions([.012,.004],.25,position_tolerance_m=.00025)
    assert all(tolerance.position == .00025 for tolerance in sent[1].goal_tolerance)
    assert feedback_tolerances == [.002,.00025]
    with pytest.raises(MotionError,match='outside calibrated'):
        arm.gripper_positions([.011,float('nan')],.25)
    assert len(sent) == 2
