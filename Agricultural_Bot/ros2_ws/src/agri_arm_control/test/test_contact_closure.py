"""A single-finger contact or failed controller result never becomes a grasp."""

from types import SimpleNamespace as NS

import pytest

from agri_arm_control.contact_closure import ContactCloser, next_finger_goals
from agri_arm_control.controller import ArmController, MotionError


def fixture(states, budget=5):
    positions = [.003, .001]
    sent, observed = [], []
    params = {'gripper_contact_timeout_s': 120., 'maximum_gripper_contact_steps': budget,
              'gripper_contact_step_m': .001, 'gripper_contact_step_s': .25,
              'gripper_max_m': .035}
    def move(goals, seconds, deadline_wall):
        sent.append(list(goals))
        assert seconds == .25 and deadline_wall > 0
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


def test_any_controller_abort_propagates_without_retry_or_attachment():
    closer, sent, observed = fixture(['NO_CONTACT'])
    def abort(*_args, **_kwargs):
        raise MotionError('gripper controller aborted -5')
    closer.arm.gripper_positions = abort
    with pytest.raises(MotionError,match='aborted -5'):
        closer.close()
    assert not sent and observed == ['NO_CONTACT']


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
    sent, feedback = [], []
    arm._action = lambda _client, goal, _timeout, _label, successful, **kwargs: (
        sent.append(goal) or successful(NS(error_code=0)))
    arm.wait_joints = lambda names, positions, *_args, **_kwargs: feedback.append(list(positions))
    arm.gripper_positions([.011,.003],.25)
    assert list(sent[0].trajectory.points[0].positions) == [.011,.003]
    assert feedback == [[.011,.003]]
    with pytest.raises(MotionError,match='outside calibrated'):
        arm.gripper_positions([.011,float('nan')],.25)
    assert len(sent) == 1
