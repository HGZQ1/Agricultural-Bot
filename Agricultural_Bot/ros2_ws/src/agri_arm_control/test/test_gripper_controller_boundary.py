"""Controller outcomes cannot substitute for independently measured contact."""

from concurrent.futures import Future
from types import SimpleNamespace as NS
import threading

import pytest

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from agri_arm_control.controller import ArmController, ControllerToleranceError, MotionError
from agri_arm_control.safety import JointEvidence


def action_fixture(status, error_code):
    controller = ArmController.__new__(ArmController)
    controller.node = NS(context=NS(ok=lambda: True))
    controller.p = {'goal_response_timeout_s': .01, 'cancel_timeout_s': .01}
    controller.cancellation = threading.Event()
    controller.check_permit = lambda: True
    controller._mutex = threading.RLock()
    controller._uncertain_submission = False
    controller._active_handle = controller._active_result = None
    result_future = Future()
    result_future.set_result(NS(status=status, result=NS(error_code=error_code, error_string='')))
    handle = NS(accepted=True, get_result_async=lambda: result_future)
    submission = Future()
    submission.set_result(handle)
    client = NS(server_is_ready=lambda: True, send_goal_async=lambda _goal: submission)
    return controller, client


@pytest.mark.parametrize('error_code', [
    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
])
def test_only_real_terminal_gripper_tolerance_abort_is_contact_query_candidate(error_code):
    controller, client = action_fixture(GoalStatus.STATUS_ABORTED, error_code)
    with pytest.raises(ControllerToleranceError) as caught:
        controller._action(client, object(), .02, 'gripper', lambda result: result.error_code == 0)
    assert caught.value.controller == 'gripper'
    assert caught.value.error_code == error_code
    assert controller.terminal_confirmed


@pytest.mark.parametrize('label,status,error_code', [
    ('arm', GoalStatus.STATUS_ABORTED, -4),
    ('arm', GoalStatus.STATUS_ABORTED, -5),
    ('gripper', GoalStatus.STATUS_ABORTED, -1),
    ('gripper', GoalStatus.STATUS_ABORTED, -2),
    ('gripper', GoalStatus.STATUS_ABORTED, -3),
    ('gripper', GoalStatus.STATUS_CANCELED, -5),
    ('gripper', GoalStatus.STATUS_SUCCEEDED, -5),
])
def test_other_controller_failures_cannot_be_reinterpreted_as_contact(label, status, error_code):
    controller, client = action_fixture(status, error_code)
    with pytest.raises(MotionError) as caught:
        controller._action(client, object(), .02, label, lambda result: result.error_code == 0)
    assert not isinstance(caught.value, ControllerToleranceError)


def test_lost_parking_permission_takes_precedence_over_tolerance_abort():
    controller, client = action_fixture(GoalStatus.STATUS_ABORTED, -5)
    checks = [0]

    def permit():
        checks[0] += 1
        return checks[0] == 1

    controller.check_permit = permit
    with pytest.raises(MotionError, match='base permit lost') as caught:
        controller._action(client, object(), .02, 'gripper', lambda result: result.error_code == 0)
    assert not isinstance(caught.value, ControllerToleranceError)


def position_fixture():
    controller = ArmController.__new__(ArmController)
    controller.gripper_joints = ['left', 'right']
    controller.gripper_action = object()
    controller.p = {
        'gripper_max_m': .035, 'gripper_move_s': 1., 'gripper_timeout_s': 4.,
        'gripper_wall_timeout_s': 20., 'feedback_timeout_s': 3.,
        'gripper_position_tolerance_m': .002, 'gripper_velocity_tolerance_mps': .003,
    }
    return controller


def test_small_closure_tolerance_applies_to_action_goal_and_measured_settlement():
    controller = position_fixture()
    submitted, waited = [], []
    controller._action = lambda _client, goal, *_args, **_kwargs: submitted.append(goal)
    controller.wait_joints = lambda *args, **kwargs: waited.append((args, kwargs))
    controller.gripper_positions([.001, .001], .25, position_tolerance_m=.00025)
    assert [item.name for item in submitted[0].goal_tolerance] == ['left', 'right']
    assert all(item.position == pytest.approx(.00025) for item in submitted[0].goal_tolerance)
    assert waited[0][1]['position_tolerance'] == pytest.approx(.00025)


@pytest.mark.parametrize('positions', [[0., 0.], [.009, .011]])
def test_opening_and_measured_hold_do_not_swallow_tolerance_abort(positions):
    controller = position_fixture()
    waited = []

    def abort(*_args, **_kwargs):
        raise ControllerToleranceError('gripper', GoalStatus.STATUS_ABORTED, -5)

    controller._action = abort
    controller.wait_joints = lambda *_args, **_kwargs: waited.append(True)
    with pytest.raises(ControllerToleranceError):
        controller.gripper_positions(positions, .25)
    assert waited == []


def test_stopped_finger_evidence_requires_fresh_feedback_and_actual_velocity_bound(monkeypatch):
    import agri_arm_control.controller as module

    controller = position_fixture()
    controller.p['joint_settle_s'] = .25
    controller._mutex = threading.RLock()
    controller._uncertain_submission = False
    controller._active_handle = controller._active_result = None
    controller.cancellation = threading.Event()
    controller.check_permit = lambda: True
    controller.evidence = JointEvidence(max_age=.5)
    wall = [0.]
    stamp = [1_000_000_000]
    controller.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=stamp[0])))
    # 0.0025 m/s meets the normal configured 0.003 limit, but cannot establish
    # the stricter stopped prerequisite for physical contact recovery.
    assert controller.evidence.update(['left', 'right'], [.009, .011], [.0025, 0.],
                                      stamp[0], stamp[0], wall[0])
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(module.time, 'sleep', lambda seconds: wall.__setitem__(0, wall[0]+seconds))
    with pytest.raises(MotionError, match='stopped dual-finger'):
        controller.wait_gripper_stopped(.3)
    wall[0] = .6
    # Even zero velocity cannot establish stopped state from stale receipt.
    controller.evidence.values = {'left': (.009, 0.), 'right': (.011, 0.)}
    with pytest.raises(MotionError, match='stopped dual-finger'):
        controller.wait_gripper_stopped(.9)


def test_stopped_measurements_cannot_authorize_recovery_with_active_controller(monkeypatch):
    import agri_arm_control.controller as module

    controller = position_fixture()
    controller.p['joint_settle_s'] = .25
    controller._mutex = threading.RLock()
    controller._uncertain_submission = False
    controller._active_handle = object()
    controller.cancellation = threading.Event()
    controller.check_permit = lambda: True
    controller.evidence = NS(stationary=lambda *_args: pytest.fail('active goal must reject first'))
    monkeypatch.setattr(module.time, 'monotonic', lambda: 0.)
    with pytest.raises(MotionError, match='terminal controller'):
        controller.wait_gripper_stopped(1.)
