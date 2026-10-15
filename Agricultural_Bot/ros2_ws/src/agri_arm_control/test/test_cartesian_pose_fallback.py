"""A basket path fallback must reach the full pose without executing a prefix.

These fixtures never construct a ROS node or submit a controller trajectory.
"""

import threading
from types import SimpleNamespace as NS

import pytest
from builtin_interfaces.msg import Time
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from agri_arm_control.controller import ArmController, MotionError


POSITION = (0.17, 0.0, 0.60)
ORIENTATION = (1.0, 0.0, 0.0, 0.0)


def cartesian_fixture(fraction=1.0, error_code=1, positions=(0.0, 0.1)):
    controller = ArmController.__new__(ArmController)
    warnings, events, requests = [], [], []
    controller.node = NS(
        get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=7))),
        get_logger=lambda: NS(warning=warnings.append, warn=warnings.append),
    )
    controller.p = {
        'arm_group': 'arm', 'cartesian_step_m': .005,
        'cartesian_joint_jump_rad': .35, 'velocity_scaling': .2,
        'acceleration_scaling': .2, 'plan_timeout_s': 5.,
        'feedback_timeout_s': 3.,
    }
    controller.frame, controller.tip = 'base_link', 'grasp_tcp'
    controller.joints = ['arm_joint1']
    controller._mutex = threading.RLock()
    controller._execution_status = {}
    controller._check = lambda: events.append(('checked',))
    controller.robot_state = lambda: RobotState(is_diff=True)
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = controller.joints
    trajectory.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[position]) for position in positions]
    result = NS(error_code=NS(val=error_code), fraction=fraction, solution=trajectory)

    def service(key, request, timeout):
        requests.append((key, request, timeout))
        return result

    controller._service = service
    controller.execute = lambda planned: events.append(('execute', planned))
    controller.wait_joints = lambda *args: events.append(('settlement', args))
    controller.move_pose = lambda position, orientation: events.append(
        ('full_pose', position, orientation))
    return controller, events, requests, trajectory


def test_partial_basket_line_is_discarded_before_full_original_pose_is_planned():
    controller, events, requests, _ = cartesian_fixture(fraction=.2667)

    controller.cartesian(POSITION, ORIENTATION, pose_fallback=True)

    assert [event for event in events if event[0] != 'checked'] == [
        ('full_pose', POSITION, ORIENTATION)]
    assert any(event[0] == 'checked' for event in events)
    request = requests[0][1]
    assert request.avoid_collisions
    assert request.start_state.is_diff  # Preserve the live attached fruit.
    assert len(request.waypoints) == 1
    assert (request.waypoints[0].position.x, request.waypoints[0].position.y,
            request.waypoints[0].position.z) == POSITION


def test_ordinary_cartesian_motion_still_rejects_an_incomplete_path():
    controller, events, _, _ = cartesian_fixture(fraction=.2667)

    with pytest.raises(MotionError, match='Cartesian path incomplete'):
        controller.cartesian(POSITION, ORIENTATION)

    assert not [event for event in events if event[0] != 'checked']


def test_full_cartesian_path_executes_and_checks_measured_endpoint_without_fallback():
    controller, events, _, trajectory = cartesian_fixture()

    controller.cartesian(POSITION, ORIENTATION, pose_fallback=True)

    assert [event for event in events if event[0] != 'checked'] == [
        ('execute', trajectory),
        ('settlement', (trajectory.joint_trajectory.joint_names,
                        trajectory.joint_trajectory.points[-1].positions, 3.))]


@pytest.mark.parametrize('failing_stage', ['execute', 'wait_joints'])
def test_failure_after_motion_submission_never_changes_to_another_path(failing_stage):
    controller, events, _, _ = cartesian_fixture()

    def fail(*_args):
        raise MotionError('submitted motion failed')

    setattr(controller, failing_stage, fail)
    with pytest.raises(MotionError, match='submitted motion failed'):
        controller.cartesian(POSITION, ORIENTATION, pose_fallback=True)

    assert not any(event[0] == 'full_pose' for event in events)


def test_full_pose_planning_failure_propagates_without_executing_partial_cartesian():
    controller, events, _, _ = cartesian_fixture(fraction=.2667)

    def fail(*_args):
        raise MotionError('full basket pose is unavailable')

    controller.move_pose = fail
    with pytest.raises(MotionError, match='full basket pose is unavailable'):
        controller.cartesian(POSITION, ORIENTATION, pose_fallback=True)

    assert not any(event[0] in {'execute', 'settlement'} for event in events)


def test_joint_jump_rejects_original_path_and_can_plan_full_pose_before_motion():
    controller, events, _, _ = cartesian_fixture(positions=(0.0, .6))

    controller.cartesian(POSITION, ORIENTATION, pose_fallback=True)

    assert [event for event in events if event[0] != 'checked'] == [
        ('full_pose', POSITION, ORIENTATION)]


def test_joint_jump_remains_rejected_for_strict_approach_and_retreat():
    controller, events, _, _ = cartesian_fixture(positions=(0.0, .6))

    with pytest.raises(MotionError, match='joint jump'):
        controller.cartesian(POSITION, ORIENTATION)

    assert not [event for event in events if event[0] != 'checked']


def test_permit_loss_before_fallback_cannot_submit_the_alternate_plan():
    controller, events, _, _ = cartesian_fixture(fraction=.2667)

    def no_permit():
        raise MotionError('station permit lost')

    controller._check = no_permit
    with pytest.raises(MotionError, match='station permit lost'):
        controller.cartesian(POSITION, ORIENTATION, pose_fallback=True)

    assert not any(event[0] in {'execute', 'full_pose'} for event in events)
