"""Regression checks for geometry, stale feedback and false-success hazards."""

from concurrent.futures import Future
from types import SimpleNamespace as NS
import threading
import time

import pytest

from agri_arm_control.controller import ArmController, MotionError
from agri_arm_control.geometry import basket_poses, grasp_candidates, rotate, transform_pose
from agri_arm_control.grasp_executor import GraspExecutor
from agri_arm_control.safety import JointEvidence, station_matches
from moveit_msgs.msg import RobotTrajectory, RobotState
from builtin_interfaces.msg import Time
from trajectory_msgs.msg import JointTrajectoryPoint


def test_basket_raises_standard_z_then_tf_rotates_to_cad_y():
    drop, above = basket_poses()
    assert above[2]-drop[2] == pytest.approx(.15)
    # inverse of CAD->ROS rpy(pi/2,0,pi/2): ROS up becomes CAD +Y
    transform = NS(transform=NS(rotation=NS(x=-.5, y=-.5, z=-.5, w=.5),
                                translation=NS(x=0., y=0., z=0.)))
    cad_drop, _ = transform_pose(drop, (0., 0., 0., 1.), transform)
    cad_above, _ = transform_pose(above, (0., 0., 0., 1.), transform)
    assert cad_above[1]-cad_drop[1] == pytest.approx(.15)
    assert cad_above[2] == pytest.approx(cad_drop[2])


@pytest.mark.parametrize('side,sign', [('left', 1), ('right', -1)])
def test_tcp_approach_axis_and_pregrasp_side(side, sign):
    candidate = next(grasp_candidates((-.18, sign*.65, .95), side))
    assert rotate(candidate['orientation'], (0., 0., 1.)) == pytest.approx((0., sign, 0.))
    assert sign*candidate['pre'][1] < sign*candidate['centre'][1]
    assert candidate['centre'][1] == pytest.approx(sign*.675)


def test_position_alone_cannot_supply_stow_and_stale_velocity_revokes():
    evidence = JointEvidence(max_age=.5)
    assert not evidence.update(['j'], [0.], [], 1_000_000_000, 1_000_000_000, 0.)
    assert evidence.update(['j'], [0.], [.0], 1_000_000_000, 1_000_000_000, 0.)
    assert not evidence.at(['j'], [0.], 1_000_000_000, 0., settle=.25)
    assert evidence.update(['j'], [0.], [.0], 1_300_000_000, 1_300_000_000, .3)
    assert evidence.at(['j'], [0.], 1_300_000_000, .3, settle=.25)
    assert not evidence.at(['j'], [0.], 1_900_000_000, .9, settle=.25)
    assert evidence.update(['j'], [0.], [.1], 2_000_000_000, 2_000_000_000, 1.)
    assert not evidence.at(['j'], [0.], 2_000_000_000, 1., settle=.25)


def test_nonce_ack_and_lock_required_even_for_valid_target():
    request = NS(run_id='r', work_id='w', waypoint_id='p', plant_id='plant', work_side='left')
    station = dict(vars(request), active=True, station_started_ns=1_000_000_000)
    route = dict(run_id='r', work_id='w', current={'waypoint_id': 'p'},
                 require_harvest_handshake=True, state='AT_STATION', base_lock_observed=True,
                 status_stamp_ns=2_000_000_000, work_phase='WORKING', work_terminal_confirmed=False)
    assert station_matches(route, station, request, 2_000_000_000, 1., 1.)
    route['work_terminal_confirmed'] = True
    assert not station_matches(route, station, request, 2_000_000_000, 1., 1.)
    route['work_terminal_confirmed'] = False
    route['work_id'] = 'old'
    assert not station_matches(route, station, request, 2_000_000_000, 1., 1.)


def controller_fixture():
    controller = ArmController.__new__(ArmController)
    controller.node = NS(context=NS(ok=lambda: True))
    controller.p = {'goal_response_timeout_s': .01, 'cancel_timeout_s': .01}
    controller.cancellation = threading.Event()
    controller.check_permit = lambda: True
    controller._mutex = threading.RLock()
    controller._uncertain_submission = False
    controller._active_handle = controller._active_result = None
    return controller


def test_rejected_gripper_goal_is_failure():
    controller = controller_fixture()
    submission = Future()
    submission.set_result(NS(accepted=False))
    client = NS(server_is_ready=lambda: True, send_goal_async=lambda _goal: submission)
    with pytest.raises(MotionError, match='rejected'):
        controller._action(client, object(), .02, 'gripper', lambda _result: True)
    assert controller.terminal_confirmed


@pytest.mark.parametrize('status,error_code', [(6, 0), (4, -1)])
def test_terminal_status_and_controller_error_must_both_indicate_success(status, error_code):
    controller = controller_fixture()
    wrapped = Future()
    wrapped.set_result(NS(status=status, result=NS(error_code=error_code)))
    handle = NS(accepted=True, get_result_async=lambda: wrapped)
    submission = Future()
    submission.set_result(handle)
    client = NS(server_is_ready=lambda: True, send_goal_async=lambda _goal: submission)
    with pytest.raises(MotionError, match='did not succeed'):
        controller._action(client, object(), .02, 'gripper', lambda result: result.error_code == 0)
    assert controller.terminal_confirmed


def test_stale_planned_start_cannot_be_submitted_to_controller():
    controller = controller_fixture()
    controller.joints = ['arm_joint1']
    controller.p['trajectory_start_tolerance_rad'] = .05
    controller.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000))
    controller.evidence = NS(snapshot=lambda *_args: {'arm_joint1': .2})
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ['arm_joint1']
    trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.])]
    with pytest.raises(MotionError, match='start no longer matches'):
        controller.execute(trajectory)


def test_measured_joint_overlay_preserves_planning_scene_attached_fruit():
    controller = controller_fixture()
    controller.joints, controller.gripper_joints = ['arm'], ['left', 'right']
    controller._planning_joint_bounds = {'arm': (-3., 3.)}
    controller.node.get_clock = lambda: NS(now=lambda: NS(
        nanoseconds=1_000_000_000, to_msg=lambda: Time(sec=1)))
    controller.evidence = NS(snapshot=lambda *_args: {'arm': 2.5, 'left': .012, 'right': .012})
    state = controller.robot_state()
    assert state.is_diff  # MoveIt merges joint feedback with the scene's attached bodies
    assert not state.attached_collision_objects
    assert dict(zip(state.joint_state.name, state.joint_state.position)) == {
        'arm': 2.5, 'left': .012, 'right': .012}


def test_planning_state_clips_roundoff_copy_without_changing_actual_joint_evidence():
    import copy
    controller = controller_fixture()
    controller.joints, controller.gripper_joints = ['cr5_joint6'], ['left','right']
    controller._cache_planning_bounds('<robot name="actual"><joint name="cr5_joint6" '
        'type="revolute"><limit lower="-3.1416" upper="3.1416"/></joint></robot>')
    controller.node.get_clock = lambda: NS(now=lambda: NS(
        nanoseconds=1_000_000_000, to_msg=lambda: Time(sec=1)))
    messages = []
    controller.node.get_logger = lambda: NS(warning=messages.append)
    controller.evidence = JointEvidence(.5)
    actual = -3.1416000000000075
    assert controller.evidence.update(['cr5_joint6','left','right'],
        [actual,.012,.012],[0.,0.,0.],1_000_000_000,1_000_000_000,time.monotonic())
    before = copy.deepcopy(controller.evidence.values)
    state = controller.robot_state()
    assert dict(zip(state.joint_state.name,state.joint_state.position)) == {
        'cr5_joint6':-3.1416,'left':.012,'right':.012}
    assert state.is_diff and not state.attached_collision_objects
    assert controller.evidence.values == before
    assert controller.evidence.values['cr5_joint6'][0] == actual
    controller.robot_state()
    assert len(messages) == 1


def test_planning_state_refuses_material_limit_violation_without_changing_feedback():
    controller = controller_fixture();controller.joints = ['cr5_joint6']
    controller._cache_planning_bounds('<robot name="actual"><joint name="cr5_joint6" '
        'type="revolute"><limit lower="-3.1416" upper="3.1416"/></joint></robot>')
    measured = {'cr5_joint6':-3.1416-1.1e-6}
    with pytest.raises(MotionError,match='more than 1e-6'):
        controller._planning_state_values(measured)
    assert measured == {'cr5_joint6':-3.1416-1.1e-6}


def test_ik_joint_only_response_reused_as_seed_still_inherits_attached_fruit():
    controller = controller_fixture()
    controller.joints = ['arm']
    controller._planning_joint_bounds = {'arm': (-3., 3.)}
    controller.p.update(arm_group='arm', ik_timeout_s=.8)
    controller.frame, controller.tip = 'base_link', 'grasp_tcp'
    controller.node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=1)))
    requests = []
    solution = RobotState()
    solution.joint_state.name = ['arm'];solution.joint_state.position = [.5]
    controller._service = lambda _key, request, _timeout: (
        requests.append(request) or NS(error_code=NS(val=1), solution=solution))
    seed = RobotState(is_diff=False)
    assert controller.ik((.1, .2, .3), (0., 0., 0., 1.), seed=seed) is not None
    assert requests[0].ik_request.robot_state.is_diff
    assert not seed.is_diff  # caller's IK solution is not mutated


def bound_ik_fixture(values):
    controller = controller_fixture();controller.joints = ['cr5_joint6']
    controller._cache_planning_bounds('<robot name="actual"><joint name="cr5_joint6" '
        'type="revolute"><limit lower="-3.1416" upper="3.1416"/></joint></robot>')
    controller.p.update(arm_group='arm',ik_timeout_s=.8)
    controller.frame,controller.tip = 'base_link','grasp_tcp'
    controller.node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=1)))
    controller.robot_state = lambda: RobotState(is_diff=True)
    solutions = []
    for value in values:
        solution = RobotState()
        solution.joint_state.name = ['cr5_joint6'];solution.joint_state.position = [value]
        solutions.append(solution)
    replies = iter(solutions)
    controller._service = lambda *_: NS(error_code=NS(val=1),solution=next(replies))
    return controller,solutions


def test_collision_ik_solver_success_at_wrist_bound_is_rejected_without_changing_solution():
    controller,solutions = bound_ik_fixture([-3.1416])
    assert controller.ik((.1,.2,.3),(0.,0.,0.,1.)) is None
    status = controller.last_ik_status
    assert status['error_code'] == 1 and status['accepted'] is False
    assert status['reason'] == 'nearphysicalbound'
    assert status['near_bound_joints_rad'] == {'cr5_joint6':0.}
    assert list(solutions[0].joint_state.position) == [-3.1416]


def test_safe_alternate_wrist_roll_is_accepted_after_near_bound_candidate():
    controller,solutions = bound_ik_fixture([-3.1416+.015,-1.5708])
    assert controller.ik((.1,.2,.3),(0.,0.,0.,1.)) is None
    safe = controller.ik((.1,.2,.3),(0.,0.,.7071067811865476,.7071067811865476))
    assert safe is solutions[1]
    assert controller.last_ik_status['accepted'] is True
    assert controller.last_ik_status['error_code'] == 1
    assert controller.last_ik_status['minimum_bound_margin_rad'] == pytest.approx(1.5708)


def test_successful_joint_plan_endpoint_inside_goal_tolerance_still_needs_physical_margin():
    controller = controller_fixture();controller.joints = ['cr5_joint6']
    controller._planning_joint_bounds = {'cr5_joint6':(-3.1416,3.1416)}
    controller.p.update(arm_group='arm',planning_attempts=2,plan_timeout_s=5.,
                        velocity_scaling=.2,acceleration_scaling=.2)
    controller.robot_state = lambda: RobotState(is_diff=True)
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ['cr5_joint6']
    trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[-3.1416+.005])]
    controller._service = lambda *_: NS(motion_plan_response=NS(
        error_code=NS(val=1),trajectory=trajectory))
    with pytest.raises(MotionError,match='endpoint near physical bound'):
        controller.plan_joints([-3.1416+.01])
    trajectory.joint_trajectory.points[-1].positions = [-3.1416+.02]
    assert controller.plan_joints([-3.1416+.02]) is trajectory


def test_failed_collision_ik_preserves_error_code_for_station_diagnosis():
    controller = controller_fixture()
    controller.p.update(arm_group='arm', ik_timeout_s=.8)
    controller.frame, controller.tip = 'base_link', 'grasp_tcp'
    controller.node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=1)))
    controller.robot_state = lambda: RobotState(is_diff=True)
    controller._service = lambda *_args: NS(error_code=NS(val=-31), solution=RobotState())
    assert controller.ik((.1, .2, .3), (0., 0., 0., 1.)) is None
    assert controller.last_ik_status['error_code'] == -31
    assert controller.last_ik_status['position'] == [.1, .2, .3]
    assert controller.last_ik_status['avoid_collisions']


def test_timeout_cancels_actual_trajectory_and_unconfirmed_terminal_keeps_lock():
    controller = controller_fixture()
    cancels = []
    result = Future()
    handle = NS(accepted=True, get_result_async=lambda: result,
                cancel_goal_async=lambda: cancels.append(True))
    submission = Future()
    submission.set_result(handle)
    client = NS(server_is_ready=lambda: True, send_goal_async=lambda _goal: submission)
    with pytest.raises(MotionError, match='terminal result is unconfirmed'):
        controller._action(client, object(), .01, 'arm', lambda _result: True)
    assert cancels and not controller.terminal_confirmed


def test_late_acceptance_is_canceled_and_only_terminal_result_clears_uncertainty():
    controller = controller_fixture()
    submission, result = Future(), Future()
    cancels = []
    client = NS(server_is_ready=lambda: True, send_goal_async=lambda _goal: submission)
    with pytest.raises(MotionError, match='acceptance unconfirmed'):
        controller._action(client, object(), .01, 'arm', lambda _result: True)
    assert not controller.terminal_confirmed
    handle = NS(accepted=True, get_result_async=lambda: result,
                cancel_goal_async=lambda: cancels.append(True))
    submission.set_result(handle)
    assert cancels and not controller.terminal_confirmed
    result.set_result(NS(status=5))  # GoalStatus.STATUS_CANCELED
    assert controller.terminal_confirmed


def chain_fixture(fail_gripper=False, fail_stow=False, fruit_state='IN_BASKET'):
    moves = []
    def move(joints):
        if fail_stow:
            raise MotionError('stow failed')
        moves.append(joints)
    def grip(_position):
        if fail_gripper:
            raise MotionError('gripper unavailable')
    arm = NS(_check=lambda: None, gripper=grip,
             ik=lambda *a, **k: NS(joint_state=NS(name=['j'], position=[0.])),
             joints=['j'], plan_joints=lambda *_args: NS(),
             measured_gripper_positions=lambda: [.011,.003],
             gripper_positions=lambda *_args, **_kwargs: None,
             move_pose=lambda *a, **k: None, cartesian=lambda *a: None,
             move_joints=move, stowed=lambda: not fail_stow)
    p = {'gripper_open_m': 0., 'gripper_close_m': .012, 'pre_offset_m': .08,
         'retreat_offset_m': .12, 'surface_centre_offset_m': .025,
         'maximum_ik_candidates': 9, 'basket_drop_xyz_m': [.17, 0., .60],
         'basket_clearance_m': .15, 'basket_quaternion_xyzw': [1., 0., 0., 0.],
         'stow_joints': [0.]*6}
    p.update(gripper_contact_timeout_s=120., maximum_gripper_contact_steps=50,
             gripper_contact_step_m=.001, gripper_contact_step_s=.25, gripper_max_m=.035)
    def scene(operation, **_kwargs):
        state = {'attach': 'ATTACHED', 'release': 'RELEASED', 'wait_in_basket': fruit_state,
                 'check_grip': 'CONTACT_CONFIRMED'}.get(operation, '')
        if operation == 'wait_in_basket' and state != 'IN_BASKET':
            raise MotionError('fruit in basket unconfirmed')
        return NS(state=state, success=True)
    return GraspExecutor(arm, p, lambda x, q: (x, q), scene, lambda _p: None), moves


@pytest.mark.parametrize('failure', ['gripper', 'stow', 'fruit'])
def test_failed_gripper_stow_or_fruit_can_never_be_reported_success(failure):
    chain, _moves = chain_fixture(fail_gripper=failure == 'gripper', fail_stow=failure == 'stow',
                                 fruit_state='LOST' if failure == 'fruit' else 'IN_BASKET')
    with pytest.raises(MotionError):
        chain.execute((-.18, .65, .95), 'left')


def test_complete_chain_requires_confirmed_basket_and_stow():
    chain, moves = chain_fixture()
    assert chain.execute((-.18, .65, .95), 'left')
    assert moves == [[0.]*6]


def test_nine_rejected_centres_report_exact_stage_without_arm_movement():
    chain, moves = chain_fixture()
    queries = []
    trajectories = []
    snapshots = []
    def ik(position, orientation, seed=None):
        queries.append(seed is not None)
        code = -31 if seed is not None else 1
        chain.arm.last_ik_status = {'error_code': code, 'avoid_collisions': True}
        return None if code == -31 else NS(joint_state=NS(name=['j'], position=[0.]))
    chain.arm.ik = ik
    chain.arm.move_pose = lambda *_args, **_kwargs: trajectories.append(True)
    chain.arm.cartesian = lambda *_args: trajectories.append(True)
    chain.diagnostic = lambda candidates: snapshots.append(len(candidates))
    with pytest.raises(MotionError, match='no collision-aware'):
        chain.execute((-.18, .65, .95), 'left')
    assert queries == [False, True]*9
    assert len(chain.candidates_checked) == 9
    assert all(candidate['centre_ik']['error_code'] == -31
               and candidate['pre_ik']['accepted']
               and 'retreat_ik' not in candidate for candidate in chain.candidates_checked)
    assert snapshots[-1] == 9
    assert not moves and not trajectories


def test_ik_candidate_with_blocked_pregrasp_plan_is_skipped_before_motion():
    chain, _moves = chain_fixture()
    plans = []
    def plan(_joints):
        plans.append(True)
        if len(plans) == 1:
            raise MotionError('first candidate OMPL failed')
        return NS()
    chain.arm.plan_joints = plan
    assert chain.execute((-.18, .65, .95), 'left')
    assert len(plans) == 2
