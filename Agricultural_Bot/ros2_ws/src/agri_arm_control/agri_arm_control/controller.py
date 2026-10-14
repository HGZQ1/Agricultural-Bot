"""Direct MoveIt clients with bounded submission, cancellation and feedback."""

import math
import copy
import threading
import time
from uuid import uuid4
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Duration, Time as TimeMessage
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_action_status_default
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID

from .hardware import validate_sim_hardware
from .deadline import TrajectoryDeadline
from .safety import ControllerGoalEvidence, clamp_gripper_measurement


class MotionError(RuntimeError):
    """A rejected, failed, stale or unconfirmed movement is never success."""


class WorkCanceled(MotionError):
    """Cancellation is acknowledged only after underlying motion is terminal."""


def duration(seconds):
    total = int(seconds*1e9)
    return Duration(sec=total//1_000_000_000, nanosec=total % 1_000_000_000)


def make_pose(position, orientation):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(
        float, orientation)
    return pose


class ArmController:
    """One owner sends motion; executor threads continue receiving results."""

    def __init__(self, node, params, evidence, check_permit, cancellation, callback_group):
        self.node, self.p, self.evidence = node, params, evidence
        self.check_permit, self.cancellation = check_permit, cancellation
        self.joints = list(params['arm_joint_names'])
        self.gripper_joints = list(params['gripper_joint_names'])
        self.frame, self.tip = params['planning_frame'], params['tip_link']
        self._active_handle = None
        self._active_result = None
        self._uncertain_submission = False
        self._mutex = threading.RLock()
        self._execution_status = {}
        self._ik_status = {}
        self._controller_goals = ControllerGoalEvidence()
        self._owned_goal_ids, self._terminal_goal_ids = set(), set()
        self._initialization_idle_labels = set()
        self._runtime_description = None
        self._planning_joint_bounds = None
        self._reported_bounds_roundoff = set()
        self.environment_check = None
        self.clients = {
            'plan': node.create_client(GetMotionPlan, '/plan_kinematic_path',
                                      callback_group=callback_group),
            'cartesian': node.create_client(GetCartesianPath, '/compute_cartesian_path',
                                           callback_group=callback_group),
            'ik': node.create_client(GetPositionIK, '/compute_ik', callback_group=callback_group),
            'fk': node.create_client(GetPositionFK, '/compute_fk', callback_group=callback_group),
            'controllers': node.create_client(ListControllers, '/controller_manager/list_controllers',
                                             callback_group=callback_group),
            'hardware': node.create_client(ListHardwareComponents,
                                          '/controller_manager/list_hardware_components',
                                          callback_group=callback_group),
            'description': node.create_client(GetParameters, params['robot_description_service'],
                                              callback_group=callback_group),
        }
        self.arm_action = ActionClient(node, FollowJointTrajectory, params['arm_action'],
                                       callback_group=callback_group)
        self.gripper_action = ActionClient(
            node, FollowJointTrajectory, params['gripper_action'], callback_group=callback_group)
        self._status_subscriptions = [node.create_subscription(
            GoalStatusArray, params[f'{label}_action']+'/_action/status',
            lambda message, key=label: self._controller_goals.update(
                key, [(bytes(goal.goal_info.goal_id.uuid), goal.status)
                      for goal in message.status_list]),
            qos_profile_action_status_default, callback_group=callback_group)
            for label in ('arm', 'gripper')]
        for label in ('arm', 'gripper'):
            self.clients[label+'_cancel'] = node.create_client(
                CancelGoal, params[label+'_action']+'/_action/cancel_goal',
                callback_group=callback_group)

    @property
    def terminal_confirmed(self):
        with self._mutex:
            local = not self._uncertain_submission and self._active_handle is None
            evidence = getattr(self, '_controller_goals', None)
            return local and (evidence is None or evidence.terminal(self._terminal_goal_ids))

    @property
    def execution_status(self):
        with self._mutex:
            report = copy.deepcopy(self._execution_status)
            evidence = getattr(self, '_controller_goals', None)
            report['controller_goals'] = evidence.snapshot() if evidence is not None else {}
            return report

    @property
    def last_ik_status(self):
        with self._mutex:
            return copy.deepcopy(getattr(self, '_ik_status', {}))

    def _check(self, check_cancel=True):
        if check_cancel and self.cancellation.is_set():
            raise WorkCanceled('harvest action cancellation requested')
        if not self.check_permit():
            raise MotionError('fresh locked and stationary base permit lost')

    def _wait(self, future, timeout, label, check=True):
        deadline = time.monotonic()+timeout
        while not future.done():
            if check:
                self._check()
            if not self.node.context.ok() or time.monotonic() >= deadline:
                future.cancel()  # service requests never command motion
                raise MotionError(f'{label} response timed out')
            time.sleep(0.01)
        if check:
            self._check()
        try:
            result = future.result()
        except Exception as exc:
            raise MotionError(f'{label} failed: {exc}') from exc
        if result is None:
            raise MotionError(f'{label} returned no response')
        return result

    def _service(self, key, request, timeout, check=True):
        client = self.clients[key]
        if not client.service_is_ready():
            raise MotionError(f'{key} service unavailable')
        return self._wait(client.call_async(request), timeout, key, check)

    def robot_state(self):
        values = self.evidence.snapshot(self.node.get_clock().now().nanoseconds, time.monotonic())
        if not all(joint in values for joint in (*self.joints, *self.gripper_joints)):
            raise MotionError('complete measured arm and gripper state required')
        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        state.joint_state.name = list(values)
        state.joint_state.position = list(self._planning_state_values(values).values())
        # This is a joint-only measurement overlay. A complete empty attachment
        # list would clear the fruit that the scene bridge attached to the hand.
        state.is_diff = True
        return state

    def _cache_planning_bounds(self, description):
        """Use the live runtime model, never a guessed or local-file limit."""
        try:
            robot = ET.fromstring(description)
            model = {joint.attrib['name']: joint for joint in robot.findall('joint')}
            bounds = {}
            for name in self.joints:
                joint = model[name]
                if joint.get('type') == 'continuous':
                    bounds[name] = None
                    continue
                limit = joint.find('limit')
                low, high = float(limit.attrib['lower']), float(limit.attrib['upper'])
                if not (math.isfinite(low) and math.isfinite(high) and low <= high):
                    raise ValueError('invalid finite limits')
                bounds[name] = (low, high)
        except (ET.ParseError, KeyError, AttributeError, TypeError, ValueError) as exc:
            raise MotionError('runtime arm position bounds unavailable: '+str(exc)) from exc
        self._planning_joint_bounds = bounds
        self._runtime_description = description

    def _planning_state_values(self, measured):
        """Clip only <=1e-6 numerical overshoot in a planning-request copy.

        JointEvidence, published feedback and execution-start comparison retain
        the exact measured values. Material limit violations remain failures.
        """
        bounds = getattr(self, '_planning_joint_bounds', None)
        if bounds is None:
            raise MotionError('runtime arm bounds must be acknowledged before planning')
        values = dict(measured)
        for name in self.joints:
            value = values[name]
            if not math.isfinite(value):
                raise MotionError('nonfinite measured arm position: '+name)
            limit = bounds[name]
            if limit is None:
                continue
            low, high = limit
            boundary, edge = (low, 'lower') if value < low else (high, 'upper')
            if low <= value <= high:
                continue
            if abs(value-boundary) > 1e-6:
                raise MotionError(f'measured {name} exceeds runtime {edge} bound by more than 1e-6 rad')
            values[name] = boundary
            reported = getattr(self, '_reported_bounds_roundoff', set())
            if (name, edge) not in reported:
                self.node.get_logger().warning(f'Planning-only {name} {edge}-bound roundoff: '
                    f'measured={value:.17g}, request_copy={boundary:.17g}; actual feedback unchanged')
                reported.add((name, edge));self._reported_bounds_roundoff = reported
        return values

    def _physical_bound_margins(self, names, positions):
        """Return live-model margins of a solution, without modifying it."""
        if (len(names) != len(positions) or len(set(names)) != len(names) or
                not all(math.isfinite(value) for value in positions)):
            raise MotionError('invalid planned joint solution')
        values = dict(zip(names, positions))
        if not all(name in values for name in self.joints):
            raise MotionError('incomplete planned arm joint solution')
        bounds = getattr(self, '_planning_joint_bounds', None)
        if bounds is None:
            raise MotionError('runtime arm bounds must be acknowledged before planning')
        return {name: min(values[name]-bounds[name][0], bounds[name][1]-values[name])
                for name in self.joints if bounds[name] is not None}

    def ready(self, require_status=True):
        """Reject physical hardware plugins and inactive/mismatched controllers."""
        if require_status and not self._controller_goals.ready(
                self._owned_goal_ids, self._terminal_goal_ids):
            raise MotionError('both actual controller status snapshots and no foreign active goal required')
        if not require_status and self.p['backend_mode'] != 'gazebo':
            raise MotionError('status-free backend inspection is only for explicit Gazebo initialization')
        response = self._service('hardware', ListHardwareComponents.Request(), 2., False)
        description = getattr(self, '_runtime_description', None)
        if description is None:
            request = GetParameters.Request(names=['robot_description'])
            parameter = self._service('description', request, 2., False)
            if len(parameter.values) != 1 or parameter.values[0].type != 4:
                raise MotionError('runtime robot_state_publisher robot_description unavailable')
            description = parameter.values[0].string_value
        try:
            validate_sim_hardware(response.component, self.p['backend_mode'],
                                  (*self.joints, *self.gripper_joints), description)
        except ValueError as exc:
            raise MotionError(str(exc)) from exc
        if getattr(self, '_planning_joint_bounds', None) is None:
            self._cache_planning_bounds(description)
        controllers = self._service('controllers', ListControllers.Request(), 2., False)
        active = {c.name: c for c in controllers.controller if c.state == 'active'}
        for name, joints in ((self.p['arm_controller'], self.joints),
                             (self.p['gripper_controller'], self.gripper_joints)):
            if name not in active:
                raise MotionError(f'{name} controller is not active')
            if not set(f'{joint}/position' for joint in joints).issubset(
                    set(active[name].claimed_interfaces)):
                raise MotionError(f'{name} does not own all configured joints')
        if not (self.arm_action.server_is_ready() and self.gripper_action.server_is_ready() and
                all(self.clients[k].service_is_ready() for k in ('plan', 'cartesian', 'ik', 'fk'))):
            raise MotionError('MoveIt execution/planning or gripper Action unavailable')
        self.robot_state()
        return True

    def ik(self, position, orientation, seed=None):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.p['arm_group']
        request.ik_request.ik_link_name = self.tip
        request.ik_request.robot_state = copy.deepcopy(seed) if seed is not None else self.robot_state()
        # MoveIt IK responses omit attached bodies and may reset is_diff=False.
        # A reused joint seed must remain an overlay on the live collision scene.
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = duration(self.p['ik_timeout_s'])
        stamped = PoseStamped()
        stamped.header.frame_id = self.frame
        stamped.header.stamp = self.node.get_clock().now().to_msg()
        stamped.pose = make_pose(position, orientation)
        request.ik_request.pose_stamped = stamped
        started = time.monotonic()
        result = self._service('ik', request, self.p['ik_timeout_s']+1.)
        with self._mutex:
            self._ik_status = {'frame': self.frame, 'tip': self.tip,
                'position': list(position), 'orientation': list(orientation),
                'avoid_collisions': True, 'error_code': result.error_code.val,
                'accepted': False,
                'wall_elapsed_s': time.monotonic()-started}
        if result.error_code.val != 1:
            return None
        try:
            margins = self._physical_bound_margins(result.solution.joint_state.name,
                                                   result.solution.joint_state.position)
        except MotionError as error:
            with self._mutex:
                self._ik_status.update(reason='invalid_joint_solution', detail=str(error))
            return None
        near = {name: margin for name, margin in margins.items() if margin < .02}
        with self._mutex:
            self._ik_status.update(required_bound_margin_rad=.02,
                minimum_bound_margin_rad=min(margins.values()) if margins else None,
                physical_bound_margins_rad=margins, accepted=not near)
            if near:
                # Preserve MoveIt's solver success code. This candidate is
                # rejected for physical-limit clearance so the existing roll/
                # pitch candidate loop can try a different wrist orientation.
                self._ik_status.update(reason='nearphysicalbound', near_bound_joints_rad=near)
        if near:
            return None
        return result.solution

    def plan_joints(self, positions):
        self._check()
        if getattr(self, 'environment_check', None) is not None:
            self.environment_check()
        if len(positions) != len(self.joints) or not all(math.isfinite(v) for v in positions):
            raise MotionError('invalid arm joint goal')
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name, plan.pipeline_id = self.p['arm_group'], 'ompl'
        plan.num_planning_attempts = self.p['planning_attempts']
        plan.allowed_planning_time = self.p['plan_timeout_s']
        plan.max_velocity_scaling_factor = self.p['velocity_scaling']
        plan.max_acceleration_scaling_factor = self.p['acceleration_scaling']
        plan.start_state = self.robot_state()
        goal = Constraints()
        for name, value in zip(self.joints, positions):
            constraint = JointConstraint(joint_name=name, position=float(value),
                                         tolerance_above=0.005, tolerance_below=0.005, weight=1.)
            goal.joint_constraints.append(constraint)
        plan.goal_constraints = [goal]
        result = self._service('plan', request, self.p['plan_timeout_s']+2.)
        if result.motion_plan_response.error_code.val != 1:
            raise MotionError(f'joint plan failed: {result.motion_plan_response.error_code.val}')
        trajectory = result.motion_plan_response.trajectory
        if not trajectory.joint_trajectory.points:
            raise MotionError('joint plan returned an empty trajectory')
        margins = self._physical_bound_margins(trajectory.joint_trajectory.joint_names,
                                               trajectory.joint_trajectory.points[-1].positions)
        near = {name: margin for name, margin in margins.items() if margin < .01}
        if near:
            raise MotionError('planned endpoint near physical bound (minimum 0.01 rad): '+str(near))
        return trajectory

    def move_joints(self, positions):
        self.execute(self.plan_joints(positions))
        self.wait_joints(self.joints, positions, self.p['feedback_timeout_s'])

    def move_pose(self, position, orientation, solution=None, planned=None):
        solution = solution or self.ik(position, orientation)
        if solution is None:
            raise MotionError('collision-aware IK unavailable')
        values = dict(zip(solution.joint_state.name, solution.joint_state.position))
        positions = [values[name] for name in self.joints]
        if planned is None:
            self.move_joints(positions)
        else:
            self.execute(planned)
            self.wait_joints(self.joints, positions, self.p['feedback_timeout_s'])

    def cartesian(self, position, orientation):
        """Require a full collision-checked, bounded-jump Cartesian path."""
        if getattr(self, 'environment_check', None) is not None:
            self.environment_check()
        request = GetCartesianPath.Request()
        request.header.frame_id = self.frame
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.start_state = self.robot_state()
        request.group_name, request.link_name = self.p['arm_group'], self.tip
        request.waypoints = [make_pose(position, orientation)]
        request.max_step, request.jump_threshold = self.p['cartesian_step_m'], 0.
        request.revolute_jump_threshold = self.p['cartesian_joint_jump_rad']
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.p['velocity_scaling']
        request.max_acceleration_scaling_factor = self.p['acceleration_scaling']
        result = self._service('cartesian', request, self.p['plan_timeout_s']+2.)
        if result.error_code.val != 1 or result.fraction < 1.-1e-6:
            raise MotionError(f'Cartesian path incomplete: fraction={result.fraction:.4f}, '
                              f'error={result.error_code.val}')
        points = result.solution.joint_trajectory.points
        if any(max(abs(a-b) for a, b in zip(first.positions, second.positions)) >
               self.p['cartesian_joint_jump_rad'] for first, second in zip(points, points[1:])):
            raise MotionError('Cartesian path contains a joint jump')
        self.execute(result.solution)
        self.wait_joints(result.solution.joint_trajectory.joint_names,
                         points[-1].positions, self.p['feedback_timeout_s'])

    def execute(self, trajectory):
        if (not trajectory.joint_trajectory.points or
                set(trajectory.joint_trajectory.joint_names) != set(self.joints) or
                len(trajectory.joint_trajectory.joint_names) != len(self.joints)):
            raise MotionError('empty trajectory')
        measured = self.evidence.snapshot(self.node.get_clock().now().nanoseconds, time.monotonic())
        first = trajectory.joint_trajectory.points[0]
        if (len(first.positions) != len(self.joints) or
                any(abs(measured[name]-value) > self.p['trajectory_start_tolerance_rad']
                    for name, value in zip(trajectory.joint_trajectory.joint_names, first.positions))):
            raise MotionError('planned trajectory start no longer matches measured arm joints')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = copy.deepcopy(trajectory.joint_trajectory)
        goal.trajectory.header.stamp = TimeMessage()  # start on receipt after the measured-start gate
        goal.goal_time_tolerance = duration(0.5)
        self._action(self.arm_action, goal, self.p['execute_timeout_s'], 'arm',
                     lambda result: result.error_code == FollowJointTrajectory.Result.SUCCESSFUL,
                     trajectory_duration_s=self._trajectory_duration(goal.trajectory),
                     hard_wall_timeout_s=self.p['execute_wall_timeout_s'])

    def gripper(self, position):
        return self.gripper_positions([position]*len(self.gripper_joints))

    def measured_gripper_positions(self):
        values = self.evidence.snapshot(self.node.get_clock().now().nanoseconds, time.monotonic())
        if not all(name in values for name in self.gripper_joints):
            raise MotionError('fresh complete dual-finger measurement required')
        try:
            return [clamp_gripper_measurement(values[name], self.p['gripper_max_m'])
                    for name in self.gripper_joints]
        except ValueError as exc:
            raise MotionError(str(exc)) from exc

    def gripper_positions(self, positions, move_s=None, deadline_wall=None):
        if (len(positions) != len(self.gripper_joints) or
                not all(math.isfinite(value) and 0 <= value <= self.p['gripper_max_m']
                        for value in positions)):
            raise MotionError('gripper joint goal outside calibrated bound')
        seconds = self.p['gripper_move_s'] if move_s is None else move_s
        if not math.isfinite(seconds) or not 0 < seconds <= self.p['gripper_timeout_s']:
            raise MotionError('gripper trajectory duration outside configured bound')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joints
        point = JointTrajectoryPoint()
        point.positions = list(map(float, positions))
        point.time_from_start = duration(seconds)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = duration(0.5)
        remaining = (self.p['gripper_wall_timeout_s'] if deadline_wall is None else
                     min(self.p['gripper_wall_timeout_s'], deadline_wall-time.monotonic()))
        if remaining <= 0:
            raise MotionError('contact closure wall deadline expired before finger trajectory')
        self._action(self.gripper_action, goal, self.p['gripper_timeout_s'], 'gripper',
                     lambda result: result.error_code == FollowJointTrajectory.Result.SUCCESSFUL,
                     trajectory_duration_s=self._trajectory_duration(goal.trajectory),
                     hard_wall_timeout_s=remaining)
        remaining = (self.p['feedback_timeout_s'] if deadline_wall is None else
                     min(self.p['feedback_timeout_s'], deadline_wall-time.monotonic()))
        if remaining <= 0:
            raise MotionError('contact closure wall deadline expired before finger feedback')
        self.wait_joints(self.gripper_joints, point.positions, remaining,
                         position_tolerance=self.p['gripper_position_tolerance_m'],
                         velocity_tolerance=self.p['gripper_velocity_tolerance_mps'])

    def _late_submission(self, future):
        """An accepted goal after deadline must be canceled and observed terminal."""
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._uncertain_submission = False
                return
            with self._mutex:
                self._active_handle, self._active_result = handle, handle.get_result_async()
                self._uncertain_submission = False
            self._active_result.add_done_callback(self._late_terminal)
            handle.cancel_goal_async()
        except Exception:
            self._uncertain_submission = True

    def _late_terminal(self, future):
        try:
            terminal = future.result().status in {
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}
        except Exception:
            terminal = False
        if terminal:
            with self._mutex:
                if future is not self._active_result:
                    return  # a delayed callback cannot clear a newer controller goal
                handle = self._active_handle
                if handle is not None and hasattr(handle, 'goal_id'):
                    self._terminal_goal_ids.add(bytes(handle.goal_id.uuid))
                self._active_handle = self._active_result = None

    def cancel_and_confirm(self):
        with self._mutex:
            handle, result = self._active_handle, self._active_result
        if handle is None:
            return self.terminal_confirmed
        handle.cancel_goal_async()
        deadline = time.monotonic()+self.p['cancel_timeout_s']
        while result is not None and not result.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if result is not None and result.done():
            self._late_terminal(result)
        return self.terminal_confirmed

    def prepare_initialization_hold(self):
        """Cancel every old goal and wait for observed nonterminal goals to finish."""
        self._initialization_idle_labels.clear()
        if self._uncertain_submission or self._active_handle is not None:
            raise MotionError('local controller submission is not terminal')
        for label in ('arm', 'gripper'):
            response = self._service(label+'_cancel', CancelGoal.Request(),
                                     self.p['cancel_timeout_s'])
            if response.return_code != CancelGoal.Response.ERROR_NONE:
                raise MotionError(f'{label} cancel-all was not acknowledged')
            deadline = time.monotonic()+self.p['cancel_timeout_s']
            while not self._controller_goals.idle_for_initialization(self._terminal_goal_ids):
                self._check()
                if time.monotonic() >= deadline:
                    raise MotionError('controller cancellation still lacks actual terminal status')
                time.sleep(.02)
            if response.goals_canceling:
                # A returned goal must appear terminal in the real status array.
                ids = {bytes(info.goal_id.uuid) for info in response.goals_canceling}
                while not ids.issubset(self._controller_goals.terminal_identifiers()):
                    self._check()
                    if time.monotonic() >= deadline:
                        raise MotionError('cancel-all goal terminal status is unconfirmed')
                    time.sleep(.02)
            self._initialization_idle_labels.add(label)

    def hold_current_for_initialization(self, label):
        """A current-feedback hold creates genuine controller status, never stow."""
        self._check()
        values = self.evidence.snapshot(self.node.get_clock().now().nanoseconds, time.monotonic())
        joints = self.joints if label == 'arm' else self.gripper_joints
        positions = [values[joint] for joint in joints]
        if label == 'gripper':
            try:
                positions = [clamp_gripper_measurement(value, self.p['gripper_max_m'])
                             for value in positions]
            except ValueError as exc:
                raise MotionError(str(exc)) from exc
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=positions, velocities=[0.]*len(joints),
            time_from_start=duration(.25))]
        goal.goal_time_tolerance = duration(.5)
        self._action(self.arm_action if label == 'arm' else self.gripper_action, goal,
                     4., label, lambda result: result.error_code == 0,
                     trajectory_duration_s=.25, hard_wall_timeout_s=20., initialization_hold=True)
        self._initialization_idle_labels.discard(label)

    @staticmethod
    def _trajectory_duration(trajectory):
        times = [point.time_from_start.sec+point.time_from_start.nanosec/1e9
                 for point in trajectory.points]
        if (not times or times[-1] <= 0 or any(not math.isfinite(value) or value < 0
                                               for value in times) or
                any(first >= second for first, second in zip(times, times[1:]))):
            raise MotionError('trajectory time_from_start must be finite and strictly increasing')
        return times[-1]

    def _action(self, client, goal, timeout, label, successful,
                trajectory_duration_s=None, hard_wall_timeout_s=None, initialization_hold=False):
        self._check()
        bootstrap = (initialization_hold and self.p['backend_mode'] == 'gazebo' and
                     label in self._initialization_idle_labels and
                     not self._uncertain_submission and self._active_handle is None and
                     self._controller_goals.idle_for_initialization(self._terminal_goal_ids))
        if not self.terminal_confirmed and not bootstrap:
            raise MotionError('previous trajectory is not confirmed terminal')
        if not client.server_is_ready():
            raise MotionError(f'{label} action server unavailable')
        execution_deadline = None
        if trajectory_duration_s is not None:
            try:
                # Validate before submission; start the same budget on acceptance.
                execution_deadline = TrajectoryDeadline(
                    trajectory_duration_s, self.node.get_clock().now().nanoseconds,
                    time.monotonic(), timeout, hard_wall_timeout_s,
                    self.p['execution_clock_stall_s'], self.p['execution_margin_factor'],
                    self.p['execution_margin_s'])
            except ValueError as exc:
                raise MotionError(str(exc)) from exc
        self._uncertain_submission = True
        if hasattr(self, '_owned_goal_ids'):
            identifier = uuid4().bytes
            self._owned_goal_ids.add(identifier)
            submission = client.send_goal_async(goal, goal_uuid=UUID(uuid=list(identifier)))
        else:  # small isolated Action boundary fixtures do not construct subscriptions
            submission = client.send_goal_async(goal)
        deadline = time.monotonic()+self.p['goal_response_timeout_s']
        error = None
        while not submission.done():
            try:
                self._check()
            except MotionError as exc:
                error = exc
                break
            if time.monotonic() >= deadline or not self.node.context.ok():
                error = MotionError(f'{label} goal acceptance unconfirmed')
                break
            time.sleep(0.01)
        if error is not None:
            self._uncertain_submission = True
            submission.add_done_callback(self._late_submission)
            raise error
        handle = submission.result()
        if handle is None or not handle.accepted:
            self._uncertain_submission = False
            raise MotionError(f'{label} goal rejected')
        result_future = handle.get_result_async()
        with self._mutex:
            self._active_handle, self._active_result = handle, result_future
            self._uncertain_submission = False
        deadline = time.monotonic()+timeout
        if execution_deadline is not None:
            execution_deadline = TrajectoryDeadline(
                trajectory_duration_s, self.node.get_clock().now().nanoseconds,
                time.monotonic(), timeout, hard_wall_timeout_s,
                self.p['execution_clock_stall_s'], self.p['execution_margin_factor'],
                self.p['execution_margin_s'])
            self.node.get_logger().info(f'{label} trajectory duration={trajectory_duration_s:.3f} '
                                        f'SIM s; execution budget={execution_deadline.sim_limit:.3f} '
                                        f'SIM s/{hard_wall_timeout_s:.1f} WALL s')
        with self._mutex:
            self._execution_status = {'controller': label, 'phase': 'EXECUTING'}
        while not result_future.done():
            try:
                self._check()
            except MotionError as exc:
                error = exc
                break
            now_wall = time.monotonic()
            reason = (execution_deadline.update(self.node.get_clock().now().nanoseconds, now_wall)
                      if execution_deadline is not None else
                      ('wall execution deadline reached' if now_wall >= deadline else None))
            if execution_deadline is not None:
                with self._mutex:
                    self._execution_status.update(execution_deadline.report)
            if reason is not None or not self.node.context.ok():
                error = MotionError(f'{label} execution timed out: {reason or "ROS shutdown"}')
                break
            time.sleep(0.01)
        if error is not None:
            terminal = self.cancel_and_confirm()
            with self._mutex:
                self._execution_status.update(phase='CANCELED_AFTER_FAILURE', error=str(error),
                                              terminal_confirmed=terminal)
            if not terminal:
                raise MotionError(f'{error}; trajectory terminal result is unconfirmed') from error
            raise error
        wrapped = result_future.result()
        if execution_deadline is not None:
            execution_deadline.update(self.node.get_clock().now().nanoseconds, time.monotonic())
            with self._mutex:
                self._execution_status.update(execution_deadline.report)
        self._late_terminal(result_future)
        local_terminal = not self._uncertain_submission and self._active_handle is None
        if not self.terminal_confirmed and not (bootstrap and local_terminal):
            raise MotionError(f'{label} terminal status unavailable')
        self._check()
        with self._mutex:
            self._execution_status.update(phase='TERMINAL', terminal_status=wrapped.status,
                                          terminal_confirmed=True)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not successful(wrapped.result):
            raise MotionError(f'{label} did not succeed: status={wrapped.status}, '
                              f'result={wrapped.result}')

    def wait_joints(self, names, positions, timeout, position_tolerance=None,
                    velocity_tolerance=None):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self._check()
            if self.evidence.at(names, positions, self.node.get_clock().now().nanoseconds,
                                time.monotonic(),
                                position_tolerance or self.p['joint_position_tolerance_rad'],
                                velocity_tolerance or self.p['joint_velocity_tolerance_rps'],
                                self.p['joint_settle_s']):
                return
            time.sleep(0.02)
        raise MotionError('measured joint position/velocity settlement timed out')

    def stowed(self):
        return self.terminal_confirmed and self.evidence.at(
            self.joints, self.p['stow_joints'], self.node.get_clock().now().nanoseconds,
            time.monotonic(), self.p['joint_position_tolerance_rad'],
            self.p['joint_velocity_tolerance_rps'], self.p['joint_settle_s'])

    def stow_report(self):
        report = self.evidence.pose_report(
            self.joints, self.p['stow_joints'], self.node.get_clock().now().nanoseconds,
            time.monotonic(), self.p['joint_position_tolerance_rad'],
            self.p['joint_velocity_tolerance_rps'], self.p['joint_settle_s'])
        terminal = self.terminal_confirmed
        report['controller_terminal_confirmed'] = terminal
        if not terminal:
            report['passed'] = False
            report['reasons'].append('controller_goal_not_confirmed_terminal')
        return report

    def destroy(self):
        self.cancellation.set()
        self.cancel_and_confirm()
        self.arm_action.destroy()
        self.gripper_action.destroy()
