"""Single-owner HarvestWork server with parked-station and measured-stow gates."""

import copy
import json
import math
import os
import threading
import time
from pathlib import Path

from agri_harvest_interfaces.action import HarvestWork
from agri_harvest_interfaces.msg import HarvestWorkerState, TomatoTargetArray
from agri_harvest_interfaces.srv import SimFruitCommand
from agri_task_manager.harvest_coordinator import side_and_window, target_quality, transform_position
from agri_task_manager.harvest_safety import StoppedMotion, stamp_ns
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import yaml

from .controller import ArmController, MotionError, WorkCanceled
from .geometry import transform_pose
from .grasp_executor import GraspExecutor
from .observation_scan import camera_pose, descending_heights, tcp_pose_for_camera
from .safety import JointEvidence, fresh, station_matches


class HarvestArmWorker(Node):
    """Accept one identified observation or one guarded scan transaction."""

    def __init__(self):
        super().__init__('harvest_arm_worker')
        path = Path(get_package_share_directory('agri_arm_control')) / 'config/harvest_arm_worker.yaml'
        defaults = yaml.safe_load(path.read_text())['harvest_arm_worker']['ros__parameters']
        self.p = {}
        for name, default in defaults.items():
            if name == 'use_sim_time':
                continue
            descriptor = ParameterDescriptor(dynamic_typing=isinstance(default, list) and not default)
            self.declare_parameter(name, default, descriptor)
            self.p[name] = self.get_parameter(name).value
        self.declare_parameter('ideal_simulation', os.getenv('AGRI_IDEAL_SIMULATION') == '1')
        self._ideal_simulation = (bool(self.get_parameter('ideal_simulation').value) and
                                  bool(self.get_parameter('use_sim_time').value))
        self._validate_parameters()
        self.group = ReentrantCallbackGroup()
        # A single feedback stream must not run concurrently on the six worker
        # executor threads and commit a newer sample before an older callback.
        # Other callbacks remain reentrant so actions cannot block feedback.
        self._joint_feedback_group = MutuallyExclusiveCallbackGroup()
        # Parking history also needs FIFO updates; a newer odometry callback
        # must not finish before an older callback on another executor thread.
        self._odom_feedback_group = MutuallyExclusiveCallbackGroup()
        self._mutex = threading.RLock()
        self._busy, self._fault, self._healthy = False, False, False
        self._startup_checked = False
        self._phase, self._work_context = 'IDLE', ('', '', '')
        self._last_heartbeat = None
        self._request = self._handle = None
        self._started_wall = None
        self._route = self._station = None
        self._route_wall = self._lock_wall = None
        self._locked = False
        self._base_velocity = None
        self._targets, self._targets_wall = None, None
        self._observation = None
        self._scan_diagnostics = {}
        self._selected_target = {}
        self._last_error = ''
        self._last_health_error = ''
        self._scene_ready_diagnostics = {}
        self._grasp_diagnostics = []
        self._contact_diagnostics = {}
        self._used_work_ids, self._target_history = set(), {}
        self._cancel = threading.Event()
        self._closing = threading.Event()
        self.evidence = JointEvidence(self.p['feedback_max_age_s'])
        self.motion = StoppedMotion()
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self._pub = self.create_publisher(HarvestWorkerState, self.p['worker_state_topic'], 10)
        self._status_pub = self.create_publisher(String, '~/status', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, self.p['route_status_topic'], self._route_callback,
                                 qos, callback_group=self.group)
        self.create_subscription(String, self.p['station_topic'], self._station_callback,
                                 qos, callback_group=self.group)
        self.create_subscription(Bool, self.p['motion_lock_topic'], self._lock_callback,
                                 qos, callback_group=self.group)
        self.create_subscription(Odometry, self.p['odom_topic'], self._odom_callback,
                                 10, callback_group=self._odom_feedback_group)
        self.create_subscription(JointState, self.p['joint_states_topic'], self._joint_callback,
                                 10, callback_group=self._joint_feedback_group)
        self.create_subscription(TomatoTargetArray, self.p['targets_topic'], self._targets_callback,
                                 10, callback_group=self.group)
        self.arm = ArmController(self, self.p, self.evidence, self._permit,
                                 self._cancel, self.group)
        self._scene_clients = {
            'contact': self.create_client(SimFruitCommand, '/harvest_scene/contact',
                                          callback_group=self.group),
            'fruit': self.create_client(SimFruitCommand, '/harvest_sim/command',
                                       callback_group=self.group),
        }
        self.arm.environment_check = self._environment_ready
        self._server = ActionServer(
            self, HarvestWork, self.p['work_action'], execute_callback=self._execute,
            goal_callback=self._goal, cancel_callback=self._cancel_goal,
            callback_group=self.group)
        self.create_service(Trigger, '~/recover_stow', self._recover, callback_group=self.group)
        self.create_service(Trigger, '~/initialize_sim_hold', self._initialize_sim_hold,
                            callback_group=self.group)
        self.create_service(Trigger, '~/force_initialize_sim_hold', self._force_initialize_sim_hold,
                            callback_group=self.group)
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self._heartbeat, callback_group=self.group, clock=self._steady)
        self._health_thread = threading.Thread(target=self._health, name='arm-health', daemon=True)
        self._health_thread.start()

    def _validate_parameters(self):
        if self.p['backend_mode'] not in {'gazebo', 'mock'}:
            raise ValueError('only gazebo or explicitly selected mock backend is supported')
        if self.p['task_frame'] != 'base_footprint':
            raise ValueError('task_frame must use the project standard base_footprint axes')
        if len(self.p['arm_joint_names']) != 6 or len(self.p['stow_joints']) != 6:
            raise ValueError('six named arm joints and stow positions required')
        if len(self.p['gripper_joint_names']) != 2:
            raise ValueError('both independent simulated finger joints are required')
        for name in ('scan_left_joints', 'scan_right_joints'):
            if self.p[name] is None:
                self.p[name] = []
            if len(self.p[name]) not in {0, 6}:
                raise ValueError(f'{name} must be empty or contain six measured joint positions')
        if self.p['scan_mode'] not in {'vertical', 'preset'}:
            raise ValueError('scan_mode must be vertical or preset')
        if not self.p['scan_camera_frame'] or not all(math.isfinite(self.p[name]) for name in
                ('scan_camera_x_m', 'scan_camera_lateral_m', 'target_minimum_height_m',
                 'target_maximum_height_m')):
            raise ValueError('finite camera scan geometry and target height bounds required')
        if (self.p['scan_camera_lateral_m'] <= 0 or
                not 0 <= self.p['target_minimum_height_m'] < self.p['target_maximum_height_m']):
            raise ValueError('positive scan lateral distance and ordered target heights required')
        descending_heights(self.p['scan_top_height_m'], self.p['scan_bottom_height_m'],
                           self.p['scan_height_step_m'])
        for name in ('joint_settle_s', 'feedback_max_age_s', 'feedback_timeout_s', 'ik_timeout_s',
                     'plan_timeout_s', 'execute_timeout_s', 'goal_response_timeout_s',
                     'execute_wall_timeout_s', 'gripper_wall_timeout_s',
                     'execution_clock_stall_s', 'execution_margin_factor', 'execution_margin_s',
                     'health_interval_s',
                     'gripper_contact_timeout_s', 'gripper_contact_step_s',
                     'cancel_timeout_s', 'work_deadline_s', 'target_max_age_s',
                     'cartesian_step_m', 'cartesian_joint_jump_rad', 'selection_timeout_s',
                     'scan_dwell_s',
                     'scene_service_timeout_s', 'basket_verification_timeout_s'):
            if not math.isfinite(self.p[name]) or self.p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('scene_ready_attempt_timeout_s', 'scene_ready_retry_interval_s'):
            if not math.isfinite(self.p[name]) or self.p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not (0 <= self.p['surface_centre_offset_m'] <= 0.04 and
                0 <= self.p['gripper_open_m'] < self.p['gripper_close_m'] <=
                self.p['gripper_max_m'] <= 0.035):
            raise ValueError('gripper stroke or surface-offset geometric bound exceeded')
        if any(not 0 < self.p[name] <= 0.5 for name in ('velocity_scaling', 'acceleration_scaling')):
            raise ValueError('arm scaling must be in (0,0.5]')
        if (not math.isfinite(self.p['navigation_stow_velocity_tolerance_rps']) or
                not self.p['joint_velocity_tolerance_rps'] <=
                self.p['navigation_stow_velocity_tolerance_rps'] <= .25):
            raise ValueError('navigation stow velocity tolerance must be within strict limit and 0.25 rad/s')
        if (not math.isfinite(self.p['gripper_contact_step_m']) or
                not 0 < self.p['gripper_contact_step_m'] <= .001 or
                not 1 <= self.p['maximum_gripper_contact_steps'] <= 100 or
                self.p['gripper_contact_step_s'] > self.p['gripper_timeout_s']):
            raise ValueError('contact closure step or budget exceeds bounded controller contract')

    def _health(self):
        last_error = None
        while not self._closing.is_set() and self.context.ok():
            if self._phase == 'INITIALIZING_SIM_HOLD':
                self._closing.wait(self.p['health_interval_s'])
                continue
            try:
                self.arm.ready()
                # Scene readiness belongs to initialization and actual work /
                # planning operations. Repeated whole-scene RPCs while driving
                # must not revoke an otherwise healthy measured stow lease.
                with self._mutex:
                    self._healthy = True
                    self._last_health_error = ''
                self._detect_startup_pose()
                last_error = None
            except Exception as exc:
                with self._mutex:
                    self._healthy = False
                    self._last_health_error = str(exc)
                    self.arm.reset_navigation_stow()
                if str(exc) != last_error:
                    self.get_logger().warning('arm backend not ready: '+str(exc))
                    last_error = str(exc)
            self._closing.wait(self.p['health_interval_s'])

    def _detect_startup_pose(self):
        """A restart away from stow is a recoverable fault, never an idle lease."""
        with self._mutex:
            if self._startup_checked or self._busy or self._fault or not self._healthy:
                return
            if not self.arm.terminal_confirmed:
                return
            if not self.evidence.stationary(
                    self.p['arm_joint_names'],
                    self.get_clock().now().nanoseconds, time.monotonic(),
                    self.p['joint_velocity_tolerance_rps'], self.p['joint_settle_s']):
                return
            if not self.evidence.stationary(
                    self.p['gripper_joint_names'], self.get_clock().now().nanoseconds,
                    time.monotonic(), self.p['gripper_velocity_tolerance_mps'],
                    self.p['joint_settle_s']):
                return
            report = self.evidence.pose_report(
                self.p['arm_joint_names'], self.p['stow_joints'],
                self.get_clock().now().nanoseconds, time.monotonic(),
                self.p['joint_position_tolerance_rad'],
                self.p['joint_velocity_tolerance_rps'], self.p['joint_settle_s'])
            if (not report['fresh'] or not report['complete'] or
                    'velocity_above_stopped_tolerance' in report['reasons']):
                return
            if 'position_outside_stow_tolerance' in report['reasons']:
                self._startup_checked = True
                self._fault, self._phase = True, 'FAULT'
                self.get_logger().warning(
                    'startup measured stationary arm is not STOWED; guarded recovery required')
            elif self.arm.stowed():
                self._startup_checked = True
            # stowed() begins its own at-pose settlement after initialization.
            # Pending settlement is no evidence of a measured non-stow pose.

    def _route_callback(self, message):
        with self._mutex:
            try:
                self._route = json.loads(message.data)
                self._route_wall = time.monotonic()
            except (ValueError, TypeError):
                self._route = None

    def _station_callback(self, message):
        with self._mutex:
            try:
                self._station = json.loads(message.data)
            except (ValueError, TypeError):
                self._station = None

    def _lock_callback(self, message):
        self._locked, self._lock_wall = bool(message.data), time.monotonic()

    def _odom_callback(self, message):
        velocity = message.twist.twist
        self._base_velocity = (velocity.linear.x, velocity.linear.y, velocity.linear.z,
                               velocity.angular.z)
        self.motion.update(stamp_ns(message.header.stamp), self.get_clock().now().nanoseconds,
                           time.monotonic(), (velocity.linear.x, velocity.linear.y, velocity.linear.z),
                           velocity.angular.z)

    def _joint_callback(self, message):
        accepted = self.evidence.update(message.name, message.position, message.velocity,
                                        stamp_ns(message.header.stamp), self.get_clock().now().nanoseconds,
                                        time.monotonic())
        # update() releases the evidence lock before acquiring the controller
        # lock here, avoiding an evidence/controller lock-order inversion.
        if not accepted and not self.evidence.last_update.get('ignored_exact_duplicate', False):
            self.arm.reset_navigation_stow(clear_settlement=True)
        elif self.arm.navigation_stow_latched and not self.arm.navigation_stow_report()['envelope_valid']:
            self.arm.reset_navigation_stow(clear_settlement=True)

    def _targets_callback(self, message):
        with self._mutex:
            self._targets, self._targets_wall = message, time.monotonic()

    def _parked(self):
        if getattr(self, '_ideal_simulation', False):
            values = getattr(self, '_base_velocity', None)
            return bool(values is not None and all(math.isfinite(value) for value in values) and
                        math.sqrt(sum(value*value for value in values[:3])) < .01 and
                        abs(values[3]) < .02)
        wall = time.monotonic()
        return (self._locked and self._lock_wall is not None and wall-self._lock_wall <= 0.35 and
                self.motion.ready(self.get_clock().now().nanoseconds, wall))

    def _permit(self):
        with self._mutex:
            if getattr(self, '_ideal_simulation', False):
                request = self._request
                return bool(request is not None and self._parked() and
                            (request.run_id, request.waypoint_id, request.work_id) ==
                            self._work_context)
            return (self._healthy and self._parked() and self._request is not None and
                    (self._started_wall is None or time.monotonic()-self._started_wall <
                     self.p['work_deadline_s']) and
                    station_matches(self._route, self._station, self._request,
                                    self.get_clock().now().nanoseconds, self._route_wall,
                                    time.monotonic()))

    def _ideal_station_matches(self, request):
        """Keep station/work identity, without heartbeat or stow leases."""
        route, station = self._route, self._station
        if not isinstance(route, dict) or not isinstance(station, dict):
            return False
        current = route.get('current')
        return bool(isinstance(current, dict) and route.get('require_harvest_handshake') and
                    route.get('state') == 'AT_STATION' and station.get('active') and
                    route.get('work_phase') == 'WORKING' and
                    route.get('work_terminal_confirmed') is False and
                    current.get('waypoint_id') == request.waypoint_id and
                    route.get('run_id') == request.run_id and
                    route.get('work_id') == request.work_id and
                    all(station.get(name) == getattr(request, name) for name in
                        ('run_id', 'waypoint_id', 'work_id', 'plant_id', 'work_side')))

    def _goal(self, request):
        with self._mutex:
            scan = bool(getattr(request, 'scan_before_selection', False))
            ideal = getattr(self, '_ideal_simulation', False)
            identity_matches = (self._ideal_station_matches(request) if ideal else
                station_matches(self._route, self._station, request,
                    self.get_clock().now().nanoseconds, self._route_wall,
                    time.monotonic(), accepting=True))
            if (self._busy or
                    (not ideal and (self._fault or not self._healthy or not self.arm.stowed())) or
                    not self._parked() or
                    not all((request.run_id, request.work_id, request.waypoint_id, request.plant_id)) or
                    request.work_id in self._used_work_ids or
                    not identity_matches or
                    request.work_side not in {'left', 'right'} or
                    (scan and self.p['scan_mode'] == 'preset' and
                     not self.p[f'scan_{request.work_side}_joints'])):
                return GoalResponse.REJECT
            if not scan:
                try:
                    self._validate_target(request.target, request.header, request.work_side)
                except (MotionError, ValueError):
                    return GoalResponse.REJECT
            # Reserve before ACCEPT so simultaneous goals cannot both leave stow.
            self._busy = True
            if ideal:
                self._fault = False
            self._request = request
            self._work_context = (request.run_id, request.waypoint_id, request.work_id)
            self._used_work_ids.add(request.work_id)
            self._cancel.clear()
            self._phase = 'DEPLOYING'
            self.arm.reset_navigation_stow()
            self._started_wall = time.monotonic()
            return GoalResponse.ACCEPT

    def _cancel_goal(self, _handle):
        self._cancel.set()
        return CancelResponse.ACCEPT

    def _set_phase(self, phase):
        with self._mutex:
            previous = self._phase
            self._phase = phase
            if phase not in {'IDLE', 'STOWED'}:
                self.arm.reset_navigation_stow()
        if phase != previous:
            self.get_logger().info('harvest phase: '+phase)
        handle = self._handle
        if handle is not None and handle.is_active:
            handle.publish_feedback(HarvestWork.Feedback(phase=phase, arm_stowed=False))

    def _validate_target(self, target, header, side, earliest_ns=0):
        now_ns = self.get_clock().now().nanoseconds
        stamp = stamp_ns(header.stamp)
        if (not header.frame_id or not fresh(stamp, now_ns, self.p['target_max_age_s']) or
                stamp < earliest_ns or
                not target_quality(target, self.p['minimum_confidence'], self.p['require_segmentation'])):
            raise MotionError('fresh stable mature segmented observation required')
        context = self._work_context[:2]
        if self._target_history.get(target.target_id, set()) - {context}:
            raise MotionError('target ID belongs to a previous work station')
        try:
            transform = (None if header.frame_id == self.p['task_frame'] else
                         self.tf.lookup_transform(self.p['task_frame'], header.frame_id,
                                                  Time.from_msg(header.stamp)))
            xyz = ((target.position.x, target.position.y, target.position.z) if transform is None else
                   transform_position(target.position, transform))
        except Exception as exc:
            raise MotionError(f'observation-time target TF unavailable: {exc}') from exc
        if not side_and_window(xyz, side, self.p['target_longitudinal_offset_m'],
                               self.p['longitudinal_window_m'], self.p['maximum_target_distance_m'],
                               self.p['target_minimum_height_m'], self.p['target_maximum_height_m']):
            raise MotionError('target outside current work-side/window')
        output = copy.deepcopy(target)
        output.position.x, output.position.y, output.position.z = xyz
        output_header = copy.deepcopy(header)
        output_header.frame_id = self.p['task_frame']
        return output, output_header

    def _select_after_scan(self, request):
        self._set_phase('SCANNING')
        if self.p['scan_mode'] == 'preset':
            self.arm.move_joints(self.p[f'scan_{request.work_side}_joints'])
            self._scan_diagnostics = {'mode': 'preset', 'work_side': request.work_side}
            target = self._wait_scan_target(request, self.p['selection_timeout_s'])
            if target is not None:
                return target
            raise MotionError('no new acceptable target after scan pose settled')
        # Both frames share a rigid wrist mount. Compose in the standard task
        # frame first: CAD base_link does not use task +Z as its vertical axis.
        try:
            relative = self.tf.lookup_transform(self.p['scan_camera_frame'],
                                                self.p['tip_link'], Time())
            t, q = relative.transform.translation, relative.transform.rotation
            camera_to_tcp = ((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        except Exception as exc:
            raise MotionError(f'camera-to-TCP scan TF unavailable: {exc}') from exc
        heights = descending_heights(self.p['scan_top_height_m'], self.p['scan_bottom_height_m'],
                                     self.p['scan_height_step_m'])
        self._scan_diagnostics = {'mode': 'vertical', 'frame_id': self.p['task_frame'],
                                  'work_side': request.work_side, 'heights_m': heights,
                                  'levels': [], 'selected_target_id': ''}
        started = False
        for height in heights:
            self.arm._check()
            optical = camera_pose(request.work_side, self.p['scan_camera_x_m'],
                                  self.p['scan_camera_lateral_m'], height)
            tcp = tcp_pose_for_camera(*optical, *camera_to_tcp)
            planning = self._to_planning(*tcp)
            entry = {'camera_xyz_m': list(optical[0]), 'height_m': height,
                     'state': 'PLANNING'}
            self._scan_diagnostics['levels'].append(entry)
            if not started:
                solution = self.arm.ik(*planning)
                if solution is None:
                    entry.update(state='SKIPPED_UNREACHABLE_ENTRY',
                                 ik=getattr(self.arm, 'last_ik_status', {}))
                    continue
                values = dict(zip(solution.joint_state.name, solution.joint_state.position))
                try:
                    planned = self.arm.plan_joints([values[name] for name in self.arm.joints])
                except MotionError as exc:
                    # Only a rejected, unexecuted candidate can be skipped.
                    # A lost permit, service/scene error or executed motion
                    # must propagate instead of continuing the search.
                    self.arm._check()
                    if (not self.arm.terminal_confirmed or not str(exc).startswith(
                            ('joint plan failed:', 'planned endpoint near physical bound'))):
                        raise
                    entry.update(state='SKIPPED_UNREACHABLE_ENTRY', reason=str(exc))
                    continue
                self.get_logger().info(f'vertical scan entry: camera z={height:.2f} m')
                self.arm.move_pose(*planning, solution=solution, planned=planned)
                started = True
            else:
                self.get_logger().info(f'vertical scan descending: camera z={height:.2f} m')
                self.arm.cartesian(*planning)
            entry['state'] = 'OBSERVING'
            target = self._wait_scan_target(request, self.p['scan_dwell_s'])
            if target is not None:
                entry['state'] = 'TARGET_FOUND'
                self._scan_diagnostics['selected_target_id'] = target[0].target_id
                return target
            entry['state'] = 'NO_ACCEPTABLE_TARGET'
        raise MotionError('vertical scan found no new stable mature target' if started else
                          'no reachable vertical scan entry pose')

    def _wait_scan_target(self, request, timeout_s):
        # Every level starts its own post-settlement image-time window. Never
        # use an observation captured during the previous camera movement.
        earliest_ns = self.get_clock().now().nanoseconds
        deadline = time.monotonic()+timeout_s
        while time.monotonic() < deadline:
            self.arm._check()
            with self._mutex:
                array, received = self._targets, self._targets_wall
            if (array is not None and received is not None and
                    time.monotonic()-received <= self.p['target_max_age_s'] and
                    array.observation_valid and
                    fresh(stamp_ns(array.depth_stamp), self.get_clock().now().nanoseconds,
                          self.p['target_max_age_s']) and
                    stamp_ns(array.depth_stamp) >= earliest_ns and
                    abs(stamp_ns(array.depth_stamp)-stamp_ns(array.header.stamp)) <= 50_000_000):
                candidates = []
                for target in array.targets:
                    try:
                        candidate = self._validate_target(target, array.header, request.work_side,
                                                          earliest_ns)
                        candidates.append(candidate)
                    except (MotionError, ValueError):
                        continue
                if candidates:
                    return max(candidates, key=lambda candidate: candidate[0].confidence)
            time.sleep(0.05)
        return None

    def _to_planning(self, position, orientation):
        # The task frame is chassis-fixed. Image-time TF was already applied to
        # the observation; this TF carries geometry poses into the CAD frame.
        try:
            transform = self.tf.lookup_transform(self.p['planning_frame'], self.p['task_frame'], Time())
            return transform_pose(position, orientation, transform)
        except Exception as exc:
            raise MotionError(f'task-to-planning rigid TF unavailable: {exc}') from exc

    def _scene_request(self, operation):
        if operation == 'ready':
            request = SimFruitCommand.Request()
            request.header.stamp = self.get_clock().now().to_msg()
            request.header.frame_id = self.p['task_frame']
            request.operation = operation
            request.work_id = 'readiness:'+self.get_fully_qualified_name()
            if self._request is not None:
                request.work_id, request.plant_id = self._request.work_id, self._request.plant_id
            return request
        target, header = self._observation
        request = SimFruitCommand.Request()
        request.header = copy.deepcopy(header)
        request.target_id, request.work_id = target.target_id, self._request.work_id
        request.plant_id, request.position = self._request.plant_id, copy.deepcopy(target.position)
        request.operation = operation
        return request

    def _scene(self, operation, check=True, timeout_s=None):
        if operation == 'wait_in_basket':
            deadline = time.monotonic()+self.p['basket_verification_timeout_s']
            while time.monotonic() < deadline:
                response = self._scene('status')
                if response.state == 'IN_BASKET':
                    return response
                if response.state == 'LOST':
                    raise MotionError('released fruit was lost outside the basket')
                time.sleep(0.1)
            raise MotionError('stable fruit-in-basket state was not confirmed')
        client = self._scene_clients['contact' if operation in {'select', 'clear', 'ready'} else 'fruit']
        if not client.service_is_ready():
            raise MotionError(f'{operation} simulation/scene service unavailable')
        started = time.monotonic()
        deadline = started+min(self.p['scene_service_timeout_s'],
                              self.p['scene_service_timeout_s'] if timeout_s is None else timeout_s)
        attempts, last_error = 0, ''
        while True:
            if check:
                self.arm._check()
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise MotionError(f'{operation} total scene deadline reached: {last_error}')
            attempts += 1
            try:
                response = self.arm._wait(
                    client.call_async(self._scene_request(operation)),
                    min(remaining, self.p['scene_ready_attempt_timeout_s'])
                    if operation in {'ready', 'check_grip'} else remaining, operation, check)
                if response.success:
                    if operation == 'ready':
                        self._scene_ready_diagnostics = {'attempts': attempts,
                            'elapsed_wall_s': time.monotonic()-started, 'state': response.state}
                    return response
                last_error = f'{operation} rejected: {response.message}'
                retryable = response.state == 'BUSY' or self._transport_timeout(last_error)
            except MotionError as exc:
                last_error = str(exc)
                retryable = self._transport_timeout(last_error)
            if operation not in {'ready', 'check_grip'} or not retryable:
                raise MotionError(last_error)
            if operation == 'ready':
                self._scene_ready_diagnostics = {'attempts': attempts,
                    'elapsed_wall_s': time.monotonic()-started, 'state': 'RETRYING',
                    'last_error': last_error}
            # Read-only readiness/grip queries may be repeated; contact selection
            # and physical commands stay one-shot after ambiguous transport loss.
            pause_until = min(deadline, time.monotonic()+self.p['scene_ready_retry_interval_s'])
            while time.monotonic() < pause_until:
                if check:
                    self.arm._check()
                time.sleep(.01)

    @staticmethod
    def _transport_timeout(message):
        text = message.lower()
        return any(token in text for token in ('timed out', 'timeout', 'host unreachable'))

    def _environment_ready(self, check=True):
        if self.p['backend_mode'] == 'mock':
            return
        response = self._scene('ready', check=check)
        if response.state != 'SCENE_READY':
            raise MotionError(f'complete current collision scene unconfirmed: {response.state}')

    def _execute(self, handle):
        self._handle = handle
        result = HarvestWork.Result()
        verified = False
        self._grasp_diagnostics = []
        self._contact_diagnostics = {}
        self._scan_diagnostics = {}
        self._selected_target = {}
        self._last_error = ''
        try:
            # An accepted work starts from measured stow. A failed new scan
            # must not report the target from a previous completed station.
            self._observation = None
            if getattr(self, '_ideal_simulation', False):
                # Actual backend/controller readiness remains a work-start
                # check, not a continuous condition that locks the chassis.
                self.arm.ready()
            self._environment_ready()
            request = handle.request
            if getattr(request, 'scan_before_selection', False):
                self._observation = self._select_after_scan(request)
            else:
                self._observation = self._validate_target(
                    request.target, request.header, request.work_side,
                    int(self._station['station_started_ns']))
            target, header = self._observation
            self._target_history.setdefault(target.target_id, set()).add(self._work_context[:2])
            xyz = (target.position.x, target.position.y, target.position.z)
            self._selected_target = {
                'frame_id': header.frame_id, 'position_xyz_m': list(xyz),
                'target_id': target.target_id, 'class_name': target.class_name,
                'confidence': target.confidence, 'mature': target.mature,
                'observation_stamp_ns': stamp_ns(header.stamp),
                'continuous_visual_detection_required': False,
            }
            chain = GraspExecutor(self.arm, self.p, self._to_planning, self._scene,
                                  self._set_phase, self._record_grasp_diagnostics,
                                  self._record_contact_diagnostics)
            verified = chain.execute(xyz, request.work_side)
            self.arm._check()
            self._set_phase('STOWED')
            measured_stowed = self.arm.stowed()
            ideal = getattr(self, '_ideal_simulation', False)
            result.success = bool(verified and (ideal or measured_stowed))
            result.arm_stowed = bool(measured_stowed)
            result.sim_fruit_verified = bool(verified)
            result.target_id = target.target_id
            result.message = ('simulation fruit IN_BASKET confirmed; ideal simulation does not gate '
                              'base movement on arm stow' if ideal else
                              'simulation fruit IN_BASKET and measured six-joint STOWED confirmed')
            if not result.success:
                raise MotionError('fruit or measured arm stow verification failed')
            handle.succeed()
        except Exception as exc:
            self._last_error = str(exc)
            terminal = self.arm.cancel_and_confirm()
            # Retraction needs an explicit guarded recovery request after fault;
            # never plan over an unconfirmed active trajectory or auto-unlock.
            self._fault = True
            self._set_phase('FAULT' if terminal else 'FAULT_MOTION_UNCONFIRMED')
            result.success = False
            result.arm_stowed = False
            result.sim_fruit_verified = bool(verified)
            result.target_id = self._observation[0].target_id if self._observation else ''
            result.message = (f'{exc}; ideal simulation does not request a protective base lock'
                if getattr(self, '_ideal_simulation', False) else
                f'{exc}; base lock retained; measured guarded recovery required')
            self.get_logger().error(result.message)
            if handle.is_cancel_requested and terminal:
                handle.canceled()
            else:
                handle.abort()
        finally:
            with self._mutex:
                self._busy = False
                self._handle = None
                self._started_wall = None
            self._heartbeat()
        return result

    def _record_grasp_diagnostics(self, candidates):
        with self._mutex:
            self._grasp_diagnostics = copy.deepcopy(candidates)

    def _record_contact_diagnostics(self, report):
        with self._mutex:
            self._contact_diagnostics = copy.deepcopy(report)

    def _recover(self, _request, response):
        with self._mutex:
            if (self._busy or not self._fault or
                    (not getattr(self, '_ideal_simulation', False) and not self._healthy) or
                    not self._parked()):
                response.success = False
                response.message = ('recovery needs idle FAULT and measured locked/stopped base; '
                                    f'busy={self._busy}, fault={self._fault}, '
                                    f'backend_ready={self._healthy}, parked={self._parked()}')
                return response
            if not self.arm.cancel_and_confirm():
                response.success, response.message = False, 'active trajectory terminal result unconfirmed'
                return response
            self._busy = True
            self._cancel.clear()
        old_permit = self.arm.check_permit
        # FAILED route may no longer offer a work permit. Recovery may move only
        # with independently fresh lock+zero-velocity evidence and healthy state.
        self.arm.check_permit = lambda: (getattr(self, '_ideal_simulation', False) or
                                       self._healthy) and self._parked()
        try:
            self._set_phase('RECOVERING_STOW')
            if self._observation is not None:
                self._scene('clear')
            self.arm.move_joints(self.p['stow_joints'])
            if not self.arm.stowed():
                raise MotionError('stow feedback unconfirmed')
            self._fault = False
            self._set_phase('STOWED')
            response.success, response.message = True, 'measured STOWED; route reset remains an explicit operation'
        except Exception as exc:
            self.arm.cancel_and_confirm()
            self._set_phase('FAULT')
            response.success, response.message = False, str(exc)
        finally:
            self.arm.check_permit = old_permit
            self._busy = False
        return response

    def _force_initialize_sim_hold(self, request, response):
        """Explicit Gazebo bootstrap without the base parking/settlement gate."""
        return self._initialize_sim_hold(request, response, force=True)

    def _initialize_sim_hold(self, _request, response, force=False):
        """Explicit parked simulation bootstrap when JTC has never published status."""
        with self._mutex:
            if self._busy or self.p['backend_mode'] != 'gazebo':
                response.success = False
                response.message = ('initialization needs idle Gazebo; '
                                    f'busy={self._busy}, backend_mode={self.p["backend_mode"]}')
                return response
            if force and self._observation is not None:
                response.success, response.message = False, 'finish or recover the current fruit transaction before forced initialization'
                return response
            if not force and not self._parked():
                response.success, response.message = False, 'initialization needs idle Gazebo and locked/stopped base'
                return response
            self._busy, self._phase = True, 'INITIALIZING_SIM_HOLD'
            self._cancel.clear()
        old_permit = self.arm.check_permit
        def permit():
            now, wall = self.get_clock().now().nanoseconds, time.monotonic()
            return ((force or self._parked()) and
                    self.evidence.available((*self.p['arm_joint_names'],
                                             *self.p['gripper_joint_names']), now, wall))
        self.arm.check_permit = permit
        try:
            self.arm.ready(require_status=False)
            self._environment_ready(check=False)
            deadline = time.monotonic()+self.p['feedback_timeout_s']
            if force:
                self.get_logger().warning('explicit simulation-only forced initialization: '
                    'bypassing base parking and pre-hold settlement; commanding measured current positions')
                self.arm._check()
            while not force and not (self.evidence.stationary(
                    self.p['arm_joint_names'], self.get_clock().now().nanoseconds,
                    time.monotonic(), self.p['joint_velocity_tolerance_rps'], self.p['joint_settle_s']) and
                    self.evidence.stationary(
                    self.p['gripper_joint_names'], self.get_clock().now().nanoseconds,
                    time.monotonic(), self.p['gripper_velocity_tolerance_mps'], self.p['joint_settle_s'])):
                self.arm._check()
                if time.monotonic() >= deadline:
                    raise MotionError('fresh stationary arm and fingers required for initialization')
                time.sleep(.02)
            self.arm.prepare_initialization_hold()
            for label in ('arm', 'gripper'):
                self.arm.hold_current_for_initialization(label)
            deadline = time.monotonic()+self.p['goal_response_timeout_s']
            while not self.arm.terminal_confirmed:
                self.arm._check()
                if time.monotonic() >= deadline:
                    raise MotionError('two actual controller status snapshots remain unconfirmed')
                time.sleep(.02)
            self.arm.ready()
            self._healthy = True
            if force:
                self._fault = False
                self._startup_checked = False
            response.success = True
            response.message = ('forced simulation initialization: ' if force else '') + \
                'two real current-position hold goals terminal; startup pose checked separately'
        except Exception as exc:
            self.arm.cancel_and_confirm()
            self._healthy, self._fault = False, True
            response.success, response.message = False, str(exc)
        finally:
            self.arm._initialization_idle_labels.clear()
            self.arm.check_permit = old_permit
            with self._mutex:
                self._busy = False
                self._phase = 'FAULT' if self._fault else 'IDLE'
            self._detect_startup_pose()
            if response.success and self._fault:
                response.message += '; measured non-stow arm requires recover_stow'
        return response

    def _heartbeat(self):
        with self._mutex:
            # STEADY timers and execute-finally can run between /clock updates.
            # Keep the real sample time; never manufacture increasing stamps.
            now = self.get_clock().now()
            if not self._healthy:
                denial = 'backend_unhealthy'
            elif self._fault:
                denial = 'worker_fault'
            elif self._busy:
                denial = 'worker_busy'
            elif self._phase not in {'IDLE', 'STOWED'}:
                denial = 'active_arm_phase'
            else:
                denial = ''
            if denial:
                self.arm.reset_navigation_stow()
                stowed = False
            else:
                stowed = self.arm.navigation_stowed()
            navigation_report = self.arm.navigation_stow_report()
            if not stowed and not denial:
                denial = ','.join(navigation_report['reasons'])
            payload = (self.p['task_frame'], *self._work_context,
                       self._phase, bool(stowed))
            heartbeat = (now.nanoseconds, payload)
            if heartbeat == self._last_heartbeat:
                return
            # A changed report at the same stamp must still reach the base gate.
            # Publishing while holding this lock also orders timer/finally calls.
            state = HarvestWorkerState()
            state.header.stamp = now.to_msg()
            state.header.frame_id = self.p['task_frame']
            state.run_id, state.waypoint_id, state.work_id = self._work_context
            state.phase, state.arm_stowed = self._phase, bool(stowed)
            self._pub.publish(state)
            self._last_heartbeat = heartbeat
            self._status_pub.publish(String(data=json.dumps({
                'phase': self._phase, 'busy': self._busy, 'fault': self._fault,
                'ideal_simulation': getattr(self, '_ideal_simulation', False),
                'startup_pose_checked': self._startup_checked,
                'backend_ready': self._healthy, 'measured_stowed': bool(stowed),
                'last_health_error': self._last_health_error,
                'stow_denial_reason': denial,
                'stow_evidence': self.arm.stow_report(),
                'navigation_stow_evidence': navigation_report,
                'motion_terminal_confirmed': self.arm.terminal_confirmed,
                'motion': self.arm.execution_status,
                'grasp_candidates': self._grasp_diagnostics,
                'grip_closure': self._contact_diagnostics,
                'scan': self._scan_diagnostics,
                'selected_target': self._selected_target,
                'last_error': self._last_error,
                'scene_readiness': self._scene_ready_diagnostics,
                'context': self._work_context, 'actual_grasp_verified': False,
                'tcp_calibration': 'geometric_candidate_pending_physical_calibration',
                'stamp_ns': now.nanoseconds}, sort_keys=True)))

    def close(self):
        self._closing.set()
        self._cancel.set()
        self.arm.cancel_and_confirm()
        self._health_thread.join(timeout=2.)

    def destroy_node(self):
        """Destroy waitables only after the owning executor stopped."""
        self.arm.destroy()
        self._server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HarvestArmWorker()
    except Exception:
        if rclpy.ok():
            rclpy.shutdown()
        raise
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        executor.shutdown(timeout_sec=6.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
