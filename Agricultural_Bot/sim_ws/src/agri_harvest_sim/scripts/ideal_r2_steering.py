#!/usr/bin/env python3
"""Explicit simulation-only ideal r2 steering, preserving upstream safeguards."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
from agri_harvest_interfaces.msg import HarvestWorkerState
from controller_manager_msgs.srv import ListHardwareComponents
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.double_pb2 import Double
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode, NodeOptions
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

from ideal_steering_command import IdealSteeringCommand
from ideal_status_process import NativeStatusProcess, NativeStatusRPCUnavailable


class NativeStatusUnavailable(RuntimeError):
    """Only an absent status service permits loading a new system."""


class NativeStatusPending(RuntimeError):
    """An advertised plugin has not published its first full update yet."""


class IdealR2Steering(Node):
    def __init__(self, args):
        super().__init__('ideal_r2_steering_compensator', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        if self.get_parameter('use_sim_time').value is not True:
            raise RuntimeError('ideal steering requires use_sim_time=true')
        if os.environ.get('GZ_PARTITION') != args.gz_partition:
            raise RuntimeError('explicit partition must match GZ_PARTITION in this terminal')
        self.args = args
        options = NodeOptions(); options.partition = args.gz_partition
        self.gz = GzNode(options)
        if self.gz.options.partition != args.gz_partition:
            raise RuntimeError('Gazebo transport partition mismatch')
        if args.ideal_simulation:
            self.command_topic = f'/world/{args.world_name}/ideal_steering/four_wheel_commands'
            self.status_service = f'/world/{args.world_name}/ideal_steering/four_wheel_status'
            self.gz_publisher = self.gz.advertise(self.command_topic, Double_V)
        else:
            self.command_topic = f'/world/{args.world_name}/ideal_steering/r2_command'
            self.status_service = f'/world/{args.world_name}/ideal_steering/status'
            self.gz_publisher = self.gz.advertise(self.command_topic, Double)
        self.states, self.receipts = {}, {}
        self.validator = None
        self.armed = False
        self.ever_armed = False
        self.suspended = False
        self.resume_pending = False
        self._parking_transition_deadline = None
        self._parking_lock_requested = False
        self.failed = False
        self.reason = 'checking parked simulation and command ownership'
        self.requires_worker = True
        self.sent = 0
        self.last_native = None
        self.status_process = None
        self.native_receipt_wall = None
        self.reference_ns = 0
        self.next_status_at = 0.
        self.ideal_error_count = 0
        self._ideal_startup_pending = False
        self._ideal_startup_error = ''
        self._ideal_startup_next_poll = 0.
        self._ideal_startup_next_log = 0.
        self._ideal_load_attempted = False
        self.status_publisher = self.create_publisher(String, '~/status', 10)
        self.lock_publisher = self.create_publisher(Bool, '/base_motion/lock_cmd', 10)
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for key, typ, topic, quality in (
                ('locked', Bool, '/base_motion/locked', transient),
                ('odom', Odometry, '/odom', 10),
                ('joints', JointState, '/joint_states', 10),
                ('worker', HarvestWorkerState, '/harvest_worker/state', 10)):
            self.create_subscription(typ, topic, lambda m,k=key:self.receive(k,m), quality)
        self.create_subscription(Float64MultiArray, '/steering_controller/commands',
                                 self.reference, QoSProfile(depth=1))
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.02, self.watchdog, clock=self._steady)

    def receive(self, key, message):
        self.states[key], self.receipts[key] = message, time.monotonic()

    def spin_until(self, predicate, timeout):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)
            if predicate():return
        raise RuntimeError('fresh simulation startup evidence deadline expired')

    def parameters(self, node, names):
        client = self.create_client(GetParameters, node+'/get_parameters')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('parameter service unavailable: '+node)
        future = client.call_async(GetParameters.Request(names=names))
        self.spin_until(future.done, 5.)
        return future.result().values

    @staticmethod
    def ns(stamp):return stamp.sec*10**9+stamp.nanosec

    @property
    def ideal_simulation(self):
        return bool(getattr(self.args, 'ideal_simulation', False))

    @property
    def wall_feedback_timeout(self):
        return getattr(self.args, 'wall_feedback_timeout', .5)

    def feedback(self):
        now, wall = self.get_clock().now().nanoseconds, time.monotonic()
        for key in ('locked','odom','joints')+ (('worker',) if self.requires_worker else ()):
            if key not in self.states or wall-self.receipts[key] > self.wall_feedback_timeout:
                raise RuntimeError('fresh '+key+' feedback missing')
        for key in ('joints','odom')+ (('worker',) if self.requires_worker else ()):
            stamp = self.ns(self.states[key].header.stamp)
            if stamp <= 0 or not -.05 <= (now-stamp)/1e9 <= .25:
                raise RuntimeError(key+' feedback is stale, future or zero')
        twist = self.states['odom'].twist.twist
        if not all(math.isfinite(v) for v in
                   (twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z)):
            raise RuntimeError('nonfinite real odometry feedback')
        joint = self.states['joints']
        if joint.name.count('r2_steer_joint') != 1:
            raise RuntimeError('unambiguous real r2 joint feedback required')
        i = joint.name.index('r2_steer_joint')
        if i >= len(joint.position) or i >= len(joint.velocity):
            raise RuntimeError('incomplete real r2 joint feedback')
        if not all(math.isfinite(v) for v in (joint.position[i],joint.velocity[i])):
            raise RuntimeError('nonfinite real r2 joint feedback')
        return joint.position[i], self.ns(joint.header.stamp)

    def stopped(self):
        t = self.states['odom'].twist.twist
        # Match the worker's StoppedMotion criteria, including total linear
        # speed and its angular bound. A different threshold creates a race
        # where the worker accepts parked work but compensation faults on it.
        speeds = (t.linear.x, t.linear.y, t.linear.z, t.angular.z)
        return (all(math.isfinite(v) for v in speeds) and
                math.sqrt(sum(v*v for v in speeds[:3])) < .01 and abs(speeds[3]) < .02)

    @property
    def parking_transition_timeout(self):
        return getattr(self.args, 'parking_transition_timeout', 2.)

    def movement_state(self):
        worker, locked, odom = (self.states.get(key) for key in ('worker', 'locked', 'odom'))
        t = None if odom is None else odom.twist.twist
        return {'worker_phase': None if worker is None else worker.phase,
                'worker_arm_stowed': None if worker is None else bool(worker.arm_stowed),
                'base_locked': None if locked is None else bool(locked.data),
                'base_linear_xyz_mps': None if t is None else [t.linear.x, t.linear.y, t.linear.z],
                'base_angular_z_rps': None if t is None else t.angular.z}

    def park_for_transition(self):
        if self.ideal_simulation:
            return True
        wall = time.monotonic()
        deadline = getattr(self, '_parking_transition_deadline', None)
        if deadline is not None and wall >= deadline:
            raise RuntimeError('arm interlock transition did not reach a locked/stopped base '
                               'within the parking deadline')
        if self.states['locked'].data and self.stopped():
            self._parking_transition_deadline = None
            self._parking_lock_requested = False
            return True
        if deadline is None:
            deadline = wall+self.parking_transition_timeout
            self._parking_transition_deadline = deadline
        if not getattr(self, '_parking_lock_requested', False):
            self.lock_publisher.publish(Bool(data=True))
            self._parking_lock_requested = True
        self.reason = 'ideal steering suspended; waiting for fresh locked/stopped base acknowledgement'
        return False

    def reference_fresh(self):
        age = (self.get_clock().now().nanoseconds-self.reference_ns)/1e9
        return (self.reference_ns > 0 and -.05 <= age <= .2 and
                time.monotonic()-self.receipts.get('reference',0) <= self.wall_feedback_timeout)

    def movement_permission(self):
        if self.ideal_simulation:
            return True
        if not self.requires_worker:
            return True
        w = self.states['worker']
        if w.arm_stowed and w.phase in {'IDLE','STOWED'}:
            # Independent ROS topics can arrive in different callback order.
            # Once permission was withdrawn, never reclaim before the lock and
            # stopped feedback catch up, even if stow has already returned.
            if self.resume_pending and not self.park_for_transition():
                return False
            if self.suspended:
                self.reason = 'measured stow restored; fresh matching reference required'
            self.suspended = False
            return True
        # Parked arm work yields control to the normal actuator. Resume requires
        # a fresh target matching actual position, rather than replaying a cache.
        self.suspended = True
        self.resume_pending = True
        self.reason = 'parked arm work: ideal steering suspended until measured stow'
        self.validator.invalidate(self.reason)
        # Suspend before requesting a lock: do not send or renew an ideal
        # reference while normal braking and lock acknowledgement complete.
        self.park_for_transition()
        return False

    def owned_source(self):
        sources = self.get_publishers_info_by_topic('/steering_controller/commands')
        return len(sources)==1 and sources[0].node_name=='four_wheel_steering_node'

    def validate_native_status(self, result, require_ready=None):
        if (isinstance(result, dict) and result.get('idealized') is True and
                result.get('ready') is False and
                set(result) <= {'idealized', 'ready', 'active'}):
            raise NativeStatusPending('plugin advertised; waiting for its first simulation update')
        if not isinstance(result, dict):
            raise RuntimeError('native steering status must identify a Gazebo plugin')
        ideal_identity = self.ideal_simulation and (
            result.get('system') == 'agri::IdealFourWheelSteeringSystem' and
            result.get('version') == 'four_wheel_ideal_steering_v1' and
            result.get('joint_names') == ['f1_steer_joint', 'f2_steer_joint',
                                          'r1_steer_joint', 'r2_steer_joint'])
        strict_identity = (not self.ideal_simulation and
            result.get('system') == 'agri::IdealSteeringSystem' and
            result.get('version') == 'r2_ideal_steering_v1' and
            result.get('joint_name') == 'r2_steer_joint')
        if (result.get('idealized') is not True or
                (not ideal_identity and not strict_identity) or
                result.get('enabled') is not True or
                result.get('robot_name') != self.args.robot_name or
                result.get('command_topic') != self.command_topic):
            raise RuntimeError('Gazebo ideal steering identity mismatch')
        if bool(result.get('ideal_simulation', False)) != self.ideal_simulation:
            raise RuntimeError('native steering mode differs from this terminal; restart Gazebo '
                               'with the current harvest_demo_env.bash')
        if require_ready is None:
            require_ready = not self.ideal_simulation
        if require_ready and result.get('ready') is not True:
            raise NativeStatusPending('native steering is not ready: '+str(result.get('expiry_reason')))
        return result

    def native_status(self):
        # Startup only. Periodic checks use a spawned status-only process: the
        # installed Gazebo Python request call holds its process's GIL.
        request = StringMsg();request.data = json.dumps({'op':'status'})
        ok, reply = self.gz.request(self.status_service, request, StringMsg, StringMsg, 250)
        if not ok:raise NativeStatusUnavailable('ideal steering system status unavailable')
        result = self.validate_native_status(json.loads(reply.data))
        self.last_native = result
        self.native_receipt_wall = time.monotonic()
        return result

    def poll_native_status(self):
        process = getattr(self, 'status_process', None)
        if process is None:
            return None
        packet = process.poll()
        if packet is not None:
            result = self.validate_native_status(packet['data'])
            sim_ns = result.get('sim_ns')
            now = self.get_clock().now().nanoseconds
            if (type(sim_ns) is not int or sim_ns <= 0 or
                    not -.05 <= (now-sim_ns)/1e9 <= .25 or
                    now < packet['submitted_sim_ns']):
                raise RuntimeError('native status subprocess result stale, future or clock reversed')
            self.last_native = result
            self.native_receipt_wall = packet['completed_wall']
        return packet

    def warm_native_status(self):
        try:packet = self.poll_native_status()
        except NativeStatusRPCUnavailable:
            # First-participant discovery may take more than one 250 ms RPC.
            # Only this unarmed, bounded five-second status-only phase retries.
            if self.armed:raise
            return False
        if packet is not None:return True
        if self.status_process.ready and self.status_process.inflight is None:
            self.status_process.submit(self.get_clock().now().nanoseconds)
        return False

    def startup_feedback_ready(self):
        try:
            self.feedback()
            return self.reference_fresh()
        except RuntimeError:return False

    def parked_stow_startup_ready(self):
        """Wait for measured permission without changing arm or gate state."""
        if not self.startup_feedback_ready():return False
        if not self.states['locked'].data or not self.stopped():return False
        if self.requires_worker:
            worker=self.states['worker']
            if not worker.arm_stowed or worker.phase not in {'IDLE','STOWED'}:return False
        return True

    def wait_for_parked_stow_startup(self):
        try:self.spin_until(self.parked_stow_startup_ready,5.)
        except RuntimeError as error:
            worker=self.states.get('worker')
            raise RuntimeError('startup requires fresh measured stow and locked/stopped base; '
                f'worker_phase={getattr(worker,"phase",None)}, '
                f'arm_stowed={getattr(worker,"arm_stowed",None)}; '
                'feedback_ages='+json.dumps(self.feedback_ages(),sort_keys=True)) from error

    def feedback_ages(self):
        wall, now = time.monotonic(), self.get_clock().now().nanoseconds
        return {key:{'receipt_age_wall_s':wall-self.receipts[key],
            'stamp_age_sim_s':(now-self.ns(message.header.stamp))/1e9
                if hasattr(getattr(getattr(message,'header',None),'stamp',None),'nanosec') else None}
            for key,message in self.states.items() if key in self.receipts}

    def start(self):
        urdf = self.parameters('/robot_state_publisher',['robot_description'])[0].string_value
        model = ET.fromstring(urdf)
        joints = self.parameters('/steering_controller',['joints','interface_name'])
        expected = [f'{prefix}_steer_joint' for prefix in ('f1','f2','r1','r2')]
        if list(joints[0].string_array_value)!=expected or joints[1].string_value!='position':
            raise RuntimeError('original four-wheel position controller joint order required')
        components = self.create_client(ListHardwareComponents,
                                        '/controller_manager/list_hardware_components')
        if not components.wait_for_service(timeout_sec=5.):
            raise RuntimeError('actual Gazebo controller manager required')
        future=components.call_async(ListHardwareComponents.Request())
        self.spin_until(future.done,5.)
        controls={c.get('name'):c.findtext('hardware/plugin') for c in model.findall('ros2_control')}
        hardware=future.result().component
        if not hardware or any((c.plugin_name or controls.get(c.name)) !=
                                'gz_ros2_control/GazeboSimSystem' for c in hardware):
            raise RuntimeError('ideal compensation refuses non-Gazebo hardware')
        limit=model.find("joint[@name='r2_steer_joint']/limit")
        if limit is None:raise RuntimeError('runtime r2 URDF limits missing')
        self._ideal_limits = (float(limit.get('lower')), float(limit.get('upper')))
        if self.ideal_simulation:
            self.start_ideal_operation()
            return
        self.validator=IdealSteeringCommand(float(limit.get('lower')),float(limit.get('upper')),
                                          wall_timeout=self.wall_feedback_timeout)
        self.requires_worker=self.parameters('/base_velocity_gate',
            ['require_harvest_interlock'])[0].bool_value
        self.spin_until(lambda: all(k in self.states for k in
            ('locked','odom','joints','reference')+(('worker',)if self.requires_worker else ())),15.)
        self.wait_for_parked_stow_startup()
        self.feedback()
        if not self.states['locked'].data or not self.stopped():
            raise RuntimeError('park and lock the base before enabling ideal steering')
        if not self.movement_permission():
            raise RuntimeError('measured stow required before enabling ideal steering')
        if not self.owned_source():raise RuntimeError('one original steering publisher required')
        if sum(name=='ideal_r2_steering_compensator' for name,_ in
               self.get_node_names_and_namespaces())!=1:
            raise RuntimeError('another ideal steering compensator already exists')
        actual,_=self.feedback()
        if not self.reference_fresh() or len(self.states['reference'].data)!=4 or not all(
                math.isfinite(v) for v in self.states['reference'].data) or abs(
                self.states['reference'].data[3]-actual)>.03:
            raise RuntimeError('current steering reference must match parked actual r2 within .03 rad')
        try:self.native_status()
        except NativeStatusUnavailable:
            if not self.args.load_system:raise
            library=Path(self.args.plugin_path) if self.args.plugin_path else (
                Path(get_package_prefix('agri_harvest_sim'))/'lib/libagri_ideal_steering_system.so')
            if not library.is_file():raise RuntimeError('build agri_harvest_sim before loading ideal steering')
            request=EntityPlugin_V();request.entity.name=self.args.world_name
            request.entity.type=Entity.WORLD
            plugin=request.plugins.add();plugin.name='agri::IdealSteeringSystem'
            plugin.filename=str(library.resolve())
            plugin.innerxml=(f'<enabled>true</enabled><robot_name>{self.args.robot_name}</robot_name>'
                             '<joint_name>r2_steer_joint</joint_name>'
                             '<ideal_simulation>false</ideal_simulation>')
            ok,reply=self.gz.request(f'/world/{self.args.world_name}/entity/system/add',
                                     request, EntityPlugin_V, Boolean,3000)
            if not ok or not reply.data:raise RuntimeError('explicit ideal steering plugin load failed')
            self.spin_until(lambda:self.system_ready(),5.)
        self.spin_until(self.startup_feedback_ready,5.)
        actual,_=self.feedback()
        if (not self.states['locked'].data or not self.stopped() or
                not self.movement_permission() or not self.reference_fresh() or
                not self.owned_source() or
                abs(self.states['reference'].data[3]-actual)>.03):
            raise RuntimeError('parked and matching startup evidence changed while loading system')
        self.status_process = NativeStatusProcess(self.args.gz_partition,self.status_service)
        # Import/client creation/discovery occurs before arming. Keep processing
        # real ROS feedback while awaiting the first bounded, identified status.
        self.spin_until(self.warm_native_status,5.)
        actual,_=self.feedback()
        if (not self.states['locked'].data or not self.stopped() or
                not self.movement_permission() or not self.reference_fresh() or
                not self.owned_source() or abs(self.states['reference'].data[3]-actual)>.03):
            raise RuntimeError('parked startup evidence changed while warming status subprocess')
        self.armed=True
        self.ever_armed=True
        self.reason='explicit simulation-only ideal r2 mode enabled'
        self.get_logger().warning(self.reason+'; torque and tire scrub are idealized for this joint')

    def start_ideal_operation(self):
        """Enable ideal actuation without arm, parking or feedback leases."""
        self.requires_worker = False
        result = None
        missing = False
        # A first 250 ms discovery miss is not proof that a plugin is absent.
        # Probe a few times before the one allowed add request.
        for _attempt in range(3):
            try:
                result = self.native_status()
                missing = False
                break
            except NativeStatusPending as error:
                self.reason = str(error)
                missing = False
                break
            except NativeStatusUnavailable as error:
                self.reason = str(error)
                missing = True
                rclpy.spin_once(self, timeout_sec=.05)
        if missing and self.args.load_system:
            library = Path(self.args.plugin_path) if self.args.plugin_path else (
                Path(get_package_prefix('agri_harvest_sim'))/
                'lib/libagri_ideal_four_wheel_steering_system.so')
            if not library.is_file():
                raise RuntimeError('build agri_harvest_sim before loading ideal steering')
            request=EntityPlugin_V();request.entity.name=self.args.world_name
            request.entity.type=Entity.WORLD
            plugin=request.plugins.add();plugin.name='agri::IdealFourWheelSteeringSystem'
            plugin.filename=str(library.resolve())
            plugin.innerxml=(f'<enabled>true</enabled><robot_name>{self.args.robot_name}</robot_name>'
                             '<ideal_simulation>true</ideal_simulation>')
            self._ideal_load_attempted = True
            ok,reply=self.gz.request(f'/world/{self.args.world_name}/entity/system/add',
                                     request, EntityPlugin_V, Boolean,3000)
            self.reason = ('native plugin load requested; waiting for actual readiness' if ok and reply.data
                           else 'native plugin load acknowledgement unavailable; waiting for its status service')
        if result is not None and result.get('ready') is True:
            self.enable_ideal_operation()
            return
        # Keep the ROS executor responsive while the read-only child retries.
        # Loading is attempted at most once; there is no overall 5 s deadline.
        self._ideal_startup_pending = True
        self._ideal_startup_next_poll = 0.
        self._ideal_startup_next_log = 0.
        self.get_logger().warning(self.reason)

    def enable_ideal_operation(self):
        self._ideal_startup_pending = False
        process = getattr(self, 'status_process', None)
        if process is not None:
            process.close()
            self.status_process = None
        # No periodic status watchdog: the native simulation actuator retains
        # the last real source target without a communication lease.
        self.armed = self.ever_armed = True
        self.reason = 'ideal simulation enabled; automatic base-lock protections disabled'
        self.get_logger().warning(self.reason)
        if 'reference' in self.states:
            self.publish_ideal_reference(self.states['reference'])

    def poll_ideal_startup(self):
        wall = time.monotonic()
        if wall < getattr(self, '_ideal_startup_next_poll', 0.):
            return
        process = getattr(self, 'status_process', None)
        if process is None:
            try:
                self.status_process = process = NativeStatusProcess(
                    self.args.gz_partition, self.status_service)
            except (RuntimeError, OSError) as error:
                self._ideal_startup_next_poll = wall + 1.
                self.reason = 'waiting for native steering service: ' + str(error)
                process = None
        if process is None:
            if wall >= getattr(self, '_ideal_startup_next_log', 0.):
                self._ideal_startup_next_log = wall + 2.
                self.get_logger().warning(self.reason)
            return
        try:
            packet = process.poll()
        except RuntimeError as error:
            process.close()
            self.status_process = None
            self._ideal_startup_next_poll = wall+1.
            self.reason = 'waiting for native steering service: '+str(error)
        else:
            if packet is not None:
                try:
                    result = self.validate_native_status(packet['data'], require_ready=False)
                except NativeStatusPending as error:
                    self.reason = str(error)
                except RuntimeError as error:
                    # Configuration errors are not discovery delays. Display
                    # the real reason and do not add a competing second plugin.
                    self._ideal_startup_pending = False
                    self._ideal_startup_error = str(error)
                    self.reason = str(error)
                    process.close()
                    self.status_process = None
                    self.get_logger().error(self.reason)
                    return
                else:
                    self.last_native = result
                    self.native_receipt_wall = packet['completed_wall']
                    if result.get('ready') is True:
                        self.enable_ideal_operation()
                        return
                    self.reason = ('waiting for native steering readiness: '
                        f'paused={result.get("paused")}; reason={result.get("expiry_reason")}; '
                        'components='+json.dumps(result.get('ready_components', {}), sort_keys=True))
                self._ideal_startup_next_poll = wall+.5
            elif process.ready and process.inflight is None:
                try:
                    process.submit(self.get_clock().now().nanoseconds, wall)
                except (RuntimeError, OSError) as error:
                    process.close()
                    self.status_process = None
                    self._ideal_startup_next_poll = wall + 1.
                    self.reason = 'waiting for native steering service: ' + str(error)
        if wall >= getattr(self, '_ideal_startup_next_log', 0.):
            self._ideal_startup_next_log = wall+2.
            self.get_logger().warning(self.reason)

    def publish_ideal_reference(self, message):
        """Only new source packets change the held native steering target."""
        try:
            values = list(message.data)
            if len(values) != 4 or not all(math.isfinite(v) for v in values):
                raise ValueError('four finite steering targets required')
            stamp = self.reference_ns
            if self.ideal_simulation:
                native = Double_V()
                native.data.extend(values)
            else:
                lower, upper = self._ideal_limits
                angle = float(values[3])
                if not lower <= angle <= upper:
                    raise ValueError('r2 target outside runtime joint limits')
                native=Double();native.data=angle
                native.header.stamp.sec=stamp//10**9
                native.header.stamp.nsec=stamp%10**9
            if not self.gz_publisher.publish(native):
                raise RuntimeError('native Gazebo publication failed')
            self.sent += 1
            self.reason = 'ideal steering holds the last valid source target; automatic locks disabled'
        except Exception as error:
            self.fail(str(error))

    def system_ready(self):
        try:self.native_status();return True
        except RuntimeError:return False

    def reference(self, message):
        self.receive('reference',message)
        self.reference_ns=self.get_clock().now().nanoseconds
        if not self.armed or self.failed:return
        if self.ideal_simulation:
            self.publish_ideal_reference(message)
            return
        try:
            actual,feedback_stamp=self.feedback()
            if not self.owned_source():raise RuntimeError('steering publisher ownership changed')
            if not self.movement_permission():return
            if self.resume_pending and (not self.states['locked'].data or not self.stopped()):
                raise RuntimeError('base moved before continuous steering reclaim')
            command=self.validator.accept(message.data,
                ros_ns=self.reference_ns,wall_now=time.monotonic(),
                actual=actual,feedback_stamp_ns=feedback_stamp)
            if command is None:
                if (self.resume_pending and self.reference_ns==self.validator.last_stamp_ns and
                        list(message.data)[3]==self.validator.last_target):return
                if not self.validator.active:raise RuntimeError(self.validator.reason)
                return
            native=Double();native.data=command['position_rad']
            native.header.stamp.sec=command['stamp_ns']//10**9
            native.header.stamp.nsec=command['stamp_ns']%10**9
            if not self.gz_publisher.publish(native):raise RuntimeError('native Gazebo publication failed')
            self.resume_pending=False
            self.sent+=1
        except Exception as error:self.fail(str(error))

    def fail(self, reason):
        if self.ideal_simulation:
            self.ideal_error_count = getattr(self, 'ideal_error_count', 0)+1
            report = ('ideal simulation: '+reason+
                      '; base lock not requested; last native target retained')
            if report != self.reason:
                self.get_logger().warning(report)
            self.reason = report
            return
        self.failed=True;self.armed=False
        self.reason=(reason+'; wall_feedback_timeout_s='+str(self.wall_feedback_timeout)+
                     '; interlock='+json.dumps(self.movement_state(),sort_keys=True)+
                     '; feedback_ages='+json.dumps(self.feedback_ages(),sort_keys=True))
        process=getattr(self,'status_process',None)
        if process is not None:process.close()
        if self.validator:self.validator.invalidate(reason)
        self.lock_publisher.publish(Bool(data=True))
        self.get_logger().error('ideal r2 compensation stopped; lock requested: '+self.reason)

    def watchdog(self):
        if self.ideal_simulation and getattr(self, '_ideal_startup_pending', False):
            self.poll_ideal_startup()
        if self.armed and not self.ideal_simulation:
            try:
                self.poll_native_status()
                self.feedback()
                if not self.reference_fresh():
                    raise RuntimeError('original steering reference stream expired')
                allowed=self.movement_permission()
                if allowed and self.resume_pending:
                    if not self.states['locked'].data or not self.stopped():
                        raise RuntimeError('base moved before continuous steering reclaim')
                elif allowed and self.sent and not self.validator.current(
                        self.get_clock().now().nanoseconds,time.monotonic()):
                    raise RuntimeError(self.validator.reason)
            except Exception as error:self.fail(str(error))
        wall=time.monotonic()
        if wall>=self.next_status_at:
            self.next_status_at=wall+.5
            if self.armed and not self.ideal_simulation:
                try:
                    process=getattr(self,'status_process',None)
                    if process is None:raise RuntimeError('native status subprocess missing')
                    process.submit(self.get_clock().now().nanoseconds,wall)
                except Exception as error:self.fail(str(error))
            self.status_publisher.publish(String(data=json.dumps({
                'simulation_only':True,'idealized':True,'armed':self.armed,
                'ideal_simulation':self.ideal_simulation,
                'waiting_for_native_startup':getattr(self, '_ideal_startup_pending', False),
                'native_startup_error':getattr(self, '_ideal_startup_error', ''),
                'automatic_base_locks_enabled':not self.ideal_simulation,
                'ideal_packet_errors':getattr(self, 'ideal_error_count', 0),
                'suspended_for_arm_work':self.suspended,
                'awaiting_continuous_reclaim':self.resume_pending,
                'fault':self.failed,'reason':self.reason,'joint_name':'r2_steer_joint',
                'gz_partition':self.args.gz_partition,'ros_domain_id':os.getenv('ROS_DOMAIN_ID'),
                'sent_references':self.sent,'reference_time':'real ROS receipt; source array has no header',
                'wall_feedback_timeout_s':self.wall_feedback_timeout,
                'parking_transition_timeout_s':self.parking_transition_timeout,
                'interlock':self.movement_state(),
                'feedback_ages':self.feedback_ages(),
                'native_receipt_age_wall_s':None if getattr(self,'native_receipt_wall',None) is None
                    else wall-self.native_receipt_wall,
                'native_is_cached_snapshot':True,
                'native_status_subprocess':None if getattr(self,'status_process',None) is None else {
                    'pid':self.status_process.process.pid,'ready':self.status_process.ready,
                    'closed':self.status_process.closed,'inflight':self.status_process.inflight is not None},
                'native':self.last_native},sort_keys=True)))

    def stop(self):
        self.armed=False
        process=getattr(self,'status_process',None)
        if process is not None:process.close()
        if self.ideal_simulation:
            return
        if not self.ever_armed:
            if process is not None:process.close(wait=True)
            return
        for _ in range(3):
            self.lock_publisher.publish(Bool(data=True))
            end=time.monotonic()+.03
            while rclpy.ok() and time.monotonic()<end:
                rclpy.spin_once(self,timeout_sec=.01)
        if process is not None:process.close(wait=True)
        # No renewal or cached-reference republish; native lease expires and
        # the original Gazebo actuator resumes. No other joint is reset.


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gz-partition',required=True)
    parser.add_argument('--world-name',default='field')
    parser.add_argument('--robot-name',default='agri_robot')
    parser.add_argument('--load-system',action='store_true',
                        help='Explicitly allow adding the opt-in Gazebo system')
    parser.add_argument('--plugin-path',default='')
    parser.add_argument('--ideal-simulation',action=argparse.BooleanOptionalAction,
                        default=os.getenv('AGRI_IDEAL_SIMULATION') == '1',
                        help='Gazebo ideal operation: no automatic base lock or command leases')
    parser.add_argument('--wall-feedback-timeout',type=float,default=1.5,
                        help='Python feedback/reference receipt timeout in wall seconds '
                             '(default: 1.5; native steering lease remains unchanged)')
    parser.add_argument('--parking-transition-timeout',type=float,default=2.,
                        help='Bounded wall seconds for lock/stopped acknowledgement after stow '
                             'permission is withdrawn; ideal references are suspended immediately')
    args,ros_args=parser.parse_known_args()
    if not math.isfinite(args.wall_feedback_timeout) or not .5 <= args.wall_feedback_timeout <= 5.:
        parser.error('--wall-feedback-timeout must be finite and between 0.5 and 5 seconds')
    if (not math.isfinite(args.parking_transition_timeout) or
            not .1 <= args.parking_transition_timeout <= 5.):
        parser.error('--parking-transition-timeout must be finite and between 0.1 and 5 seconds')
    for value in (args.gz_partition,args.world_name,args.robot_name):
        if not re.fullmatch(r'[A-Za-z0-9_.-]+',value):parser.error('simple explicit Gazebo names required')
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(_signum,_frame):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt)
    signal.signal(signal.SIGTERM,interrupt)
    node=None
    try:
        node=IdealR2Steering(args);node.start();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:
            if rclpy.ok():node.stop()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
