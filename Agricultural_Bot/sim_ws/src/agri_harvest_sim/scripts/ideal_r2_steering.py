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
        self.command_topic = f'/world/{args.world_name}/ideal_steering/r2_command'
        self.status_service = f'/world/{args.world_name}/ideal_steering/status'
        self.gz_publisher = self.gz.advertise(self.command_topic, Double)
        self.states, self.receipts = {}, {}
        self.validator = None
        self.armed = False
        self.ever_armed = False
        self.suspended = False
        self.resume_pending = False
        self.failed = False
        self.reason = 'checking parked simulation and command ownership'
        self.requires_worker = True
        self.sent = 0
        self.last_native = None
        self.status_process = None
        self.native_receipt_wall = None
        self.reference_ns = 0
        self.next_status_at = 0.
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
        speeds = (t.linear.x, t.linear.y, t.angular.z)
        return all(math.isfinite(v) and abs(v) < .01 for v in speeds)

    def reference_fresh(self):
        age = (self.get_clock().now().nanoseconds-self.reference_ns)/1e9
        return (self.reference_ns > 0 and -.05 <= age <= .2 and
                time.monotonic()-self.receipts.get('reference',0) <= self.wall_feedback_timeout)

    def movement_permission(self):
        if not self.requires_worker:
            return True
        w = self.states['worker']
        if w.arm_stowed and w.phase in {'IDLE','STOWED'}:
            if self.suspended:
                self.reason = 'measured stow restored; fresh matching reference required'
            self.suspended = False
            return True
        if not self.states['locked'].data or not self.stopped():
            raise RuntimeError('measured stow unavailable while base is not parked and locked')
        # Parked arm work yields control to the normal actuator. Resume requires
        # a fresh target matching actual position, rather than replaying a cache.
        self.suspended = True
        self.resume_pending = True
        self.reason = 'parked arm work: ideal steering suspended until measured stow'
        self.validator.invalidate(self.reason)
        return False

    def owned_source(self):
        sources = self.get_publishers_info_by_topic('/steering_controller/commands')
        return len(sources)==1 and sources[0].node_name=='four_wheel_steering_node'

    def validate_native_status(self, result):
        if (result.get('idealized') is not True or
                result.get('system') != 'agri::IdealSteeringSystem' or
                result.get('version') != 'r2_ideal_steering_v1' or
                result.get('enabled') is not True or result.get('ready') is not True or
                result.get('joint_name') != 'r2_steer_joint' or
                result.get('robot_name') != self.args.robot_name or
                result.get('command_topic') != self.command_topic):
            raise RuntimeError('Gazebo ideal steering identity/readiness mismatch')
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
                             '<joint_name>r2_steer_joint</joint_name>')
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

    def system_ready(self):
        try:self.native_status();return True
        except RuntimeError:return False

    def reference(self, message):
        self.receive('reference',message)
        self.reference_ns=self.get_clock().now().nanoseconds
        if not self.armed or self.failed:return
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
        self.failed=True;self.armed=False
        self.reason=(reason+'; wall_feedback_timeout_s='+str(self.wall_feedback_timeout)+
                     '; feedback_ages='+json.dumps(self.feedback_ages(),sort_keys=True))
        process=getattr(self,'status_process',None)
        if process is not None:process.close()
        if self.validator:self.validator.invalidate(reason)
        self.lock_publisher.publish(Bool(data=True))
        self.get_logger().error('ideal r2 compensation stopped; lock requested: '+self.reason)

    def watchdog(self):
        if self.armed:
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
            if self.armed:
                try:
                    process=getattr(self,'status_process',None)
                    if process is None:raise RuntimeError('native status subprocess missing')
                    process.submit(self.get_clock().now().nanoseconds,wall)
                except Exception as error:self.fail(str(error))
            self.status_publisher.publish(String(data=json.dumps({
                'simulation_only':True,'idealized':True,'armed':self.armed,
                'suspended_for_arm_work':self.suspended,
                'awaiting_continuous_reclaim':self.resume_pending,
                'fault':self.failed,'reason':self.reason,'joint_name':'r2_steer_joint',
                'gz_partition':self.args.gz_partition,'ros_domain_id':os.getenv('ROS_DOMAIN_ID'),
                'sent_references':self.sent,'reference_time':'real ROS receipt; source array has no header',
                'wall_feedback_timeout_s':self.wall_feedback_timeout,
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
    parser.add_argument('--wall-feedback-timeout',type=float,default=1.5,
                        help='Python feedback/reference receipt timeout in wall seconds '
                             '(default: 1.5; native steering lease remains unchanged)')
    args,ros_args=parser.parse_known_args()
    if not math.isfinite(args.wall_feedback_timeout) or not .5 <= args.wall_feedback_timeout <= 5.:
        parser.error('--wall-feedback-timeout must be finite and between 0.5 and 5 seconds')
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
