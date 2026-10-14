"""Exercise real bridge callbacks with isolated messages and no ROS/Gazebo node.

Transport constructors are replaced by failing sentinels; only the validator
and production callback methods execute. The test never creates DDS traffic.
"""
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest


SCRIPTS = Path(__file__).parents[1] / 'scripts'
BASE_NS = 829_447_000_000
ACTUAL = -.824242667524138


class Message:
    def __init__(self, data=None):
        self.data = data
        self.header = NS(stamp=NS(sec=0, nsec=0))


class NoTransport:
    def __init__(self, *_a, **_kw):
        pytest.fail('offline bridge test must not construct ROS or Gazebo transport')


class Publisher:
    def __init__(self):
        self.messages = []
        self.success = True

    def publish(self, message):
        self.messages.append(message)
        return self.success


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def bridge(monkeypatch):
    def install(name, **attributes):
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
        return module

    install('ament_index_python.packages', get_package_prefix=lambda _n: '/unused')
    install('agri_harvest_interfaces.msg', HarvestWorkerState=Message)
    install('controller_manager_msgs.srv', ListHardwareComponents=NS(Request=lambda: NS()))
    install('gz.msgs10.boolean_pb2', Boolean=Message)
    install('gz.msgs10.double_pb2', Double=Message)
    install('gz.msgs10.entity_pb2', Entity=NS(WORLD=9))
    install('gz.msgs10.entity_plugin_v_pb2', EntityPlugin_V=NoTransport)
    install('gz.msgs10.stringmsg_pb2', StringMsg=Message)
    install('gz.transport13', Node=NoTransport, NodeOptions=NoTransport)
    install('nav_msgs.msg', Odometry=Message)
    install('rcl_interfaces.srv', GetParameters=NS(Request=lambda **kw: NS(**kw)))
    install('rclpy', ok=lambda: True, spin_once=lambda *_a, **_kw: None)
    install('rclpy.clock', Clock=NoTransport, ClockType=NS(STEADY_TIME=1))
    install('rclpy.node', Node=type('OfflineNode', (), {}))
    install('rclpy.qos', DurabilityPolicy=NS(TRANSIENT_LOCAL=1),
            QoSProfile=NoTransport, ReliabilityPolicy=NS(RELIABLE=1))
    install('rclpy.signals', SignalHandlerOptions=NS(NO=0))
    install('sensor_msgs.msg', JointState=Message)
    install('std_msgs.msg', Bool=Message, Float64MultiArray=Message, String=Message)
    helper = load(SCRIPTS / 'ideal_steering_command.py', 'isolated_ideal_command')
    monkeypatch.setitem(sys.modules, 'ideal_steering_command', helper)
    status_helper = load(SCRIPTS / 'ideal_status_process.py', 'isolated_status_process')
    monkeypatch.setitem(sys.modules, 'ideal_status_process', status_helper)
    module = load(SCRIPTS / 'ideal_r2_steering.py', 'isolated_ideal_bridge')
    node = object.__new__(module.IdealR2Steering)
    node.clock = NS(ros=BASE_NS, wall=10.)
    monkeypatch.setattr(module.time, 'monotonic', lambda: node.clock.wall)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=node.clock.ros))
    node.logs = []
    node.get_logger = lambda: NS(error=node.logs.append, warning=node.logs.append)
    node.states, node.receipts = {}, {}
    node.validator = helper.IdealSteeringCommand(-3.1416, 3.1416)
    node.armed = node.ever_armed = True
    node.suspended = node.resume_pending = node.failed = False
    node.requires_worker = True
    node.reason = 'test armed'
    node.sent, node.reference_ns, node.next_status_at = 0, 0, 100.
    node.last_native = None
    node.status_process = None
    node.gz_publisher = Publisher()
    node.lock_publisher = Publisher()
    node.status_publisher = Publisher()
    node.get_publishers_info_by_topic = lambda _t: [
        NS(node_name='four_wheel_steering_node', node_namespace='/')]
    node.args = NS(robot_name='agri_robot', world_name='field', gz_partition='offline')
    node.command_topic = '/world/field/ideal_steering/r2_command'
    node.status_service = '/world/field/ideal_steering/status'

    def refresh():
        header = lambda: NS(stamp=NS(sec=node.clock.ros // 10**9,
                                   nanosec=node.clock.ros % 10**9))
        node.states['locked'] = Message(True)
        node.states['odom'] = NS(header=header(), twist=NS(twist=NS(
            linear=NS(x=0., y=0.), angular=NS(z=0.))))
        # Place r2 at index 1 deliberately: feedback lookup must use its name.
        node.states['joints'] = NS(header=header(),
            name=['arm_joint1', 'r2_steer_joint', 'f1_steer_joint'],
            position=[0., ACTUAL, .11], velocity=[0., 0., 0.])
        node.states['worker'] = NS(header=header(), arm_stowed=True, phase='STOWED')
        for key in node.states:
            node.receipts[key] = node.clock.wall

    node.refresh = refresh
    refresh()
    return module, node


def reference(node, target=ACTUAL):
    node.reference(Message([.11, -.22, .33, target]))


def advance(node, seconds=.02):
    node.clock.ros += round(seconds * 1e9)
    node.clock.wall += seconds
    node.refresh()


def suspend(node):
    reference(node)
    advance(node)
    node.states['worker'].phase = 'WORKING'
    node.states['worker'].arm_stowed = False
    reference(node)


def test_real_callback_uses_named_actual_feedback_and_exact_receipt_stamp(bridge):
    _module, node = bridge
    reference(node)
    assert node.sent == 1 and node.armed and not node.failed
    packet = node.gz_publisher.messages[0]
    assert packet.data == ACTUAL
    assert packet.header.stamp.sec * 10**9 + packet.header.stamp.nsec == BASE_NS


def test_duplicate_callback_does_not_republish_or_refresh_validator_lease(bridge):
    _module, node = bridge
    reference(node)
    node.clock.wall += .1
    reference(node)
    assert node.sent == 1
    assert node.validator.last_wall == 10.
    assert not node.failed


def test_parked_arm_work_suspends_without_fault_or_cached_republication(bridge):
    _module, node = bridge
    suspend(node)
    assert node.armed and node.suspended and node.resume_pending
    assert not node.failed and not node.validator.active
    assert node.sent == 1 and not node.lock_publisher.messages
    node.watchdog()
    assert node.sent == 1 and node.armed


def test_stow_restored_watchdog_waits_for_new_matching_reference(bridge):
    _module, node = bridge
    suspend(node)
    advance(node)
    node.watchdog()
    assert node.armed and node.resume_pending and not node.suspended
    assert not node.validator.active and node.sent == 1
    reference(node)
    assert node.armed and not node.resume_pending and node.validator.active
    assert node.sent == 2


@pytest.mark.parametrize('lost_condition', ['locked', 'stopped'])
def test_resume_callback_cannot_clear_pending_after_unlock_or_motion(bridge, lost_condition):
    _module, node = bridge
    suspend(node)
    advance(node)
    if lost_condition == 'locked':
        node.states['locked'].data = False
    else:
        node.states['odom'].twist.twist.linear.x = .02
    # Callback arrives before the 20 ms watchdog, the important DDS ordering.
    reference(node)
    assert node.failed and not node.armed
    assert node.sent == 1
    assert node.lock_publisher.messages[-1].data is True


def test_reclaim_discontinuity_faults_and_requests_lock(bridge):
    _module, node = bridge
    suspend(node)
    advance(node)
    reference(node, ACTUAL + .1)
    assert node.failed and not node.armed
    assert node.sent == 1 and node.lock_publisher.messages[-1].data is True


@pytest.mark.parametrize('lost_condition', ['locked', 'stopped'])
def test_worker_not_stowed_during_motion_faults_instead_of_suspending(bridge, lost_condition):
    _module, node = bridge
    reference(node)
    advance(node)
    node.states['worker'].phase = 'WORKING'
    node.states['worker'].arm_stowed = False
    if lost_condition == 'locked':
        node.states['locked'].data = False
    else:
        node.states['odom'].twist.twist.linear.y = .02
    reference(node)
    assert node.failed and not node.armed
    assert node.sent == 1 and node.lock_publisher.messages[-1].data is True


@pytest.mark.parametrize('component', ['x', 'y', 'z'])
def test_nonfinite_odometry_is_never_treated_as_parked(bridge, component):
    _module, node = bridge
    twist = node.states['odom'].twist.twist
    setattr(twist.angular if component == 'z' else twist.linear, component, float('nan'))
    assert not node.stopped()
    node.states['worker'].phase = 'WORKING'
    node.states['worker'].arm_stowed = False
    reference(node)
    assert node.failed and not node.gz_publisher.messages


def test_missing_reference_stream_faults_even_with_fresh_feedback(bridge):
    _module, node = bridge
    reference(node)
    advance(node, .201)
    node.watchdog()
    assert node.failed and not node.armed
    assert node.sent == 1 and node.lock_publisher.messages[-1].data is True


def test_expired_worker_receipt_faults_without_native_publication(bridge):
    _module, node = bridge
    node.receipts['worker'] = 9.49
    reference(node)
    assert node.failed and not node.gz_publisher.messages


def test_source_ownership_change_faults_before_native_publication(bridge):
    _module, node = bridge
    node.get_publishers_info_by_topic = lambda _t: [NS(node_name='other_node')]
    reference(node)
    assert node.failed and not node.gz_publisher.messages


def test_native_publish_failure_faults_and_does_not_clear_reclaim(bridge):
    _module, node = bridge
    node.resume_pending = True
    node.gz_publisher.success = False
    reference(node)
    assert node.failed and node.resume_pending
    assert node.sent == 0 and node.lock_publisher.messages[-1].data is True


def test_status_unavailable_is_distinct_from_identity_mismatch(bridge):
    module, node = bridge
    node.gz = NS(request=lambda *_a: (False, Message('')))
    with pytest.raises(module.NativeStatusUnavailable):
        node.native_status()
    node.gz = NS(request=lambda *_a: (True, Message(json.dumps({
        'idealized': True, 'system': 'unrelated-system'}))))
    with pytest.raises(RuntimeError, match='identity/readiness') as error:
        node.native_status()
    assert not isinstance(error.value, module.NativeStatusUnavailable)


def test_stop_locks_three_times_without_publishing_cached_target(bridge, monkeypatch):
    module, node = bridge
    reference(node)
    monkeypatch.setattr(module.rclpy, 'spin_once',
                        lambda *_a, **_kw: setattr(node.clock, 'wall', node.clock.wall + .01))
    node.stop()
    assert not node.armed
    assert [m.data for m in node.lock_publisher.messages] == [True, True, True]
    assert node.sent == 1


def test_failed_start_does_not_lock_an_unowned_simulation(bridge):
    _module, node = bridge
    node.armed = node.ever_armed = False
    node.stop()
    assert not node.lock_publisher.messages and not node.gz_publisher.messages


@pytest.mark.parametrize('field',['position','velocity'])
def test_nonfinite_r2_readback_stops_before_native_publication(bridge,field):
    _module,node=bridge
    getattr(node.states['joints'],field)[1]=float('nan')
    reference(node)
    assert node.failed and not node.gz_publisher.messages
    assert node.lock_publisher.messages[-1].data is True


class AsyncStatusFixture:
    def __init__(self,packet=None,error=None):
        self.packet,self.error = packet,error
        self.process=NS(pid=12345)
        self.ready,self.closed,self.inflight=True,False,None
        self.submissions=[];self.close_modes=[]

    def poll(self):
        if self.error:
            error,self.error=self.error,None
            raise error
        packet,self.packet=self.packet,None
        return packet

    def submit(self,sim_ns,wall=None):
        if self.inflight is not None:return False
        self.submissions.append((sim_ns,wall));self.inflight={'seq':1}
        return True

    def close(self,*,wait=False):
        self.closed=True;self.close_modes.append(wait)


def status_packet(node,*,age_sim_s=0.):
    return {'submitted_sim_ns':node.clock.ros,'completed_wall':node.clock.wall-.01,
        'data':{'idealized':True,'system':'agri::IdealSteeringSystem',
            'version':'r2_ideal_steering_v1','enabled':True,'ready':True,
            'joint_name':'r2_steer_joint','robot_name':'agri_robot',
            'command_topic':node.command_topic,'sim_ns':node.clock.ros-round(age_sim_s*1e9)}}


def test_periodic_watchdog_never_calls_synchronous_native_rpc(bridge):
    _module,node=bridge;reference(node)
    node.native_status=lambda:pytest.fail('armed watchdog must not call blocking GZ request')
    node.status_process=AsyncStatusFixture()
    node.next_status_at=0.
    node.watchdog();advance(node);node.watchdog()
    assert node.armed and not node.failed
    assert len(node.status_process.submissions)==1 and node.sent==1


@pytest.mark.parametrize('fault',['rpc','stale','identity'])
def test_async_fault_locks_and_late_success_cannot_rearm_or_publish_cached_reference(bridge,fault):
    _module,node=bridge;reference(node)
    packet=status_packet(node,age_sim_s=.251 if fault=='stale' else 0.)
    if fault=='identity':packet['data']['system']='foreign'
    node.status_process=AsyncStatusFixture(packet,
        RuntimeError('actual 250ms native timeout') if fault=='rpc' else None)
    node.watchdog()
    assert node.failed and not node.armed and node.status_process.closed
    assert node.sent==1 and node.lock_publisher.messages[-1].data is True
    node.status_process.packet=status_packet(node)
    advance(node);node.watchdog();reference(node)
    assert node.failed and not node.armed and node.sent==1
    assert 'feedback_ages=' in node.reason


def test_real_feedback_expiry_still_fails_with_nonblocking_pending_status(bridge):
    _module,node=bridge;reference(node)
    node.status_process=AsyncStatusFixture()
    node.receipts['odom']=node.clock.wall-.501
    node.watchdog()
    assert node.failed and 'fresh odom feedback missing' in node.reason
    assert node.lock_publisher.messages[-1].data is True


def test_status_result_retains_child_completion_age_instead_of_renewing_cached_native_age(bridge):
    _module,node=bridge
    packet=status_packet(node);packet['completed_wall']=node.clock.wall-.2
    node.status_process=AsyncStatusFixture(packet)
    node.poll_native_status()
    assert node.native_receipt_wall==node.clock.wall-.2


def test_stop_publishes_lock_before_any_blocking_owned_child_join(bridge,monkeypatch):
    module,node=bridge;reference(node)
    node.status_process=AsyncStatusFixture()
    original=node.status_process.close
    def close(*,wait=False):
        if wait:assert len(node.lock_publisher.messages)==3
        original(wait=wait)
    node.status_process.close=close
    monkeypatch.setattr(module.rclpy,'spin_once',
        lambda *_a,**_kw:setattr(node.clock,'wall',node.clock.wall+.01))
    node.stop()
    assert node.status_process.close_modes==[False,True]


def test_unarmed_discovery_retry_is_bounded_by_startup_caller_and_never_locks(bridge):
    module,node=bridge;node.armed=node.ever_armed=False
    node.status_process=AsyncStatusFixture(error=module.NativeStatusRPCUnavailable('discovering'))
    assert not node.warm_native_status()
    assert not node.failed and not node.lock_publisher.messages
    assert not node.warm_native_status() and len(node.status_process.submissions)==1
    node.status_process.packet=status_packet(node)
    assert node.warm_native_status() and not node.armed


def test_startup_wait_accepts_only_new_fresh_stow_after_pending_settlement(bridge):
    _module,node=bridge;node.armed=node.ever_armed=False
    reference(node)
    node.states['worker'].arm_stowed=False
    original=node.validator.active
    checks=[]
    def spin(predicate,timeout):
        assert timeout==5.
        checks.append(predicate())
        advance(node,.1)
        reference(node)
        node.states['worker'].arm_stowed=False
        checks.append(predicate())
        advance(node,.2)  # genuinely new measured worker state, not a cached rewrite
        reference(node)
        checks.append(predicate())
        assert checks[-1]
    node.spin_until=spin
    node.wait_for_parked_stow_startup()
    assert checks==[False,False,True]
    assert not node.armed and not node.suspended and not node.resume_pending
    assert node.validator.active==original
    assert not node.gz_publisher.messages and not node.lock_publisher.messages


@pytest.mark.parametrize('condition',['FAULT','nonstowed','stale','unlocked','moving'])
def test_startup_wait_timeout_keeps_real_fault_nonstow_or_missing_permit_rejected(bridge,condition):
    _module,node=bridge;node.armed=node.ever_armed=False;reference(node)
    if condition=='FAULT':
        node.states['worker'].phase='FAULT';node.states['worker'].arm_stowed=False
    elif condition=='nonstowed':node.states['worker'].arm_stowed=False
    elif condition=='stale':node.receipts['worker']=node.clock.wall-.501
    elif condition=='unlocked':node.states['locked'].data=False
    else:node.states['odom'].twist.twist.linear.x=.02
    def spin(predicate,timeout):
        assert timeout==5. and not predicate()
        raise RuntimeError('bounded startup deadline')
    node.spin_until=spin
    with pytest.raises(RuntimeError,match='worker_phase=.*arm_stowed=.*feedback_ages='):
        node.wait_for_parked_stow_startup()
    assert not node.armed and not node.failed
    assert not node.gz_publisher.messages and not node.lock_publisher.messages
