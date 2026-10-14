"""Offline real worker-to-gate regression; requires the built ROS workspace."""

import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import TwistStamped
from rclpy.time import Time


ROOT = Path(__file__).resolve().parents[1]
for package in ('agri_arm_control', 'agri_base_adapter'):
    sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / package))

from agri_arm_control.harvest_worker import HarvestArmWorker  # noqa: E402
from agri_arm_control.controller import ArmController  # noqa: E402
from agri_arm_control.safety import JointEvidence  # noqa: E402
from agri_base_adapter.harvest_interlock import HarvestInterlock  # noqa: E402
from agri_base_adapter.velocity_gate import VelocityGateCore  # noqa: E402
from agri_base_adapter.velocity_gate_node import VelocityGateNode  # noqa: E402


@pytest.fixture
def pair(monkeypatch):
    clock = NS(ns=100_000_000_000, wall=10.)
    node = object.__new__(VelocityGateNode)
    node._core = VelocityGateCore(('manual', 'navigation'),
                                  {'manual': .35, 'navigation': .35}, (.3, .08, .5))
    node._core.set_locked(True)
    node._harvest_interlock = HarvestInterlock()
    node._stopping = False
    node._interlock_reason = node._last_source = None
    node._frame_id = 'base_footprint'
    node._maximum_input_age, node._maximum_future_skew = .5, .05
    node.get_clock = lambda: NS(now=lambda: Time(nanoseconds=clock.ns))
    node.get_logger = lambda: NS(warning=lambda _message: None)
    outputs = []
    node._output = NS(publish=outputs.append)
    node._locked_state = node._active_source = NS(publish=lambda _message: None)
    monkeypatch.setattr('agri_base_adapter.velocity_gate_node.time.monotonic',
                        lambda: clock.wall)
    states, statuses = [], []

    def publish(message):
        states.append(message)
        node._worker_state_callback(message)

    worker = NS(
        _mutex=threading.RLock(), _last_heartbeat=None,
        _healthy=True, _fault=False, _busy=False, _phase='IDLE',
        _work_context=('', '', ''), _startup_checked=True,
        p={'task_frame': 'base_footprint'},
        arm=NS(stowed=lambda: True, stow_report=lambda: {'passed': True},
               terminal_confirmed=True, execution_status={}),
        _grasp_diagnostics=[], _contact_diagnostics={}, _scene_ready_diagnostics={},
        get_clock=node.get_clock, _pub=NS(publish=publish),
        _status_pub=NS(publish=lambda message: statuses.append(json.loads(message.data))))
    return worker, node, clock, outputs, states


def _start_motion(pair):
    worker, node, clock, outputs, _states = pair
    HarvestArmWorker._heartbeat(worker)
    assert node._core.locked
    assert node._set_locked(False)
    command = TwistStamped()
    command.header.stamp = Time(nanoseconds=clock.ns).to_msg()
    command.header.frame_id = 'base_footprint'
    command.twist.linear.x, command.twist.linear.y = .06, .03225
    node._on_command('navigation', command)
    node._publish_selected()
    assert outputs[-1].twist.linear.x == .06


def test_timer_finally_duplicates_do_not_stop_or_renew_then_paused_clock_locks(pair):
    worker, node, clock, outputs, states = pair
    _start_motion(pair)
    for delta in (.1, .2, .4):
        clock.wall = 10. + delta
        HarvestArmWorker._heartbeat(worker)
        node._publish_selected()
        assert not node._core.locked
    assert len(states) == 1
    assert node._harvest_interlock._received_at == 10.
    clock.wall = 10.76
    HarvestArmWorker._heartbeat(worker)
    node._publish_selected()
    assert node._core.locked
    assert outputs[-1].twist.linear.x == outputs[-1].twist.linear.y == 0.


def test_same_sample_deployment_stops_and_stowed_replay_cannot_resume(pair):
    worker, node, clock, outputs, states = pair
    _start_motion(pair)
    worker._busy, worker._phase = True, 'DEPLOYING'
    clock.wall += .1
    HarvestArmWorker._heartbeat(worker)
    assert len(states) == 2
    assert node._core.locked
    assert outputs[-1].twist.linear.x == outputs[-1].twist.linear.y == 0.
    worker._busy, worker._phase = False, 'STOWED'
    HarvestArmWorker._heartbeat(worker)
    assert not node._set_locked(False)
    clock.ns += 100_000_000
    HarvestArmWorker._heartbeat(worker)
    assert node._core.locked
    assert node._set_locked(False)
    node._publish_selected()
    assert outputs[-1].twist.linear.x == outputs[-1].twist.linear.y == 0.


def test_real_backward_sample_immediately_locks_and_requires_next_sample(pair):
    worker, node, clock, outputs, states = pair
    _start_motion(pair)
    clock.ns = 20_000_000_000
    clock.wall += .1
    HarvestArmWorker._heartbeat(worker)
    assert states[-1].header.stamp.sec == 20
    assert node._core.locked
    assert outputs[-1].twist.linear.x == 0.
    assert not node._set_locked(False)
    clock.ns += 100_000_000
    HarvestArmWorker._heartbeat(worker)
    assert node._core.locked
    assert node._set_locked(False)


def test_real_joint_duplicate_preserves_stow_but_changed_same_sample_locks(pair):
    worker, node, clock, outputs, _states = pair
    evidence = JointEvidence(.5)
    names = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'cr5_joint6']
    arm = NS(evidence=evidence, joints=names, node=worker,
             terminal_confirmed=True, p={'stow_joints': [0.]*6,
                'joint_position_tolerance_rad': .03,
                'joint_velocity_tolerance_rps': .03, 'joint_settle_s': .25})
    worker.arm.stowed = lambda: ArmController.stowed(arm)
    worker.arm.stow_report = lambda: ArmController.stow_report(arm)
    assert evidence.update(names, [0.]*6, [0.]*6, clock.ns, clock.ns, clock.wall)
    assert not arm.evidence.at(names, [0.]*6, clock.ns, clock.wall)
    clock.ns += 300_000_000
    clock.wall += .3
    assert evidence.update(names, [0.]*6, [0.]*6, clock.ns, clock.ns, clock.wall)
    _start_motion(pair)
    old_receipt = evidence.wall
    clock.wall += .1
    assert not evidence.update(names, [0.]*6, [0.]*6, clock.ns, clock.ns, clock.wall)
    assert evidence.wall == old_receipt
    clock.ns += 1_000_000
    HarvestArmWorker._heartbeat(worker)
    assert worker.arm.stowed() and not node._core.locked
    # Even a tiny changed velocity at the duplicate timestamp remains an
    # ambiguous sample. No tolerance or TTL is added by the duplicate fix.
    changed = [0.]*6
    changed[1] = .000001
    assert not evidence.update(names, [0.]*6, changed, evidence.stamp, clock.ns, clock.wall)
    HarvestArmWorker._heartbeat(worker)
    assert node._core.locked
    assert outputs[-1].twist.linear.x == outputs[-1].twist.linear.y == 0.
