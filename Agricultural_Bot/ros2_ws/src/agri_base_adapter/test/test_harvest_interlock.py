"""Safety contracts for an independently enforced stowed-arm motion lease."""

from types import SimpleNamespace

from agri_base_adapter.harvest_interlock import HarvestInterlock
from agri_base_adapter.velocity_gate import VelocityGateCore, ZERO_TWIST
from agri_base_adapter.velocity_gate_node import VelocityGateNode
from geometry_msgs.msg import TwistStamped
import pytest
from rclpy.time import Time


def _heartbeat(interlock, *, stamp=100.0, ros_now=100.0, wall_now=10.0,
               stowed=True, phase='IDLE'):
    return interlock.observe(stamp=stamp, ros_now=ros_now, wall_now=wall_now,
                             arm_stowed=stowed, phase=phase)


def test_no_feedback_is_not_unlock_permission():
    interlock = HarvestInterlock()
    assert not interlock.permits_unlock(ros_now=100.0, wall_now=10.0)


@pytest.mark.parametrize('stamp', [0.0, 99.4, 100.06, float('nan')])
def test_zero_old_future_or_invalid_stamp_revokes_permission(stamp):
    interlock = HarvestInterlock()
    assert _heartbeat(interlock)
    assert not _heartbeat(interlock, stamp=stamp, wall_now=10.1)
    assert not interlock.permits_unlock(ros_now=100.0, wall_now=10.1)


@pytest.mark.parametrize('stowed,phase', [
    (False, 'STOWED'), (True, 'DEPLOYING'), (True, 'SCANNING'),
    (True, 'RETRACTING'), (True, 'UNKNOWN'),
])
def test_only_explicit_stowed_idle_phases_permit_unlock(stowed, phase):
    interlock = HarvestInterlock()
    assert _heartbeat(interlock)
    assert not _heartbeat(interlock, stamp=100.1, ros_now=100.1,
                          wall_now=10.1, stowed=stowed, phase=phase)


def test_old_stowed_message_cannot_override_newer_deployed_feedback():
    interlock = HarvestInterlock()
    assert _heartbeat(interlock)
    assert not _heartbeat(interlock, stamp=100.2, ros_now=100.2,
                          wall_now=10.2, stowed=False, phase='WORKING')
    assert not _heartbeat(interlock, stamp=100.1, ros_now=100.2, wall_now=10.3)
    assert not interlock.permits_unlock(ros_now=100.2, wall_now=10.3)


def test_duplicate_heartbeats_cannot_extend_stowed_lease():
    interlock = HarvestInterlock()
    assert _heartbeat(interlock)
    assert not _heartbeat(interlock, wall_now=10.4)
    assert not interlock.permits_unlock(ros_now=100.0, wall_now=10.4)


def test_receipt_watchdog_expires_while_ros_clock_is_paused():
    interlock = HarvestInterlock()
    assert _heartbeat(interlock)
    assert interlock.permits_unlock(ros_now=100.0, wall_now=10.5)
    assert not interlock.permits_unlock(ros_now=100.0, wall_now=10.76)


def test_ros_clock_jump_requires_a_new_heartbeat():
    interlock = HarvestInterlock()
    assert _heartbeat(interlock)
    assert not interlock.permits_unlock(ros_now=20.0, wall_now=10.1)
    assert _heartbeat(interlock, stamp=20.1, ros_now=20.1, wall_now=10.2)


@pytest.mark.parametrize('kwargs', [
    {'receipt_timeout': 0.0}, {'maximum_age': float('inf')}, {'future_skew': -0.1},
])
def test_bad_lease_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        HarvestInterlock(**kwargs)


def _node(monkeypatch, *, enabled=True):
    """Exercise real node methods without opening a DDS graph."""
    node = object.__new__(VelocityGateNode)
    node._core = VelocityGateCore(('manual', 'navigation'),
                                  {'manual': 0.35, 'navigation': 0.35},
                                  (0.3, 0.0, 0.5))
    node._harvest_interlock = HarvestInterlock() if enabled else None
    if enabled:
        node._core.set_locked(True)
    node._stopping = False
    node._interlock_reason = None
    node._last_source = None
    node._frame_id = 'base_footprint'
    node._maximum_input_age = 0.5
    node._maximum_future_skew = 0.05
    clock = SimpleNamespace(ros=100.0, wall=10.0)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=clock.ros))
    node.get_logger = lambda: SimpleNamespace(warning=lambda message: None)
    node.outputs = []
    node.locks = []
    node._output = SimpleNamespace(publish=node.outputs.append)
    node._locked_state = SimpleNamespace(publish=node.locks.append)
    node._active_source = SimpleNamespace(publish=lambda message: None)
    monkeypatch.setattr('agri_base_adapter.velocity_gate_node.time.monotonic',
                        lambda: clock.wall)
    return node, clock


def _worker(node, clock, *, stowed=True, phase='IDLE'):
    node._worker_state_callback(SimpleNamespace(
        header=SimpleNamespace(stamp=Time(seconds=clock.ros).to_msg()),
        arm_stowed=stowed, phase=phase))


def _command(clock):
    command = TwistStamped()
    command.header.stamp = Time(seconds=clock.ros).to_msg()
    command.header.frame_id = 'base_footprint'
    command.twist.linear.x = 0.2
    return command


def test_direct_service_and_topic_unlock_are_rejected_without_worker(monkeypatch):
    node, clock = _node(monkeypatch)
    response = node._set_lock_service(SimpleNamespace(data=False), SimpleNamespace())
    assert not response.success
    assert node._core.locked
    assert not node._set_locked(False)  # Same method used by lock_cmd topic.
    node._on_command('manual', _command(clock))
    assert node._core.select(clock.ros)[0] == ZERO_TWIST
    assert node._set_lock_service(SimpleNamespace(data=True), SimpleNamespace()).success


def test_deployment_immediately_stops_both_sources_and_no_auto_resume(monkeypatch):
    node, clock = _node(monkeypatch)
    _worker(node, clock)
    assert node._core.locked  # Receiving STOWED must never unlock automatically.
    assert node._set_locked(False)
    node._on_command('manual', _command(clock))
    node._on_command('navigation', _command(clock))
    node._publish_selected()
    assert node.outputs[-1].twist.linear.x == 0.2
    clock.ros += 0.1
    clock.wall += 0.1
    _worker(node, clock, stowed=False, phase='DEPLOYING')
    assert node._core.locked
    assert node.outputs[-1].twist.linear.x == 0.0
    assert not node._set_locked(False)
    clock.ros += 0.1
    clock.wall += 0.1
    _worker(node, clock, phase='STOWED')
    assert node._core.locked
    assert node._set_locked(False)
    node._publish_selected()
    assert node.outputs[-1].twist.linear.x == 0.0  # Previous commands were cleared.


def test_periodic_watchdog_relocks_during_clock_pause(monkeypatch):
    node, clock = _node(monkeypatch)
    _worker(node, clock)
    assert node._set_locked(False)
    node._on_command('manual', _command(clock))
    clock.wall += 0.8
    node._publish_selected()
    assert node._core.locked
    assert node.outputs[-1].twist.linear.x == 0.0
    assert not node._set_locked(False)


def test_shutdown_rejects_unlock_and_drops_commands_even_with_stowed_worker(monkeypatch):
    node, clock = _node(monkeypatch)
    _worker(node, clock)
    assert node._set_locked(False)
    node._on_command('manual', _command(clock))
    node.stop()
    assert node._core.locked
    assert all(output.twist.linear.x == 0.0 for output in node.outputs[-3:])
    assert not node._set_lock_service(SimpleNamespace(data=False), SimpleNamespace()).success
    node._on_command('manual', _command(clock))
    node._publish_selected()
    assert node.outputs[-1].twist.linear.x == 0.0


def test_disabled_mode_keeps_existing_manual_navigation_behavior(monkeypatch):
    node, clock = _node(monkeypatch, enabled=False)
    assert node._set_locked(False)
    node._on_command('manual', _command(clock))
    node._publish_selected()
    assert node.outputs[-1].twist.linear.x == 0.2
