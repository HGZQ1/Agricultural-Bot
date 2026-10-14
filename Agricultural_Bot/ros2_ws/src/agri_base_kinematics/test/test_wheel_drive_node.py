"""Exercise the node output boundary with deterministic ROS/SIM timestamps."""

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from rclpy.time import Time

from agri_base_kinematics.four_wheel_steering_node import FourWheelSteeringNode
from agri_base_kinematics.kinematics import FourWheelSteeringKinematics
from agri_base_kinematics.wheel_drive import WheelDriveLimiter
from agri_base_kinematics.steering_reference import SteeringReferenceLimiter


class SimClock:
    stamp_ns = 1_000_000_000

    def now(self):
        return Time(nanoseconds=self.stamp_ns)

    def advance(self, seconds):
        self.stamp_ns += round(seconds*1e9)


def node_fixture():
    # The actual _update() method is exercised; no ROS graph or motors are
    # started. Publisher outputs and the clock are the only transport doubles.
    node = FourWheelSteeringNode.__new__(FourWheelSteeringNode)
    clock, outputs, odometry = SimClock(), [], []
    node.get_clock = lambda: clock
    node.get_logger = lambda: SimpleNamespace(warning=lambda message: None)
    node._wheel_names = ['f1', 'f2', 'r1', 'r2']
    node._steering_signs = [-1.]*4
    node._steering_offsets = [0.]*4
    node._wheel_signs = [1.]*4
    node._kinematics = FourWheelSteeringKinematics(
        {'f1': [.23, .17], 'f2': [.23, -.17],
         'r1': [-.23, .17], 'r2': [-.23, -.17]}, .127)
    node._drive_limiter = WheelDriveLimiter(max_acceleration=.8)
    node._steering_limiter = SteeringReferenceLimiter([(-3.14159, 3.14159)]*4)
    node._steering = dict.fromkeys(node._wheel_names, 0.)
    node._wheel_velocity = dict.fromkeys(node._wheel_names, 0.)
    node._twist = (.15, 0., 0.)
    node._command_timeout = .5
    node._joint_state_timeout = .25
    node._alignment_full_speed_error = .05
    node._alignment_stop_error = .35
    node._command_received = node._joint_state_received = True
    node._last_command_time = node._last_joint_state_time = node._last_odom_time = clock.now()
    node._last_clock_ns = clock.stamp_ns
    node._x = node._y = node._yaw = 0.
    node._steering_pub = SimpleNamespace(publish=lambda message: None)
    node._wheel_pub = SimpleNamespace(publish=lambda message: outputs.append(list(message.data)))
    node._publish_odom = lambda stamp, *twist: odometry.append((stamp.nanoseconds, twist))
    return node, clock, outputs, odometry


def drive(node, clock, cycles=20):
    for _ in range(cycles):
        clock.advance(.02)
        node._last_command_time = node._last_joint_state_time = clock.now()
        node._update()


def test_node_limits_final_wheel_output_and_keeps_odometry_measured():
    node, clock, outputs, odometry = node_fixture()
    node._wheel_velocity = dict.fromkeys(node._wheel_names, .4)
    drive(node, clock)
    assert outputs[-1] == pytest.approx([.304]*4)
    assert all(max(abs(b-a) for a, b in zip(first, second)) <= .016+1e-12
               for first, second in zip(outputs, outputs[1:]))
    assert odometry[-1][1] == pytest.approx((.4*.127, 0., 0.))


def test_node_ramps_after_steering_drive_permit_recovers():
    node, clock, outputs, _ = node_fixture()
    node._steering = dict.fromkeys(node._wheel_names, .5)
    drive(node, clock)
    assert outputs[-1] == [0.]*4
    node._steering = dict.fromkeys(node._wheel_names, 0.)
    drive(node, clock, 1)
    assert outputs[-1] == pytest.approx([.016]*4)


def test_node_checks_raw_target_even_while_reference_is_near_feedback():
    node, clock, outputs, _ = node_fixture()
    steering_outputs = []
    node._steering_pub = SimpleNamespace(
        publish=lambda message: steering_outputs.append(list(message.data)))
    node._twist = (0., .15, 0.)
    drive(node, clock, 2)
    assert outputs[-1] == [0.]*4
    assert steering_outputs[0] == [0.]*4  # Fresh measured initialization.
    assert max(abs(value) for value in steering_outputs[1]) == pytest.approx(.0004)
    # Comparing with the filtered heading would falsely allow drive here.
    assert node._alignment_full_speed_error > .0004


def test_node_lock_holds_fresh_measured_steering_and_stops_wheels():
    node, clock, outputs, _ = node_fixture()
    steering_outputs = []
    node._steering_pub = SimpleNamespace(
        publish=lambda message: steering_outputs.append(list(message.data)))
    drive(node, clock)
    node._steering = dict.fromkeys(node._wheel_names, .12)
    node._twist = (0., 0., 0.)
    drive(node, clock, 1)
    assert outputs[-1] == [0.]*4
    assert steering_outputs[-1] == pytest.approx([-.12]*4)
    assert node._steering_limiter.velocity == [0.]*4


def test_node_missing_feedback_does_not_publish_stale_steering():
    node, clock, outputs, _ = node_fixture()
    steering_outputs = []
    node._steering_pub = SimpleNamespace(
        publish=lambda message: steering_outputs.append(list(message.data)))
    drive(node, clock, 2)
    node._joint_state_received = False
    node._update()
    assert len(steering_outputs) == 2
    assert outputs[-1] == [0.]*4
    assert node._steering_limiter.position is None


def test_node_does_not_exceed_reduced_steering_drive_permit():
    node, clock, outputs, _ = node_fixture()
    drive(node, clock, 80)
    assert outputs[-1] == pytest.approx([.15/.127]*4)
    node._steering = dict.fromkeys(node._wheel_names, .30)
    drive(node, clock, 1)
    cap = (.15/.127)*(.35-.30)/(.35-.05)
    assert outputs[-1] == pytest.approx([cap]*4)
    node._steering = dict.fromkeys(node._wheel_names, 0.)
    drive(node, clock, 1)
    assert outputs[-1] == pytest.approx([cap+.016]*4)


@pytest.mark.parametrize('cause', ['locked_zero', 'command_timeout',
                                 'feedback_timeout', 'feedback_missing', 'clock_jump'])
def test_node_stop_guards_bypass_ramp_and_clear_history(cause):
    node, clock, outputs, _ = node_fixture()
    drive(node, clock)
    assert min(outputs[-1]) > .3
    clock.advance(.02)
    if cause == 'locked_zero':
        node._twist = (0., 0., 0.)
    elif cause == 'command_timeout':
        node._last_command_time = Time(nanoseconds=clock.stamp_ns-510_000_000)
    elif cause == 'feedback_timeout':
        node._last_joint_state_time = Time(nanoseconds=clock.stamp_ns-260_000_000)
    elif cause == 'feedback_missing':
        node._joint_state_received = False
    else:
        clock.stamp_ns -= 200_000_000
    node._update()
    assert outputs[-1] == [0.]*4
    node._twist = (.15, 0., 0.)
    node._command_received = node._joint_state_received = True
    drive(node, clock, 1)
    assert outputs[-1] == pytest.approx([.016]*4)
