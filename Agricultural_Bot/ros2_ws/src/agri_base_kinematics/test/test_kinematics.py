import math

from agri_base_kinematics.kinematics import (
    apply_joint_signs,
    FourWheelSteeringKinematics,
    integrate_planar_pose,
    physical_steering_to_joint,
    steering_alignment_scale,
    steering_feedback_to_physical,
)
import pytest


POSITIONS = {
    'f1': (0.5, 0.4),
    'f2': (0.5, -0.4),
    'r1': (-0.5, 0.4),
    'r2': (-0.5, -0.4),
}


@pytest.fixture
def model():
    return FourWheelSteeringKinematics(POSITIONS, 0.1, max_wheel_speed=100.0)


def test_straight_forward(model):
    commands = model.inverse(1.0, 0.0, 0.0)
    assert all(abs(command.steering) < 1e-12 for command in commands.values())
    assert all(abs(command.velocity - 10.0) < 1e-12 for command in commands.values())


def test_in_place_rotation(model):
    commands = model.inverse(0.0, 0.0, 1.0)
    assert commands['f1'].steering == pytest.approx(math.atan2(0.5, -0.4))
    assert commands['f2'].steering == pytest.approx(math.atan2(0.5, 0.4))
    assert commands['r1'].steering == pytest.approx(math.atan2(-0.5, -0.4))
    assert commands['r2'].steering == pytest.approx(math.atan2(-0.5, 0.4))


def test_zero_velocity_holds_angle(model):
    commands = model.inverse(
        0.0, 0.0, 0.0,
        {'f1': 0.3, 'f2': -0.4, 'r1': 0.2, 'r2': -0.1})
    assert commands['f1'].steering == pytest.approx(0.3)
    assert all(command.velocity == 0.0 for command in commands.values())


def test_shortest_angle_flip(model):
    commands = model.inverse(1.0, 0.0, 0.0, {'f1': math.pi - 0.1})
    assert abs(abs(commands['f1'].steering) - math.pi) < 1e-12
    assert commands['f1'].velocity == pytest.approx(-10.0)


def test_odometry_inverse(model):
    steering = {name: 0.0 for name in POSITIONS}
    velocity = {name: 10.0 for name in POSITIONS}
    vx, vy, wz = model.body_velocity(steering, velocity)
    assert (vx, vy, wz) == pytest.approx((1.0, 0.0, 0.0))


def test_joint_sign_conversion_is_consistent():
    physical = [0.2, -0.3, 1.0, -2.0]
    signs = [-1.0, -1.0, 1.0, 1.0]
    joint_commands = apply_joint_signs(physical, signs)
    feedback = apply_joint_signs(joint_commands, signs)
    assert feedback == pytest.approx(physical)


def test_steering_offsets_round_trip_between_joint_and_physical_frames():
    physical = [0.0, 0.4, -0.8, math.pi - 0.1]
    signs = [-1.0, -1.0, -1.0, -1.0]
    offsets = [0.0056, -0.0056, 0.0056, -0.0056]
    joint = physical_steering_to_joint(physical, signs, offsets)
    recovered = steering_feedback_to_physical(joint, signs, offsets)
    assert recovered == pytest.approx(physical)


def test_speed_limit_preserves_wheel_ratios():
    unlimited = FourWheelSteeringKinematics(
        POSITIONS, 0.1, max_wheel_speed=100.0)
    limited = FourWheelSteeringKinematics(
        POSITIONS, 0.1, max_wheel_speed=4.0)
    expected = unlimited.inverse(0.5, 0.2, 1.0)
    actual = limited.inverse(0.5, 0.2, 1.0)
    scale = 4.0 / max(abs(command.velocity) for command in expected.values())

    for name in POSITIONS:
        assert actual[name].steering == pytest.approx(expected[name].steering)
        assert actual[name].velocity == pytest.approx(
            expected[name].velocity * scale)


def test_alignment_scale_stops_then_ramps_all_wheels_together():
    assert steering_alignment_scale([0.0, 0.02], 0.05, 0.35) == 1.0
    assert steering_alignment_scale([0.01, 0.35], 0.05, 0.35) == 0.0
    assert steering_alignment_scale([0.20], 0.05, 0.35) == pytest.approx(0.5)
    assert steering_alignment_scale([2 * math.pi - 0.02], 0.05, 0.35) == 1.0


def test_alignment_thresholds_are_validated():
    with pytest.raises(ValueError):
        steering_alignment_scale([], 0.1, 0.1)
    with pytest.raises(ValueError):
        steering_alignment_scale([math.nan], 0.05, 0.35)


def test_midpoint_pose_integration_reduces_arc_bias():
    x, y, yaw = integrate_planar_pose(0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.1)
    assert x == pytest.approx(math.cos(0.05) * 0.1)
    assert y == pytest.approx(math.sin(0.05) * 0.1)
    assert yaw == pytest.approx(0.1)
