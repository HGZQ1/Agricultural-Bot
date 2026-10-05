import math

from agri_base_kinematics.kinematics import (
    apply_joint_signs,
    FourWheelSteeringKinematics,
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
