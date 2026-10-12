import math
from pathlib import Path
import xml.etree.ElementTree as ET

from agri_base_kinematics.kinematics import (
    apply_joint_signs,
    calibrated_steering_limits,
    FourWheelSteeringKinematics,
    integrate_planar_pose,
    physical_steering_to_joint,
    steering_alignment_scale,
    steering_feedback_to_physical,
)
import pytest
import yaml


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
    assert commands['f1'].steering == pytest.approx(math.pi)
    assert abs(commands['f1'].steering - (math.pi - 0.1)) == pytest.approx(0.1)
    assert commands['f1'].velocity == pytest.approx(-10.0)


@pytest.mark.parametrize('sign', [-1.0, 1.0])
def test_bounded_steering_does_not_cross_opposite_pi_limits(model, sign):
    current = sign * (math.pi - 0.1)
    desired = -sign * (math.pi - 0.02)
    vx, vy = math.cos(desired), math.sin(desired)
    commands = model.inverse(
        vx, vy, 0.0, {name: current for name in POSITIONS})

    for command in commands.values():
        assert command.steering == pytest.approx(sign * 0.02)
        assert command.velocity == pytest.approx(-10.0)
        assert abs(command.steering - current) < math.pi
        assert command.velocity * model.wheel_radius * math.cos(
            command.steering) == pytest.approx(vx)
        assert command.velocity * model.wheel_radius * math.sin(
            command.steering) == pytest.approx(vy)


@pytest.mark.parametrize('sign', [-1.0, 1.0])
def test_bounded_endpoint_flip_preserves_reverse_twist_and_speed_limit(sign):
    model = FourWheelSteeringKinematics(
        POSITIONS, 0.1, max_wheel_speed=4.0)
    current = sign * (math.pi - 0.1)
    commands = model.inverse(
        1.0, 0.0, 0.0, {name: current for name in POSITIONS})

    assert all(command.steering == pytest.approx(sign * math.pi)
               for command in commands.values())
    assert all(command.velocity == pytest.approx(-4.0)
               for command in commands.values())
    assert model.body_velocity(
        {name: command.steering for name, command in commands.items()},
        {name: command.velocity for name, command in commands.items()},
    ) == pytest.approx((0.4, 0.0, 0.0))


def test_narrow_steering_limits_select_feasible_reverse_candidate():
    model = FourWheelSteeringKinematics(
        POSITIONS, 0.1, max_wheel_speed=100.0,
        steering_limits=(-0.5, 0.5))
    commands = model.inverse(
        -1.0, 0.0, 0.0, {name: 0.1 for name in POSITIONS})

    assert all(command.steering == pytest.approx(0.0)
               for command in commands.values())
    assert all(command.velocity == pytest.approx(-10.0)
               for command in commands.values())


@pytest.mark.parametrize('sign', [-1.0, 1.0])
def test_same_direction_endpoint_is_held_with_either_atan2_sign(model, sign):
    current = sign * (math.pi - 0.01)
    commands = model.inverse(
        -1.0, -sign * 0.0, 0.0, {name: current for name in POSITIONS})
    assert all(command.steering == pytest.approx(sign * math.pi)
               for command in commands.values())
    assert all(command.velocity == pytest.approx(10.0)
               for command in commands.values())


@pytest.mark.parametrize('with_feedback', [False, True])
def test_unreachable_heading_stops_all_wheels_instead_of_clipping(with_feedback):
    model = FourWheelSteeringKinematics(
        POSITIONS, 0.1, max_wheel_speed=100.0,
        steering_limits_by_wheel={'r2': (-0.1, 0.1)})
    current = {'f1': 0.3, 'f2': -0.4, 'r1': 0.2, 'r2': 0.05}
    commands = model.inverse(
        0.0, 1.0, 0.0, current if with_feedback else None)
    assert all(command.velocity == 0.0 for command in commands.values())
    assert {name: command.steering for name, command in commands.items()} == pytest.approx(
        current if with_feedback else {name: 0.0 for name in POSITIONS})


def test_per_wheel_bounds_choose_reverse_heading_without_feedback():
    model = FourWheelSteeringKinematics(
        POSITIONS, 0.1, max_wheel_speed=100.0,
        steering_limits_by_wheel={'f2': (-0.1, 0.1)})
    commands = model.inverse(-1.0, 0.0, 0.0)
    assert commands['f2'].steering == pytest.approx(0.0)
    assert commands['f2'].velocity == pytest.approx(-10.0)
    for command in commands.values():
        assert command.velocity * 0.1 * math.cos(command.steering) == pytest.approx(-1.0)
        assert command.velocity * 0.1 * math.sin(command.steering) == pytest.approx(0.0)


def test_zero_velocity_clamps_only_out_of_limit_feedback_and_stops_drive():
    model = FourWheelSteeringKinematics(
        POSITIONS, 0.1, steering_limits_by_wheel={'f2': (-0.2, 0.2)})
    commands = model.inverse(0.0, 0.0, 0.0, {'f2': 0.21, 'r1': -0.5})
    assert commands['f2'].steering == pytest.approx(0.2)
    assert commands['r1'].steering == pytest.approx(-0.5)
    assert all(command.velocity == 0.0 for command in commands.values())


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


@pytest.mark.parametrize('sign', [-1.0, 1.0])
@pytest.mark.parametrize('offset', [-0.0056, 0.0056])
def test_calibrated_feedback_preserves_bounded_endpoints(sign, offset):
    joints = [-math.pi, math.pi, -math.pi + 0.001, math.pi - 0.001]
    signs, offsets = [sign] * 4, [offset] * 4
    physical = steering_feedback_to_physical(joints, signs, offsets)
    assert physical == pytest.approx([joint * sign + offset for joint in joints])
    assert physical_steering_to_joint(physical, signs, offsets) == pytest.approx(joints)


def test_calibrated_limits_match_configured_urdf_and_keep_commands_in_bounds():
    package = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((package / 'config/four_wheel_steering.yaml').read_text())[
        'four_wheel_steering_node']['ros__parameters']
    urdf = ET.parse(package.parent / 'agri_robot_description/urdf/xacro/wheel_modules.xacro')
    joint_bounds = params['steering_joint_limits']
    for name in params['steering_joint_names']:
        steer_joint = next(joint for joint in urdf.iter('joint') if joint.get('name') == name)
        assert steer_joint.get('type') == 'revolute'
        assert joint_bounds == pytest.approx([
            float(steer_joint.find('limit').get('lower')),
            float(steer_joint.find('limit').get('upper')),
        ])
    names = params['wheel_names']
    signs, offsets = params['steering_joint_signs'], params['steering_joint_offsets']
    physical_bounds = calibrated_steering_limits(
        params['steering_limits'], joint_bounds, signs, offsets)
    model = FourWheelSteeringKinematics(
        {name: params['wheel_positions'][2 * index:2 * index + 2]
         for index, name in enumerate(names)},
        params['wheel_radius'], max_wheel_speed=100.0,
        steering_limits=tuple(params['steering_limits']),
        steering_limits_by_wheel=dict(zip(names, physical_bounds)))

    for endpoint in joint_bounds:
        raw_joint = [endpoint * 0.97] * 4
        physical = steering_feedback_to_physical(raw_joint, signs, offsets)
        for vx, vy, wz in [(0.1, 0.0, 0.0), (-0.1, -0.001, 0.0), (-0.1, 0.0, 0.1)]:
            commands = model.inverse(vx, vy, wz, dict(zip(names, physical)))
            target = [commands[name].steering for name in names]
            raw_command = physical_steering_to_joint(target, signs, offsets)
            for index, name in enumerate(names):
                assert joint_bounds[0] <= raw_command[index] <= joint_bounds[1]
                assert abs(raw_command[index] - raw_joint[index]) <= math.pi
                x, y = model.wheel_positions[name]
                command = commands[name]
                speed = command.velocity * model.wheel_radius
                assert speed * math.cos(command.steering) == pytest.approx(vx - wz * y)
                assert speed * math.sin(command.steering) == pytest.approx(vy + wz * x)


@pytest.mark.parametrize('limits', [(-1.0,), (1.0, -1.0), (0.0, math.inf)])
def test_invalid_bounded_limits_are_rejected(limits):
    with pytest.raises(ValueError):
        FourWheelSteeringKinematics(POSITIONS, 0.1, steering_limits=limits)
    with pytest.raises(ValueError):
        calibrated_steering_limits((-math.pi, math.pi), limits, [-1.0], [0.0])


def test_disjoint_calibrated_limits_and_unknown_wheels_are_rejected():
    with pytest.raises(ValueError):
        calibrated_steering_limits((-0.1, 0.1), (-0.1, 0.1), [1.0], [1.0])
    with pytest.raises(ValueError):
        FourWheelSteeringKinematics(
            POSITIONS, 0.1, steering_limits_by_wheel={'unknown': (-1.0, 1.0)})


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


@pytest.mark.parametrize('error', [2 * math.pi - 0.02, -2 * math.pi + 0.02, 2 * math.pi + 0.02])
def test_bounded_alignment_does_not_accept_modulo_shortcuts(error):
    assert steering_alignment_scale([error], 0.05, 0.35) == 0.0


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
