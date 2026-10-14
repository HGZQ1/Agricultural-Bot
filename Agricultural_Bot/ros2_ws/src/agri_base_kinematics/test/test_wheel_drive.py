"""Drive acceleration and stop boundaries, independent of ROS scheduling."""

import pytest

from agri_base_kinematics.kinematics import (
    FourWheelSteeringKinematics, steering_alignment_scale,
)
from agri_base_kinematics.wheel_drive import WheelDriveLimiter


def test_start_from_rest_limits_every_wheel_and_preserves_ratios():
    limiter = WheelDriveLimiter(max_acceleration=0.8)
    target = [1.2, 0.6, -0.3, -1.2]
    assert limiter.step(target, 1_000_000_000) == [0.] * 4
    previous = [0.] * 4
    for index in range(1, 81):
        output = limiter.step(target, 1_000_000_000+index*20_000_000)
        assert max(abs(a-b) for a, b in zip(output, previous)) <= 0.016+1e-12
        assert output == pytest.approx([target[0]*output[0]/1.2,
                                       target[1]*output[0]/1.2,
                                       target[2]*output[0]/1.2,
                                       target[3]*output[0]/1.2])
        previous = output
    assert output == pytest.approx(target)


def test_steering_hold_then_alignment_recovery_does_not_release_full_speed():
    kinematics = FourWheelSteeringKinematics(
        {'f1': [.23, .17], 'f2': [.23, -.17],
         'r1': [-.23, .17], 'r2': [-.23, -.17]}, .127)
    limiter = WheelDriveLimiter(max_acceleration=.8)
    commands = kinematics.inverse(.15, 0., 0.)
    full = [command.velocity for command in commands.values()]
    blocked_scale = steering_alignment_scale([.5]*4, .05, .35)
    assert limiter.step([v*blocked_scale for v in full], 1_000_000_000) == [0.]*4
    for index in range(1, 30):
        assert limiter.step([v*blocked_scale for v in full],
                            1_000_000_000+index*20_000_000) == [0.]*4
    recovered = steering_alignment_scale([0.]*4, .05, .35)
    output = limiter.step([v*recovered for v in full], 1_600_000_000)
    assert output == pytest.approx([.016]*4)
    assert max(output) < min(full)


def test_steering_permission_decrease_precedes_the_ramp():
    limiter = WheelDriveLimiter(max_acceleration=1.)
    limiter.step([1.]*4, 1_000_000_000)
    for index in range(1, 51):
        limiter.step([1.]*4, 1_000_000_000+index*20_000_000)
    assert limiter.previous == pytest.approx([1.]*4)
    output = limiter.step([.1]*4, 2_020_000_000, maximum_magnitudes=[.1]*4)
    assert output == pytest.approx([.1]*4)
    output = limiter.step([1.]*4, 2_040_000_000, maximum_magnitudes=[1.]*4)
    assert output == pytest.approx([.12]*4)


@pytest.mark.parametrize('stop_cause', ['locked_zero', 'command_timeout', 'feedback_loss'])
def test_stop_is_immediate_and_resumption_starts_at_zero(stop_cause):
    limiter = WheelDriveLimiter(max_acceleration=1.)
    limiter.step([1.]*4, 1_000_000_000)
    for index in range(1, 20):
        limiter.step([1.]*4, 1_000_000_000+index*20_000_000)
    assert min(limiter.previous) > .3
    target = [0.]*4 if stop_cause == 'locked_zero' else [1.]*4
    stopped = limiter.step(target, 1_400_000_000, enabled=stop_cause == 'locked_zero')
    assert stopped == [0.]*4
    assert limiter.step([1.]*4, 1_420_000_000) == pytest.approx([.02]*4)


@pytest.mark.parametrize('next_stamp', [1_100_000_000, 1_380_000_000, 2_000_000_000])
def test_backwards_frozen_or_gapped_sim_time_stops(next_stamp):
    limiter = WheelDriveLimiter(max_acceleration=1., maximum_interval=.25)
    limiter.step([1.]*4, 1_000_000_000)
    for index in range(1, 20):
        limiter.step([1.]*4, 1_000_000_000+index*20_000_000)
    assert limiter.step([1.]*4, next_stamp) == [0.]*4
    assert limiter.step([1.]*4, next_stamp+20_000_000) == pytest.approx([.02]*4)


def test_reversing_nonzero_commands_obeys_per_wheel_acceleration():
    limiter = WheelDriveLimiter(max_acceleration=1.)
    limiter.step([1.]*4, 1_000_000_000)
    limiter.step([1.]*4, 1_200_000_000)
    assert limiter.step([-1.]*4, 1_220_000_000) == pytest.approx([.18]*4)


@pytest.mark.parametrize('acceleration', [0., -1., float('nan'), float('inf')])
def test_invalid_acceleration_is_rejected(acceleration):
    with pytest.raises(ValueError):
        WheelDriveLimiter(max_acceleration=acceleration)
