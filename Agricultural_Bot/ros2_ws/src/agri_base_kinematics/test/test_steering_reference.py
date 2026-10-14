"""Check bounded steering references and real SIM-time motion limits."""

import pytest

from agri_base_kinematics.steering_reference import SteeringReferenceLimiter


def test_step_target_obeys_speed_and_acceleration_and_arrives():
    limiter = SteeringReferenceLimiter([(-3.14, 3.14)])
    previous = limiter.step([1.], [.2], 1_000_000_000)[0]
    velocity = 0.
    assert previous == .2
    for index in range(1, 301):
        value = limiter.step([1.], [.2], 1_000_000_000+index*20_000_000)[0]
        next_velocity = (value-previous)/.02
        assert abs(next_velocity) <= .5+1e-12
        assert abs(next_velocity-velocity)/.02 <= 1.+1e-10
        assert .2 <= value <= 1.
        previous, velocity = value, next_velocity
    assert previous == pytest.approx(1., abs=1e-8)
    assert velocity == pytest.approx(0., abs=1e-8)


def test_target_reversal_brakes_without_a_velocity_jump():
    limiter = SteeringReferenceLimiter([(-3.14, 3.14)])
    previous = limiter.step([2.], [0.], 1_000_000_000)[0]
    velocity = 0.
    for index in range(1, 351):
        target = 2. if index < 70 else -.3
        value = limiter.step([target], [0.], 1_000_000_000+index*20_000_000)[0]
        next_velocity = (value-previous)/.02
        assert abs(next_velocity) <= .5+1e-12
        assert abs(next_velocity-velocity)/.02 <= 1.+1e-10
        assert -3.14 <= value <= 3.14
        if index == 70:
            assert next_velocity > 0.  # Brake before reversing the old motion.
        previous, velocity = value, next_velocity
    assert previous == pytest.approx(-.3, abs=1e-8)


def test_near_joint_stop_remains_bounded_without_wrapping():
    limiter = SteeringReferenceLimiter([(-3.14, 3.14)])
    previous = limiter.step([3.14], [3.13], 1_000_000_000)[0]
    velocity = 0.
    for index in range(1, 1601):
        target = 3.14 if index < 100 else -3.14
        value = limiter.step([target], [3.13], 1_000_000_000+index*20_000_000)[0]
        next_velocity = (value-previous)/.02
        assert -3.14 <= value <= 3.14
        assert abs(next_velocity-velocity)/.02 <= 1.+1e-10
        previous, velocity = value, next_velocity
    assert previous == pytest.approx(-3.14, abs=1e-8)


def test_hold_uses_measured_angle_and_resumes_from_rest():
    limiter = SteeringReferenceLimiter([(-3.14, 3.14)])
    limiter.step([2.], [.2], 1_000_000_000)
    limiter.step([2.], [.2], 1_020_000_000)
    assert limiter.step([2.], [.15], 1_040_000_000, hold=True) == [.15]
    assert limiter.velocity == [0.]
    assert limiter.step([2.], [.15], 1_060_000_000) == pytest.approx([.1504])


@pytest.mark.parametrize('stamp', [1_010_000_000, 1_020_000_000, 1_400_000_000])
def test_bad_sim_time_clears_history(stamp):
    limiter = SteeringReferenceLimiter([(-3.14, 3.14)])
    limiter.step([2.], [.2], 1_000_000_000)
    limiter.step([2.], [.2], 1_020_000_000)
    assert limiter.step([2.], [.3], stamp) is None
    assert limiter.position is None
    assert limiter.step([2.], [.3], stamp+20_000_000) == [.3]


def test_missing_and_invalid_feedback_cannot_publish_a_reference():
    limiter = SteeringReferenceLimiter([(-1., 1.)])
    limiter.step([.8], [.2], 1_000_000_000)
    assert limiter.step([.8], [.2], 1_020_000_000, enabled=False) is None
    assert limiter.position is None
    for measured in ([float('nan')], [1.1], []):
        assert limiter.step([.8], measured, 1_040_000_000) is None
    assert limiter.step([2.], [.2], 1_060_000_000) is None


@pytest.mark.parametrize('parameter', ['max_velocity', 'max_acceleration', 'maximum_interval'])
@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf')])
def test_invalid_motion_limits_are_rejected(parameter, value):
    with pytest.raises(ValueError):
        SteeringReferenceLimiter([(-1., 1.)], **{parameter: value})
