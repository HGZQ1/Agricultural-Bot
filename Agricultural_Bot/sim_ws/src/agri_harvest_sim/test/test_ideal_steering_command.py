"""Offline fail-closed checks for the simulation-only r2 command bridge.

No ROS node or Gazebo transport is created. Tests exercise the production
validator with the actual stalled r2 angle and receipt-clock semantics.
"""
import importlib.util
import math
from pathlib import Path

import pytest


PATH = Path(__file__).parents[1] / 'scripts' / 'ideal_steering_command.py'
SPEC = importlib.util.spec_from_file_location('tested_ideal_steering_command', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
IdealSteeringCommand = MODULE.IdealSteeringCommand
BASE_NS = 829_447_000_000
ACTUAL = -.824242667524138


def validator(**kwargs):
    return IdealSteeringCommand(-3.1416, 3.1416, **kwargs)


def receive(command, *, target=ACTUAL, ros_ns=BASE_NS, wall_now=10.,
            actual=ACTUAL, feedback_stamp_ns=None, values=None):
    return command.accept(
        [.11, -.22, .33, target] if values is None else values,
        ros_ns=ros_ns, wall_now=wall_now, actual=actual,
        feedback_stamp_ns=ros_ns if feedback_stamp_ns is None else feedback_stamp_ns)


def test_selects_only_fourth_reference_and_preserves_real_receipt_stamp():
    command = validator()
    report = receive(command)
    assert report == {
        'joint_name': 'r2_steer_joint', 'position_rad': ACTUAL,
        'stamp_ns': BASE_NS, 'simulation_only': True, 'idealized': True,
    }
    assert command.current(BASE_NS + 100_000_000, 10.1)


def test_current_watchdog_does_not_publish_or_renew_received_reference():
    command = validator()
    receive(command)
    for offset_ns in (50_000_000, 100_000_000, 150_000_000, 200_000_000):
        assert command.current(BASE_NS + offset_ns, 10. + offset_ns / 1e9)
        assert command.last_stamp_ns == BASE_NS
        assert command.last_wall == 10.
    assert not command.current(BASE_NS + 200_000_001, 10.200000001)
    assert not command.active


def test_exact_duplicate_does_not_republish_or_extend_wall_lease():
    command = validator()
    receive(command)
    assert receive(command, wall_now=10.49) is None
    assert command.active
    assert command.last_wall == 10.
    assert command.current(BASE_NS, 10.5)
    assert receive(command, wall_now=10.500001) is None
    assert not command.active


def test_changed_target_at_same_stamp_invalidates_instead_of_inventing_time():
    command = validator()
    receive(command)
    assert receive(command, target=ACTUAL + .01, wall_now=10.01) is None
    assert not command.active
    assert 'without clock advance' in command.reason
    assert command.last_stamp_ns == BASE_NS
    # Repeated old data at that stamp cannot revive the invalidated lease.
    assert receive(command, wall_now=10.02) is None
    assert not command.active


@pytest.mark.parametrize('target,accepted', [(0.03, True), (.030001, False)])
def test_initial_reference_has_continuity_guard(target, accepted):
    command = validator()
    report = receive(command, target=target, actual=0.)
    assert (report is not None) is accepted
    assert command.active is accepted


def test_actual_stalled_joint_cannot_jump_directly_to_old_moving_reference():
    command = validator()
    assert receive(command, target=-.4988342751840266) is None
    assert not command.active
    assert 'discontinuous' in command.reason


def test_active_reference_can_follow_upstream_without_extra_hidden_slew():
    command = validator()
    receive(command)
    report = receive(command, target=ACTUAL + .05, ros_ns=BASE_NS + 100_000_000,
                     wall_now=10.1)
    assert report['position_rad'] == pytest.approx(ACTUAL + .05)
    assert report['stamp_ns'] == BASE_NS + 100_000_000


@pytest.mark.parametrize('watchdog', ['sim', 'wall'])
def test_expired_reference_requires_continuous_reclaim(watchdog):
    command = validator()
    receive(command)
    ros_ns = BASE_NS + (210_000_000 if watchdog == 'sim' else 1_000_000)
    wall_now = 10.01 if watchdog == 'sim' else 10.51
    assert not command.current(ros_ns, wall_now)
    assert receive(command, target=ACTUAL + .1, ros_ns=ros_ns + 1_000_000,
                   wall_now=wall_now + .01) is None
    assert not command.active
    report = receive(command, target=ACTUAL + .02, ros_ns=ros_ns + 2_000_000,
                     wall_now=wall_now + .02)
    assert report is not None


@pytest.mark.parametrize('clock', ['ros', 'wall'])
def test_clock_rollback_clears_target_and_requires_new_continuous_data(clock):
    command = validator()
    receive(command)
    ros_ns = BASE_NS - 1 if clock == 'ros' else BASE_NS
    wall_now = 10. if clock == 'ros' else 9.999
    assert not command.current(ros_ns, wall_now)
    assert not command.active
    assert command.last_stamp_ns is command.last_target is command.last_wall is None
    # This new clock epoch is valid only after a fresh matching source message.
    report = receive(command, ros_ns=ros_ns + 1, wall_now=wall_now + .001)
    assert report is not None
    assert report['stamp_ns'] == ros_ns + 1


def test_explicit_invalidation_does_not_leave_cached_target_usable():
    command = validator()
    receive(command)
    assert command.invalidate('operator stopped compensation') is None
    assert not command.current(BASE_NS + 1, 10.001)
    assert receive(command, target=ACTUAL + .1, ros_ns=BASE_NS + 2,
                   wall_now=10.002) is None
    assert not command.active


@pytest.mark.parametrize('age_ns,accepted', [
    (-50_000_000, True), (-50_000_001, False),
    (250_000_000, True), (250_000_001, False),
])
def test_feedback_age_and_future_bounds_are_not_relaxed(age_ns, accepted):
    command = validator()
    report = receive(command, feedback_stamp_ns=BASE_NS - age_ns)
    assert (report is not None) is accepted
    assert command.active is accepted


@pytest.mark.parametrize('values', [
    [], [0., 0., ACTUAL], [0., 0., 0., ACTUAL, 0.],
    [math.nan, 0., 0., ACTUAL], [0., math.inf, 0., ACTUAL],
    [0., 0., -math.inf, ACTUAL], [0., 0., 0., math.nan],
    [0., 0., 0., 3.141601], [0., 0., 0., -3.141601],
])
def test_bad_reference_invalidates_existing_lease(values):
    command = validator()
    receive(command)
    assert receive(command, values=values, ros_ns=BASE_NS + 1,
                   wall_now=10.001) is None
    assert not command.active


@pytest.mark.parametrize('actual', [math.nan, math.inf, -3.141601, 3.141601])
def test_bad_actual_feedback_invalidates_existing_lease(actual):
    command = validator()
    receive(command)
    assert receive(command, actual=actual, ros_ns=BASE_NS + 1,
                   wall_now=10.001) is None
    assert not command.active


@pytest.mark.parametrize('overrides', [
    {'values': None}, {'values': [0., 0., 0., 'bad']},
    {'actual': None}, {'feedback_stamp_ns': None},
    {'feedback_stamp_ns': True}, {'feedback_stamp_ns': float(BASE_NS)},
    {'ros_ns': True}, {'ros_ns': float(BASE_NS)},
    {'wall_now': None}, {'wall_now': 'not a clock'},
])
def test_malformed_types_fail_closed_without_crashing_callback(overrides):
    command = validator()
    receive(command)
    args = {
        'values': [0., 0., 0., ACTUAL], 'ros_ns': BASE_NS + 1,
        'wall_now': 10.001, 'actual': ACTUAL,
        'feedback_stamp_ns': BASE_NS + 1,
    }
    args.update(overrides)
    assert command.accept(**args) is None
    assert not command.active


@pytest.mark.parametrize('ros_ns,wall_now', [
    (0, 10.), (-1, 10.), (BASE_NS, math.nan), (BASE_NS, math.inf),
])
def test_invalid_clocks_disable_active_compensation(ros_ns, wall_now):
    command = validator()
    receive(command)
    assert not command.current(ros_ns, wall_now)
    assert not command.active


@pytest.mark.parametrize('ros_ns,feedback_stamp_ns', [(True, True), (1, True)])
def test_boolean_stamp_cannot_activate_an_uninitialized_bridge(ros_ns, feedback_stamp_ns):
    command = validator()
    assert command.accept([0., 0., 0., ACTUAL], ros_ns=ros_ns, wall_now=10.,
                          actual=ACTUAL, feedback_stamp_ns=feedback_stamp_ns) is None
    assert not command.active


@pytest.mark.parametrize('lower,upper,kwargs', [
    (0., 0., {}), (1., -1., {}), (-math.inf, 1., {}),
    (-1., math.nan, {}), (-1., 1., {'sim_timeout': 0.}),
    (-1., 1., {'wall_timeout': -1.}),
    (-1., 1., {'feedback_max_age': 0.}),
    (-1., 1., {'future_skew': -1.}),
    (-1., 1., {'startup_tolerance': 0.}),
])
def test_invalid_safety_bounds_rejected_at_construction(lower, upper, kwargs):
    with pytest.raises(ValueError):
        IdealSteeringCommand(lower, upper, **kwargs)
