import math

from agri_base_adapter.velocity_gate import (
    timestamp_is_current,
    VelocityGateCore,
    ZERO_TWIST,
)
import pytest


@pytest.fixture
def gate():
    return VelocityGateCore(
        ('manual', 'navigation'),
        {'manual': 0.5, 'navigation': 1.0},
        (0.3, 0.0, 0.5),
    )


def test_manual_has_priority_while_both_sources_are_fresh(gate):
    assert gate.update('navigation', (0.2, 0.0, 0.1), 1.0)
    assert gate.update('manual', (0.1, 0.0, -0.2), 1.1)
    assert gate.select(1.2) == ((0.1, 0.0, -0.2), 'manual')


def test_stale_manual_falls_back_to_fresh_navigation(gate):
    gate.update('navigation', (0.2, 0.0, 0.1), 1.0)
    gate.update('manual', (0.1, 0.0, -0.2), 1.0)
    assert gate.select(1.6) == ((0.2, 0.0, 0.1), 'navigation')
    assert gate.select(2.1) == (ZERO_TWIST, 'watchdog')


def test_lock_and_unlock_never_resume_a_buffered_command(gate):
    gate.update('manual', (0.1, 0.0, 0.0), 1.0)
    gate.set_locked(True)
    assert gate.select(1.1) == (ZERO_TWIST, 'locked')
    gate.update('manual', (0.2, 0.0, 0.0), 1.2)
    gate.set_locked(False)
    assert gate.select(1.2) == (ZERO_TWIST, 'watchdog')


def test_limits_disable_lateral_motion_and_clamp_planar_command(gate):
    gate.update('manual', (2.0, 1.0, -2.0), 1.0)
    assert gate.select(1.0) == ((0.3, 0.0, -0.5), 'manual')


def test_nonfinite_and_backward_time_are_rejected(gate):
    assert not gate.update('manual', (math.nan, 0.0, 0.0), 1.0)
    assert gate.select(1.0) == (ZERO_TWIST, 'watchdog')
    gate.update('manual', (0.1, 0.0, 0.0), 2.0)
    assert gate.select(1.0) == (ZERO_TWIST, 'clock_reset')
    assert gate.select(2.1) == (ZERO_TWIST, 'watchdog')


def test_configuration_validation():
    with pytest.raises(ValueError):
        VelocityGateCore(('manual',), {'manual': 0.0}, (1.0, 1.0, 1.0))
    with pytest.raises(ValueError):
        VelocityGateCore(('manual',), {'manual': math.nan}, (1.0, 1.0, 1.0))
    with pytest.raises(ValueError):
        VelocityGateCore(('manual',), {'manual': 1.0}, (-1.0, 1.0, 1.0))


def test_source_can_be_invalidated_immediately(gate):
    gate.update('manual', (0.1, 0.0, 0.0), 1.0)
    gate.clear_source('manual')
    assert gate.select(1.1) == (ZERO_TWIST, 'watchdog')


def test_required_timestamp_window():
    assert timestamp_is_current(9.8, 10.0, 0.5, 0.05)
    assert timestamp_is_current(10.04, 10.0, 0.5, 0.05)
    assert not timestamp_is_current(0.0, 10.0, 0.5, 0.05)
    assert not timestamp_is_current(9.4, 10.0, 0.5, 0.05)
    assert not timestamp_is_current(10.06, 10.0, 0.5, 0.05)
