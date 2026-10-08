"""Exercise the frame and rotation checks used by the field motion acceptance script."""

import importlib.util
import math
from pathlib import Path
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[4] / 'scripts' / 'check_chassis_motion.py'
SPEC = importlib.util.spec_from_file_location('check_chassis_motion', SCRIPT)
motion = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = motion
SPEC.loader.exec_module(motion)


def pose(stamp, x=0.0, y=0.0, yaw=0.0):
    return motion.Pose(stamp, x, y, 0.0, 0.0, 0.0, yaw)


def test_equal_endpoints_without_motion_cannot_pass_full_rotation():
    assert not motion.rotation_passed([0.0] * 100, 2 * math.pi)


def test_full_rotation_counts_across_wrapped_yaw_boundary():
    yaw = [motion.wrap_angle(math.pi / 2 + i * 2 * math.pi / 200) for i in range(201)]
    assert motion.rotation_passed(yaw, 2 * math.pi)
    assert motion.yaw_progress(yaw) == pytest.approx(2 * math.pi)
    assert not motion.rotation_passed(yaw[::-1], 2 * math.pi)


def test_odometry_is_aligned_to_spawn_translation_and_heading():
    initial_odom = pose(0, 2, 3, -math.pi / 2)
    initial_truth = pose(0, 0, -6, math.pi / 2)
    aligned = motion.align_odometry(pose(10, 2, 2, -math.pi / 2), initial_odom, initial_truth)
    assert aligned.x == pytest.approx(0, abs=1e-12)
    assert aligned.y == pytest.approx(-5)
    assert aligned.yaw == pytest.approx(math.pi / 2)


def test_arc_endpoint_uses_robot_heading_in_world_frame():
    end = motion.expected_endpoint(pose(0, 0, -6, math.pi / 2), 0.1, 0.15, 3)
    assert end.x < 0 and end.y > -6
    assert end.yaw == pytest.approx(math.pi / 2 + 0.45)


def test_stopped_check_rejects_short_window_and_translation():
    assert not motion.stop_metrics([pose(0), pose(0.4)])['passed']
    assert not motion.stop_metrics([pose(0), pose(0.5, x=0.02)])['passed']
    assert not motion.stop_metrics([pose(0), pose(0.5)])['passed']
    assert motion.stop_metrics([pose(i / 10) for i in range(6)])['passed']


def test_sparse_truth_cannot_prove_sustained_rest():
    metrics = motion.stop_metrics([pose(0), pose(0.5)])
    assert not metrics['passed']
    assert metrics['maximum_truth_sample_gap_s'] == pytest.approx(0.5)


def test_watchdog_checks_stopping_deadline_and_prior_actual_motion():
    before = [pose(i / 20, x=i / 200) for i in range(-10, 1)]
    timely = [pose(i / 20, x=0.1 * min(i / 20, 0.55)) for i in range(41)]
    late = [pose(i / 20, x=0.1 * min(i / 20, 0.8)) for i in range(41)]
    assert motion.command_timeout_metrics(before, timely, 0)['passed']
    assert not motion.command_timeout_metrics(before, late, 0)['passed']
    assert not motion.command_timeout_metrics([pose(-0.5), pose(0)], timely, 0)['passed']


def test_sustained_threshold_ignores_short_spike():
    samples = [
        (0.00, 0.0), (0.05, 1.0), (0.10, 0.0),
        (0.20, 1.0), (0.30, 1.0), (0.40, 1.0),
    ]
    assert motion.first_sustained_threshold(samples, 0.9) == pytest.approx(0.2)


def test_sustained_threshold_requires_complete_window():
    assert motion.first_sustained_threshold([(0.0, 1.0), (0.1, 1.0)], 0.9) is None
