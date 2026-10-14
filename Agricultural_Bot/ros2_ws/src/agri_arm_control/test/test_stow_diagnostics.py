"""Diagnostic reads explain stow rejection without altering lease evidence."""

import pytest
from concurrent.futures import ThreadPoolExecutor
import threading

from agri_arm_control.safety import JointEvidence


def test_read_only_pose_diagnostic_does_not_begin_or_reset_settlement():
    evidence = JointEvidence(.5)
    evidence.update(['arm'], [0.], [0.], 1_000_000_000, 1_000_000_000, 0.)
    report = evidence.pose_report(['arm'], [0.], 1_000_000_000, 0.)
    assert report['fresh'] and report['complete']
    assert report['reasons'] == ['settlement_not_confirmed']
    assert not evidence._settled
    assert not evidence.at(['arm'], [0.], 1_000_000_000, 0.)
    settled = dict(evidence._settled)
    report = evidence.pose_report(['arm'], [0.], 1_300_000_000, .3)
    assert report['passed'] and report['settlement_elapsed_wall_s'] == pytest.approx(.3)
    assert evidence._settled == settled
    report = evidence.pose_report(['arm'], [0.], 1_600_000_000, .6)
    assert not report['passed'] and 'wall_feedback_stale' in report['reasons']
    assert 'ros_feedback_stale_zero_or_future' in report['reasons']
    assert evidence._settled == settled


@pytest.mark.parametrize('position,velocity,reason', [
    (.04, 0., 'position_outside_stow_tolerance'),
    (0., .04, 'velocity_above_stopped_tolerance')])
def test_position_and_velocity_failures_include_actual_named_feedback(position, velocity, reason):
    evidence = JointEvidence(.5)
    evidence.update(['arm'], [position], [velocity], 1_000_000_000, 1_000_000_000, 0.)
    report = evidence.pose_report(['arm'], [0.], 1_000_000_000, 0.)
    assert reason in report['reasons']
    assert report['joints']['arm']['position'] == position
    assert report['joints']['arm']['velocity'] == velocity
    assert report['joints']['arm']['position_error'] == position


def test_exact_duplicate_is_visible_and_does_not_revoke_original_fresh_sample():
    evidence = JointEvidence(.5)
    assert evidence.update(['arm'], [0.], [0.], 1_000_000_000, 1_000_000_000, 0.)
    assert not evidence.update(['arm'], [0.], [0.], 1_000_000_000, 1_000_000_000, .1)
    report = evidence.pose_report(['arm'], [0.], 1_000_000_000, .1)
    assert not report['passed'] and report['complete']
    assert report['wall_age_s'] == .1
    assert report['last_update']['ignored_exact_duplicate']
    assert report['last_update']['reasons'] == ['nonincreasing_stamp']
    assert report['last_update']['wall_age_s'] == 0.


def test_identical_replay_cannot_renew_feedback_or_extend_paused_clock_lease():
    evidence = JointEvidence(.5)
    stamp = 1_000_000_000
    assert evidence.update(['arm'], [0.], [0.], stamp, stamp, 0.)
    assert not evidence.at(['arm'], [0.], stamp, 0.)
    assert evidence.at(['arm'], [0.], stamp+300_000_000, .3)
    settled = dict(evidence._settled)
    for wall in (.31, .4, .5):
        assert not evidence.update(['arm'], [0.], [0.], stamp, stamp+300_000_000, wall)
        assert evidence.stamp == stamp and evidence.wall == 0.
        assert evidence._settled == settled
        assert evidence.available(['arm'], stamp+300_000_000, wall)
    assert not evidence.update(['arm'], [0.], [0.], stamp, stamp+300_000_000, .500001)
    assert not evidence.available(['arm'], stamp+300_000_000, .500001)
    assert not evidence.values and not evidence._settled


@pytest.mark.parametrize('change', ['position', 'velocity', 'names', 'older', 'clock', 'wall'])
def test_nonidentical_nonincreasing_or_backwards_clock_still_revoke_immediately(change):
    evidence = JointEvidence(.5)
    stamp = 1_000_000_000
    evidence.update(['arm'], [0.], [0.], stamp, stamp+10_000_000, 0.)
    evidence.at(['arm'], [0.], stamp+10_000_000, 0.)
    names, q, qd, received, now, wall = ['arm'], [0.], [0.], stamp, stamp+20_000_000, .1
    if change == 'position':q = [.000001]  # No new numerical tolerance.
    elif change == 'velocity':qd = [.000001]
    elif change == 'names':names = ['other']
    elif change == 'older':received -= 1
    elif change == 'clock':now = stamp+9_000_000
    elif change == 'wall':wall = -.001
    assert not evidence.update(names, q, qd, received, now, wall)
    assert not evidence.values and not evidence._settled
    assert not evidence.last_update.get('ignored_exact_duplicate', False)


def test_duplicate_does_not_hide_velocity_excursion_or_warm_new_settlement():
    evidence = JointEvidence(.5)
    stamp = 1_000_000_000
    evidence.update(['arm'], [0.], [.04], stamp, stamp, 0.)
    for wall in (.1, .2, .3):
        assert not evidence.update(['arm'], [0.], [.04], stamp, stamp+300_000_000, wall)
        assert not evidence.at(['arm'], [0.], stamp+300_000_000, wall)
    assert not evidence._settled
    report = evidence.pose_report(['arm'], [0.], stamp+300_000_000, .3)
    assert 'velocity_above_stopped_tolerance' in report['reasons']


def test_concurrent_exact_replays_are_atomic_without_lease_or_settlement_changes():
    evidence = JointEvidence(.5)
    stamp = 1_000_000_000
    evidence.update(['arm'], [0.], [0.], stamp, stamp, 0.)
    evidence.at(['arm'], [0.], stamp, 0.)
    before = dict(evidence._settled)
    barrier = threading.Barrier(8)

    def replay(_index):
        barrier.wait(timeout=2.)
        return evidence.update(['arm'], [0.], [0.], stamp, stamp+300_000_000, .3)

    with ThreadPoolExecutor(max_workers=8) as pool:
        assert list(pool.map(replay, range(8))) == [False]*8
    assert evidence.wall == 0. and evidence.stamp == stamp
    assert evidence._settled == before
    assert evidence.at(['arm'], [0.], stamp+300_000_000, .3)


def test_incomplete_nonfinite_and_timestamp_failures_are_distinguishable():
    evidence = JointEvidence(.5)
    assert not evidence.update(['arm'], [float('nan')], [], 0, 1_000_000_000, 0.)
    report = evidence.pose_report(['arm'], [0.], 1_000_000_000, 0.)
    assert {'incomplete_joint_arrays', 'nonfinite_joint_measurement',
            'stale_zero_or_future_stamp', 'nonincreasing_stamp'} <= set(
        report['last_update']['reasons'])
