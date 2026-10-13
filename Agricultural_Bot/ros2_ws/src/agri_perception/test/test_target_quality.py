"""Regression cases for depth contamination, association and stale identities."""

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from agri_perception.target_quality import (
    Candidate, DepthConfig, DepthEstimate, ImagePairBuffer, TargetTracker,
    observation_time_error, project_candidates, sample_depth,
)


def test_random_depth_median_rejects_outliers_and_reports_quality():
    depth = np.full((41, 41), 0.8)
    depth[15:18, 15:25] = 1.8
    depth[20:22, 20:24] = np.nan
    estimate = sample_depth(depth, (5, 5, 35, 35), np.random.default_rng(123))
    assert estimate.valid and estimate.depth_m == pytest.approx(0.8)
    assert 15 <= estimate.sample_count < 81
    assert 0.6 <= estimate.valid_fraction < 1
    assert estimate.mad_m == 0


def test_mask_sampling_avoids_background_and_requires_enough_pixels():
    depth = np.full((41, 41), 1.5)
    mask = np.zeros_like(depth, dtype=bool)
    mask[17:24, 17:24] = True
    depth[mask] = 0.65
    estimate = sample_depth(depth, (5, 5, 35, 35), np.random.default_rng(7), mask=mask)
    assert estimate.valid and estimate.segmentation_used
    assert estimate.depth_m == pytest.approx(0.65)
    mask[:] = False
    assert sample_depth(depth, (5, 5, 35, 35), np.random.default_rng(7), mask=mask).reason == 'insufficient_sampling_pixels'
    required = replace(DepthConfig(), require_segmentation=True)
    assert sample_depth(depth, (5, 5, 35, 35), np.random.default_rng(7), required).reason == 'segmentation_required'


@pytest.mark.parametrize('depth', [np.zeros((41, 41)), np.full((41, 41), np.nan),
                                  np.full((41, 41), 0.06), np.full((41, 41), 2.01)])
def test_invalid_depth_fails_closed(depth):
    estimate = sample_depth(depth, (5, 5, 35, 35), np.random.default_rng(123))
    assert not estimate.valid and estimate.reason == 'insufficient_valid_depth'


def test_large_depth_dispersion_fails_even_when_all_samples_in_range():
    depth = np.linspace(0.2, 1.8, 41*41).reshape(41, 41)
    estimate = sample_depth(depth, (5, 5, 35, 35), np.random.default_rng(123))
    assert not estimate.valid and estimate.reason == 'depth_dispersion'
    assert estimate.valid_fraction == 1 and estimate.mad_m > 0.03


def test_green_and_failed_red_depth_cannot_shift_class_position_association():
    depth = np.zeros((80, 100))
    depth[10:40, 10:40] = 0.7  # Green, although valid, must not become a target.
    depth[10:40, 70:100] = 0.9  # Red detection index 2 has valid depth.
    candidates, rejected = project_candidates(
        [(10, 10, 40, 40), (40, 10, 70, 40), (70, 10, 100, 40)],
        [1, 0, 0], [0.99, 0.95, 0.87], {0: 'tomato_R', 1: 'tomato_G'},
        depth, (100, 100, 50, 40), 42)
    assert [(c.detection_index, c.class_name, c.confidence) for c in candidates] == [(2, 'tomato_R', 0.87)]
    assert candidates[0].position == pytest.approx((0.315, -0.135, 0.9))
    assert rejected == [(1, 'insufficient_valid_depth')]
    all_candidates, _ = project_candidates(
        [(10, 10, 40, 40), (70, 10, 100, 40)], [1, 0], [0.99, 0.87],
        {0: 'tomato_R', 1: 'tomato_G'}, depth, (100, 100, 50, 40), 42,
        mature_only=False)
    assert [c.class_name for c in all_candidates] == ['tomato_G', 'tomato_R']


@pytest.mark.parametrize(('rgb', 'depth', 'now', 'reason'), [
    (0, 1_000_000_000, 1_000_000_000, 'zero_timestamp'),
    (1_000_000_000, 1_060_000_000, 1_100_000_000, 'rgb_depth_timestamp_skew'),
    (1_000_000_000, 1_000_000_000, 999_000_000, 'future_observation'),
    (1_000_000_000, 1_000_000_000, 1_510_000_000, 'expired_observation'),
    (1_000_000_000, 1_030_000_000, 1_100_000_000, ''),
])
def test_capture_timestamps_are_checked(rgb, depth, now, reason):
    assert observation_time_error(rgb, depth, now) == reason


def _candidate(index, xyz, bbox, label='tomato_R'):
    return Candidate(index, label, 0.9, bbox, xyz, DepthEstimate(True, '', 0.8, 81, 1, 0))


def test_ids_survive_detection_reordering_without_mixing_nearby_fruit():
    tracker = TargetTracker('test')
    a = _candidate(0, (0, 0, 0.8), (5, 5, 35, 35))
    b = _candidate(1, (0.15, 0, 0.8), (35, 5, 65, 35))
    initial = tracker.update([a, b], 1_000_000_000)
    later = tracker.update([replace(b, detection_index=0), replace(a, detection_index=1)], 1_100_000_000)
    stable = tracker.update([b, a], 1_200_000_000)
    assert [n[1] for n in later] == [initial[1][1], initial[0][1]]
    assert all(n[2] and n[3] == 3 for n in stable)


def test_motion_expiration_and_clock_reset_remove_stable_claims():
    tracker = TargetTracker('test')
    a = _candidate(0, (0, 0, 0.8), (5, 5, 35, 35))
    for stamp in (1_000_000_000, 1_100_000_000, 1_200_000_000):
        stable = tracker.update([a], stamp)[0]
    assert stable[2]
    moved = tracker.update([replace(a, position=(0.04, 0, 0.8))], 1_300_000_000)[0]
    assert moved[1] == stable[1] and not moved[2] and moved[3] == 1
    expired = tracker.update([a], 2_000_000_000)[0]
    assert expired[1] != stable[1] and not expired[2]
    rewind = tracker.update([a], 1_000_000_000)[0]
    assert rewind[1] != expired[1] and not rewind[2]
    assert tracker.update([], 1_100_000_000) == []
    returned = tracker.update([a], 1_200_000_000)[0]
    assert returned[1] != rewind[1] and returned[3] == 1 and not returned[2]


def test_pair_buffer_handles_arrival_order_skew_duplicates_and_bounded_queue():
    buffer = ImagePairBuffer(capacity=2)
    now = 1_200_000_000
    assert buffer.push('rgb', 1_000_000_000, 'rgb1', now) is None
    assert buffer.push('depth', 1_090_000_000, 'depth-wrong', now) is None
    assert buffer.push('depth', 1_020_000_000, 'depth1', now) == ('rgb1', 'depth1')
    assert buffer.push('rgb', 1_000_000_000, 'duplicate', now) is None
    assert buffer.push('depth', 1_150_000_000, 'depth2', now) is None
    assert buffer.push('rgb', 1_150_000_000, 'rgb2', now) == ('rgb2', 'depth2')
    for i in range(5):
        buffer.push('rgb', now+i*1_000_000, i, now+10_000_000)
    assert len(buffer.rgb) == 2
    assert buffer.push('rgb', 1, 'expired', now) is None
