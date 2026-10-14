"""Geometric and fail-closed tests independent of field metadata or ROS."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from agri_perception.fruit_size import (
    SizeUncertainty, classify_aperture_interval, estimate_sphere_diameter,
)


def _synthetic_sphere_observation(centre, radius, k=(500., 490., 424., 240.)):
    """Analytic tangent projection and surface depth at the arithmetic box centre."""
    x, y, z = centre
    fx, fy, cx, cy = k
    pairs = []
    for a, focal, principal in ((x, fx, cx), (y, fy, cy)):
        tangent = radius*math.sqrt(a*a+z*z-radius*radius)
        pairs.append((focal*(a*z-tangent)/(z*z-radius*radius)+principal,
                      focal*(a*z+tangent)/(z*z-radius*radius)+principal))
    bbox = (pairs[0][0], pairs[1][0], pairs[0][1], pairs[1][1])
    ray = (((bbox[0]+bbox[2])/2-cx)/fx, ((bbox[1]+bbox[3])/2-cy)/fy, 1.)
    aa = sum(a*a for a in ray)
    bb = sum(a*b for a, b in zip(ray, centre))
    cc = sum(a*a for a in centre)-radius*radius
    surface_z = (bb-math.sqrt(bb*bb-aa*cc))/aa
    return bbox, surface_z, k


def test_naive_bbox_depth_can_misclassify_oversized_sphere():
    # Generic 78 mm fruit and 74 mm aperture: no target/entity truth is consulted.
    bbox, surface, k = _synthetic_sphere_observation((0., 0., .539), .039)
    diagnostic = estimate_sphere_diameter(bbox, surface, k, (480, 848))
    assert diagnostic.valid
    assert diagnostic.naive_bbox_extent_m < .074
    assert diagnostic.diameter_m == pytest.approx(.078, abs=1e-10)
    assert not diagnostic.bounds_verified
    decision = classify_aperture_interval(diagnostic.sensitivity_low_m,
        diagnostic.sensitivity_high_m, .074, bounds_verified=diagnostic.bounds_verified)
    assert decision.state == 'UNKNOWN'


@pytest.mark.parametrize('centre', [(0., 0., .4), (.10, -.07, .45), (-.16, .05, .6)])
def test_off_axis_complete_ideal_sphere_inversion(centre):
    bbox, surface, k = _synthetic_sphere_observation(centre, .031)
    result = estimate_sphere_diameter(bbox, surface, k, (480, 848))
    assert result.valid
    assert result.diameter_m == pytest.approx(.062, abs=1e-10)
    assert result.axis_ratio_mismatch < 1e-10
    assert result.sensitivity_low_m < result.diameter_m < result.sensitivity_high_m


def test_mad_and_pixel_error_expand_diagnostic_sensitivity_not_certification():
    bbox, surface, k = _synthetic_sphere_observation((.1, 0., .5), .033)
    narrow = estimate_sphere_diameter(bbox, surface, k, (480, 848))
    wider = estimate_sphere_diameter(bbox, surface, k, (480, 848), depth_mad_m=.01,
                                    uncertainty=SizeUncertainty(pixel_error_px=3.))
    assert wider.valid and wider.depth_sensitivity_m == pytest.approx(.03)
    assert wider.sensitivity_low_m < narrow.sensitivity_low_m
    assert wider.sensitivity_high_m > narrow.sensitivity_high_m
    assert not wider.bounds_verified


@pytest.mark.parametrize(('bbox', 'depth', 'k', 'shape'), [
    ((0., 200., 50., 250.), .5, (500., 490., 424., 240.), (480, 848)),
    ((400., 200., 500., 480.), .5, (500., 490., 424., 240.), (480, 848)),
    ((400., 200., 401., 201.), .5, (500., 490., 424., 240.), (480, 848)),
    ((400., 200., 450., 250.), 0., (500., 490., 424., 240.), (480, 848)),
    ((400., 200., 450., 250.), float('nan'), (500., 490., 424., 240.), (480, 848)),
    ((400., 200., 450., 250.), .5, (0., 490., 424., 240.), (480, 848)),
    ((400., 200., 450., 250.), .5, (500., 490., 424.), (480, 848)),
    ((400., 200., 450., 250.), .5, (500., 490., 424., 240.), (0, 848)),
])
def test_missing_invalid_or_clipped_size_evidence_is_not_valid(bbox, depth, k, shape):
    assert not estimate_sphere_diameter(bbox, depth, k, shape).valid


def test_an_elongated_or_partial_box_does_not_become_a_spherical_measurement():
    result = estimate_sphere_diameter((400., 200., 500., 230.), .5,
                                      (500., 490., 424., 240.), (480, 848))
    assert not result.valid
    assert result.reason == 'non_spherical_or_incomplete_silhouette'


def test_uncertainty_crossing_zero_depth_fails_closed():
    result = estimate_sphere_diameter((400., 200., 450., 250.), .02,
                                      (500., 490., 424., 240.), (480, 848), depth_mad_m=.01)
    assert not result.valid and result.reason == 'depth_uncertainty_crosses_zero'


@pytest.mark.parametrize(('lower', 'upper', 'verified', 'state'), [
    (.060, .065, False, 'UNKNOWN'),
    (.060, .065, True, 'ELIGIBLE'),
    (.077, .082, True, 'TOO_LARGE'),
    (.070, .074, True, 'UNKNOWN'),
    (.070, .072, True, 'ELIGIBLE'),
    (float('nan'), .065, True, 'UNKNOWN'),
    (.060, float('inf'), True, 'UNKNOWN'),
    (0., .065, True, 'UNKNOWN'),
    (.065, .060, True, 'UNKNOWN'),
])
def test_only_verified_size_upper_bound_with_clearance_can_admit(lower, upper, verified, state):
    result = classify_aperture_interval(lower, upper, .074, .002, bounds_verified=verified)
    assert result.state == state
    assert result.available_diameter_m == pytest.approx(.072)


@pytest.mark.parametrize(('aperture', 'margin'), [(0., .002), (.074, -.001), (.074, .074),
                                               (float('nan'), .002)])
def test_invalid_gripper_configuration_is_rejected(aperture, margin):
    with pytest.raises(ValueError):
        classify_aperture_interval(.06, .065, aperture, margin)


@pytest.mark.parametrize('uncertainty', [dict(pixel_error_px=-1.), dict(depth_floor_m=float('nan')),
                                       dict(focal_relative_error=1.)])
def test_invalid_uncertainty_configuration_is_rejected(uncertainty):
    with pytest.raises(ValueError):
        SizeUncertainty(**uncertainty)
