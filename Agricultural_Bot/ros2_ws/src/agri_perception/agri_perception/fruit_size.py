"""ROS-independent image/depth size diagnostics and conservative aperture admission.

An RGB detection box and central-neighbourhood depth do not certify the diameter
of an occluded tomato. The numerical estimate below assumes a complete, tight
spherical silhouette and depth on the box-centre ray. Its sensitivity interval
is NOT a guaranteed size bound: MAD is spread, not a bound on systematic error.
In particular, the current tomato visual mesh differs from its collision sphere.
Do not pass this diagnostic interval as verified aperture evidence.

The module consumes no field metadata, simulation entity identity, or fruit
position/size truth. Runtime users must preserve acquisition stamp, optical
frame and original detection_index when attaching results to observations.
"""

from dataclasses import dataclass
from itertools import product
import math


@dataclass(frozen=True)
class SizeUncertainty:
    pixel_error_px: float = 1.5
    depth_floor_m: float = 0.003
    depth_mad_multiplier: float = 3.0
    focal_relative_error: float = 0.01
    maximum_axis_ratio_mismatch: float = 0.2

    def __post_init__(self):
        values = (self.pixel_error_px, self.depth_floor_m, self.depth_mad_multiplier,
                  self.focal_relative_error, self.maximum_axis_ratio_mismatch)
        if not all(math.isfinite(v) and v >= 0 for v in values):
            raise ValueError('size uncertainties must be finite and nonnegative')
        if self.focal_relative_error >= 1 or self.maximum_axis_ratio_mismatch >= 1:
            raise ValueError('relative size uncertainties must be less than one')


@dataclass(frozen=True)
class SphereSizeDiagnostic:
    valid: bool
    reason: str
    diameter_m: float = 0.0
    sensitivity_low_m: float = 0.0
    sensitivity_high_m: float = 0.0
    naive_bbox_extent_m: float = 0.0
    depth_sensitivity_m: float = 0.0
    axis_ratio_mismatch: float = 0.0
    bounds_verified: bool = False


@dataclass(frozen=True)
class ApertureDecision:
    state: str  # ELIGIBLE, TOO_LARGE, UNKNOWN
    reason: str
    available_diameter_m: float


def _sphere_hypothesis(bbox, depth, intrinsics):
    """Invert tangent planes and a box-centre-ray surface intersection.

    Works off the principal axis as well. Near the principal axis this reduces
    to diameter = 2*z*sin(alpha)/(1-sin(alpha)), rather than bbox_px*z/focal.
    """
    xmin, ymin, xmax, ymax = bbox
    fx, fy, cx, cy = intrinsics
    ratios, directions = [], []
    for lo, hi, focal, principal in ((xmin, xmax, fx, cx), (ymin, ymax, fy, cy)):
        a, b = math.atan((lo-principal)/focal), math.atan((hi-principal)/focal)
        centre, half_angle = (a+b)/2, (b-a)/2
        directions.append(math.tan(centre))
        ratios.append(math.sin(half_angle)/math.cos(centre))
    q = sum(ratios)/2  # Hypothesized radius / centre optical Z.
    mismatch = abs(ratios[0]-ratios[1])/max(ratios)
    centre_ray = (directions[0], directions[1], 1.)
    observed_ray = (((xmin+xmax)/2-cx)/fx, ((ymin+ymax)/2-cy)/fy, 1.)
    aa = sum(v*v for v in observed_ray)
    bb = sum(v*w for v, w in zip(observed_ray, centre_ray))
    cc = sum(v*v for v in centre_ray)-q*q
    discriminant = bb*bb-aa*cc
    if discriminant < -1e-12 or not 0 < q < 1:
        raise ValueError('sphere_hypothesis_invalid')
    surface_ratio = (bb-math.sqrt(max(0., discriminant)))/aa
    if surface_ratio <= 0:
        raise ValueError('sphere_hypothesis_invalid')
    return 2*q*depth/surface_ratio, mismatch


def estimate_sphere_diameter(bbox, depth_m, intrinsics, image_shape,
                             depth_mad_m=0., uncertainty=SizeUncertainty()):
    """Return a diagnostic only; incomplete contour cannot be auto-admitted.

    `bbox` is original un-eroded full RGB xyxy, `intrinsics` is fx/fy/cx/cy,
    `image_shape` is height/width. Inputs must refer to the same synchronized
    observation. The existing eroded segmentation remains useful for depth.
    Sensitivity samples vary box expansion, depth and each focal independently;
    they do not claim a statistical confidence interval or proven error bound.
    """
    try:
        box, k = tuple(map(float, bbox)), tuple(map(float, intrinsics))
        height, width = image_shape
        values = (*box, *k, depth_m, depth_mad_m, height, width)
        if (len(box) != 4 or len(k) != 4 or not all(math.isfinite(v) for v in values)
                or min(k[:2]) <= 0 or depth_m <= 0 or depth_mad_m < 0
                or min(height, width) <= 0):
            raise ValueError('invalid_size_inputs')
        xmin, ymin, xmax, ymax = box
        if xmax <= xmin or ymax <= ymin:
            raise ValueError('invalid_bbox')
        margin = uncertainty.pixel_error_px
        if not (margin < xmin < xmax < width-margin and margin < ymin < ymax < height-margin):
            raise ValueError('bbox_border_or_clipping')
        if min(xmax-xmin, ymax-ymin) <= 2*margin:
            raise ValueError('bbox_too_small_for_uncertainty')
        diameter, mismatch = _sphere_hypothesis(box, depth_m, k)
        if mismatch > uncertainty.maximum_axis_ratio_mismatch:
            raise ValueError('non_spherical_or_incomplete_silhouette')
        depth_error = max(uncertainty.depth_floor_m,
                          uncertainty.depth_mad_multiplier*depth_mad_m)
        if depth_m-depth_error <= 0:
            raise ValueError('depth_uncertainty_crosses_zero')
        sensitive = [diameter]
        for direction, fx_scale, fy_scale, dz in product((-1., 1.), repeat=4):
            expansion = direction*margin
            varied_box = (xmin-expansion, ymin-expansion, xmax+expansion, ymax+expansion)
            varied_k = (k[0]*(1+fx_scale*uncertainty.focal_relative_error),
                        k[1]*(1+fy_scale*uncertainty.focal_relative_error), k[2], k[3])
            value, _ = _sphere_hypothesis(varied_box, depth_m+dz*depth_error, varied_k)
            sensitive.append(value)
        naive = max((xmax-xmin)*depth_m/k[0], (ymax-ymin)*depth_m/k[1])
        return SphereSizeDiagnostic(True, 'ideal_sphere_diagnostic_not_verified_bounds',
                                    diameter, min(sensitive), max(sensitive), naive,
                                    depth_error, mismatch)
    except (ValueError, TypeError, OverflowError, ZeroDivisionError) as exc:
        return SphereSizeDiagnostic(False, str(exc))


def classify_aperture_interval(lower_m, upper_m, aperture_m, clearance_m=0.002,
                               *, bounds_verified=False):
    """Admit only independently verified, same-observation diameter bounds.

    `bounds_verified` must remain false for the box/median-depth diagnostic
    above. A future caller needs calibrated visual-to-gripper geometry,
    validated uncertainty coverage and complete contour/multi-view evidence.
    An interval crossing the usable aperture is UNKNOWN, never ELIGIBLE.
    """
    if not (math.isfinite(aperture_m) and math.isfinite(clearance_m)
            and aperture_m > 0 and 0 <= clearance_m < aperture_m):
        raise ValueError('invalid aperture or clearance')
    usable = aperture_m-clearance_m
    if not bounds_verified:
        return ApertureDecision('UNKNOWN', 'diameter_bounds_unverified', usable)
    if not (math.isfinite(lower_m) and math.isfinite(upper_m) and 0 < lower_m <= upper_m):
        return ApertureDecision('UNKNOWN', 'invalid_diameter_bounds', usable)
    if lower_m > usable:
        return ApertureDecision('TOO_LARGE', 'verified_lower_bound_exceeds_usable_aperture', usable)
    if upper_m <= usable:
        return ApertureDecision('ELIGIBLE', 'verified_upper_bound_within_usable_aperture', usable)
    return ApertureDecision('UNKNOWN', 'diameter_interval_crosses_usable_aperture', usable)
