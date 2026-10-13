"""ROS-independent checks for RGB-D observations and short-lived target tracks.

The sampled point estimates a visible surface depth.  It is not a fruit centre,
TCP pose, IK result, or persistent identity across camera sessions.
"""

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np


@dataclass(frozen=True)
class DepthConfig:
    sample_count: int = 81
    radius_px: float = 8.0
    bbox_radius_fraction: float = 0.25
    minimum_m: float = 0.07
    maximum_m: float = 2.0
    minimum_samples: int = 15
    minimum_valid_fraction: float = 0.6
    maximum_mad_m: float = 0.03
    require_segmentation: bool = False

    def __post_init__(self):
        if self.sample_count < 1 or not 1 <= self.minimum_samples <= self.sample_count:
            raise ValueError('depth sample counts must be positive and consistent')
        if not (math.isfinite(self.radius_px) and self.radius_px > 0):
            raise ValueError('depth radius must be finite and positive')
        if not 0 < self.bbox_radius_fraction <= 0.5:
            raise ValueError('bbox_radius_fraction must be in (0, 0.5]')
        if not (0 < self.minimum_m < self.maximum_m < math.inf):
            raise ValueError('depth range must be finite, positive and increasing')
        if not 0 < self.minimum_valid_fraction <= 1:
            raise ValueError('minimum_valid_fraction must be in (0, 1]')
        if not (math.isfinite(self.maximum_mad_m) and self.maximum_mad_m >= 0):
            raise ValueError('maximum_mad_m must be finite and nonnegative')


@dataclass(frozen=True)
class DepthEstimate:
    valid: bool
    reason: str
    depth_m: float = 0.0
    sample_count: int = 0
    valid_fraction: float = 0.0
    mad_m: float = 0.0
    segmentation_used: bool = False


def observation_time_error(rgb_ns, depth_ns, now_ns, skew_s=0.05, age_s=0.5):
    """Use capture stamps, including positive time and future-data rejection."""
    if rgb_ns <= 0 or depth_ns <= 0 or now_ns <= 0:
        return 'zero_timestamp'
    if abs(rgb_ns - depth_ns) > round(skew_s * 1e9):
        return 'rgb_depth_timestamp_skew'
    if rgb_ns > now_ns or depth_ns > now_ns:
        return 'future_observation'
    if now_ns - min(rgb_ns, depth_ns) > round(age_s * 1e9):
        return 'expired_observation'
    return ''


def sample_depth(depth_m, bbox, rng, config=DepthConfig(), mask=None):
    """Randomly sample distinct central-neighbourhood pixels and take median.

    With a segmentation mask the eligible disk is restricted to mask pixels.
    ``sample_count`` in the result is the number of range-valid sampled pixels;
    valid_fraction uses all sampled pixels, including NaN/zero/out-of-range.
    """
    depth = np.asarray(depth_m)
    if depth.ndim != 2:
        return DepthEstimate(False, 'invalid_depth_shape')
    coordinates = np.asarray(bbox, dtype=float)
    if coordinates.shape != (4,) or not np.isfinite(coordinates).all():
        return DepthEstimate(False, 'invalid_bbox')
    xmin, ymin, xmax, ymax = coordinates
    if xmax <= xmin or ymax <= ymin:
        return DepthEstimate(False, 'invalid_bbox')
    u, v = (xmin + xmax) / 2, (ymin + ymax) / 2
    height, width = depth.shape
    if not (0 <= u < width and 0 <= v < height):
        return DepthEstimate(False, 'bbox_center_outside_image')
    segmentation_used = mask is not None
    if mask is not None and np.asarray(mask).shape != depth.shape:
        return DepthEstimate(False, 'segmentation_shape_mismatch')
    if config.require_segmentation and mask is None:
        return DepthEstimate(False, 'segmentation_required')
    radius = min(config.radius_px,
                 (xmax - xmin) * config.bbox_radius_fraction,
                 (ymax - ymin) * config.bbox_radius_fraction)
    x0, x1 = max(0, math.ceil(u-radius)), min(width-1, math.floor(u+radius))
    y0, y1 = max(0, math.ceil(v-radius)), min(height-1, math.floor(v+radius))
    yy, xx = np.mgrid[y0:y1+1, x0:x1+1]
    eligible = ((xx-u)**2 + (yy-v)**2 <= radius**2)
    eligible &= (xx >= xmin) & (xx <= xmax) & (yy >= ymin) & (yy <= ymax)
    if mask is not None:
        eligible &= np.asarray(mask, dtype=bool)[y0:y1+1, x0:x1+1]
    candidates = np.flatnonzero(eligible)
    if len(candidates) < config.minimum_samples:
        return DepthEstimate(False, 'insufficient_sampling_pixels',
                             segmentation_used=segmentation_used)
    chosen = rng.choice(candidates, min(config.sample_count, len(candidates)),
                        replace=False)
    values = depth[yy.ravel()[chosen], xx.ravel()[chosen]]
    valid = values[np.isfinite(values) & (values >= config.minimum_m)
                   & (values <= config.maximum_m)]
    fraction = len(valid) / len(values)
    if len(valid) < config.minimum_samples or fraction < config.minimum_valid_fraction:
        return DepthEstimate(False, 'insufficient_valid_depth',
                             sample_count=len(valid), valid_fraction=fraction,
                             segmentation_used=segmentation_used)
    median = float(np.median(valid))
    mad = float(np.median(np.abs(valid-median)))
    if mad > config.maximum_mad_m:
        return DepthEstimate(False, 'depth_dispersion', median,
                             len(valid), fraction, mad, segmentation_used)
    return DepthEstimate(True, '', median, len(valid), fraction, mad,
                         segmentation_used)


@dataclass(frozen=True)
class Candidate:
    detection_index: int
    class_name: str
    confidence: float
    bbox: tuple
    position: tuple
    depth: DepthEstimate


def project_candidates(boxes, classes, confidences, names, depth_m, intrinsics,
                       seed, config=DepthConfig(), masks=None,
                       mature_only=True, mature_class='tomato_R'):
    """Keep each 3D point attached to its original class and detection index."""
    fx, fy, cx, cy = map(float, intrinsics)
    if not all(math.isfinite(n) for n in (fx, fy, cx, cy)) or min(fx, fy) <= 0:
        raise ValueError('invalid camera intrinsics')
    output, rejected = [], []
    for index, (bbox, class_id, confidence) in enumerate(zip(boxes, classes, confidences)):
        class_name = str(names[int(class_id)])
        if mature_only and class_name != mature_class:
            continue
        if not math.isfinite(float(confidence)) or not 0 <= float(confidence) <= 1:
            rejected.append((index, 'invalid_confidence'))
            continue
        mask = None if masks is None or index >= len(masks) else masks[index]
        estimate = sample_depth(depth_m, bbox,
                                np.random.default_rng(np.random.SeedSequence([seed, index])),
                                config, mask)
        if not estimate.valid:
            rejected.append((index, estimate.reason))
            continue
        xmin, ymin, xmax, ymax = map(float, bbox)
        u, v = (xmin+xmax)/2, (ymin+ymax)/2
        z = estimate.depth_m
        output.append(Candidate(index, class_name, float(confidence), tuple(bbox),
                                ((u-cx)*z/fx, (v-cy)*z/fy, z), estimate))
    return output, rejected


def _iou(a, b):
    width = max(0.0, min(a[2], b[2])-max(a[0], b[0]))
    height = max(0.0, min(a[3], b[3])-max(a[1], b[1]))
    intersection = width*height
    union = ((a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection)
    return intersection/union if union > 0 else 0.0


@dataclass
class _Track:
    target_id: str
    class_name: str
    bbox: tuple
    observations: deque = field(default_factory=deque)


class TargetTracker:
    """Conservative geometric tracking, scoped to one camera/node session."""

    def __init__(self, prefix, minimum_frames=3, window_s=0.5,
                 maximum_spread_m=0.025, match_distance_m=0.08,
                 minimum_span_s=0.1):
        if minimum_frames < 2 or not 0 <= minimum_span_s < window_s < math.inf:
            raise ValueError('invalid temporal stability window')
        if not (0 < maximum_spread_m <= match_distance_m < math.inf):
            raise ValueError('invalid tracking distances')
        self.prefix = prefix
        self.minimum_frames = minimum_frames
        self.window_ns = round(window_s*1e9)
        self.minimum_span_ns = round(minimum_span_s*1e9)
        self.maximum_spread_m = maximum_spread_m
        self.match_distance_m = match_distance_m
        self._tracks = {}
        self._counter = 0
        self._last_stamp = 0

    def clear(self):
        self._tracks.clear()
        self._last_stamp = 0

    def update(self, candidates, stamp_ns):
        if stamp_ns <= self._last_stamp:
            self.clear()
        self._last_stamp = stamp_ns
        self._tracks = {key: track for key, track in self._tracks.items()
                        if stamp_ns-track.observations[-1][0] <= self.window_ns}
        associations = []
        for index, candidate in enumerate(candidates):
            for key, track in self._tracks.items():
                distance = np.linalg.norm(np.asarray(candidate.position)
                                          - np.asarray(track.observations[-1][1]))
                if (candidate.class_name == track.class_name
                        and distance <= self.match_distance_m
                        and _iou(candidate.bbox, track.bbox) >= 0.2):
                    associations.append((float(distance), index, key))
        assigned, used_tracks = {}, set()
        for _, index, key in sorted(associations):
            if index not in assigned and key not in used_tracks:
                assigned[index] = key
                used_tracks.add(key)
        # A missed detection restarts both the identity and stability claim.
        # This avoids reusing pre-occlusion history to authorize fresh work.
        self._tracks = {key: track for key, track in self._tracks.items()
                        if key in used_tracks}
        output = []
        for index, candidate in enumerate(candidates):
            key = assigned.get(index)
            if key is None:
                self._counter += 1
                key = f'{self.prefix}-{self._counter:06d}'
                self._tracks[key] = _Track(key, candidate.class_name, candidate.bbox)
            track = self._tracks[key]
            track.bbox = candidate.bbox
            track.observations.append((stamp_ns, candidate.position))
            while stamp_ns-track.observations[0][0] > self.window_ns:
                track.observations.popleft()
            # Large movement keeps identity briefly, but restarts stability.
            while len(track.observations) > 1:
                points = np.asarray([point for _, point in track.observations])
                spread = np.max(np.linalg.norm(points-points[-1], axis=1))
                if spread <= self.maximum_spread_m:
                    break
                track.observations.popleft()
            count = len(track.observations)
            stable = (count >= self.minimum_frames
                      and stamp_ns-track.observations[0][0] >= self.minimum_span_ns)
            output.append((candidate, key, stable, count))
        return output


class ImagePairBuffer:
    """Bounded nearest-stamp matcher; each message is used at most once."""

    def __init__(self, skew_s=0.05, age_s=0.5, capacity=10):
        if not (0 <= skew_s < age_s < math.inf) or capacity < 1:
            raise ValueError('invalid image pairing limits')
        self.skew_ns = round(skew_s*1e9)
        self.age_ns = round(age_s*1e9)
        self.capacity = capacity
        self.rgb, self.depth = [], []
        self.last_rgb_stamp = 0

    def clear(self):
        self.rgb.clear()
        self.depth.clear()
        self.last_rgb_stamp = 0

    def push(self, kind, stamp_ns, message, now_ns):
        queue = self.rgb if kind == 'rgb' else self.depth
        if stamp_ns <= 0 or stamp_ns > now_ns or now_ns-stamp_ns > self.age_ns:
            return None
        if kind == 'rgb' and stamp_ns <= self.last_rgb_stamp:
            return None
        if all(stamp != stamp_ns for stamp, _ in queue):
            queue.append((stamp_ns, message))
        self.rgb[:] = [(stamp, value) for stamp, value in self.rgb
                       if 0 <= now_ns-stamp <= self.age_ns and stamp > self.last_rgb_stamp][-self.capacity:]
        self.depth[:] = [(stamp, value) for stamp, value in self.depth
                         if 0 <= now_ns-stamp <= self.age_ns][-self.capacity:]
        matches = [(abs(rstamp-dstamp), rstamp, ri, di)
                   for ri, (rstamp, _) in enumerate(self.rgb)
                   for di, (dstamp, _) in enumerate(self.depth)
                   if abs(rstamp-dstamp) <= self.skew_ns]
        if not matches:
            return None
        _, _, ri, di = min(matches)
        rstamp, rgb = self.rgb.pop(ri)
        _, depth = self.depth.pop(di)
        self.last_rgb_stamp = rstamp
        self.rgb[:] = [(stamp, value) for stamp, value in self.rgb if stamp > rstamp]
        return rgb, depth
