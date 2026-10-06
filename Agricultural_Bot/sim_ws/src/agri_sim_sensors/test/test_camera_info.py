"""Verify the public calibration recovers Gazebo's actual pixel-centre rays."""

from copy import deepcopy
import math

import pytest
from sensor_msgs.msg import CameraInfo, RegionOfInterest

from agri_sim_sensors.normalize_camera_info import _validate_topics, normalize_camera_info


def camera_info():
    fx, fy, cx, cy = 446.802773119128, 432.971461265142, 424., 240.
    info = CameraInfo(
        width=848, height=480, distortion_model='plumb_bob',
        d=[0.001, -0.002, 0.003, -0.004, 0.005],
        k=[fx, 0., cx, 0., fy, cy, 0., 0., 1.],
        r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
        p=[fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.],
        binning_x=2, binning_y=3,
        roi=RegionOfInterest(x_offset=7, y_offset=13, width=200, height=100, do_rectify=True),
    )
    info.header.frame_id = 'camera_optical_frame'
    info.header.stamp.sec, info.header.stamp.nanosec = 12, 34567890
    return info


@pytest.mark.parametrize('u,v,z', [(424., 240., .3), (525., 164., 1.6), (0., 479., 2.)])
def test_integer_ros_backprojection_matches_actual_half_pixel_gazebo_sample(u, v, z):
    raw = camera_info()
    normalized = normalize_camera_info(raw)
    rendered_x = (u + .5 - raw.k[2]) / raw.k[0] * z
    rendered_y = (v + .5 - raw.k[5]) / raw.k[4] * z
    assert (u-normalized.k[2]) / normalized.k[0] * z == pytest.approx(rendered_x, abs=1e-12)
    assert (v-normalized.k[5]) / normalized.k[4] * z == pytest.approx(rendered_y, abs=1e-12)
    assert normalized.p[2] == normalized.k[2] == 423.5
    assert normalized.p[6] == normalized.k[5] == 239.5


def test_only_principal_points_change_without_mutating_or_aliasing_any_input_fields():
    original = camera_info()
    snapshot = deepcopy(original)
    expected = deepcopy(original)
    expected.k[2] -= .5
    expected.k[5] -= .5
    expected.p[2] -= .5
    expected.p[6] -= .5
    normalized = normalize_camera_info(original)
    assert original == snapshot
    assert normalized == expected
    # Header/ROI/arrays must also be independent after the callback publishes.
    normalized.header.stamp.sec = 99
    normalized.roi.x_offset = 99
    normalized.d[0] = 99.
    normalized.r[0] = 99.
    assert original == snapshot


def test_zero_offset_disables_conversion_and_returns_independent_metadata():
    original = camera_info()
    normalized = normalize_camera_info(original, 0.)
    assert normalized == original
    assert normalized is not original
    normalized.k[2] = 123.
    normalized.header.frame_id = 'changed'
    assert original.k[2] == 424.
    assert original.header.frame_id == 'camera_optical_frame'


@pytest.mark.parametrize('offset', [math.nan, math.inf, -math.inf, -0.01, 1.01, True, '0.5'])
def test_invalid_pixel_center_offsets_are_rejected(offset):
    with pytest.raises(ValueError, match='pixel_center_offset'):
        normalize_camera_info(camera_info(), offset)


@pytest.mark.parametrize('field,index,value', [
    ('k', 0, 0.), ('k', 4, -1.), ('p', 0, math.nan), ('p', 5, math.inf),
    ('k', 2, math.nan), ('p', 6, math.inf), ('k', 2, .25), ('p', 2, 849.),
])
def test_uncalibrated_nonfinite_or_outside_image_calibration_is_rejected(field, index, value):
    info = camera_info()
    getattr(info, field)[index] = value
    with pytest.raises(ValueError):
        normalize_camera_info(info)


@pytest.mark.parametrize('dimension', ['width', 'height'])
def test_invalid_image_resolution_is_rejected(dimension):
    info = camera_info()
    setattr(info, dimension, 0)
    with pytest.raises(ValueError, match='resolution'):
        normalize_camera_info(info)


@pytest.mark.parametrize('input_topic,output_topic', [
    ('', '/out'), ('/in', ''), ('  ', '/out'), ('/in', '/in'), (None, '/out'),
])
def test_topic_feedback_or_empty_topics_are_rejected(input_topic, output_topic):
    with pytest.raises(ValueError):
        _validate_topics(input_topic, output_topic)
