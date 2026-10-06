"""Test range-aware fixtures and metric RGB-D geometry without a ROS graph."""

from array import array
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image


sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
from check_d405_scene import evaluate, scene_depth_range, scene_fixtures  # noqa: E402
from setup_d405_validation_scene import FIXTURES, build_fixtures, write_fixture  # noqa: E402


def metadata(max_depth):
    return {'depth_contract_m': [0.07, max_depth], 'fixtures': build_fixtures(max_depth)}


def render_scene(max_depth, rendered_max_depth=None):
    """Ray trace known sphere/box surfaces to exercise independent backprojection."""
    fixtures = build_fixtures(max_depth)
    width, height, fx, fy, cx, cy = 848, 480, 446.802773119128, 432.971461265142, 424., 240.
    rows, columns = np.indices((height, width))
    rays = np.stack(((columns-cx)/fx, (rows-cy)/fy, np.ones_like(columns)), axis=-1)
    nearest = np.full((height, width), np.inf)
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    for fixture in fixtures:
        center = np.array(fixture['center'])
        if fixture['shape'] == 'sphere':
            a = np.sum(rays*rays, axis=-1)
            b = rays @ center
            c = center @ center - fixture['radius']**2
            discriminant = b*b - a*c
            distance = (b - np.sqrt(np.maximum(discriminant, 0))) / a
            distance = np.where((discriminant >= 0) & (distance > 0), distance, np.inf)
        else:
            half = np.array(fixture['size'])/2
            with np.errstate(divide='ignore', invalid='ignore'):
                boundaries = np.stack(((center-half)/rays, (center+half)/rays))
            lower = np.max(np.min(boundaries, axis=0), axis=-1)
            upper = np.min(np.max(boundaries, axis=0), axis=-1)
            distance = np.where((lower <= upper) & (lower > 0), lower, np.inf)
        front = distance < nearest
        nearest[front] = distance[front]
        rgb[front] = np.array(fixture['rgb']) * 255
    upper_limit = max_depth if rendered_max_depth is None else rendered_max_depth
    depth = np.where(nearest < 0.07, -np.inf,
                     np.where(nearest > upper_limit, np.inf, nearest)).astype('<f4')
    color = Image(height=height, width=width, encoding='rgb8', step=width*3,
                  data=array('B', rgb.tobytes()))
    depth_message = Image(height=height, width=width, encoding='32FC1', step=width*4,
                          data=array('B', depth.tobytes()))
    info = CameraInfo(width=width, height=height, distortion_model='plumb_bob', d=[0.]*5,
                      k=[fx, 0., cx, 0., fy, cy, 0., 0., 1.],
                      r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                      p=[fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.])
    for message in (color, depth_message, info):
        message.header.frame_id = 'camera_optical_frame'
        message.header.stamp.sec = 1
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0., z=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    pose = (np.zeros(3), np.eye(3))
    return (color, depth_message, info), transform, pose, {item['name']: item for item in fixtures}


def test_default_fixture_geometry_stays_identical(tmp_path):
    assert build_fixtures() == FIXTURES
    path = tmp_path/'baseline.sdf'
    write_fixture(path, np.zeros(3), np.eye(3))
    assert len(ET.parse(path).findall('model/link/visual')) == 5
    assert 'magenta_sphere' not in scene_fixtures(metadata(0.5), (0.07, 0.5))


@pytest.mark.parametrize('max_depth', [2., 5.])
def test_far_targets_track_range_and_keep_visible_angular_size(max_depth, tmp_path):
    fixtures = scene_fixtures(metadata(max_depth), (0.07, max_depth))
    far, sphere = fixtures['blue_far'], fixtures['magenta_sphere']
    assert far['center'][2] - far['size'][2]/2 > max_depth
    assert sphere['center'][2] == pytest.approx(0.8*max_depth)
    assert sphere['center'][2] - sphere['radius'] > 0.5
    assert sphere['center'][2] + sphere['radius'] < max_depth
    assert 446.8*sphere['radius']/sphere['center'][2] > 17
    path = tmp_path/'extended.sdf'
    write_fixture(path, np.zeros(3), np.eye(3), list(fixtures.values()))
    assert len(ET.parse(path).findall('model/link/visual')) == 6


@pytest.mark.parametrize('max_depth', [float('nan'), float('inf'), -1., 0., 0.49, True])
def test_invalid_fixture_maximum_is_rejected(max_depth):
    with pytest.raises(ValueError):
        build_fixtures(max_depth)


def test_scene_range_uses_metadata_and_checks_explicit_argument():
    assert scene_depth_range({}) == (0.07, 0.5)  # Historical baseline artifacts.
    assert scene_depth_range(metadata(2.)) == (0.07, 2.)
    assert scene_depth_range(metadata(2.), 2.) == (0.07, 2.)
    with pytest.raises(ValueError, match='does not match'):
        scene_depth_range(metadata(2.), 0.5)


@pytest.mark.parametrize('interval', [None, [], [0.07], [0.07, 2, 3], [0., 2.],
                                     [2., 0.07], [0.07, float('nan')],
                                     [0.07, float('inf')], [True, 2.], [0.07, '2']])
def test_scene_range_rejects_invalid_contract(interval):
    with pytest.raises(ValueError):
        scene_depth_range({'depth_contract_m': interval})


@pytest.mark.parametrize('requested', [float('nan'), float('inf'), 0., -1., True])
def test_scene_range_rejects_invalid_explicit_maximum(requested):
    with pytest.raises(ValueError):
        scene_depth_range(metadata(2.), requested)


@pytest.mark.parametrize('corruption', ['duplicate', 'missing_extended', 'old_far',
                                       'negative_radius', 'nonfinite_center'])
def test_scene_range_rejects_inconsistent_geometry(corruption):
    scene = metadata(2.)
    if corruption == 'duplicate':
        scene['fixtures'].append(deepcopy(scene['fixtures'][0]))
    elif corruption == 'missing_extended':
        scene['fixtures'] = [item for item in scene['fixtures'] if item['name'] != 'magenta_sphere']
    else:
        indexed = {item['name']: item for item in scene['fixtures']}
        if corruption == 'old_far':
            indexed['blue_far']['center'][2] = 0.70
        elif corruption == 'negative_radius':
            indexed['magenta_sphere']['radius'] = -0.1
        else:
            indexed['red_sphere']['center'][0] = float('nan')
    with pytest.raises(ValueError):
        scene_fixtures(scene, (0.07, 2.))


@pytest.mark.parametrize('max_depth', [0.5, 2.])
def test_metric_backprojection_and_clipping_pass_for_baseline_and_extended(max_depth):
    group, transform, pose, fixtures = render_scene(max_depth)
    report, _, _ = evaluate(group, transform, pose, pose, fixtures, depth_range=(0.07, max_depth))
    assert report['passed'], report['errors']
    assert report['targets']['blue_far']['valid_depth_fraction'] == 0
    assert report['targets']['yellow_near']['valid_depth_fraction'] == 0
    if max_depth > 0.5:
        extended = report['targets']['magenta_sphere']
        assert extended['core_pixels'] > 500
        assert extended['min_depth_m'] > 0.5
        assert extended['p95_radial_error_m'] < 0.002


def test_extended_scene_fails_when_sensor_still_clips_at_original_limit():
    group, transform, pose, fixtures = render_scene(2., rendered_max_depth=0.5)
    report, _, _ = evaluate(group, transform, pose, pose, fixtures, depth_range=(0.07, 2.))
    assert not report['passed']
    assert any('magenta_sphere' in error for error in report['errors'])


def test_scene_reports_out_of_range_and_finite_negative_depth():
    group, transform, pose, fixtures = render_scene(2.)
    depth = np.frombuffer(group[1].data, dtype='<f4').copy()
    depth[:2] = [2.1, -0.1]
    group[1].data = array('B', depth.tobytes())
    report, _, _ = evaluate(group, transform, pose, pose, fixtures, depth_range=(0.07, 2.))
    assert not report['passed']
    assert report['depth']['range_violations'] == 1
    assert report['depth']['negative_values'] == 1
