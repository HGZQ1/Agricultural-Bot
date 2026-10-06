"""Reject an inverted camera even when its rendering and TF agree."""

from pathlib import Path
import sys

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
from check_d405_scene import assess_upright  # noqa: E402
from setup_d405_validation_scene import world_up_fixture_rotation  # noqa: E402


UPRIGHT = np.array([[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]])


def reference_masks():
    red, green = np.zeros((9, 9), dtype=bool), np.zeros((9, 9), dtype=bool)
    red[1:3, 1:3] = True
    green[6:8, 6:8] = True
    return {'red_sphere': red, 'green_sphere': green}


def test_fixture_uses_world_up_instead_of_copying_camera_roll():
    inverted = UPRIGHT @ np.diag([-1., -1., 1.])
    fixture = world_up_fixture_rotation(UPRIGHT[:, 2])
    np.testing.assert_allclose(fixture, world_up_fixture_rotation(inverted[:, 2]))
    np.testing.assert_allclose(fixture, UPRIGHT)
    assert np.linalg.det(fixture) == pytest.approx(1)
    assert fixture[2, 1] == -1  # fixture down stays down for both cameras


@pytest.mark.parametrize('forward', [[1, 2, -0.2], [-2, 1, 0.1]])
def test_fixture_follows_only_forward_with_a_positive_world_up(forward):
    fixture = world_up_fixture_rotation(forward)
    np.testing.assert_allclose(fixture.T @ fixture, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(fixture[:, 2], np.array(forward) / np.linalg.norm(forward))
    assert -fixture[2, 1] > 0.99
    assert np.linalg.det(fixture) == pytest.approx(1)


@pytest.mark.parametrize('flip_image,flip_tf', [(False, False), (True, False),
                                               (False, True), (True, True)])
def test_upright_detects_image_tf_and_consistent_combined_inversion(flip_image, flip_tf):
    masks = reference_masks()
    if flip_image:
        masks = {name: np.flip(mask) for name, mask in masks.items()}
    orientation = UPRIGHT @ np.diag([-1., -1., 1.]) if flip_tf else UPRIGHT
    report = assess_upright(masks, orientation, (4, 4))
    assert report['passed'] == (not flip_image and not flip_tf)
    if flip_tf:
        assert report['image_up_dot_world_up'] == -1
    if flip_image:
        assert any('World-upper' in error for error in report['errors'])
        assert any('World-left' in error for error in report['errors'])


def test_upright_cannot_pass_without_a_reference_target():
    masks = reference_masks()
    masks['red_sphere'][:] = False
    report = assess_upright(masks, UPRIGHT, (4, 4))
    assert not report['passed']
    assert any('missing' in error for error in report['errors'])


@pytest.mark.parametrize('forward', [[0, 0, 1], [0, 0, -1], [0, 0, 0], [np.nan, 0, 1]])
def test_fixture_rejects_undefined_world_up(forward):
    with pytest.raises(ValueError):
        world_up_fixture_rotation(forward)
