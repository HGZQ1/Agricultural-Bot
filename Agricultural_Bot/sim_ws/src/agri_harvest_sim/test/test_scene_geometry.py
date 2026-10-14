import importlib.util
import math
from pathlib import Path

import pytest

SCRIPT = Path(__file__).parents[1] / 'scripts'
spec = importlib.util.spec_from_file_location('geometry', SCRIPT/'scene_geometry.py')
geometry = importlib.util.module_from_spec(spec); spec.loader.exec_module(geometry)


def test_actual_base_pose_avoids_map_alignment_assumption():
    pose = {'xyz': [4, -3, 0.4], 'xyzw': [0, 0, math.sqrt(.5), math.sqrt(.5)]}
    assert geometry.inverse_point(pose, [4, -2, 1.4]) == pytest.approx([1, 0, 1])


def test_matching_accepts_depth_surface_and_rejects_other_plant_and_unripe():
    snapshot = {'base_pose': {'xyz': [0,0,0], 'xyzw': [0,0,0,1]}, 'fruits': [
        {'entity_id': 'fruit', 'plant_id': 'p', 'ripe': True, 'state': 'ON_PLANT',
         'radius': .03, 'xyz': [.5, 0, 1]},
    ]}
    assert geometry.match_observation(snapshot, [.47, 0, 1], 'p')['entity_id'] == 'fruit'
    with pytest.raises(ValueError): geometry.match_observation(snapshot, [.47, 0, 1], 'other')
    snapshot['fruits'][0]['ripe'] = False
    with pytest.raises(ValueError): geometry.match_observation(snapshot, [.47, 0, 1], 'p')


def test_ambiguous_or_picked_fruit_is_not_silently_selected():
    a = {'entity_id': 'a', 'plant_id': 'p', 'ripe': True, 'state': 'ON_PLANT',
         'radius': .03, 'xyz': [.5, .032, 1]}
    b = {**a, 'entity_id': 'b', 'xyz': [.5, -.032, 1]}
    snapshot = {'base_pose': {'xyz': [0,0,0], 'xyzw': [0,0,0,1]}, 'fruits': [a,b]}
    with pytest.raises(ValueError, match='ambiguous'): geometry.match_observation(snapshot, [.47, 0, 1], 'p')
    a['state'] = b['state'] = 'IN_BASKET'
    with pytest.raises(ValueError): geometry.match_observation(snapshot, [.47, 0, 1], 'p')
