"""Geometry, occupancy preservation and source guards for the SIM blueprint."""

import copy
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('collision_navigation',
                                            ROOT/'scripts/prepare_collision_navigation.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
BACKUP = ROOT/'artifacts/harvest_demo_profile_20261013/previous_default'
SDF = BACKUP/'tomato_field_default.sdf'
METADATA = BACKUP/'tomato_field_default.json'
MISSION = BACKUP/'tomato_field_default_single_corridor.yaml'
MAP = ROOT/'ros2_ws/src/agri_navigation/maps/tomato_field_default.yaml'
MODELS = ROOT/'sim_ws/src/agri_greenhouse_worlds/models'


def test_pgm_binary_header_does_not_eat_whitespace_or_hash_pixels(tmp_path):
    image = tmp_path/'pixels.pgm'
    image.write_bytes(b'P5\r\n# comment\r\n4 1\r\n255\r\n'+bytes([10, 35, 13, 32]))
    assert module.read_pgm(image).tolist() == [[10, 35, 13, 32]]
    module.write_pgm(image, np.array([[10, 35, 13, 32]], dtype=np.uint8))
    assert module.read_pgm(image).tolist() == [[10, 35, 13, 32]]


def test_pgm_ascii_comments_and_payload_validation(tmp_path):
    image = tmp_path/'ascii.pgm'
    image.write_bytes(b'P2\n2 2\n255\n0 205 # pixels\n254 255\n')
    assert module.read_pgm(image).tolist() == [[0, 205], [254, 255]]
    image.write_bytes(b'P5\n2 2\n255\n'+bytes([0, 205]))
    with pytest.raises(ValueError, match='payload size'):
        module.read_pgm(image)


def test_small_fruit_circle_marks_intersected_cells_even_without_cell_centers_inside():
    mask = np.zeros((3, 3), dtype=bool)
    module.raster_circle(mask, [.95, .95], .10)
    assert mask[2, 0] and mask[2, 1] and mask[1, 0] and mask[1, 1]
    assert mask.sum() == 4


def test_stem_polygon_marks_corner_intersections_and_preserves_image_y_direction():
    mask = np.zeros((3, 3), dtype=bool)
    module.raster_polygon(mask, [[.9, .9], [1.1, .9], [1.1, 1.1], [.9, 1.1]])
    assert np.array_equal(mask, [[False, False, False],
                                 [True, True, False], [True, True, False]])


def test_world_to_map_and_rotated_grid_origin():
    grid = {'resolution': .05, 'origin': [-.401, -2.058, 0.]}
    point = module.world_to_grid([[-1.5, -5]], grid, [6., 0., -np.pi/2])[0]
    assert point == pytest.approx([(1.+.401)/.05, (1.5+2.058)/.05])
    grid = {'resolution': .5, 'origin': [1., 2., np.pi/2]}
    assert module.world_to_grid([[1., 3.]], grid, [0., 0., 0.])[0] == pytest.approx([2., 0.])


def test_collision_projection_uses_whole_box_with_pose_and_rejects_mesh():
    collision = ET.fromstring('<collision><pose>1 2 3 0 0 0.7853981633974483</pose>'
                              '<geometry><box><size>0.06 0.06 2</size></box></geometry></collision>')
    footprint, detail = module.collision_footprint(collision, np.eye(4))
    assert footprint[0] == 'polygon'
    assert detail['world_center_xyz'] == [1., 2., 3.]
    assert np.max(footprint[1][:, 0]) == pytest.approx(1.+.03*np.sqrt(2))
    assert detail['world_z_min_m'] == 2.
    unsupported = ET.fromstring('<collision><geometry><mesh><uri>unknown.stl</uri>'
                                '</mesh></geometry></collision>')
    with pytest.raises(ValueError, match='unsupported physical collision'):
        module.collision_footprint(unsupported, np.eye(4))


def test_only_known_foliage_is_erased_and_nonplant_obstacles_are_protected():
    pixels = np.array([[0, 0, 205, 0, 254, 254]], dtype=np.uint8)
    grid = {'pixels': pixels, 'description': {'negate': 0},
            'known': np.array([[True, True, False, True, True, True]])}
    masks = {'foliage': np.array([[True, True, True, False, True, False]]),
             'physical': np.array([[False, True, True, False, False, True]]),
             'nonplant': np.array([[False, True, False, False, False, False]])}
    result = module.collision_grid(grid, masks)
    assert result.tolist() == [[255, 0, 205, 0, 255, 0]]


def test_negated_map_preserves_unknown_and_paints_correct_black_semantics():
    grid = {'pixels': np.array([[255, 127, 0]], dtype=np.uint8),
            'description': {'negate': 1}, 'known': np.array([[True, False, True]])}
    masks = {'foliage': np.array([[True, True, False]]),
             'physical': np.array([[False, True, True]]),
             'nonplant': np.zeros((1, 3), dtype=bool)}
    assert module.collision_grid(grid, masks).tolist() == [[0, 127, 255]]


def test_output_alias_guards_fail_before_writing(tmp_path):
    original = tmp_path/'original.json'
    original.write_text('KEEP')
    output = tmp_path/'candidate'
    output.mkdir()
    (output/'tomato_field.json').symlink_to(original)
    with pytest.raises(ValueError, match='protected source or symlink'):
        module.guard_outputs(output, [original])
    assert original.read_text() == 'KEEP'
    (tmp_path/'alias').symlink_to(output, target_is_directory=True)
    with pytest.raises(ValueError, match='traverse symlinks'):
        module.guard_outputs(tmp_path/'alias', [])
    with pytest.raises(ValueError, match='protected source or symlink'):
        module.guard_outputs(tmp_path/'protected', [], [tmp_path/'protected'])
    assert not (tmp_path/'protected').exists()


@pytest.mark.parametrize('change', ['field_hash', 'image_hash', 'origin', 'alignment'])
def test_mismatched_registration_is_rejected(change):
    mission = yaml.safe_load(MISSION.read_text())
    grid = module.load_map(MAP)
    requested = None
    if change == 'field_hash':
        mission['field']['sdf_sha256'] = '0'*64
    elif change == 'image_hash':
        mission['map']['image_sha256'] = '0'*64
    elif change == 'origin':
        mission['map']['origin'][0] += .1
    else:
        requested = [6., 0., 0.]
    with pytest.raises(ValueError):
        module.validate_registration(mission, SDF, METADATA, grid, requested)


def test_real_field_preserves_original_fruit_collision_and_visual_geometry(tmp_path):
    protected = [SDF, METADATA, MISSION, MAP, MAP.with_suffix('.pgm')]
    before_hashes = {str(path): module.digest(path) for path in protected}
    manifest = module.prepare(SDF, METADATA, MAP, MISSION, tmp_path/'candidate', MODELS)
    assert manifest['plant_count'] == 60 and manifest['fruit_count'] == 224
    assert manifest['fruit_subtrees_preserved'] and manifest['physical_collisions_preserved']
    assert manifest['metadata_byte_identical']
    assert (tmp_path/'candidate/tomato_field.json').read_bytes() == METADATA.read_bytes()
    source = ET.parse(SDF).getroot()
    marked = ET.parse(manifest['world']).getroot()
    fruit_ids = {fruit['id'] for fruit in json.loads(METADATA.read_text())['fruits']}
    def normalized(element):
        element = copy.deepcopy(element)
        for child in element.iter():
            child.text = child.text.strip() if child.text else None
            child.tail = None
        return ET.tostring(element)
    original_models = {model.get('name'): model for model in source.find('world').findall('model')}
    marked_models = {model.get('name'): model for model in marked.find('world').findall('model')}
    for identifier in fruit_ids:
        assert normalized(original_models[identifier]) == normalized(marked_models[identifier])
    for before, after in zip(source.iter('collision'), marked.iter('collision')):
        assert normalized(before) == normalized(after)
    assert len(list(source.iter('collision'))) == len(list(marked.iter('collision')))
    assert len(manifest['soft_visuals']) == 180
    assert len(manifest['navigation_proxies']) == 60
    for record in manifest['soft_visuals']:
        visual = marked_models[record['model']].find(
            f"link[@name='{record['link']}']/visual[@name='{record['visual']}']")
        assert int(visual.findtext('visibility_flags')) == 4
        original = original_models[record['model']].find(
            f"link[@name='{record['link']}']/visual[@name='{record['visual']}']")
        visual.remove(visual.find('visibility_flags'))
        assert normalized(visual) == normalized(original)
    for record in manifest['navigation_proxies']:
        link = marked_models[record['model']].find(f"link[@name='{record['link']}']")
        collision = link.find(f"collision[@name='{record['collision']}']")
        proxy = link.find(f"visual[@name='{record['visual']}']")
        assert normalized(proxy.find('geometry')) == normalized(collision.find('geometry'))
        assert normalized(proxy.find('pose')) == normalized(collision.find('pose'))
        assert int(proxy.findtext('visibility_flags')) == 0x20000000
    old = module.load_map(MAP)
    new = module.load_map(manifest['navigation_map_yaml'])
    foliage = module.read_pgm(tmp_path/'candidate/foliage_envelope_mask.pgm') == 255
    physical = module.read_pgm(tmp_path/'candidate/physical_collision_mask.pgm') == 255
    assert new['pixels'].shape == old['pixels'].shape
    assert new['origin'] == old['origin'] and new['resolution'] == old['resolution']
    assert np.array_equal(new['pixels'][~old['known']], old['pixels'][~old['known']])
    assert np.array_equal(new['pixels'][~foliage & ~physical], old['pixels'][~foliage & ~physical])
    assert np.all(new['pixels'][old['known'] & physical] == 0)
    assert np.array_equal(new['pixels'][~foliage & old['occupied']], old['pixels'][~foliage & old['occupied']])
    assert before_hashes == {str(path): module.digest(path) for path in protected}
