"""Geometric regressions on the actual 224-fruit world; no ROS/Gazebo actions."""

import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'artifacts/close_harvest_20261014/harvestable'
SDF = SOURCE / 'tomato_field_harvestable.sdf'
METADATA = SOURCE / 'tomato_field.json'
MESH = ROOT / 'sim_ws/src/agri_greenhouse_worlds/models/tomato_fruit/meshes/fruit.dae'
SPEC = importlib.util.spec_from_file_location('small_fruit', ROOT / 'scripts/prepare_small_fruit_candidate.py')
small = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(small)


def inputs():
    return ET.parse(SDF).getroot(), json.loads(METADATA.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_actual_224_fruits_preserve_placement_labels_leaves_and_native_config():
    original, source_metadata = inputs()
    hashes = {path: sha(path) for path in (SDF, METADATA, MESH)}
    converted, metadata, report = small.convert(original, source_metadata, MESH)
    assert len(source_metadata['fruits']) == len(metadata['fruits']) == 224
    assert report['ripe_count'] == 165
    assert report['all_original_poses_and_nonfruit_content_preserved']
    original_models = {item.get('name'): item for item in original.find('world').findall('model')}
    new_models = {item.get('name'): item for item in converted.find('world').findall('model')}
    assert original_models.keys() == new_models.keys()
    ids = {fruit['id'] for fruit in metadata['fruits']}
    # This checks every leaf, branch, stem collision, proxy visual and plant pose.
    for name in new_models.keys() - ids:
        assert small.signature(original_models[name]) == small.signature(new_models[name])
    native = json.loads(converted.find('world/plugin[@name="agri::HarvestSystem"]/config').text)
    assert len(native['fruits']) == 224
    for old, new, config in zip(source_metadata['fruits'], metadata['fruits'], native['fruits']):
        unchanged = {key: value for key, value in new.items() if key != 'radius'}
        assert unchanged == {key: value for key, value in old.items() if key != 'radius'}
        expected_diameter = .05 + (old['radius'] * 2 - .06) * (.01 / .03)
        assert new['radius'] == pytest.approx(expected_diameter / 2, abs=1e-15)
        assert config == {key: new[key] for key in ('id', 'plant_id', 'radius', 'ripe')}
        old_model, model = original_models[new['id']], new_models[new['id']]
        assert model.findtext('pose') == old_model.findtext('pose')
        assert small.signature(model.find('link/visual/material')) == small.signature(
            old_model.find('link/visual/material'))
        assert model.findtext('static') == 'false'
        radius = float(model.findtext('link/collision/geometry/sphere/radius'))
        scales = [float(item) for item in model.findtext('link/visual/geometry/mesh/scale').split()]
        assert radius == new['radius']
        assert scales == pytest.approx([radius*2]*3, abs=1e-15)
        factor = radius / old['radius']
        mass = float(model.findtext('link/inertial/mass'))
        old_mass = float(old_model.findtext('link/inertial/mass'))
        # Independently check density and all six inertial tensor entries.
        assert mass / radius**3 == pytest.approx(old_mass / old['radius']**3, rel=1e-12)
        assert mass == pytest.approx(old_mass * factor**3, rel=1e-12)
        for axis in ('ixx', 'iyy', 'izz'):
            assert float(model.findtext('link/inertial/inertia/'+axis)) == pytest.approx(
                .4 * mass * radius**2, rel=1e-12)
        for axis in ('ixy', 'ixz', 'iyz'):
            assert float(model.findtext('link/inertial/inertia/'+axis)) == 0
        assert .05 <= radius*2 <= .06
    comparison = copy.deepcopy(metadata)
    comparison.pop('fruit_size_profile')
    comparison['parameters'] = copy.deepcopy(source_metadata['parameters'])
    for item, old in zip(comparison['fruits'], source_metadata['fruits']):
        item['radius'] = old['radius']
    assert comparison == source_metadata
    assert metadata['seed'] == 42
    assert metadata['fruit_size_profile']['runtime_target_selection_source'] is False
    assert report['runtime_navigation_and_harvest_passed'] is False
    assert hashes == {path: sha(path) for path in hashes}


def test_all_24976_whole_sphere_gaps_increase_and_mesh_stays_inside_collision():
    root, metadata = inputs()
    _, candidate, report = small.convert(root, metadata, MESH)
    original_gaps, candidate_gaps = small.pair_gaps(metadata['fruits']), small.pair_gaps(candidate['fruits'])
    assert len(original_gaps) == 24976
    assert all(candidate_gaps[key] > value for key, value in original_gaps.items())
    assert report['mesh']['normalized_max_radius_m'] == pytest.approx(.5, abs=1e-7)
    # Scaling the normalized radius-0.5 mesh by the actual candidate diameter
    # encloses every original mesh vertex in the new physical sphere.
    assert all(report['mesh']['normalized_max_radius_m'] * fruit['radius'] * 2
               <= fruit['radius'] + 1e-8 for fruit in candidate['fruits'])
    first = next(item for item in candidate['fruits'] if item['id'] == 'tomato_20_fruit_2')
    assert first['radius'] * 2 == pytest.approx(.05606208853306143, abs=1e-12)
    # No ripe fruit is invented at the known empty harvest plant.
    assert not any(item['ripe'] for item in candidate['fruits'] if item['plant_id'] == 'tomato_35')


@pytest.mark.parametrize('value', [.05999, .09001, 0, float('nan'), float('inf')])
def test_out_of_original_diameter_domain_refuses_silent_clamp(value):
    with pytest.raises(ValueError, match='within'):
        small.resize_diameter(value)


def test_exact_endpoints():
    assert small.resize_diameter(.06) == .05
    assert small.resize_diameter(.09) == .06


@pytest.mark.parametrize('mismatch', ['pose', 'radius', 'mesh', 'native', 'inertia', 'duplicate'])
def test_mismatched_physical_and_metadata_state_is_rejected(mismatch):
    root, metadata = inputs()
    model = root.find('world/model[@name="tomato_0_fruit_0"]')
    if mismatch == 'pose':
        model.find('pose').text = '0 0 0 0 0 0'
    elif mismatch == 'radius':
        model.find('link/collision/geometry/sphere/radius').text = '.04'
    elif mismatch == 'mesh':
        model.find('link/visual/geometry/mesh/scale').text = '.01 .01 .01'
    elif mismatch == 'native':
        config = root.find('world/plugin[@name="agri::HarvestSystem"]/config')
        value = json.loads(config.text)
        value['fruits'][0]['ripe'] = not value['fruits'][0]['ripe']
        config.text = json.dumps(value)
    elif mismatch == 'inertia':
        model.find('link/inertial/inertia/ixx').text = '0'
    else:
        metadata['fruits'].append(copy.deepcopy(metadata['fruits'][0]))
    with pytest.raises(ValueError):
        small.convert(root, metadata, MESH)


def test_bundle_has_new_hash_provenance_and_existing_outputs_cannot_be_overwritten(tmp_path):
    destination = tmp_path / 'candidate'
    source_hashes = {path: sha(path) for path in (SDF, METADATA, MESH)}
    manifest = small.prepare(SDF, METADATA, destination, MESH)
    assert manifest['candidate']['sdf_sha256'] == sha(destination/'tomato_field_harvestable.sdf')
    assert manifest['candidate']['metadata_sha256'] == sha(destination/'tomato_field.json')
    assert manifest['source']['sdf_sha256'] == source_hashes[SDF]
    new_metadata = json.loads((destination/'tomato_field.json').read_text())
    assert new_metadata['fruit_size_profile']['provenance'] == manifest['source']
    saved = {path: sha(path) for path in destination.iterdir()}
    with pytest.raises(ValueError, match='overwrite'):
        small.prepare(SDF, METADATA, destination, MESH)
    assert saved == {path: sha(path) for path in destination.iterdir()}
    assert source_hashes == {path: sha(path) for path in source_hashes}


def test_source_directory_and_symlink_guards(tmp_path):
    with pytest.raises(ValueError, match='overwrite'):
        small.prepare(SDF, METADATA, SOURCE, MESH)
    with pytest.raises(ValueError, match='protected'):
        small.prepare(SDF, METADATA, ROOT/'sim_ws/src/../src/forbidden_candidate', MESH)
    alias = tmp_path/'alias'
    alias.symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError, match='symlink'):
        small.prepare(SDF, METADATA, alias/'candidate', MESH)
    with pytest.raises(ValueError, match='symlink'):
        small.prepare(alias/'source.sdf', METADATA, tmp_path/'candidate', MESH)


def test_failed_validation_leaves_no_partial_candidate(tmp_path):
    root, metadata = inputs()
    metadata['fruits'][0]['radius'] = .1
    invalid = tmp_path/'invalid.json'
    invalid.write_text(json.dumps(metadata))
    output = tmp_path/'candidate'
    with pytest.raises(ValueError):
        small.prepare(SDF, invalid, output, MESH)
    assert not output.exists()


@pytest.mark.parametrize('snapshot', ['state', 'attachment', 'native_state'])
def test_runtime_snapshot_cannot_be_relabelled_as_initial_scene(snapshot):
    root, metadata = inputs()
    if snapshot == 'state':
        ET.SubElement(root, 'state', world_name='field')
    elif snapshot == 'attachment':
        ET.SubElement(root.find('world'), 'joint', name='picked_fruit_attachment', type='fixed')
    else:
        node = root.find('world/plugin[@name="agri::HarvestSystem"]/config')
        config = json.loads(node.text)
        config['fruits'][0]['state'] = 'IN_BASKET'
        node.text = json.dumps(config)
    with pytest.raises(ValueError, match='state|attachment'):
        small.convert(root, metadata, MESH)


def test_source_mutation_during_output_rolls_back_own_outputs(tmp_path, monkeypatch):
    sdf, metadata = tmp_path/'source.sdf', tmp_path/'source.json'
    sdf.write_bytes(SDF.read_bytes())
    metadata.write_bytes(METADATA.read_bytes())
    output = tmp_path/'candidate'
    real_digest = small.digest
    calls = 0

    def changing_digest(path):
        nonlocal calls
        if Path(path) == sdf:
            calls += 1
            if calls == 3:  # Final verification after exclusive output creation.
                sdf.write_bytes(sdf.read_bytes()+b'\n')
        return real_digest(path)

    monkeypatch.setattr(small, 'digest', changing_digest)
    with pytest.raises(RuntimeError, match='source unexpectedly changed'):
        small.prepare(sdf, metadata, output, MESH)
    assert output.is_dir()
    assert list(output.iterdir()) == []
