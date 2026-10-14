#!/usr/bin/env python3
"""Resize existing simulated fruit, preserving their original placement and identity.

This offline tool never samples a random generator or changes the default scene.
The input must be an initial harvestable world with independent spherical fruit.
It creates a new bundle; it neither executes harvesting nor selects a visual target.
"""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE_MIN_M, SOURCE_MAX_M = .060, .090
TARGET_MIN_M, TARGET_MAX_M = .050, .060
PROFILE_ID = 'preserved_positions_fruit_diameter_50_60_mm'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resize_diameter(diameter):
    """Map the declared original range, refusing out-of-range/invalid fruit."""
    if not math.isfinite(diameter) or not SOURCE_MIN_M <= diameter <= SOURCE_MAX_M:
        raise ValueError('source fruit diameter must be finite and within [0.060, 0.090] m')
    return TARGET_MIN_M + (diameter - SOURCE_MIN_M) * (
        (TARGET_MAX_M - TARGET_MIN_M) / (SOURCE_MAX_M - SOURCE_MIN_M))


def signature(element):
    """Compare XML content without serialization indentation differences."""
    return (element.tag, tuple(sorted(element.attrib.items())),
            (element.text or '').strip(), tuple(signature(child) for child in element))


def finite_values(text, size, description):
    try:
        values = tuple(float(value) for value in text.split())
    except (ValueError, AttributeError) as error:
        raise ValueError(f'invalid {description}') from error
    if len(values) != size or not all(math.isfinite(value) for value in values):
        raise ValueError(f'invalid {description}')
    return values


def mesh_bound(mesh_file):
    """Verify the unchanged DAE has identity transforms and radius 0.5 metres."""
    root = ET.parse(mesh_file).getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    unit = root.find('c:asset/c:unit', ns)
    if unit is None or float(unit.get('meter')) != 1.:
        raise ValueError('fruit mesh units must be metres')
    nodes = root.findall('.//c:visual_scene//c:node', ns)
    if not nodes:
        raise ValueError('fruit mesh has no visual scene')
    identity = (1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.)
    for node in nodes:
        for transform in node:
            kind = transform.tag.split('}')[-1]
            if kind == 'matrix' and finite_values(transform.text, 16, 'mesh matrix') != identity:
                raise ValueError('nonidentity fruit mesh transform is unsupported')
            if kind in ('translate', 'rotate', 'scale', 'lookat', 'skew'):
                raise ValueError('fruit mesh transform is unsupported')
    positions = root.findall('.//c:geometry/c:mesh/c:source[@id="fruit-positions"]', ns)
    if len(positions) != 1:
        raise ValueError('expected one normalized fruit position source')
    values = finite_values(positions[0].find('c:float_array', ns).text,
                           int(positions[0].find('c:float_array', ns).get('count')),
                           'mesh positions')
    if len(values) % 3:
        raise ValueError('invalid mesh position stride')
    vertices = [values[index:index+3] for index in range(0, len(values), 3)]
    maximum = max(math.sqrt(sum(axis*axis for axis in vertex)) for vertex in vertices)
    if not math.isclose(maximum, .5, abs_tol=1e-7):
        raise ValueError('fruit mesh must be normalized to radius 0.5')
    return {'mesh_sha256': digest(mesh_file), 'vertices': len(vertices),
            'normalized_max_radius_m': maximum, 'node_transforms_identity': True}


def pair_gaps(fruits):
    """Initial whole-sphere clearances; shrinking cannot create a new overlap."""
    gaps = {}
    for index, fruit in enumerate(fruits):
        for other in fruits[index+1:]:
            distance = math.sqrt(sum((fruit[axis] - other[axis])**2 for axis in ('x', 'y', 'z')))
            gaps[(fruit['id'], other['id'])] = distance - fruit['radius'] - other['radius']
    return gaps


def convert(source_root, source_metadata, mesh_file):
    """Validate the initial physical world and resize only its existing fruit."""
    root, metadata = copy.deepcopy(source_root), copy.deepcopy(source_metadata)
    if 'fruit_size_profile' in metadata:
        raise ValueError('refusing to resize a previously resized candidate')
    parameters = metadata['parameters']
    if (parameters.get('fruit_diameter_min'), parameters.get('fruit_diameter_max')) != (
            SOURCE_MIN_M, SOURCE_MAX_M):
        raise ValueError('metadata must declare original [0.060, 0.090] m diameter range')
    fruits = metadata['fruits']
    if not fruits:
        raise ValueError('no fruit to resize')
    ids = [fruit['id'] for fruit in fruits]
    if len(set(ids)) != len(ids):
        raise ValueError('duplicate metadata fruit IDs')
    world = root.find('world')
    if world is None:
        raise ValueError('expected one world')
    if root.findall('.//state') or world.findall('joint'):
        raise ValueError('runtime state or world attachment joints are not an initial scene')
    model_list = world.findall('model')
    models = {model.get('name'): model for model in model_list}
    if len(models) != len(model_list):
        raise ValueError('duplicate model names')
    physical_ids = {model.get('name') for model in model_list
                    if model.find('link/collision[@name="fruit_collision"]') is not None}
    if physical_ids != set(ids):
        raise ValueError('metadata fruit IDs differ from physical fruit models')
    plugins = world.findall('plugin[@name="agri::HarvestSystem"]')
    if len(plugins) != 1 or plugins[0].find('config') is None:
        raise ValueError('expected exactly one native HarvestSystem configuration')
    config_node = plugins[0].find('config')
    config = json.loads(config_node.text)
    if set(config) != {'robot_model', 'service', 'tcp_offset', 'fruits'}:
        raise ValueError('unsupported native configuration/state fields')
    config_fruits = config['fruits']
    if any(set(fruit) != {'id', 'plant_id', 'radius', 'ripe'} for fruit in config_fruits):
        raise ValueError('unsupported native fruit configuration/state fields')
    if [fruit['id'] for fruit in config_fruits] != ids:
        raise ValueError('native fruit configuration differs from metadata order/identity')
    mesh_report = mesh_bound(mesh_file)
    changes = []
    permitted = []
    for fruit, native in zip(fruits, config_fruits):
        fruit_id = fruit['id']
        if any(native[key] != fruit[key] for key in ('id', 'plant_id', 'radius', 'ripe')):
            raise ValueError(f'native fruit configuration mismatch: {fruit_id}')
        model = models[fruit_id]
        if model.findtext('static') != 'false' or len(model.findall('link')) != 1:
            raise ValueError(f'expected initial independent dynamic fruit: {fruit_id}')
        if model.findall('joint') or model.findall('plugin'):
            raise ValueError('fruit-local runtime attachment or plugin is unsupported')
        pose = finite_values(model.findtext('pose'), 6, f'{fruit_id} pose')
        if pose[:3] != tuple(fruit[axis] for axis in ('x', 'y', 'z')) or pose[3:] != (0., 0., 0.):
            raise ValueError(f'fruit is displaced from its initial metadata pose: {fruit_id}')
        link = model.find('link')
        if link.get('name') != 'fruit_link' or len(link.findall('collision')) != 1:
            raise ValueError(f'unsupported fruit link/collision: {fruit_id}')
        # The original world has all these local origins at identity. Refuse
        # offsets rather than shrinking a visual around a different centre.
        collision = link.find('collision[@name="fruit_collision"]')
        visual = link.find('visual[@name="fruit_visual"]')
        inertial = link.find('inertial')
        if collision is None or visual is None or inertial is None:
            raise ValueError(f'missing fruit geometry/dynamics: {fruit_id}')
        if len(link.findall('visual')) != 1:
            raise ValueError(f'unsupported extra fruit visuals: {fruit_id}')
        for element in (link, collision, visual, inertial):
            if element.find('pose') is not None and finite_values(
                    element.findtext('pose'), 6, 'local pose') != (0.,)*6:
                raise ValueError('nonidentity fruit local pose is unsupported')
        radius_node = collision.find('geometry/sphere/radius')
        mesh = visual.find('geometry/mesh')
        if radius_node is None or mesh is None or mesh.findtext('uri') != (
                'model://tomato_fruit/meshes/fruit.dae'):
            raise ValueError(f'unsupported physical/visual fruit shape: {fruit_id}')
        old_radius = float(radius_node.text)
        if not math.isfinite(old_radius) or old_radius != fruit['radius']:
            raise ValueError(f'collision radius differs from metadata: {fruit_id}')
        old_diameter = old_radius * 2
        new_diameter = resize_diameter(old_diameter)
        scale = new_diameter / old_diameter
        scale_node = mesh.find('scale')
        old_scales = finite_values(scale_node.text if scale_node is not None else '', 3,
                                  'fruit mesh scale')
        if any(not math.isclose(axis, old_diameter, abs_tol=1e-12) for axis in old_scales):
            raise ValueError(f'mesh scale differs from physical fruit diameter: {fruit_id}')
        mass_node = inertial.find('mass')
        old_mass = float(mass_node.text) if mass_node is not None else float('nan')
        inertia = inertial.find('inertia')
        if not math.isfinite(old_mass) or old_mass <= 0 or inertia is None:
            raise ValueError(f'invalid fruit mass/inertia: {fruit_id}')
        moment = .4 * old_mass * old_radius**2
        for axis in ('ixx', 'iyy', 'izz'):
            if not math.isclose(float(inertia.findtext(axis)), moment, rel_tol=1e-10):
                raise ValueError(f'fruit inertia is not the declared sphere: {fruit_id}')
        if any(float(inertia.findtext(axis)) != 0. for axis in ('ixy', 'ixz', 'iyz')):
            raise ValueError(f'fruit products of inertia must be zero: {fruit_id}')
        radius_node.text = repr(new_diameter / 2)
        scale_node.text = ' '.join(repr(axis * scale) for axis in old_scales)
        mass_node.text = repr(old_mass * scale**3)
        for axis in ('ixx', 'iyy', 'izz'):
            inertia.find(axis).text = repr(float(inertia.findtext(axis)) * scale**5)
        fruit['radius'] = new_diameter / 2
        native['radius'] = fruit['radius']
        changes.append({'id': fruit_id, 'old_diameter_m': old_diameter,
                        'new_diameter_m': new_diameter, 'scale': scale,
                        'old_mass_kg': old_mass, 'new_mass_kg': float(mass_node.text)})
        permitted.append((fruit_id, old_radius, old_scales, old_mass, moment))
    config_node.text = json.dumps(config, separators=(',', ':'))
    # Restore only permitted changes in a comparison copy. This proves all
    # world/plant/leaf/fruit poses, attachments, contact parameters and labels
    # survive the conversion, rather than testing a list of assumed fields.
    comparison = copy.deepcopy(root)
    comparison_models = {model.get('name'): model for model in comparison.find('world').findall('model')}
    original_models = {model.get('name'): model for model in source_root.find('world').findall('model')}
    for fruit_id, *_ in permitted:
        model = comparison_models[fruit_id]
        original = original_models[fruit_id]
        for path in ('link/collision/geometry/sphere/radius', 'link/visual/geometry/mesh/scale',
                     'link/inertial/mass', 'link/inertial/inertia/ixx',
                     'link/inertial/inertia/iyy', 'link/inertial/inertia/izz'):
            model.find(path).text = original.find(path).text
    comparison.find('world/plugin[@name="agri::HarvestSystem"]/config').text = (
        source_root.find('world/plugin[@name="agri::HarvestSystem"]/config').text)
    if signature(comparison) != signature(source_root):
        raise ValueError('conversion changed content outside permitted fruit size/dynamics')
    old_gaps, new_gaps = pair_gaps(source_metadata['fruits']), pair_gaps(fruits)
    if any(new_gaps[key] + 1e-12 < gap for key, gap in old_gaps.items()):
        raise ValueError('conversion reduced fruit-pair clearance')
    parameters['fruit_diameter_min'], parameters['fruit_diameter_max'] = TARGET_MIN_M, TARGET_MAX_M
    metadata['fruit_size_profile'] = {
        'id': PROFILE_ID, 'operation': 'deterministic_existing_fruit_resize',
        'source_diameter_range_m': [SOURCE_MIN_M, SOURCE_MAX_M],
        'target_diameter_range_m': [TARGET_MIN_M, TARGET_MAX_M],
        'formula': '0.050 + (old_diameter_m - 0.060) * (0.010 / 0.030)',
        'mass_model': 'preserve_each_original_fruit_density: mass*scale^3, inertia*scale^5',
        'placement_sampling': 'none; original seed records retained without rerunning RNG',
        'positions_ids_plant_ids_ripe_order_preserved': True,
        'runtime_target_selection_source': False,
        'original_parameters': copy.deepcopy(source_metadata['parameters']),
    }
    report = {
        'fruit_count': len(fruits), 'ripe_count': sum(fruit['ripe'] for fruit in fruits),
        'original_diameter_range_actual_m': [min(x['old_diameter_m'] for x in changes),
                                              max(x['old_diameter_m'] for x in changes)],
        'candidate_diameter_range_actual_m': [min(x['new_diameter_m'] for x in changes),
                                               max(x['new_diameter_m'] for x in changes)],
        'mesh': mesh_report, 'all_original_poses_and_nonfruit_content_preserved': True,
        'native_harvest_radius_matches_collision_and_metadata': True,
        'each_new_physical_sphere_is_a_concentric_subset_of_original': True,
        'clearance_to_unchanged_stems_and_other_obstacles_cannot_decrease': True,
        'all_pair_clearances_nondecreasing': True, 'pair_count': len(old_gaps),
        'original_min_fruit_pair_gap_m': min(old_gaps.values(), default=None),
        'candidate_min_fruit_pair_gap_m': min(new_gaps.values(), default=None),
        'fruits': changes,
        'runtime_navigation_and_harvest_passed': False,
        'all_plants_have_ripe_fruit': False if any(
            not any(fruit['ripe'] and fruit['plant_id'] == plant['id'] for fruit in fruits)
            for plant in metadata['plants']) else True,
    }
    return root, metadata, report


def reject_symlink(path):
    if any(item.is_symlink() for item in (path, *path.parents)):
        raise ValueError('source/output paths must not contain symlinks')


def prepare(source_sdf, source_metadata, output_dir, mesh_file):
    source_sdf, source_metadata, output_dir, mesh_file = (
        Path(path).absolute() for path in (source_sdf, source_metadata, output_dir, mesh_file))
    for path in (source_sdf, source_metadata, output_dir, mesh_file):
        reject_symlink(path)
    source_sdf, source_metadata, output_dir, mesh_file = (
        path.resolve() for path in (source_sdf, source_metadata, output_dir, mesh_file))
    if any(output_dir.is_relative_to(path) for path in (
            ROOT / 'sim_ws/src', ROOT / 'ros2_ws/src', ROOT / 'scripts', ROOT / 'docs')):
        raise ValueError('output cannot be in protected source/default directories')
    targets = [output_dir / name for name in (
        'tomato_field_harvestable.sdf', 'tomato_field.json', 'manifest_small_fruit.json')]
    if any(path.exists() or path in (source_sdf, source_metadata, mesh_file) for path in targets):
        raise ValueError('refusing to overwrite an existing/source output')
    source_hashes = {'sdf_sha256': digest(source_sdf), 'metadata_sha256': digest(source_metadata)}
    root, metadata, report = convert(
        ET.parse(source_sdf).getroot(), json.loads(source_metadata.read_text()), mesh_file)
    provenance = {'source_sdf': str(source_sdf), 'source_metadata': str(source_metadata),
                  **source_hashes}
    metadata['fruit_size_profile']['provenance'] = provenance
    ET.indent(root, space='  ')
    sdf_bytes = ET.tostring(root, encoding='utf-8', xml_declaration=True) + b'\n'
    metadata_bytes = (json.dumps(metadata, ensure_ascii=False, indent=2) + '\n').encode()
    manifest = {'schema_version': 1, 'profile_id': PROFILE_ID, 'source': provenance,
                'candidate': {'sdf': str(targets[0]), 'metadata': str(targets[1]),
                              'sdf_sha256': hashlib.sha256(sdf_bytes).hexdigest(),
                              'metadata_sha256': hashlib.sha256(metadata_bytes).hexdigest()},
                'geometry_regressions': report,
                'scope': 'independent simulation candidate; default/current-running assets unchanged',
                'map_and_mission_rebinding_required': True,
                'runtime_target_selection_uses_metadata': False}
    contents = [sdf_bytes, metadata_bytes, (json.dumps(manifest, indent=2) + '\n').encode()]
    # No destination is changed until all 224 fruits and preservation checks pass.
    # Exclusive creation prevents replacing sources or a concurrent candidate.
    current_hashes = lambda: {'sdf_sha256': digest(source_sdf),
                              'metadata_sha256': digest(source_metadata)}
    if source_hashes != current_hashes():
        raise RuntimeError('source unexpectedly changed while preparing candidate')
    output_dir.mkdir(parents=True, exist_ok=True)
    written = []
    try:
        for path, content in zip(targets, contents):
            with path.open('xb') as stream:
                stream.write(content)
            written.append(path)
        if source_hashes != current_hashes():
            raise RuntimeError('source unexpectedly changed while preparing candidate')
    except Exception:
        for path in written:
            path.unlink()
        raise
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    base = ROOT / 'artifacts/close_harvest_20261014'
    parser.add_argument('--source-sdf', type=Path, default=base/'harvestable/tomato_field_harvestable.sdf')
    parser.add_argument('--metadata', type=Path, default=base/'harvestable/tomato_field.json')
    parser.add_argument('--output-dir', type=Path, default=base/'small_fruit_50_60_candidate')
    parser.add_argument('--mesh', type=Path, default=(
        ROOT/'sim_ws/src/agri_greenhouse_worlds/models/tomato_fruit/meshes/fruit.dae'))
    args = parser.parse_args()
    try:
        result = prepare(args.source_sdf, args.metadata, args.output_dir, args.mesh)
    except (ValueError, OSError, KeyError, TypeError, ET.ParseError) as error:
        parser.error(str(error))
    print(json.dumps({**result['candidate'],
                      'fruit_count': result['geometry_regressions']['fruit_count'],
                      'diameter_range_m': result['geometry_regressions']['candidate_diameter_range_actual_m']},
                     indent=2))


if __name__ == '__main__':
    main()
