#!/usr/bin/env python3
"""Prepare a separate, simulation-only navigation collision blueprint.

The localization map and source plant/fruit geometry remain unchanged. Soft
plant visuals are tagged for sensor masks; a navigation-only visual copies each
existing stem collision. The candidate occupancy grid keeps unknown pixels and
replaces known foliage-envelope pixels with conservative physical footprints.
This is a Gazebo scene blueprint, not semantic filtering of a real SLAM map.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import yaml


SOFT_VISIBILITY = 4
NAVIGATION_VISIBILITY = 0x20000000
SOFT_SUBMESHES = {'Branch1', 'Leaf1', 'Leaf2'}
EPSILON = 1e-9
OUTPUT_NAMES = (
    'tomato_field_collision_navigation.sdf', 'tomato_field.json',
    'navigation_collision.yaml', 'navigation_collision.pgm',
    'foliage_envelope_mask.pgm', 'physical_collision_mask.pgm',
    'nonplant_collision_mask.pgm', 'known_source_mask.pgm',
    'removed_leaf_occupancy_mask.pgm', 'manifest_collision_navigation.json',
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def numbers(text, count, name):
    try:
        result = [float(value) for value in text.split()]
    except (AttributeError, ValueError) as error:
        raise ValueError(f'{name}: expected {count} finite numbers') from error
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError(f'{name}: expected {count} finite numbers')
    return result


def read_pgm(path):
    """Read P2/P5, retaining every pixel (including whitespace-valued bytes)."""
    data = Path(path).read_bytes()
    cursor = 0

    def token():
        nonlocal cursor
        while cursor < len(data):
            if data[cursor] in b' \t\r\n':
                cursor += 1
            elif data[cursor] == ord('#'):
                end = data.find(b'\n', cursor)
                if end < 0:
                    raise ValueError('truncated PGM comment')
                cursor = end+1
            else:
                break
        start = cursor
        while cursor < len(data) and data[cursor] not in b' \t\r\n#':
            cursor += 1
        if start == cursor:
            raise ValueError('truncated PGM header or pixels')
        return data[start:cursor]

    magic = token()
    width, height, maximum = (int(token()) for _ in range(3))
    if magic not in (b'P2', b'P5') or width < 1 or height < 1 or maximum != 255:
        raise ValueError('expected P2/P5 PGM with positive dimensions and maxval 255')
    if magic == b'P5':
        if cursor >= len(data) or data[cursor] not in b' \t\r\n':
            raise ValueError('PGM binary header must terminate with whitespace')
        delimiter = data[cursor]
        cursor += 1
        if delimiter == ord('\r') and cursor < len(data) and data[cursor] == ord('\n'):
            cursor += 1
        payload = data[cursor:]
        if len(payload) != width*height:
            raise ValueError('PGM binary payload size does not match dimensions')
        pixels = np.frombuffer(payload, dtype=np.uint8).copy()
    else:
        values = [int(token()) for _ in range(width*height)]
        if any(not 0 <= value <= 255 for value in values):
            raise ValueError('PGM pixel is outside maxval')
        pixels = np.array(values, dtype=np.uint8)
    return pixels.reshape(height, width)


def write_pgm(path, pixels):
    pixels = np.asarray(pixels, dtype=np.uint8)
    height, width = pixels.shape
    Path(path).write_bytes(f'P5\n{width} {height}\n255\n'.encode()+pixels.tobytes())


def load_map(path):
    path = Path(path).resolve()
    description = yaml.safe_load(path.read_text())
    if not isinstance(description, dict) or description.get('mode', 'trinary') != 'trinary':
        raise ValueError('localization map must use trinary mode')
    image = description.get('image')
    if not isinstance(image, str) or not image:
        raise ValueError('map image path is missing')
    image_path = (path.parent/image).resolve()
    pixels = read_pgm(image_path)
    resolution = float(description['resolution'])
    origin = [float(value) for value in description['origin']]
    occupied, free = (float(description[key]) for key in ('occupied_thresh', 'free_thresh'))
    if (not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3 or
            not all(math.isfinite(value) for value in origin) or
            not 0 <= free < occupied <= 1 or description.get('negate', 0) not in (0, 1)):
        raise ValueError('invalid map geometry or thresholds')
    probability = pixels.astype(float)/255.
    if not description.get('negate', 0):
        probability = 1.-probability
    known = (probability > occupied) | (probability < free)
    return {'yaml': path, 'image': image_path, 'description': description,
            'pixels': pixels, 'known': known, 'occupied': probability > occupied,
            'resolution': resolution, 'origin': origin,
            'width': pixels.shape[1], 'height': pixels.shape[0]}


def validate_registration(mission, source_sdf, metadata_path, grid, requested=None):
    """Require the original mission to bind both the world and saved-map geometry."""
    field, binding = mission['field'], mission['map']
    if (field['sdf_sha256'] != digest(source_sdf) or
            field['metadata_sha256'] != digest(metadata_path)):
        raise ValueError('registration mission does not bind the source SDF/metadata')
    if binding['image_sha256'] != digest(grid['image']):
        raise ValueError('registration mission does not bind the localization image')
    if (binding['width_px'] != grid['width'] or binding['height_px'] != grid['height'] or
            not math.isclose(binding['resolution_m'], grid['resolution'], abs_tol=EPSILON) or
            not np.allclose(binding['origin'], grid['origin'], rtol=0, atol=EPSILON)):
        raise ValueError('localization map registration differs from source mission')
    for mission_key, yaml_key in (('mode', 'mode'), ('negate', 'negate'),
                                 ('occupied_thresh', 'occupied_thresh'),
                                 ('free_thresh', 'free_thresh')):
        if binding[mission_key] != grid['description'].get(yaml_key, 'trinary' if yaml_key == 'mode' else 0):
            raise ValueError(f'localization map {yaml_key} differs from source mission')
    alignment = [float(value) for value in mission['alignment']['world_to_map']]
    if len(alignment) != 3 or not all(math.isfinite(value) for value in alignment):
        raise ValueError('registration requires a finite world_to_map [x, y, yaw]')
    if requested is not None and not np.allclose(requested, alignment, rtol=0, atol=EPSILON):
        raise ValueError('requested world_to_map differs from source mission registration')
    return alignment


def pose_matrix(element):
    pose = element.find('pose')
    result = np.eye(4)
    if pose is None:
        return result
    if pose.attrib:
        raise ValueError('relative_to/quaternion SDF poses are unsupported; explicit parent poses required')
    x, y, z, roll, pitch, yaw = numbers(pose.text, 6, 'SDF pose')
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                       [-sp, cp*sr, cp*cr]]
    result[:3, 3] = [x, y, z]
    return result


def transform_points(points, matrix):
    points = np.asarray(points, dtype=float)
    return points@matrix[:3, :3].T+matrix[:3, 3]


def convex_hull(points):
    points = sorted(set(tuple(point) for point in points))
    if len(points) < 3:
        raise ValueError('projected polygon requires at least three distinct points')

    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])

    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return np.array(lower[:-1]+upper[:-1])


def world_to_grid(points, grid, alignment):
    """World XY to continuous image-bottom grid coordinates (cell edges)."""
    tx, ty, yaw = alignment
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    mapped = np.asarray(points)@rotation.T+[tx, ty]
    ox, oy, angle = grid['origin']
    inverse = np.array([[math.cos(angle), math.sin(angle)],
                        [-math.sin(angle), math.cos(angle)]])
    return (mapped-[ox, oy])@inverse.T/grid['resolution']


def raster_polygon(mask, polygon):
    """Fill every closed grid cell intersecting a convex polygon using SAT."""
    height, width = mask.shape
    polygon = convex_hull(polygon)
    edges = np.roll(polygon, -1, axis=0)-polygon
    axes = np.vstack((np.eye(2), np.column_stack((-edges[:, 1], edges[:, 0]))))
    projections = polygon@axes.T
    minimum, maximum = projections.min(axis=0), projections.max(axis=0)
    low = np.floor(polygon.min(axis=0)-EPSILON).astype(int)
    high = np.floor(polygon.max(axis=0)+EPSILON).astype(int)
    for row in range(max(0, low[1]), min(height-1, high[1])+1):
        for column in range(max(0, low[0]), min(width-1, high[0])+1):
            cell = np.array([[column, row], [column+1, row],
                             [column+1, row+1], [column, row+1]])@axes.T
            if np.all((cell.max(axis=0) >= minimum-EPSILON) &
                      (cell.min(axis=0) <= maximum+EPSILON)):
                mask[height-1-row, column] = True


def raster_circle(mask, center, radius):
    """Fill cells by circle/closed-square intersection, including small fruit."""
    height, width = mask.shape
    center = np.asarray(center)
    low = np.floor(center-radius-EPSILON).astype(int)
    high = np.floor(center+radius+EPSILON).astype(int)
    for row in range(max(0, low[1]), min(height-1, high[1])+1):
        for column in range(max(0, low[0]), min(width-1, high[0])+1):
            closest = np.maximum([column, row], np.minimum(center, [column+1, row+1]))
            if np.sum((center-closest)**2) <= radius*radius+EPSILON:
                mask[height-1-row, column] = True


def mesh_vertices(path, submesh):
    """Resolve COLLADA named scene submesh and its position accessor."""
    root = ET.parse(path).getroot()
    ns = {'c': root.tag.split('}')[0][1:]}
    if (root.findtext('c:asset/c:up_axis', namespaces=ns) != 'Z_UP' or
            float(root.find('c:asset/c:unit', ns).get('meter')) != 1.):
        raise ValueError('foliage mesh must be meter/Z_UP COLLADA')
    nodes = [node for node in root.findall('.//c:visual_scene//c:node', ns)
             if node.get('name') == submesh]
    if len(nodes) != 1:
        raise ValueError(f'expected one COLLADA node named {submesh}')
    node = nodes[0]
    transformations = [child for child in node
                       if child.tag.split('}')[-1] in ('matrix', 'translate', 'rotate', 'scale')]
    if any(child.tag.split('}')[-1] != 'matrix' or
           not np.allclose(np.array(numbers(child.text, 16, 'COLLADA matrix')).reshape(4, 4),
                           np.eye(4), rtol=0, atol=EPSILON) for child in transformations):
        raise ValueError('nonidentity COLLADA node transformations are unsupported')
    instances = node.findall('c:instance_geometry', ns)
    if len(instances) != 1:
        raise ValueError('COLLADA submesh needs one geometry instance')
    identifier = instances[0].get('url').removeprefix('#')
    geometry = root.find(f'c:library_geometries/c:geometry[@id="{identifier}"]/c:mesh', ns)
    vertex_input = geometry.find('c:vertices/c:input[@semantic="POSITION"]', ns)
    source_id = vertex_input.get('source').removeprefix('#')
    source = geometry.find(f'c:source[@id="{source_id}"]', ns)
    accessor = source.find('c:technique_common/c:accessor', ns)
    if int(accessor.get('stride')) != 3 or int(accessor.get('offset', '0')) != 0:
        raise ValueError('foliage COLLADA position accessor must have stride three and zero offset')
    values = numbers(source.findtext('c:float_array', namespaces=ns),
                     int(accessor.get('count'))*3, 'COLLADA positions')
    return np.array(values).reshape(-1, 3)


def collision_footprint(collision, matrix):
    geometry = collision.find('geometry')
    if geometry is None or len(geometry) != 1:
        raise ValueError('each collision needs exactly one geometry')
    shape = geometry[0]
    matrix = matrix@pose_matrix(collision)
    detail = {'kind': shape.tag, 'world_center_xyz': matrix[:3, 3].tolist()}
    if shape.tag == 'box':
        size = numbers(shape.findtext('size'), 3, 'collision box size')
        if min(size) <= 0:
            raise ValueError('collision box dimensions must be positive')
        corners = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5)
                            for z in (-.5, .5)])*size
        transformed = transform_points(corners, matrix)
        polygon = convex_hull(transformed[:, :2])
        detail.update(size_xyz_m=size, world_polygon_xy=polygon.tolist(),
                      world_z_min_m=float(transformed[:, 2].min()),
                      world_z_max_m=float(transformed[:, 2].max()))
        return ('polygon', polygon, None), detail
    if shape.tag in ('sphere', 'cylinder'):
        radius = float(shape.findtext('radius'))
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('collision radius must be finite and positive')
        if shape.tag == 'cylinder':
            length = float(shape.findtext('length'))
            if (not math.isfinite(length) or length <= 0 or
                    not np.allclose(np.abs(matrix[:3, 2]), [0., 0., 1.], atol=EPSILON)):
                raise ValueError('only positive vertical collision cylinders are supported')
            detail['length_m'] = length
        detail['radius_m'] = radius
        return ('circle', matrix[:2, 3], radius), detail
    raise ValueError(f'unsupported physical collision geometry: {shape.tag}')


def paint_footprint(mask, footprint, grid, alignment):
    kind, points, radius = footprint
    if kind == 'polygon':
        raster_polygon(mask, world_to_grid(points, grid, alignment))
    else:
        raster_circle(mask, world_to_grid([points], grid, alignment)[0], radius/grid['resolution'])


def marked_world_and_masks(source_root, metadata, grid, alignment, model_root):
    """Mark soft visuals while preserving every collision and fruit subtree."""
    root = copy.deepcopy(source_root)
    world = root.find('world')
    if world is None or world.findall('.//include') or world.findall('.//model/model'):
        raise ValueError('expanded, nonnested SDF world required; unresolved models are unsafe')
    plant_ids = {plant['id'] for plant in metadata['plants']}
    plants = {plant['id']: plant for plant in metadata['plants']}
    fruits = {fruit['id']: fruit for fruit in metadata['fruits']}
    if len(plant_ids) != len(metadata['plants']) or len(fruits) != len(metadata['fruits']):
        raise ValueError('metadata has duplicate plant or fruit IDs')
    models = world.findall('model')
    model_ids = {model.get('name') for model in models}
    if len(model_ids) != len(models) or not (plant_ids | set(fruits)) <= model_ids:
        raise ValueError('SDF model IDs are duplicate or do not cover metadata')
    foliage = np.zeros_like(grid['known'])
    physical, nonplant = np.zeros_like(foliage), np.zeros_like(foliage)
    records, envelope_records, tagged, proxies = [], [], [], []
    cache = {}
    for model in models:
        identifier = model.get('name')
        if model.findtext('static', 'false').strip().lower() != 'true':
            raise ValueError('collision blueprint source must contain static models only')
        matrix = pose_matrix(model)
        plant = identifier in plant_ids
        fruit = identifier in fruits
        if plant:
            expected = plants[identifier]
            expected_matrix = ET.Element('model')
            ET.SubElement(expected_matrix, 'pose').text = (
                f"{expected['x']} {expected['y']} 0 0 0 {expected['yaw']}")
            if not np.allclose(matrix, pose_matrix(expected_matrix), rtol=0, atol=EPSILON):
                raise ValueError(f'plant {identifier} SDF/metadata pose mismatch')
        if fruit:
            expected = fruits[identifier]
            if not np.allclose(matrix[:3, 3], [expected[k] for k in ('x', 'y', 'z')],
                               rtol=0, atol=EPSILON):
                raise ValueError(f'fruit {identifier} SDF/metadata position mismatch')
        envelope_points = []
        for link in model.findall('link'):
            link_matrix = matrix@pose_matrix(link)
            for collision in link.findall('collision'):
                footprint, detail = collision_footprint(collision, link_matrix)
                detail.update(model=identifier, link=link.get('name'), collision=collision.get('name'),
                              source='plant' if plant else 'fruit' if fruit else 'other')
                if identifier == 'ground_plane':
                    if detail['kind'] != 'box' or detail['world_z_max_m'] > EPSILON:
                        raise ValueError('excluded ground must be a box at or below world z=0')
                    detail['excluded'] = 'ground plane'
                    records.append(detail)
                    continue
                if fruit and (detail['kind'] != 'sphere' or not math.isclose(
                        detail['radius_m'], fruits[identifier]['radius'], abs_tol=EPSILON) or
                        not np.allclose(detail['world_center_xyz'],
                                        [fruits[identifier][k] for k in ('x', 'y', 'z')],
                                        rtol=0, atol=EPSILON)):
                    raise ValueError(f'fruit {identifier} SDF/metadata sphere mismatch')
                paint_footprint(physical, footprint, grid, alignment)
                if not plant and not fruit:
                    paint_footprint(nonplant, footprint, grid, alignment)
                records.append(detail)
                if plant:
                    proxy_name = 'navigation_collision_proxy_'+collision.get('name', '')
                    if link.find(f'visual[@name="{proxy_name}"]') is not None:
                        raise ValueError('source world already contains navigation collision proxies')
                    proxy = ET.SubElement(link, 'visual', {'name': proxy_name})
                    if collision.find('pose') is not None:
                        proxy.append(copy.deepcopy(collision.find('pose')))
                    proxy.append(copy.deepcopy(collision.find('geometry')))
                    ET.SubElement(proxy, 'visibility_flags').text = str(NAVIGATION_VISIBILITY)
                    ET.SubElement(proxy, 'cast_shadows').text = 'false'
                    proxies.append({'model': identifier, 'link': link.get('name'),
                                    'collision': collision.get('name'), 'visual': proxy_name})
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if not plant or mesh is None or mesh.findtext('submesh/name') not in SOFT_SUBMESHES:
                    continue
                if mesh.findtext('submesh/center', 'false').lower() != 'false':
                    raise ValueError('foliage submesh recentering is unsupported')
                uri, submesh = mesh.findtext('uri'), mesh.findtext('submesh/name')
                if not uri or not uri.startswith('model://'):
                    raise ValueError('foliage mesh must use a model:// resource')
                mesh_path = (model_root/uri.removeprefix('model://')).resolve()
                if not mesh_path.is_relative_to(model_root.resolve()):
                    raise ValueError('foliage mesh escapes the model resource directory')
                key = (mesh_path, submesh)
                if key not in cache:
                    cache[key] = mesh_vertices(mesh_path, submesh)
                scale = numbers(mesh.findtext('scale', '1 1 1'), 3, 'foliage mesh scale')
                if min(scale) <= 0:
                    raise ValueError('foliage scale must be positive')
                points = transform_points(cache[key]*scale, link_matrix@pose_matrix(visual))
                envelope_points.extend(points[:, :2].tolist())
                flags = visual.find('visibility_flags')
                if flags is None:
                    flags = ET.SubElement(visual, 'visibility_flags')
                flags.text = str(SOFT_VISIBILITY)
                tagged.append({'model': identifier, 'link': link.get('name'),
                               'visual': visual.get('name'), 'submesh': submesh,
                               'mesh': str(mesh_path), 'mesh_sha256': digest(mesh_path),
                               'mesh_scale': scale})
        if plant:
            if not envelope_points:
                raise ValueError(f'plant {identifier} has no supported foliage envelope')
            polygon = convex_hull(envelope_points)
            raster_polygon(foliage, world_to_grid(polygon, grid, alignment))
            envelope_records.append({'plant': identifier, 'world_polygon_xy': polygon.tolist(),
                                     'method': 'convex hull of unchanged branch/leaf mesh vertices'})
    return root, {'foliage': foliage, 'physical': physical, 'nonplant': nonplant}, {
        'collision_geometry': records, 'foliage_envelopes': envelope_records,
        'soft_visuals': tagged, 'navigation_proxies': proxies,
        'plant_count': len(plant_ids), 'fruit_count': len(fruits)}


def collision_grid(grid, masks):
    pixels = grid['pixels'].copy()
    free_pixel, occupied_pixel = ((0, 255) if grid['description'].get('negate', 0) else (255, 0))
    erase = grid['known'] & masks['foliage'] & ~masks['nonplant']
    pixels[erase] = free_pixel
    pixels[grid['known'] & masks['physical']] = occupied_pixel
    return pixels


def guard_outputs(output_dir, protected_files, protected_dirs=()):
    """Reject aliases before mkdir or any output mutation."""
    destination = Path(output_dir).absolute()
    for path in (destination, *destination.parents):
        if path.is_symlink():
            raise ValueError('output directories must not traverse symlinks')
    destination = destination.resolve()
    protected = {Path(path).resolve() for path in protected_files}
    roots = [Path(path).resolve() for path in protected_dirs]
    for name in OUTPUT_NAMES:
        path = destination/name
        if path.is_symlink() or path.resolve() in protected or any(
                path.resolve().is_relative_to(root) for root in roots):
            raise ValueError('output must not overwrite a protected source or symlink')
    return destination


def prepare(source_sdf, metadata_path, localization_map, registration_mission,
            output_dir, model_root, alignment=None, protected_dirs=()):
    source_sdf, metadata_path, registration_mission = (
        Path(path).resolve() for path in (source_sdf, metadata_path, registration_mission))
    grid = load_map(localization_map)
    mission = yaml.safe_load(registration_mission.read_text())
    alignment = validate_registration(mission, source_sdf, metadata_path, grid, alignment)
    metadata = json.loads(metadata_path.read_text())
    source_root = ET.parse(source_sdf).getroot()
    destination = guard_outputs(output_dir, [source_sdf, metadata_path, registration_mission,
                                            grid['yaml'], grid['image']], protected_dirs)
    marked, masks, details = marked_world_and_masks(source_root, metadata, grid, alignment,
                                                   Path(model_root))
    pixels = collision_grid(grid, masks)
    # Validate preservation before creating any output.
    before_fruits = [ET.tostring(model) for model in source_root.find('world').findall('model')
                     if model.get('name') in {fruit['id'] for fruit in metadata['fruits']}]
    after_fruits = [ET.tostring(model) for model in marked.find('world').findall('model')
                    if model.get('name') in {fruit['id'] for fruit in metadata['fruits']}]
    before_collisions = [ET.tostring(collision) for collision in source_root.iter('collision')]
    after_collisions = [ET.tostring(collision) for collision in marked.iter('collision')]
    if before_fruits != after_fruits or before_collisions != after_collisions:
        raise ValueError('candidate changed physical collisions or fruit geometry')
    if not np.array_equal(pixels[~grid['known']], grid['pixels'][~grid['known']]):
        raise ValueError('candidate changed unknown map pixels')
    destination.mkdir(parents=True, exist_ok=True)
    world_file = destination/'tomato_field_collision_navigation.sdf'
    metadata_file = destination/'tomato_field.json'
    ET.indent(marked, space='  ')
    ET.ElementTree(marked).write(world_file, encoding='utf-8', xml_declaration=True)
    shutil.copyfile(metadata_path, metadata_file)
    image_file = destination/'navigation_collision.pgm'
    write_pgm(image_file, pixels)
    description = copy.deepcopy(grid['description'])
    description['image'] = image_file.name
    yaml_file = destination/'navigation_collision.yaml'
    yaml_file.write_text(yaml.safe_dump(description, sort_keys=False))
    removed = grid['occupied'] & ~((pixels == 0) if not description.get('negate', 0) else (pixels == 255))
    all_masks = {'foliage_envelope_mask.pgm': masks['foliage'],
                 'physical_collision_mask.pgm': masks['physical'],
                 'nonplant_collision_mask.pgm': masks['nonplant'],
                 'known_source_mask.pgm': grid['known'],
                 'removed_leaf_occupancy_mask.pgm': removed}
    mask_details = {}
    for name, mask in all_masks.items():
        path = destination/name
        write_pgm(path, mask.astype(np.uint8)*255)
        mask_details[name] = {'path': str(path), 'sha256': digest(path),
                             'true_pixel': 255, 'false_pixel': 0,
                             'true_cells': int(mask.sum())}
    manifest = {
        'schema_version': 1,
        'scope': 'SIMULATION ONLY: collision blueprint using known Gazebo geometry; '
                 'no SLAM semantic classification or real-lidar validation',
        'world_to_map': alignment,
        'source_sdf': str(source_sdf), 'source_sdf_sha256': digest(source_sdf),
        'source_metadata': str(metadata_path), 'source_metadata_sha256': digest(metadata_path),
        'registration_mission': str(registration_mission),
        'registration_mission_sha256': digest(registration_mission),
        'localization_map_yaml': str(grid['yaml']), 'localization_map_yaml_sha256': digest(grid['yaml']),
        'localization_map_image': str(grid['image']), 'localization_map_image_sha256': digest(grid['image']),
        'world': str(world_file), 'world_sha256': digest(world_file),
        'metadata': str(metadata_file), 'metadata_sha256': digest(metadata_file),
        'navigation_map_yaml': str(yaml_file), 'navigation_map_yaml_sha256': digest(yaml_file),
        'navigation_map_image': str(image_file), 'navigation_map_image_sha256': digest(image_file),
        'map_geometry': {'width': grid['width'], 'height': grid['height'],
                         'resolution_m': grid['resolution'], 'origin': grid['origin']},
        'masks': mask_details,
        'changed_cells': int(np.count_nonzero(pixels != grid['pixels'])),
        'removed_occupied_cells': int(removed.sum()),
        'added_occupied_cells': int(np.count_nonzero(grid['known'] & masks['physical'] & ~grid['occupied'])),
        'unknown_cells_preserved': int(np.count_nonzero(~grid['known'])),
        'physical_collision_cells_in_unknown': int(np.count_nonzero(
            masks['physical'] & ~grid['known'])),
        'fruit_subtrees_preserved': True, 'physical_collisions_preserved': True,
        'metadata_byte_identical': digest(metadata_path) == digest(metadata_file),
        'visibility': {'soft_noncollision_visuals': SOFT_VISIBILITY,
                       'navigation_only_stem_proxies': NAVIGATION_VISIBILITY,
                       'sensor_mask_validation': 'pending runtime validation by consuming sensors'},
        'preservation_rule': 'Only known cells intersecting original foliage convex envelopes '
                             'may be erased; all nonplant physical footprints are protected. '
                             'Known cells intersecting any physical footprint are occupied.',
        **details,
    }
    (destination/'manifest_collision_navigation.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False)+'\n')
    return manifest


def main():
    root = Path(__file__).resolve().parents[1]
    backup = root/'artifacts/harvest_demo_profile_20261013/previous_default'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-sdf', type=Path, default=backup/'tomato_field_default.sdf')
    parser.add_argument('--metadata', type=Path, default=backup/'tomato_field_default.json')
    parser.add_argument('--registration-mission', type=Path,
                        default=backup/'tomato_field_default_single_corridor.yaml')
    parser.add_argument('--localization-map', type=Path,
                        default=root/'ros2_ws/src/agri_navigation/maps/tomato_field_default.yaml')
    parser.add_argument('--output-dir', type=Path, default=root/'artifacts/close_harvest_20261014')
    parser.add_argument('--model-root', type=Path,
                        default=root/'sim_ws/src/agri_greenhouse_worlds/models')
    parser.add_argument('--world-to-map', type=float, nargs=3)
    args = parser.parse_args()
    try:
        manifest = prepare(args.source_sdf, args.metadata, args.localization_map,
                           args.registration_mission, args.output_dir, args.model_root,
                           args.world_to_map, [root/'sim_ws/src', root/'ros2_ws/src', backup])
    except (OSError, ValueError, KeyError, TypeError, ET.ParseError) as error:
        parser.error(str(error))
    print(json.dumps({key: manifest[key] for key in (
        'world', 'navigation_map_yaml', 'plant_count', 'fruit_count', 'changed_cells',
        'removed_occupied_cells', 'added_occupied_cells', 'unknown_cells_preserved')}, indent=2))


if __name__ == '__main__':
    main()
