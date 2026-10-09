#!/usr/bin/env python3
"""
Generate a Nav2 keepout mask from a parameterized tomato-field metadata file.

The field generator writes plant poses in the Gazebo ``world`` frame.  A saved
Nav2 map is usually expressed in a different ``map`` frame, so this utility
requires an explicit world-to-map planar transform.  Requiring the transform
prevents a silently misaligned mask when SLAM starts at a different pose.

The output mask uses the same image dimensions, resolution, and origin as the
saved map.  Black pixels are keepout cells and white pixels are free cells;
the resulting YAML can be used by Nav2's KeepoutFilter through a
``costmap_filter_info_server``.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Iterable, Sequence

import yaml


DEFAULT_FREE_PIXEL = 255
DEFAULT_KEEPOUT_PIXEL = 0
DEFAULT_MAP_MODE = 'trinary'
PGM_MAX_VALUE = 255


def _finite(value: float, name: str) -> float:
    """Return a finite float or raise a user-facing validation error."""
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


def _yaml_number(value, name: str) -> float:
    """Read one numeric YAML value with a useful error message."""
    try:
        return _finite(value, name)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'map YAML field {name!r} must be numeric') from exc


def _read_pgm_token(stream) -> bytes:
    """Read one PGM header token, skipping whitespace and comment lines."""
    token = bytearray()
    while True:
        byte = stream.read(1)
        if not byte:
            raise ValueError('truncated PGM header')
        if byte in b' \t\r\n':
            continue
        if byte == b'#':
            stream.readline()
            continue
        token.append(byte[0])
        break
    while True:
        byte = stream.read(1)
        if not byte or byte in b' \t\r\n':
            break
        if byte == b'#':
            stream.readline()
            break
        token.append(byte[0])
    return bytes(token)


def read_pgm_dimensions(path: Path) -> tuple[int, int, int, str]:
    """Read PGM width, height, max value, and magic without decoding pixels."""
    with path.open('rb') as stream:
        magic = _read_pgm_token(stream).decode('ascii', errors='strict')
        if magic not in ('P5', 'P2'):
            raise ValueError(f'{path} is not a PGM image (expected P5 or P2)')
        width = int(_read_pgm_token(stream))
        height = int(_read_pgm_token(stream))
        max_value = int(_read_pgm_token(stream))
    if width < 1 or height < 1 or not 1 <= max_value <= PGM_MAX_VALUE:
        raise ValueError(f'invalid PGM dimensions or max value in {path}')
    return width, height, max_value, magic


def load_map_description(path: Path) -> dict:
    """Load a saved-map YAML and resolve its image dimensions and geometry."""
    try:
        description = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'unable to read map YAML {path}: {exc}') from exc
    if not isinstance(description, dict):
        raise ValueError('map YAML must contain a mapping')
    image_name = description.get('image')
    if not isinstance(image_name, str) or not image_name:
        raise ValueError('map YAML must contain a non-empty image field')
    image_path = Path(image_name)
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    try:
        resolution = _yaml_number(description.get('resolution'), 'resolution')
    except ValueError as exc:
        raise ValueError(str(exc)) from exc
    if resolution <= 0.0:
        raise ValueError('map resolution must be positive')
    origin = description.get('origin', [0.0, 0.0, 0.0])
    if not isinstance(origin, Sequence) or isinstance(origin, (str, bytes)) or len(origin) < 2:
        raise ValueError('map YAML origin must contain at least x and y')
    origin_xyz = [
        _yaml_number(origin[0], 'origin[0]'),
        _yaml_number(origin[1], 'origin[1]'),
        _yaml_number(origin[2], 'origin[2]') if len(origin) > 2 else 0.0,
    ]
    width, height, max_value, magic = read_pgm_dimensions(image_path)
    return {
        'path': path,
        'image_path': image_path,
        'image_name': image_path.name,
        'resolution': resolution,
        'origin': origin_xyz,
        'width': width,
        'height': height,
        'source_max_value': max_value,
        'source_magic': magic,
        'mode': description.get('mode', DEFAULT_MAP_MODE),
    }


def load_field_metadata(path: Path) -> dict:
    """Load and validate generator metadata, returning its plant list."""
    try:
        metadata = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'unable to read field metadata {path}: {exc}') from exc
    if not isinstance(metadata, dict):
        raise ValueError('field metadata must contain a JSON object')
    plants = metadata.get('plants')
    if not isinstance(plants, list) or not plants:
        raise ValueError('field metadata must contain a non-empty plants list')
    parameters = metadata.get('parameters', {})
    if not isinstance(parameters, dict):
        raise ValueError('field metadata parameters must be an object')
    geometry = metadata.get('plant_geometry', {})
    if not isinstance(geometry, dict):
        raise ValueError('field metadata plant_geometry must be an object')
    clean_plants = []
    for index, plant in enumerate(plants):
        if not isinstance(plant, dict):
            raise ValueError(f'plant {index} must be an object')
        try:
            x = _finite(plant['x'], f'plants[{index}].x')
            y = _finite(plant['y'], f'plants[{index}].y')
            yaw = _finite(plant.get('yaw', 0.0), f'plants[{index}].yaw')
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'invalid plant {index} in metadata') from exc
        clean_plants.append({
            'id': str(plant.get('id', index)), 'x': x, 'y': y, 'yaw': yaw,
        })
    return {
        'metadata': metadata, 'parameters': parameters,
        'geometry': geometry, 'plants': clean_plants,
    }


def _parameter_float(parameters: dict, name: str, fallback: float | None = None) -> float | None:
    """Read an optional generator parameter as a finite float."""
    value = parameters.get(name, fallback)
    if value is None:
        return None
    return _finite(value, f'metadata parameters.{name}')


def _parameter_int(parameters: dict, name: str, fallback: int | None = None) -> int | None:
    """Read an optional generator parameter as a positive integer."""
    value = parameters.get(name, fallback)
    if value is None:
        return None
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'metadata parameters.{name} must be an integer') from exc
    if result < 1:
        raise ValueError(f'metadata parameters.{name} must be positive')
    return result


def resolve_layout(field: dict, expected: dict[str, float | int | None]) -> dict:
    """Resolve and validate rows, counts, spacing, and canopy width."""
    parameters = field['parameters']
    rows = _parameter_int(parameters, 'rows', expected.get('rows'))
    plants_per_row = _parameter_int(parameters, 'plants_per_row', expected.get('plants_per_row'))
    if rows is None or plants_per_row is None:
        raise ValueError('metadata must include rows and plants_per_row')
    for name, value in (('rows', rows), ('plants_per_row', plants_per_row)):
        supplied = expected.get(name)
        if supplied is not None and int(supplied) != value:
            raise ValueError(f'{name} does not match field metadata ({supplied} != {value})')
    if len(field['plants']) != rows * plants_per_row:
        plant_count = len(field['plants'])
        raise ValueError(
            f'metadata has {plant_count} plants but rows*plants_per_row is '
            f'{rows * plants_per_row}; regenerate the field and metadata together'
        )
    row_spacing = _parameter_float(parameters, 'row_spacing', expected.get('row_spacing'))
    plant_spacing = _parameter_float(parameters, 'plant_spacing', expected.get('plant_spacing'))
    if row_spacing is None or plant_spacing is None or row_spacing <= 0.0 or plant_spacing <= 0.0:
        raise ValueError('metadata must include positive row_spacing and plant_spacing')
    for name, value in (('row_spacing', row_spacing), ('plant_spacing', plant_spacing)):
        supplied = expected.get(name)
        if supplied is not None and not math.isclose(
                float(supplied), value, rel_tol=1e-6, abs_tol=1e-6):
            raise ValueError(f'{name} does not match field metadata ({supplied} != {value})')
    width = expected.get('plant_width')
    if width is None:
        width = field['geometry'].get('width')
    if width is None:
        width = 0.866695
    width = _finite(width, 'plant-width')
    if width <= 0.0:
        raise ValueError('plant-width must be positive')
    return {
        'rows': rows,
        'plants_per_row': plants_per_row,
        'row_spacing': row_spacing,
        'plant_spacing': plant_spacing,
        'plant_width': width,
    }


def world_to_map(point: tuple[float, float], transform: Sequence[float]) -> tuple[float, float]:
    """Apply ``map = translation + R(yaw) * world`` to a planar point."""
    tx, ty, yaw = transform
    cosine, sine = math.cos(yaw), math.sin(yaw)
    x, y = point
    return tx + cosine * x - sine * y, ty + sine * x + cosine * y


def transform_from_robot_start(start: Sequence[float]) -> list[float]:
    """Return ``map <- world`` when map origin is the robot start pose.

    SLAM Toolbox starts its map frame at the robot pose used to begin mapping.
    ``start`` is that robot pose in the Gazebo world frame as ``x y yaw``.
    """
    start_x, start_y, start_yaw = start
    yaw = -start_yaw
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        -(cosine * start_x - sine * start_y),
        -(sine * start_x + cosine * start_y),
        yaw,
    ]


def map_to_image(point: tuple[float, float], map_info: dict) -> tuple[float, float]:
    """Convert map-frame metres to continuous image coordinates in pixels."""
    origin_x, origin_y, origin_yaw = map_info['origin']
    dx, dy = point[0] - origin_x, point[1] - origin_y
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    resolution = map_info['resolution']
    return local_x / resolution, local_y / resolution


def _distance_to_segment(
        px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    """Return Euclidean distance from a pixel point to a segment."""
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-15:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    return math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def _paint_disk(
        pixels: bytearray, width: int, height: int,
        cx: float, cy: float, radius_px: float) -> int:
    """Paint a conservative disk and return the number of newly painted cells."""
    # Include half a pixel diagonal so a continuous radius cannot leave a
    # one-cell free slit between the mask and the visual/collision geometry.
    threshold = max(0.0, radius_px) + math.sqrt(0.5)
    min_x = max(0, math.floor(cx - threshold))
    max_x = min(width - 1, math.ceil(cx + threshold))
    min_y = max(0, math.floor(cy - threshold))
    max_y = min(height - 1, math.ceil(cy + threshold))
    changed = 0
    threshold_squared = threshold * threshold
    for row_bottom in range(min_y, max_y + 1):
        for col in range(min_x, max_x + 1):
            dx = (col + 0.5) - cx
            dy = (row_bottom + 0.5) - cy
            if dx * dx + dy * dy > threshold_squared:
                continue
            index = (height - 1 - row_bottom) * width + col
            if pixels[index] != DEFAULT_KEEPOUT_PIXEL:
                pixels[index] = DEFAULT_KEEPOUT_PIXEL
                changed += 1
    return changed


def _paint_capsule(
    pixels: bytearray,
    width: int,
    height: int,
    first: tuple[float, float],
    second: tuple[float, float],
    radius_px: float,
) -> int:
    """Paint a capsule between two image-space points."""
    threshold = max(0.0, radius_px) + math.sqrt(0.5)
    min_x = max(0, math.floor(min(first[0], second[0]) - threshold))
    max_x = min(width - 1, math.ceil(max(first[0], second[0]) + threshold))
    min_y = max(0, math.floor(min(first[1], second[1]) - threshold))
    max_y = min(height - 1, math.ceil(max(first[1], second[1]) + threshold))
    changed = 0
    for row_bottom in range(min_y, max_y + 1):
        for col in range(min_x, max_x + 1):
            px, py = col + 0.5, row_bottom + 0.5
            if _distance_to_segment(px, py, first[0], first[1], second[0], second[1]) > threshold:
                continue
            index = (height - 1 - row_bottom) * width + col
            if pixels[index] != DEFAULT_KEEPOUT_PIXEL:
                pixels[index] = DEFAULT_KEEPOUT_PIXEL
                changed += 1
    return changed


def render_mask(field: dict, map_info: dict, layout: dict, transform: Sequence[float],
                margin: float, row_band: bool) -> tuple[bytearray, dict]:
    """Render plant canopy keepouts into the saved map's pixel grid."""
    if margin < 0.0 or not math.isfinite(margin):
        raise ValueError('keepout-margin must be finite and nonnegative')
    width, height = map_info['width'], map_info['height']
    pixels = bytearray([DEFAULT_FREE_PIXEL]) * (width * height)
    radius_m = layout['plant_width'] / 2.0 + margin
    radius_px = radius_m / map_info['resolution']
    points_by_row: list[list[tuple[float, float]]] = [[] for _ in range(layout['rows'])]
    painted = 0
    for index, plant in enumerate(field['plants']):
        row = index // layout['plants_per_row']
        map_point = world_to_map((plant['x'], plant['y']), transform)
        image_point = map_to_image(map_point, map_info)
        points_by_row[row].append(image_point)
        painted += _paint_disk(pixels, width, height, image_point[0], image_point[1], radius_px)
    if row_band:
        for points in points_by_row:
            for first, second in zip(points, points[1:]):
                painted += _paint_capsule(pixels, width, height, first, second, radius_px)
    occupied = sum(pixel == DEFAULT_KEEPOUT_PIXEL for pixel in pixels)
    return pixels, {
        'plant_count': len(field['plants']),
        'rows': layout['rows'],
        'plants_per_row': layout['plants_per_row'],
        'plant_width_m': layout['plant_width'],
        'keepout_margin_m': margin,
        'keepout_radius_m': radius_m,
        'row_band': row_band,
        'paint_operations_new_cells': painted,
        'keepout_cells': occupied,
        'free_cells': len(pixels) - occupied,
    }


def write_pgm(path: Path, width: int, height: int, pixels: bytes) -> None:
    """Write an 8-bit binary PGM image."""
    if len(pixels) != width * height:
        raise ValueError('pixel buffer length does not match image dimensions')
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('wb') as stream:
        stream.write(f'P5\n{width} {height}\n{PGM_MAX_VALUE}\n'.encode('ascii'))
        stream.write(pixels)


def write_mask_yaml(path: Path, image_name: str, map_info: dict) -> None:
    """Write a Nav2-compatible occupancy/keepout mask YAML."""
    origin = map_info['origin']
    resolution = map_info['resolution']
    text = (
        f'image: {image_name}\n'
        f'mode: {DEFAULT_MAP_MODE}\n'
        f'resolution: {resolution:.9g}\n'
        f'origin: [{origin[0]:.9g}, {origin[1]:.9g}, {origin[2]:.9g}]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')


def build_parser() -> argparse.ArgumentParser:
    """Create the command-line parser."""
    parser = argparse.ArgumentParser(
        description='Generate a Nav2 keepout mask from tomato-field metadata and a saved map.'
    )
    parser.add_argument('--metadata', type=Path, required=True,
                        help='JSON emitted by generate_tomato_field.py --metadata.')
    parser.add_argument('--map-yaml', type=Path, required=True,
                        help='Saved Nav2 map YAML; mask geometry matches this map exactly.')
    parser.add_argument('--output', type=Path, required=True,
                        help='Output basename, e.g. artifacts/maps/field/keepout_mask.')
    transform = parser.add_mutually_exclusive_group(required=True)
    transform.add_argument(
        '--world-to-map', type=float, nargs=3, metavar=('X', 'Y', 'YAW'),
        help='Planar transform: map = [X,Y] + R(YAW) * world.',
    )
    transform.add_argument(
        '--robot-start-world', type=float, nargs=3,
        metavar=('X', 'Y', 'YAW'),
        help='Gazebo world pose where mapping started; derives map <- world.',
    )
    parser.add_argument('--keepout-margin', type=float, default=0.15,
                        help='Canopy/safety margin added around each plant (meters).')
    parser.add_argument('--plant-width', type=float,
                        help='Override metadata canopy width used for the mask (meters).')
    parser.add_argument('--rows', type=int,
                        help='Expected row count; errors if it differs from metadata.')
    parser.add_argument('--plants-per-row', type=int,
                        help='Expected plants per row; errors if it differs from metadata.')
    parser.add_argument('--row-spacing', type=float,
                        help='Expected row spacing; errors if it differs from metadata.')
    parser.add_argument('--plant-spacing', type=float,
                        help='Expected plant spacing; errors if it differs from metadata.')
    parser.add_argument('--no-row-band', action='store_true',
                        help='Mask individual canopy disks only; default connects each row.')
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    """Generate the mask and return a shell-friendly status code."""
    args = build_parser().parse_args(argv)
    try:
        field = load_field_metadata(args.metadata)
        map_info = load_map_description(args.map_yaml)
        expected = {
            'rows': args.rows,
            'plants_per_row': args.plants_per_row,
            'row_spacing': args.row_spacing,
            'plant_spacing': args.plant_spacing,
            'plant_width': args.plant_width,
        }
        layout = resolve_layout(field, expected)
        if args.world_to_map is not None:
            transform = [
                _finite(value, 'world-to-map transform')
                for value in args.world_to_map
            ]
        else:
            start = [
                _finite(value, 'robot-start-world pose')
                for value in args.robot_start_world
            ]
            transform = transform_from_robot_start(start)
        pixels, report = render_mask(
            field, map_info, layout, transform, float(args.keepout_margin), not args.no_row_band
        )
        output_base = args.output.with_suffix('')
        pgm_path = output_base.with_suffix('.pgm')
        yaml_path = output_base.with_suffix('.yaml')
        report_path = output_base.with_suffix('.json')
        write_pgm(pgm_path, map_info['width'], map_info['height'], pixels)
        write_mask_yaml(yaml_path, pgm_path.name, map_info)
        report.update({
            'schema_version': 1,
            'metadata': str(args.metadata.resolve()),
            'source_map': str(args.map_yaml.resolve()),
            'world_to_map': transform,
            'map': {
                'width': map_info['width'],
                'height': map_info['height'],
                'resolution_m': map_info['resolution'],
                'origin': map_info['origin'],
            },
            'outputs': {'mask_yaml': str(yaml_path), 'mask_pgm': str(pgm_path)},
        })
        report_text = json.dumps(report, indent=2, ensure_ascii=False) + '\n'
        report_path.write_text(report_text, encoding='utf-8')
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 2
    print(f'Generated Nav2 keepout mask: {yaml_path.resolve()}')
    print(
        f"Keepout cells: {report['keepout_cells']} / "
        f"{map_info['width'] * map_info['height']}"
    )
    print(f'Report: {report_path.resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
