"""Strict, dependency-light schema for versioned harvest missions."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import re
from typing import Any, Mapping, Sequence

import yaml


SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')
VALID_WORK_SIDES = frozenset({'left', 'right'})
VALID_TASKS = frozenset({'harvest', 'scan'})
VALID_ROUTE_LAYOUTS = frozenset({'shared_pairs', 'independent_side_stops'})


class MissionConfigError(ValueError):
    """Raised when a mission file is unsafe or internally inconsistent."""


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MissionConfigError(f'{name} must be a mapping')
    return value


def _sequence(value: Any, name: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise MissionConfigError(f'{name} must be a sequence')
    return value


def _string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise MissionConfigError(f'{name} must be a non-empty string')
    return value.strip()


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise MissionConfigError(f'{name} must be numeric') from exc
    if not math.isfinite(result):
        raise MissionConfigError(f'{name} must be finite')
    return result


def _positive(value: Any, name: str, allow_zero: bool = False) -> float:
    result = _finite(value, name)
    if result < 0.0 if allow_zero else result <= 0.0:
        qualifier = 'nonnegative' if allow_zero else 'positive'
        raise MissionConfigError(f'{name} must be {qualifier}')
    return result


def _integer(value: Any, name: str, minimum: int = 0) -> int:
    if isinstance(value, bool):
        raise MissionConfigError(f'{name} must be an integer')
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise MissionConfigError(f'{name} must be an integer') from exc
    if result != value or result < minimum:
        raise MissionConfigError(f'{name} must be an integer >= {minimum}')
    return result


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise MissionConfigError(f'{name} must be a boolean')
    return value


def _sha256(value: Any, name: str) -> str:
    result = _string(value, name).lower()
    if not SHA256_PATTERN.fullmatch(result):
        raise MissionConfigError(f'{name} must be a lowercase SHA-256')
    return result


def normalized_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in the route's map frame."""

    x: float
    y: float
    yaw: float

    @classmethod
    def from_mapping(cls, value: Any, name: str) -> 'Pose2D':
        """Build a pose from a strict mapping."""
        data = _mapping(value, name)
        return cls(
            x=_finite(data.get('x'), f'{name}.x'),
            y=_finite(data.get('y'), f'{name}.y'),
            yaw=_finite(data.get('yaw'), f'{name}.yaw'),
        )


@dataclass(frozen=True)
class CorridorBounds:
    """Allowed base-pose envelope for the single corridor."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    required_yaw: float
    yaw_tolerance: float

    @classmethod
    def from_mapping(cls, value: Any) -> 'CorridorBounds':
        """Build and validate corridor limits."""
        data = _mapping(value, 'route.corridor')
        result = cls(
            x_min=_finite(data.get('x_min'), 'route.corridor.x_min'),
            x_max=_finite(data.get('x_max'), 'route.corridor.x_max'),
            y_min=_finite(data.get('y_min'), 'route.corridor.y_min'),
            y_max=_finite(data.get('y_max'), 'route.corridor.y_max'),
            required_yaw=_finite(
                data.get('required_yaw'), 'route.corridor.required_yaw'),
            yaw_tolerance=_positive(
                data.get('yaw_tolerance'),
                'route.corridor.yaw_tolerance', allow_zero=True),
        )
        if result.x_min >= result.x_max or result.y_min >= result.y_max:
            raise MissionConfigError('route.corridor bounds are empty')
        if result.yaw_tolerance > math.pi:
            raise MissionConfigError(
                'route.corridor.yaw_tolerance cannot exceed pi')
        return result

    def validate_pose(self, pose: Pose2D, name: str) -> None:
        """Reject a pose outside the configured corridor or heading."""
        if not self.x_min <= pose.x <= self.x_max:
            raise MissionConfigError(f'{name}.x is outside corridor bounds')
        if not self.y_min <= pose.y <= self.y_max:
            raise MissionConfigError(f'{name}.y is outside corridor bounds')
        error = abs(normalized_angle(pose.yaw - self.required_yaw))
        if error > self.yaw_tolerance + 1e-12:
            raise MissionConfigError(
                f'{name}.yaw differs from required corridor heading')


@dataclass(frozen=True)
class MicroAdjust:
    """Permitted base adjustment around one nominal harvest pose."""

    longitudinal_m: float
    lateral_m: float
    yaw_rad: float

    @classmethod
    def from_mapping(cls, value: Any, name: str) -> 'MicroAdjust':
        """Build nonnegative micro-adjustment limits."""
        data = _mapping(value, name)
        return cls(
            longitudinal_m=_positive(
                data.get('longitudinal_m'),
                f'{name}.longitudinal_m', allow_zero=True),
            lateral_m=_positive(
                data.get('lateral_m'),
                f'{name}.lateral_m', allow_zero=True),
            yaw_rad=_positive(
                data.get('yaw_rad'), f'{name}.yaw_rad', allow_zero=True),
        )


@dataclass(frozen=True)
class PlantReference:
    """Offline plant identity and its fixed field/map pose."""

    plant_id: str
    row_id: int
    index_in_row: int
    pose_world: Pose2D
    pose_map: Pose2D

    @classmethod
    def from_mapping(cls, value: Any, name: str) -> 'PlantReference':
        """Build one plant reference from mission data."""
        data = _mapping(value, name)
        return cls(
            plant_id=_string(data.get('id'), f'{name}.id'),
            row_id=_integer(data.get('row_id'), f'{name}.row_id'),
            index_in_row=_integer(
                data.get('index_in_row'), f'{name}.index_in_row'),
            pose_world=Pose2D.from_mapping(
                data.get('pose_world'), f'{name}.pose_world'),
            pose_map=Pose2D.from_mapping(
                data.get('pose_map'), f'{name}.pose_map'),
        )


@dataclass(frozen=True)
class HarvestWaypoint:
    """One logical plant task and its associated base stop."""

    waypoint_id: str
    sequence: int
    stop_id: str
    pose: Pose2D
    plant: PlantReference
    work_side: str
    task: str
    scan_pose_id: str
    arm_pose_id: str
    micro_adjust: MicroAdjust
    goal_timeout_s: float
    station_dwell_s: float
    enabled: bool
    validation: Mapping[str, Any]

    @classmethod
    def from_mapping(cls, value: Any, index: int) -> 'HarvestWaypoint':
        """Build one logical plant task from mission data."""
        name = f'route.waypoints[{index}]'
        data = _mapping(value, name)
        side = _string(data.get('work_side'), f'{name}.work_side')
        if side not in VALID_WORK_SIDES:
            raise MissionConfigError(
                f'{name}.work_side must be left or right')
        task = _string(data.get('task'), f'{name}.task')
        if task not in VALID_TASKS:
            raise MissionConfigError(
                f'{name}.task must be one of {sorted(VALID_TASKS)}')
        validation = _mapping(data.get('validation', {}),
                              f'{name}.validation')
        return cls(
            waypoint_id=_string(data.get('id'), f'{name}.id'),
            sequence=_integer(
                data.get('sequence'), f'{name}.sequence', minimum=1),
            stop_id=_string(data.get('stop_id'), f'{name}.stop_id'),
            pose=Pose2D.from_mapping(data.get('base_pose'),
                                     f'{name}.base_pose'),
            plant=PlantReference.from_mapping(
                data.get('plant'), f'{name}.plant'),
            work_side=side,
            task=task,
            scan_pose_id=_string(
                data.get('scan_pose_id'), f'{name}.scan_pose_id'),
            arm_pose_id=_string(
                data.get('arm_pose_id'), f'{name}.arm_pose_id'),
            micro_adjust=MicroAdjust.from_mapping(
                data.get('micro_adjust'), f'{name}.micro_adjust'),
            goal_timeout_s=_positive(
                data.get('goal_timeout_s'), f'{name}.goal_timeout_s'),
            station_dwell_s=_positive(
                data.get('station_dwell_s'),
                f'{name}.station_dwell_s', allow_zero=True),
            enabled=_boolean(data.get('enabled'), f'{name}.enabled'),
            validation=dict(validation),
        )

    def as_event(self) -> dict[str, Any]:
        """Return stable metadata for status/event JSON."""
        return {
            'waypoint_id': self.waypoint_id,
            'sequence': self.sequence,
            'stop_id': self.stop_id,
            'plant_id': self.plant.plant_id,
            'row_id': self.plant.row_id,
            'index_in_row': self.plant.index_in_row,
            'base_pose': {
                'x': self.pose.x, 'y': self.pose.y, 'yaw': self.pose.yaw,
            },
            'plant_pose_map': {
                'x': self.plant.pose_map.x,
                'y': self.plant.pose_map.y,
                'yaw': self.plant.pose_map.yaw,
            },
            'work_side': self.work_side,
            'task': self.task,
            'scan_pose_id': self.scan_pose_id,
            'arm_pose_id': self.arm_pose_id,
            'micro_adjust': {
                'longitudinal_m': self.micro_adjust.longitudinal_m,
                'lateral_m': self.micro_adjust.lateral_m,
                'yaw_rad': self.micro_adjust.yaw_rad,
            },
        }


@dataclass(frozen=True)
class MapBinding:
    """Installed map identity used to fail closed on route/map mismatch."""

    frame_id: str
    package: str
    yaml_relative_path: str
    image_sha256: str
    occupancy_sha256: str
    keepout_yaml_relative_path: str
    keepout_image_sha256: str
    keepout_occupancy_sha256: str
    resolution_m: float
    origin: tuple[float, float, float]
    width_px: int
    height_px: int
    mode: str
    negate: bool
    occupied_thresh: float
    free_thresh: float

    @classmethod
    def from_mapping(cls, value: Any) -> 'MapBinding':
        """Build a saved-map and keepout-mask identity binding."""
        data = _mapping(value, 'map')
        origin = _sequence(data.get('origin'), 'map.origin')
        if len(origin) != 3:
            raise MissionConfigError('map.origin must contain x, y, yaw')
        result = cls(
            frame_id=_string(data.get('frame_id'), 'map.frame_id'),
            package=_string(data.get('package'), 'map.package'),
            yaml_relative_path=_string(
                data.get('yaml_relative_path'), 'map.yaml_relative_path'),
            image_sha256=_sha256(
                data.get('image_sha256'), 'map.image_sha256'),
            occupancy_sha256=_sha256(
                data.get('occupancy_sha256'), 'map.occupancy_sha256'),
            keepout_yaml_relative_path=_string(
                data.get('keepout_yaml_relative_path'),
                'map.keepout_yaml_relative_path'),
            keepout_image_sha256=_sha256(
                data.get('keepout_image_sha256'),
                'map.keepout_image_sha256'),
            keepout_occupancy_sha256=_sha256(
                data.get('keepout_occupancy_sha256'),
                'map.keepout_occupancy_sha256'),
            resolution_m=_positive(
                data.get('resolution_m'), 'map.resolution_m'),
            origin=tuple(
                _finite(item, f'map.origin[{index}]')
                for index, item in enumerate(origin)
            ),
            width_px=_integer(data.get('width_px'), 'map.width_px', minimum=1),
            height_px=_integer(
                data.get('height_px'), 'map.height_px', minimum=1),
            mode=_string(data.get('mode'), 'map.mode'),
            negate=_boolean(data.get('negate'), 'map.negate'),
            occupied_thresh=_positive(
                data.get('occupied_thresh'), 'map.occupied_thresh'),
            free_thresh=_positive(
                data.get('free_thresh'), 'map.free_thresh', allow_zero=True),
        )
        if result.mode != 'trinary':
            raise MissionConfigError('map.mode must be trinary')
        if not 0.0 <= result.free_thresh < result.occupied_thresh <= 1.0:
            raise MissionConfigError('map occupancy thresholds are invalid')
        return result


@dataclass(frozen=True)
class FieldBinding:
    """Field-generator provenance tied to the mission."""

    field_id: str
    metadata_sha256: str
    sdf_sha256: str
    seed: int
    rows: int
    plants_per_row: int
    row_spacing_m: float
    plant_spacing_m: float
    plant_width_m: float
    plant_collision_width_m: float
    selected_rows: tuple[int, ...]
    excluded_plant_count: int

    @classmethod
    def from_mapping(cls, value: Any) -> 'FieldBinding':
        """Build field-generator provenance from mission data."""
        data = _mapping(value, 'field')
        rows = _integer(data.get('rows'), 'field.rows', minimum=1)
        plants_per_row = _integer(
            data.get('plants_per_row'),
            'field.plants_per_row', minimum=1)
        selected_raw = _sequence(
            data.get('selected_rows'), 'field.selected_rows')
        selected = tuple(
            _integer(item, f'field.selected_rows[{index}]')
            for index, item in enumerate(selected_raw)
        )
        if not selected or len(set(selected)) != len(selected):
            raise MissionConfigError(
                'field.selected_rows must contain unique row indexes')
        if any(row >= rows for row in selected):
            raise MissionConfigError(
                'field.selected_rows contains invalid row')
        return cls(
            field_id=_string(data.get('id'), 'field.id'),
            metadata_sha256=_sha256(
                data.get('metadata_sha256'), 'field.metadata_sha256'),
            sdf_sha256=_sha256(data.get('sdf_sha256'), 'field.sdf_sha256'),
            seed=_integer(data.get('seed'), 'field.seed'),
            rows=rows,
            plants_per_row=plants_per_row,
            row_spacing_m=_positive(
                data.get('row_spacing_m'), 'field.row_spacing_m'),
            plant_spacing_m=_positive(
                data.get('plant_spacing_m'), 'field.plant_spacing_m'),
            plant_width_m=_positive(
                data.get('plant_width_m'), 'field.plant_width_m'),
            plant_collision_width_m=_positive(
                data.get('plant_collision_width_m'),
                'field.plant_collision_width_m'),
            selected_rows=selected,
            excluded_plant_count=_integer(
                data.get('excluded_plant_count'),
                'field.excluded_plant_count'),
        )


@dataclass(frozen=True)
class TransitWaypoint:
    """Navigation-only pose visited before an enabled harvest station."""

    transit_id: str
    before_waypoint_id: str
    pose: Pose2D
    goal_timeout_s: float = 120.0

    @classmethod
    def from_mapping(cls, value: Any, index: int) -> 'TransitWaypoint':
        """Parse one transit while preserving the file's execution order."""
        name = f'route.transits[{index}]'
        data = _mapping(value, name)
        return cls(
            transit_id=_string(data.get('id'), f'{name}.id'),
            before_waypoint_id=_string(
                data.get('before_waypoint_id'), f'{name}.before_waypoint_id'),
            pose=Pose2D.from_mapping(data.get('pose'), f'{name}.pose'),
            goal_timeout_s=_positive(
                data.get('goal_timeout_s', 120.0), f'{name}.goal_timeout_s'),
        )


@dataclass(frozen=True)
class Mission:
    """Validated single-corridor mission."""

    schema_version: int
    mission_id: str
    field: FieldBinding
    map_binding: MapBinding
    world_to_map: tuple[float, float, float]
    work_side_reference: str
    corridor: CorridorBounds
    home: Pose2D
    return_home: bool
    footprint: tuple[float, float, float]
    max_unknown_cells: int
    waypoints: tuple[HarvestWaypoint, ...]
    route_layout: str = 'shared_pairs'
    route_transits: tuple[TransitWaypoint, ...] = ()

    @property
    def enabled_waypoints(self) -> tuple[HarvestWaypoint, ...]:
        """Return enabled tasks in their file order."""
        return tuple(point for point in self.waypoints if point.enabled)


def load_mission(path: str | Path) -> Mission:
    """Load and fully validate a mission YAML file."""
    mission_path = Path(path)
    try:
        raw = yaml.safe_load(mission_path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise MissionConfigError(
            f'unable to read mission {mission_path}: {exc}') from exc
    data = _mapping(raw, 'mission')
    schema_version = _integer(
        data.get('schema_version'), 'schema_version', minimum=1)
    if schema_version != 1:
        raise MissionConfigError(
            f'unsupported schema_version {schema_version}; expected 1')
    field = FieldBinding.from_mapping(data.get('field'))
    map_binding = MapBinding.from_mapping(data.get('map'))
    alignment = _mapping(data.get('alignment'), 'alignment')
    transform = _sequence(
        alignment.get('world_to_map'), 'alignment.world_to_map')
    if len(transform) != 3:
        raise MissionConfigError(
            'alignment.world_to_map must contain x, y, yaw')
    world_to_map = tuple(
        _finite(item, f'alignment.world_to_map[{index}]')
        for index, item in enumerate(transform)
    )
    route = _mapping(data.get('route'), 'route')
    route_layout = _string(
        route.get('layout', 'shared_pairs'), 'route.layout')
    if route_layout not in VALID_ROUTE_LAYOUTS:
        raise MissionConfigError(
            'route.layout must be shared_pairs or independent_side_stops')
    corridor = CorridorBounds.from_mapping(route.get('corridor'))
    home_data = _mapping(route.get('home'), 'route.home')
    home = Pose2D.from_mapping(home_data.get('pose'), 'route.home.pose')
    corridor.validate_pose(home, 'route.home.pose')
    footprint_data = _mapping(route.get('footprint'), 'route.footprint')
    footprint = (
        _positive(footprint_data.get('front_m'),
                  'route.footprint.front_m'),
        _positive(footprint_data.get('rear_m'),
                  'route.footprint.rear_m'),
        _positive(footprint_data.get('half_width_m'),
                  'route.footprint.half_width_m'),
    )
    waypoint_raw = _sequence(route.get('waypoints'), 'route.waypoints')
    waypoints = tuple(
        HarvestWaypoint.from_mapping(value, index)
        for index, value in enumerate(waypoint_raw)
    )
    if not waypoints:
        raise MissionConfigError('route.waypoints cannot be empty')
    transit_raw = _sequence(route.get('transits', []), 'route.transits')
    route_transits = tuple(
        TransitWaypoint.from_mapping(value, index)
        for index, value in enumerate(transit_raw)
    )
    result = Mission(
        schema_version=schema_version,
        mission_id=_string(data.get('mission_id'), 'mission_id'),
        field=field,
        map_binding=map_binding,
        world_to_map=world_to_map,
        work_side_reference=_string(
            data.get('work_side_reference'), 'work_side_reference'),
        corridor=corridor,
        home=home,
        return_home=_boolean(
            route.get('return_home'), 'route.return_home'),
        footprint=footprint,
        max_unknown_cells=_integer(
            route.get('max_unknown_cells'),
            'route.max_unknown_cells'),
        waypoints=waypoints,
        route_layout=route_layout,
        route_transits=route_transits,
    )
    _validate_cross_references(result)
    return result


def _validate_cross_references(mission: Mission) -> None:
    if mission.map_binding.frame_id != 'map':
        raise MissionConfigError('map.frame_id must be map')
    if mission.work_side_reference != 'base_footprint_at_goal':
        raise MissionConfigError(
            'work_side_reference must be base_footprint_at_goal')
    if len(mission.field.selected_rows) != 2:
        raise MissionConfigError(
            'single-corridor missions require exactly two selected rows')
    expected_excluded = (
        mission.field.rows * mission.field.plants_per_row -
        len(mission.field.selected_rows) * mission.field.plants_per_row)
    if mission.field.excluded_plant_count != expected_excluded:
        raise MissionConfigError(
            'field.excluded_plant_count disagrees with the field layout')
    ids = [point.waypoint_id for point in mission.waypoints]
    plants = [point.plant.plant_id for point in mission.waypoints]
    sequences = [point.sequence for point in mission.waypoints]
    if len(set(ids)) != len(ids):
        raise MissionConfigError('waypoint IDs must be unique')
    transit_ids = [point.transit_id for point in mission.route_transits]
    if len(set(transit_ids)) != len(transit_ids):
        raise MissionConfigError('transit IDs must be unique')
    if set(transit_ids) & set(ids):
        raise MissionConfigError('transit IDs must differ from waypoint IDs')
    enabled_ids = {point.waypoint_id for point in mission.enabled_waypoints}
    for transit in mission.route_transits:
        if transit.before_waypoint_id not in enabled_ids:
            raise MissionConfigError(
                f'transit {transit.transit_id} must precede an enabled '
                'harvest waypoint')
        mission.corridor.validate_pose(
            transit.pose, f'transit {transit.transit_id}.pose')
        if abs(normalized_angle(transit.pose.yaw)) > 1e-8:
            raise MissionConfigError('transit poses require yaw=0')
    if len(set(plants)) != len(plants):
        raise MissionConfigError('each plant must appear exactly once')
    if len(set(sequences)) != len(sequences):
        raise MissionConfigError('waypoint sequences must be unique')
    if sorted(sequences) != list(range(1, len(sequences) + 1)):
        raise MissionConfigError(
            'waypoint sequences must be contiguous from 1')
    if sequences != list(range(1, len(sequences) + 1)):
        raise MissionConfigError(
            'waypoints must be stored in ascending sequence order')
    expected = len(mission.field.selected_rows) * mission.field.plants_per_row
    if len(mission.waypoints) != expected:
        raise MissionConfigError(
            f'expected {expected} plant waypoints for selected rows, '
            f'found {len(mission.waypoints)}')
    if mission.route_layout == 'independent_side_stops':
        stops = [point.stop_id for point in mission.waypoints]
        if len(set(stops)) != len(stops):
            raise MissionConfigError(
                'independent side stops require unique stop IDs')
        poses = [mission.home] + [point.pose for point in mission.waypoints]
        if (abs(normalized_angle(mission.corridor.required_yaw)) > 1e-8 or
                any(abs(normalized_angle(pose.yaw)) > 1e-8 for pose in poses)):
            raise MissionConfigError(
                'independent side stops require home and station yaw=0')
    stop_poses: dict[str, Pose2D] = {}
    stop_sides: dict[str, set[str]] = {}
    stop_members: dict[str, list[HarvestWaypoint]] = {}
    plant_slots: set[tuple[int, int]] = set()
    transform_x, transform_y, transform_yaw = mission.world_to_map
    transform_cosine = math.cos(transform_yaw)
    transform_sine = math.sin(transform_yaw)
    for point in mission.waypoints:
        mission.corridor.validate_pose(
            point.pose, f'waypoint {point.waypoint_id}.base_pose')
        if point.plant.row_id not in mission.field.selected_rows:
            raise MissionConfigError(
                f'{point.plant.plant_id} is outside selected rows')
        if not 0 <= point.plant.index_in_row < mission.field.plants_per_row:
            raise MissionConfigError(
                f'{point.plant.plant_id} has an invalid index_in_row')
        slot = (point.plant.row_id, point.plant.index_in_row)
        if slot in plant_slots:
            raise MissionConfigError(
                'each selected row/index slot must appear exactly once')
        plant_slots.add(slot)
        expected_map_x = (
            transform_x + transform_cosine * point.plant.pose_world.x -
            transform_sine * point.plant.pose_world.y)
        expected_map_y = (
            transform_y + transform_sine * point.plant.pose_world.x +
            transform_cosine * point.plant.pose_world.y)
        expected_map_yaw = normalized_angle(
            point.plant.pose_world.yaw + transform_yaw)
        if (not math.isclose(
                point.plant.pose_map.x, expected_map_x,
                rel_tol=0.0, abs_tol=1e-5) or
                not math.isclose(
                    point.plant.pose_map.y, expected_map_y,
                    rel_tol=0.0, abs_tol=1e-5) or
                abs(normalized_angle(
                    point.plant.pose_map.yaw - expected_map_yaw)) > 1e-5):
            raise MissionConfigError(
                f'{point.plant.plant_id} pose_map disagrees with world_to_map')
        lateral_delta = point.plant.pose_map.y - point.pose.y
        expected_side = 'left' if lateral_delta > 0.0 else 'right'
        if abs(lateral_delta) < 1e-6 or point.work_side != expected_side:
            raise MissionConfigError(
                f'{point.waypoint_id} work_side disagrees with plant pose')
        old_pose = stop_poses.setdefault(point.stop_id, point.pose)
        if old_pose != point.pose:
            raise MissionConfigError(
                f'shared stop {point.stop_id} has inconsistent poses')
        sides = stop_sides.setdefault(point.stop_id, set())
        if point.work_side in sides:
            raise MissionConfigError(
                f'shared stop {point.stop_id} repeats work side')
        sides.add(point.work_side)
        stop_members.setdefault(point.stop_id, []).append(point)
    expected_slots = {
        (row, index)
        for row in mission.field.selected_rows
        for index in range(mission.field.plants_per_row)
    }
    if plant_slots != expected_slots:
        raise MissionConfigError(
            'mission does not cover every selected row/index slot')
    if mission.route_layout == 'independent_side_stops':
        count = mission.field.plants_per_row
        left_leg = mission.waypoints[:count]
        right_leg = mission.waypoints[count:]
        if (any(point.work_side != 'left' for point in left_leg) or
                any(point.work_side != 'right' for point in right_leg) or
                [point.plant.index_in_row for point in left_leg] !=
                list(range(count)) or
                [point.plant.index_in_row for point in right_leg] !=
                list(reversed(range(count))) or
                len({point.plant.row_id for point in left_leg}) != 1 or
                len({point.plant.row_id for point in right_leg}) != 1):
            raise MissionConfigError(
                'independent side stops require ascending left-side '
                'outbound and descending right-side return plant columns')
        return
    for stop_id, members in stop_members.items():
        if len(members) != 2 or stop_sides[stop_id] != {'left', 'right'}:
            raise MissionConfigError(
                f'shared stop {stop_id} must contain one task per work side')
        if ({item.plant.row_id for item in members} !=
                set(mission.field.selected_rows) or
                len({item.plant.index_in_row for item in members}) != 1):
            raise MissionConfigError(
                f'shared stop {stop_id} must pair the same column in both '
                'rows')
    ordered = sorted(mission.waypoints, key=lambda point: point.sequence)
    for column in range(mission.field.plants_per_row):
        pair = ordered[column * 2:(column + 1) * 2]
        if (len(pair) != 2 or pair[0].stop_id != pair[1].stop_id or
                [point.work_side for point in pair] != ['left', 'right'] or
                any(point.plant.index_in_row != column for point in pair)):
            raise MissionConfigError(
                'each sequential stop must pair left/right tasks for the '
                'same ascending plant column')


def sha256_file(path: str | Path) -> str:
    """Return a streaming SHA-256 digest for an asset."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()
