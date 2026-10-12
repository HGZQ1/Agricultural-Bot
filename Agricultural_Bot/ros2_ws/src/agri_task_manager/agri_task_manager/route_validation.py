"""Saved-map identity and footprint validation for field routes."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import BinaryIO

import yaml

from .waypoint_schema import (
    Mission, MissionConfigError, normalized_angle, Pose2D, sha256_file)


FREE = 'free'
OCCUPIED = 'occupied'
UNKNOWN = 'unknown'
OUTSIDE = 'outside'


def _read_token(stream: BinaryIO) -> bytes:
    token = bytearray()
    while True:
        byte = stream.read(1)
        if not byte:
            raise MissionConfigError('truncated PGM header')
        if byte in b' \t\r\n':
            continue
        if byte == b'#':
            stream.readline()
            continue
        token.extend(byte)
        break
    while True:
        byte = stream.read(1)
        if not byte or byte in b' \t\r\n':
            break
        if byte == b'#':
            stream.readline()
            break
        token.extend(byte)
    return bytes(token)


def _load_pgm(path: Path) -> tuple[int, int, int, bytes]:
    try:
        with path.open('rb') as stream:
            magic = _read_token(stream)
            width = int(_read_token(stream))
            height = int(_read_token(stream))
            maximum = int(_read_token(stream))
            if magic == b'P5':
                pixels = stream.read()
            elif magic == b'P2':
                values = []
                while len(values) < width * height:
                    try:
                        values.append(int(_read_token(stream)))
                    except MissionConfigError:
                        break
                pixels = bytes(values)
            else:
                raise MissionConfigError(
                    f'{path} is not a PGM image (expected P5 or P2)')
    except OSError as exc:
        raise MissionConfigError(
            f'unable to read map image {path}: {exc}') from exc
    if width < 1 or height < 1 or not 1 <= maximum <= 255:
        raise MissionConfigError(f'invalid PGM header in {path}')
    expected = width * height
    if len(pixels) != expected:
        raise MissionConfigError(
            f'{path} contains {len(pixels)} pixels; expected {expected}')
    return width, height, maximum, pixels


@dataclass(frozen=True)
class FootprintReport:
    """Cell intersections and base-origin obstacle distance at one pose."""

    free_cells: int
    unknown_cells: int
    occupied_cells: int
    outside_cells: int
    nearest_occupied_m: float | None

    @property
    def clear(self) -> bool:
        """Return whether the footprint avoids occupied and outside cells."""
        return self.occupied_cells == 0 and self.outside_cells == 0


@dataclass(frozen=True)
class MapGrid:
    """ROS map-server PGM decoded into bottom-left map-grid coordinates."""

    yaml_path: Path
    image_path: Path
    width: int
    height: int
    maximum: int
    pixels: bytes
    resolution: float
    origin: tuple[float, float, float]
    negate: bool
    occupied_thresh: float
    free_thresh: float
    mode: str

    @classmethod
    def from_yaml(cls, yaml_path: str | Path) -> 'MapGrid':
        """Load map-server YAML and its PGM image."""
        path = Path(yaml_path)
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8'))
        except (OSError, yaml.YAMLError) as exc:
            raise MissionConfigError(
                f'unable to read map YAML {path}: {exc}') from exc
        if not isinstance(data, dict):
            raise MissionConfigError('map YAML must be a mapping')
        image_name = data.get('image')
        if not isinstance(image_name, str) or not image_name:
            raise MissionConfigError('map YAML image must be a path')
        image = Path(image_name)
        if not image.is_absolute():
            image = path.parent / image
        try:
            resolution = float(data['resolution'])
            origin_raw = data['origin']
            origin = tuple(float(value) for value in origin_raw)
            negate = bool(int(data.get('negate', 0)))
            occupied_thresh = float(data.get('occupied_thresh', 0.65))
            free_thresh = float(data.get('free_thresh', 0.196))
            mode = str(data.get('mode', 'trinary'))
        except (KeyError, TypeError, ValueError) as exc:
            raise MissionConfigError('invalid numeric map YAML field') from exc
        if (resolution <= 0.0 or len(origin) != 3 or
                not 0.0 <= free_thresh < occupied_thresh <= 1.0):
            raise MissionConfigError('invalid map geometry or thresholds')
        if mode != 'trinary':
            raise MissionConfigError('only trinary map mode is supported')
        width, height, maximum, pixels = _load_pgm(image)
        return cls(
            yaml_path=path, image_path=image, width=width, height=height,
            maximum=maximum, pixels=pixels, resolution=resolution,
            origin=origin, negate=negate, occupied_thresh=occupied_thresh,
            free_thresh=free_thresh, mode=mode,
        )

    def _grid_index(self, x: int, y: int) -> int | None:
        if not 0 <= x < self.width or not 0 <= y < self.height:
            return None
        return (self.height - 1 - y) * self.width + x

    def cell_center(self, x: int, y: int) -> tuple[float, float]:
        """Return the map-frame centre of one bottom-left grid cell."""
        local_x = (x + 0.5) * self.resolution
        local_y = (y + 0.5) * self.resolution
        cosine = math.cos(self.origin[2])
        sine = math.sin(self.origin[2])
        return (
            self.origin[0] + cosine * local_x - sine * local_y,
            self.origin[1] + sine * local_x + cosine * local_y,
        )

    def cell_status(self, x: int, y: int) -> str:
        """Classify a bottom-left grid cell using map-server semantics."""
        index = self._grid_index(x, y)
        if index is None:
            return OUTSIDE
        normalized = self.pixels[index] / self.maximum
        occupancy = normalized if self.negate else 1.0 - normalized
        if occupancy > self.occupied_thresh:
            return OCCUPIED
        if occupancy < self.free_thresh:
            return FREE
        return UNKNOWN

    def map_to_grid(self, x: float, y: float) -> tuple[float, float]:
        """Convert map-frame coordinates into continuous grid coordinates."""
        dx = x - self.origin[0]
        dy = y - self.origin[1]
        cosine = math.cos(self.origin[2])
        sine = math.sin(self.origin[2])
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        return local_x / self.resolution, local_y / self.resolution

    def occupied_centres(self) -> list[tuple[float, float]]:
        """Return the map-frame centres of all occupied cells."""
        result = []
        for y in range(self.height):
            for x in range(self.width):
                if self.cell_status(x, y) == OCCUPIED:
                    result.append(self.cell_center(x, y))
        return result

    def occupancy_bytes(self) -> bytes:
        """Return ROS OccupancyGrid data in bottom-left row-major order."""
        values = {FREE: 0, OCCUPIED: 100, UNKNOWN: 255}
        return bytes(
            values[self.cell_status(x, y)]
            for y in range(self.height)
            for x in range(self.width)
        )

    def occupancy_sha256(self) -> str:
        """Return a digest comparable with a live OccupancyGrid message."""
        return hashlib.sha256(self.occupancy_bytes()).hexdigest()

    @staticmethod
    def _obb_intersects(
        first_center: tuple[float, float],
        first_axes: tuple[tuple[float, float], tuple[float, float]],
        first_extents: tuple[float, float],
        second_center: tuple[float, float],
        second_axes: tuple[tuple[float, float], tuple[float, float]],
        second_extents: tuple[float, float],
    ) -> bool:
        """Return whether two oriented rectangles overlap or touch."""
        delta = (
            second_center[0] - first_center[0],
            second_center[1] - first_center[1],
        )
        for axis in first_axes + second_axes:
            separation = abs(delta[0] * axis[0] + delta[1] * axis[1])
            first_radius = sum(
                extent * abs(vector[0] * axis[0] + vector[1] * axis[1])
                for vector, extent in zip(first_axes, first_extents)
            )
            second_radius = sum(
                extent * abs(vector[0] * axis[0] + vector[1] * axis[1])
                for vector, extent in zip(second_axes, second_extents)
            )
            if separation > first_radius + second_radius + 1e-12:
                return False
        return True

    def footprint_report(
        self,
        pose: Pose2D,
        front_m: float,
        rear_m: float,
        half_width_m: float,
        occupied_centres: list[tuple[float, float]] | None = None,
    ) -> FootprintReport:
        """
        Classify every grid-cell square touched by the footprint.

        This uses an oriented-box intersection test.  Checking only whether a
        cell centre lies inside the footprint can miss collisions along an
        edge by as much as half a map cell.
        """
        corners = []
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        for local_x in (-rear_m, front_m):
            for local_y in (-half_width_m, half_width_m):
                corners.append((
                    pose.x + cosine * local_x - sine * local_y,
                    pose.y + sine * local_x + cosine * local_y,
                ))
        grid_corners = [self.map_to_grid(x, y) for x, y in corners]
        x_min = math.floor(min(point[0] for point in grid_corners)) - 1
        x_max = math.ceil(max(point[0] for point in grid_corners)) + 1
        y_min = math.floor(min(point[1] for point in grid_corners)) - 1
        y_max = math.ceil(max(point[1] for point in grid_corners)) + 1
        counts = {FREE: 0, UNKNOWN: 0, OCCUPIED: 0, OUTSIDE: 0}
        base_axes = ((cosine, sine), (-sine, cosine))
        base_extents = ((front_m + rear_m) / 2.0, half_width_m)
        centre_offset = (front_m - rear_m) / 2.0
        base_center = (
            pose.x + cosine * centre_offset,
            pose.y + sine * centre_offset,
        )
        map_cosine = math.cos(self.origin[2])
        map_sine = math.sin(self.origin[2])
        cell_axes = (
            (map_cosine, map_sine),
            (-map_sine, map_cosine),
        )
        cell_extents = (self.resolution / 2.0, self.resolution / 2.0)
        for grid_y in range(y_min, y_max + 1):
            for grid_x in range(x_min, x_max + 1):
                cell_center = self.cell_center(grid_x, grid_y)
                if not self._obb_intersects(
                        base_center, base_axes, base_extents,
                        cell_center, cell_axes, cell_extents):
                    continue
                counts[self.cell_status(grid_x, grid_y)] += 1
        centres = (occupied_centres if occupied_centres is not None
                   else self.occupied_centres())
        nearest = None
        if centres:
            nearest = min(
                math.hypot(pose.x - x, pose.y - y) for x, y in centres)
        return FootprintReport(
            free_cells=counts[FREE],
            unknown_cells=counts[UNKNOWN],
            occupied_cells=counts[OCCUPIED],
            outside_cells=counts[OUTSIDE],
            nearest_occupied_m=nearest,
        )


def verify_mission_map(
    mission: Mission,
    map_yaml_path: str | Path,
    keepout_yaml_path: str | Path | None = None,
) -> tuple[MapGrid, dict[str, FootprintReport]]:
    """Verify map/mask identity, stops, adjustments, and route segments."""
    grid = MapGrid.from_yaml(map_yaml_path)
    binding = mission.map_binding
    actual_hash = sha256_file(grid.image_path)
    if actual_hash != binding.image_sha256:
        raise MissionConfigError(
            'map image SHA-256 does not match mission binding')
    if not math.isclose(
            grid.resolution, binding.resolution_m,
            rel_tol=0.0, abs_tol=1e-12):
        raise MissionConfigError(
            'map resolution does not match mission binding')
    if any(not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9)
           for actual, expected in zip(grid.origin, binding.origin)):
        raise MissionConfigError('map origin does not match mission binding')
    if grid.width != binding.width_px or grid.height != binding.height_px:
        raise MissionConfigError('map dimensions do not match mission binding')
    if (grid.mode != binding.mode or grid.negate != binding.negate or
            not math.isclose(
                grid.occupied_thresh, binding.occupied_thresh,
                rel_tol=0.0, abs_tol=1e-12) or
            not math.isclose(
                grid.free_thresh, binding.free_thresh,
                rel_tol=0.0, abs_tol=1e-12)):
        raise MissionConfigError(
            'map occupancy semantics do not match binding')
    if grid.occupancy_sha256() != binding.occupancy_sha256:
        raise MissionConfigError(
            'map OccupancyGrid SHA-256 does not match mission binding')

    if keepout_yaml_path is None:
        keepout_name = Path(binding.keepout_yaml_relative_path).name
        keepout_yaml_path = grid.yaml_path.parent / keepout_name
    keepout = MapGrid.from_yaml(keepout_yaml_path)
    if sha256_file(keepout.image_path) != binding.keepout_image_sha256:
        raise MissionConfigError(
            'keepout image SHA-256 does not match mission binding')
    if keepout.occupancy_sha256() != binding.keepout_occupancy_sha256:
        raise MissionConfigError(
            'keepout OccupancyGrid SHA-256 does not match mission binding')
    if (keepout.width != grid.width or keepout.height != grid.height or
            not math.isclose(
                keepout.resolution, grid.resolution,
                rel_tol=0.0, abs_tol=1e-12) or
            any(not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9)
                for actual, expected in zip(keepout.origin, grid.origin))):
        raise MissionConfigError(
            'keepout mask geometry does not match the saved map')

    front, rear, half_width = mission.footprint
    occupied = grid.occupied_centres()
    keepout_occupied = keepout.occupied_centres()

    def check_pose(pose: Pose2D, label: str) -> FootprintReport:
        report = grid.footprint_report(
            pose, front, rear, half_width, occupied)
        if not report.clear:
            raise MissionConfigError(
                f'{label} footprint intersects {report.occupied_cells} '
                f'occupied and {report.outside_cells} outside cells')
        if report.unknown_cells > mission.max_unknown_cells:
            raise MissionConfigError(
                f'{label} footprint contains {report.unknown_cells} unknown '
                f'cells; maximum is {mission.max_unknown_cells}')
        mask_report = keepout.footprint_report(
            pose, front, rear, half_width, keepout_occupied)
        if not mask_report.clear or mask_report.unknown_cells:
            raise MissionConfigError(
                f'{label} footprint intersects the keepout mask')
        return report

    check_pose(mission.home, 'home')
    reports = {}
    ordered_poses = [mission.home]
    enabled = sorted(
        mission.enabled_waypoints, key=lambda point: point.sequence)
    for point in enabled:
        for transit in mission.route_transits:
            if transit.before_waypoint_id != point.waypoint_id:
                continue
            label = f'transit:{transit.transit_id}'
            reports[label] = check_pose(transit.pose, label)
            ordered_poses.append(transit.pose)
        if point.stop_id not in reports:
            report = check_pose(point.pose, point.stop_id)
            reports[point.stop_id] = report
            ordered_poses.append(point.pose)

        # The mission advertises these as allowed adjustment limits.  Validate
        # the whole sampled box, including combined translation and yaw.
        adjust = point.micro_adjust
        for longitudinal in (-adjust.longitudinal_m, 0.0,
                             adjust.longitudinal_m):
            for lateral in (-adjust.lateral_m, 0.0, adjust.lateral_m):
                for yaw_delta in (-adjust.yaw_rad, 0.0, adjust.yaw_rad):
                    pose_cosine = math.cos(point.pose.yaw)
                    pose_sine = math.sin(point.pose.yaw)
                    adjusted = Pose2D(
                        point.pose.x + pose_cosine * longitudinal -
                        pose_sine * lateral,
                        point.pose.y + pose_sine * longitudinal +
                        pose_cosine * lateral,
                        normalized_angle(point.pose.yaw + yaw_delta),
                    )
                    check_pose(adjusted, (
                        f'{point.waypoint_id} micro-adjust envelope'))

    # Check the actual preset order, including side-change transit goals.
    # Return-home sends a direct Nav2 goal rather than replaying the outbound
    # stops. Check that direct connection from every anchor, because the
    # operator can also request it after canceling navigation at a work stop.
    segments = [(first, second, 'preset route segment')
                for first, second in
                zip(ordered_poses, ordered_poses[1:])]
    segments.extend((pose, mission.home, 'return-home route segment')
                    for pose in ordered_poses[1:])
    for first, second, label in segments:
        distance = math.hypot(second.x - first.x, second.y - first.y)
        sample_count = max(
            1, math.ceil(distance / (grid.resolution * 0.5)))
        yaw_delta = normalized_angle(second.yaw - first.yaw)
        for index in range(sample_count + 1):
            ratio = index / sample_count
            pose = Pose2D(
                first.x + (second.x - first.x) * ratio,
                first.y + (second.y - first.y) * ratio,
                normalized_angle(first.yaw + yaw_delta * ratio),
            )
            check_pose(pose, label)
    return grid, reports
