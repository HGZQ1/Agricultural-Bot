"""Offline safety regressions using real, independent natural-scene candidates.

No ROS nodes, actions, default asset edits, or mocked geometry validators are
used.  The collision blueprint and harvestable world are built under /tmp.
"""

import hashlib
import importlib.util
import math
from pathlib import Path
import shutil
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
BACKUP = ROOT / 'artifacts/harvest_demo_profile_20261013/previous_default'
SDF = BACKUP / 'tomato_field_default.sdf'
METADATA = BACKUP / 'tomato_field_default.json'
MISSION = BACKUP / 'tomato_field_default_single_corridor.yaml'
LOCALIZATION_MAP = ROOT / 'ros2_ws/src/agri_navigation/maps/tomato_field_default.yaml'
MODELS = ROOT / 'sim_ws/src/agri_greenhouse_worlds/models'
SOURCE_MISSION = yaml.safe_load(MISSION.read_text())
KEEPOUT = (ROOT / 'ros2_ws/src/agri_navigation' /
           SOURCE_MISSION['map']['keepout_yaml_relative_path'])

sys.path.insert(0, str(ROOT / 'ros2_ws/src/agri_task_manager'))
from agri_task_manager.route_validation import MapGrid, UNKNOWN, verify_mission_map
from agri_task_manager.waypoint_schema import MissionConfigError, Pose2D, load_mission


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


collision = load_script('prepare_collision_navigation')
close = load_script('prepare_close_harvest')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot(directory):
    """Include every output, so failed preparation cannot leave partial assets."""
    return {str(path.relative_to(directory)): digest(path)
            for path in directory.rglob('*') if path.is_file()}


@pytest.fixture(scope='module')
def real_candidate_source(tmp_path_factory):
    directory = tmp_path_factory.mktemp('natural_close_source') / 'candidate'
    assert directory.is_relative_to(Path('/tmp'))
    protected = [SDF, METADATA, MISSION, LOCALIZATION_MAP,
                 MapGrid.from_yaml(LOCALIZATION_MAP).image_path,
                 KEEPOUT, MapGrid.from_yaml(KEEPOUT).image_path]
    before = {path: digest(path) for path in protected}
    collision.prepare(
        SDF, METADATA, LOCALIZATION_MAP, MISSION, directory, MODELS,
        protected_dirs=[ROOT / 'sim_ws/src', ROOT / 'ros2_ws/src', BACKUP])
    assert before == {path: digest(path) for path in protected}
    yield directory
    assert before == {path: digest(path) for path in protected}


@pytest.fixture
def candidate(tmp_path, real_candidate_source):
    directory = tmp_path / 'candidate'
    assert directory.is_relative_to(Path('/tmp'))
    shutil.copytree(real_candidate_source, directory)
    return directory


def assert_clear(pose, mission, grid, keepout, occupied, keepout_occupied):
    report = grid.footprint_report(pose, *mission.footprint, occupied)
    mask = keepout.footprint_report(pose, *mission.footprint, keepout_occupied)
    assert report.occupied_cells == report.outside_cells == 0
    assert report.unknown_cells <= mission.max_unknown_cells
    assert mask.occupied_cells == mask.outside_cells == mask.unknown_cells == 0


def test_default_first_station_preserves_assets_and_passes_real_geometry(candidate):
    inputs = [candidate / 'tomato_field_collision_navigation.sdf',
              candidate / 'tomato_field.json', candidate / 'navigation_collision.yaml',
              candidate / 'navigation_collision.pgm', KEEPOUT,
              MapGrid.from_yaml(KEEPOUT).image_path]
    input_hashes = {path: digest(path) for path in inputs}
    result = close.prepare(candidate, .75)  # Exercise the public default mode.
    static = load_mission(candidate / 'close_mission_static.yaml')
    grid, reports = verify_mission_map(
        static, candidate / 'navigation_collision.yaml', KEEPOUT)
    keepout = MapGrid.from_yaml(KEEPOUT)
    localization = MapGrid.from_yaml(LOCALIZATION_MAP)
    assert len(static.waypoints) == 20
    assert [point.waypoint_id for point in static.enabled_waypoints] == ['harvest_tomato_20']
    assert static.route_transits == ()
    assert static.return_home is False
    assert set(reports) == {'stop_00_left'}
    assert static.footprint == pytest.approx((.495, .535, .395))
    point = static.enabled_waypoints[0]
    assert (point.pose.x, point.pose.y, point.pose.yaw) == pytest.approx((1.18, .75, 0.))
    assert result['enabled_station_count'] == 1
    assert result['full_route_validated'] is False
    assert result['runtime_navigation_and_harvest_passed'] is False

    # Independently enumerate the advertised adjustment box and direct outbound
    # / return segments with the full padded body, including edge-intersected cells.
    occupied, keepout_occupied = grid.occupied_centres(), keepout.occupied_centres()
    adjust = point.micro_adjust
    for dx in (-adjust.longitudinal_m, 0., adjust.longitudinal_m):
        for dy in (-adjust.lateral_m, 0., adjust.lateral_m):
            for yaw in (-adjust.yaw_rad, 0., adjust.yaw_rad):
                assert_clear(Pose2D(point.pose.x + dx, point.pose.y + dy, yaw),
                             static, grid, keepout, occupied, keepout_occupied)
    samples = math.ceil(math.hypot(point.pose.x - static.home.x,
                                   point.pose.y - static.home.y) / (grid.resolution / 2.))
    for first, second in ((static.home, point.pose), (point.pose, static.home)):
        for index in range(samples + 1):
            ratio = index / samples
            pose = Pose2D(first.x + (second.x - first.x) * ratio,
                          first.y + (second.y - first.y) * ratio, 0.)
            assert_clear(pose, static, grid, keepout, occupied, keepout_occupied)

    assert (grid.width, grid.height, grid.resolution, grid.origin) == (
        localization.width, localization.height, localization.resolution, localization.origin)
    old_unknown = bytes(status == 255 for status in localization.occupancy_bytes())
    new_unknown = bytes(status == 255 for status in grid.occupancy_bytes())
    assert old_unknown.count(1) > 0
    assert hashlib.sha256(old_unknown).hexdigest() == hashlib.sha256(new_unknown).hexdigest()
    for y in range(grid.height):
        for x in range(grid.width):
            if localization.cell_status(x, y) == UNKNOWN:
                index = (grid.height - 1 - y) * grid.width + x
                assert grid.pixels[index] == localization.pixels[index]
    assert len(grid.occupied_centres()) < len(localization.occupied_centres())
    assert static.map_binding.keepout_image_sha256 == SOURCE_MISSION['map']['keepout_image_sha256']
    assert static.map_binding.keepout_occupancy_sha256 == SOURCE_MISSION['map']['keepout_occupancy_sha256']
    assert static.map_binding.keepout_image_sha256 == digest(keepout.image_path)
    assert static.map_binding.keepout_occupancy_sha256 == keepout.occupancy_sha256()
    assert (candidate / 'tomato_field.json').read_bytes() == METADATA.read_bytes()
    assert (candidate / 'harvestable/tomato_field.json').read_bytes() == METADATA.read_bytes()
    assert result['natural_metadata_sha256'] == digest(METADATA)
    assert static.field.metadata_sha256 == digest(METADATA)
    assert input_hashes == {path: digest(path) for path in inputs}

    # Conversion must carry the same collision map / keepout bindings through
    # to the runnable harvest mission, with an actual independent SDF binding.
    harvest_path = candidate / 'harvestable/harvest_mission.yaml'
    harvest = load_mission(harvest_path)
    verify_mission_map(harvest, candidate / 'navigation_collision.yaml', KEEPOUT)
    assert [item.waypoint_id for item in harvest.enabled_waypoints] == ['harvest_tomato_20']
    assert harvest.map_binding == static.map_binding
    assert harvest.field.sdf_sha256 == digest(
        candidate / 'harvestable/tomato_field_harvestable.sdf')


def test_rejected_full_route_cannot_replace_previous_valid_outputs(candidate):
    close.prepare(candidate, .75)
    valid_files = snapshot(candidate)
    with pytest.raises(MissionConfigError, match='preset route segment footprint intersects'):
        close.prepare(candidate, .75, first_station_only=False)
    assert snapshot(candidate) == valid_files
    mission = load_mission(candidate / 'close_mission_static.yaml')
    assert [point.waypoint_id for point in mission.enabled_waypoints] == ['harvest_tomato_20']
    verify_mission_map(mission, candidate / 'navigation_collision.yaml', KEEPOUT)


def test_changed_natural_metadata_is_rejected_before_valid_outputs_are_replaced(candidate):
    close.prepare(candidate, .75)
    metadata = candidate / 'tomato_field.json'
    metadata.write_bytes(metadata.read_bytes() + b'\n')
    # Even valid JSON with an innocuous byte change must invalidate its binding.
    before = snapshot(candidate)
    with pytest.raises(ValueError, match='natural fruit metadata was changed'):
        close.prepare(candidate, .75)
    assert snapshot(candidate) == before
    assert METADATA.read_bytes() != metadata.read_bytes()
