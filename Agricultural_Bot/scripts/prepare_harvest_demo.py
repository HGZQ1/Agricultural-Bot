#!/usr/bin/env python3
"""Create a harvestable world and rebind a copied mission to its exact SDF."""

import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml


def digest(path):
    """Return the full source-file binding."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    """Keep saved map bindings and geometry while adding simulated fruit dynamics."""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path,
                        default=root / 'artifacts/harvest_demo/current')
    parser.add_argument('--source-sdf', type=Path, default=root /
                        'sim_ws/src/agri_greenhouse_worlds/worlds/tomato_field_default.sdf')
    parser.add_argument('--metadata', type=Path, default=root /
                        'sim_ws/src/agri_greenhouse_worlds/worlds/tomato_field_default.json')
    parser.add_argument('--mission', type=Path, default=root /
                        'ros2_ws/src/agri_task_manager/config/missions/'
                        'tomato_field_default_single_corridor.yaml')
    args = parser.parse_args()
    source_sdf, metadata, mission_path = (
        path.resolve() for path in (args.source_sdf, args.metadata, args.mission))
    source_mission = yaml.safe_load(mission_path.read_text())
    if (source_mission['field']['sdf_sha256'] != digest(source_sdf) or
            source_mission['field']['metadata_sha256'] != digest(metadata)):
        parser.error('source world/metadata do not match the source mission binding')
    destination = args.output_dir.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    world_file = destination / 'tomato_field_harvestable.sdf'
    metadata_file = destination / 'tomato_field.json'
    mission_file = destination / 'harvest_mission.yaml'
    first_mission_file = destination / 'harvest_mission_first_station.yaml'
    manifest_file = destination / 'manifest.json'
    protected = {source_sdf, metadata, mission_path}
    if any(path.resolve() in protected for path in (
            world_file, metadata_file, mission_file, first_mission_file, manifest_file)):
        parser.error('output must not overwrite a source asset')
    converter = root / 'sim_ws/src/agri_harvest_sim/scripts/make_harvestable_world.py'
    spec = importlib.util.spec_from_file_location('harvest_world_converter', converter)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    converted = module.convert(ET.parse(source_sdf).getroot(),
                               json.loads(metadata.read_text()), 'agri_robot')
    ET.indent(converted, space='  ')
    ET.ElementTree(converted).write(world_file, encoding='utf-8', xml_declaration=True)
    shutil.copy2(metadata, metadata_file)
    source_mission['field']['sdf_sha256'] = digest(world_file)
    source_mission['field']['metadata_sha256'] = digest(metadata_file)
    source_mission['mission_id'] += '_harvestable'
    source_mission['description'] += ' Simulated fruit dynamics with preserved initial geometry.'
    source_mission['robot']['note'] = (
        'Geometry-derived grasp TCP and bounded arm worker; runtime IK/FOV required per target.')
    mission_file.write_text(yaml.safe_dump(source_mission, sort_keys=False))
    first_mission = copy.deepcopy(source_mission)
    first_mission['mission_id'] += '_first_station'
    enabled = [point for point in first_mission['route']['waypoints'] if point['enabled']]
    if not enabled:
        parser.error('source mission has no enabled station')
    first_id = enabled[0]['id']
    for point in first_mission['route']['waypoints']:
        point['enabled'] = point['id'] == first_id
    first_mission['route']['transits'] = []
    first_mission['route']['return_home'] = False
    first_mission_file.write_text(yaml.safe_dump(first_mission, sort_keys=False))
    manifest = {
        'schema_version': 1,
        'source_sdf': str(source_sdf), 'source_sdf_sha256': digest(source_sdf),
        'source_metadata': str(metadata), 'metadata_sha256': digest(metadata_file),
        'source_mission': str(mission_path), 'source_mission_sha256': digest(mission_path),
        'world': str(world_file), 'world_sha256': digest(world_file),
        'metadata': str(metadata_file), 'mission': str(mission_file),
        'mission_sha256': digest(mission_file),
        'first_station_mission': str(first_mission_file),
        'first_station_mission_sha256': digest(first_mission_file),
        'map_and_keepout_bindings_preserved': True,
        'scope': 'same initial field; fruit attachment/release added, no new SLAM map',
    }
    manifest_file.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
