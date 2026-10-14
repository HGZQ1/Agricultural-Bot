#!/usr/bin/env python3
"""Preserve a mapped field's initial geometry; change only fruit dynamics."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def convert(root, metadata, robot_model='agri_robot'):
    world = root.find('world')
    if world is None:
        raise ValueError('a world SDF is required')
    models = {model.get('name'): model for model in world.findall('model')}
    for fruit in metadata['fruits']:
        model = models.get(fruit['id'])
        if model is None or model.find('link').get('name') != 'fruit_link':
            raise ValueError(f"missing independent fruit model {fruit['id']}")
        model.find('static').text = 'false'
        link = model.find('link')
        if link.find('inertial') is not None:
            raise ValueError('fruit already has dynamics; refusing to convert twice')
        inertial = ET.SubElement(link, 'inertial')
        mass = 0.025
        moment = 0.4 * mass * fruit['radius'] ** 2
        ET.SubElement(inertial, 'mass').text = str(mass)
        inertia = ET.SubElement(inertial, 'inertia')
        for field in ('ixx', 'iyy', 'izz'):
            ET.SubElement(inertia, field).text = str(moment)
        for field in ('ixy', 'ixz', 'iyz'):
            ET.SubElement(inertia, field).text = '0'
        # The visual supporting branch belongs to the plant, not the picked fruit.
        support = link.find("visual[@name='harvest_support_visual']")
        if support is not None:
            plant = models.get(fruit['plant_id'])
            if plant is None:
                raise ValueError('inline plant models are required for harvest supports')
            plant_pose = [float(x) for x in plant.find('pose').text.split()]
            support_pose = [float(x) for x in support.find('pose').text.split()]
            dx = fruit['x'] + support_pose[0] - plant_pose[0]
            dy = fruit['y'] + support_pose[1] - plant_pose[1]
            yaw = plant_pose[5]
            support_pose[:3] = [math.cos(yaw)*dx + math.sin(yaw)*dy,
                               -math.sin(yaw)*dx + math.cos(yaw)*dy,
                               fruit['z'] + support_pose[2] - plant_pose[2]]
            support_pose[5] -= yaw
            moved = copy.deepcopy(support)
            moved.set('name', fruit['id'] + '_support_visual')
            moved.find('pose').text = ' '.join(str(x) for x in support_pose)
            plant.find('link').append(moved)
            link.remove(support)
    plugin = ET.SubElement(world, 'plugin', filename='libagri_harvest_system.so',
                           name='agri::HarvestSystem')
    ET.SubElement(plugin, 'config').text = json.dumps({
        'robot_model': robot_model,
        'service': f"/world/{world.get('name')}/harvest/command",
        'tcp_offset': [-0.18, 0, 0],
        'fruits': [{key: fruit[key] for key in ('id', 'plant_id', 'radius', 'ripe')}
                   for fruit in metadata['fruits']],
    }, separators=(',', ':'))
    return root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--robot-model', default='agri_robot')
    args = parser.parse_args()
    if args.output.resolve() in (args.source.resolve(), args.metadata.resolve()):
        parser.error('output must be a separate world; source assets are preserved')
    if args.manifest.resolve() in (args.source.resolve(), args.metadata.resolve(), args.output.resolve()):
        parser.error('manifest must be a separate file')
    root = convert(ET.parse(args.source).getroot(), json.loads(args.metadata.read_text()), args.robot_model)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(args.output, encoding='utf-8', xml_declaration=True)
    manifest = {
        'schema_version': 1, 'variant': 'harvestable_dynamic_fruit',
        'source_sdf': str(args.source.resolve()),
        'source_sdf_sha256': hashlib.sha256(args.source.read_bytes()).hexdigest(),
        'metadata': str(args.metadata.resolve()),
        'metadata_sha256': hashlib.sha256(args.metadata.read_bytes()).hexdigest(),
        'output_sdf': str(args.output.resolve()),
        'output_sdf_sha256': hashlib.sha256(args.output.read_bytes()).hexdigest(),
        'initial_visual_and_collision_geometry_preserved': True,
        'map_world_alignment_assumed': False,
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest))


if __name__ == '__main__':
    main()
