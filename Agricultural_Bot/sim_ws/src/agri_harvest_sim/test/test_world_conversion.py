import copy
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

PACKAGE = Path(__file__).parents[1]
ROOT = PACKAGE.parents[2]
spec = importlib.util.spec_from_file_location('converter', PACKAGE/'scripts/make_harvestable_world.py')
converter = importlib.util.module_from_spec(spec); spec.loader.exec_module(converter)


def test_conversion_preserves_initial_sensor_and_collision_geometry():
    path = ROOT/'sim_ws/src/agri_greenhouse_worlds/worlds/tomato_field_default.sdf'
    metadata = json.loads(path.with_suffix('.json').read_text())
    source = ET.parse(path).getroot()
    converted = converter.convert(copy.deepcopy(source), metadata)
    before = {model.get('name'): model for model in source.find('world').findall('model')}
    after = {model.get('name'): model for model in converted.find('world').findall('model')}
    assert before.keys() == after.keys()
    for fruit in metadata['fruits']:
        original, changed = before[fruit['id']], after[fruit['id']]
        assert original.find('pose').text == changed.find('pose').text
        assert changed.find('static').text == 'false'
        assert ET.tostring(original.find('link/collision')) == ET.tostring(changed.find('link/collision'))
        assert ET.tostring(original.find('link/visual/geometry')) == ET.tostring(changed.find('link/visual/geometry'))
        assert float(changed.find('link/inertial/mass').text) > 0
        support = original.find("link/visual[@name='harvest_support_visual']")
        if support is not None:
            moved = after[fruit['plant_id']].find(f"link/visual[@name='{fruit['id']}_support_visual']")
            plant = metadata['plants'][int(fruit['plant_id'].split('_')[-1])]
            position = [float(x) for x in moved.find('pose').text.split()[:3]]
            yaw = plant['yaw']
            actual = [plant['x']+math.cos(yaw)*position[0]-math.sin(yaw)*position[1],
                      plant['y']+math.sin(yaw)*position[0]+math.cos(yaw)*position[1], position[2]]
            local = [float(x) for x in support.find('pose').text.split()[:3]]
            expected = [fruit['x']+local[0], fruit['y']+local[1], fruit['z']+local[2]]
            assert actual == pytest.approx(expected, abs=1e-9)
    config = json.loads(converted.find("world/plugin[@name='agri::HarvestSystem']/config").text)
    assert len(config['fruits']) == len(metadata['fruits'])
    assert all('x' not in item for item in config['fruits'])
    with pytest.raises(ValueError, match='convert twice'):
        converter.convert(converted, metadata)
