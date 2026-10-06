"""Invalid sensor contracts must fail before Gazebo starts."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from agri_sim_sensors.configuration import load_config, resolve_backend, write_bridge, write_world


CONFIG = Path(__file__).parents[1] / 'config' / 'mid360.yaml'


@pytest.mark.parametrize('key,value', [
    ('vertical_min_angle', 1.0), ('min_range', 50.0),
    ('horizontal_samples', 1.5), ('noise_stddev', -0.1),
    ('update_rate', float('nan')), ('points_topic', '/wrong'),
    ('frame_id', 'base_link'), ('render_engine', 'invalid_engine'),
])
def test_invalid_contract(tmp_path, key, value):
    data = yaml.safe_load(CONFIG.read_text())
    data['mid360'][key] = value
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_config(path)


def test_missing_rgl_does_not_silently_switch_explicit_mode(tmp_path):
    with pytest.raises(RuntimeError, match='setup_mid360_rgl'):
        resolve_backend('rgl', tmp_path, tmp_path)
    mode, reason = resolve_backend('auto', tmp_path, tmp_path)
    assert mode == 'gpu_lidar'
    assert 'falls back' in reason


def test_bridge_contract_and_world_backend_are_consistent(tmp_path):
    config = load_config(CONFIG)
    source = tmp_path / 'source.sdf'
    source.write_text('<sdf version="1.10"><world name="test"/></sdf>')
    destination = tmp_path / 'world.sdf'
    write_world(source, destination, config, 'rgl')
    world = ET.parse(destination).getroot().find('world')
    assert world.find("plugin[@name='gz::sim::systems::Sensors']") is not None
    manager = world.find("plugin[@name='rgl::RGLServerPluginManager']")
    assert manager.findtext('do_ignore_entities_in_lidar_link') == 'true'
    write_world(destination, destination, config, 'gpu_lidar')
    assert ET.parse(destination).getroot().find('.//plugin[@name="rgl::RGLServerPluginManager"]') is None
    bridge_path = tmp_path / 'bridge.yaml'
    write_bridge(config, bridge_path)
    bridge = yaml.safe_load(bridge_path.read_text())[0]
    assert bridge['direction'] == 'GZ_TO_ROS'
    assert bridge['qos_profile'] == 'SENSOR_DATA'
    assert bridge['frame_id'] == 'mid360_sensor_frame'
    write_bridge(config, bridge_path, 'rgl')
    bridge = yaml.safe_load(bridge_path.read_text())[0]
    assert bridge['ros_topic_name'] == '/mid360/rgl_raw'
    assert bridge['gz_topic_name'] == '/mid360/points'
