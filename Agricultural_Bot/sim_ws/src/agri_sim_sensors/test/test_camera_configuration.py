"""Check the aligned RGB-D projection and reject invalid camera contracts."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

from agri_sim_sensors.camera_configuration import (
    camera_xacro_mappings,
    image_bridge_environment,
    load_camera_config,
    write_camera_bridge,
    write_camera_rviz,
)


PACKAGE = Path(__file__).parents[1]
CONFIG = PACKAGE / 'config' / 'd405.yaml'


@pytest.mark.parametrize('key,value', [
    ('sensor_frame', 'base_link'), ('optical_frame', 'd405_sensor_frame'),
    ('width', 1), ('height', 1), ('width', 848.0), ('height', 480.5),
    ('width', True), ('height', '480'),
    ('update_rate', 0), ('update_rate', -1), ('update_rate', float('inf')),
    ('update_rate', '30'), ('update_rate', True),
    ('hfov_deg', 0), ('hfov_deg', 180), ('vfov_deg', -1), ('vfov_deg', 181),
    ('hfov_deg', float('nan')), ('vfov_deg', float('inf')),
    ('color_near', 0), ('color_near', 0.08), ('color_near', float('nan')),
    ('depth_near', 0.5), ('depth_far', 0.06), ('depth_far', float('inf')),
    ('color_far', 0.4),
    ('pixel_center_offset', float('nan')), ('pixel_center_offset', '0.5'),
    ('pixel_center_offset', True), ('pixel_center_offset', -0.5), ('pixel_center_offset', 1.5),
    ('gz_topic', 'd405'), ('color_topic', '/d405//color/image_raw'),
    ('depth_topic', '/d405/depth/'), ('info_topic', '/d405/1camera_info'),
    ('info_topic', None), ('info_topic', '/d405/color/image_raw'),
    ('color_topic', '/d405'),
])
def test_invalid_camera_contract(tmp_path, key, value):
    document = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    document['d405'][key] = value
    destination = tmp_path / 'invalid.yaml'
    destination.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(ValueError):
        load_camera_config(destination)


@pytest.mark.parametrize('document', [None, [], {'d405': None}, {'d405': []}, {'d405': {}}])
def test_missing_camera_mapping_and_fields(tmp_path, document):
    destination = tmp_path / 'missing.yaml'
    destination.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(ValueError):
        load_camera_config(destination)


def test_equal_color_and_depth_clips_are_valid(tmp_path):
    document = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    document['d405']['color_near'] = 0.07
    document['d405']['color_far'] = 0.5
    destination = tmp_path / 'equal-clips.yaml'
    destination.write_text(yaml.safe_dump(document), encoding='utf-8')
    assert load_camera_config(destination)['depth_near'] == 0.07


def test_nominal_fov_uses_two_independent_focal_lengths():
    config = load_camera_config(CONFIG)
    mappings = camera_xacro_mappings(config)
    fx, fy = float(mappings['d405_fx']), float(mappings['d405_fy'])
    assert fx == pytest.approx(446.802773119128)
    assert fy == pytest.approx(432.971461265142)
    assert fx != pytest.approx(fy)
    assert float(mappings['d405_cx']) == 424
    assert float(mappings['d405_cy']) == 240
    assert math.degrees(2 * math.atan2(424, fx)) == pytest.approx(87)
    assert math.degrees(2 * math.atan2(240, fy)) == pytest.approx(58)
    # Using fx for both axes would miss the specified vertical field of view.
    assert math.degrees(2 * math.atan2(240, fx)) == pytest.approx(56.484915760385)


def test_projection_scales_with_resolution_without_changing_fov():
    config = load_camera_config(CONFIG)
    config.update(width=424, height=240)
    mappings = camera_xacro_mappings(config)
    assert float(mappings['d405_fx']) == pytest.approx(223.401386559564)
    assert float(mappings['d405_fy']) == pytest.approx(216.485730632571)
    assert float(mappings['d405_cx']) == 212
    assert float(mappings['d405_cy']) == 120


def test_camera_bridge_has_only_three_aligned_sensor_streams(tmp_path):
    config = load_camera_config(CONFIG)
    destination = tmp_path / 'bridge.yaml'
    write_camera_bridge(config, destination)
    bridge = yaml.safe_load(destination.read_text(encoding='utf-8'))
    assert len(bridge) == 3
    assert [(entry['gz_topic_name'], entry['ros_topic_name'], entry['ros_type_name'])
            for entry in bridge] == [
        ('/d405/image', '/d405/color/image_raw', 'sensor_msgs/msg/Image'),
        ('/d405/depth_image', '/d405/aligned_depth_to_color/image_raw', 'sensor_msgs/msg/Image'),
        ('/d405/camera_info', '/d405/color/camera_info', 'sensor_msgs/msg/CameraInfo'),
    ]
    for entry in bridge:
        assert entry['direction'] == 'GZ_TO_ROS'
        assert entry['qos_profile'] == 'SENSOR_DATA'
        assert entry['frame_id'] == 'camera_optical_frame'
        assert entry['lazy'] is False
        assert entry['gz_type_name'] == ('gz.msgs.CameraInfo' if
                                        entry['ros_type_name'].endswith('CameraInfo') else
                                        'gz.msgs.Image')


@pytest.mark.parametrize('config_name,depth_far', [
    ('d405.yaml', 0.5), ('d405_extended_sim.yaml', 2.0),
])
def test_expanded_sensor_has_real_intrinsics_and_separate_clipping(tmp_path, config_name, depth_far):
    mappings = camera_xacro_mappings(load_camera_config(PACKAGE / 'config' / config_name))
    invocation = {
        'frame_id': 'd405_frame', 'optical_frame': 'd405_optical_frame',
        'topic': 'd405_topic', 'width': 'd405_width', 'height': 'd405_height',
        'rate': 'd405_rate', 'hfov': 'd405_hfov', 'fx': 'd405_fx', 'fy': 'd405_fy',
        'cx': 'd405_cx', 'cy': 'd405_cy', 'color_near': 'd405_color_near',
        'color_far': 'd405_color_far', 'depth_near': 'd405_depth_near',
        'depth_far': 'd405_depth_far',
    }
    namespace = 'http://www.ros.org/wiki/xacro'
    ET.register_namespace('xacro', namespace)
    robot = ET.Element('robot', {'name': 'camera_test'})
    ET.SubElement(robot, f'{{{namespace}}}include', {
        'filename': str(PACKAGE / 'urdf' / 'd405.gazebo.xacro'),
    })
    ET.SubElement(robot, f'{{{namespace}}}agri_d405_simulation', {
        argument: mappings[key] for argument, key in invocation.items()
    })
    fixture = tmp_path / 'camera.urdf.xacro'
    ET.ElementTree(robot).write(fixture, encoding='utf-8', xml_declaration=True)
    expanded = ET.fromstring(xacro.process_file(str(fixture)).toxml())
    sensor = expanded.find("gazebo[@reference='d405_sensor_frame']/sensor")
    assert sensor is not None and sensor.attrib['type'] == 'rgbd_camera'
    assert sensor.findtext('gz_frame_id') == 'd405_sensor_frame'
    assert sensor.findtext('topic') == '/d405'
    assert float(sensor.findtext('update_rate')) == 30
    assert sensor.find('.//noise') is None
    camera = sensor.find('camera')
    assert camera.findtext('optical_frame_id') == 'camera_optical_frame'
    assert camera.findtext('image/width') == '848'
    assert camera.findtext('image/height') == '480'
    assert camera.findtext('image/format') == 'R8G8B8'
    assert float(camera.findtext('clip/near')) == 0.01
    assert float(camera.findtext('clip/far')) == 10
    assert float(camera.findtext('depth_camera/clip/near')) == 0.07
    assert float(camera.findtext('depth_camera/clip/far')) == depth_far
    assert float(camera.findtext('lens/intrinsics/fx')) == pytest.approx(446.802773119128)
    assert float(camera.findtext('lens/intrinsics/fy')) == pytest.approx(432.971461265142)
    assert float(camera.findtext('lens/intrinsics/cx')) == 424
    assert float(camera.findtext('lens/intrinsics/cy')) == 240
    assert float(camera.findtext('lens/intrinsics/s')) == 0
    assert float(camera.findtext('lens/projection/p_fx')) == pytest.approx(446.802773119128)
    assert float(camera.findtext('lens/projection/p_fy')) == pytest.approx(432.971461265142)
    assert float(camera.findtext('lens/projection/p_cx')) == 424
    assert float(camera.findtext('lens/projection/p_cy')) == 240
    assert float(camera.findtext('lens/projection/tx')) == 0
    assert float(camera.findtext('lens/projection/ty')) == 0


@pytest.mark.parametrize('config_name', ['d405.yaml', 'd405_extended_sim.yaml'])
def test_rviz_depth_range_tracks_sensor_without_clipping_valid_depth(tmp_path, config_name):
    config = load_camera_config(PACKAGE / 'config' / config_name)
    config.update(color_topic='/test/color', depth_topic='/test/depth')
    template = PACKAGE / 'rviz' / 'sensors.rviz'
    original = template.read_bytes()
    destination = tmp_path / 'sensors.rviz'
    write_camera_rviz(config, template, destination)
    displays = {display['Name']: display for display in
                yaml.safe_load(destination.read_text())['Visualization Manager']['Displays']}
    assert displays['D405 Color']['Topic']['Value'] == '/test/color'
    depth = displays['D405 Depth']
    assert depth['Topic']['Value'] == '/test/depth'
    assert depth['Normalize Range'] is False
    assert depth['Min Value'] == config['depth_near']
    assert depth['Max Value'] == config['depth_far']
    assert displays['MID-360']['Topic']['Value'] == '/mid360/points'
    assert template.read_bytes() == original


def test_image_bridge_does_not_duplicate_camera_info_bridge(tmp_path):
    destination = tmp_path / 'info.yaml'
    write_camera_bridge(load_camera_config(CONFIG), destination, include_images=False)
    bridge = yaml.safe_load(destination.read_text(encoding='utf-8'))
    assert len(bridge) == 1
    assert bridge[0]['ros_type_name'] == 'sensor_msgs/msg/CameraInfo'


def test_camera_info_bridge_routes_to_internal_topic_for_public_calibration(tmp_path):
    config = load_camera_config(CONFIG)
    destination = tmp_path / 'info.yaml'
    raw_topic = config['info_topic'] + '/gz_raw'
    write_camera_bridge(config, destination, include_images=False, info_topic_override=raw_topic)
    bridge = yaml.safe_load(destination.read_text())
    assert len(bridge) == 1
    assert bridge[0]['ros_topic_name'] == raw_topic
    assert bridge[0]['gz_topic_name'] == '/d405/camera_info'


@pytest.mark.parametrize('environment', [
    {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'},
    {'FASTDDS_DEFAULT_PROFILES_FILE': '/custom/fastdds.xml'},
    {'FASTRTPS_DEFAULT_PROFILES_FILE': '/custom/legacy.xml'},
])
def test_image_bridge_preserves_user_dds_settings(environment):
    assert image_bridge_environment(CONFIG, environment) == {}


def test_image_bridge_shm_accommodates_multiple_large_rgbd_samples():
    profile = PACKAGE / 'config' / 'd405_fastdds.xml'
    env = image_bridge_environment(profile, {})
    assert env == {'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile)}
    ns = {'dds': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    xml = ET.parse(profile)
    shm = next(transport for transport in xml.findall('.//dds:transport_descriptor', ns)
               if transport.findtext('dds:type', namespaces=ns) == 'SHM')
    # At least 5 histories of both streams plus serialization overhead.
    image_bytes = 848 * 480 * (3 + 4)
    assert int(shm.findtext('dds:segment_size', namespaces=ns)) > 5 * image_bytes
    assert int(shm.findtext('dds:maxMessageSize', namespaces=ns)) > 848 * 480 * 4
