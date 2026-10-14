"""Prevent frame relabeling and keep synthetic hard obstacles out of RGB/localization."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

from agri_sim_sensors.collision_scan_configuration import (
    HARD_OBSTACLE_VISIBILITY, NORMAL_SENSOR_VISIBILITY,
    SOFT_PLANT_VISIBILITY, STEM_PROXY_VISIBILITY,
    collision_scan_xacro_mappings, load_collision_scan_config, write_collision_scan_bridge,
)


PACKAGE = Path(__file__).parents[1]
CONFIG = PACKAGE / "config" / "collision_scan.yaml"


@pytest.mark.parametrize("key,value", [
    ("frame_id", "base_footprint"), ("points_topic", "/scan"), ("gz_topic", "/mid360"),
    ("min_range", 0), ("max_range", .05), ("update_rate", 0),
    ("range_resolution", -1), ("noise_stddev", -1), ("horizontal_samples", 1),
    ("vertical_samples", 16.0), ("vertical_samples", True),
    ("vertical_min_angle", 1.0), ("horizontal_max_angle", 4.0),
    ("max_range", float("nan")), ("update_rate", "10"),
])
def test_invalid_collision_observation_fails_before_spawn(tmp_path, key, value):
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    document["collision_scan"][key] = value
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError):
        load_collision_scan_config(invalid)


@pytest.mark.parametrize("document", [None, [], {}, {"collision_scan": []}, {"collision_scan": {}}])
def test_missing_collision_scan_contract(tmp_path, document):
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError):
        load_collision_scan_config(invalid)


def test_soft_visual_and_hard_proxy_masks_are_separate():
    # Evaluate the actual rendering rule, including ordinary default flags.
    assert NORMAL_SENSOR_VISIBILITY & SOFT_PLANT_VISIBILITY
    assert not NORMAL_SENSOR_VISIBILITY & STEM_PROXY_VISIBILITY
    assert not HARD_OBSTACLE_VISIBILITY & SOFT_PLANT_VISIBILITY
    assert HARD_OBSTACLE_VISIBILITY & STEM_PROXY_VISIBILITY
    assert HARD_OBSTACLE_VISIBILITY & 1
    assert HARD_OBSTACLE_VISIBILITY & 0xFFFFFFFF


def test_bridge_preserves_sensor_coordinates_and_never_replaces_scan(tmp_path):
    config = load_collision_scan_config(CONFIG)
    destination = tmp_path / "bridge.yaml"
    write_collision_scan_bridge(config, destination)
    bridge = yaml.safe_load(destination.read_text(encoding="utf-8"))
    assert len(bridge) == 1
    assert bridge[0]["gz_topic_name"] == bridge[0]["ros_topic_name"] == "/hard_obstacles/points"
    assert bridge[0]["frame_id"] == "mid360_sensor_frame"
    assert bridge[0]["direction"] == "GZ_TO_ROS"
    assert bridge[0]["qos_profile"] == "SENSOR_DATA"
    assert bridge[0]["lazy"] is False


def test_xacro_has_same_optical_pose_and_more_downward_coverage(tmp_path):
    config = load_collision_scan_config(CONFIG)
    kwargs = collision_scan_xacro_mappings(config)
    parameters = {
        name.removeprefix("collision_scan_"): value for name, value in kwargs.items()
    }
    parameters["frame_id"] = parameters.pop("frame")
    arguments = " ".join(f'{key}="{value}"' for key, value in parameters.items())
    fixture = tmp_path / "sensors.xacro"
    fixture.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="sensor_contract">'
        f'<xacro:include filename="{PACKAGE / "urdf" / "collision_scan.gazebo.xacro"}"/>'
        f'<xacro:agri_collision_scan_simulation {arguments}/>'
        '</robot>', encoding="utf-8")
    root = ET.fromstring(xacro.process_file(str(fixture)).toxml())
    sensor = root.find("gazebo[@reference='mid360_sensor_frame']/sensor")
    assert sensor.get("type") == "gpu_lidar"
    assert sensor.findtext("pose") == "0 0 0 0 0 0"
    assert sensor.findtext("gz_frame_id") == config["frame_id"]
    assert sensor.findtext("topic") == config["gz_topic"]
    assert int(sensor.findtext("lidar/visibility_mask")) == HARD_OBSTACLE_VISIBILITY
    assert sensor.findtext("lidar/scan/horizontal/samples") == "720"
    assert sensor.findtext("lidar/scan/vertical/samples") == "16"
    assert float(sensor.findtext("lidar/scan/vertical/min_angle")) < -.1221730476
    assert float(sensor.findtext("lidar/range/max")) == 8.0
    assert sensor.findtext("visualize") == "false"
    assert not root.findall("link")  # Sensor macro cannot add a substitute robot/TF tree.


def test_existing_camera_and_gpu_lidar_exclude_hidden_proxy():
    camera = ET.parse(PACKAGE / "urdf" / "d405.gazebo.xacro").getroot()
    lidar = ET.parse(PACKAGE / "urdf" / "mid360.gazebo.xacro").getroot()
    assert int(camera.findtext(".//camera/visibility_mask")) == NORMAL_SENSOR_VISIBILITY
    assert int(lidar.findtext(".//lidar/visibility_mask")) == NORMAL_SENSOR_VISIBILITY
