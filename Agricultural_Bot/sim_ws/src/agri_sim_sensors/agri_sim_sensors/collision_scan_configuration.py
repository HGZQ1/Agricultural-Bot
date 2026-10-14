"""Independent simulation-only hard-obstacle observation contract."""

from __future__ import annotations

import math
from pathlib import Path

import yaml


# Kept in sync with collision_scan.gazebo.xacro. Decimal text is portable SDF uint.
SOFT_PLANT_VISIBILITY = 4
STEM_PROXY_VISIBILITY = 0x20000000
NORMAL_SENSOR_VISIBILITY = 0x0FFFFFFF
HARD_OBSTACLE_VISIBILITY = 0x2FFFFFFB


def load_collision_scan_config(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    config = document.get("collision_scan") if isinstance(document, dict) else None
    if not isinstance(config, dict):
        raise ValueError("collision_scan configuration must contain a mapping")
    required = (
        "frame_id", "gz_topic", "points_topic", "update_rate", "min_range",
        "max_range", "horizontal_samples", "vertical_samples",
        "horizontal_min_angle", "horizontal_max_angle", "vertical_min_angle",
        "vertical_max_angle", "range_resolution", "noise_stddev",
    )
    for key in required:
        if key not in config:
            raise ValueError(f"collision_scan configuration is missing {key}")
    if config["frame_id"] != "mid360_sensor_frame":
        raise ValueError("collision_scan frame_id must be canonical mid360_sensor_frame")
    if config["gz_topic"] != "/hard_obstacles" or config["points_topic"] != "/hard_obstacles/points":
        raise ValueError("collision_scan topics must be /hard_obstacles and /hard_obstacles/points")
    for key in required[3:]:
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f"collision_scan {key} must be a finite number")
    if not 0 < config["min_range"] < config["max_range"]:
        raise ValueError("collision_scan requires 0 < min_range < max_range")
    if config["update_rate"] <= 0 or config["range_resolution"] <= 0:
        raise ValueError("collision_scan rate and range resolution must be positive")
    if config["noise_stddev"] < 0:
        raise ValueError("collision_scan noise cannot be negative")
    for axis in ("horizontal", "vertical"):
        samples = config[f"{axis}_samples"]
        if not isinstance(samples, int) or samples < 2:
            raise ValueError(f"collision_scan {axis}_samples must be an integer >= 2")
        low, high = config[f"{axis}_min_angle"], config[f"{axis}_max_angle"]
        if low >= high or not -math.pi <= low < high <= math.pi:
            raise ValueError(f"collision_scan {axis} angle bounds must increase within [-pi, pi]")
    return config


def collision_scan_xacro_mappings(config: dict) -> dict[str, str]:
    names = {
        "collision_scan_frame": "frame_id", "collision_scan_topic": "gz_topic",
        "collision_scan_rate": "update_rate", "collision_scan_min_range": "min_range",
        "collision_scan_max_range": "max_range", "collision_scan_h_samples": "horizontal_samples",
        "collision_scan_v_samples": "vertical_samples", "collision_scan_h_min": "horizontal_min_angle",
        "collision_scan_h_max": "horizontal_max_angle", "collision_scan_v_min": "vertical_min_angle",
        "collision_scan_v_max": "vertical_max_angle", "collision_scan_resolution": "range_resolution",
        "collision_scan_noise": "noise_stddev",
    }
    return {name: str(config[key]) for name, key in names.items()}


def write_collision_scan_bridge(config: dict, destination: Path) -> None:
    bridge = [{
        "ros_topic_name": config["points_topic"], "gz_topic_name": config["points_topic"],
        "ros_type_name": "sensor_msgs/msg/PointCloud2", "gz_type_name": "gz.msgs.PointCloudPacked",
        "direction": "GZ_TO_ROS", "qos_profile": "SENSOR_DATA", "lazy": False,
        "frame_id": config["frame_id"],
    }]
    destination.write_text(yaml.safe_dump(bridge, sort_keys=False), encoding="utf-8")

