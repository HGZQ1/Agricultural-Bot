"""Regression checks for field generation and sensor-launch integration.

Run after building and sourcing both workspaces.
"""

import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from agri_sim_sensors.configuration import load_config, write_world


def load_launch(path):
    spec = importlib.util.spec_from_file_location("field_launch_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TomatoFieldTest(unittest.TestCase):
    def test_no_duplicate_arguments_and_safe_defaults(self):
        launch_dir = Path(__file__).parents[1] / "launch"
        for path in launch_dir.glob("*.launch.py"):
            description = load_launch(path).generate_launch_description()
            arguments = [e for e in description.entities if isinstance(e, DeclareLaunchArgument)]
            names = [a.name for a in arguments]
            self.assertEqual(len(names), len(set(names)), path.name)
            if path.name == "tomato_field.launch.py":
                defaults = {a.name: perform_substitutions(LaunchContext(), a.default_value)
                            for a in arguments if a.name != "use_kinematics"}
                self.assertEqual(Path(defaults["world"]).name, "tomato_field_default.sdf")
                self.assertEqual(defaults["paused"], "true")
                for switch in ("use_control", "use_lidar", "use_camera"):
                    self.assertEqual(defaults[switch], "false")

    def test_installed_world_reproducible_and_sensor_rewrite_preserves_field(self):
        share = Path(get_package_share_directory("agri_greenhouse_worlds"))
        source = share / "worlds/tomato_field_22x14.sdf"
        generator = share.parents[1] / "lib/agri_greenhouse_worlds/generate_tomato_field.py"
        sensor_share = Path(get_package_share_directory("agri_sim_sensors"))
        with tempfile.TemporaryDirectory() as directory:
            generated = Path(directory) / "generated.sdf"
            subprocess.run(["python3", str(generator), "--output", str(generated)], check=True)
            self.assertEqual(source.read_bytes(), generated.read_bytes())
            runtime = Path(directory) / "runtime.sdf"
            write_world(source, runtime, load_config(sensor_share / "config/mid360.yaml"), "gpu_lidar")
            world = ET.parse(runtime).getroot().find("world")
            self.assertEqual(world.get("name"), "field")
            self.assertEqual(len(world.findall("include")), 150)
            self.assertIsNotNone(world.find("model[@name='ground_plane']/link/collision"))
            self.assertIsNotNone(world.find("plugin[@name='gz::sim::systems::Sensors']"))
            for include in world.findall("include"):
                self.assertEqual(include.findtext("uri"), "model://tomato_0")
            for element in ET.parse(share / "models/tomato_0/model.sdf").iter():
                if element.text and element.text.strip().startswith("model://tomato_0/"):
                    target = share / "models" / element.text.strip().removeprefix("model://")
                    self.assertTrue(target.exists(), str(target))

    def test_default_world_matches_compact_navigation_profile(self):
        share = Path(get_package_share_directory("agri_greenhouse_worlds"))
        world_path = share / "worlds/tomato_field_default.sdf"
        metadata_path = share / "worlds/tomato_field_default.json"
        data = json.loads(metadata_path.read_text(encoding="utf-8"))
        parameters = data["parameters"]
        expected = {
            "rows": 6,
            "plants_per_row": 10,
            "row_spacing": 2.0,
            "plant_spacing": 0.7,
            "origin_x": -5.0,
            "origin_y": -5.0,
            "plant_height": 2.0,
            "plant_width": 1.0,
            "plant_collision_width": 0.06,
        }
        self.assertEqual({key: parameters[key] for key in expected}, expected)
        self.assertEqual(len(data["plants"]), 60)
        self.assertEqual(len(data["fruits"]), 224)

        world = ET.parse(world_path).getroot().find("world")
        plant_names = {plant["id"] for plant in data["plants"]}
        fruit_names = {fruit["id"] for fruit in data["fruits"]}
        model_names = {model.get("name") for model in world.findall("model")}
        self.assertTrue(plant_names <= model_names)
        self.assertTrue(fruit_names <= model_names)


if __name__ == "__main__":
    unittest.main()
