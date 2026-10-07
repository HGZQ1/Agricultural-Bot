"""Regression checks for field generation and sensor-launch integration.

Run after building and sourcing both workspaces.
"""

import importlib.util
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


if __name__ == "__main__":
    unittest.main()
