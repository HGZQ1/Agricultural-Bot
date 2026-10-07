"""Black-box checks for reproducible fruit generation and its scene metadata."""

import collections
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


WORLD_PACKAGE = Path(__file__).resolve().parents[2] / "agri_greenhouse_worlds"
GENERATOR = WORLD_PACKAGE / "scripts/generate_tomato_field.py"


class RandomFruitTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.mesh_vertices = {}

    def generate(self, name="scene", arguments=(), randomize=True, check=True):
        output = self.root / f"{name}.sdf"
        metadata = self.root / f"{name}.json"
        command = [sys.executable, str(GENERATOR), "--output", str(output)]
        if randomize:
            command += ["--randomize-fruits", "--metadata", str(metadata)]
        result = subprocess.run(command + list(arguments), check=check,
                                capture_output=True, text=True, timeout=20)
        return output, metadata, result

    def read_scene(self, output, metadata):
        world = ET.parse(output).getroot().find("world")
        data = json.loads(metadata.read_text(encoding="utf-8"))
        return world, data

    def assert_scene(self, world, data, count_range=(2, 6),
                     height_range=(0.6, 1.1), diameter_range=(0.06, 0.09),
                     radius_range=(0.12, 0.25), clearance=0.01):
        self.assertEqual(data["schema_version"], 1)
        self.assertEqual(data["coordinate_frame"], "world")
        self.assertIsInstance(data["parameters"], dict)
        plants = {plant["id"]: plant for plant in data["plants"]}
        includes = world.findall("include")
        self.assertEqual(len(plants), len(includes))
        self.assertEqual(set(plants), {inc.findtext("name") for inc in includes})
        for include in includes:
            self.assertEqual(include.findtext("uri"), "model://tomato_plant")
            pose = [float(value) for value in include.findtext("pose").split()]
            plant = plants[include.findtext("name")]
            self.assertAlmostEqual(pose[0], plant["x"], delta=1e-6)
            self.assertAlmostEqual(pose[1], plant["y"], delta=1e-6)
            self.assertEqual(pose[2:5], [0, 0, 0])
            self.assertAlmostEqual(pose[5], plant["yaw"], delta=1e-6)

        fruits = data["fruits"]
        counts = collections.Counter(fruit["plant_id"] for fruit in fruits)
        for plant_id in plants:
            self.assertGreaterEqual(counts[plant_id], count_range[0])
            self.assertLessEqual(counts[plant_id], count_range[1])
        fruit_models = [model for model in world.findall("model")
                        if model.get("name") != "ground_plane"]
        models = {model.get("name"): model for model in fruit_models}
        self.assertEqual(len(models), len(fruit_models), "duplicate model names")
        self.assertEqual(set(models), {fruit["id"] for fruit in fruits})
        self.assertEqual(len({fruit["id"] for fruit in fruits}), len(fruits))

        for fruit in fruits:
            self.assertIn(fruit["plant_id"], plants)
            model = models[fruit["id"]]
            self.assertEqual(model.findtext("static"), "true")
            pose = [float(value) for value in model.findtext("pose").split()]
            for actual, key in zip(pose[:3], ("x", "y", "z")):
                self.assertAlmostEqual(actual, fruit[key], delta=1e-8)
            self.assertEqual(pose[3:], [0, 0, 0])
            collision_radius = float(model.findtext("link/collision/geometry/sphere/radius"))
            self.assertAlmostEqual(collision_radius, fruit["radius"], delta=1e-8)
            visual = model.find("link/visual")
            if data["parameters"]["fruit_visual"] == "mesh":
                self.assertIsNone(visual.find("geometry/sphere"))
                mesh = visual.find("geometry/mesh")
                self.assertIsNotNone(mesh)
                uri = mesh.findtext("uri")
                self.assertTrue(uri.startswith("model://"), uri)
                mesh_file = WORLD_PACKAGE / "models" / uri.removeprefix("model://")
                self.assertTrue(mesh_file.is_file(), uri)
                scale = [float(value) for value in mesh.findtext("scale").split()]
                self.assertEqual(len(scale), 3)
                for dimension in scale:
                    self.assertAlmostEqual(dimension, 2 * fruit["radius"], delta=1e-8)
                if uri not in self.mesh_vertices:
                    namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
                    collada = ET.parse(mesh_file).getroot()
                    geometry = collada.find("c:library_geometries/c:geometry/c:mesh", namespace)
                    position_input = geometry.find("c:vertices/c:input[@semantic='POSITION']", namespace)
                    source = geometry.find(
                        f"c:source[@id='{position_input.get('source').removeprefix('#')}']",
                        namespace)
                    coordinates = [float(value) for value in
                                   source.findtext("c:float_array", namespaces=namespace).split()]
                    self.mesh_vertices[uri] = list(zip(*[iter(coordinates)] * 3))
                # An imported mesh must fit its collision sphere at the metadata center.
                # This catches inherited whole-plant offsets and wrong mesh scale.
                self.assertTrue(self.mesh_vertices[uri])
                for vertex in self.mesh_vertices[uri]:
                    displacement = math.sqrt(sum((coordinate * axis_scale) ** 2
                                                 for coordinate, axis_scale in zip(vertex, scale)))
                    self.assertLessEqual(displacement, collision_radius + 1e-8)
                albedo = visual.findtext("material/pbr/metal/albedo_map")
                texture = "AG15frt1.png" if fruit["ripe"] else "AG15frt4.png"
                self.assertEqual(albedo, f"model://tomato_0/materials/textures/{texture}")
                self.assertTrue((WORLD_PACKAGE / "models" /
                                 albedo.removeprefix("model://")).is_file())
                self.assertEqual([float(value) for value in
                                  visual.findtext("material/diffuse").split()], [1, 1, 1, 1])
            else:
                self.assertEqual(data["parameters"]["fruit_visual"], "sphere")
                self.assertIsNone(visual.find("geometry/mesh"))
                visual_radius = float(visual.findtext("geometry/sphere/radius"))
                self.assertEqual(visual_radius, collision_radius)
                color = [float(value) for value in visual.findtext("material/diffuse").split()]
                self.assertEqual(len(color), 4)
                self.assertEqual(color[3], 1)
                dominant = 0 if fruit["ripe"] else 1
                self.assertGreater(color[dominant], max(color[channel] for channel in (0, 1, 2)
                                                       if channel != dominant))
            self.assertGreaterEqual(fruit["z"], height_range[0] - 1e-8)
            self.assertLessEqual(fruit["z"], height_range[1] + 1e-8)
            self.assertGreaterEqual(2 * fruit["radius"], diameter_range[0] - 1e-8)
            self.assertLessEqual(2 * fruit["radius"], diameter_range[1] + 1e-8)
            self.assertGreaterEqual(fruit["z"] - fruit["radius"], clearance - 1e-8)
            self.assertIsInstance(fruit["ripe"], bool)
            plant = plants[fruit["plant_id"]]
            stem_distance = math.hypot(fruit["x"] - plant["x"],
                                       fruit["y"] - plant["y"])
            self.assertGreaterEqual(stem_distance, radius_range[0] - 1e-8)
            self.assertLessEqual(stem_distance, radius_range[1] + 1e-8)

            # Check sphere/rotated-stem distances from generated geometry.
            # The source stem is a 0.1 x 0.1 x 1.2416 m box grounded at z=0.
            for other in plants.values():
                dx, dy = fruit["x"] - other["x"], fruit["y"] - other["y"]
                cosine, sine = math.cos(other["yaw"]), math.sin(other["yaw"])
                local_x, local_y = cosine * dx + sine * dy, -sine * dx + cosine * dy
                separation = math.sqrt(
                    max(abs(local_x) - 0.05, 0) ** 2
                    + max(abs(local_y) - 0.05, 0) ** 2
                    + max(-fruit["z"], fruit["z"] - 1.2416, 0) ** 2)
                self.assertGreaterEqual(separation, fruit["radius"] + clearance - 1e-8)

        for first, second in itertools.combinations(fruits, 2):
            distance = math.dist([first[key] for key in ("x", "y", "z")],
                                 [second[key] for key in ("x", "y", "z")])
            self.assertGreaterEqual(distance, first["radius"] + second["radius"]
                                    + clearance - 1e-8)

    def test_default_field_remains_byte_for_byte_reproducible(self):
        output, _, _ = self.generate(randomize=False)
        baseline = WORLD_PACKAGE / "worlds/tomato_field_22x14.sdf"
        self.assertEqual(output.read_bytes(), baseline.read_bytes())

    def test_random_fruits_geometry_ranges_and_ground_truth_match(self):
        output, metadata, _ = self.generate(arguments=("--rows", "2", "--plants-per-row", "3"))
        world, data = self.read_scene(output, metadata)
        self.assertEqual(data["seed"], 42)
        self.assertEqual(len(data["plants"]), 6)
        self.assert_scene(world, data)

    def test_seed_repeatability_and_plant_poses_stay_independent(self):
        common = ("--rows", "2", "--plants-per-row", "3")
        first, first_meta, _ = self.generate("first", common)
        repeat, repeat_meta, _ = self.generate("repeat", common)
        changed, changed_meta, _ = self.generate("changed", common + ("--seed", "43"))
        baseline, _, _ = self.generate("original", common, randomize=False)
        self.assertEqual(first.read_bytes(), repeat.read_bytes())
        self.assertEqual(first_meta.read_bytes(), repeat_meta.read_bytes())
        self.assertNotEqual(first.read_bytes(), changed.read_bytes())
        self.assertNotEqual(json.loads(first_meta.read_text())["fruits"],
                            json.loads(changed_meta.read_text())["fruits"])
        poses = lambda path: [inc.findtext("pose")
                              for inc in ET.parse(path).getroot().find("world").findall("include")]
        self.assertEqual(poses(first), poses(baseline))

    def test_explicit_count_height_diameter_and_radial_ranges(self):
        arguments = ("--rows", "2", "--plants-per-row", "2",
                     "--fruit-count-min", "4", "--fruit-count-max", "4",
                     "--fruit-height-min", "0.8", "--fruit-height-max", "1.2",
                     "--fruit-diameter-min", "0.04", "--fruit-diameter-max", "0.05",
                     "--fruit-radius-min", "0.15", "--fruit-radius-max", "0.3",
                     "--fruit-min-clearance", "0.02")
        output, metadata, _ = self.generate(arguments=arguments)
        world, data = self.read_scene(output, metadata)
        self.assertEqual(len(data["fruits"]), 16)
        self.assert_scene(world, data, (4, 4), (0.8, 1.2), (0.04, 0.05),
                          (0.15, 0.3), 0.02)

    def test_mesh_and_sphere_modes_preserve_seeded_targets(self):
        common = ("--rows", "2", "--plants-per-row", "3", "--seed", "47")
        scenes = {}
        for name, arguments in (("default", ()), ("mesh", ("--fruit-visual", "mesh")),
                                ("sphere", ("--fruit-visual", "sphere"))):
            output, metadata, _ = self.generate(name, common + arguments)
            world, data = self.read_scene(output, metadata)
            self.assert_scene(world, data)
            scenes[name] = output.read_bytes(), data
        self.assertEqual(scenes["default"][0], scenes["mesh"][0])
        self.assertNotEqual(scenes["mesh"][0], scenes["sphere"][0])
        for key in ("seed", "plants", "fruits"):
            self.assertEqual(scenes["mesh"][1][key], scenes["sphere"][1][key])
        mesh_parameters = dict(scenes["mesh"][1]["parameters"])
        sphere_parameters = dict(scenes["sphere"][1]["parameters"])
        mesh_parameters.pop("fruit_visual")
        sphere_parameters.pop("fruit_visual")
        self.assertEqual(mesh_parameters, sphere_parameters)

    def test_empty_fruit_scene_and_maturity_extremes(self):
        for ripe_ratio in (0, 1):
            with self.subTest(ripe_ratio=ripe_ratio):
                output, metadata, _ = self.generate(
                    f"ripe_{ripe_ratio}", ("--rows", "1", "--plants-per-row", "1",
                                           "--ripe-ratio", str(ripe_ratio)))
                world, data = self.read_scene(output, metadata)
                self.assert_scene(world, data)
                self.assertTrue(all(fruit["ripe"] == bool(ripe_ratio)
                                    for fruit in data["fruits"]))
        output, metadata, _ = self.generate(
            "empty", ("--fruit-count-min", "0", "--fruit-count-max", "0"))
        world, data = self.read_scene(output, metadata)
        self.assertEqual(data["fruits"], [])
        self.assert_scene(world, data, (0, 0))

    def test_invalid_parameters_preserve_existing_files(self):
        invalid = [
            ("--fruit-count-min", "-1"),
            ("--fruit-count-min", "7", "--fruit-count-max", "6"),
            ("--fruit-height-min", "1.2", "--fruit-height-max", "0.6"),
            ("--fruit-height-min", "0.01"),
            ("--fruit-diameter-min", "0"),
            ("--fruit-diameter-min", "0.1", "--fruit-diameter-max", "0.05"),
            ("--fruit-radius-min", "0.3", "--fruit-radius-max", "0.2"),
            ("--fruit-radius-min", "0.01"),
            ("--ripe-ratio", "1.1"),
            ("--fruit-min-clearance", "-0.01"),
            ("--fruit-height-max", "nan"),
            ("--fruit-radius-max", "inf"),
            ("--fruit-visual", "missing"),
        ]
        for index, arguments in enumerate(invalid):
            with self.subTest(arguments=arguments):
                output, metadata = self.root / f"invalid{index}.sdf", self.root / f"invalid{index}.json"
                output.write_text("original world", encoding="utf-8")
                metadata.write_text("original metadata", encoding="utf-8")
                _, _, result = self.generate(f"invalid{index}", arguments, check=False)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("error:", result.stderr)
                self.assertEqual(output.read_text(), "original world")
                self.assertEqual(metadata.read_text(), "original metadata")

    def test_impossible_packing_fails_without_partial_scene(self):
        output, metadata = self.root / "packed.sdf", self.root / "packed.json"
        output.write_text("original world", encoding="utf-8")
        metadata.write_text("original metadata", encoding="utf-8")
        _, _, result = self.generate("packed", (
            "--rows", "1", "--plants-per-row", "1",
            "--fruit-count-min", "100", "--fruit-count-max", "100",
            "--fruit-height-min", "0.6", "--fruit-height-max", "0.6",
            "--fruit-diameter-min", "0.09", "--fruit-diameter-max", "0.09",
            "--fruit-radius-min", "0.12", "--fruit-radius-max", "0.12"), check=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unable to place fruit", result.stderr)
        self.assertEqual(output.read_text(), "original world")
        self.assertEqual(metadata.read_text(), "original metadata")

    def test_fruit_free_asset_reuses_real_leaf_submeshes_and_is_grounded(self):
        model = ET.parse(WORLD_PACKAGE / "models/tomato_plant/model.sdf").getroot().find("model")
        self.assertEqual(model.findtext("static"), "true")
        self.assertEqual(len(model.findall("link/visual")), 3)
        self.assertEqual({visual.findtext("geometry/mesh/submesh/name")
                          for visual in model.findall("link/visual")},
                         {"Branch1", "Leaf1", "Leaf2"})
        collada = ET.parse(WORLD_PACKAGE / "models/tomato_0/meshes/tomato.dae").getroot()
        namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        source_nodes = {node.get("name") for node in collada.findall(".//c:node", namespace)}
        for visual in model.findall("link/visual"):
            self.assertIn(visual.findtext("geometry/mesh/submesh/name"), source_nodes)
            self.assertEqual([float(value) for value in visual.findtext("material/diffuse").split()],
                             [1, 1, 1, 1])
        for element in model.iter():
            if element.text and element.text.strip().startswith("model://"):
                uri = element.text.strip()
                self.assertTrue(uri.startswith("model://tomato_0/"), uri)
                self.assertTrue((WORLD_PACKAGE / "models" / uri.removeprefix("model://")).exists(), uri)
        self.assertEqual(len(model.findall("link/visual/material/pbr/metal/albedo_map")), 3)
        collision = model.find("link/collision")
        self.assertEqual([float(value) for value in collision.findtext("geometry/box/size").split()],
                         [0.1, 0.1, 1.2416])
        pose = [float(value) for value in collision.findtext("pose").split()]
        self.assertEqual(pose, [0, 0, 0.6208, 0, 0, 0])
        self.assertAlmostEqual(pose[2] - 1.2416 / 2, 0)
        config = ET.parse(WORLD_PACKAGE / "models/tomato_plant/model.config").getroot()
        self.assertEqual(config.findtext("sdf"), "model.sdf")
        self.assertEqual(config.find("sdf").get("version"), "1.6")


if __name__ == "__main__":
    unittest.main()
