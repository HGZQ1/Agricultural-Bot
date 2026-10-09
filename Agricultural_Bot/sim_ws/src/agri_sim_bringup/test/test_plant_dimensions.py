"""Validate resized plant geometry independently of the world generator."""

import hashlib
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
NAMESPACE = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
FOLIAGE_NAMES = {"Branch1", "Leaf1", "Leaf2"}


def source_vertices():
    """Resolve the real source geometry through each COLLADA scene node."""
    root = ET.parse(WORLD_PACKAGE / "models/tomato_0/meshes/tomato.dae").getroot()
    result = {}
    for name in FOLIAGE_NAMES:
        node = root.find(f".//c:visual_scene/c:node[@name='{name}']", NAMESPACE)
        matrix = [float(value) for value in node.findtext("c:matrix", namespaces=NAMESPACE).split()]
        if matrix != [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]:
            raise AssertionError("The source scene acquired a non-identity transform")
        geometry_id = node.find("c:instance_geometry", NAMESPACE).get("url")[1:]
        mesh = root.find(f"c:library_geometries/c:geometry[@id='{geometry_id}']/c:mesh", NAMESPACE)
        position_id = mesh.find("c:vertices/c:input[@semantic='POSITION']", NAMESPACE).get("source")[1:]
        source = mesh.find(f"c:source[@id='{position_id}']", NAMESPACE)
        values = [float(value) for value in source.findtext("c:float_array", namespaces=NAMESPACE).split()]
        accessor = source.find("c:technique_common/c:accessor", NAMESPACE)
        stride = int(accessor.get("stride", "1"))
        offset = int(accessor.get("offset", "0"))
        positions = [tuple(values[offset + index * stride:offset + index * stride + 3])
                     for index in range(int(accessor.get("count")))]
        used = set()
        for primitive in mesh.findall("c:triangles", NAMESPACE):
            inputs = primitive.findall("c:input", NAMESPACE)
            vertex_input = next(item for item in inputs if item.get("semantic") == "VERTEX")
            index_stride = max(int(item.get("offset")) for item in inputs) + 1
            index_offset = int(vertex_input.get("offset"))
            indices = [int(value) for value in primitive.findtext("c:p", namespaces=NAMESPACE).split()]
            used.update(indices[index_offset::index_stride])
        if not used:
            raise AssertionError(f"No source faces for {name}")
        result[name] = [positions[index] for index in sorted(used)]
    return result


def box_distance(fruit, plant, dimensions):
    """Sphere-center distance to a rotated, grounded box from generated SDF."""
    dx, dy = fruit["x"] - plant["x"], fruit["y"] - plant["y"]
    cosine, sine = math.cos(plant["yaw"]), math.sin(plant["yaw"])
    x, y = cosine * dx + sine * dy, -sine * dx + cosine * dy
    return math.sqrt(max(abs(x) - dimensions[0] / 2, 0) ** 2
                     + max(abs(y) - dimensions[1] / 2, 0) ** 2
                     + max(-fruit["z"], fruit["z"] - dimensions[2], 0) ** 2)


def element_payload(element):
    """Compare XML values while allowing serializer indentation changes."""
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or "").strip(),
            tuple(element_payload(child) for child in element))


class PlantDimensionsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vertices = source_vertices()
        points = [point for group in cls.vertices.values() for point in group]
        cls.source_height = max(point[2] for point in points)
        cls.source_spans = [max(point[axis] for point in points) - min(point[axis] for point in points)
                            for axis in (0, 1)]
        cls.source_width = max(cls.source_spans)

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)

    def generate(self, name="scene", arguments=(), randomize=True, check=True):
        output, metadata = self.root / f"{name}.sdf", self.root / f"{name}.json"
        command = [sys.executable, str(GENERATOR), "--output", str(output), "--metadata", str(metadata)]
        if randomize:
            command.append("--randomize-fruits")
        result = subprocess.run(command + list(arguments), capture_output=True, text=True,
                                check=check, timeout=20)
        if result.returncode:
            return output, metadata, result
        return ET.parse(output).getroot().find("world"), json.loads(metadata.read_text()), result

    def plant_models(self, world, data):
        return [world.find(f"model[@name='{plant['id']}']") for plant in data["plants"]]

    def assert_dimensions(self, world, data, expected_height=None, expected_width=None):
        expected_height = self.source_height if expected_height is None else expected_height
        expected_width = self.source_width if expected_width is None else expected_width
        scales = [expected_width / self.source_width] * 2 + [expected_height / self.source_height]
        expected_stem = [0.1 * scales[0], 0.1 * scales[1], 1.2416 * scales[2]]
        self.assertEqual(world.findall("include"), [])
        models = self.plant_models(world, data)
        self.assertTrue(models)
        for model, plant in zip(models, data["plants"]):
            self.assertIsNotNone(model)
            self.assertEqual(model.findtext("static"), "true")
            pose = [float(value) for value in model.findtext("pose").split()]
            self.assertEqual(pose, [plant["x"], plant["y"], 0, 0, 0, plant["yaw"]])
            vertices = []
            foliage = [visual for visual in model.findall("link/visual")
                       if visual.findtext("geometry/mesh/submesh/name") in FOLIAGE_NAMES]
            self.assertEqual(len(foliage), 3)
            for visual in foliage:
                mesh = visual.find("geometry/mesh")
                self.assertEqual(mesh.findtext("uri"), "model://tomato_0/meshes/tomato.dae")
                self.assertEqual(mesh.findtext("submesh/center"), "false")
                scale = [float(value) for value in mesh.findtext("scale").split()]
                for actual, expected in zip(scale, scales):
                    self.assertAlmostEqual(actual, expected, delta=1e-10)
                local_pose = [float(value) for value in visual.findtext("pose", "0 0 0 0 0 0").split()]
                self.assertEqual(local_pose, [0] * 6)
                vertices.extend(tuple(value * factor for value, factor in zip(point, scale))
                                for point in self.vertices[mesh.findtext("submesh/name")])
            self.assertAlmostEqual(max(point[2] for point in vertices), expected_height, delta=1e-9)
            spans = [max(point[axis] for point in vertices) - min(point[axis] for point in vertices)
                     for axis in (0, 1)]
            self.assertAlmostEqual(max(spans), expected_width, delta=1e-9)
            for actual, original in zip(spans, self.source_spans):
                self.assertAlmostEqual(actual, original * scales[0], delta=1e-9)
            collisions = model.findall("link/collision")
            self.assertEqual(len(collisions), 1)
            size = [float(value) for value in collisions[0].findtext("geometry/box/size").split()]
            center = [float(value) for value in collisions[0].findtext("pose").split()]
            for actual, expected in zip(size, expected_stem):
                self.assertAlmostEqual(actual, expected, delta=1e-10)
            self.assertEqual(center[:2] + center[3:], [0] * 5)
            self.assertAlmostEqual(center[2] - size[2] / 2, 0, delta=1e-10)
            self.assertAlmostEqual(center[2] + size[2] / 2, expected_stem[2], delta=1e-10)
        geometry = data["plant_geometry"]
        self.assertAlmostEqual(geometry["height"], expected_height, delta=1e-9)
        self.assertAlmostEqual(geometry["width"], expected_width, delta=1e-9)
        self.assertAlmostEqual(geometry["stem_center_z"], expected_stem[2] / 2, delta=1e-9)
        for key, expected in (("mesh_scale", scales), ("stem_size", expected_stem)):
            for actual, value in zip(geometry[key], expected):
                self.assertAlmostEqual(actual, value, delta=1e-9)

    def test_no_dimensions_keep_baseline_bytes_and_original_assets(self):
        assets = [WORLD_PACKAGE / "models/tomato_0/model.sdf",
                  WORLD_PACKAGE / "models/tomato_plant/model.sdf",
                  WORLD_PACKAGE / "models/tomato_0/meshes/tomato.dae"]
        hashes = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in assets}
        _, baseline_data, _ = self.generate("baseline", randomize=False)
        self.assertEqual((self.root / "baseline.sdf").read_bytes(),
                         (WORLD_PACKAGE / "worlds/tomato_field_22x14.sdf").read_bytes())
        legacy = baseline_data["plant_geometry"]
        self.assertEqual(legacy["stem_size"], [0.1, 0.1, 1])
        self.assertEqual(legacy["stem_center_z"], 0)
        self.assertEqual(legacy["mesh_scale"], [1, 1, 1])
        self.assertAlmostEqual(legacy["height"], self.source_height, delta=1e-9)
        self.assertAlmostEqual(legacy["width"], self.source_width, delta=1e-9)
        world, data, _ = self.generate("random", ("--rows", "1", "--plants-per-row", "1"))
        self.assertEqual(len(world.findall("include")), 1)
        self.assertEqual(world.findtext("include/uri"), "model://tomato_plant")
        self.assertIsNone(data["parameters"]["plant_height"])
        self.assertIsNone(data["parameters"]["plant_width"])
        self.generate("resized", ("--rows", "1", "--plants-per-row", "1", "--plant-height", "2"))
        self.assertEqual(hashes, {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in assets})

    def test_height_only_reaches_mesh_top_without_widening_crown(self):
        for randomize in (False, True):
            with self.subTest(randomize=randomize):
                world, data, _ = self.generate(str(randomize),
                    ("--rows", "2", "--plants-per-row", "2", "--plant-height", "2"), randomize)
                self.assert_dimensions(world, data, expected_height=2)
                self.assertEqual(data["parameters"]["plant_height"], 2)
                self.assertIsNone(data["parameters"]["plant_width"])

    def test_width_only_and_combined_geometry_have_independent_scales(self):
        for randomize in (False, True):
            for height in (None, 1.8):
                with self.subTest(randomize=randomize, height=height):
                    arguments = ("--rows", "1", "--plants-per-row", "2", "--plant-width", "0.75")
                    if height is not None:
                        arguments += ("--plant-height", str(height))
                    world, data, _ = self.generate(f"{randomize}_{height}", arguments, randomize)
                    self.assert_dimensions(world, data, height, 0.75)

    def test_collision_box_is_independent_from_visual_canopy(self):
        arguments = (
            "--rows", "1", "--plants-per-row", "1",
            "--plant-height", "2.6", "--plant-width", "1.0",
            "--plant-collision-width", "0.06",
            "--plant-collision-height", "1.10",
            "--fruit-count-min", "0", "--fruit-count-max", "0",
        )
        for randomize in (False, True):
            with self.subTest(randomize=randomize):
                world, data, _ = self.generate(
                    f"collision_{randomize}", arguments, randomize
                )
                model = self.plant_models(world, data)[0]
                collision = model.find("link/collision")
                size = [float(value) for value in collision.findtext(
                    "geometry/box/size").split()]
                pose = [float(value) for value in collision.findtext("pose").split()]
                self.assertEqual(size, [0.06, 0.06, 1.10])
                self.assertEqual(pose[:2] + pose[3:], [0.0] * 5)
                self.assertAlmostEqual(pose[2], 0.55, delta=1e-12)
                foliage = [visual for visual in model.findall("link/visual")
                           if visual.findtext("geometry/mesh/submesh/name") in FOLIAGE_NAMES]
                self.assertEqual(len(foliage), 3)
                for visual in foliage:
                    scale = [float(value) for value in visual.findtext(
                        "geometry/mesh/scale").split()]
                    expected_width_scale = 1.0 / self.source_width
                    self.assertAlmostEqual(scale[0], expected_width_scale, delta=1e-12)
                    self.assertAlmostEqual(scale[1], expected_width_scale, delta=1e-12)
                    self.assertAlmostEqual(scale[2], 2.6 / self.source_height, delta=1e-12)
                geometry = data["plant_geometry"]
                self.assertEqual(geometry["collision_width"], 0.06)
                self.assertEqual(geometry["collision_height"], 1.10)
                self.assertEqual(data["parameters"]["plant_collision_width"], 0.06)
                self.assertEqual(data["parameters"]["plant_collision_height"], 1.10)

    def test_original_fixed_fruit_and_blossom_meshes_are_not_rescaled(self):
        world, data, _ = self.generate(arguments=("--rows", "1", "--plants-per-row", "1",
                                                  "--plant-height", "2", "--plant-width", "1.5"),
                                        randomize=False)
        self.assert_dimensions(world, data, 2, 1.5)
        source = ET.parse(WORLD_PACKAGE / "models/tomato_0/model.sdf").getroot().find("model")
        fixed = [visual for visual in source.findall("link/visual")
                 if visual.findtext("geometry/mesh/submesh/name") not in FOLIAGE_NAMES]
        self.assertTrue(fixed)
        plant = self.plant_models(world, data)[0]
        for original in fixed:
            copied = plant.find(f"link/visual[@name='{original.get('name')}']")
            self.assertIsNotNone(copied)
            self.assertEqual(element_payload(copied.find("geometry")),
                             element_payload(original.find("geometry")))
            self.assertEqual(copied.findtext("pose"), original.findtext("pose"))
        self.assertEqual(data["fruits"], [])

    def test_height_change_preserves_independent_fruit_size_and_world_z(self):
        common = ("--rows", "2", "--plants-per-row", "2", "--fruit-count-min", "3",
                  "--fruit-count-max", "3", "--fruit-height-min", "0.9", "--fruit-height-max", "0.9",
                  "--fruit-diameter-min", "0.08", "--fruit-diameter-max", "0.08", "--seed", "51")
        _, original, _ = self.generate("original", common)
        for height, width in ((2, None), (1.8, 0.7)):
            with self.subTest(height=height, width=width):
                arguments = common + ("--plant-height", str(height))
                if width is not None:
                    arguments += ("--plant-width", str(width))
                world, data, _ = self.generate(f"h{height}", arguments)
                self.assert_dimensions(world, data, height, width)
                self.assertEqual(len(data["fruits"]), 12)
                for fruit in data["fruits"]:
                    self.assertEqual(fruit["z"], 0.9)
                    self.assertEqual(fruit["radius"], 0.04)
                    model = world.find(f"model[@name='{fruit['id']}']")
                    self.assertEqual([float(value) for value in model.findtext(
                        "link/visual/geometry/mesh/scale").split()], [0.08] * 3)
                if width is None:
                    self.assertEqual(data["fruits"], original["fruits"])

    def test_resized_scene_and_metadata_repeat_for_the_same_seed(self):
        arguments = ("--rows", "2", "--plants-per-row", "2", "--plant-height", "2",
                     "--plant-width", "0.8", "--seed", "47")
        self.generate("first", arguments)
        self.generate("repeat", arguments)
        self.generate("other", arguments + ("--seed", "48"))
        for suffix in ("sdf", "json"):
            self.assertEqual((self.root / f"first.{suffix}").read_bytes(),
                             (self.root / f"repeat.{suffix}").read_bytes())
            self.assertNotEqual((self.root / f"first.{suffix}").read_bytes(),
                                (self.root / f"other.{suffix}").read_bytes())

    def test_invalid_dimensions_and_scaled_stem_radius_preserve_existing_outputs(self):
        invalid = [(flag, value) for flag in (
            "--plant-height", "--plant-width",
            "--plant-collision-width", "--plant-collision-height",
        )
                   for value in ("nan", "inf", "0", "-0.1")]
        invalid.append(("--plant-width", "2"))  # Default 0.12 m fruit radius invades widened stem.
        invalid.append(("--plant-collision-width", "2"))
        for index, arguments in enumerate(invalid):
            with self.subTest(arguments=arguments):
                name = f"invalid{index}"
                output, metadata = self.root / f"{name}.sdf", self.root / f"{name}.json"
                output.write_text("original world")
                metadata.write_text("original metadata")
                _, _, result = self.generate(name, arguments, check=False)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("error:", result.stderr)
                self.assertEqual(output.read_text(), "original world")
                self.assertEqual(metadata.read_text(), "original metadata")

    def test_layout_bounds_reject_rows_that_leave_the_ground(self):
        output, metadata = self.root / "out_of_bounds.sdf", self.root / "out_of_bounds.json"
        output.write_text("original world")
        metadata.write_text("original metadata")
        _, _, result = self.generate(
            "out_of_bounds",
            ("--rows", "10", "--plants-per-row", "15", "--row-spacing", "3.0"),
            randomize=False,
            check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("plant rows exceed ground-x", result.stderr)
        self.assertEqual(output.read_text(), "original world")
        self.assertEqual(metadata.read_text(), "original metadata")

    def test_scaled_stem_neighbor_clearance_uses_actual_width_and_height(self):
        common = ("--rows", "3", "--plants-per-row", "3", "--row-spacing", "0.45",
                  "--plant-spacing", "0.45", "--origin-x", "0", "--origin-y", "0",
                  "--yaw-jitter", "0.5", "--fruit-count-min", "4", "--fruit-count-max", "4",
                  "--fruit-height-min", "1.4", "--fruit-height-max", "1.7",
                  "--fruit-diameter-min", "0.04", "--fruit-diameter-max", "0.06",
                  "--fruit-radius-min", "0.22", "--fruit-radius-max", "0.35",
                  "--fruit-min-clearance", "0.01", "--seed", "62")
        _, control, _ = self.generate("control", common)
        world, data, _ = self.generate("scaled", common + ("--plant-height", "2", "--plant-width", "2.6"))
        self.assert_dimensions(world, data, 2, 2.6)
        dimensions = [float(value) for value in self.plant_models(world, data)[0].findtext(
            "link/collision/geometry/box/size").split()]
        self.assertEqual(len(data["fruits"]), 36)
        # This control establishes that the fixture exercises neighboring stems,
        # rather than accidentally avoiding them with the selected seed.
        neighbor_conflicts = [(fruit, plant) for fruit in control["fruits"] for plant in control["plants"]
                              if fruit["plant_id"] != plant["id"]
                              and box_distance(fruit, plant, dimensions) < fruit["radius"] + 0.01]
        self.assertTrue(neighbor_conflicts)
        self.assertNotEqual(data["fruits"], control["fruits"])
        for fruit in data["fruits"]:
            for plant in data["plants"]:
                self.assertGreaterEqual(box_distance(fruit, plant, dimensions),
                                        fruit["radius"] + 0.01 - 1e-10)


if __name__ == "__main__":
    unittest.main()
