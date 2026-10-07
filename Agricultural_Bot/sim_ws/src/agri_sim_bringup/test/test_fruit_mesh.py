"""Check the extracted tomato against the original mesh, independently of its exporter."""

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
SOURCE = WORLD_PACKAGE / "models/tomato_0/meshes/tomato.dae"
FRUIT = WORLD_PACKAGE / "models/tomato_fruit/meshes/fruit.dae"
EXTRACTOR = WORLD_PACKAGE / "scripts/extract_tomato_fruit.py"
NAMESPACE = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def read_mesh(path, node_name):
    """Read independent per-corner position, normal and UV indices from COLLADA."""
    root = ET.parse(path).getroot()
    node = root.find(f".//c:visual_scene/c:node[@name='{node_name}']", NAMESPACE)
    geometry_id = node.find("c:instance_geometry", NAMESPACE).get("url").removeprefix("#")
    mesh = root.find(f"c:library_geometries/c:geometry[@id='{geometry_id}']/c:mesh", NAMESPACE)
    arrays = {}
    for source in mesh.findall("c:source", NAMESPACE):
        values = [float(value) for value in source.findtext("c:float_array", namespaces=NAMESPACE).split()]
        accessor = source.find("c:technique_common/c:accessor", NAMESPACE)
        stride = int(accessor.get("stride", "1"))
        offset = int(accessor.get("offset", "0"))
        arrays[source.get("id")] = [tuple(values[offset + index * stride:offset + (index + 1) * stride])
                                     for index in range(int(accessor.get("count")))]
    positions_id = mesh.find("c:vertices/c:input[@semantic='POSITION']", NAMESPACE).get("source")[1:]
    faces = mesh.findall("c:triangles", NAMESPACE)
    if len(faces) != 1:
        raise AssertionError("Expected one triangle primitive")
    primitive = faces[0]
    inputs = {entry.get("semantic"): entry for entry in primitive.findall("c:input", NAMESPACE)}
    stride = max(int(entry.get("offset")) for entry in inputs.values()) + 1
    flat = [int(value) for value in primitive.findtext("c:p", namespaces=NAMESPACE).split()]
    index_rows = [flat[start:start + stride] for start in range(0, len(flat), stride)]
    payload = {}
    for semantic in ("VERTEX", "NORMAL", "TEXCOORD"):
        entry = inputs[semantic]
        source_id = positions_id if semantic == "VERTEX" else entry.get("source")[1:]
        payload[semantic] = arrays[source_id]
    corners = [tuple(row[int(inputs[semantic].get("offset"))]
                     for semantic in ("VERTEX", "NORMAL", "TEXCOORD")) for row in index_rows]
    for corner in corners:
        for index, semantic in zip(corner, ("VERTEX", "NORMAL", "TEXCOORD")):
            if not 0 <= index < len(payload[semantic]):
                raise AssertionError(f"Invalid {semantic} index")
    payload["triangles"] = [corners[start:start + 3] for start in range(0, len(corners), 3)]
    payload["root"] = root
    payload["node"] = node
    return payload


def connected_vertex_sets(triangles):
    """Walk an adjacency graph; do not reuse the extraction implementation."""
    neighbors = {}
    for triangle in triangles:
        indices = [corner[0] for corner in triangle]
        for index in indices:
            neighbors.setdefault(index, set()).update(indices)
    remaining = set(neighbors)
    groups = []
    while remaining:
        pending = [min(remaining)]
        visited = set()
        while pending:
            index = pending.pop()
            if index in visited:
                continue
            visited.add(index)
            pending.extend(neighbors[index] - visited)
        groups.append(visited)
        remaining.difference_update(visited)
    return groups


class FruitMeshTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = read_mesh(SOURCE, "Fruit1")
        cls.fruit = read_mesh(FRUIT, "Fruit")

    def test_asset_is_one_centered_fruit_inside_unit_diameter_sphere(self):
        self.assertEqual(len(self.fruit["root"].findall("c:library_geometries/c:geometry", NAMESPACE)), 1)
        self.assertEqual(len(connected_vertex_sets(self.source["triangles"])), 3)
        self.assertEqual(len(connected_vertex_sets(self.fruit["triangles"])), 1)
        self.assertEqual(len(self.fruit["VERTEX"]), 576)
        self.assertEqual(len(self.fruit["triangles"]), 1080)
        unit = self.fruit["root"].find("c:asset/c:unit", NAMESPACE)
        self.assertEqual(float(unit.get("meter")), 1)
        self.assertEqual(self.fruit["root"].findtext("c:asset/c:up_axis", namespaces=NAMESPACE), "Z_UP")
        identity = [1 if index % 5 == 0 else 0 for index in range(16)]
        self.assertEqual([float(value) for value in
                          self.fruit["node"].findtext("c:matrix", namespaces=NAMESPACE).split()], identity)
        vertices = self.fruit["VERTEX"]
        self.assertAlmostEqual(max(math.dist(vertex, (0, 0, 0)) for vertex in vertices), 0.5, delta=1e-12)
        for axis in range(3):
            low = min(vertex[axis] for vertex in vertices)
            high = max(vertex[axis] for vertex in vertices)
            self.assertAlmostEqual((low + high) / 2, 0, delta=1e-12)
            self.assertGreater(high - low, 0.8)
            self.assertLess(high - low, 1)

    def test_original_single_body_triangle_winding_normals_and_uv_are_preserved(self):
        component = connected_vertex_sets(self.source["triangles"])[2]
        original_triangles = [triangle for triangle in self.source["triangles"]
                              if triangle[0][0] in component]
        self.assertEqual(len(original_triangles), len(self.fruit["triangles"]))
        original_positions = [self.source["VERTEX"][index] for index in component]
        center = tuple((min(position[axis] for position in original_positions)
                        + max(position[axis] for position in original_positions)) / 2 for axis in range(3))
        radius = max(math.dist(position, center) for position in original_positions)
        # Compare expanded triangle corners so remapped UV seam indices cannot hide a mismatch.
        for original, extracted in zip(original_triangles, self.fruit["triangles"]):
            for original_corner, extracted_corner in zip(original, extracted):
                for coordinate, axis in zip(self.fruit["VERTEX"][extracted_corner[0]], range(3)):
                    expected = (self.source["VERTEX"][original_corner[0]][axis] - center[axis]) / (2 * radius)
                    self.assertAlmostEqual(coordinate, expected, delta=1e-12)
                self.assertEqual(self.source["NORMAL"][original_corner[1]],
                                 self.fruit["NORMAL"][extracted_corner[1]])
                self.assertEqual(self.source["TEXCOORD"][original_corner[2]],
                                 self.fruit["TEXCOORD"][extracted_corner[2]])
        metadata = json.loads((FRUIT.parent.parent / "SOURCE.json").read_text())
        self.assertEqual(metadata["source_sha256"], hashlib.sha256(SOURCE.read_bytes()).hexdigest())
        self.assertEqual(metadata["output_sha256"], hashlib.sha256(FRUIT.read_bytes()).hexdigest())
        self.assertEqual(metadata["source_component_count"], 3)
        self.assertEqual(metadata["source_component_index"], 2)
        self.assertEqual(set(metadata["source_position_indices"]), component)

    def test_reextract_is_reproducible_and_matches_checked_in_asset(self):
        source_before = SOURCE.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            outputs = []
            for name in ("first", "second"):
                output, metadata = root / f"{name}.dae", root / f"{name}.json"
                subprocess.run([sys.executable, str(EXTRACTOR), "--source", str(SOURCE),
                                "--output", str(output), "--metadata", str(metadata)],
                               check=True, capture_output=True, text=True, timeout=20)
                outputs.append((output.read_bytes(), metadata.read_bytes()))
            self.assertEqual(outputs[0], outputs[1])
            self.assertEqual(outputs[0][0], FRUIT.read_bytes())
        self.assertEqual(SOURCE.read_bytes(), source_before)

    def test_invalid_extraction_does_not_overwrite_source_or_existing_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, output, metadata = root / "source.dae", root / "fruit.dae", root / "source.json"
            source.write_bytes(SOURCE.read_bytes())
            output.write_text("existing fruit")
            metadata.write_text("existing metadata")
            for arguments in (("--component-index", "3"), ("--source-node", "missing"),
                              ("--output", str(source)), ("--metadata", str(source)),
                              ("--metadata", str(output))):
                result = subprocess.run([sys.executable, str(EXTRACTOR), "--source", str(source),
                                         "--output", str(output), "--metadata", str(metadata),
                                         *arguments], capture_output=True, text=True, timeout=20)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("error:", result.stderr)
                self.assertEqual(source.read_bytes(), SOURCE.read_bytes())
                self.assertEqual(output.read_text(), "existing fruit")
                self.assertEqual(metadata.read_text(), "existing metadata")


if __name__ == "__main__":
    unittest.main()
