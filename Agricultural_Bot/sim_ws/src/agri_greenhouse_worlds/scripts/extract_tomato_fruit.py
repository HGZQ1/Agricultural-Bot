#!/usr/bin/env python3
"""Extract one textured fruit body from the bundled, licensed tomato mesh.

The COLLADA remains material-free, like the source asset. The SDF visual uses
the original albedo texture; UV coordinates and normals are retained here.
This asset preparation tool needs only the Python standard library.
"""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


NS = "http://www.collada.org/2005/11/COLLADASchema"
ET.register_namespace("", NS)
PACKAGE = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PACKAGE / "models/tomato_0/meshes/tomato.dae"
DEFAULT_OUTPUT = PACKAGE / "models/tomato_fruit/meshes/fruit.dae"


def tag(name):
    return f"{{{NS}}}{name}"


def child(parent, element_name, text=None, **attributes):
    element = ET.SubElement(parent, tag(element_name), attributes)
    element.text = text
    return element


def float_source(mesh, source_id):
    source = mesh.find(f"{tag('source')}[@id='{source_id}']")
    if source is None:
        raise ValueError(f"Missing source {source_id}")
    values = list(map(float, source.find(tag("float_array")).text.split()))
    accessor = source.find(f"{tag('technique_common')}/{tag('accessor')}")
    stride = int(accessor.get("stride", "1"))
    offset = int(accessor.get("offset", "0"))
    count = int(accessor.get("count"))
    if offset + count * stride > len(values):
        raise ValueError(f"Invalid array bounds for {source_id}")
    return source, [tuple(values[offset + i * stride:offset + (i + 1) * stride])
                    for i in range(count)]


def components(triangles):
    """Vertex-index connected components, ordered by the first source vertex."""
    parents = {}

    def root(index):
        parents.setdefault(index, index)
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for triangle in triangles:
        a, b, c = triangle
        parents[root(b)] = root(a)
        parents[root(c)] = root(a)
    groups = {}
    for index in parents:
        groups.setdefault(root(index), set()).add(index)
    return sorted(groups.values(), key=min)


def make_source(mesh, identifier, source_template, values):
    """Keep accessor semantics while compacting the referenced source array."""
    stride = len(values[0])
    source = child(mesh, "source", id=identifier)
    child(source, "float_array",
          " ".join(format(value, ".17g") for row in values for value in row),
          id=f"{identifier}-array", count=str(len(values) * stride))
    technique = child(source, "technique_common")
    accessor = child(technique, "accessor", source=f"#{identifier}-array",
                     count=str(len(values)), stride=str(stride))
    original = source_template.find(f"{tag('technique_common')}/{tag('accessor')}")
    for param in original.findall(tag("param")):
        accessor.append(copy.deepcopy(param))


def extract(source_path, output_path, node_name="Fruit1", component_index=2):
    document = ET.parse(source_path).getroot()
    node = document.find(
        f"{tag('library_visual_scenes')}/{tag('visual_scene')}/"
        f"{tag('node')}[@name='{node_name}']")
    if node is None:
        raise ValueError(f"Unknown fruit node {node_name}")
    matrices = node.findall(tag("matrix"))
    identity = [1.0 if i % 5 == 0 else 0.0 for i in range(16)]
    if any(list(map(float, matrix.text.split())) != identity for matrix in matrices):
        raise ValueError("Only source nodes with identity transforms are supported")
    if any(node.findall(tag(name)) for name in ("rotate", "translate", "scale")):
        raise ValueError("Source node transforms must be baked into the mesh")
    instance = node.find(tag("instance_geometry"))
    geometry_id = instance.get("url").removeprefix("#")
    geometry = document.find(f"{tag('library_geometries')}/"
                             f"{tag('geometry')}[@id='{geometry_id}']")
    mesh = geometry.find(tag("mesh"))
    primitives = [entry for entry in mesh if entry.tag in
                  {tag("triangles"), tag("polylist"), tag("polygons")}]
    if len(primitives) != 1 or primitives[0].tag != tag("triangles"):
        raise ValueError("Expected one triangle primitive in the source fruit")
    primitive = primitives[0]
    inputs = primitive.findall(tag("input"))
    semantics = {entry.get("semantic"): entry for entry in inputs}
    if set(semantics) != {"VERTEX", "NORMAL", "TEXCOORD"}:
        raise ValueError("Expected positions, normals and UV coordinates")
    stride = max(int(entry.get("offset")) for entry in inputs) + 1
    indices = list(map(int, primitive.find(tag("p")).text.split()))
    if len(indices) != int(primitive.get("count")) * stride * 3:
        raise ValueError("Invalid triangle index count")
    corners = [indices[index:index + stride] for index in range(0, len(indices), stride)]
    vertex_offset = int(semantics["VERTEX"].get("offset"))
    triangle_vertices = [[corner[vertex_offset] for corner in corners[i:i + 3]]
                         for i in range(0, len(corners), 3)]
    groups = components(triangle_vertices)
    if not 0 <= component_index < len(groups):
        raise ValueError(f"Component must be in 0..{len(groups) - 1}")
    selected_vertices = groups[component_index]
    selected_corners = []
    for start, vertices in zip(range(0, len(corners), 3), triangle_vertices):
        if vertices[0] in selected_vertices:
            selected_corners.extend(corners[start:start + 3])

    sources = {}
    vertices_id = semantics["VERTEX"].get("source").removeprefix("#")
    vertices = mesh.find(f"{tag('vertices')}[@id='{vertices_id}']")
    position_id = vertices.find(f"{tag('input')}[@semantic='POSITION']").get("source")[1:]
    for semantic, entry in semantics.items():
        source_id = position_id if semantic == "VERTEX" else entry.get("source")[1:]
        template, values = float_source(mesh, source_id)
        offset = int(entry.get("offset"))
        used = sorted({corner[offset] for corner in selected_corners})
        sources[semantic] = (template, [values[index] for index in used],
                             {old: new for new, old in enumerate(used)})
    original_points = sources["VERTEX"][1]
    if any(len(point) != 3 for point in original_points):
        raise ValueError("Positions must have three dimensions")
    low = [min(point[axis] for point in original_points) for axis in range(3)]
    high = [max(point[axis] for point in original_points) for axis in range(3)]
    center = [(low[axis] + high[axis]) / 2 for axis in range(3)]
    radius = max(math.dist(point, center) for point in original_points)
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("Source fruit has no finite volume")
    multiplier = 0.5 / radius
    normalized = [tuple((point[axis] - center[axis]) * multiplier for axis in range(3))
                  for point in original_points]

    output = ET.Element(tag("COLLADA"), {"version": "1.4.1"})
    asset = copy.deepcopy(document.find(tag("asset")))
    contributor = child(asset, "contributor")
    child(contributor, "author", "Agricultural-Bot contributors")
    child(contributor, "authoring_tool", "extract_tomato_fruit.py (Python standard library)")
    output.append(asset)
    child(output, "library_effects")
    library = child(output, "library_geometries")
    result_geometry = child(library, "geometry", id="fruit-mesh", name="Fruit")
    result_mesh = child(result_geometry, "mesh")
    names = {"VERTEX": "fruit-positions", "NORMAL": "fruit-normals", "TEXCOORD": "fruit-uv"}
    for semantic in ("VERTEX", "NORMAL", "TEXCOORD"):
        template, values, _ = sources[semantic]
        make_source(result_mesh, names[semantic], template,
                    normalized if semantic == "VERTEX" else values)
    result_vertices = child(result_mesh, "vertices", id="fruit-vertices")
    child(result_vertices, "input", semantic="POSITION", source="#fruit-positions")
    result_triangles = child(result_mesh, "triangles", count=str(len(selected_corners) // 3))
    ordered_semantics = ("VERTEX", "NORMAL", "TEXCOORD")
    for offset, semantic in enumerate(ordered_semantics):
        attrs = {"semantic": semantic, "offset": str(offset),
                 "source": "#fruit-vertices" if semantic == "VERTEX" else f"#{names[semantic]}"}
        if semantic == "TEXCOORD":
            attrs["set"] = semantics[semantic].get("set", "0")
        child(result_triangles, "input", **attrs)
    remapped = []
    for corner in selected_corners:
        for semantic in ordered_semantics:
            remapped.append(str(sources[semantic][2][corner[int(semantics[semantic].get("offset"))]]))
    child(result_triangles, "p", " ".join(remapped))
    scenes = child(output, "library_visual_scenes")
    scene = child(scenes, "visual_scene", id="Scene", name="Scene")
    fruit_node = child(scene, "node", id="Fruit", name="Fruit", type="NODE")
    child(fruit_node, "matrix", "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1", sid="transform")
    child(fruit_node, "instance_geometry", url="#fruit-mesh", name="Fruit")
    child(child(output, "scene"), "instance_visual_scene", url="#Scene")
    ET.indent(output, space="  ")
    output_bytes = ET.tostring(output, encoding="utf-8", xml_declaration=True) + b"\n"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(output_bytes)
    metadata = {
        "schema_version": 1,
        "upstream_repository": "https://github.com/LCAS/aoc_tomato_farm",
        "upstream_revision": "d8243e48c92377fbd9754400e0ddb1fcffded9d7",
        "license": "Apache-2.0 (see package LICENSE and NOTICE)",
        "source_asset": "models/tomato_0/meshes/tomato.dae",
        "source_sha256": hashlib.sha256(source_path.read_bytes()).hexdigest(),
        "source_node": node_name,
        "source_geometry": geometry_id,
        "source_component_index": component_index,
        "source_component_count": len(groups),
        "source_component_first_vertex": min(selected_vertices),
        "source_position_indices": sorted(sources["VERTEX"][2]),
        "source_normal_indices": sorted(sources["NORMAL"][2]),
        "source_uv_indices": sorted(sources["TEXCOORD"][2]),
        "source_triangle_indices": [index for index, triangle in enumerate(triangle_vertices)
                                    if triangle[0] in selected_vertices],
        "source_bbox_min_m": low,
        "source_bbox_max_m": high,
        "source_center_m": center,
        "source_bounding_radius_m": radius,
        "normalization_multiplier": multiplier,
        "normalized_bounding_sphere_diameter": 1.0,
        "normalized_max_vertex_radius": max(math.dist(point, (0, 0, 0)) for point in normalized),
        "normalized_bbox_min": [min(point[axis] for point in normalized) for axis in range(3)],
        "normalized_bbox_max": [max(point[axis] for point in normalized) for axis in range(3)],
        "vertices": len(normalized),
        "triangles": len(selected_corners) // 3,
        "normal_and_uv_values": "Preserved; only index arrays are compacted/remapped",
        "ripe_texture": "model://tomato_0/materials/textures/AG15frt1.png",
        "unripe_texture": "model://tomato_0/materials/textures/AG15frt4.png",
        "output_asset": "models/tomato_fruit/meshes/fruit.dae",
        "output_sha256": hashlib.sha256(output_bytes).hexdigest(),
    }
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--source-node", default="Fruit1")
    parser.add_argument("--component-index", type=int, default=2)
    parser.add_argument("--metadata", type=Path,
                        help="Default: SOURCE.json in the output model directory")
    args = parser.parse_args()
    if args.source.resolve() == args.output.resolve():
        parser.error("The original source and extracted output must be different files")
    metadata_path = args.metadata or args.output.parent.parent / "SOURCE.json"
    if metadata_path.resolve() in {args.source.resolve(), args.output.resolve()}:
        parser.error("Metadata must use a separate path")
    try:
        metadata = extract(args.source, args.output, args.source_node, args.component_index)
    except (ValueError, ET.ParseError) as error:
        parser.error(str(error))
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    print(f"Extracted {metadata['vertices']} vertices / {metadata['triangles']} triangles to {args.output}")


if __name__ == "__main__":
    main()
