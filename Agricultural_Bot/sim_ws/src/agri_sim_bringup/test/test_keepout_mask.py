"""Offline checks for parameterized tomato-field Nav2 mask generation."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SCRIPT = (
    Path(__file__).parents[2] / 'agri_greenhouse_worlds'
    / 'scripts' / 'generate_keepout_mask.py'
)
SPEC = importlib.util.spec_from_file_location('generate_keepout_mask', SCRIPT)
MASK = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MASK)


class KeepoutMaskTest(unittest.TestCase):
    """Check map geometry, layout validation, and deterministic raster output."""

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.map_yaml = self.root / 'map.yaml'
        self.map_pgm = self.root / 'map.pgm'
        MASK.write_pgm(self.map_pgm, 160, 160, bytes([255]) * (160 * 160))
        self.map_yaml.write_text(
            'image: map.pgm\nmode: trinary\nresolution: 0.05\n'
            'origin: [0.0, 0.0, 0.0]\nnegate: 0\n'
            'occupied_thresh: 0.65\nfree_thresh: 0.196\n',
            encoding='utf-8',
        )
        plants = []
        for row in range(2):
            for plant in range(3):
                plants.append({
                    'id': f'tomato_{len(plants)}',
                    'x': float(row * 2.0 + 1.0),
                    'y': float(plant * 0.7 + 2.0),
                    'yaw': 0.0,
                })
        self.metadata = self.root / 'field.json'
        self.metadata.write_text(json.dumps({
            'schema_version': 1,
            'coordinate_frame': 'world',
            'parameters': {
                'rows': 2,
                'plants_per_row': 3,
                'row_spacing': 2.0,
                'plant_spacing': 0.7,
            },
            'plant_geometry': {'width': 0.8},
            'plants': plants,
        }), encoding='utf-8')

    def test_render_matches_saved_map_dimensions_and_marks_rows(self):
        field = MASK.load_field_metadata(self.metadata)
        map_info = MASK.load_map_description(self.map_yaml)
        layout = MASK.resolve_layout(field, {
            'rows': 2, 'plants_per_row': 3,
            'row_spacing': 2.0, 'plant_spacing': 0.7,
            'plant_width': None,
        })
        pixels, report = MASK.render_mask(
            field, map_info, layout, (0.0, 0.0, 0.0), 0.1, True,
        )
        self.assertEqual(len(pixels), 160 * 160)
        self.assertGreater(report['keepout_cells'], 0)
        self.assertLess(report['keepout_cells'], len(pixels))
        self.assertEqual(pixels[0], 255)

    def test_layout_mismatch_is_rejected_before_rasterization(self):
        field = MASK.load_field_metadata(self.metadata)
        with self.assertRaisesRegex(ValueError, 'row_spacing'):
            MASK.resolve_layout(field, {
                'rows': 2, 'plants_per_row': 3,
                'row_spacing': 1.0, 'plant_spacing': 0.7,
                'plant_width': None,
            })

    def test_world_to_map_rotation_is_explicit(self):
        point = MASK.world_to_map((0.0, 2.0), (6.0, 0.0, -1.5707963267948966))
        self.assertAlmostEqual(point[0], 8.0)
        self.assertAlmostEqual(point[1], 0.0)


if __name__ == '__main__':
    unittest.main()
