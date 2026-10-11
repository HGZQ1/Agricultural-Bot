"""Negative contracts for exact footprint and mask validation."""

from pathlib import Path

from agri_task_manager.route_validation import MapGrid, OCCUPIED
from agri_task_manager.waypoint_schema import Pose2D


def _write_grid(tmp_path: Path) -> MapGrid:
    image = tmp_path / 'grid.pgm'
    image.write_text(
        'P2\n3 3\n255\n'
        '255 255 255\n'
        '255 0 255\n'
        '255 255 255\n',
        encoding='ascii',
    )
    config = tmp_path / 'grid.yaml'
    config.write_text(
        'image: grid.pgm\n'
        'mode: trinary\n'
        'resolution: 1.0\n'
        'origin: [0.0, 0.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n',
        encoding='utf-8',
    )
    return MapGrid.from_yaml(config)


def test_footprint_detects_cell_square_when_cell_centre_is_outside(tmp_path):
    """Catch a collision that a cell-centre-only test would miss."""
    grid = _write_grid(tmp_path)
    assert grid.cell_status(1, 1) == OCCUPIED
    report = grid.footprint_report(
        Pose2D(0.5, 1.5, 0.0),
        front_m=0.51,
        rear_m=0.10,
        half_width_m=0.40,
    )
    assert report.occupied_cells == 1
    assert not report.clear


def test_footprint_counts_cells_outside_the_map(tmp_path):
    """Reject a footprint whose origin is valid but body crosses the map."""
    grid = _write_grid(tmp_path)
    report = grid.footprint_report(
        Pose2D(0.1, 0.1, 0.0),
        front_m=0.2,
        rear_m=0.2,
        half_width_m=0.2,
    )
    assert report.outside_cells > 0
    assert not report.clear
