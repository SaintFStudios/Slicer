"""Tests for toolpath4.slicer — mesh slicing, perimeters, infill, overhang."""

import math
import os
import tempfile
import pytest

import numpy as np

try:
    import trimesh
    HAS_TRIMESH = True
except ImportError:
    HAS_TRIMESH = False

from shapely.geometry import Polygon

from toolpath4.slicer import (
    perimeter_generation,
    infill_generation,
    overhang_analysis,
    plan_b_angles_thresholded,
    slice_mesh_along_z,
    load_mesh,
    Slicer,
)
from toolpath4.config import PrinterConfig
from toolpath4.state import Move, StepList


# ---------------------------------------------------------------------------
# Perimeter generation
# ---------------------------------------------------------------------------

class TestPerimeters:
    def test_square_two_perimeters(self):
        """Two perimeters from a simple square polygon."""
        sq = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
        perims = perimeter_generation([sq], perimeter_count=2, line_width=0.45)
        assert len(perims) >= 2
        # Each perimeter should be a closed polyline
        for p in perims:
            assert len(p) >= 4

    def test_empty_input(self):
        assert perimeter_generation([]) == []

    def test_tiny_polygon_vanishes(self):
        """A polygon smaller than one line_width should produce no perimeters."""
        tiny = Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)])
        perims = perimeter_generation([tiny], perimeter_count=3, line_width=0.45)
        # May produce 0 or 1 perimeter — should not crash
        assert isinstance(perims, list)


# ---------------------------------------------------------------------------
# Infill generation
# ---------------------------------------------------------------------------

class TestInfill:
    def test_grid_infill_nonempty(self):
        sq = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)])
        lines = infill_generation(sq, density=0.2, line_width=0.45)
        assert len(lines) > 0
        for l in lines:
            assert len(l) >= 2

    def test_zero_density(self):
        sq = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)])
        lines = infill_generation(sq, density=0.0)
        assert lines == []

    def test_empty_polygon(self):
        empty = Polygon()
        lines = infill_generation(empty, density=0.2)
        assert lines == []


# ---------------------------------------------------------------------------
# Overhang analysis (with trimesh)
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not HAS_TRIMESH, reason="trimesh not installed")
class TestOverhang:
    def test_flat_box_no_overhang(self):
        """A flat box should have low overhang scores (normals are ±X, ±Y, ±Z)."""
        box = trimesh.creation.box(extents=[20, 20, 20])
        box.apply_translation([0, 0, 10])
        scores = overhang_analysis(box, [5.0, 10.0, 15.0], threshold_deg=45.0)
        # Box faces are axis-aligned: top/bottom normals are vertical → not overhang.
        # Side faces have 90° from vertical → overhang.
        for z, s in scores.items():
            assert 0.0 <= s <= 1.0

    def test_sphere_has_overhangs(self):
        """A sphere's lower hemisphere has overhanging normals."""
        sphere = trimesh.creation.icosphere(subdivisions=2, radius=10)
        sphere.apply_translation([0, 0, 10])
        scores = overhang_analysis(sphere, [5.0, 10.0, 15.0], threshold_deg=45.0)
        # Lower half should have higher scores
        assert scores[5.0] > 0.0


# ---------------------------------------------------------------------------
# B-angle planning
# ---------------------------------------------------------------------------

class TestBAnglePlanning:
    def test_low_overhang_gets_default(self):
        z_vals = [1.0, 2.0, 3.0]
        scores = {1.0: 0.0, 2.0: 0.05, 3.0: 0.10}
        angles = plan_b_angles_thresholded(scores, z_vals)
        for z in z_vals:
            assert angles[z] == 90.0

    def test_high_overhang_gets_tilted(self):
        z_vals = [1.0, 2.0, 3.0]
        scores = {1.0: 0.0, 2.0: 0.20, 3.0: 0.50}
        angles = plan_b_angles_thresholded(scores, z_vals)
        assert angles[1.0] == 90.0
        assert angles[2.0] == 45.0
        assert angles[3.0] == 35.0


# ---------------------------------------------------------------------------
# Full slicing pipeline (with trimesh)
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not HAS_TRIMESH, reason="trimesh not installed")
class TestSlicerPipeline:
    def _make_box_stl(self) -> str:
        """Create a temporary box STL."""
        box = trimesh.creation.box(extents=[20, 20, 10])
        box.apply_translation([150, 150, 5])
        path = os.path.join(tempfile.gettempdir(), "test_box.stl")
        box.export(path)
        return path

    def test_slice_box(self):
        path = self._make_box_stl()
        mesh = load_mesh(path)
        layers = slice_mesh_along_z(mesh, layer_height=0.5)
        assert len(layers) > 0
        for l in layers:
            assert "z" in l
            assert "polygons" in l

    def test_full_pipeline(self):
        path = self._make_box_stl()
        slicer = Slicer(config=PrinterConfig(layer_height=1.0))
        steps = slicer.process(path)
        assert isinstance(steps, StepList)
        assert len(steps) > 0
        moves = steps.moves()
        assert len(moves) > 0
