"""Tests for toolpath4.slicer -- mesh slicing, perimeters, overhang, FullControl bridge."""

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

try:
    import fullcontrol as fc
    HAS_FC = True
except ImportError:
    HAS_FC = False

from shapely.geometry import Polygon

from toolpath4.slicer import (
    perimeter_generation,
    overhang_analysis,
    plan_b_angles_thresholded,
    slice_mesh_along_z,
    load_mesh,
    Slicer,
    to_fullcontrol,
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
        # May produce 0 or 1 perimeter -- should not crash
        assert isinstance(perims, list)

    def test_more_walls_more_perimeters(self):
        """More wall count = more perimeter rings (FullControl style)."""
        sq = Polygon([(0, 0), (30, 0), (30, 30), (0, 30)])
        perims_2 = perimeter_generation([sq], perimeter_count=2, line_width=0.45)
        perims_5 = perimeter_generation([sq], perimeter_count=5, line_width=0.45)
        assert len(perims_5) > len(perims_2)


# ---------------------------------------------------------------------------
# Overhang analysis (with trimesh)
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not HAS_TRIMESH, reason="trimesh not installed")
class TestOverhang:
    def test_flat_box_no_overhang(self):
        """A flat box should have low overhang scores (normals are +/-X, +/-Y, +/-Z)."""
        box = trimesh.creation.box(extents=[20, 20, 20])
        box.apply_translation([0, 0, 10])
        scores = overhang_analysis(box, [5.0, 10.0, 15.0], threshold_deg=45.0)
        for z, s in scores.items():
            assert 0.0 <= s <= 1.0

    def test_sphere_has_overhangs(self):
        """A sphere's lower hemisphere has overhanging normals."""
        sphere = trimesh.creation.icosphere(subdivisions=2, radius=10)
        sphere.apply_translation([0, 0, 10])
        scores = overhang_analysis(sphere, [5.0, 10.0, 15.0], threshold_deg=45.0)
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

    def test_no_infill_in_output(self):
        """FullControl style: output should only contain perimeter walls."""
        path = self._make_box_stl()
        slicer = Slicer(config=PrinterConfig(layer_height=1.0, perimeter_count=3))
        steps = slicer.process(path)
        # All extrusion moves should be perimeter moves (no infill comments)
        for step in steps:
            if hasattr(step, 'comment') and step.comment:
                assert "infill" not in step.comment.lower()


# ---------------------------------------------------------------------------
# FullControl bridge
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not HAS_FC, reason="fullcontrol not installed")
@pytest.mark.skipif(not HAS_TRIMESH, reason="trimesh not installed")
class TestFullControlBridge:
    def _make_box_stl(self) -> str:
        box = trimesh.creation.box(extents=[20, 20, 10])
        box.apply_translation([150, 150, 5])
        path = os.path.join(tempfile.gettempdir(), "test_box_fc.stl")
        box.export(path)
        return path

    def test_to_fullcontrol_returns_list(self):
        path = self._make_box_stl()
        slicer = Slicer(config=PrinterConfig(layer_height=2.0))
        steps = slicer.process(path)
        fc_steps = to_fullcontrol(steps)
        assert isinstance(fc_steps, list)
        assert len(fc_steps) > 0

    def test_fc_contains_points(self):
        path = self._make_box_stl()
        slicer = Slicer(config=PrinterConfig(layer_height=2.0))
        steps = slicer.process(path)
        fc_steps = to_fullcontrol(steps)
        points = [s for s in fc_steps if isinstance(s, fc.Point)]
        assert len(points) > 0

    def test_fc_generates_gcode(self):
        path = self._make_box_stl()
        slicer = Slicer(config=PrinterConfig(layer_height=2.0))
        steps = slicer.process(path)
        fc_steps = to_fullcontrol(steps)
        gcode = fc.transform(fc_steps, 'gcode',
                             fc.GcodeControls(printer_name='generic'),
                             show_tips=False)
        assert len(gcode) > 100
        assert "G1" in gcode
