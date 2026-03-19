"""Tests for toolpath4.compiler — G-code generation and dry-run stats."""

import math
import pytest

from toolpath4.config import PrinterConfig
from toolpath4.state import (
    ExtrusionMode,
    Move,
    State,
    StateChange,
    StepList,
)
from toolpath4.compiler import compile_gcode, dry_run, GcodeStats


CFG = PrinterConfig()


# ---------------------------------------------------------------------------
# Dry run
# ---------------------------------------------------------------------------

class TestDryRun:
    def test_empty_toolpath(self):
        stats = dry_run([], CFG)
        assert stats.total_path_length_mm == 0.0
        assert stats.move_count == 0

    def test_single_move(self):
        steps = [Move(x=0, y=0, z=0, b=0)]
        stats = dry_run(steps, CFG)
        assert stats.move_count == 1
        assert stats.total_path_length_mm == 0.0  # no previous move

    def test_two_moves_distance(self):
        s = State(feedrate=1200, extrusion_mode=ExtrusionMode.ON)
        steps = [
            Move(x=0, y=0, z=0, b=0, state=s),
            Move(x=10, y=0, z=0, b=0, state=s),
        ]
        stats = dry_run(steps, CFG)
        assert abs(stats.total_path_length_mm - 10.0) < 1e-6
        assert stats.total_extrusion_mm > 0

    def test_b_range(self):
        steps = [
            Move(x=0, y=0, z=0, b=-30),
            Move(x=10, y=0, z=0, b=45),
            Move(x=20, y=0, z=0, b=10),
        ]
        stats = dry_run(steps, CFG)
        assert stats.b_min_deg == -30.0
        assert stats.b_max_deg == 45.0

    def test_travel_no_extrusion(self):
        s_off = State(feedrate=7200, extrusion_mode=ExtrusionMode.OFF)
        steps = [
            Move(x=0, y=0, z=0, b=0, state=s_off),
            Move(x=50, y=0, z=0, b=0, state=s_off),
        ]
        stats = dry_run(steps, CFG)
        assert stats.total_extrusion_mm == 0.0


# ---------------------------------------------------------------------------
# G-code compilation
# ---------------------------------------------------------------------------

class TestCompileGcode:
    def test_contains_start_end(self):
        steps = [Move(x=100, y=100, z=0.2, b=0)]
        gcode = compile_gcode(steps, CFG, transform_to_machine=False)
        assert "G21" in gcode  # mm units in start_gcode
        assert "M84" in gcode  # steppers off in end_gcode

    def test_b_axis_emitted(self):
        steps = [
            Move(x=100, y=100, z=0.2, b=0, state=State(feedrate=1200)),
            Move(x=110, y=100, z=0.2, b=45, state=State(feedrate=1200)),
        ]
        gcode = compile_gcode(steps, CFG, transform_to_machine=False)
        assert "B45" in gcode or "B45.0" in gcode

    def test_retract_unretract(self):
        steps = [
            StateChange.retract(0.8),
            Move(x=50, y=50, z=1.0, b=0, state=State(
                feedrate=7200, extrusion_mode=ExtrusionMode.OFF
            )),
            StateChange.unretract(0.8),
        ]
        gcode = compile_gcode(steps, CFG, transform_to_machine=False)
        assert "E-0.8" in gcode
        assert "E0.8" in gcode

    def test_fan_command(self):
        steps = [StateChange.set_fan(0.5)]
        gcode = compile_gcode(steps, CFG)
        assert "M106 S127" in gcode  # 0.5 * 255 ≈ 127

    def test_no_transform_preserves_coords(self):
        """With transform_to_machine=False, XYZ pass through directly."""
        steps = [Move(x=100.0, y=200.0, z=5.0, b=0, state=State(feedrate=1200))]
        gcode = compile_gcode(steps, CFG, transform_to_machine=False)
        assert "X100" in gcode
        assert "Y200" in gcode
        assert "Z5" in gcode

    def test_modal_optimization(self):
        """Unchanged axis values should not be re-emitted."""
        s = State(feedrate=1200, extrusion_mode=ExtrusionMode.ON)
        steps = [
            Move(x=10, y=20, z=1, b=0, state=s),
            Move(x=20, y=20, z=1, b=0, state=s),  # Y, Z, B unchanged
        ]
        gcode = compile_gcode(steps, CFG, transform_to_machine=False)
        lines = gcode.strip().split("\n")
        # Find the second G1 line that has X20 — it should not repeat Y20
        g1_lines = [l for l in lines if l.startswith("G1") and "X20" in l]
        if g1_lines:
            assert "Y20" not in g1_lines[0] or "Y" not in g1_lines[0]
