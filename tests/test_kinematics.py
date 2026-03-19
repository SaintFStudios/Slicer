"""Tests for toolpath4.kinematics — XYZB1 forward / inverse transforms."""

import math
import pytest
from toolpath4.config import PrinterConfig
from toolpath4.kinematics import tip_to_pivot, pivot_to_tip, check_collision_margin
from toolpath4.state import Move


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _approx(a, b, tol=1e-6):
    """Assert two tuples of floats are close."""
    for va, vb in zip(a, b):
        assert abs(va - vb) < tol, f"{a} != {b} (tol={tol})"


CFG = PrinterConfig(b_offset_z=35.0)


# ---------------------------------------------------------------------------
# tip_to_pivot / pivot_to_tip round-trip
# ---------------------------------------------------------------------------

class TestRoundTrip:
    """tip_to_pivot and pivot_to_tip must be exact inverses."""

    def test_b_zero(self):
        """At B=0 the pivot is directly above the tip by b_offset_z."""
        tip = (100.0, 100.0, 0.0)
        pivot = tip_to_pivot(tip, 0.0, CFG)
        _approx(pivot, (100.0, 100.0, 35.0))
        tip_back = pivot_to_tip(pivot, 0.0, CFG)
        _approx(tip_back, tip)

    def test_b_90(self):
        """At B=+90 the nozzle points in +X, so pivot shifts in +X."""
        tip = (100.0, 100.0, 0.0)
        pivot = tip_to_pivot(tip, 90.0, CFG)
        # R_y(90): v=[0,0,35] -> [35, 0, 0]  (sin90=1, cos90≈0)
        _approx(pivot, (135.0, 100.0, 0.0), tol=1e-4)
        tip_back = pivot_to_tip(pivot, 90.0, CFG)
        _approx(tip_back, tip, tol=1e-4)

    def test_b_neg_45(self):
        """Round-trip at B = -45°."""
        tip = (150.0, 150.0, 10.0)
        pivot = tip_to_pivot(tip, -45.0, CFG)
        tip_back = pivot_to_tip(pivot, -45.0, CFG)
        _approx(tip_back, tip, tol=1e-6)

    def test_arbitrary_angles(self):
        """Round-trip holds for many angles."""
        tip = (200.0, 50.0, 5.0)
        for deg in range(-120, 121, 15):
            pivot = tip_to_pivot(tip, float(deg), CFG)
            tip_back = pivot_to_tip(pivot, float(deg), CFG)
            _approx(tip_back, tip, tol=1e-6)


# ---------------------------------------------------------------------------
# Specific geometry checks
# ---------------------------------------------------------------------------

class TestGeometry:
    """Verify the rotation matrix and offset vector."""

    def test_b_180(self):
        """At B=180 the nozzle points in -Z (upward), pivot below tip."""
        tip = (100.0, 100.0, 50.0)
        pivot = tip_to_pivot(tip, 180.0, CFG)
        # R_y(180): v=[0,0,35] -> [0, 0, -35]
        _approx(pivot, (100.0, 100.0, 15.0), tol=1e-4)

    def test_y_unchanged(self):
        """B rotates about Y, so the Y component must never change."""
        tip = (100.0, 77.7, 20.0)
        for deg in [-90, -45, 0, 30, 60, 90, 120]:
            pivot = tip_to_pivot(tip, float(deg), CFG)
            assert abs(pivot[1] - 77.7) < 1e-10


# ---------------------------------------------------------------------------
# Collision check
# ---------------------------------------------------------------------------

class TestCollision:
    def test_centre_of_bed_ok(self):
        assert check_collision_margin((150, 150, 10), 0.0, CFG)

    def test_far_outside_fails(self):
        assert not check_collision_margin((-100, 150, 10), 0.0, CFG)
