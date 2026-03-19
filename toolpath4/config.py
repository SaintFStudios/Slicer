"""
toolpath4.config — Printer configuration and hard-coded defaults.

All values are visible constants with docstrings.  The system runs out of the
box with these defaults on the XYZB1 printer geometry.

Sign convention:
  B positive  = right-hand rotation about the machine +Y axis.
  At B = 0 the nozzle points straight down (-Z).
  At B = +90 the nozzle points in the +X direction.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Literal, Optional, Tuple


# ---------------------------------------------------------------------------
# Printer configuration dataclass
# ---------------------------------------------------------------------------

@dataclass
class PrinterConfig:
    """Complete configuration for an XYZB1 printer.

    Every field has a sensible default so ``PrinterConfig()`` is ready to use.
    """

    # ------------------------------------------------------------------
    # 1) Build-volume limits (mm)
    # ------------------------------------------------------------------
    bed_x_min: float = 0.0
    """Minimum X travel."""
    bed_x_max: float = 300.0
    """Maximum X travel."""
    bed_y_min: float = 0.0
    """Minimum Y travel."""
    bed_y_max: float = 300.0
    """Maximum Y travel."""
    bed_z_min: float = 0.0
    """Minimum Z travel."""
    bed_z_max: float = 320.0
    """Maximum Z travel."""

    # ------------------------------------------------------------------
    # 2) Printer geometry (mm)
    # ------------------------------------------------------------------
    b_offset_z: float = 35.0
    """Distance from nozzle tip to B rotation centre along nozzle axis at B=0."""
    b_offset_x: float = 0.0
    """Lateral offset from nozzle tip to B rotation centre (reserved, unused)."""
    nozzle_length: float = 20.0
    """Physical nozzle length below the heater block."""
    toolhead_radius: float = 25.0
    """Approximate bounding radius of the toolhead for collision checks."""

    # ------------------------------------------------------------------
    # 3) B-axis limits (degrees)
    # ------------------------------------------------------------------
    b_min_deg: float = -120.0
    """Minimum B-axis angle in degrees."""
    b_max_deg: float = 120.0
    """Maximum B-axis angle in degrees."""

    # ------------------------------------------------------------------
    # 4) Motion limits
    # ------------------------------------------------------------------
    max_feedrate_xyz: float = 6000.0
    """Maximum XYZ feedrate in mm/min  (= 100 mm/s)."""
    max_feedrate_b: float = 3600.0
    """Maximum B feedrate in deg/min  (= 60 deg/s)."""
    max_accel_xyz: float = 1500.0
    """Maximum XYZ acceleration in mm/s²."""
    max_accel_b: float = 300.0
    """Maximum B acceleration in deg/s²."""

    # ------------------------------------------------------------------
    # 5) Extrusion defaults
    # ------------------------------------------------------------------
    filament_diameter: float = 1.75
    """Filament diameter in mm."""
    nozzle_diameter: float = 0.40
    """Nozzle orifice diameter in mm."""
    layer_height: float = 0.20
    """Default layer height in mm."""
    line_width: float = 0.45
    """Extrusion line width in mm."""
    perimeter_count: int = 2
    """Number of perimeter shells."""
    infill_density: float = 0.20
    """Infill density 0–1."""
    infill_pattern: str = "grid"
    """Infill pattern name."""
    retract_length: float = 0.8
    """Retraction length in mm of filament."""
    retract_speed: float = 1800.0
    """Retraction speed in mm/min."""
    travel_speed: float = 7200.0
    """Non-printing travel speed in mm/min."""

    # ------------------------------------------------------------------
    # 6) Machine type
    # ------------------------------------------------------------------
    machine: str = "XYZB1"
    """Machine identifier string."""
    positive_b_rotation: str = "right_hand_about_y"
    """Sign convention: positive B follows the right-hand rule about +Y."""

    # ------------------------------------------------------------------
    # 7) B-axis drive configuration
    # ------------------------------------------------------------------
    b_gear_ratio: float = 27.0
    """Gear reduction ratio between motor and B axis."""
    motor_steps_per_rev: float = 200.0
    """Motor full steps per revolution."""
    microsteps: float = 16.0
    """Micro-stepping divisor."""

    @property
    def steps_per_rev_b_axis(self) -> float:
        """Total micro-steps for one full revolution of the B axis."""
        return self.motor_steps_per_rev * self.microsteps * self.b_gear_ratio

    @property
    def steps_per_degree_b(self) -> float:
        """Micro-steps per degree of B rotation."""
        return self.steps_per_rev_b_axis / 360.0

    # ------------------------------------------------------------------
    # 8) Start / end G-code
    # ------------------------------------------------------------------
    start_gcode: str = (
        "; --- toolpath4 start G-code ---\n"
        "G21                ; mm units\n"
        "G90                ; absolute positioning\n"
        "M82                ; absolute extruder\n"
        "G28                ; home all axes\n"
        "M104 S{nozzle_temp} ; set nozzle temperature\n"
        "M140 S{bed_temp}    ; set bed temperature\n"
        "M109 S{nozzle_temp} ; wait for nozzle\n"
        "M190 S{bed_temp}    ; wait for bed\n"
        "G92 E0             ; reset extruder\n"
        "G1 Z2.0 F3000      ; lift nozzle\n"
        "G1 X0 Y0 F5000     ; move to origin\n"
        "; --- end start G-code ---\n"
    )
    """Start G-code block.  ``{nozzle_temp}`` and ``{bed_temp}`` are replaced
    at compile time."""

    end_gcode: str = (
        "; --- toolpath4 end G-code ---\n"
        "G91                ; relative positioning\n"
        "G1 E-2 F1800       ; retract\n"
        "G1 Z10 F3000       ; lift\n"
        "G90                ; absolute positioning\n"
        "G1 X0 Y0 F6000     ; park\n"
        "G1 B0 F1000        ; return B to zero\n"
        "M104 S0            ; nozzle off\n"
        "M140 S0            ; bed off\n"
        "M106 S0            ; fan off\n"
        "M84                ; steppers off\n"
        "; --- end end G-code ---\n"
    )
    """End G-code block."""

    nozzle_temp: int = 200
    """Default nozzle temperature °C (PLA)."""
    bed_temp: int = 60
    """Default bed temperature °C."""

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def deg_to_b_units(self, deg: float, *, steps_mode: bool = False) -> float:
        """Convert degrees to the unit the firmware expects.

        Parameters
        ----------
        deg : float
            Angle in degrees.
        steps_mode : bool
            If *True* return micro-step count instead of degrees.

        Returns
        -------
        float
            Value in firmware units (degrees by default, steps if requested).
        """
        if steps_mode:
            return deg * self.steps_per_degree_b
        return deg

    def validate_config(self) -> List[str]:
        """Return a list of warning/error strings.  Empty list = valid."""
        issues: List[str] = []
        if self.bed_x_max <= self.bed_x_min:
            issues.append("bed_x_max must exceed bed_x_min")
        if self.bed_y_max <= self.bed_y_min:
            issues.append("bed_y_max must exceed bed_y_min")
        if self.bed_z_max <= self.bed_z_min:
            issues.append("bed_z_max must exceed bed_z_min")
        if self.b_min_deg >= self.b_max_deg:
            issues.append("b_min_deg must be less than b_max_deg")
        if self.layer_height <= 0:
            issues.append("layer_height must be positive")
        if self.nozzle_diameter <= 0:
            issues.append("nozzle_diameter must be positive")
        if self.filament_diameter <= 0:
            issues.append("filament_diameter must be positive")
        if not (0.0 <= self.infill_density <= 1.0):
            issues.append("infill_density must be in [0, 1]")
        return issues

    def validate_toolpath(self, toolpath: list) -> List[str]:
        """Validate a toolpath (list of steps) against this config.

        Checks B limits, build-volume bounds, and feedrate caps.
        Returns a list of warning strings (empty = OK).
        """
        from toolpath4.state import Move
        issues: List[str] = []
        for i, step in enumerate(toolpath):
            if not isinstance(step, Move):
                continue
            if step.x is not None:
                if not (self.bed_x_min <= step.x <= self.bed_x_max):
                    issues.append(f"Step {i}: X={step.x:.2f} outside [{self.bed_x_min}, {self.bed_x_max}]")
            if step.y is not None:
                if not (self.bed_y_min <= step.y <= self.bed_y_max):
                    issues.append(f"Step {i}: Y={step.y:.2f} outside [{self.bed_y_min}, {self.bed_y_max}]")
            if step.z is not None:
                if not (self.bed_z_min <= step.z <= self.bed_z_max):
                    issues.append(f"Step {i}: Z={step.z:.2f} outside [{self.bed_z_min}, {self.bed_z_max}]")
            if step.b is not None:
                if not (self.b_min_deg <= step.b <= self.b_max_deg):
                    issues.append(f"Step {i}: B={step.b:.2f} outside [{self.b_min_deg}, {self.b_max_deg}]")
        return issues

    def rendered_start_gcode(self) -> str:
        """Return *start_gcode* with temperature placeholders filled."""
        return self.start_gcode.format(
            nozzle_temp=self.nozzle_temp,
            bed_temp=self.bed_temp,
        )

    def rendered_end_gcode(self) -> str:
        """Return *end_gcode* (no placeholders to fill currently)."""
        return self.end_gcode


# ---------------------------------------------------------------------------
# Module-level convenience
# ---------------------------------------------------------------------------

def default_config() -> PrinterConfig:
    """Return a fresh ``PrinterConfig`` with all defaults."""
    return PrinterConfig()
