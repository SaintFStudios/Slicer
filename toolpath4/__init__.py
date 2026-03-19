"""
toolpath4 — A Python-first 4-axis (XYZB) toolpath and slicing framework.

Architecture inspired by:
  - PrusaSlicer's "core engine + wrappers + tests" structure (AGPL-3.0).
  - FullControl's "stream of points + state" concept (GPL-3.0).
No verbatim code from either project is included.

Axes:
  X, Y, Z — linear translation in mm.
  B       — rotary axis rotating the nozzle about the machine Y-axis.

The core data model is a list of Step objects (Move or StateChange).
"""

__version__ = "0.1.0"

from toolpath4.state import State, Move, StateChange, StepList
from toolpath4.geometry import polyline, circle, helix, oscillating_b, densify
from toolpath4.kinematics import tip_to_pivot, pivot_to_tip
from toolpath4.config import PrinterConfig, default_config
from toolpath4.compiler import compile_gcode, dry_run
from toolpath4.preview import preview_matplotlib, preview_plotly
from toolpath4.slicer import Slicer

__all__ = [
    "State", "Move", "StateChange", "StepList",
    "polyline", "circle", "helix", "oscillating_b", "densify",
    "tip_to_pivot", "pivot_to_tip",
    "PrinterConfig", "default_config",
    "compile_gcode", "dry_run",
    "preview_matplotlib", "preview_plotly",
    "Slicer",
]
