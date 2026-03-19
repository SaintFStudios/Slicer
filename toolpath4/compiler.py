"""
toolpath4.compiler — Convert a point+state stream into Duet/RepRap G-code.

Output flavour targets Duet / RepRapFirmware but stays generic enough for
most Marlin-compatible controllers.

Modal optimisation: unchanged axis values and feedrates are not re-emitted.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from toolpath4.config import PrinterConfig, default_config
from toolpath4.state import (
    ExtrusionMode,
    Move,
    State,
    StateChange,
    StateChangeKind,
    StepList,
)
from toolpath4.kinematics import tip_to_pivot


# ---------------------------------------------------------------------------
# Stats returned by dry_run
# ---------------------------------------------------------------------------

@dataclass
class GcodeStats:
    """Summary statistics for a compiled toolpath."""
    total_path_length_mm: float
    total_extrusion_mm: float
    b_min_deg: float
    b_max_deg: float
    max_feedrate_xy: float
    layer_count: int
    move_count: int
    line_count: int


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _extrusion_per_mm(config: PrinterConfig) -> float:
    """Filament length per mm of toolpath (volume-conservation model).

    cross_section_deposit = line_width * layer_height
    cross_section_filament = π * (filament_diameter/2)²
    E_per_mm = deposit / filament
    """
    deposit = config.line_width * config.layer_height
    filament = math.pi * (config.filament_diameter / 2.0) ** 2
    return deposit / filament


def _fmt(v: float, decimals: int = 3) -> str:
    """Format a float to *decimals* places, stripping trailing zeros."""
    return f"{v:.{decimals}f}".rstrip("0").rstrip(".")


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def compile_gcode(
    toolpath: list,
    config: PrinterConfig | None = None,
    *,
    transform_to_machine: bool = True,
) -> str:
    """Compile a toolpath (list of Move / StateChange) into G-code text.

    Parameters
    ----------
    toolpath : list of Move and StateChange objects.
    config : printer configuration.
    transform_to_machine : if *True*, apply tip→pivot kinematic transform
        before emitting XYZ values.  Set to *False* if the toolpath is
        already in machine co-ordinates.

    Returns
    -------
    G-code as a single string.
    """
    if config is None:
        config = default_config()

    e_per_mm = _extrusion_per_mm(config)
    lines: List[str] = []

    # Start G-code
    lines.append(config.rendered_start_gcode())

    # Modal state tracking — we only emit values that change.
    prev_x: Optional[float] = None
    prev_y: Optional[float] = None
    prev_z: Optional[float] = None
    prev_b: Optional[float] = None
    prev_f: Optional[float] = None
    e_total: float = 0.0  # cumulative extrusion

    for step in toolpath:
        if isinstance(step, StateChange):
            lines.append(_compile_state_change(step, config))
            continue

        if not isinstance(step, Move):
            continue

        # Determine machine co-ordinates
        mx, my, mz, mb = step.x, step.y, step.z, step.b
        if transform_to_machine and None not in (mx, my, mz, mb):
            mx, my, mz = tip_to_pivot((mx, my, mz), mb, config)

        # Build G-code tokens
        is_extrude = step.state.extrusion_mode == ExtrusionMode.ON
        cmd = "G1" if is_extrude or step.state.feedrate != config.travel_speed else "G0"
        tokens: List[str] = [cmd]

        if mx is not None and mx != prev_x:
            tokens.append(f"X{_fmt(mx)}")
            prev_x = mx
        if my is not None and my != prev_y:
            tokens.append(f"Y{_fmt(my)}")
            prev_y = my
        if mz is not None and mz != prev_z:
            tokens.append(f"Z{_fmt(mz)}")
            prev_z = mz
        if mb is not None and mb != prev_b:
            tokens.append(f"B{_fmt(mb, 2)}")
            prev_b = mb

        # Extrusion
        if is_extrude and prev_x is not None and prev_y is not None:
            # Distance from previous position
            dx = (mx or 0) - (prev_x or 0) if mx != prev_x else 0
            dy = (my or 0) - (prev_y or 0) if my != prev_y else 0
            dz = (mz or 0) - (prev_z or 0) if mz != prev_z else 0
            # Recalc — we just set prev_*, so compute from the actual deltas
            # We need the *old* prev values. Fix: track separately.
            pass  # handled below

        # We need old positions for distance calc — fix the modal tracking.
        # Recalculate using step's own coordinates.
        if is_extrude:
            dist = _move_distance(step, toolpath, config, transform_to_machine)
            if dist > 0:
                de = dist * e_per_mm * step.state.extrusion_multiplier
                e_total += de
                tokens.append(f"E{_fmt(e_total, 4)}")

        # Feedrate
        f = step.state.feedrate
        if f != prev_f:
            tokens.append(f"F{_fmt(f, 0)}")
            prev_f = f

        if len(tokens) > 1:  # more than just the G command
            lines.append(" ".join(tokens))

    # End G-code
    lines.append(config.rendered_end_gcode())
    return "\n".join(lines)


def _move_distance(step: Move, toolpath: list, config: PrinterConfig,
                   transform: bool) -> float:
    """Compute XYZ distance from the previous Move to *step*."""
    idx = None
    for i, s in enumerate(toolpath):
        if s is step:
            idx = i
            break
    if idx is None or idx == 0:
        return 0.0
    # Walk back to find the previous Move
    for j in range(idx - 1, -1, -1):
        prev = toolpath[j]
        if isinstance(prev, Move):
            x0, y0, z0 = prev.x or 0, prev.y or 0, prev.z or 0
            x1, y1, z1 = step.x or 0, step.y or 0, step.z or 0
            if transform and None not in (prev.x, prev.y, prev.z, prev.b):
                x0, y0, z0 = tip_to_pivot((x0, y0, z0), prev.b or 0, config)
            if transform and None not in (step.x, step.y, step.z, step.b):
                x1, y1, z1 = tip_to_pivot((x1, y1, z1), step.b or 0, config)
            return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)
    return 0.0


def _compile_state_change(sc: StateChange, config: PrinterConfig) -> str:
    """Emit G-code for a StateChange."""
    if sc.kind == StateChangeKind.SET_NOZZLE_TEMP:
        return f"M104 S{_fmt(sc.value or 0, 0)}"
    if sc.kind == StateChangeKind.SET_BED_TEMP:
        return f"M140 S{_fmt(sc.value or 0, 0)}"
    if sc.kind == StateChangeKind.SET_FAN:
        pwm = int((sc.value or 0) * 255)
        return f"M106 S{pwm}"
    if sc.kind == StateChangeKind.RETRACT:
        return f"G1 E-{_fmt(sc.value or config.retract_length)} F{_fmt(config.retract_speed, 0)}"
    if sc.kind == StateChangeKind.UNRETRACT:
        return f"G1 E{_fmt(sc.value or config.retract_length)} F{_fmt(config.retract_speed, 0)}"
    if sc.kind == StateChangeKind.COMMENT:
        return f"; {sc.comment or ''}"
    return f"; StateChange {sc.kind.name}"


# ---------------------------------------------------------------------------
# Dry run
# ---------------------------------------------------------------------------

def dry_run(
    toolpath: list,
    config: PrinterConfig | None = None,
) -> GcodeStats:
    """Analyse a toolpath without generating G-code.

    Returns
    -------
    GcodeStats with summary metrics.
    """
    if config is None:
        config = default_config()

    e_per_mm = _extrusion_per_mm(config)
    total_dist = 0.0
    total_e = 0.0
    max_f = 0.0
    b_vals: List[float] = []
    move_count = 0
    prev_move: Optional[Move] = None

    for step in toolpath:
        if not isinstance(step, Move):
            continue
        move_count += 1
        if step.b is not None:
            b_vals.append(step.b)
        if step.state.feedrate > max_f:
            max_f = step.state.feedrate
        if prev_move is not None:
            d = prev_move.distance_to(step)
            total_dist += d
            if step.state.extrusion_mode == ExtrusionMode.ON:
                total_e += d * e_per_mm * step.state.extrusion_multiplier
        prev_move = step

    # Rough layer count: count distinct z values
    zs = {m.z for m in toolpath if isinstance(m, Move) and m.z is not None}

    return GcodeStats(
        total_path_length_mm=total_dist,
        total_extrusion_mm=total_e,
        b_min_deg=min(b_vals) if b_vals else 0.0,
        b_max_deg=max(b_vals) if b_vals else 0.0,
        max_feedrate_xy=max_f,
        layer_count=len(zs),
        move_count=move_count,
        line_count=0,  # filled if compile_gcode is called
    )
