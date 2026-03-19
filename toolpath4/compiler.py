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

    # Previous machine-coordinate positions (for modal optimisation and
    # distance computation).  Initialised to None → first move always emitted.
    prev_mx: Optional[float] = None
    prev_my: Optional[float] = None
    prev_mz: Optional[float] = None
    prev_mb: Optional[float] = None
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

        # Compute XYZ distance from previous position BEFORE updating
        # modal state — needed for extrusion calculation.
        is_extrude = step.state.extrusion_mode == ExtrusionMode.ON
        dist = 0.0
        if is_extrude and prev_mx is not None:
            dx = (mx or 0) - prev_mx
            dy = (my or 0) - prev_my
            dz = (mz or 0) - prev_mz
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        # Build G-code tokens (modal — only emit changed values)
        cmd = "G1" if is_extrude or step.state.feedrate != config.travel_speed else "G0"
        tokens: List[str] = [cmd]

        if mx is not None and mx != prev_mx:
            tokens.append(f"X{_fmt(mx)}")
        if my is not None and my != prev_my:
            tokens.append(f"Y{_fmt(my)}")
        if mz is not None and mz != prev_mz:
            tokens.append(f"Z{_fmt(mz)}")
        if mb is not None and mb != prev_mb:
            tokens.append(f"B{_fmt(mb, 2)}")

        # Extrusion
        if dist > 0:
            de = dist * e_per_mm * step.state.extrusion_multiplier
            e_total += de
            tokens.append(f"E{_fmt(e_total, 4)}")

        # Feedrate (modal)
        f = step.state.feedrate
        if f != prev_f:
            tokens.append(f"F{_fmt(f, 0)}")
            prev_f = f

        # Update previous positions AFTER building the line
        if mx is not None:
            prev_mx = mx
        if my is not None:
            prev_my = my
        if mz is not None:
            prev_mz = mz
        if mb is not None:
            prev_mb = mb

        if len(tokens) > 1:  # more than just the G command
            lines.append(" ".join(tokens))

    # End G-code
    lines.append(config.rendered_end_gcode())
    return "\n".join(lines)


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
