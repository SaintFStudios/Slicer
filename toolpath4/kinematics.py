"""
toolpath4.kinematics — XYZB1 forward and inverse kinematics.

Co-ordinate spaces
------------------
**World / toolpath** co-ordinates describe the nozzle **tip** position and the
B angle.  This is what the designer / slicer works with.

**Machine** co-ordinates describe the position of the B-axis **pivot** (rotation
centre) — the point that the linear XYZ axes actually drive to.

Relationship (B rotates about the machine +Y axis)::

    R_y(θ) = [[ cos θ,  0,  sin θ],
              [     0,  1,      0],
              [-sin θ,  0,  cos θ]]

At B = 0 the nozzle hangs straight down, so the vector from TIP to PIVOT is::

    v = [0, 0, b_offset_z]          (pointing upward along +Z)

    pivot  = tip  + R_y(B) · v      →  tip_to_pivot
    tip    = pivot - R_y(B) · v      →  pivot_to_tip

All angles are in **degrees** at the public API boundary.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from toolpath4.config import PrinterConfig, default_config


# ---------------------------------------------------------------------------
# Rotation matrix about Y
# ---------------------------------------------------------------------------

def _Ry(theta_rad: float) -> np.ndarray:
    """3×3 rotation matrix about the Y axis (right-hand rule).

    Parameters
    ----------
    theta_rad : angle in **radians**.
    """
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    return np.array([
        [ c, 0.0,  s],
        [0.0, 1.0, 0.0],
        [-s, 0.0,  c],
    ])


# ---------------------------------------------------------------------------
# Public transforms
# ---------------------------------------------------------------------------

def tip_to_pivot(
    tip_xyz: Tuple[float, float, float],
    b_deg: float,
    config: PrinterConfig | None = None,
) -> Tuple[float, float, float]:
    """Convert nozzle-tip position to pivot (machine XYZ) position.

    Parameters
    ----------
    tip_xyz : (x, y, z) of the nozzle tip in mm.
    b_deg   : B-axis angle in degrees.
    config  : printer config (uses default if *None*).

    Returns
    -------
    (x, y, z) of the B-axis rotation centre (pivot) in mm.
    """
    if config is None:
        config = default_config()
    tip = np.asarray(tip_xyz, dtype=float)
    v = np.array([0.0, 0.0, config.b_offset_z])
    R = _Ry(math.radians(b_deg))
    pivot = tip + R @ v
    return (float(pivot[0]), float(pivot[1]), float(pivot[2]))


def pivot_to_tip(
    pivot_xyz: Tuple[float, float, float],
    b_deg: float,
    config: PrinterConfig | None = None,
) -> Tuple[float, float, float]:
    """Convert pivot (machine XYZ) position to nozzle-tip position.

    Parameters
    ----------
    pivot_xyz : (x, y, z) of the B-axis rotation centre in mm.
    b_deg     : B-axis angle in degrees.
    config    : printer config (uses default if *None*).

    Returns
    -------
    (x, y, z) of the nozzle tip in mm.
    """
    if config is None:
        config = default_config()
    pivot = np.asarray(pivot_xyz, dtype=float)
    v = np.array([0.0, 0.0, config.b_offset_z])
    R = _Ry(math.radians(b_deg))
    tip = pivot - R @ v
    return (float(tip[0]), float(tip[1]), float(tip[2]))


# ---------------------------------------------------------------------------
# Batch transform for toolpaths
# ---------------------------------------------------------------------------

def toolpath_tip_to_machine(
    moves: list,
    config: PrinterConfig | None = None,
) -> list:
    """Convert every Move in a toolpath from tip to machine (pivot) co-ords.

    Returns a **new** list; originals are not mutated.

    Only moves with all of x, y, z, b defined are transformed.  Others are
    passed through unchanged.
    """
    from toolpath4.state import Move as MoveType
    if config is None:
        config = default_config()
    out = []
    for step in moves:
        if isinstance(step, MoveType) and None not in (step.x, step.y, step.z, step.b):
            px, py, pz = tip_to_pivot((step.x, step.y, step.z), step.b, config)
            new_move = MoveType(x=px, y=py, z=pz, b=step.b, state=step.state)
            out.append(new_move)
        else:
            out.append(step)
    return out


# ---------------------------------------------------------------------------
# Collision margin check (stub)
# ---------------------------------------------------------------------------

def check_collision_margin(
    tip_xyz: Tuple[float, float, float],
    b_deg: float,
    config: PrinterConfig | None = None,
) -> bool:
    """Return *True* if the toolhead clears the build volume with margin.

    This is a simplified bounding-sphere check using ``toolhead_radius``.
    A full collision model would require the actual toolhead geometry.

    Parameters
    ----------
    tip_xyz : nozzle-tip position.
    b_deg   : B angle.
    config  : printer config.

    Returns
    -------
    True if no collision detected.
    """
    if config is None:
        config = default_config()
    px, py, pz = tip_to_pivot(tip_xyz, b_deg, config)
    r = config.toolhead_radius
    ok = (
        (config.bed_x_min + r) <= px <= (config.bed_x_max - r)
        and (config.bed_y_min + r) <= py <= (config.bed_y_max - r)
        and config.bed_z_min <= pz <= (config.bed_z_max - r)
    )
    return ok
