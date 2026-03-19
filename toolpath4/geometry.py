"""
toolpath4.geometry — XYZB geometry primitives.

Every function returns a list of ``Move`` objects so the output can be
directly appended to a ``StepList``.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from toolpath4.state import ExtrusionMode, Move, State


# ---------------------------------------------------------------------------
# Polyline
# ---------------------------------------------------------------------------

def polyline(
    points: Sequence[Tuple[float, float, float]],
    b: float = 0.0,
    feedrate: float = 1200.0,
    extrusion: ExtrusionMode = ExtrusionMode.ON,
) -> List[Move]:
    """Convert a sequence of (x, y, z) tuples into a list of Moves at
    constant B angle.

    Parameters
    ----------
    points : sequence of (x, y, z)
    b : B-axis angle in degrees.
    feedrate : mm/min.
    extrusion : whether to extrude.
    """
    state = State(feedrate=feedrate, extrusion_mode=extrusion, b_angle_deg=b)
    return [Move(x=p[0], y=p[1], z=p[2], b=b, state=state) for p in points]


# ---------------------------------------------------------------------------
# Circle
# ---------------------------------------------------------------------------

def circle(
    center: Tuple[float, float],
    radius: float,
    z: float,
    b: float = 0.0,
    n: int = 64,
    feedrate: float = 1200.0,
) -> List[Move]:
    """Generate *n* points on a circle in the XY plane at height *z*.

    The circle is closed (first point repeated at end).
    """
    state = State(feedrate=feedrate, extrusion_mode=ExtrusionMode.ON, b_angle_deg=b)
    moves: List[Move] = []
    for i in range(n + 1):
        theta = 2.0 * math.pi * i / n
        x = center[0] + radius * math.cos(theta)
        y = center[1] + radius * math.sin(theta)
        moves.append(Move(x=x, y=y, z=z, b=b, state=state))
    return moves


# ---------------------------------------------------------------------------
# Helix
# ---------------------------------------------------------------------------

def helix(
    center: Tuple[float, float],
    radius: float,
    z_start: float,
    z_end: float,
    turns: float = 5.0,
    b: float = 0.0,
    points_per_turn: int = 64,
    feedrate: float = 1200.0,
) -> List[Move]:
    """Generate a helix about a vertical axis.

    Parameters
    ----------
    center : (x, y) centre of the helix.
    radius : helix radius in mm.
    z_start, z_end : vertical extents.
    turns : number of full revolutions.
    b : constant B angle.
    points_per_turn : resolution.
    """
    n = max(int(turns * points_per_turn), 2)
    state = State(feedrate=feedrate, extrusion_mode=ExtrusionMode.ON, b_angle_deg=b)
    moves: List[Move] = []
    for i in range(n + 1):
        t = i / n
        theta = 2.0 * math.pi * turns * t
        x = center[0] + radius * math.cos(theta)
        y = center[1] + radius * math.sin(theta)
        z = z_start + (z_end - z_start) * t
        moves.append(Move(x=x, y=y, z=z, b=b, state=state))
    return moves


# ---------------------------------------------------------------------------
# Oscillating B along a path
# ---------------------------------------------------------------------------

def oscillating_b(
    start_b: float,
    end_b: float,
    frequency: float,
    phase: float,
    path: List[Move],
) -> List[Move]:
    """Overlay a sinusoidal B oscillation onto an existing path.

    The B value at each point is::

        b(t) = mean + amplitude * sin(2π * frequency * t + phase)

    where *t* is the normalised progress [0, 1] along *path*.

    Parameters
    ----------
    start_b, end_b : B-angle envelope bounds (degrees).
    frequency : oscillation cycles over the full path.
    phase : phase offset in radians.
    path : existing Move list to augment.

    Returns
    -------
    New list of Move objects (originals are not mutated).
    """
    if not path:
        return []
    mean = (start_b + end_b) / 2.0
    amplitude = (end_b - start_b) / 2.0
    n = len(path)
    out: List[Move] = []
    for i, m in enumerate(path):
        t = i / max(n - 1, 1)
        b_val = mean + amplitude * math.sin(2.0 * math.pi * frequency * t + phase)
        new_state = m.state.copy(b_angle_deg=b_val)
        out.append(Move(x=m.x, y=m.y, z=m.z, b=b_val, state=new_state))
    return out


# ---------------------------------------------------------------------------
# Densify
# ---------------------------------------------------------------------------

def densify(
    moves: List[Move],
    max_chord_error: float = 0.1,
) -> List[Move]:
    """Insert intermediate points so that no segment exceeds *max_chord_error*.

    Linear interpolation is used for x, y, z and B.

    Parameters
    ----------
    moves : input move list.
    max_chord_error : maximum allowed chord length in mm.

    Returns
    -------
    Densified list of Move objects.
    """
    if len(moves) < 2:
        return list(moves)
    out: List[Move] = [moves[0]]
    for prev, cur in zip(moves, moves[1:]):
        dist = prev.distance_to(cur)
        if dist <= max_chord_error or dist == 0:
            out.append(cur)
            continue
        n_segs = max(int(math.ceil(dist / max_chord_error)), 1)
        for k in range(1, n_segs + 1):
            t = k / n_segs
            x = _lerp(prev.x, cur.x, t)
            y = _lerp(prev.y, cur.y, t)
            z = _lerp(prev.z, cur.z, t)
            b = _lerp(prev.b, cur.b, t)
            state = cur.state.copy(b_angle_deg=b if b is not None else 0.0)
            out.append(Move(x=x, y=y, z=z, b=b, state=state))
    return out


def _lerp(a: Optional[float], b: Optional[float], t: float) -> Optional[float]:
    if a is None or b is None:
        return b
    return a + (b - a) * t
