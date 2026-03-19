"""
toolpath4.state — Core data model for the point+state stream.

Design philosophy (inspired by FullControl's "steps list"):
  The toolpath is a flat list of *Step* objects.  Each step is either:
    - a **Move**: a target position (x, y, z, b) with associated extrusion
      and motion state, *or*
    - a **StateChange**: a command that modifies machine state (temperature,
      fan speed, extrusion mode, etc.) without motion.

There is no global mutable state.  Every Move carries its own State snapshot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Union


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class ExtrusionMode(Enum):
    """Whether the extruder is actively depositing material."""
    OFF = auto()
    ON = auto()


class StateChangeKind(Enum):
    """The kind of machine-state mutation a StateChange represents."""
    SET_NOZZLE_TEMP = auto()
    SET_BED_TEMP = auto()
    SET_FAN = auto()
    SET_SPEED = auto()
    SET_EXTRUSION_MODE = auto()
    RETRACT = auto()
    UNRETRACT = auto()
    COMMENT = auto()


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

@dataclass
class State:
    """Snapshot of all non-positional machine state attached to a Move.

    Attributes
    ----------
    feedrate : float
        Feedrate in mm/min for this move.
    extrusion_mode : ExtrusionMode
        Whether material is being deposited.
    extrusion_multiplier : float
        Flow-rate multiplier (1.0 = nominal).
    temp_nozzle : Optional[float]
        Current nozzle temperature setpoint (°C), or None if unchanged.
    temp_bed : Optional[float]
        Current bed temperature setpoint (°C), or None if unchanged.
    fan : Optional[float]
        Fan duty cycle 0–1, or None if unchanged.
    b_angle_deg : float
        B-axis angle in degrees (duplicated here for convenience; the Move
        also stores *b* for the positional component).
    """
    feedrate: float = 1200.0
    extrusion_mode: ExtrusionMode = ExtrusionMode.ON
    extrusion_multiplier: float = 1.0
    temp_nozzle: Optional[float] = None
    temp_bed: Optional[float] = None
    fan: Optional[float] = None
    b_angle_deg: float = 0.0

    def copy(self, **overrides) -> State:
        """Return a shallow copy with optional field overrides."""
        import dataclasses
        return dataclasses.replace(self, **overrides)


# ---------------------------------------------------------------------------
# Move
# ---------------------------------------------------------------------------

@dataclass
class Move:
    """A single toolpath point with position and state.

    Co-ordinate convention:
      x, y, z — nozzle **tip** position in mm (world / toolpath space).
      b       — B-axis angle in degrees.

    Any axis set to *None* means "unchanged from previous move".
    """
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    b: Optional[float] = None
    state: State = field(default_factory=State)

    # Convenience read-only properties --------------------------------
    @property
    def feedrate(self) -> float:
        return self.state.feedrate

    @property
    def is_extruding(self) -> bool:
        return self.state.extrusion_mode == ExtrusionMode.ON

    def xyz(self) -> tuple:
        """Return (x, y, z) tuple (Nones preserved)."""
        return (self.x, self.y, self.z)

    def xyzb(self) -> tuple:
        """Return (x, y, z, b) tuple (Nones preserved)."""
        return (self.x, self.y, self.z, self.b)

    def distance_to(self, other: Move) -> float:
        """Euclidean XYZ distance to *other*, treating None as 0."""
        dx = (other.x or 0.0) - (self.x or 0.0)
        dy = (other.y or 0.0) - (self.y or 0.0)
        dz = (other.z or 0.0) - (self.z or 0.0)
        return math.sqrt(dx * dx + dy * dy + dz * dz)


# ---------------------------------------------------------------------------
# StateChange
# ---------------------------------------------------------------------------

@dataclass
class StateChange:
    """A non-motion command that mutates machine state.

    Examples: set nozzle temperature, toggle fan, retract filament.
    """
    kind: StateChangeKind
    value: Optional[float] = None
    comment: Optional[str] = None

    # Convenient constructors -------------------------------------------
    @classmethod
    def set_nozzle_temp(cls, temp: float) -> StateChange:
        return cls(kind=StateChangeKind.SET_NOZZLE_TEMP, value=temp)

    @classmethod
    def set_bed_temp(cls, temp: float) -> StateChange:
        return cls(kind=StateChangeKind.SET_BED_TEMP, value=temp)

    @classmethod
    def set_fan(cls, duty: float) -> StateChange:
        """*duty* is 0–1."""
        return cls(kind=StateChangeKind.SET_FAN, value=duty)

    @classmethod
    def retract(cls, length: float = 0.8) -> StateChange:
        return cls(kind=StateChangeKind.RETRACT, value=length)

    @classmethod
    def unretract(cls, length: float = 0.8) -> StateChange:
        return cls(kind=StateChangeKind.UNRETRACT, value=length)

    @classmethod
    def make_comment(cls, text: str) -> StateChange:
        """Create a COMMENT StateChange.

        Named ``make_comment`` (not ``comment``) to avoid shadowing the
        ``comment`` dataclass field when it defaults to ``None``.
        """
        return cls(kind=StateChangeKind.COMMENT, comment=text)


# ---------------------------------------------------------------------------
# StepList — typed alias with helper methods
# ---------------------------------------------------------------------------

Step = Union[Move, StateChange]


class StepList(list):
    """A list of Step objects with convenience helpers.

    Inherits from list so it is fully compatible with normal list operations.
    """

    def moves(self) -> List[Move]:
        """Return only Move steps."""
        return [s for s in self if isinstance(s, Move)]

    def state_changes(self) -> List[StateChange]:
        """Return only StateChange steps."""
        return [s for s in self if isinstance(s, StateChange)]

    def b_range(self) -> tuple:
        """Return (min_b, max_b) across all Move steps that define B."""
        bs = [m.b for m in self.moves() if m.b is not None]
        if not bs:
            return (0.0, 0.0)
        return (min(bs), max(bs))

    def total_distance(self) -> float:
        """Sum of XYZ Euclidean distances between consecutive Moves."""
        mvs = self.moves()
        dist = 0.0
        for a, b in zip(mvs, mvs[1:]):
            dist += a.distance_to(b)
        return dist

    def append_travel(self, x: float, y: float, z: float, b: float,
                      speed: float, retract_len: float = 0.8) -> None:
        """Convenience: retract → rapid travel → unretract."""
        self.append(StateChange.retract(retract_len))
        self.append(Move(
            x=x, y=y, z=z, b=b,
            state=State(feedrate=speed, extrusion_mode=ExtrusionMode.OFF),
        ))
        self.append(StateChange.unretract(retract_len))
