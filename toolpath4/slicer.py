"""
toolpath4.slicer — Mesh slicing, overhang analysis, B-angle planning,
and explicit toolpath generation.

Pipeline
--------
1. load_mesh          — import & validate an STL via trimesh.
2. slice_mesh_along_z — horizontal cross-sections → 2-D polygons.
3. perimeter_generation — repeated negative buffer → perimeter polylines.
4. infill_generation    — grid lines clipped to inner polygon.
5. overhang_analysis    — per-layer overhang score from face normals.
6. B-angle planning     — threshold-based angle assignment with transitions.
7. Toolpath emission    — perimeters + infill → Move / StateChange stream.

Every decision is explicit geometry — no hidden "magic".
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import trimesh
except ImportError:
    trimesh = None  # type: ignore[assignment]

try:
    from shapely.geometry import (
        LinearRing,
        LineString,
        MultiLineString,
        MultiPolygon,
        Polygon,
    )
    from shapely.ops import unary_union
    from shapely import affinity
except ImportError:
    raise ImportError("shapely is required:  pip install shapely")

from toolpath4.config import PrinterConfig, default_config
from toolpath4.state import (
    ExtrusionMode,
    Move,
    State,
    StateChange,
    StepList,
)


# ---------------------------------------------------------------------------
# 1) Mesh loading
# ---------------------------------------------------------------------------

def load_mesh(path: str, *, attempt_repair: bool = True) -> "trimesh.Trimesh":
    """Load an STL file and validate it.

    Parameters
    ----------
    path : filesystem path to ``.stl``.
    attempt_repair : try to fill holes and remove degenerate faces.

    Returns
    -------
    trimesh.Trimesh

    Raises
    ------
    ValueError
        If the mesh is not watertight after repair.
    RuntimeError
        If trimesh is not installed.
    """
    if trimesh is None:
        raise RuntimeError("trimesh is required:  pip install trimesh")

    mesh = trimesh.load(path)
    if isinstance(mesh, trimesh.Scene):
        # Flatten scene to single mesh
        mesh = mesh.dump(concatenate=True)

    if attempt_repair:
        trimesh.repair.fill_holes(mesh)
        trimesh.repair.fix_normals(mesh)
        # Remove degenerate/duplicate faces using the current trimesh API
        mask = mesh.nondegenerate_faces()
        mesh.update_faces(mask)
        mesh.merge_vertices()

    if not mesh.is_watertight:
        print(f"WARNING: mesh at {path} is not watertight after repair. "
              "Slicing may produce incomplete layers.")

    print(f"Mesh loaded: {len(mesh.faces)} faces, "
          f"bounds {mesh.bounds[0]} → {mesh.bounds[1]}, "
          f"watertight={mesh.is_watertight}")
    return mesh


# ---------------------------------------------------------------------------
# 2) Planar slicing
# ---------------------------------------------------------------------------

def _make_slice_layer(z, polygons, index):
    """Simple dict-based layer representation."""
    return {"z": z, "polygons": polygons, "index": index}


def slice_mesh_along_z(
    mesh: "trimesh.Trimesh",
    layer_height: float = 0.2,
    z_min: Optional[float] = None,
    z_max: Optional[float] = None,
) -> List[dict]:
    """Slice *mesh* into horizontal layers.

    Uses ``trimesh.section_multiplane`` with plane_normal = [0, 0, 1].

    Parameters
    ----------
    mesh : loaded trimesh.
    layer_height : vertical spacing in mm.
    z_min, z_max : optional overrides for the slicing range.

    Returns
    -------
    List of dicts ``{"z": float, "polygons": [shapely.Polygon], "index": int}``.
    """
    if z_min is None:
        z_min = float(mesh.bounds[0][2])
    if z_max is None:
        z_max = float(mesh.bounds[1][2])

    # Heights from first layer (half-height offset) to top
    heights = np.arange(z_min + layer_height / 2.0, z_max, layer_height)
    if len(heights) == 0:
        return []

    plane_origin = np.array([0.0, 0.0, z_min])
    plane_normal = np.array([0.0, 0.0, 1.0])
    relative_heights = heights - z_min

    sections = mesh.section_multiplane(
        plane_origin=plane_origin,
        plane_normal=plane_normal,
        heights=relative_heights.tolist(),
    )

    layers: List[dict] = []
    for idx, (z_val, section) in enumerate(zip(heights, sections)):
        if section is None:
            continue
        polys = _section_to_polygons(section)
        if polys:
            layers.append(_make_slice_layer(float(z_val), polys, idx))

    print(f"Sliced {len(layers)} layers from z={z_min:.2f} to z={z_max:.2f}")
    return layers


def _section_to_polygons(section) -> List[Polygon]:
    """Convert a trimesh Path2D/Path3D section to shapely Polygons."""
    polygons: List[Polygon] = []
    try:
        # section is a trimesh.path.Path3D; convert to 2D
        path2d, _ = section.to_planar()
    except Exception:
        path2d = section  # may already be 2D

    try:
        # Try using polygons_closed which returns (n,m,2) arrays
        for poly_verts in path2d.polygons_closed:
            if poly_verts is not None:
                try:
                    p = Polygon(poly_verts)
                    if p.is_valid and p.area > 1e-6:
                        polygons.append(p)
                except Exception:
                    pass
    except (AttributeError, TypeError):
        pass

    if not polygons:
        # Fallback: build polygons from discrete paths
        try:
            for entity_idx, entity in enumerate(path2d.entities):
                pts = path2d.vertices[entity.points]
                if len(pts) >= 3:
                    try:
                        p = Polygon(pts[:, :2])
                        if p.is_valid and p.area > 1e-6:
                            polygons.append(p)
                    except Exception:
                        pass
        except Exception:
            pass

    return polygons


# ---------------------------------------------------------------------------
# 3) Perimeter generation
# ---------------------------------------------------------------------------

def perimeter_generation(
    polygons: List[Polygon],
    perimeter_count: int = 2,
    line_width: float = 0.45,
) -> List[List[Tuple[float, float]]]:
    """Generate *perimeter_count* inset perimeters from *polygons*.

    Each perimeter is a list of (x, y) coordinate tuples forming a closed
    polyline.

    Algorithm: union all polygons, then repeatedly buffer inward by
    ``-line_width``.
    """
    if not polygons:
        return []
    merged = unary_union(polygons)
    if merged.is_empty:
        return []

    perimeters: List[List[Tuple[float, float]]] = []
    current = merged
    for i in range(perimeter_count):
        offset = -(i + 0.5) * line_width
        inset = current.buffer(offset, join_style="mitre", mitre_limit=2.0)
        if inset.is_empty:
            break
        for ring in _polygon_exterior_rings(inset):
            perimeters.append(list(ring.coords))
    return perimeters


def _polygon_exterior_rings(geom) -> list:
    """Extract exterior LinearRings from a Polygon or MultiPolygon."""
    rings = []
    if isinstance(geom, Polygon):
        if not geom.is_empty:
            rings.append(geom.exterior)
    elif isinstance(geom, MultiPolygon):
        for p in geom.geoms:
            if not p.is_empty:
                rings.append(p.exterior)
    return rings


# ---------------------------------------------------------------------------
# 4) Infill generation
# ---------------------------------------------------------------------------

def infill_generation(
    inner_polygon: Polygon | MultiPolygon,
    density: float = 0.20,
    line_width: float = 0.45,
    angle_deg: float = 45.0,
) -> List[List[Tuple[float, float]]]:
    """Generate grid infill lines clipped to *inner_polygon*.

    Parameters
    ----------
    inner_polygon : the region to fill.
    density : 0–1 infill fraction.
    line_width : extrusion width.
    angle_deg : grid rotation angle.

    Returns
    -------
    List of polylines, each a list of (x, y) tuples.
    """
    if inner_polygon is None or inner_polygon.is_empty:
        return []
    if density <= 0:
        return []

    spacing = line_width / density if density < 1.0 else line_width
    bounds = inner_polygon.bounds  # (minx, miny, maxx, maxy)
    minx, miny, maxx, maxy = bounds
    # Expand for rotation margin
    diag = math.sqrt((maxx - minx) ** 2 + (maxy - miny) ** 2)
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2

    lines: List[LineString] = []
    # Generate axis-aligned lines then rotate
    y = cy - diag / 2
    while y <= cy + diag / 2:
        l = LineString([(cx - diag / 2, y), (cx + diag / 2, y)])
        lines.append(l)
        y += spacing

    # Rotate lines
    rotated = [affinity.rotate(l, angle_deg, origin=(cx, cy)) for l in lines]

    # Clip to polygon
    result: List[List[Tuple[float, float]]] = []
    for line in rotated:
        clipped = inner_polygon.intersection(line)
        if clipped.is_empty:
            continue
        if isinstance(clipped, LineString):
            result.append(list(clipped.coords))
        elif isinstance(clipped, MultiLineString):
            for seg in clipped.geoms:
                result.append(list(seg.coords))
    return result


# ---------------------------------------------------------------------------
# 5) Overhang analysis
# ---------------------------------------------------------------------------

def overhang_analysis(
    mesh: "trimesh.Trimesh",
    z_values: Sequence[float],
    threshold_deg: float = 45.0,
    band_height: Optional[float] = None,
) -> Dict[float, float]:
    """Compute a per-layer overhang score.

    For each z value, find faces whose centroid is within ±band_height/2 and
    whose normal makes an angle > *threshold_deg* from vertical (+Z).

    The score is the fraction of such faces relative to all faces in that band.

    Parameters
    ----------
    mesh : trimesh mesh.
    z_values : z heights to analyse.
    threshold_deg : angle from vertical above which a face is "overhang".
    band_height : z band width; defaults to layer height estimate.

    Returns
    -------
    Dict mapping z → overhang score (0–1).
    """
    if band_height is None:
        if len(z_values) >= 2:
            band_height = abs(z_values[1] - z_values[0])
        else:
            band_height = 0.2

    normals = mesh.face_normals
    centroids = mesh.triangles_center
    vertical = np.array([0.0, 0.0, 1.0])
    cos_threshold = math.cos(math.radians(threshold_deg))

    # Dot products with vertical for all faces
    dots = normals @ vertical  # shape (n_faces,)

    scores: Dict[float, float] = {}
    half = band_height / 2.0
    for z in z_values:
        mask_z = (centroids[:, 2] >= z - half) & (centroids[:, 2] < z + half)
        n_band = int(mask_z.sum())
        if n_band == 0:
            scores[z] = 0.0
            continue
        # Overhang: normal tilted far from vertical (dot < cos_threshold)
        overhang = mask_z & (dots < cos_threshold)
        scores[z] = float(overhang.sum()) / n_band
    return scores


# ---------------------------------------------------------------------------
# 6) B-angle planning
# ---------------------------------------------------------------------------

# Candidate angles ordered from most vertical to most tilted
CANDIDATE_ANGLES = [90.0, 45.0, 35.0]

# Overhang score thresholds for angle escalation
OVERHANG_THRESHOLD_1 = 0.15
OVERHANG_THRESHOLD_2 = 0.35


def plan_b_angles_thresholded(
    overhang_scores: Dict[float, float],
    z_values: Sequence[float],
    *,
    candidate_angles: Sequence[float] = CANDIDATE_ANGLES,
    threshold_1: float = OVERHANG_THRESHOLD_1,
    threshold_2: float = OVERHANG_THRESHOLD_2,
) -> Dict[float, float]:
    """Assign B angle per layer using overhang thresholds.

    Policy:
      - Default B = candidate_angles[0]  (90°, nozzle down).
      - If overhang > threshold_1  → candidate_angles[1]  (45°).
      - If overhang > threshold_2  → candidate_angles[2]  (35°).

    Parameters
    ----------
    overhang_scores : z → overhang score mapping.
    z_values : ordered layer z heights.

    Returns
    -------
    Dict z → B angle (degrees).
    """
    angles: Dict[float, float] = {}
    a0, a1, a2 = (list(candidate_angles) + [35.0, 35.0])[:3]
    for z in z_values:
        score = overhang_scores.get(z, 0.0)
        if score > threshold_2:
            angles[z] = a2
        elif score > threshold_1:
            angles[z] = a1
        else:
            angles[z] = a0
    return angles


def apply_z_range_overrides(
    angles: Dict[float, float],
    overrides: List[Tuple[float, float, float]],
) -> Dict[float, float]:
    """Apply user-specified (z_min, z_max, b_deg) overrides.

    Any layer whose z falls in [z_min, z_max) gets its angle replaced.
    """
    for z in angles:
        for z_lo, z_hi, b in overrides:
            if z_lo <= z < z_hi:
                angles[z] = b
    return angles


# --- Transition strategies ---

def transition_linear(
    angles: Dict[float, float],
    z_values: Sequence[float],
    n_transition_layers: int = 5,
) -> Dict[float, float]:
    """Linearly ramp B between regions of different target angles."""
    return _apply_transition(angles, z_values, n_transition_layers, "linear")


def transition_smooth(
    angles: Dict[float, float],
    z_values: Sequence[float],
    n_transition_layers: int = 5,
) -> Dict[float, float]:
    """S-curve ramp B between regions of different target angles."""
    return _apply_transition(angles, z_values, n_transition_layers, "smooth")


def transition_adaptive(
    angles: Dict[float, float],
    overhang_scores: Dict[float, float],
    z_values: Sequence[float],
    hysteresis: float = 0.05,
) -> Dict[float, float]:
    """Choose angle per layer with hysteresis to avoid rapid oscillation."""
    result: Dict[float, float] = {}
    prev_angle = angles.get(z_values[0], 90.0) if z_values else 90.0
    for z in z_values:
        target = angles.get(z, 90.0)
        score = overhang_scores.get(z, 0.0)
        # Only switch if the overhang score moved by more than hysteresis
        if abs(target - prev_angle) > 1e-3:
            # Check if score truly crossed threshold with margin
            if score > OVERHANG_THRESHOLD_2 + hysteresis:
                result[z] = CANDIDATE_ANGLES[2] if len(CANDIDATE_ANGLES) > 2 else target
            elif score > OVERHANG_THRESHOLD_1 + hysteresis:
                result[z] = CANDIDATE_ANGLES[1] if len(CANDIDATE_ANGLES) > 1 else target
            elif score < OVERHANG_THRESHOLD_1 - hysteresis:
                result[z] = CANDIDATE_ANGLES[0]
            else:
                result[z] = prev_angle  # hold
        else:
            result[z] = target
        prev_angle = result[z]
    return result


def _apply_transition(
    angles: Dict[float, float],
    z_values: Sequence[float],
    n_layers: int,
    kind: str,
) -> Dict[float, float]:
    """Apply linear or smooth transition between adjacent angle changes."""
    z_list = list(z_values)
    raw = [angles.get(z, 90.0) for z in z_list]
    smoothed = list(raw)

    # Find transition boundaries
    i = 0
    while i < len(raw):
        if i > 0 and abs(raw[i] - raw[i - 1]) > 1e-3:
            # Transition zone: ramp over n_layers
            start_i = max(0, i - n_layers // 2)
            end_i = min(len(raw), i + n_layers // 2)
            a_from = raw[start_i]
            a_to = raw[min(end_i, len(raw) - 1)]
            for j in range(start_i, end_i):
                t = (j - start_i) / max(end_i - start_i - 1, 1)
                if kind == "smooth":
                    t = 3 * t * t - 2 * t * t * t  # Hermite smoothstep
                smoothed[j] = a_from + (a_to - a_from) * t
        i += 1

    return {z: a for z, a in zip(z_list, smoothed)}


# ---------------------------------------------------------------------------
# 7) Toolpath emission
# ---------------------------------------------------------------------------

class Slicer:
    """Complete slicing pipeline: mesh → list of Steps.

    Usage::

        slicer = Slicer(config=my_config)
        steps = slicer.process("model.stl")
    """

    def __init__(self, config: PrinterConfig | None = None):
        self.config = config or default_config()
        self.mesh = None
        self.layers: List[dict] = []
        self.overhang_scores: Dict[float, float] = {}
        self.b_angles: Dict[float, float] = {}

    def process(
        self,
        stl_path: str,
        *,
        z_overrides: Optional[List[Tuple[float, float, float]]] = None,
        transition: str = "smooth",
    ) -> StepList:
        """Run the full pipeline and return a StepList.

        Parameters
        ----------
        stl_path : path to STL file.
        z_overrides : list of (z_min, z_max, b_deg) angle overrides.
        transition : ``"linear"``, ``"smooth"``, or ``"adaptive"``.

        Returns
        -------
        StepList of Move and StateChange objects.
        """
        cfg = self.config

        # 1) Load
        self.mesh = load_mesh(stl_path)

        # 2) Slice
        self.layers = slice_mesh_along_z(self.mesh, cfg.layer_height)
        if not self.layers:
            print("WARNING: no layers produced.")
            return StepList()

        z_values = [l["z"] for l in self.layers]

        # 5) Overhang analysis
        self.overhang_scores = overhang_analysis(self.mesh, z_values)

        # 6) B-angle planning
        self.b_angles = plan_b_angles_thresholded(self.overhang_scores, z_values)
        if z_overrides:
            self.b_angles = apply_z_range_overrides(self.b_angles, z_overrides)

        # Apply transition
        if transition == "linear":
            self.b_angles = transition_linear(self.b_angles, z_values)
        elif transition == "smooth":
            self.b_angles = transition_smooth(self.b_angles, z_values)
        elif transition == "adaptive":
            self.b_angles = transition_adaptive(
                self.b_angles, self.overhang_scores, z_values
            )

        # 7) Emit toolpath
        steps = StepList()
        steps.append(StateChange.set_nozzle_temp(cfg.nozzle_temp))
        steps.append(StateChange.set_bed_temp(cfg.bed_temp))
        steps.append(StateChange.set_fan(0.0))

        prev_b = 0.0
        for layer in self.layers:
            z = layer["z"]
            b_deg = self.b_angles.get(z, 90.0)
            polys = layer["polygons"]

            steps.append(StateChange.comment(
                f"Layer {layer['index']}  z={z:.2f}  B={b_deg:.1f}°"
            ))

            # Turn fan on after first few layers
            if layer["index"] == 3:
                steps.append(StateChange.set_fan(1.0))

            # 3) Perimeters
            perims = perimeter_generation(polys, cfg.perimeter_count, cfg.line_width)
            print_state = State(
                feedrate=cfg.max_feedrate_xyz * 0.6,
                extrusion_mode=ExtrusionMode.ON,
                b_angle_deg=b_deg,
            )
            travel_state = State(
                feedrate=cfg.travel_speed,
                extrusion_mode=ExtrusionMode.OFF,
                b_angle_deg=b_deg,
            )

            for perim in perims:
                if not perim:
                    continue
                # Travel to start of perimeter
                sx, sy = perim[0]
                steps.append(StateChange.retract(cfg.retract_length))
                steps.append(Move(x=sx, y=sy, z=z, b=b_deg, state=travel_state))
                steps.append(StateChange.unretract(cfg.retract_length))
                # Print perimeter
                for px, py in perim[1:]:
                    steps.append(Move(x=px, y=py, z=z, b=b_deg, state=print_state))

            # 4) Infill
            if cfg.infill_density > 0 and polys:
                inner_offset = -(cfg.perimeter_count + 0.5) * cfg.line_width
                inner_region = unary_union(polys).buffer(
                    inner_offset, join_style="mitre", mitre_limit=2.0
                )
                infill_angle = 45.0 if layer["index"] % 2 == 0 else 135.0
                infill_lines = infill_generation(
                    inner_region, cfg.infill_density, cfg.line_width, infill_angle
                )
                for line in infill_lines:
                    if len(line) < 2:
                        continue
                    sx, sy = line[0]
                    steps.append(StateChange.retract(cfg.retract_length))
                    steps.append(Move(x=sx, y=sy, z=z, b=b_deg, state=travel_state))
                    steps.append(StateChange.unretract(cfg.retract_length))
                    for px, py in line[1:]:
                        steps.append(Move(x=px, y=py, z=z, b=b_deg, state=print_state))

            prev_b = b_deg

        return steps

    def rotation_schedule_summary(self) -> str:
        """Return a human-readable summary of B angles by z range."""
        if not self.b_angles:
            return "No rotation schedule computed yet."
        lines = ["Z range (mm)       B angle (deg)  Overhang score"]
        lines.append("-" * 52)
        for z in sorted(self.b_angles):
            score = self.overhang_scores.get(z, 0.0)
            lines.append(f"  z={z:8.2f}       B={self.b_angles[z]:6.1f}°     {score:.3f}")
        return "\n".join(lines)
