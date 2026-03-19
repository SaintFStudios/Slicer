"""
toolpath4.slicer -- Mesh slicing and toolpath generation (FullControl-style).

Inspired by FullControl's philosophy: the toolpath IS the structure.
No infill generation -- perimeter walls define the geometry.  Every
toolpath is an explicit stream of points + state, nothing hidden.

Pipeline
--------
1. load_mesh              -- import & validate an STL via trimesh.
2. slice_mesh_along_z     -- horizontal cross-sections -> 2-D polygons.
   slice_mesh_along_dir   -- variable build-direction slicing (tilted planes).
3. perimeter_generation   -- repeated inset offsets -> perimeter polylines.
4. overhang_analysis      -- per-layer overhang score from face normals.
5. B-angle planning       -- threshold-based angle assignment with transitions.
6. Toolpath emission      -- perimeters -> Move / StateChange stream.

FullControl bridge: ``to_fullcontrol(steps)`` converts our Step list to a
FullControl-compatible list of ``fc.Point`` / ``fc.Extruder`` / etc. for
gcode generation and visualisation via ``fc.transform()``.

Variable Slicing Coordinate System
-----------------------------------
A ``BuildZone`` defines a region where all layers share the same build
direction (slicing plane orientation).  The build direction is determined
by the B angle:

  B =  0 deg -> build_dir = [0, 0, 1]  (horizontal, normal 3-axis printing)
  B = 45 deg -> build_dir = [sin45, 0, cos45]  (tilted 45 deg)
  B = 90 deg -> build_dir = [1, 0, 0]  (vertical / sideways)

The slicing plane is always perpendicular to the build direction.
2-D toolpath geometry (perimeters) is generated in the plane's local
coordinate system, then transformed back to 3-D world coordinates
via the ``to_3D`` affine matrix returned by ``trimesh.path.to_planar()``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
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


# ===================================================================
# BuildZone -- variable slicing coordinate system
# ===================================================================

@dataclass
class BuildZone:
    """A region of the print with a specific build-plane orientation.

    Attributes
    ----------
    b_deg : float
        B-axis angle in degrees.  Determines the build (layer-stacking)
        direction via ``build_direction(b_deg)``.
    z_start : float
        Approximate world-Z lower bound for this zone.
    z_end : float
        Approximate world-Z upper bound for this zone.
    label : str
        Optional human-readable name (e.g. "base", "overhang").
    """
    b_deg: float = 0.0
    z_start: float = 0.0
    z_end: float = float("inf")
    label: str = ""

    def __str__(self) -> str:
        return (f"BuildZone(B={self.b_deg:.1f} deg, "
                f"z=[{self.z_start:.1f}, {self.z_end:.1f}], "
                f"'{self.label}')")


def build_direction(b_deg: float) -> np.ndarray:
    """Unit vector for the build (layer-stacking) direction at angle *b_deg*.

    At B = 0 the direction is [0, 0, 1] (straight up).
    Rotation is about the +Y axis (right-hand rule).
    """
    theta = math.radians(b_deg)
    return np.array([math.sin(theta), 0.0, math.cos(theta)])


def plane_local_frame(b_deg: float):
    """Return an orthonormal basis ``(u, v, n)`` for the slicing plane.

    ``n`` = build direction (normal to the slicing plane).
    ``u`` = in-plane "X" (lies in the world XZ plane).
    ``v`` = in-plane "Y" (always world +Y since we rotate about Y).
    """
    theta = math.radians(b_deg)
    n = np.array([math.sin(theta), 0.0, math.cos(theta)])
    u = np.array([math.cos(theta), 0.0, -math.sin(theta)])
    v = np.array([0.0, 1.0, 0.0])
    return u, v, n


# ===================================================================
# 1) Mesh loading
# ===================================================================

def load_mesh(path: str, *, attempt_repair: bool = True) -> "trimesh.Trimesh":
    """Load an STL file and validate it.

    Parameters
    ----------
    path : filesystem path to ``.stl``.
    attempt_repair : try to fill holes and remove degenerate faces.

    Returns
    -------
    trimesh.Trimesh
    """
    if trimesh is None:
        raise RuntimeError("trimesh is required:  pip install trimesh")

    mesh = trimesh.load(path)
    if isinstance(mesh, trimesh.Scene):
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
          f"bounds {mesh.bounds[0]} -> {mesh.bounds[1]}, "
          f"watertight={mesh.is_watertight}")
    return mesh


# ===================================================================
# 2a) Horizontal slicing (B = 0, classic mode)
# ===================================================================

def _make_slice_layer(z, polygons, index, *, b_deg=0.0, to_3D=None):
    """Dict-based layer representation."""
    d = {"z": z, "polygons": polygons, "index": index,
         "b_deg": b_deg, "to_3D": to_3D}
    return d


def slice_mesh_along_z(
    mesh: "trimesh.Trimesh",
    layer_height: float = 0.2,
    z_min: Optional[float] = None,
    z_max: Optional[float] = None,
) -> List[dict]:
    """Slice *mesh* into horizontal layers (B = 0).

    Uses ``trimesh.section_multiplane`` with plane_normal = [0, 0, 1].
    """
    if z_min is None:
        z_min = float(mesh.bounds[0][2])
    if z_max is None:
        z_max = float(mesh.bounds[1][2])

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
            layers.append(_make_slice_layer(float(z_val), polys, idx, b_deg=0.0))

    print(f"Sliced {len(layers)} layers from z={z_min:.2f} to z={z_max:.2f}")
    return layers


# ===================================================================
# 2b) Variable-direction slicing (arbitrary B angle)
# ===================================================================

def slice_mesh_along_direction(
    mesh: "trimesh.Trimesh",
    b_deg: float,
    layer_height: float = 0.2,
    z_start: Optional[float] = None,
    z_end: Optional[float] = None,
) -> List[dict]:
    """Slice *mesh* with planes perpendicular to the build direction at *b_deg*.

    The slicing plane normal is ``build_direction(b_deg)``.
    For each section, the 3-D path is projected into the plane's local 2-D
    coordinate system.  The ``to_3D`` transform matrix is stored in each
    layer dict so toolpath points can be mapped back to world coordinates.

    Parameters
    ----------
    mesh : loaded trimesh.
    b_deg : B-axis angle (degrees).  0 = horizontal, 90 = vertical.
    layer_height : spacing between layers measured along the build direction.
    z_start, z_end : optional world-Z bounds to filter layers.

    Returns
    -------
    List of layer dicts.  Each dict contains:
      ``z``        -- approximate world-Z of the layer centre.
      ``polygons`` -- list of shapely Polygons in the plane's 2-D frame.
      ``to_3D``    -- 4x4 affine matrix mapping (x2d, y2d, 0) -> world XYZ.
      ``b_deg``    -- the B angle for this zone.
      ``index``    -- sequential layer index.
    """
    n = build_direction(b_deg)

    # Project all vertices onto the build direction to find extent
    projections = mesh.vertices @ n
    p_min, p_max = float(projections.min()), float(projections.max())

    # Reference origin: centroid of mesh projected back to p_min along n
    centroid_xy = mesh.centroid - n * (mesh.centroid @ n)
    ref_origin = centroid_xy + n * p_min

    # Layer distances along n, relative to ref_origin
    rel_heights = np.arange(layer_height / 2.0, p_max - p_min, layer_height)
    if len(rel_heights) == 0:
        return []

    sections = mesh.section_multiplane(
        plane_origin=ref_origin,
        plane_normal=n,
        heights=rel_heights.tolist(),
    )

    layers: List[dict] = []
    for idx, (rel_h, section) in enumerate(zip(rel_heights, sections)):
        if section is None:
            continue

        # World position of this layer's centre
        layer_centre = ref_origin + n * rel_h
        world_z = float(layer_centre[2])

        # Filter by z_start / z_end if provided
        if z_start is not None and world_z < z_start - layer_height:
            continue
        if z_end is not None and world_z > z_end + layer_height:
            continue

        polys, to_3D = _section_to_polygons_with_transform(section)
        if polys:
            layers.append(_make_slice_layer(
                world_z, polys, idx, b_deg=b_deg, to_3D=to_3D,
            ))

    print(f"Sliced {len(layers)} layers along B={b_deg:.1f} deg "
          f"(z~{layers[0]['z']:.1f}-{layers[-1]['z']:.1f})" if layers else
          f"Sliced 0 layers along B={b_deg:.1f} deg")
    return layers


# ===================================================================
# Section -> polygon helpers
# ===================================================================

def _section_to_polygons(section) -> List[Polygon]:
    """Convert a trimesh Path3D section to shapely Polygons (2-D only)."""
    polys, _ = _section_to_polygons_with_transform(section)
    return polys


def _section_to_polygons_with_transform(section):
    """Return ``(polygons, to_3D_matrix)`` for a section.

    ``to_3D`` is the 4x4 affine that maps ``[x2d, y2d, 0, 1]`` back
    to world coordinates.
    """
    to_3D = np.eye(4)
    try:
        path2d, to_3D = section.to_planar()
    except Exception:
        path2d = section

    polygons = _extract_polygons_from_path2d(path2d)
    return polygons, to_3D


def _extract_polygons_from_path2d(path2d) -> List[Polygon]:
    """Extract shapely Polygons from a trimesh Path2D.

    Tries ``polygons_full`` first (standard trimesh API returning Shapely
    Polygons with holes resolved), then falls back to ``polygons_closed``
    and entity-based reconstruction.
    """
    polygons: List[Polygon] = []

    # Method 1: polygons_full (returns Shapely Polygons directly)
    try:
        for p in path2d.polygons_full:
            if p is not None and p.is_valid and p.area > 1e-6:
                polygons.append(p)
        if polygons:
            return polygons
    except (AttributeError, TypeError, Exception):
        pass

    # Method 2: polygons_closed (returns vertex arrays)
    try:
        for poly_verts in path2d.polygons_closed:
            if poly_verts is not None:
                try:
                    p = Polygon(poly_verts)
                    if p.is_valid and p.area > 1e-6:
                        polygons.append(p)
                except Exception:
                    pass
        if polygons:
            return polygons
    except (AttributeError, TypeError):
        pass

    # Method 3: entity-based fallback
    try:
        for entity in path2d.entities:
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


# ===================================================================
# 2-D <-> 3-D coordinate helpers
# ===================================================================

def _transform_2d_to_3d(x2d: float, y2d: float, to_3D: np.ndarray):
    """Map a 2-D point in a section plane back to world (x, y, z).

    Parameters
    ----------
    x2d, y2d : coordinates in the section's local 2-D frame.
    to_3D : 4x4 affine from ``section.to_planar()``.

    Returns
    -------
    (world_x, world_y, world_z)
    """
    pt = to_3D @ np.array([x2d, y2d, 0.0, 1.0])
    return float(pt[0]), float(pt[1]), float(pt[2])


# ===================================================================
# 3) Perimeter generation (FullControl style -- walls ARE the structure)
# ===================================================================

def perimeter_generation(
    polygons: List[Polygon],
    perimeter_count: int = 2,
    line_width: float = 0.45,
) -> List[List[Tuple[float, float]]]:
    """Generate *perimeter_count* inset perimeters from *polygons*.

    FullControl philosophy: the perimeter walls define the part geometry.
    No infill -- just walls.  Use more perimeters for stronger parts.

    Each perimeter is a list of (x, y) coordinate tuples forming a closed
    polyline.
    """
    if not polygons:
        return []
    merged = unary_union(polygons)
    if merged.is_empty:
        return []

    perimeters: List[List[Tuple[float, float]]] = []
    for i in range(perimeter_count):
        offset = -(i + 0.5) * line_width
        inset = merged.buffer(offset, join_style="mitre", mitre_limit=2.0)
        if inset.is_empty:
            break
        for ring in _polygon_rings(inset):
            perimeters.append(list(ring.coords))
    return perimeters


def _polygon_rings(geom) -> list:
    """Extract all exterior and interior rings from a geometry."""
    rings = []
    if isinstance(geom, Polygon):
        if not geom.is_empty:
            rings.append(geom.exterior)
            for interior in geom.interiors:
                rings.append(interior)
    elif isinstance(geom, MultiPolygon):
        for p in geom.geoms:
            if not p.is_empty:
                rings.append(p.exterior)
                for interior in p.interiors:
                    rings.append(interior)
    return rings


# ===================================================================
# 4) Overhang analysis
# ===================================================================

def overhang_analysis(
    mesh: "trimesh.Trimesh",
    z_values: Sequence[float],
    threshold_deg: float = 45.0,
    band_height: Optional[float] = None,
) -> Dict[float, float]:
    """Compute a per-layer overhang score (fraction of overhang faces)."""
    if band_height is None:
        if len(z_values) >= 2:
            band_height = abs(z_values[1] - z_values[0])
        else:
            band_height = 0.2

    normals = mesh.face_normals
    centroids = mesh.triangles_center
    vertical = np.array([0.0, 0.0, 1.0])
    cos_threshold = math.cos(math.radians(threshold_deg))
    dots = normals @ vertical

    scores: Dict[float, float] = {}
    half = band_height / 2.0
    for z in z_values:
        mask_z = (centroids[:, 2] >= z - half) & (centroids[:, 2] < z + half)
        n_band = int(mask_z.sum())
        if n_band == 0:
            scores[z] = 0.0
            continue
        overhang = mask_z & (dots < cos_threshold)
        scores[z] = float(overhang.sum()) / n_band
    return scores


# ===================================================================
# 5) B-angle planning
# ===================================================================

CANDIDATE_ANGLES = [90.0, 45.0, 35.0]
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
    """Assign B angle per layer using overhang thresholds."""
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
    for z in angles:
        for z_lo, z_hi, b in overrides:
            if z_lo <= z < z_hi:
                angles[z] = b
    return angles


def transition_linear(angles, z_values, n_transition_layers=5):
    return _apply_transition(angles, z_values, n_transition_layers, "linear")

def transition_smooth(angles, z_values, n_transition_layers=5):
    return _apply_transition(angles, z_values, n_transition_layers, "smooth")

def transition_adaptive(angles, overhang_scores, z_values, hysteresis=0.05):
    result: Dict[float, float] = {}
    prev_angle = angles.get(z_values[0], 90.0) if z_values else 90.0
    for z in z_values:
        target = angles.get(z, 90.0)
        score = overhang_scores.get(z, 0.0)
        if abs(target - prev_angle) > 1e-3:
            if score > OVERHANG_THRESHOLD_2 + hysteresis:
                result[z] = CANDIDATE_ANGLES[2]
            elif score > OVERHANG_THRESHOLD_1 + hysteresis:
                result[z] = CANDIDATE_ANGLES[1]
            elif score < OVERHANG_THRESHOLD_1 - hysteresis:
                result[z] = CANDIDATE_ANGLES[0]
            else:
                result[z] = prev_angle
        else:
            result[z] = target
        prev_angle = result[z]
    return result


def _apply_transition(angles, z_values, n_layers, kind):
    z_list = list(z_values)
    raw = [angles.get(z, 90.0) for z in z_list]
    smoothed = list(raw)
    i = 0
    while i < len(raw):
        if i > 0 and abs(raw[i] - raw[i - 1]) > 1e-3:
            start_i = max(0, i - n_layers // 2)
            end_i = min(len(raw), i + n_layers // 2)
            a_from = raw[start_i]
            a_to = raw[min(end_i, len(raw) - 1)]
            for j in range(start_i, end_i):
                t = (j - start_i) / max(end_i - start_i - 1, 1)
                if kind == "smooth":
                    t = 3 * t * t - 2 * t * t * t
                smoothed[j] = a_from + (a_to - a_from) * t
        i += 1
    return {z: a for z, a in zip(z_list, smoothed)}


# ===================================================================
# 6) Toolpath emission -- layer -> Moves (perimeters only, no infill)
# ===================================================================

def _emit_layer_toolpath(
    layer: dict,
    cfg: PrinterConfig,
    steps: StepList,
) -> None:
    """Append Move / StateChange steps for one layer to *steps*.

    FullControl style: only perimeter walls are emitted.  No infill.
    The walls ARE the structure.

    Handles both horizontal layers (``to_3D is None``) and tilted layers
    (``to_3D`` is a 4x4 affine from the section plane).
    """
    z = layer["z"]
    b_deg = layer.get("b_deg", 0.0)
    polys = layer["polygons"]
    to_3D = layer.get("to_3D")  # None for classic horizontal layers

    def _world(px, py):
        """Map 2-D toolpath point -> 3-D world coordinates."""
        if to_3D is not None:
            return _transform_2d_to_3d(px, py, to_3D)
        return (px, py, z)

    steps.append(StateChange.make_comment(
        f"Layer {layer['index']}  z~{z:.2f}  B={b_deg:.1f}"
    ))

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

    # --- Perimeters (walls = the structure, FullControl style) ---
    perims = perimeter_generation(polys, cfg.perimeter_count, cfg.line_width)
    for perim in perims:
        if not perim:
            continue
        sx, sy = perim[0]
        wx, wy, wz = _world(sx, sy)
        # Travel to start of perimeter (retract -> move -> unretract)
        steps.append(StateChange.retract(cfg.retract_length))
        steps.append(Move(x=wx, y=wy, z=wz, b=b_deg, state=travel_state))
        steps.append(StateChange.unretract(cfg.retract_length))
        # Extrude along perimeter
        for px, py in perim[1:]:
            wx, wy, wz = _world(px, py)
            steps.append(Move(x=wx, y=wy, z=wz, b=b_deg, state=print_state))


# ===================================================================
# FullControl bridge -- convert our steps to fc-compatible list
# ===================================================================

def to_fullcontrol(steps: list, cfg: PrinterConfig | None = None):
    """Convert a toolpath4 StepList to a FullControl-compatible list.

    Returns a list of ``fc.Point``, ``fc.Extruder``, ``fc.Hotend``,
    ``fc.Buildplate``, ``fc.Fan``, ``fc.ManualGcode``, etc.

    Requires ``fullcontrol`` to be installed.

    Parameters
    ----------
    steps : our StepList (Move + StateChange objects).
    cfg : printer config for initial setup.

    Returns
    -------
    list of FullControl objects ready for ``fc.transform()``.
    """
    try:
        import fullcontrol as fc
    except ImportError:
        raise ImportError(
            "fullcontrol is required for this feature.\n"
            "Install with:  pip install git+https://github.com/FullControlXYZ/fullcontrol"
        )

    if cfg is None:
        cfg = default_config()

    fc_steps = []

    # Initial setup
    fc_steps.append(fc.Hotend(temp=cfg.nozzle_temp, wait=True))
    fc_steps.append(fc.Buildplate(temp=cfg.bed_temp))
    fc_steps.append(fc.Fan(speed_percent=0))
    fc_steps.append(fc.ExtrusionGeometry(width=cfg.line_width, height=cfg.layer_height))
    fc_steps.append(fc.Printer(
        print_speed=cfg.max_feedrate_xyz * 0.6,
        travel_speed=cfg.travel_speed,
    ))

    extruding = True  # fc starts with extrusion on

    for step in steps:
        if isinstance(step, Move):
            if None in (step.x, step.y, step.z):
                continue

            want_extrude = step.state.extrusion_mode == ExtrusionMode.ON

            if want_extrude and not extruding:
                fc_steps.append(fc.Extruder(on=True))
                extruding = True
            elif not want_extrude and extruding:
                fc_steps.append(fc.Extruder(on=False))
                extruding = False

            # B axis as a manual G-code annotation
            pt = fc.Point(x=step.x, y=step.y, z=step.z)
            fc_steps.append(pt)

        elif isinstance(step, StateChange):
            from toolpath4.state import StateChangeKind
            if step.kind == StateChangeKind.SET_NOZZLE_TEMP:
                fc_steps.append(fc.Hotend(temp=int(step.value or 0)))
            elif step.kind == StateChangeKind.SET_BED_TEMP:
                fc_steps.append(fc.Buildplate(temp=int(step.value or 0)))
            elif step.kind == StateChangeKind.SET_FAN:
                fc_steps.append(fc.Fan(speed_percent=int((step.value or 0) * 100)))
            elif step.kind == StateChangeKind.COMMENT:
                fc_steps.append(fc.ManualGcode(
                    text=f"; {step.comment or ''}"))
            # Retract/unretract handled by fc.Extruder on/off

    return fc_steps


# ===================================================================
# Slicer -- main pipeline
# ===================================================================

class Slicer:
    """Complete slicing pipeline: mesh -> list of Steps.

    FullControl-inspired: perimeter walls define the geometry.  No infill.
    More perimeters = stronger parts.

    Supports two modes:

    1. **Auto mode** (default): horizontal slicing with overhang-based
       B-angle assignment.  Call ``process(stl_path)``.

    2. **Zone mode**: user-defined ``BuildZone`` list where each zone
       has its own build-plane orientation.  Call
       ``process(stl_path, zones=[...])``.
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
        zones: Optional[List[BuildZone]] = None,
        z_overrides: Optional[List[Tuple[float, float, float]]] = None,
        transition: str = "smooth",
    ) -> StepList:
        """Run the full pipeline and return a StepList.

        Parameters
        ----------
        stl_path : path to STL file.
        zones : list of BuildZone objects for variable-direction slicing.
                If *None*, uses classic horizontal slicing with overhang
                analysis.
        z_overrides : list of (z_min, z_max, b_deg) angle overrides
                      (auto mode only).
        transition : ``"linear"``, ``"smooth"``, or ``"adaptive"``
                     (auto mode only).
        """
        cfg = self.config

        # 1) Load
        self.mesh = load_mesh(stl_path)

        if zones:
            return self._process_zones(zones)
        else:
            return self._process_auto(z_overrides=z_overrides,
                                      transition=transition)

    # ------------------------------------------------------------------
    # Auto mode (classic horizontal slicing)
    # ------------------------------------------------------------------

    def _process_auto(
        self,
        *,
        z_overrides=None,
        transition="smooth",
    ) -> StepList:
        cfg = self.config
        mesh = self.mesh

        # 2) Slice horizontally
        self.layers = slice_mesh_along_z(mesh, cfg.layer_height)
        if not self.layers:
            print("WARNING: no layers produced.")
            return StepList()

        z_values = [l["z"] for l in self.layers]

        # 4) Overhang analysis
        self.overhang_scores = overhang_analysis(mesh, z_values)

        # 5) B-angle planning
        self.b_angles = plan_b_angles_thresholded(
            self.overhang_scores, z_values)
        if z_overrides:
            self.b_angles = apply_z_range_overrides(
                self.b_angles, z_overrides)

        if transition == "linear":
            self.b_angles = transition_linear(self.b_angles, z_values)
        elif transition == "smooth":
            self.b_angles = transition_smooth(self.b_angles, z_values)
        elif transition == "adaptive":
            self.b_angles = transition_adaptive(
                self.b_angles, self.overhang_scores, z_values)

        # Stamp b_deg onto each layer dict
        for layer in self.layers:
            layer["b_deg"] = self.b_angles.get(layer["z"], 0.0)

        # 6) Emit toolpath (perimeters only -- FullControl style)
        return self._emit_all_layers()

    # ------------------------------------------------------------------
    # Zone mode (variable build-plane)
    # ------------------------------------------------------------------

    def _process_zones(self, zones: List[BuildZone]) -> StepList:
        cfg = self.config
        mesh = self.mesh

        self.layers = []
        self.b_angles = {}
        self.overhang_scores = {}

        global_idx = 0
        for zone in sorted(zones, key=lambda z: z.z_start):
            if abs(zone.b_deg) < 0.01:
                # Horizontal slicing for B ~ 0
                zone_layers = slice_mesh_along_z(
                    mesh, cfg.layer_height,
                    z_min=zone.z_start,
                    z_max=zone.z_end,
                )
                for l in zone_layers:
                    l["b_deg"] = zone.b_deg
            else:
                # Tilted slicing
                zone_layers = slice_mesh_along_direction(
                    mesh, zone.b_deg, cfg.layer_height,
                    z_start=zone.z_start,
                    z_end=zone.z_end,
                )

            # Re-index and collect
            for l in zone_layers:
                l["index"] = global_idx
                global_idx += 1
                self.b_angles[l["z"]] = l.get("b_deg", zone.b_deg)
                self.layers.append(l)

        if not self.layers:
            print("WARNING: no layers produced from any zone.")
            return StepList()

        # Overhang scores for the B-angle chart
        z_values = [l["z"] for l in self.layers]
        self.overhang_scores = overhang_analysis(mesh, z_values)

        return self._emit_all_layers()

    # ------------------------------------------------------------------
    # Common emitter
    # ------------------------------------------------------------------

    def _emit_all_layers(self) -> StepList:
        cfg = self.config
        steps = StepList()
        steps.append(StateChange.set_nozzle_temp(cfg.nozzle_temp))
        steps.append(StateChange.set_bed_temp(cfg.bed_temp))
        steps.append(StateChange.set_fan(0.0))

        for layer in self.layers:
            if layer["index"] == 3:
                steps.append(StateChange.set_fan(1.0))
            _emit_layer_toolpath(layer, cfg, steps)

        return steps

    def rotation_schedule_summary(self) -> str:
        if not self.b_angles:
            return "No rotation schedule computed yet."
        lines = ["Z range (mm)       B angle (deg)  Overhang score"]
        lines.append("-" * 52)
        for z in sorted(self.b_angles):
            score = self.overhang_scores.get(z, 0.0)
            lines.append(
                f"  z={z:8.2f}       B={self.b_angles[z]:6.1f} deg     {score:.3f}")
        return "\n".join(lines)
