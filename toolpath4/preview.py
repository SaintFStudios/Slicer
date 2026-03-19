"""
toolpath4.preview — 3D visualisation of XYZB toolpaths.

Two backends:
  1. **matplotlib** (always available) — static 3D line plot.
  2. **plotly** (optional) — interactive 3D scatter/line plot.

Toolpath rendering separates extrusion moves from travel moves:
  - **Extrusion segments** are drawn as coloured lines (B angle or Z height).
  - **Travel moves** are drawn as thin, semi-transparent grey lines.
This matches the approach used by PrusaSlicer and FullControl.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

from toolpath4.state import ExtrusionMode, Move


# ---------------------------------------------------------------------------
# Segment extraction — split toolpath into extrusion & travel segments
# ---------------------------------------------------------------------------

def _extract_segments(toolpath: list):
    """Split a toolpath into extrusion segments and travel segments.

    An **extrusion segment** is a run of consecutive Moves with
    ``extrusion_mode == ON``.  A **travel segment** is a run of
    consecutive ``OFF`` Moves.

    Returns
    -------
    extrude_segs : list of (points_Nx3, b_values_N) tuples
    travel_segs  : list of points_Nx3 arrays
    all_pts      : Nx3 array of every valid point (for axis limits)
    """
    extrude_segs: List[Tuple[np.ndarray, np.ndarray]] = []
    travel_segs: List[np.ndarray] = []

    cur_pts: List[List[float]] = []
    cur_bs: List[float] = []
    cur_extruding: Optional[bool] = None
    last_pt: Optional[List[float]] = None
    last_b: float = 0.0

    all_pts: List[List[float]] = []

    for step in toolpath:
        if not isinstance(step, Move):
            continue
        if None in (step.x, step.y, step.z):
            continue

        pt = [step.x, step.y, step.z]
        b = step.b if step.b is not None else 0.0
        is_ext = step.state.extrusion_mode == ExtrusionMode.ON
        all_pts.append(pt)

        if cur_extruding is None:
            # First move
            cur_extruding = is_ext
            cur_pts.append(pt)
            cur_bs.append(b)
            last_pt, last_b = pt, b
            continue

        if is_ext != cur_extruding:
            # Mode changed — flush current segment
            if len(cur_pts) >= 2:
                if cur_extruding:
                    extrude_segs.append(
                        (np.array(cur_pts), np.array(cur_bs)))
                else:
                    travel_segs.append(np.array(cur_pts))

            # New segment starts from the last point (for continuity)
            cur_pts = [last_pt, pt]
            cur_bs = [last_b, b]
            cur_extruding = is_ext
        else:
            cur_pts.append(pt)
            cur_bs.append(b)

        last_pt, last_b = pt, b

    # Flush final segment
    if len(cur_pts) >= 2:
        if cur_extruding:
            extrude_segs.append((np.array(cur_pts), np.array(cur_bs)))
        else:
            travel_segs.append(np.array(cur_pts))

    all_arr = np.array(all_pts) if all_pts else np.zeros((0, 3))
    return extrude_segs, travel_segs, all_arr


def _extract_arrays(toolpath: list):
    """Pull arrays of x, y, z, b from a toolpath (Moves only).

    Kept for backward compatibility.
    """
    xs, ys, zs, bs = [], [], [], []
    for step in toolpath:
        if isinstance(step, Move) and None not in (step.x, step.y, step.z):
            xs.append(step.x)
            ys.append(step.y)
            zs.append(step.z)
            bs.append(step.b if step.b is not None else 0.0)
    return np.array(xs), np.array(ys), np.array(zs), np.array(bs)


# ---------------------------------------------------------------------------
# Matplotlib backend
# ---------------------------------------------------------------------------

def preview_matplotlib(
    toolpath: list,
    *,
    title: str = "XYZB Toolpath Preview",
    color_by: str = "b",
    cmap: str = "coolwarm",
    figsize: tuple = (10, 8),
    show: bool = True,
    save_path: Optional[str] = None,
    show_travels: bool = True,
):
    """Render the toolpath as a 3D matplotlib line plot.

    Extrusion moves are drawn as coloured segments (by B angle or Z height).
    Travel moves are drawn as thin grey lines.

    Parameters
    ----------
    toolpath : list of Step objects.
    color_by : ``"b"`` for B-angle colouring, ``"z"`` for layer colouring.
    cmap : matplotlib colourmap name.
    figsize : figure size.
    show : call ``plt.show()``.
    save_path : if set, save the figure to this path.
    show_travels : draw travel moves as thin grey lines.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    extrude_segs, travel_segs, all_pts = _extract_segments(toolpath)
    if len(all_pts) < 2:
        print("Not enough points to preview.")
        return

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection="3d")

    # Determine global colour range from all extrusion segments
    all_vals = []
    for pts, bs in extrude_segs:
        vals = bs if color_by == "b" else pts[:, 2]
        all_vals.append(vals)
    if all_vals:
        all_vals_cat = np.concatenate(all_vals)
        vmin, vmax = all_vals_cat.min(), all_vals_cat.max()
    else:
        vmin, vmax = 0.0, 1.0
    if abs(vmax - vmin) < 1e-6:
        vmax = vmin + 1.0
    norm = plt.Normalize(vmin=vmin, vmax=vmax)

    # Draw extrusion segments
    for pts, bs in extrude_segs:
        vals = bs if color_by == "b" else pts[:, 2]
        points = pts.reshape(-1, 1, 3)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        seg_vals = (vals[:-1] + vals[1:]) / 2.0

        lc = Line3DCollection(segments, cmap=cmap, norm=norm)
        lc.set_array(seg_vals)
        lc.set_linewidth(0.8)
        ax.add_collection3d(lc)

    # Draw travel segments
    if show_travels:
        for pts in travel_segs:
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                    color="grey", linewidth=0.3, alpha=0.3)

    # Axis limits (equalised)
    mid = all_pts.mean(axis=0)
    max_range = (all_pts.max(axis=0) - all_pts.min(axis=0)).max() / 2 * 1.1
    if max_range < 1e-6:
        max_range = 1.0
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title(title)

    # Colour bar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    label = "B angle (deg)" if color_by == "b" else "Z height (mm)"
    fig.colorbar(sm, ax=ax, label=label, shrink=0.6)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    if show:
        plt.show()
    return fig, ax


# ---------------------------------------------------------------------------
# Plotly backend (optional)
# ---------------------------------------------------------------------------

def preview_plotly(
    toolpath: list,
    *,
    title: str = "XYZB Toolpath Preview",
    color_by: str = "b",
    show_travels: bool = True,
):
    """Interactive 3D plotly visualisation.

    Requires ``plotly`` to be installed; raises ImportError otherwise.
    """
    try:
        import plotly.graph_objects as go
    except ImportError:
        raise ImportError(
            "plotly is required for interactive preview.  "
            "Install with:  pip install plotly"
        )

    extrude_segs, travel_segs, all_pts = _extract_segments(toolpath)
    if len(all_pts) < 2:
        print("Not enough points to preview.")
        return

    label = "B angle (deg)" if color_by == "b" else "Z height (mm)"
    fig = go.Figure()

    # Extrusion segments — each as a separate trace with colour
    for pts, bs in extrude_segs:
        vals = bs if color_by == "b" else pts[:, 2]
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode="lines",
            line=dict(
                color=vals,
                colorscale="RdBu",
                width=3,
                colorbar=dict(title=label),
            ),
            hovertext=[f"B={b:.1f}°  Z={z:.2f}" for b, z in zip(bs, pts[:, 2])],
            showlegend=False,
        ))

    # Travel segments
    if show_travels:
        for pts in travel_segs:
            fig.add_trace(go.Scatter3d(
                x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
                mode="lines",
                line=dict(color="grey", width=1),
                opacity=0.3,
                showlegend=False,
            ))

    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title="X (mm)",
            yaxis_title="Y (mm)",
            zaxis_title="Z (mm)",
            aspectmode="data",
        ),
    )
    fig.show()
    return fig
