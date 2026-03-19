"""
toolpath4.preview — 3D visualisation of XYZB toolpaths.

Two backends:
  1. **matplotlib** (always available) — static 3D line plot.
  2. **plotly** (optional) — interactive 3D scatter/line plot.

Colour encodes the B-axis angle by default.
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np

from toolpath4.state import Move


def _extract_arrays(toolpath: list):
    """Pull arrays of x, y, z, b from a toolpath (Moves only)."""
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
):
    """Render the toolpath as a 3D matplotlib line plot.

    Parameters
    ----------
    toolpath : list of Step objects.
    color_by : ``"b"`` for B-angle colouring, ``"z"`` for layer colouring.
    cmap : matplotlib colourmap name.
    figsize : figure size.
    show : call ``plt.show()``.
    save_path : if set, save the figure to this path.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    xs, ys, zs, bs = _extract_arrays(toolpath)
    if len(xs) < 2:
        print("Not enough points to preview.")
        return

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection="3d")

    # Build line segments coloured by the chosen metric
    vals = bs if color_by == "b" else zs
    points = np.column_stack([xs, ys, zs]).reshape(-1, 1, 3)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    norm = plt.Normalize(vmin=vals.min(), vmax=vals.max())
    # Average value per segment for colour
    seg_vals = (vals[:-1] + vals[1:]) / 2.0

    lc = Line3DCollection(segments, cmap=cmap, norm=norm)
    lc.set_array(seg_vals)
    lc.set_linewidth(0.8)
    ax.add_collection3d(lc)

    # Axis limits (equalised)
    mid_x = (xs.min() + xs.max()) / 2
    mid_y = (ys.min() + ys.max()) / 2
    mid_z = (zs.min() + zs.max()) / 2
    max_range = max(xs.ptp(), ys.ptp(), zs.ptp()) / 2 * 1.1
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

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

    xs, ys, zs, bs = _extract_arrays(toolpath)
    if len(xs) < 2:
        print("Not enough points to preview.")
        return

    vals = bs if color_by == "b" else zs
    label = "B angle (deg)" if color_by == "b" else "Z height (mm)"

    fig = go.Figure(
        data=go.Scatter3d(
            x=xs, y=ys, z=zs,
            mode="lines",
            line=dict(
                color=vals,
                colorscale="RdBu",
                width=3,
                colorbar=dict(title=label),
            ),
            hovertext=[f"B={b:.1f}°  Z={z:.2f}" for b, z in zip(bs, zs)],
        )
    )
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
