"""
toolpath4.gui — Tkinter GUI for the 4-axis XYZB slicer.

Launch with::

    python -m toolpath4          # via __main__.py
    python toolpath4/gui.py      # directly

Features:
  - STL file browser and 3D model viewer (equal-aspect, isometric home)
  - Build Zone editor for variable slicing coordinate systems
  - B-angle schedule chart (post-slice, driven by real overhang analysis)
  - 3D toolpath preview colour-coded by B angle with PrusaSlicer-style
    layer slider for scrubbing through the build
  - G-code export via Save-As dialog
"""

from __future__ import annotations

import math
import os
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk, simpledialog
from pathlib import Path
from typing import List, Optional

# Ensure the parent directory of toolpath4/ is on sys.path so that
# ``python toolpath4/gui.py`` works even without installing the package.
_PACKAGE_DIR = Path(__file__).resolve().parent
_PROJECT_DIR = _PACKAGE_DIR.parent
if str(_PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(_PROJECT_DIR))

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Line3DCollection

try:
    import trimesh
except ImportError:
    trimesh = None  # type: ignore[assignment]

from toolpath4.config import PrinterConfig
from toolpath4.state import ExtrusionMode, Move, StateChange as SC, StepList
from toolpath4.compiler import compile_gcode, dry_run
from toolpath4.slicer import BuildZone, Slicer, load_mesh, build_direction


# ===================================================================
# Helper: equalize 3-D axes
# ===================================================================

def _equalize_3d_axes(ax, verts: np.ndarray):
    """Set equal-aspect axis limits centred on the data and isometric view."""
    mid = verts.mean(axis=0)
    span = (verts.max(axis=0) - verts.min(axis=0)).max() / 2.0 * 1.25
    if span < 1e-6:
        span = 1.0
    ax.set_xlim(mid[0] - span, mid[0] + span)
    ax.set_ylim(mid[1] - span, mid[1] + span)
    ax.set_zlim(mid[2] - span, mid[2] + span)
    try:
        ax.set_box_aspect([1, 1, 1])
    except AttributeError:
        pass  # matplotlib < 3.3
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.view_init(elev=25, azim=-60)


# Maximum extrusion points to display (keeps rotation interactive)
_MAX_DISPLAY_POINTS = 8000


# ===================================================================
# 3-D model viewer
# ===================================================================

class ModelViewer:
    """Displays an STL mesh in a matplotlib 3-D subplot with proper scaling."""

    def __init__(self, parent: tk.Widget):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Toolbar — override home to be iso-view reset
        bar_frame = ttk.Frame(frame)
        bar_frame.pack(fill=tk.X)
        self._toolbar = NavigationToolbar2Tk(self.canvas, bar_frame)
        self._toolbar.update()
        self._toolbar.home = self._reset_view

        self._verts: Optional[np.ndarray] = None
        self.ax.set_title("Load an STL to begin")

    def show_model(self, stl_path: str) -> Optional["trimesh.Trimesh"]:
        """Load and render the STL.  Returns the mesh or None."""
        if trimesh is None:
            messagebox.showerror("Error", "trimesh is not installed.")
            return None
        try:
            mesh = trimesh.load(stl_path)
            if isinstance(mesh, trimesh.Scene):
                mesh = mesh.dump(concatenate=True)
        except Exception as exc:
            messagebox.showerror("Error", f"Failed to load STL:\n{exc}")
            return None

        self.ax.clear()
        verts = np.asarray(mesh.vertices)
        faces = np.asarray(mesh.faces)
        self._verts = verts

        display_faces = faces
        if len(faces) > 50_000:
            idx = np.random.choice(len(faces), 50_000, replace=False)
            display_faces = faces[idx]

        self.ax.plot_trisurf(
            verts[:, 0], verts[:, 1], verts[:, 2],
            triangles=display_faces,
            color=(0.9, 0.5, 0.1), alpha=0.8,
            edgecolor="darkgray", linewidth=0.1,
        )
        _equalize_3d_axes(self.ax, verts)
        self.ax.set_title(Path(stl_path).name)
        self.canvas.draw()
        return mesh

    def draw_build_planes(self, zones: List[BuildZone], mesh_bounds):
        """Overlay translucent build-plane indicators for each zone."""
        if not zones or mesh_bounds is None:
            return
        bmin, bmax = mesh_bounds
        cx = (bmin[0] + bmax[0]) / 2
        cy = (bmin[1] + bmax[1]) / 2
        half = max(bmax[0] - bmin[0], bmax[1] - bmin[1]) / 2 * 0.6
        colors = ["#2266cc", "#cc4422", "#22aa44", "#aa44cc", "#ccaa22"]

        for i, zone in enumerate(zones):
            color = colors[i % len(colors)]
            z_mid = (zone.z_start + zone.z_end) / 2
            n = build_direction(zone.b_deg)
            u, v = np.array([1, 0, 0], dtype=float), np.array([0, 1, 0], dtype=float)
            if abs(zone.b_deg) > 0.01:
                theta = math.radians(zone.b_deg)
                u = np.array([math.cos(theta), 0, -math.sin(theta)])
                v = np.array([0, 1, 0])
            centre = np.array([cx, cy, z_mid])
            corners = [
                centre - u * half - v * half,
                centre + u * half - v * half,
                centre + u * half + v * half,
                centre - u * half + v * half,
            ]
            xs = [c[0] for c in corners] + [corners[0][0]]
            ys = [c[1] for c in corners] + [corners[0][1]]
            zs = [c[2] for c in corners] + [corners[0][2]]
            self.ax.plot(xs, ys, zs, color=color, linewidth=2, alpha=0.7)
            self.ax.text(centre[0], centre[1], centre[2],
                         f"B={zone.b_deg:.0f}", color=color, fontsize=8)
        self.canvas.draw()

    def _reset_view(self, *_args):
        """Reset to isometric view with correct limits."""
        if self._verts is not None and len(self._verts) > 0:
            _equalize_3d_axes(self.ax, self._verts)
        else:
            self.ax.view_init(elev=25, azim=-60)
        self.canvas.draw()


# ===================================================================
# B-angle schedule chart
# ===================================================================

class BAngleChart:
    def __init__(self, parent: tk.Widget):
        self.fig = Figure(figsize=(6, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._draw_empty()

    def _draw_empty(self):
        self.ax.clear()
        self.ax.set_xlabel("Z height (mm)")
        self.ax.set_ylabel("B angle (deg)")
        self.ax.set_title("B-Angle Schedule  (slice first)")
        self.ax.grid(True, alpha=0.3)
        self.canvas.draw()

    def update(self, slicer: Slicer):
        self.ax.clear()
        if not slicer.b_angles:
            self._draw_empty()
            return

        z_vals = sorted(slicer.b_angles.keys())
        b_vals = [slicer.b_angles[z] for z in z_vals]
        scores = [slicer.overhang_scores.get(z, 0.0) for z in z_vals]

        color_b = "#2266cc"
        self.ax.plot(z_vals, b_vals, color=color_b, linewidth=2, label="B angle")
        self.ax.fill_between(z_vals, b_vals, alpha=0.15, color=color_b)
        self.ax.set_xlabel("Z height (mm)")
        self.ax.set_ylabel("B angle (deg)", color=color_b)
        self.ax.tick_params(axis="y", labelcolor=color_b)

        ax2 = self.ax.twinx()
        color_oh = "#cc4422"
        ax2.plot(z_vals, scores, color=color_oh, linewidth=1, linestyle="--",
                 alpha=0.7, label="Overhang score")
        ax2.set_ylabel("Overhang score", color=color_oh)
        ax2.tick_params(axis="y", labelcolor=color_oh)
        ax2.set_ylim(0, 1)
        ax2.axhline(0.15, color=color_oh, linewidth=0.5, linestyle=":")
        ax2.axhline(0.35, color=color_oh, linewidth=0.5, linestyle=":")

        self.ax.set_title("B-Angle Schedule (solid) & Overhang Score (dashed)")
        self.ax.grid(True, alpha=0.3)
        self.fig.tight_layout()
        self.canvas.draw()


# ===================================================================
# Toolpath 3-D preview with layer slider
# ===================================================================

def _extract_layer_segments(steps):
    """Extract extrusion segments from a subset of steps.

    Returns list of (pts_Nx3, bs_N) tuples for extrusion runs.
    """
    extrude_segs = []
    cur_pts: List[List[float]] = []
    cur_bs: List[float] = []
    cur_extruding: Optional[bool] = None
    last_pt: Optional[List[float]] = None
    last_b: float = 0.0

    for step in steps:
        if not isinstance(step, Move):
            continue
        if None in (step.x, step.y, step.z):
            continue

        pt = [step.x, step.y, step.z]
        b = step.b if step.b is not None else 0.0
        is_ext = step.state.extrusion_mode == ExtrusionMode.ON

        if cur_extruding is None:
            cur_extruding = is_ext
            cur_pts.append(pt)
            cur_bs.append(b)
            last_pt, last_b = pt, b
            continue

        if is_ext != cur_extruding:
            if len(cur_pts) >= 2 and cur_extruding:
                extrude_segs.append(
                    (np.array(cur_pts), np.array(cur_bs)))
            cur_pts = [last_pt, pt]
            cur_bs = [last_b, b]
            cur_extruding = is_ext
        else:
            cur_pts.append(pt)
            cur_bs.append(b)

        last_pt, last_b = pt, b

    if len(cur_pts) >= 2 and cur_extruding:
        extrude_segs.append((np.array(cur_pts), np.array(cur_bs)))

    return extrude_segs


class ToolpathViewer:
    """3-D toolpath preview with per-layer slider (like PrusaSlicer).

    Performance optimisations vs. naive approach:
      - One Line3DCollection per *layer* (not per segment).
      - Points downsampled to ~8 000 total.
      - Lower figure DPI (85) for faster rendering.
      - Slider toggles collection visibility (O(1) per layer).
      - Home button = iso view (overrides default matplotlib zoom-to-fit).
    """

    def __init__(self, parent: tk.Widget):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(6, 6), dpi=85)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Toolbar — override home = iso view
        bar_frame = ttk.Frame(frame)
        bar_frame.pack(fill=tk.X)
        self._toolbar = NavigationToolbar2Tk(self.canvas, bar_frame)
        self._toolbar.update()
        self._toolbar.home = self._reset_view

        # Layer slider (PrusaSlicer-style)
        slider_frame = ttk.Frame(frame)
        slider_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(slider_frame, text="Layer:").pack(side=tk.LEFT)
        self._layer_slider = tk.Scale(
            slider_frame, from_=0, to=0, orient=tk.HORIZONTAL,
            showvalue=False, command=self._on_layer_change)
        self._layer_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self._layer_label = ttk.Label(slider_frame, text="0 / 0", width=12)
        self._layer_label.pack(side=tk.RIGHT)

        self._pts: Optional[np.ndarray] = None
        self._layer_collections: List[Line3DCollection] = []
        self._max_layer: int = 0
        self.ax.set_title("Toolpath Preview  (slice first)")

    # ------------------------------------------------------------------
    # Data loading
    # ------------------------------------------------------------------

    def update(self, steps: list):
        """Parse steps into per-layer collections and display."""
        self.ax.clear()
        # Remove old colorbars
        while self.fig.axes[1:]:
            self.fig.delaxes(self.fig.axes[-1])
        self._layer_collections.clear()

        # Split steps into layers at "Layer ..." comments
        layer_groups = self._split_into_layers(steps)
        if not layer_groups:
            self.ax.set_title("No toolpath to preview")
            self.canvas.draw()
            return

        # Extract extrusion segments per layer
        per_layer_segs = []
        total_pts = 0
        for group in layer_groups:
            segs = _extract_layer_segments(group)
            if segs:
                n = sum(len(pts) for pts, _ in segs)
                per_layer_segs.append(segs)
                total_pts += n

        if not per_layer_segs:
            self.ax.set_title("No extrusion points")
            self.canvas.draw()
            return

        # Compute downsample stride
        stride = max(1, total_pts // _MAX_DISPLAY_POINTS)

        # Global colour range
        all_bs = np.concatenate(
            [bs for segs in per_layer_segs for _, bs in segs])
        vmin, vmax = float(all_bs.min()), float(all_bs.max())
        if abs(vmax - vmin) < 1e-6:
            vmax = vmin + 1.0
        norm = plt.Normalize(vmin=vmin, vmax=vmax)

        # Global point bounds (for axis limits)
        all_pts_list = [pts for segs in per_layer_segs for pts, _ in segs]
        self._pts = np.concatenate(all_pts_list)

        # Build one Line3DCollection per layer
        display_pts = 0
        for segs in per_layer_segs:
            combined_segs = []
            combined_vals = []
            for pts, bs in segs:
                # Downsample
                if stride > 1 and len(pts) > stride * 2:
                    idx = np.arange(0, len(pts), stride)
                    if idx[-1] != len(pts) - 1:
                        idx = np.append(idx, len(pts) - 1)
                    pts = pts[idx]
                    bs = bs[idx]
                display_pts += len(pts)
                p = pts.reshape(-1, 1, 3)
                seg_pairs = np.concatenate([p[:-1], p[1:]], axis=1)
                vals = (bs[:-1] + bs[1:]) / 2.0
                combined_segs.append(seg_pairs)
                combined_vals.append(vals)

            merged_segs = np.concatenate(combined_segs, axis=0)
            merged_vals = np.concatenate(combined_vals)
            lc = Line3DCollection(merged_segs, cmap="coolwarm", norm=norm)
            lc.set_array(merged_vals)
            lc.set_linewidth(0.6)
            self.ax.add_collection3d(lc)
            self._layer_collections.append(lc)

        self._max_layer = len(self._layer_collections) - 1

        # Axis setup
        _equalize_3d_axes(self.ax, self._pts)
        self.ax.set_title(
            f"Toolpath ({display_pts} pts, {self._max_layer + 1} layers)")

        # Colour bar
        sm = plt.cm.ScalarMappable(cmap="coolwarm", norm=norm)
        sm.set_array([])
        self.fig.colorbar(sm, ax=self.ax, label="B angle (deg)", shrink=0.6)

        # Configure slider
        self._layer_slider.configure(from_=0, to=self._max_layer)
        self._layer_slider.set(self._max_layer)
        self._layer_label.config(
            text=f"{self._max_layer} / {self._max_layer}")

        self.canvas.draw()

    # ------------------------------------------------------------------
    # Layer parsing
    # ------------------------------------------------------------------

    @staticmethod
    def _split_into_layers(steps: list) -> List[list]:
        """Split the step list at layer-comment boundaries."""
        layers: List[list] = []
        current: list = []
        for step in steps:
            if (isinstance(step, SC) and step.comment
                    and step.comment.startswith("Layer ")):
                if current:
                    layers.append(current)
                current = [step]
            else:
                current.append(step)
        if current:
            layers.append(current)
        return layers

    # ------------------------------------------------------------------
    # Slider callback
    # ------------------------------------------------------------------

    def _on_layer_change(self, val):
        layer_idx = int(float(val))
        self._layer_label.config(
            text=f"{layer_idx} / {self._max_layer}")
        for i, lc in enumerate(self._layer_collections):
            lc.set_visible(i <= layer_idx)
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # View reset
    # ------------------------------------------------------------------

    def _reset_view(self, *_args):
        if self._pts is not None and len(self._pts) > 0:
            _equalize_3d_axes(self.ax, self._pts)
            self.canvas.draw()


# ===================================================================
# Build Zone editor
# ===================================================================

class BuildZoneEditor:
    """Widget for creating / editing BuildZone list."""

    def __init__(self, parent: tk.Widget, on_change=None):
        self.zones: List[BuildZone] = []
        self._on_change = on_change

        frame = ttk.LabelFrame(parent, text="Build Zones", padding=5)
        frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # Mode selector
        mode_frame = ttk.Frame(frame)
        mode_frame.pack(fill=tk.X, pady=2)
        self.mode_var = tk.StringVar(value="auto")
        ttk.Radiobutton(mode_frame, text="Auto (overhang)",
                        variable=self.mode_var, value="auto",
                        command=self._mode_changed).pack(side=tk.LEFT)
        ttk.Radiobutton(mode_frame, text="Manual zones",
                        variable=self.mode_var, value="manual",
                        command=self._mode_changed).pack(side=tk.LEFT, padx=8)

        # Zone list
        self.zone_frame = ttk.Frame(frame)
        self.zone_frame.pack(fill=tk.BOTH, expand=True)

        cols = ("B (deg)", "Z start", "Z end", "Label")
        self.tree = ttk.Treeview(self.zone_frame, columns=cols, show="headings",
                                 height=4, selectmode="browse")
        for c in cols:
            self.tree.heading(c, text=c)
            self.tree.column(c, width=65, anchor=tk.CENTER)
        self.tree.pack(fill=tk.BOTH, expand=True)

        btn_frame = ttk.Frame(self.zone_frame)
        btn_frame.pack(fill=tk.X, pady=2)
        ttk.Button(btn_frame, text="Add", width=6,
                   command=self._add_zone).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="Edit", width=6,
                   command=self._edit_zone).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="Remove", width=7,
                   command=self._remove_zone).pack(side=tk.LEFT, padx=2)

        self._mode_changed()

    def _mode_changed(self):
        manual = self.mode_var.get() == "manual"
        state = tk.NORMAL if manual else tk.DISABLED
        for child in self.zone_frame.winfo_children():
            try:
                child.configure(state=state)
            except tk.TclError:
                pass
        if not manual:
            self.zones.clear()
            self._refresh_tree()

    def _refresh_tree(self):
        self.tree.delete(*self.tree.get_children())
        for z in self.zones:
            z_end = f"{z.z_end:.1f}" if z.z_end < 1e6 else "end"
            self.tree.insert("", tk.END, values=(
                f"{z.b_deg:.1f}", f"{z.z_start:.1f}", z_end, z.label))
        if self._on_change:
            self._on_change()

    def _add_zone(self):
        z = self._zone_dialog("Add Build Zone")
        if z:
            self.zones.append(z)
            self._refresh_tree()

    def _edit_zone(self):
        sel = self.tree.selection()
        if not sel:
            return
        idx = self.tree.index(sel[0])
        z = self._zone_dialog("Edit Build Zone", self.zones[idx])
        if z:
            self.zones[idx] = z
            self._refresh_tree()

    def _remove_zone(self):
        sel = self.tree.selection()
        if not sel:
            return
        idx = self.tree.index(sel[0])
        self.zones.pop(idx)
        self._refresh_tree()

    def _zone_dialog(self, title: str, existing: Optional[BuildZone] = None):
        """Pop up a small dialog to define a BuildZone."""
        dlg = tk.Toplevel()
        dlg.title(title)
        dlg.geometry("280x200")
        dlg.transient()
        dlg.grab_set()

        b_var = tk.DoubleVar(value=existing.b_deg if existing else 45.0)
        zs_var = tk.DoubleVar(value=existing.z_start if existing else 0.0)
        ze_var = tk.DoubleVar(value=existing.z_end if existing and existing.z_end < 1e6 else 999.0)
        lbl_var = tk.StringVar(value=existing.label if existing else "")

        ttk.Label(dlg, text="B angle (deg):").grid(row=0, column=0, sticky=tk.W, padx=8, pady=4)
        ttk.Spinbox(dlg, from_=-120, to=120, increment=5,
                     textvariable=b_var, width=10).grid(row=0, column=1, padx=8)
        ttk.Label(dlg, text="Z start (mm):").grid(row=1, column=0, sticky=tk.W, padx=8, pady=4)
        ttk.Entry(dlg, textvariable=zs_var, width=12).grid(row=1, column=1, padx=8)
        ttk.Label(dlg, text="Z end (mm):").grid(row=2, column=0, sticky=tk.W, padx=8, pady=4)
        ttk.Entry(dlg, textvariable=ze_var, width=12).grid(row=2, column=1, padx=8)
        ttk.Label(dlg, text="Label:").grid(row=3, column=0, sticky=tk.W, padx=8, pady=4)
        ttk.Entry(dlg, textvariable=lbl_var, width=12).grid(row=3, column=1, padx=8)

        result = [None]

        def _ok():
            result[0] = BuildZone(
                b_deg=b_var.get(),
                z_start=zs_var.get(),
                z_end=ze_var.get(),
                label=lbl_var.get(),
            )
            dlg.destroy()

        btn_f = ttk.Frame(dlg)
        btn_f.grid(row=4, column=0, columnspan=2, pady=10)
        ttk.Button(btn_f, text="OK", command=_ok).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_f, text="Cancel", command=dlg.destroy).pack(side=tk.LEFT)

        dlg.wait_window()
        return result[0]

    def get_zones(self) -> Optional[List[BuildZone]]:
        """Return zones list, or None if in auto mode."""
        if self.mode_var.get() == "auto":
            return None
        return list(self.zones) if self.zones else None


# ===================================================================
# Main application
# ===================================================================

class SlicerApp:
    """Top-level tkinter GUI for toolpath4."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("toolpath4 -- 4-Axis XYZB Slicer")
        self.root.geometry("1600x900")

        self.stl_path: Optional[str] = None
        self.mesh = None
        self.slicer: Optional[Slicer] = None
        self.steps: Optional[StepList] = None

        self._build_ui()

    # ---- UI construction -----------------------------------------------

    def _build_ui(self):
        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Left panel (controls)
        left = ttk.Frame(main, width=380)
        left.pack(side=tk.LEFT, fill=tk.BOTH, padx=5)
        left.pack_propagate(False)
        self._build_left_panel(left)

        # Right panel (tabs)
        right = ttk.Frame(main)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        self._build_right_panel(right)

    def _build_left_panel(self, parent: ttk.Frame):
        # --- File ---
        ff = ttk.LabelFrame(parent, text="1. Load Model", padding=8)
        ff.pack(fill=tk.X, pady=3)
        self.file_label = ttk.Label(ff, text="No file selected",
                                    foreground="red", wraplength=340)
        self.file_label.pack(fill=tk.X)
        ttk.Button(ff, text="Browse STL File",
                   command=self._browse).pack(fill=tk.X, pady=4)

        # --- Model info ---
        mf = ttk.LabelFrame(parent, text="Model Info", padding=8)
        mf.pack(fill=tk.X, pady=3)
        self.info_text = tk.Text(mf, height=5, width=40, state=tk.DISABLED,
                                 wrap=tk.WORD, font=("Consolas", 9))
        self.info_text.pack(fill=tk.BOTH)

        # --- Settings ---
        sf = ttk.LabelFrame(parent, text="2. Settings", padding=8)
        sf.pack(fill=tk.X, pady=3)
        row = 0

        def _spin(label, var, from_, to_, inc, r):
            ttk.Label(sf, text=label).grid(row=r, column=0, sticky=tk.W, pady=2)
            ttk.Spinbox(sf, from_=from_, to=to_, increment=inc,
                        textvariable=var, width=10).grid(
                row=r, column=1, sticky=tk.W, padx=4)

        self.layer_height_var = tk.DoubleVar(value=0.20)
        _spin("Layer Height (mm):", self.layer_height_var, 0.05, 0.50, 0.05, row); row += 1
        self.perimeter_var = tk.IntVar(value=3)
        _spin("Wall Count:", self.perimeter_var, 1, 10, 1, row); row += 1
        self.nozzle_temp_var = tk.IntVar(value=200)
        _spin("Nozzle Temp (C):", self.nozzle_temp_var, 150, 300, 5, row); row += 1
        self.bed_temp_var = tk.IntVar(value=60)
        _spin("Bed Temp (C):", self.bed_temp_var, 20, 120, 5, row); row += 1

        ttk.Label(sf, text="Transition:").grid(row=row, column=0,
                                               sticky=tk.W, pady=2)
        self.strategy_var = tk.StringVar(value="smooth")
        ttk.Combobox(sf, textvariable=self.strategy_var,
                     values=["linear", "smooth", "adaptive"],
                     width=12, state="readonly").grid(
            row=row, column=1, sticky=tk.W, padx=4)
        row += 1

        # --- Build Zones ---
        self.zone_editor = BuildZoneEditor(parent,
                                           on_change=self._on_zones_changed)

        # --- Generate ---
        gf = ttk.LabelFrame(parent, text="3. Generate", padding=8)
        gf.pack(fill=tk.X, pady=3)
        self.slice_btn = ttk.Button(gf, text="SLICE", command=self._slice,
                                    state=tk.DISABLED)
        self.slice_btn.pack(fill=tk.X, pady=2)
        self.export_btn = ttk.Button(gf, text="EXPORT G-CODE",
                                     command=self._export_gcode,
                                     state=tk.DISABLED)
        self.export_btn.pack(fill=tk.X, pady=2)

        # --- Status ---
        self.status_label = ttk.Label(parent, text="Ready", foreground="blue",
                                      wraplength=340)
        self.status_label.pack(fill=tk.X, pady=3)
        self.progress = ttk.Progressbar(parent, mode="indeterminate")
        self.progress.pack(fill=tk.X, pady=2)

        # --- Stats ---
        self.stats_text = tk.Text(parent, height=5, width=40, state=tk.DISABLED,
                                  wrap=tk.WORD, font=("Consolas", 9))
        self.stats_text.pack(fill=tk.BOTH, expand=True, pady=3)

    def _build_right_panel(self, parent: ttk.Frame):
        nb = ttk.Notebook(parent)
        nb.pack(fill=tk.BOTH, expand=True)

        tab_model = ttk.Frame(nb)
        nb.add(tab_model, text="3D Model")
        self.model_viewer = ModelViewer(tab_model)

        tab_bangle = ttk.Frame(nb)
        nb.add(tab_bangle, text="B-Angle Schedule")
        self.bangle_chart = BAngleChart(tab_bangle)

        tab_toolpath = ttk.Frame(nb)
        nb.add(tab_toolpath, text="Toolpath Preview")
        self.toolpath_viewer = ToolpathViewer(tab_toolpath)

    # ---- Callbacks -----------------------------------------------------

    def _on_zones_changed(self):
        """Redraw build-plane indicators on the 3D model."""
        if self.mesh is None:
            return
        self.model_viewer.show_model(self.stl_path)
        zones = self.zone_editor.get_zones()
        if zones:
            self.model_viewer.draw_build_planes(zones, self.mesh.bounds)

    def _browse(self):
        path = filedialog.askopenfilename(
            title="Select STL File",
            filetypes=[("STL files", "*.stl"), ("All files", "*.*")],
        )
        if not path:
            return
        self.stl_path = path
        self.file_label.config(text=Path(path).name, foreground="green")
        self.status_label.config(text="Loading model...", foreground="orange")
        self.root.update_idletasks()

        self.mesh = self.model_viewer.show_model(path)
        if self.mesh is None:
            self.status_label.config(text="Failed to load STL", foreground="red")
            return

        bounds = self.mesh.bounds
        height = bounds[1][2] - bounds[0][2]
        layers_est = int(np.ceil(height / self.layer_height_var.get()))
        info = (
            f"File:  {Path(path).name}\n"
            f"Faces: {len(self.mesh.faces)}\n"
            f"X: {bounds[0][0]:.1f} - {bounds[1][0]:.1f} mm\n"
            f"Y: {bounds[0][1]:.1f} - {bounds[1][1]:.1f} mm\n"
            f"Z: {bounds[0][2]:.1f} - {bounds[1][2]:.1f} mm\n"
            f"Height: {height:.1f} mm  (~{layers_est} layers)\n"
            f"Watertight: {self.mesh.is_watertight}"
        )
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete("1.0", tk.END)
        self.info_text.insert("1.0", info)
        self.info_text.config(state=tk.DISABLED)

        self.slice_btn.config(state=tk.NORMAL)
        self.status_label.config(text="Ready to slice", foreground="blue")

    def _build_config(self) -> PrinterConfig:
        return PrinterConfig(
            layer_height=self.layer_height_var.get(),
            perimeter_count=self.perimeter_var.get(),
            nozzle_temp=self.nozzle_temp_var.get(),
            bed_temp=self.bed_temp_var.get(),
        )

    def _slice(self):
        if not self.stl_path:
            return
        self.slice_btn.config(state=tk.DISABLED)
        self.export_btn.config(state=tk.DISABLED)
        self.status_label.config(text="Slicing... please wait",
                                 foreground="orange")
        self.progress.start(15)

        cfg = self._build_config()
        strategy = self.strategy_var.get()
        zones = self.zone_editor.get_zones()

        def _worker():
            try:
                slicer = Slicer(config=cfg)
                if zones:
                    steps = slicer.process(self.stl_path, zones=zones)
                else:
                    steps = slicer.process(self.stl_path, transition=strategy)
                self.root.after(0, self._on_slice_done, slicer, steps, None)
            except Exception as exc:
                import traceback
                traceback.print_exc()
                self.root.after(0, self._on_slice_done, None, None, exc)

        threading.Thread(target=_worker, daemon=True).start()

    def _on_slice_done(self, slicer, steps, error):
        self.progress.stop()
        if error is not None:
            self.status_label.config(text=f"Error: {error}", foreground="red")
            self.slice_btn.config(state=tk.NORMAL)
            messagebox.showerror("Slicing Failed", str(error))
            return

        try:
            self.slicer = slicer
            self.steps = steps

            self.bangle_chart.update(slicer)
            self.toolpath_viewer.update(steps)

            cfg = self._build_config()
            stats = dry_run(steps, cfg)
            txt = (
                f"Layers:      {stats.layer_count}\n"
                f"Moves:       {stats.move_count}\n"
                f"Path length: {stats.total_path_length_mm:.1f} mm\n"
                f"Extrusion:   {stats.total_extrusion_mm:.1f} mm\n"
                f"B range:     [{stats.b_min_deg:.1f}, {stats.b_max_deg:.1f}] deg\n"
            )
            self.stats_text.config(state=tk.NORMAL)
            self.stats_text.delete("1.0", tk.END)
            self.stats_text.insert("1.0", txt)
            self.stats_text.config(state=tk.DISABLED)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            self.status_label.config(
                text=f"Preview error: {exc}", foreground="red")
            self.slice_btn.config(state=tk.NORMAL)
            self.export_btn.config(state=tk.NORMAL)
            return

        self.export_btn.config(state=tk.NORMAL)
        self.slice_btn.config(state=tk.NORMAL)
        self.status_label.config(text="Slicing complete!", foreground="green")

    def _export_gcode(self):
        if not self.steps:
            return
        path = filedialog.asksaveasfilename(
            title="Save G-code",
            defaultextension=".gcode",
            filetypes=[("G-code files", "*.gcode"), ("All files", "*.*")],
            initialfile="output.gcode",
        )
        if not path:
            return
        cfg = self._build_config()
        gcode = compile_gcode(self.steps, cfg)
        with open(path, "w") as f:
            f.write(gcode)
        lines = gcode.count("\n") + 1
        self.status_label.config(
            text=f"G-code saved ({lines} lines): {Path(path).name}",
            foreground="green",
        )
        messagebox.showinfo("Exported",
                            f"G-code saved to:\n{path}\n\n{lines} lines")


# ===================================================================
# Entry point
# ===================================================================

def main():
    root = tk.Tk()
    app = SlicerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
