"""
toolpath4.gui — Tkinter GUI for the 4-axis XYZB slicer.

Launch with::

    python -m toolpath4          # via __main__.py
    python -m toolpath4.gui      # directly

Features:
  - STL file browser and 3D model viewer
  - Adjustable slicing / B-axis settings
  - B-angle schedule chart (post-slice, driven by real overhang analysis)
  - 3D toolpath preview colour-coded by B angle
  - G-code export via Save-As dialog
"""

from __future__ import annotations

import os
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from pathlib import Path
from typing import Optional

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
from toolpath4.state import Move, StepList
from toolpath4.compiler import compile_gcode, dry_run
from toolpath4.preview import _extract_arrays
from toolpath4.slicer import Slicer, load_mesh


# ---------------------------------------------------------------------------
# 3-D model viewer (matplotlib trisurf)
# ---------------------------------------------------------------------------

class ModelViewer:
    """Displays an STL mesh in a matplotlib 3-D subplot."""

    def __init__(self, parent: tk.Widget):
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._toolbar = NavigationToolbar2Tk(self.canvas, parent)
        self._toolbar.update()
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")
        self.ax.set_title("Load an STL to begin")

    def show_model(self, stl_path: str) -> Optional["trimesh.Trimesh"]:
        """Load and render the STL.  Returns the trimesh mesh or None."""
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

        # Decimate for display if very heavy
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
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")
        self.ax.set_title(Path(stl_path).name)
        self.ax.view_init(elev=20, azim=45)
        self.canvas.draw()
        return mesh


# ---------------------------------------------------------------------------
# B-angle schedule chart
# ---------------------------------------------------------------------------

class BAngleChart:
    """Plots Z vs B-angle and overhang score after slicing."""

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
        """Populate from a completed Slicer instance."""
        self.ax.clear()
        if not slicer.b_angles:
            self._draw_empty()
            return

        z_vals = sorted(slicer.b_angles.keys())
        b_vals = [slicer.b_angles[z] for z in z_vals]
        scores = [slicer.overhang_scores.get(z, 0.0) for z in z_vals]

        # Primary: B angle
        color_b = "#2266cc"
        self.ax.plot(z_vals, b_vals, color=color_b, linewidth=2, label="B angle")
        self.ax.fill_between(z_vals, b_vals, alpha=0.15, color=color_b)
        self.ax.set_xlabel("Z height (mm)")
        self.ax.set_ylabel("B angle (deg)", color=color_b)
        self.ax.tick_params(axis="y", labelcolor=color_b)

        # Secondary: overhang score
        ax2 = self.ax.twinx()
        color_oh = "#cc4422"
        ax2.plot(z_vals, scores, color=color_oh, linewidth=1, linestyle="--",
                 alpha=0.7, label="Overhang score")
        ax2.set_ylabel("Overhang score", color=color_oh)
        ax2.tick_params(axis="y", labelcolor=color_oh)
        ax2.set_ylim(0, 1)

        # Threshold lines
        ax2.axhline(0.15, color=color_oh, linewidth=0.5, linestyle=":")
        ax2.axhline(0.35, color=color_oh, linewidth=0.5, linestyle=":")

        self.ax.set_title("B-Angle Schedule (solid) & Overhang Score (dashed)")
        self.ax.grid(True, alpha=0.3)
        self.fig.tight_layout()
        self.canvas.draw()


# ---------------------------------------------------------------------------
# Toolpath 3-D preview
# ---------------------------------------------------------------------------

class ToolpathViewer:
    """Colour-coded 3-D toolpath preview (matplotlib)."""

    def __init__(self, parent: tk.Widget):
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._toolbar = NavigationToolbar2Tk(self.canvas, parent)
        self._toolbar.update()
        self.ax.set_title("Toolpath Preview  (slice first)")

    def update(self, steps: list):
        """Render the toolpath."""
        self.ax.clear()
        xs, ys, zs, bs = _extract_arrays(steps)
        if len(xs) < 2:
            self.ax.set_title("No toolpath to preview")
            self.canvas.draw()
            return

        points = np.column_stack([xs, ys, zs]).reshape(-1, 1, 3)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        seg_vals = (bs[:-1] + bs[1:]) / 2.0

        norm = plt.Normalize(vmin=bs.min(), vmax=bs.max())
        lc = Line3DCollection(segments, cmap="coolwarm", norm=norm)
        lc.set_array(seg_vals)
        lc.set_linewidth(0.6)
        self.ax.add_collection3d(lc)

        margin = 1.1
        mid_x, mid_y, mid_z = (xs.min()+xs.max())/2, (ys.min()+ys.max())/2, (zs.min()+zs.max())/2
        r = max(xs.ptp(), ys.ptp(), zs.ptp()) / 2 * margin
        self.ax.set_xlim(mid_x - r, mid_x + r)
        self.ax.set_ylim(mid_y - r, mid_y + r)
        self.ax.set_zlim(mid_z - r, mid_z + r)
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")
        self.ax.set_title(f"Toolpath ({len(xs)} points)")

        sm = plt.cm.ScalarMappable(cmap="coolwarm", norm=norm)
        sm.set_array([])
        self.fig.colorbar(sm, ax=self.ax, label="B angle (deg)", shrink=0.6)
        self.canvas.draw()


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------

class SlicerApp:
    """Top-level tkinter GUI for toolpath4."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("toolpath4 — 4-Axis XYZB Slicer")
        self.root.geometry("1600x900")

        self.stl_path: Optional[str] = None
        self.mesh = None
        self.slicer: Optional[Slicer] = None
        self.steps: Optional[StepList] = None

        self._build_ui()

    # ---- UI construction --------------------------------------------------

    def _build_ui(self):
        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Left panel (controls)
        left = ttk.Frame(main, width=370)
        left.pack(side=tk.LEFT, fill=tk.BOTH, padx=5)
        left.pack_propagate(False)
        self._build_left_panel(left)

        # Right panel (tabs)
        right = ttk.Frame(main)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        self._build_right_panel(right)

    def _build_left_panel(self, parent: ttk.Frame):
        # --- File ---
        ff = ttk.LabelFrame(parent, text="1. Load Model", padding=10)
        ff.pack(fill=tk.X, pady=5)
        self.file_label = ttk.Label(ff, text="No file selected", foreground="red",
                                    wraplength=330)
        self.file_label.pack(fill=tk.X)
        ttk.Button(ff, text="Browse STL File", command=self._browse).pack(fill=tk.X, pady=5)

        # --- Model info ---
        mf = ttk.LabelFrame(parent, text="Model Info", padding=10)
        mf.pack(fill=tk.X, pady=5)
        self.info_text = tk.Text(mf, height=6, width=40, state=tk.DISABLED, wrap=tk.WORD)
        self.info_text.pack(fill=tk.BOTH)

        # --- Settings ---
        sf = ttk.LabelFrame(parent, text="2. Settings", padding=10)
        sf.pack(fill=tk.X, pady=5)
        row = 0

        def _spin(label, var, from_, to_, inc, r):
            ttk.Label(sf, text=label).grid(row=r, column=0, sticky=tk.W, pady=2)
            sb = ttk.Spinbox(sf, from_=from_, to=to_, increment=inc,
                             textvariable=var, width=10)
            sb.grid(row=r, column=1, sticky=tk.W, padx=4)

        self.layer_height_var = tk.DoubleVar(value=0.20)
        _spin("Layer Height (mm):", self.layer_height_var, 0.05, 0.50, 0.05, row); row += 1

        self.perimeter_var = tk.IntVar(value=2)
        _spin("Perimeters:", self.perimeter_var, 1, 5, 1, row); row += 1

        self.infill_var = tk.DoubleVar(value=0.20)
        _spin("Infill Density:", self.infill_var, 0.0, 1.0, 0.05, row); row += 1

        self.nozzle_temp_var = tk.IntVar(value=200)
        _spin("Nozzle Temp (C):", self.nozzle_temp_var, 150, 300, 5, row); row += 1

        self.bed_temp_var = tk.IntVar(value=60)
        _spin("Bed Temp (C):", self.bed_temp_var, 20, 120, 5, row); row += 1

        ttk.Label(sf, text="Transition:").grid(row=row, column=0, sticky=tk.W, pady=2)
        self.strategy_var = tk.StringVar(value="smooth")
        combo = ttk.Combobox(sf, textvariable=self.strategy_var,
                             values=["linear", "smooth", "adaptive"],
                             width=12, state="readonly")
        combo.grid(row=row, column=1, sticky=tk.W, padx=4)
        row += 1

        # --- Generate ---
        gf = ttk.LabelFrame(parent, text="3. Generate", padding=10)
        gf.pack(fill=tk.X, pady=5)

        self.slice_btn = ttk.Button(gf, text="SLICE", command=self._slice,
                                    state=tk.DISABLED)
        self.slice_btn.pack(fill=tk.X, pady=3)

        self.export_btn = ttk.Button(gf, text="EXPORT G-CODE",
                                     command=self._export_gcode, state=tk.DISABLED)
        self.export_btn.pack(fill=tk.X, pady=3)

        # --- Status ---
        self.status_label = ttk.Label(parent, text="Ready", foreground="blue",
                                      wraplength=330)
        self.status_label.pack(fill=tk.X, pady=5)
        self.progress = ttk.Progressbar(parent, mode="indeterminate")
        self.progress.pack(fill=tk.X, pady=2)

        # --- Stats ---
        self.stats_text = tk.Text(parent, height=7, width=40, state=tk.DISABLED,
                                  wrap=tk.WORD, font=("Consolas", 9))
        self.stats_text.pack(fill=tk.BOTH, expand=True, pady=5)

    def _build_right_panel(self, parent: ttk.Frame):
        nb = ttk.Notebook(parent)
        nb.pack(fill=tk.BOTH, expand=True)

        # Tab 1: 3-D model
        tab_model = ttk.Frame(nb)
        nb.add(tab_model, text="3D Model")
        self.model_viewer = ModelViewer(tab_model)

        # Tab 2: B-angle schedule
        tab_bangle = ttk.Frame(nb)
        nb.add(tab_bangle, text="B-Angle Schedule")
        self.bangle_chart = BAngleChart(tab_bangle)

        # Tab 3: Toolpath preview
        tab_toolpath = ttk.Frame(nb)
        nb.add(tab_toolpath, text="Toolpath Preview")
        self.toolpath_viewer = ToolpathViewer(tab_toolpath)

    # ---- Actions ----------------------------------------------------------

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

        # Populate info
        bounds = self.mesh.bounds
        height = bounds[1][2] - bounds[0][2]
        layers_est = int(np.ceil(height / self.layer_height_var.get()))
        info = (
            f"File: {Path(path).name}\n"
            f"Faces: {len(self.mesh.faces)}\n"
            f"X: {bounds[0][0]:.1f} – {bounds[1][0]:.1f} mm\n"
            f"Y: {bounds[0][1]:.1f} – {bounds[1][1]:.1f} mm\n"
            f"Z: {bounds[0][2]:.1f} – {bounds[1][2]:.1f} mm\n"
            f"Height: {height:.1f} mm   (~{layers_est} layers)\n"
            f"Watertight: {self.mesh.is_watertight}"
        )
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete("1.0", tk.END)
        self.info_text.insert("1.0", info)
        self.info_text.config(state=tk.DISABLED)

        self.slice_btn.config(state=tk.NORMAL)
        self.status_label.config(text="Ready to slice", foreground="blue")

    def _build_config(self) -> PrinterConfig:
        """Build a PrinterConfig from the current widget values."""
        return PrinterConfig(
            layer_height=self.layer_height_var.get(),
            perimeter_count=self.perimeter_var.get(),
            infill_density=self.infill_var.get(),
            nozzle_temp=self.nozzle_temp_var.get(),
            bed_temp=self.bed_temp_var.get(),
        )

    def _slice(self):
        if not self.stl_path:
            return
        self.slice_btn.config(state=tk.DISABLED)
        self.export_btn.config(state=tk.DISABLED)
        self.status_label.config(text="Slicing... please wait", foreground="orange")
        self.progress.start(15)

        cfg = self._build_config()
        strategy = self.strategy_var.get()

        def _worker():
            try:
                slicer = Slicer(config=cfg)
                steps = slicer.process(self.stl_path, transition=strategy)
                self.root.after(0, self._on_slice_done, slicer, steps, None)
            except Exception as exc:
                self.root.after(0, self._on_slice_done, None, None, exc)

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    def _on_slice_done(self, slicer: Optional[Slicer], steps: Optional[StepList],
                       error: Optional[Exception]):
        self.progress.stop()
        if error is not None:
            self.status_label.config(text=f"Error: {error}", foreground="red")
            self.slice_btn.config(state=tk.NORMAL)
            messagebox.showerror("Slicing Failed", str(error))
            return

        self.slicer = slicer
        self.steps = steps

        # Update charts
        self.bangle_chart.update(slicer)
        self.toolpath_viewer.update(steps)

        # Stats
        cfg = self._build_config()
        stats = dry_run(steps, cfg)
        summary = slicer.rotation_schedule_summary()
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
        messagebox.showinfo("Exported", f"G-code saved to:\n{path}\n\n{lines} lines")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    root = tk.Tk()
    app = SlicerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
