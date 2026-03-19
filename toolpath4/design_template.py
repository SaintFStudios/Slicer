"""
toolpath4.design_template — Example 4-axis design and slicing pipeline.

Demonstrates:
  1. A procedural helix with oscillating B.
  2. Slicing a simple overhang STL (generated procedurally).
  3. Lower layers at B=90 then switching to B=45 / B=35 near overhangs.
  4. Preview and G-code export.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Optional

import numpy as np

from toolpath4.config import PrinterConfig, default_config
from toolpath4.state import ExtrusionMode, Move, State, StateChange, StepList
from toolpath4.geometry import helix, oscillating_b, circle
from toolpath4.compiler import compile_gcode, dry_run
from toolpath4.preview import preview_matplotlib
from toolpath4.slicer import Slicer


# ---------------------------------------------------------------------------
# 1) Procedural helix with oscillating B
# ---------------------------------------------------------------------------

def demo_helix_oscillating_b(
    config: Optional[PrinterConfig] = None,
    output_dir: str = "output",
) -> StepList:
    """Create a helix whose B angle oscillates sinusoidally.

    Returns the StepList and exports G-code + preview image.
    """
    cfg = config or default_config()
    os.makedirs(output_dir, exist_ok=True)

    # Generate a helix: 20 mm radius, 40 mm tall, 10 turns
    h = helix(
        center=(150.0, 150.0),
        radius=20.0,
        z_start=0.3,
        z_end=40.0,
        turns=10,
        b=90.0,
        points_per_turn=80,
        feedrate=cfg.max_feedrate_xyz * 0.5,
    )

    # Overlay oscillating B: sweep between 35° and 90°, 3 full cycles
    h_osc = oscillating_b(
        start_b=35.0,
        end_b=90.0,
        frequency=3.0,
        phase=0.0,
        path=h,
    )

    steps = StepList()
    steps.append(StateChange.set_nozzle_temp(cfg.nozzle_temp))
    steps.append(StateChange.set_bed_temp(cfg.bed_temp))
    steps.append(StateChange.make_comment("Helix with oscillating B demo"))
    steps.extend(h_osc)

    # Stats
    stats = dry_run(steps, cfg)
    print(f"\n=== Helix Demo Stats ===")
    print(f"  Path length : {stats.total_path_length_mm:.1f} mm")
    print(f"  Extrusion   : {stats.total_extrusion_mm:.2f} mm filament")
    print(f"  B range     : [{stats.b_min_deg:.1f}°, {stats.b_max_deg:.1f}°]")
    print(f"  Moves       : {stats.move_count}")

    # Export G-code
    gcode_path = os.path.join(output_dir, "helix_demo.gcode")
    gcode = compile_gcode(steps, cfg)
    with open(gcode_path, "w") as f:
        f.write(gcode)
    print(f"  G-code saved to {gcode_path}")

    # Preview (non-blocking save)
    try:
        preview_matplotlib(
            steps, title="Helix + Oscillating B",
            show=False,
            save_path=os.path.join(output_dir, "helix_demo.png"),
        )
        print(f"  Preview saved to {output_dir}/helix_demo.png")
    except Exception as e:
        print(f"  Preview skipped: {e}")

    return steps


# ---------------------------------------------------------------------------
# 2) Generate a simple overhang test mesh procedurally
# ---------------------------------------------------------------------------

def _generate_overhang_stl(path: str) -> str:
    """Create a simple T-shaped overhang test model and save as STL.

    The model is a vertical column (20×20×30 mm) with a horizontal shelf
    (40×20×5 mm) on top — a classic overhang test.

    Returns the path.
    """
    try:
        import trimesh
    except ImportError:
        raise RuntimeError("trimesh required to generate test STL")

    # Column
    column = trimesh.creation.box(extents=[20, 20, 30])
    column.apply_translation([0, 0, 15])

    # Shelf (overhang)
    shelf = trimesh.creation.box(extents=[40, 20, 5])
    shelf.apply_translation([0, 0, 32.5])

    mesh = trimesh.util.concatenate([column, shelf])

    # Centre on bed
    mesh.apply_translation([150, 150, 0])

    mesh.export(path)
    print(f"Generated overhang test STL: {path} ({len(mesh.faces)} faces)")
    return path


# ---------------------------------------------------------------------------
# 3) Slice an overhang STL with B-angle transitions
# ---------------------------------------------------------------------------

def demo_slice_overhang(
    stl_path: Optional[str] = None,
    config: Optional[PrinterConfig] = None,
    output_dir: str = "output",
) -> StepList:
    """Slice a model, assign B angles based on overhangs, export G-code.

    If *stl_path* is None, generates a procedural overhang test model.
    """
    cfg = config or default_config()
    os.makedirs(output_dir, exist_ok=True)

    if stl_path is None:
        stl_path = os.path.join(output_dir, "overhang_test.stl")
        _generate_overhang_stl(stl_path)

    slicer = Slicer(config=cfg)
    steps = slicer.process(stl_path, transition="smooth")

    # Print rotation schedule
    print(f"\n=== Rotation Schedule ===")
    print(slicer.rotation_schedule_summary())

    # Stats
    stats = dry_run(steps, cfg)
    print(f"\n=== Slicing Stats ===")
    print(f"  Layers      : {stats.layer_count}")
    print(f"  Path length : {stats.total_path_length_mm:.1f} mm")
    print(f"  Extrusion   : {stats.total_extrusion_mm:.2f} mm filament")
    print(f"  B range     : [{stats.b_min_deg:.1f}°, {stats.b_max_deg:.1f}°]")
    print(f"  Moves       : {stats.move_count}")

    # Export G-code
    gcode_path = os.path.join(output_dir, "overhang_slice.gcode")
    gcode = compile_gcode(steps, cfg)
    with open(gcode_path, "w") as f:
        f.write(gcode)
    print(f"  G-code saved to {gcode_path}")

    # Preview
    try:
        preview_matplotlib(
            steps, title="Overhang Slice — B-angle coloured",
            show=False,
            save_path=os.path.join(output_dir, "overhang_slice.png"),
        )
        print(f"  Preview saved to {output_dir}/overhang_slice.png")
    except Exception as e:
        print(f"  Preview skipped: {e}")

    return steps


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    """Run both demos."""
    print("=" * 60)
    print("toolpath4 — Design Template Demos")
    print("=" * 60)

    cfg = default_config()

    print("\n--- Demo 1: Helix with oscillating B ---")
    demo_helix_oscillating_b(cfg)

    print("\n--- Demo 2: Overhang slice with B transitions ---")
    demo_slice_overhang(config=cfg)

    print("\nDone.")


if __name__ == "__main__":
    main()
