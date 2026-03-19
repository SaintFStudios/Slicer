# toolpath4 — 4-Axis (XYZB) Toolpath & Slicing Framework

A Python-first, local-only framework for XYZB1 4-axis 3D printing.

## Quick Start

```bash
pip install numpy trimesh shapely matplotlib
# Optional: pip install plotly pytest

# Run the design template demos
python -m toolpath4.design_template

# Run tests
pytest tests/ -v
```

## Package Structure

```
toolpath4/
  __init__.py          # Public API
  config.py            # Printer config with all XYZB1 defaults
  state.py             # Move, StateChange, StepList data model
  geometry.py          # Primitives: polyline, circle, helix, oscillating_b
  kinematics.py        # tip↔pivot XYZB transforms
  compiler.py          # Step stream → G-code (Duet/RepRap)
  preview.py           # 3D matplotlib/plotly visualisation
  slicer.py            # Full pipeline: STL → layers → perimeters → infill → steps
  design_template.py   # Demo scripts
examples/
  notebook_xyzb_demo.ipynb   # Jupyter notebook walkthrough
tests/
  test_kinematics.py
  test_slicing.py
  test_compiler.py
```

## Architecture

Inspired by PrusaSlicer's "core engine + wrappers + tests" layout and
FullControl's "stream of points + state" data model. No verbatim code from
either project.

The core representation is a `StepList` — a flat list of `Move` and
`StateChange` objects. Every toolpath decision is explicit geometry.

## Axes

- **X, Y, Z**: Linear axes (mm)
- **B**: Rotary axis — nozzle rotates about the machine Y-axis

## Key Concepts

- **Tip coordinates**: Where the nozzle tip touches the part (what the slicer works with)
- **Pivot coordinates**: Where the B-axis rotation centre is (what the machine drives to)
- `tip_to_pivot()` / `pivot_to_tip()` handle the kinematic transform
