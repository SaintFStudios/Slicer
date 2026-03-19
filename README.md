# toolpath4 — 4-Axis (XYZB) Slicer & Toolpath Framework

A Python-first slicer for **XYZB1 4-axis 3D printing**. Loads an STL, slices it, plans B-axis rotation based on overhang analysis, generates explicit toolpaths, and compiles to Duet/RepRap G-code — all locally with no cloud APIs.

**Target hardware:** Ender 3 Neo Max + FYSETC Big Dipper (Duet3) + Y-axis rotary nozzle (B-axis)

---

## Install

```bash
pip install numpy trimesh shapely scipy networkx matplotlib
# Optional:
pip install plotly pytest
```

No cloud services or sign-in required.

---

## Usage

### GUI

```bash
python -m toolpath4
```

The GUI provides:

| Tab | Purpose |
|-----|---------|
| **3D Model** | Load and inspect STL meshes (rotate/zoom) |
| **B-Angle Schedule** | View per-layer B angle and overhang scores after slicing |
| **Toolpath Preview** | Colour-coded 3D toolpath visualisation |

**Workflow:**
1. Click **Browse STL File** and select your model
2. Adjust settings (layer height, infill, temps, transition strategy)
3. Click **SLICE** — the slicer analyses overhangs and assigns B angles
4. Inspect the B-angle chart and toolpath preview
5. Click **EXPORT G-CODE** and save

### Scripting / CLI

```python
from toolpath4 import Slicer, PrinterConfig, compile_gcode

cfg = PrinterConfig(layer_height=0.2, nozzle_temp=200)
slicer = Slicer(config=cfg)
steps = slicer.process("model.stl", transition="smooth")

gcode = compile_gcode(steps, cfg)
with open("output.gcode", "w") as f:
    f.write(gcode)
```

Run the built-in demos:

```bash
python -m toolpath4.design_template
```

### Jupyter Notebook

```bash
jupyter notebook examples/notebook_xyzb_demo.ipynb
```

---

## Package Structure

```
toolpath4/
  __init__.py          Public API
  config.py            Printer configuration (all XYZB1 defaults)
  state.py             Move, StateChange, StepList data model
  geometry.py          Primitives: polyline, circle, helix, oscillating_b
  kinematics.py        Tip-to-pivot XYZB transforms
  compiler.py          Step stream -> G-code (Duet/RepRap flavour)
  preview.py           Matplotlib / Plotly 3D visualisation
  slicer.py            Full pipeline: STL -> layers -> toolpath
  gui.py               Tkinter GUI
  design_template.py   Demo scripts
examples/
  notebook_xyzb_demo.ipynb
tests/
  test_kinematics.py
  test_slicing.py
  test_compiler.py
```

---

## Architecture

### Data Model (inspired by FullControl)

The toolpath is a flat list of **Step** objects:

- **Move** — target position `(x, y, z, b)` + `State` (feedrate, extrusion mode, temps)
- **StateChange** — set temperature, fan, retract/unretract

No global mutable state. Every Move carries its own State snapshot.

### Slicing Pipeline

1. **load_mesh** — import STL via trimesh, validate, repair
2. **slice_mesh_along_z** — `trimesh.section_multiplane` -> 2D polygons
3. **perimeter_generation** — repeated shapely buffer -> perimeter polylines
4. **infill_generation** — grid lines clipped to inner polygon
5. **overhang_analysis** — per-layer overhang score from face normals
6. **B-angle planning** — threshold-based angle assignment + transitions
7. **Toolpath emission** — perimeters + infill + travels -> Step stream

### Kinematics

Coordinates represent the **nozzle tip** position. The printer drives axes to the **B-axis pivot** (rotation centre). The transform uses rotation about Y:

```
R_y(B) = [[cos B, 0, sin B], [0, 1, 0], [-sin B, 0, cos B]]

pivot = tip + R_y(B) * [0, 0, b_offset_z]
tip   = pivot - R_y(B) * [0, 0, b_offset_z]
```

`tip_to_pivot()` and `pivot_to_tip()` handle this. The compiler applies the transform before emitting G-code.

---

## Configuration

All defaults live in `PrinterConfig` (see `toolpath4/config.py`). Key groups:

| Group | Examples |
|-------|---------|
| Build volume | `bed_x_max=300`, `bed_z_max=320` |
| Printer geometry | `b_offset_z=35.0`, `toolhead_radius=25.0` |
| B-axis limits | `b_min_deg=-120`, `b_max_deg=+120` |
| Motion | `max_feedrate_xyz=6000`, `max_accel_xyz=1500` |
| Extrusion | `nozzle_diameter=0.40`, `layer_height=0.20`, `infill_density=0.20` |
| B-axis drive | `b_gear_ratio=27`, `microsteps=16`, `motor_steps_per_rev=200` |

Override any field: `PrinterConfig(layer_height=0.3, infill_density=0.15)`

---

## B-Angle Strategies

| Strategy | Behaviour |
|----------|-----------|
| **linear** | Ramp B linearly across transition layers |
| **smooth** | S-curve (Hermite smoothstep) for gradual transitions |
| **adaptive** | Overhang score with hysteresis to avoid oscillation |

Candidate angles: `[90, 45, 35]` degrees. The planner escalates from 90 (straight down) to 45 or 35 as overhang severity increases.

---

## Duet3 RRF Configuration

Example `config.g` snippet for the B-axis:

```gcode
M584 X0 Y1 Z2 B3        ; B on drive 3
M92 B240                 ; Steps/degree (= 200 * 16 * 27 / 360)
M203 B60                 ; Max speed deg/s
M201 B300                ; Acceleration deg/s^2
M574 B0 S0               ; No endstop for B
```

Tune `M92` based on your actual gear ratio and microstepping.

---

## Running Tests

```bash
pytest tests/ -v
```

All tests run offline — no network, no STL files on disk (tests generate geometry procedurally).

---

## References

- [FullControl](https://github.com/FullControlXYZ/fullcontrol) — stream-of-points concept
- [PrusaSlicer](https://github.com/prusa3d/PrusaSlicer) — engine architecture inspiration
- [trimesh](https://trimesh.org/) — mesh loading and slicing
- [Duet3D](https://www.duet3d.com/) — RepRapFirmware documentation

---

## License

Open source. Concepts inspired by PrusaSlicer (AGPL-3.0) and FullControl (GPL-3.0) but no verbatim code is included from either project.
