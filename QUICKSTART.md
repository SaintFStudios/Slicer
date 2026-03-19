# toolpath4 — Quick Start

## 1. Install

```bash
pip install numpy trimesh shapely scipy networkx matplotlib
```

## 2. Launch the GUI

```bash
python -m toolpath4
```

1. **Browse** an STL file
2. Adjust layer height, infill, temps, and transition strategy
3. Click **SLICE**
4. Inspect the B-angle chart and toolpath preview
5. Click **EXPORT G-CODE**

## 3. Or script it

```python
from toolpath4 import Slicer, PrinterConfig, compile_gcode

cfg = PrinterConfig(layer_height=0.2)
slicer = Slicer(config=cfg)
steps = slicer.process("model.stl", transition="smooth")

with open("output.gcode", "w") as f:
    f.write(compile_gcode(steps, cfg))
```

## 4. Run tests

```bash
pip install pytest
pytest tests/ -v
```

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `ModuleNotFoundError: shapely` | `pip install shapely` |
| `ModuleNotFoundError: scipy` | `pip install scipy` |
| `ModuleNotFoundError: networkx` | `pip install networkx` |
| `No module named 'tkinter'` | Install Python with tkinter (default on Windows) |
| B-angle doesn't change | Model may have no overhangs — try a T-shaped test piece |

See `README.md` for full documentation.
