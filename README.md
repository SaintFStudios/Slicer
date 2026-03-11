# 4-Axis Slicer for Ender 3 Neo Max with Y-Axis Rotation

A Python-based slicer that generates **XYZB gcode** for 4-axis 3D printing with progressive extruder head rotation to avoid overhangs.

**Target setup:** Ender 3 Neo Max + FYSETC Big Dipper (Duet3 controller) + Y-axis stepper motor on X-carriage

---

## Quick Start

### 1. Install Dependencies

```bash
pip install numpy trimesh --break-system-packages
```

(Use `--break-system-packages` on Ubuntu/systems with system Python)

### 2. Run on Your Model

```bash
python slicer_runner.py your_model.stl
```

Or interactive mode:
```bash
python slicer_runner.py
```

Output gcode will be in `./output/output.gcode`

---

## Features

- **STL Loading & Slicing** - Slices 3D models into 2D layers at fixed Z heights
- **Progressive B-Axis Rotation** - Rotates extruder head gradually to minimize overhangs
- **Overhang Detection** - Analyzes geometry and identifies problematic features
- **Adaptive Rotation Strategies** - Linear, smooth (S-curve), or geometry-aware rotation
- **Toolpath Generation** - Creates wall perimeters and grid infill patterns
- **XYZB Gcode Output** - Full RepRapFirmware (Duet3) compatible gcode

---

## Architecture

```
slicer_runner.py           <- Main entry point (run this)
├── four_axis_slicer.py    <- Core slicing engine
├── overhang_analyzer.py   <- Overhang detection & rotation optimization
└── config_4axis.py        <- Configuration parameters
```

### Core Classes

**FourAxisSlicer**
- Loads STL and slices into layers
- Calculates optimal B rotation per layer
- Generates toolpaths (walls + infill)
- Outputs XYZB gcode

**OverhangAnalyzer**
- Detects overhanging faces in mesh
- Identifies points that need rotation relief
- Calculates optimal B angle to minimize overhangs

**AdaptiveRotationStrategy**
- Multiple rotation progression methods
- Linear, smooth (S-curve), or adaptive-by-geometry

---

## Configuration

Edit `config_4axis.py` to customize:

### Printer Settings
```python
NOZZLE_DIAMETER = 0.4      # mm
NOZZLE_TEMP = 200          # °C (PLA: 190-210)
BED_TEMP = 60              # °C (PLA: 50-65)
```

### Slicing Parameters
```python
LAYER_HEIGHT = 0.2         # mm (0.1 fine, 0.3 fast)
NUM_WALLS = 2              # Perimeter count
INFILL_DENSITY = 0.2       # 20% fill
INFILL_SPACING = 4.0       # mm between lines
```

### 4-Axis Settings
```python
ROTATION_STRATEGY = 'smooth'   # 'linear', 'smooth', 'adaptive'
MAX_ROTATION = 45.0            # Degrees (maximum rotation angle)
OVERHANG_THRESHOLD = 45.0      # Degrees from vertical
ROTATION_SPEED = 100           # mm/s (B-axis feedrate)
```

---

## Rotation Strategies

### 1. **Linear**
Rotates at constant rate from 0° to MAX_ROTATION across all layers.
- Simplest, predictable
- May not adapt to geometry

### 2. **Smooth (S-Curve)**
Uses smooth step function for rotation progression.
- Avoids jarring transitions
- Good default choice

### 3. **Adaptive**
Analyzes overhang density per layer and rotates more where needed.
- Most effective for complex shapes
- Slower to compute (requires mesh analysis per layer)

---

## Understanding the Output Gcode

The generated gcode includes:

```gcode
; Start sequence
G28                    ; Home all axes
G29                    ; Auto-leveling
M104 S200              ; Set extruder temp
M109 S200              ; Wait for extruder

; Layer 0
G1 Z0.3 F150           ; Move to layer height
G1 B0.0 F100           ; Set B rotation to 0°
G1 X10 Y10 F150        ; Travel to start
G1 X20 Y20 E1.2 F40    ; Extrude wall segment

; Layer 1
G1 Z0.5 F150           ; Move to next layer
G1 B2.5 F100           ; Rotate to 2.5°
G1 X30 Y30 E2.1 F40    ; Extrude with new rotation

... (more layers) ...

; End sequence
M104 S0                ; Turn off extruder
G28 X0 Y0              ; Home XY
M84                    ; Disable motors
```

**Key commands:**
- `G1 Z` - Move to layer height
- `G1 B` - Rotate extruder around Y-axis
- `G1 X Y ... E` - Move and extrude
- `G1 X Y ... F` - Feedrate in mm/s

---

## Workflow

### Step 1: Prepare Your Model
- Export as STL (units should be mm)
- Orient so the part faces the correct direction
- Consider centering it on the bed for balanced rotation

### Step 2: Configure Settings
Edit `config_4axis.py` for your printer and material.

### Step 3: Slice
```bash
python slicer_runner.py model.stl
```

The slicer will:
1. Load the STL
2. Analyze overhangs
3. Generate layers
4. Calculate rotation schedule
5. Create toolpaths
6. Output gcode

### Step 4: Load Gcode
- Transfer `output/output.gcode` to your Duet3 controller (via SD card or web interface)
- Verify the B-axis moves are sensible
- Run a dry run to check motion

### Step 5: Print
- Start print on the Duet3 controller
- Monitor first few layers to ensure rotation is working

---

## Customization Guide

### Changing Rotation Speed
In `config_4axis.py`:
```python
ROTATION_SPEED = 100  # Lower = slower rotation (safer)
```

### Enabling Specific Rotation Range
Modify the `_calculate_optimal_rotation()` method in `four_axis_slicer.py`:
```python
def _calculate_optimal_rotation(self, layer_idx, total_layers):
    max_rotation = 45.0  # Change this
    progress = layer_idx / (total_layers - 1)
    return progress * max_rotation
```

### Adding Support Generation
Extend `FourAxisSlicer.generate_toolpaths()` to detect layers needing support and generate support columns.

### Custom Infill Patterns
Modify `_generate_grid_infill()` to implement:
- Hexagonal infill
- Gyroid (curved infill)
- Concentric circles

Example for hexagonal:
```python
def _generate_hex_infill(self, contours, spacing):
    # Calculate hex grid points
    # Filter points inside contours
    # Return as paths
    pass
```

---

## Duet3 RRF Configuration

Your Duet3 needs a config file that defines the B-axis. Example `config.g`:

```gcode
; Axes
M669 K1                  ; CoreXY kinematics (if using)
M584 X0 Y1 Z2 B3        ; Drive assignments (B on drive 3)

; B-axis (tool rotation)
M92 B80                  ; Steps per mm (tune for your stepper)
M203 B100                ; Maximum speed (mm/s)
M201 B1000               ; Acceleration (mm/s²)
M204 P1000 T1000         ; Default acceleration
M207 S0 Z0               ; Firmware retraction (optional)

; Homing
M574 X1 S1 P"io0.in"    ; X home on min
M574 Y1 S1 P"io1.in"    ; Y home on min
M574 Z1 S1 P"io2.in"    ; Z home on min
; Note: B-axis may not have a home switch

; Leveling
M558 P9 C"io3.in" H5 F300 T4000  ; Probe (if using)
G29 S0                           ; Leveling off initially
```

Adjust `M92 B80` based on your stepper motor spec.

---

## Troubleshooting

### Gcode won't load on Duet3
- Check syntax: open in text editor
- Verify B values are within your configured range
- Test with a simpler gcode file first

### Rotation too fast/slow
- Adjust `ROTATION_SPEED` in config
- Reduce `MAX_ROTATION` for more gradual changes

### Overhangs not being detected
- Lower `OVERHANG_THRESHOLD` (more sensitive)
- Increase `MAX_ROTATION` for more aggressive rotation
- Switch to `'adaptive'` strategy

### Toolpaths look wrong
- Check that contours are being extracted correctly (visualize with Cura)
- Verify layer height isn't too aggressive
- Check that infill spacing is reasonable

### Model height is wrong
- Ensure your STL is in millimeters (not inches/cm)
- Check that the model isn't oriented upside-down

---

## Advanced Features (Future)

- [ ] Support generation for bridging gaps
- [ ] Automatic wall count based on geometry
- [ ] Multi-material toolpath merging
- [ ] Visualization of rotation schedule
- [ ] Pressure advance tuning
- [ ] Variable layer heights
- [ ] Spiral/vase mode

---

## References

- **FullControl:** https://github.com/FullControlXYZ/fullcontrol
- **FullControl Multiaxis:** https://github.com/FullControlXYZ/multiaxis
- **Trimesh Docs:** https://trimesh.org/
- **Duet3D:** https://www.duet3d.com/
- **RepRapFirmware:** https://github.com/Duet3D/RepRapFirmware

---

## License

Open-source. Use, modify, and distribute freely.

---

## Questions / Issues?

Refer to the FullControl documentation and Duet3D forums for specific hardware/firmware questions.

Good luck with your 4-axis prints!
