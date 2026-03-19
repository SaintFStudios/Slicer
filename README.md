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

- **STL Loading & Slicing** - Loads and slices 3D models into 2D layers at fixed Z heights
- **Progressive B-Axis Rotation** - Rotates extruder head gradually to minimize overhangs
- **Overhang Detection** - Analyzes geometry and identifies problematic features
- **Adaptive Rotation Strategies** - Linear, smooth (S-curve), or geometry-aware rotation
- **Toolpath Generation** - Creates wall perimeters and grid infill patterns
- **XYZB Gcode Output** - Full RepRapFirmware (Duet3) compatible gcode
- **Dual Interface** - CLI for scripting, GUI for interactive use
- **Comprehensive Configuration** - Centralized settings file (no code editing needed)
- **Detailed Statistics** - Slicing summary with rotation schedule
- **Gcode Parsing** - Analyze and visualize generated toolpaths
- **3D Visualization** - Preview models before slicing (experimental)

---

## Project Structure

```
Slicer/
├── slicer_runner.py          <- CLI entry point (run this)
├── gui_slicer.py             <- GUI interface (experimental)
├── four_axis_slicer.py       <- Core slicing engine
├── overhang_analyzer.py      <- Overhang detection & rotation optimization
├── config_4axis.py           <- Configuration parameters
├── test_setup.py             <- Setup verification
├── README.md                 <- This documentation
├── QUICKSTART.md             <- Quick reference guide
├── .vscode/                  <- VS Code configuration
└── output/                   <- Generated gcode directory
    └── output.gcode          <- Default output file
```

## Architecture

### slicer_runner.py — CLI Interface
**Main entry point** for the slicer. Provides two modes of operation:

- **Direct mode:** `python slicer_runner.py model.stl`
- **Interactive mode:** `python slicer_runner.py` (prompts for settings)

**Workflow:**
1. Load STL and validate mesh
2. Analyze overhangs (if enabled)
3. Slice into layers
4. Calculate rotation schedule
5. Generate toolpaths (walls + infill)
6. Output XYZB gcode
7. Print summary statistics

**Key functions:**
- `slice_model()` — Complete slicing workflow
- `interactive_mode()` — User prompts for custom parameters
- `main()` — Entry point dispatcher

### gui_slicer.py — GUI Interface (Experimental)
**Graphical user interface** built with Tkinter and Matplotlib. For users who prefer visual interaction.

**GcodeParser class:**
- Parses generated gcode files
- Extracts movement data (X, Y, Z, B coordinates, extrusion)
- Used for gcode visualization and verification
- Methods:
  - `parse()` — Read and parse gcode file
  - Access `movements` list for movement data

**Model3DViewer class:**
- Displays 3D model in interactive window
- Built with Matplotlib 3D plotting
- Features:
  - Rotate and zoom model
  - View all axes (X, Y, Z)
  - Inspect geometry before slicing

**GUI Application:**
- File browser to select STL
- Settings panel for layer height, max rotation, rotation strategy
- Real-time 3D model preview
- Start slicing button
- View statistics and rotation schedule
- Monitor slicing progress

**Usage:**
```bash
python gui_slicer.py
```

### four_axis_slicer.py — Core Slicing Engine
**Main slicing logic.** The heart of the project.

**FourAxisSlicer class:**
- Initialize with STL path and slicing parameters
- Key attributes:
  - `mesh` — Trimesh object
  - `bounds` — Model bounding box
  - `layers` — List of sliced layers
  - `toolpaths` — Complete toolpaths with walls and infill

**Key methods:**
- `slice()` — Slice mesh into layers with B rotation angles
- `_get_slice_at_z(z)` — Extract 2D contours at height z using plane intersection
- `_calculate_optimal_rotation()` — Determine B angle based on layer progress
- `generate_toolpaths()` — Create wall perimeters and infill patterns
- `_offset_contour()` — Offset contours inward for wall thickness
- `_generate_grid_infill()` — Generate simple grid infill pattern
- `generate_gcode()` — Convert toolpaths to XYZB gcode
- `print_summary()` — Display slicing statistics

**Output:**
- XYZB gcode file compatible with Duet3 RepRapFirmware
- Includes homing, heating, layer-by-layer movements with rotation

### overhang_analyzer.py — Overhang Detection & Optimization
**Analyzes geometry** and calculates smart rotation angles.

**OverhangAnalyzer class:**
- Analyzes mesh for overhanging surfaces
- Key methods:
  - `detect_overhang_faces()` — Find faces with normal > threshold from vertical
  - `get_overhang_points()` — Get overhang points at specific Z height
  - `calculate_optimal_b_angle()` — Compute best rotation for this layer

**AdaptiveRotationStrategy class:**
Implements three rotation progression strategies:

1. `linear_progression()` — Constant rate rotation
   - Simple: `B = (layer / total_layers) * MAX_ROTATION`
   - Predictable but not geometry-aware

2. `smooth_progression()` — S-curve (smooth step function)
   - Smooth ramp-up and plateau
   - Avoids jarring transitions
   - **Recommended default**

3. `adaptive_by_height()` — Geometry-aware rotation
   - Analyzes overhang density at each layer
   - Rotates more where overhangs are denser
   - Most effective for complex shapes
   - Slower to compute

**Helper function:**
- `calculate_rotation_schedule()` — Generate full rotation plan for all layers

### config_4axis.py — Configuration Module
**Centralized settings** for all parameters. Edit this file to customize behavior.

**Printer Settings:**
```python
NOZZLE_DIAMETER = 0.4      # mm
NOZZLE_TEMP = 200          # °C (PLA: 190-210, ABS: 220-240)
BED_TEMP = 60              # °C (PLA: 50-65, ABS: 80-110)
BED_SIZE_X, BED_SIZE_Y, BED_SIZE_Z = 300, 300, 320  # mm
```

**Slicing Parameters:**
```python
LAYER_HEIGHT = 0.2         # mm (0.1 fine detail, 0.3 fast)
FIRST_LAYER_HEIGHT = 0.3   # mm (usually thicker for adhesion)
NUM_WALLS = 2              # Perimeter count (more = stronger)
INFILL_DENSITY = 0.2       # 0.2 = 20% fill
INFILL_SPACING = 4.0       # mm between infill lines
INFILL_PATTERN = 'grid'    # Only option currently
```

**Motion Parameters:**
```python
PRINT_SPEED = 40           # mm/s (normal extrusion)
TRAVEL_SPEED = 150         # mm/s (rapid moves without extrusion)
FIRST_LAYER_SPEED = 20     # mm/s (slow first layer for adhesion)
ACCELERATION = 1000        # mm/s² (Duet default is fine)
```

**4-Axis Settings:**
```python
ROTATION_STRATEGY = 'smooth'   # 'linear', 'smooth', 'adaptive'
MAX_ROTATION = 45.0            # Maximum B rotation in degrees
OVERHANG_THRESHOLD = 45.0      # Angle threshold for overhang detection
ROTATION_CENTER_X = 150        # Center of rotation (typically bed center)
ROTATION_CENTER_Y = 150
ROTATION_SPEED = 100           # mm/s (B-axis feedrate in Duet)
DETECT_OVERHANGS = True        # Enable overhang analysis
```

**Support & Output:**
```python
SUPPORT_ENABLED = False        # Future feature
OUTPUT_GCODE_FILE = 'output.gcode'
VERBOSE = True                 # Print detailed slicing info
```

### test_setup.py — Verification Tool
**Checks dependencies** are properly installed. Run before first use:

```bash
python test_setup.py
```

Verifies:
- NumPy installation
- Trimesh availability
- Other critical imports
- Prints status and next steps

---

## Data Flow

Understanding how data moves through the system:

```
INPUT
  └─> STL File (3D geometry)
        │
        ├─────────────────────────────────────────┐
        │                                         │
        v                                         v
   [slicer_runner.py]                    [gui_slicer.py]
        │                                         │
        ├─ Load STL ────────────────────────────┬─┤
        │                                       │ │
        v                                       v │
   [four_axis_slicer.py]                       │ │
        │                                       │ │
        ├─ Load mesh (Trimesh)                 │ │
        │                                       │ │
        ├─> [overhang_analyzer.py]             │ │
        │   ├─ Detect overhangs               │ │
        │   └─ Calculate optimal B angles     │ │
        │                                       │ │
        ├─ Slice at Z heights                  │ │
        │   ├─ Extract 2D contours            │ │
        │   └─ Generate layers                │ │
        │                                       │ │
        ├─ Generate toolpaths                  │ │
        │   ├─ Wall offsets                   │ │
        │   └─ Grid infill                    │ │
        │                                       │ │
        └─────────────────────────┬────────────┘ │
                                  │              │
                                  v              │
                            Generate Gcode       │
                                  │              │
                                  v              │
                            [GcodeParser]        │
                            (gui_slicer.py)     │
                                  │              │
                                  v              │
                            Visualization      /│\
                            (3D Viewer)        / │ \
                                               │ │
                                  ┌────────────┘ │
                                  │              │
OUTPUT                            │              │
  └────────────────> output/output.gcode <──────┘
                     (XYZB RepRapFirmware)
```

**Key transformations:**
1. **STL → Mesh** — Load 3D geometry using Trimesh
2. **Mesh → Layers** — Slice with horizontal planes, extract 2D contours
3. **Layers + Geometry → Rotation Schedule** — Analyze overhangs, calculate B angles
4. **Layers + Toolpaths → Gcode** — Convert XYZ coordinates + B rotation + extrusion to gcode commands
5. **Gcode → Printer** — Transfer to Duet3 controller, execute on hardware

---

## File Reference

### Core Modules

| File | Purpose | Key Classes |
|------|---------|-------------|
| `slicer_runner.py` | CLI entry point, workflow orchestration | `slice_model()`, `interactive_mode()` |
| `gui_slicer.py` | Graphical user interface (Tkinter + Matplotlib) | `GcodeParser`, `Model3DViewer`, GUI app |
| `four_axis_slicer.py` | Core slicing engine | `FourAxisSlicer` |
| `overhang_analyzer.py` | Overhang detection & rotation optimization | `OverhangAnalyzer`, `AdaptiveRotationStrategy` |
| `config_4axis.py` | Centralized configuration settings | (Module-level constants) |
| `test_setup.py` | Dependency verification | `test_imports()` |

### Documentation Files

| File | Purpose |
|------|---------|
| `README.md` | Complete documentation (this file) |
| `QUICKSTART.md` | Quick reference guide |
| `.vscode/settings.json` | VS Code editor configuration |
| `.vscode/launch.json` | VS Code debugging configuration |

### Output Directory

| Location | Contents |
|----------|----------|
| `output/` | Generated files (created automatically) |
| `output/output.gcode` | Final XYZB gcode for printing |

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

## Usage Guide

### Method 1: Command Line (CLI)

#### Step 1: Prepare Your Model
- Export as STL (units should be mm)
- Orient so the part faces the correct direction
- Consider centering it on the bed for balanced rotation

#### Step 2: Configure Settings
Edit `config_4axis.py` for your printer and material.

#### Step 3: Slice (Direct Mode)
```bash
python slicer_runner.py model.stl
```

Or interactive mode:
```bash
python slicer_runner.py
```
(Follow prompts for layer height, max rotation, strategy, output directory)

The slicer will:
1. Load and validate the STL
2. Analyze overhangs (if enabled)
3. Generate layers with Z heights
4. Calculate B rotation schedule
5. Create wall and infill toolpaths
6. Output XYZB gcode

#### Step 4: Verify Gcode
- Check `output/output.gcode` exists
- Open in text editor to inspect (or use Cura/PrusaSlicer viewer)
- Verify B-axis commands look reasonable
- Check that rotation range matches config

#### Step 5: Load to Printer
- Transfer `output/output.gcode` to Duet3 SD card or via web interface
- Verify the B-axis moves are sensible
- Run a dry run (no filament) to check motion

#### Step 6: Print
- Start print on the Duet3 controller
- Monitor first few layers to ensure rotation is working

### Method 2: Graphical User Interface (GUI)

#### Step 1: Launch GUI
```bash
python gui_slicer.py
```

This opens a Tkinter window with:
- 3D model viewer
- Settings panel
- Slice button

#### Step 2: Load Model
- Click "Browse" to select your STL file
- Model preview appears in 3D viewer
- Inspect from all angles

#### Step 3: Configure Settings
In the GUI panel, adjust:
- Layer height
- Maximum B rotation
- Rotation strategy (linear/smooth/adaptive)
- Output directory

#### Step 4: Slice
- Click "Start Slicing"
- Progress displayed in console
- Statistics shown on completion

#### Step 5: Review Results
- View rotation schedule in GUI
- Print statistics displayed
- Check generated gcode file

#### Step 6: Send to Printer
Same as CLI method Step 5-6

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
