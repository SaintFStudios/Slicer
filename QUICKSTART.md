# 4-Axis Slicer - Quick Start Guide

## What You Have

A complete Python-based slicer for **Ender 3 Neo Max + Duet3 + Y-axis (B-axis) extruder rotation**.

### Files:
- `four_axis_slicer.py` - Core slicing engine (loads STL, slices layers, generates gcode)
- `overhang_analyzer.py` - Overhang detection and smart rotation calculation
- `config_4axis.py` - Configuration (edit this for your printer)
- `slicer_runner.py` - Main script (run this to slice)
- `test_setup.py` - Dependency checker (run this first)
- `README.md` - Full documentation

---

## Step 1: Install Dependencies (2 minutes)

Open terminal and run:

```bash
pip install numpy trimesh --break-system-packages
```

(The `--break-system-packages` flag is needed on Ubuntu/system Python environments)

---

## Step 2: Verify Setup (2 minutes)

```bash
python test_setup.py
```

This will:
- Check that numpy and trimesh are installed
- Test that all slicer modules import correctly
- Create and slice a test cube
- Generate a test gcode file

**Expected output:**
```
✓ numpy              - NumPy (numerical computing)
✓ trimesh           - Trimesh (3D geometry)
Slicing test cube... ✓ (6 layers)
Generating toolpaths... ✓
Generating gcode... ✓
All checks passed!
```

---

## Step 3: Configure for Your Printer (5 minutes)

Edit `config_4axis.py` and customize:

```python
# Temperature (PLA typical)
NOZZLE_TEMP = 200      # Adjust for your filament
BED_TEMP = 60

# Slicing
LAYER_HEIGHT = 0.2     # 0.1 = fine detail, 0.3 = speed
NUM_WALLS = 2          # Perimeter count
INFILL_DENSITY = 0.2   # 20% fill

# 4-Axis rotation
MAX_ROTATION = 45.0    # Degrees (higher = more rotation)
ROTATION_STRATEGY = 'smooth'  # 'linear', 'smooth', or 'adaptive'
```

---

## Step 4: Slice Your Model (5 minutes)

Place your model in the same folder and run:

```bash
python slicer_runner.py your_model.stl
```

Or for interactive mode (it'll ask you questions):

```bash
python slicer_runner.py
```

**Output:** `output/output.gcode`

---

## Step 5: Load Gcode on Duet3 (2 minutes)

1. Connect to Duet3 web interface (usually http://duet.local or your printer's IP)
2. Upload `output/output.gcode` via the "Upload File" button
3. Select and start the print

---

## Example Walkthrough

Let's say you have a model called `vase.stl`:

```bash
$ python slicer_runner.py vase.stl
```

You'll see:

```
============================================================
4-AXIS SLICER - Ender 3 Neo Max with Y-Axis Rotation
============================================================

Loading STL: vase.stl
Model bounds: X=50.0-250.0 mm
              Y=40.0-260.0 mm
              Z=0.0-150.0 mm
Model height: 150.0 mm

Analyzing overhangs...
Overhang faces detected: 247 / 5120

Slicing with layer height: 0.2 mm
Generated 750 layers

Rotation strategy: smooth
Max rotation: 45.0°

Generating toolpaths...
Total toolpath segments: 2250

Generating XYZB gcode...
Gcode written to: output/output.gcode

=== 4-Axis Slicer Summary ===
STL: vase.stl
Model height: 150.00 mm
Layers: 750
...

B-Axis Rotation Schedule (sample):
  Layer   0 (Z=0.0mm): B=0.0°
  Layer 150 (Z=30.0mm): B=9.0°
  Layer 300 (Z=60.0mm): B=18.0°
  ...
```

Then load `output/output.gcode` on your Duet3!

---

## Understanding the Gcode Output

Your gcode will include commands like:

```gcode
G1 Z0.3 F150        ; Move to layer height
G1 B0.0 F100        ; Set B rotation to 0°
G1 X10 Y10 F150     ; Move to start position
G1 X20 Y20 E1.2 F40 ; Extrude a wall
```

The **B** commands control your Y-axis stepper motor rotation.

---

## Troubleshooting

### "ModuleNotFoundError: No module named 'trimesh'"
Solution:
```bash
pip install trimesh --break-system-packages
```

### "Error: file not found"
- Make sure your STL is in the same folder as the scripts
- Or use full path: `python slicer_runner.py /path/to/model.stl`

### "Gcode looks weird or empty"
- Check that your STL is in millimeters
- Try the test cube first: `python test_setup.py`
- Verify contours are being sliced (see README for debugging)

### "Rotation is too fast/slow"
Edit `config_4axis.py`:
```python
ROTATION_SPEED = 100     # Lower = slower rotation
MAX_ROTATION = 45.0      # Lower = less aggressive rotation
```

---

## Next Steps

1. **Customize for your part:** Edit `config_4axis.py` based on what you're printing
2. **Try different rotation strategies:** 'linear', 'smooth', 'adaptive'
3. **Monitor first print:** Watch the first few layers to ensure B-axis moves correctly
4. **Tune extrusion:** Adjust `EXTRUSION_MULTIPLIER` based on print quality

---

## Need Help?

- See **README.md** for full documentation
- Check Duet3D forums for controller-specific questions
- Reference FullControl for multiaxis slicing concepts: https://github.com/FullControlXYZ/multiaxis

---

## Key Concepts

**B-Axis:** Rotates the extruder around the Y-axis (horizontal, perpendicular to bed travel)

**Progressive Rotation:** As layers increase in height, the part rotates more, naturally tilting away from the print direction to avoid overhangs

**Non-planar Slicing:** Each layer's toolpath accounts for the rotation, so geometry deforms gracefully

**XYZB Gcode:** Standard Cartesian motion (X, Y, Z) plus rotational motion (B)

---

Good luck! This is cutting-edge 3D printing—your prints are going to look awesome.
