"""
Configuration for 4-Axis Slicer
Edit these settings for your printer and material
"""

# =============================================================================
# PRINTER SETTINGS
# =============================================================================

# Bed dimensions (mm)
BED_SIZE_X = 300
BED_SIZE_Y = 300
BED_SIZE_Z = 320

# Nozzle and extrusion
NOZZLE_DIAMETER = 0.4  # mm
LINE_WIDTH = 0.48  # mm (typically 1.2x nozzle diameter)
EXTRUSION_MULTIPLIER = 1.0  # Fine-tune extrusion amount

# Hotend & bed temperatures (Celsius)
NOZZLE_TEMP = 200  # PLA: 190-210, ABS: 220-240
BED_TEMP = 60     # PLA: 50-65, ABS: 80-110

# =============================================================================
# SLICING PARAMETERS
# =============================================================================

LAYER_HEIGHT = 0.2      # mm (0.1 for fine details, 0.3 for speed)
FIRST_LAYER_HEIGHT = 0.3  # mm (usually thicker for adhesion)

# Wall/infill
NUM_WALLS = 2           # Number of perimeter walls
WALL_LINE_WIDTH = LINE_WIDTH
INFILL_PATTERN = 'grid'  # 'grid', 'line', 'gyroid' (grid is simplest)
INFILL_DENSITY = 0.2    # 0.0-1.0 (0.2 = 20%)
INFILL_SPACING = 4.0    # mm (spacing between infill lines)

# =============================================================================
# MOTION PARAMETERS
# =============================================================================

PRINT_SPEED = 40        # mm/s (normal print speed)
TRAVEL_SPEED = 150      # mm/s (rapid moves without extrusion)
FIRST_LAYER_SPEED = 20  # mm/s (slow first layer for adhesion)
ACCELERATION = 1000     # mm/s² (Duet default is fine)

# =============================================================================
# 4-AXIS ROTATION (B-AXIS) SETTINGS
# =============================================================================

# Rotation axis is Y (perpendicular to bed, rotates extruder)
# B-axis command in Duet RRF

# Rotation strategy
ROTATION_STRATEGY = 'smooth'  # 'linear', 'smooth', or 'adaptive'
MAX_ROTATION = 45.0  # degrees (maximum rotation angle)

# Overhang detection
OVERHANG_THRESHOLD = 45.0  # degrees from vertical
DETECT_OVERHANGS = True

# Rotation center (XY position relative to bed origin, typically bed center)
ROTATION_CENTER_X = BED_SIZE_X / 2
ROTATION_CENTER_Y = BED_SIZE_Y / 2

# Rotation speed
ROTATION_SPEED = 100  # mm/s (in Duet, this controls B-axis feedrate)

# =============================================================================
# Z-HEIGHT OFFSET DURING ROTATION
# =============================================================================

# As the part rotates around Y-axis, the nozzle height relative to bed changes
# Set the offset from the rotation center to nozzle/print point
Z_OFFSET_FROM_CENTER = 0.0  # mm (usually 0 if rotating around part centroid)

# =============================================================================
# SUPPORT SETTINGS
# =============================================================================

SUPPORT_ENABLED = False  # Basic support generation (future feature)
SUPPORT_ANGLE = 45.0    # Angle threshold for support placement

# =============================================================================
# OUTPUT SETTINGS
# =============================================================================

OUTPUT_GCODE_FILE = 'output.gcode'
OUTPUT_MODEL_FILE = 'output_model.stl'  # Sliced model visualization
VERBOSE = True  # Print detailed slicing info
