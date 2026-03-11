"""
4-Axis Slicer Example Runner
Demonstrates complete workflow: load STL → slice → generate toolpaths → output gcode

Usage:
    python slicer_runner.py your_model.stl
    
    or modify this file to set your own parameters
"""

import sys
import numpy as np
from pathlib import Path

# Import slicer modules
from four_axis_slicer import FourAxisSlicer
from overhang_analyzer import OverhangAnalyzer, AdaptiveRotationStrategy
import config_4axis as cfg


def slice_model(stl_path: str, output_dir: str = './output'):
    """
    Complete slicing workflow
    
    Args:
        stl_path: Path to input STL file
        output_dir: Directory for output files
    """
    
    # Validate input
    if not Path(stl_path).exists():
        print(f"Error: {stl_path} not found")
        return False
    
    # Create output directory
    Path(output_dir).mkdir(exist_ok=True)
    
    print("\n" + "="*60)
    print("4-AXIS SLICER - Ender 3 Neo Max with Y-Axis Rotation")
    print("="*60 + "\n")
    
    # Initialize slicer with config parameters
    print(f"Loading STL: {stl_path}")
    slicer = FourAxisSlicer(
        stl_path,
        layer_height=cfg.LAYER_HEIGHT,
        nozzle_diameter=cfg.NOZZLE_DIAMETER,
        print_speed=cfg.PRINT_SPEED
    )
    
    # Print model info
    print(f"Model bounds: X={slicer.bounds[0][0]:.1f}-{slicer.bounds[1][0]:.1f} mm")
    print(f"              Y={slicer.bounds[0][1]:.1f}-{slicer.bounds[1][1]:.1f} mm")
    print(f"              Z={slicer.bounds[0][2]:.1f}-{slicer.bounds[1][2]:.1f} mm")
    print(f"Model height: {slicer.height:.1f} mm")
    print()
    
    # Analyze overhangs if enabled
    if cfg.DETECT_OVERHANGS:
        print("Analyzing overhangs...")
        analyzer = OverhangAnalyzer(slicer.mesh, cfg.OVERHANG_THRESHOLD)
        overhang_faces = analyzer.detect_overhang_faces()
        print(f"Overhang faces detected: {np.sum(overhang_faces)} / {len(overhang_faces)}")
        print()
    
    # Slice model
    print(f"Slicing with layer height: {cfg.LAYER_HEIGHT} mm")
    layers = slicer.slice()
    print(f"Generated {len(layers)} layers")
    print()
    
    # Calculate rotation schedule
    print(f"Rotation strategy: {cfg.ROTATION_STRATEGY}")
    print(f"Max rotation: {cfg.MAX_ROTATION}°")
    
    if cfg.ROTATION_STRATEGY == 'adaptive' and cfg.DETECT_OVERHANGS:
        print("Calculating adaptive rotation angles...")
        rotation_angles = AdaptiveRotationStrategy.adaptive_by_height(
            0, len(layers), slicer.mesh, cfg.MAX_ROTATION
        )
        # Note: this calculates per-layer in the slicer automatically
    else:
        print("Using predefined rotation schedule")
    
    print()
    
    # Generate toolpaths
    print("Generating toolpaths...")
    print(f"  - Wall count: {cfg.NUM_WALLS}")
    print(f"  - Infill density: {cfg.INFILL_DENSITY*100:.0f}%")
    print(f"  - Infill spacing: {cfg.INFILL_SPACING} mm")
    toolpaths = slicer.generate_toolpaths()
    print(f"Total toolpath segments: {sum(len(tp['paths']) for tp in toolpaths)}")
    print()
    
    # Generate gcode
    print("Generating XYZB gcode...")
    output_gcode = f"{output_dir}/{cfg.OUTPUT_GCODE_FILE}"
    gcode = slicer.generate_gcode(output_gcode)
    print(f"Gcode written to: {output_gcode}")
    print()
    
    # Print slicing summary
    slicer.print_summary()
    
    # Print rotation schedule
    print(f"\nB-Axis Rotation Schedule (sample):")
    for layer in layers[::max(1, len(layers)//5)]:  # Print every ~20%
        print(f"  Layer {layer['index']:3d} (Z={layer['z']:.1f}mm): "
              f"B={layer['b_angle']:.1f}°")
    
    # Print gcode stats
    gcode_lines = gcode.split('\n')
    move_count = sum(1 for line in gcode_lines if line.startswith('G1'))
    print(f"\nGcode Statistics:")
    print(f"  Total lines: {len(gcode_lines)}")
    print(f"  Move commands (G1): {move_count}")
    
    print("\n" + "="*60)
    print("Slicing complete!")
    print("="*60 + "\n")
    
    return True


def interactive_mode():
    """Interactive mode to customize parameters"""
    
    print("\n4-Axis Slicer - Interactive Configuration\n")
    
    # Get STL file
    stl_file = input("Enter STL filename (or press Enter for 'model.stl'): ").strip()
    if not stl_file:
        stl_file = 'model.stl'
    
    # Get layer height
    layer_h = input(f"Layer height in mm (default {cfg.LAYER_HEIGHT}): ").strip()
    if layer_h:
        cfg.LAYER_HEIGHT = float(layer_h)
    
    # Get max rotation
    max_rot = input(f"Max rotation in degrees (default {cfg.MAX_ROTATION}): ").strip()
    if max_rot:
        cfg.MAX_ROTATION = float(max_rot)
    
    # Get rotation strategy
    print(f"\nRotation strategy options: linear, smooth, adaptive")
    strategy = input(f"Choose strategy (default {cfg.ROTATION_STRATEGY}): ").strip()
    if strategy in ['linear', 'smooth', 'adaptive']:
        cfg.ROTATION_STRATEGY = strategy
    
    # Get output directory
    output_dir = input("Output directory (default './output'): ").strip()
    if not output_dir:
        output_dir = './output'
    
    print(f"\nStarting slice with:")
    print(f"  STL: {stl_file}")
    print(f"  Layer height: {cfg.LAYER_HEIGHT} mm")
    print(f"  Max rotation: {cfg.MAX_ROTATION}°")
    print(f"  Strategy: {cfg.ROTATION_STRATEGY}")
    print()
    
    return slice_model(stl_file, output_dir)


def main():
    """Main entry point"""
    
    if len(sys.argv) > 1:
        # Command line mode
        stl_file = sys.argv[1]
        output_dir = sys.argv[2] if len(sys.argv) > 2 else './output'
        slice_model(stl_file, output_dir)
    else:
        # Interactive mode
        try:
            interactive_mode()
        except KeyboardInterrupt:
            print("\n\nCancelled by user")
        except Exception as e:
            print(f"\nError: {e}")
            import traceback
            traceback.print_exc()


if __name__ == '__main__':
    main()
