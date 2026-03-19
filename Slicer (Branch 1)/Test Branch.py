"""
Test script to verify dependencies and test basic functionality
Run this before slicing to ensure everything is set up correctly
"""

import sys
from pathlib import Path


def check_dependencies():
    """Check that all required packages are installed"""
    
    print("\n" + "="*60)
    print("4-AXIS SLICER - DEPENDENCY CHECK")
    print("="*60 + "\n")
    
    dependencies = {
        'numpy': 'NumPy (numerical computing)',
        'trimesh': 'Trimesh (3D geometry)',
    }
    
    missing = []
    
    for module, description in dependencies.items():
        try:
            __import__(module)
            print(f"✓ {module:15} - {description}")
        except ImportError:
            print(f"✗ {module:15} - {description} [MISSING]")
            missing.append(module)
    
    print()
    
    if missing:
        print(f"Missing packages: {', '.join(missing)}")
        print("\nInstall with:")
        print(f"  pip install {' '.join(missing)} --break-system-packages")
        print()
        return False
    else:
        print("All dependencies installed ✓")
        print()
        return True


def test_imports():
    """Test that custom modules can be imported"""
    
    print("Testing custom modules...\n")
    
    try:
        print("Importing four_axis_slicer...", end=" ")
        from four_axis_slicer import FourAxisSlicer
        print("✓")
        
        print("Importing overhang_analyzer...", end=" ")
        from overhang_analyzer import OverhangAnalyzer, AdaptiveRotationStrategy
        print("✓")
        
        print("Importing config_4axis...", end=" ")
        import config_4axis
        print("✓")
        
        print()
        return True
    except Exception as e:
        print(f"✗")
        print(f"\nError: {e}")
        return False


def test_simple_stl():
    """Create and test slicing a simple cube STL"""
    
    print("Testing with simple cube model...\n")
    
    try:
        import numpy as np
        import trimesh
        from four_axis_slicer import FourAxisSlicer
        
        # Create a simple cube
        vertices = np.array([
            [0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0],  # Bottom
            [0, 0, 10], [10, 0, 10], [10, 10, 10], [0, 10, 10]  # Top
        ])
        
        faces = np.array([
            [0, 1, 2], [0, 2, 3],  # Bottom
            [4, 5, 6], [4, 6, 7],  # Top
            [0, 1, 5], [0, 5, 4],  # Front
            [2, 3, 7], [2, 7, 6],  # Back
            [0, 3, 7], [0, 7, 4],  # Left
            [1, 2, 6], [1, 6, 5]   # Right
        ])
        
        # Create mesh and save
        cube = trimesh.Trimesh(vertices=vertices, faces=faces)
        test_stl = 'test_cube.stl'
        cube.export(test_stl)
        print(f"Created test cube: {test_stl}")
        
        # Try slicing
        print("Slicing test cube...", end=" ")
        slicer = FourAxisSlicer(test_stl, layer_height=2.0)
        layers = slicer.slice()
        print(f"✓ ({len(layers)} layers)")
        
        # Generate toolpaths
        print("Generating toolpaths...", end=" ")
        toolpaths = slicer.generate_toolpaths()
        print(f"✓ ({len(toolpaths)} toolpath groups)")
        
        # Generate gcode
        print("Generating gcode...", end=" ")
        gcode = slicer.generate_gcode('test_output.gcode')
        print("✓")
        
        print("\nTest output: test_output.gcode")
        print()
        
        # Cleanup
        Path(test_stl).unlink()
        
        return True
    except Exception as e:
        print(f"✗")
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all checks"""
    
    success = True
    
    # Check dependencies
    if not check_dependencies():
        success = False
    
    # Test imports
    if not test_imports():
        success = False
    
    # Test simple model
    if not test_simple_stl():
        success = False
    
    # Summary
    print("="*60)
    if success:
        print("All checks passed! ✓")
        print("\nYou're ready to slice. Run:")
        print("  python slicer_runner.py your_model.stl")
    else:
        print("Some checks failed. Please fix errors above.")
        sys.exit(1)
    print("="*60 + "\n")


if __name__ == '__main__':
    main()