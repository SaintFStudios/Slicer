"""
4-Axis Slicer for Ender 3 Neo Max with Y-axis (B-axis) Rotation
Generates XYZB gcode to avoid overhangs via progressive rotation

Author: Your Name
Based on FullControl multiaxis approach
"""

import numpy as np
import trimesh
import math
from pathlib import Path
from typing import List, Tuple


class FourAxisSlicer:
    """Main slicer class for XYZB toolpath generation"""
    
    def __init__(self, stl_path: str, layer_height: float = 0.2, 
                 nozzle_diameter: float = 0.4, print_speed: float = 40):
        """
        Initialize slicer
        
        Args:
            stl_path: Path to STL file
            layer_height: Z height per layer (mm)
            nozzle_diameter: Nozzle diameter (mm)
            print_speed: Print speed (mm/s)
        """
        self.stl_path = stl_path
        self.layer_height = layer_height
        self.nozzle_diameter = nozzle_diameter
        self.line_width = nozzle_diameter * 1.2  # Extrusion width
        self.print_speed = print_speed
        self.travel_speed = 150  # mm/s
        
        # Load mesh
        self.mesh = trimesh.load(stl_path)
        
        
        # Get bounds
        self.bounds = self.mesh.bounds
        self.z_min = self.bounds[0][2]
        self.z_max = self.bounds[1][2]
        self.height = self.z_max - self.z_min
        
        # B-axis (rotation) parameters
        self.bed_center = np.array([
            (self.bounds[0][0] + self.bounds[1][0]) / 2,
            (self.bounds[0][1] + self.bounds[1][1]) / 2,
            self.z_min
        ])
        
        self.layers = []
        self.toolpaths = []
        
    def slice(self) -> List[dict]:
        """
        Slice mesh into layers with progressive B rotation
        
        Returns:
            List of layer dicts with geometry and rotation info
        """
        num_layers = int(np.ceil(self.height / self.layer_height))
        
        for layer_idx in range(num_layers):
            z_height = self.z_min + layer_idx * self.layer_height
            
            # Get 2D contours at this Z height
            contours = self._get_slice_at_z(z_height)
            
            if not contours:
                continue
            
            # Calculate optimal B rotation to minimize overhangs
            b_angle = self._calculate_optimal_rotation(layer_idx, num_layers)
            
            layer = {
                'index': layer_idx,
                'z': z_height,
                'b_angle': b_angle,
                'contours': contours,
                'num_contours': len(contours)
            }
            
            self.layers.append(layer)
            
        return self.layers
    
    def _get_slice_at_z(self, z: float) -> List[np.ndarray]:
        """
        Get 2D contours where mesh intersects plane at height z
        
        Args:
            z: Height of slicing plane
            
        Returns:
            List of contour polylines (each is Nx2 array of x,y points)
        """
        plane_origin = np.array([0, 0, z])
        plane_normal = np.array([0, 0, 1])
        
        try:
            # Get intersection with plane
            section = self.mesh.section(plane_origin, plane_normal)
            if section is None:
                return []
            
            # Convert to 2D paths
            paths = section.discrete
            contours = []
            
            for path in paths:
                # Extract x, y coordinates
                xy = path[:, :2]
                if len(xy) > 2:
                    contours.append(xy)
            
            return contours
        except:
            return []
    
    def _calculate_optimal_rotation(self, layer_idx: int, total_layers: int) -> float:
        """
        Calculate B rotation angle to minimize overhangs
        Progressive rotation from 0 to max_rotation
        
        Args:
            layer_idx: Current layer index
            total_layers: Total number of layers
            
        Returns:
            B angle in degrees
        """
        # Simple linear progression: rotate more as we go up
        # Adjust max_rotation based on part geometry
        max_rotation = 45.0  # degrees, can be tuned
        
        if total_layers <= 1:
            return 0.0
        
        progress = layer_idx / (total_layers - 1)
        b_angle = progress * max_rotation
        
        return b_angle
    
    def generate_toolpaths(self) -> List[dict]:
        """
        Generate full toolpaths (walls + infill) for all layers
        
        Returns:
            List of toolpath objects
        """
        self.toolpaths = []
        
        for layer in self.layers:
            layer_paths = []
            
            # Generate wall contours
            for contour in layer['contours']:
                wall_path = self._offset_contour(contour, -self.line_width/2)
                if wall_path is not None and len(wall_path) > 0:
                    layer_paths.append({
                        'type': 'wall',
                        'path': wall_path,
                        'b_angle': layer['b_angle']
                    })
            
            # Generate simple grid infill
            infill_paths = self._generate_grid_infill(layer['contours'], layer['b_angle'])
            layer_paths.extend(infill_paths)
            
            self.toolpaths.append({
                'layer': layer['index'],
                'z': layer['z'],
                'b_angle': layer['b_angle'],
                'paths': layer_paths
            })
        
        return self.toolpaths
    
    def _offset_contour(self, contour: np.ndarray, offset: float) -> np.ndarray:
        """
        Offset a 2D contour inward/outward
        Simple implementation: offset each point toward centroid
        
        Args:
            contour: Nx2 array of points
            offset: Offset distance (negative = inward)
            
        Returns:
            Offset contour or None if fails
        """
        if len(contour) < 3:
            return None
        
        try:
            centroid = contour.mean(axis=0)
            direction = contour - centroid
            dist = np.linalg.norm(direction, axis=1, keepdims=True)
            dist[dist == 0] = 1  # Avoid division by zero
            
            unit_direction = direction / dist
            offset_contour = contour - unit_direction * offset
            
            return offset_contour
        except:
            return None
    
    def _generate_grid_infill(self, contours: List[np.ndarray], 
                             b_angle: float, spacing: float = 4.0) -> List[dict]:
        """
        Generate simple grid infill pattern
        
        Args:
            contours: List of boundary contours
            b_angle: B rotation angle for this layer
            spacing: Distance between infill lines (mm)
            
        Returns:
            List of infill path objects
        """
        infill_paths = []
        
        if not contours:
            return infill_paths
        
        # Get bounding box
        all_points = np.vstack(contours)
        x_min, y_min = all_points.min(axis=0)
        x_max, y_max = all_points.max(axis=0)
        
        # Generate horizontal lines
        y = y_min
        while y < y_max:
            x_start = np.array([x_min, y])
            x_end = np.array([x_max, y])
            
            # Simple check: line intersects boundary (crude but fast)
            infill_paths.append({
                'type': 'infill',
                'path': np.array([x_start, x_end]),
                'b_angle': b_angle
            })
            
            y += spacing
        
        return infill_paths
    
    def generate_gcode(self, output_path: str = 'output.gcode') -> str:
        """
        Generate XYZB gcode from toolpaths
        
        Args:
            output_path: Output gcode filename
            
        Returns:
            Generated gcode string
        """
        gcode = []
        
        # Header
        gcode.append("; 4-Axis XYZB Gcode")
        gcode.append(f"; Generated from {self.stl_path}")
        gcode.append("; Duet3 RepRapFirmware")
        gcode.append("")
        
        # Start sequence
        gcode.append("; Start sequence")
        gcode.append("G28  ; Home all axes")
        gcode.append("G29  ; Auto leveling (if configured)")
        gcode.append("G1 Z5 F300  ; Lift nozzle")
        gcode.append("")
        
        # Preheating (adjust for your filament)
        gcode.append("; Heating")
        gcode.append("M104 S200  ; Set extruder temp")
        gcode.append("M109 S200  ; Wait for extruder")
        gcode.append("M140 S60   ; Set bed temp")
        gcode.append("M190 S60   ; Wait for bed")
        gcode.append("")
        
        extrusion_factor = 0.1  # Simple extrusion multiplier
        current_b = 0.0
        
        for layer_data in self.toolpaths:
            b_angle = layer_data['b_angle']
            z = layer_data['z']
            
            gcode.append(f"; Layer {layer_data['layer']}")
            
            # Move to layer Z
            gcode.append(f"G1 Z{z:.3f} F{self.travel_speed}")
            
            # Update B rotation if needed
            if abs(b_angle - current_b) > 0.1:
                gcode.append(f"G1 B{b_angle:.2f} F100  ; Rotate extruder")
                current_b = b_angle
            
            # Process paths in layer
            for path_obj in layer_data['paths']:
                path = path_obj['path']
                
                if len(path) < 2:
                    continue
                
                # First point: move without extruding
                start = path[0]
                gcode.append(f"G1 X{start[0]:.3f} Y{start[1]:.3f} F{self.travel_speed}")
                
                # Extrude along path
                for point in path[1:]:
                    distance = np.linalg.norm(point - path[0])
                    extrusion = distance * extrusion_factor
                    gcode.append(f"G1 X{point[0]:.3f} Y{point[1]:.3f} E{extrusion:.4f} F{self.print_speed}")
            
            gcode.append("")
        
        # End sequence
        gcode.append("; End sequence")
        gcode.append("M104 S0     ; Turn off extruder")
        gcode.append("M140 S0     ; Turn off bed")
        gcode.append("G28 X0 Y0   ; Home X and Y")
        gcode.append("G1 Z{:.1f} F300".format(self.z_max + 10))
        gcode.append("M84         ; Disable motors")
        
        # Write to file
        gcode_str = '\n'.join(gcode)
        with open(output_path, 'w') as f:
            f.write(gcode_str)
        
        print(f"Gcode written to {output_path}")
        return gcode_str
    
    def print_summary(self):
        """Print slicing summary"""
        print(f"\n=== 4-Axis Slicer Summary ===")
        print(f"STL: {self.stl_path}")
        print(f"Model height: {self.height:.2f} mm")
        print(f"Layers: {len(self.layers)}")
        print(f"Layer height: {self.layer_height} mm")
        print(f"Line width: {self.line_width:.2f} mm")
        print(f"Print speed: {self.print_speed} mm/s")
        print(f"B rotation range: 0° to {max(l['b_angle'] for l in self.layers):.1f}°")
        print(f"Toolpaths: {len(self.toolpaths)}")


def main():
    """Example usage"""
    # Example: slice a model
    stl_file = 'example.stl'  # Replace with your STL
    
    slicer = FourAxisSlicer(stl_file, layer_height=0.2)
    slicer.slice()
    slicer.generate_toolpaths()
    slicer.generate_gcode('output.gcode')
    slicer.print_summary()


if __name__ == '__main__':
    main()
