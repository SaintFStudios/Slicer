"""
Overhang Detection & B-Angle Optimization
Calculates optimal Y-axis (B-axis) rotation to minimize overhangs
"""

import numpy as np
import trimesh
from typing import Tuple, List


class OverhangAnalyzer:
    """Analyzes and detects overhangs in sliced geometry"""
    
    def __init__(self, mesh: trimesh.Trimesh, overhang_threshold: float = 45.0):
        """
        Initialize analyzer
        
        Args:
            mesh: Trimesh object
            overhang_threshold: Angle threshold for overhang detection (degrees, from vertical)
        """
        self.mesh = mesh
        self.threshold_rad = np.radians(overhang_threshold)
        self.face_normals = mesh.face_normals
        
    def detect_overhang_faces(self) -> np.ndarray:
        """
        Detect which faces are overhanging (pointing upward/outward too much)
        
        Returns:
            Boolean array marking overhang faces
        """
        # Vertical direction is Z
        vertical = np.array([0, 0, 1])
        
        # Dot product of face normals with vertical
        dot_products = np.dot(self.face_normals, vertical)
        
        # Overhang if normal points too far from vertical
        # (angle from vertical > threshold)
        overhang_mask = np.abs(np.arccos(np.clip(dot_products, -1, 1))) > self.threshold_rad
        
        return overhang_mask
    
    def get_overhang_points(self, layer_z: float) -> np.ndarray:
        """
        Get points on the overhang boundary at a specific Z height
        
        Args:
            layer_z: Z height to analyze
            
        Returns:
            Array of overhanging point positions
        """
        # Get overhang faces
        overhang_faces = self.detect_overhang_faces()
        
        # Get vertices of overhang faces
        overhang_vertices_idx = self.mesh.faces[overhang_faces].flatten()
        overhang_vertices = self.mesh.vertices[np.unique(overhang_vertices_idx)]
        
        # Filter to points near layer_z
        z_tolerance = 2.0  # mm
        near_layer = np.abs(overhang_vertices[:, 2] - layer_z) < z_tolerance
        
        return overhang_vertices[near_layer]
    
    def calculate_optimal_b_angle(self, layer_z: float, 
                                 bed_center: np.ndarray,
                                 max_rotation: float = 45.0) -> float:
        """
        Calculate B rotation angle to minimize overhangs at this layer
        
        Args:
            layer_z: Current layer Z height
            bed_center: Center point of bed (rotation center)
            max_rotation: Maximum rotation allowed (degrees)
            
        Returns:
            Optimal B angle in degrees
        """
        overhang_points = self.get_overhang_points(layer_z)
        
        if len(overhang_points) == 0:
            return 0.0
        
        # Get XY positions relative to bed center
        xy_overhang = overhang_points[:, :2] - bed_center[:2]
        
        if len(xy_overhang) == 0:
            return 0.0
        
        # Calculate angles to overhang points
        angles = np.arctan2(xy_overhang[:, 1], xy_overhang[:, 0])
        angles_deg = np.degrees(angles)
        
        # Find direction with most overhangs
        hist, bin_edges = np.histogram(angles_deg, bins=8, range=(-180, 180))
        max_bin = np.argmax(hist)
        overhang_direction = bin_edges[max_bin]
        
        # Rotate to face that direction away (rotate opposite)
        b_angle = -overhang_direction
        
        # Clamp to max rotation
        b_angle = np.clip(b_angle, -max_rotation, max_rotation)
        
        return b_angle


class AdaptiveRotationStrategy:
    """Adaptive strategy for B-angle based on geometry"""
    
    @staticmethod
    def linear_progression(layer_idx: int, total_layers: int, 
                          max_angle: float = 45.0) -> float:
        """
        Linear rotation progression (basic)
        
        Args:
            layer_idx: Current layer
            total_layers: Total layers
            max_angle: Maximum rotation angle
            
        Returns:
            B angle in degrees
        """
        if total_layers <= 1:
            return 0.0
        progress = layer_idx / (total_layers - 1)
        return progress * max_angle
    
    @staticmethod
    def smooth_progression(layer_idx: int, total_layers: int,
                          max_angle: float = 45.0) -> float:
        """
        Smooth S-curve progression (ramps up, plateaus, ramps down)
        Better for avoiding rapid rotations
        
        Args:
            layer_idx: Current layer
            total_layers: Total layers
            max_angle: Maximum rotation angle
            
        Returns:
            B angle in degrees
        """
        if total_layers <= 1:
            return 0.0
        
        progress = layer_idx / (total_layers - 1)
        
        # S-curve: smooth step function
        smooth = 3 * progress**2 - 2 * progress**3
        
        return smooth * max_angle
    
    @staticmethod
    def adaptive_by_height(layer_idx: int, total_layers: int,
                          mesh: trimesh.Trimesh = None,
                          max_angle: float = 45.0) -> float:
        """
        Adaptive rotation based on model complexity at each layer
        More rotation where part has more overhangs
        
        Args:
            layer_idx: Current layer
            total_layers: Total layers
            mesh: Trimesh object for analysis
            max_angle: Maximum rotation angle
            
        Returns:
            B angle in degrees
        """
        if mesh is None:
            return AdaptiveRotationStrategy.smooth_progression(
                layer_idx, total_layers, max_angle
            )
        
        # Analyze overhangs at this height
        analyzer = OverhangAnalyzer(mesh)
        z_height = mesh.bounds[0][2] + (layer_idx / total_layers) * \
                   (mesh.bounds[1][2] - mesh.bounds[0][2])
        
        overhang_count = len(analyzer.get_overhang_points(z_height))
        max_overhang = mesh.vertices.shape[0] / 10  # Normalize
        
        overhang_ratio = min(overhang_count / max_overhang, 1.0)
        
        return overhang_ratio * max_angle


def calculate_rotation_schedule(num_layers: int, mesh: trimesh.Trimesh = None,
                               strategy: str = 'smooth',
                               max_angle: float = 45.0) -> List[float]:
    """
    Generate full rotation schedule for all layers
    
    Args:
        num_layers: Total number of layers
        mesh: Trimesh object (optional, for adaptive)
        strategy: 'linear', 'smooth', or 'adaptive'
        max_angle: Maximum rotation angle
        
    Returns:
        List of B angles for each layer
    """
    angles = []
    
    for layer_idx in range(num_layers):
        if strategy == 'linear':
            angle = AdaptiveRotationStrategy.linear_progression(
                layer_idx, num_layers, max_angle
            )
        elif strategy == 'smooth':
            angle = AdaptiveRotationStrategy.smooth_progression(
                layer_idx, num_layers, max_angle
            )
        elif strategy == 'adaptive':
            angle = AdaptiveRotationStrategy.adaptive_by_height(
                layer_idx, num_layers, mesh, max_angle
            )
        else:
            angle = 0.0
        
        angles.append(angle)
    
    return angles
