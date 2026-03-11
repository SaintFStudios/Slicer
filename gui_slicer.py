"""
4-Axis Slicer GUI with 3D Model Viewer - Like PrusaSlicer
Full 3D visualization with VTK, settings controls, and B-axis preview
"""

import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import threading
from pathlib import Path

from mpl_toolkits.mplot3d import Axes3D
import trimesh

from four_axis_slicer import FourAxisSlicer
import config_4axis as cfg


class GcodeParser:
    """Parse and extract movement data from gcode"""
    
    def __init__(self, gcode_file):
        self.gcode_file = gcode_file
        self.movements = []
        self.parse()
    
    def parse(self):
        """Parse gcode and extract movements"""
        try:
            with open(self.gcode_file, 'r') as f:
                lines = f.readlines()
            
            current_x, current_y, current_z, current_b = 0, 0, 0, 0
            
            for line in lines:
                # Skip comments and empty lines
                if line.startswith(';') or line.strip() == '':
                    continue
                
                # Remove inline comments
                line = line.split(';')[0].strip()
                
                # Parse G1 commands (movement)
                if line.startswith('G1'):
                    movement = {
                        'x': current_x,
                        'y': current_y,
                        'z': current_z,
                        'b': current_b
                    }
                    
                    # Extract coordinates
                    parts = line.split()
                    for part in parts:
                        if part.startswith('X'):
                            current_x = float(part[1:])
                            movement['x'] = current_x
                        elif part.startswith('Y'):
                            current_y = float(part[1:])
                            movement['y'] = current_y
                        elif part.startswith('Z'):
                            current_z = float(part[1:])
                            movement['z'] = current_z
                        elif part.startswith('B'):
                            current_b = float(part[1:])
                            movement['b'] = current_b
                    
                    # Only add if it's a movement (has E extrusion)
                    if 'E' in line:
                        self.movements.append(movement)
        except Exception as e:
            print(f"Error parsing gcode: {e}")


class Model3DViewer:
    """3D Model Viewer using Matplotlib"""
    
    def __init__(self, parent_frame):
        self.frame = parent_frame
        self.mesh = None
        
        # Create figure
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Embed in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Initial empty plot
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('3D Model Viewer')
        
    def show_model(self, stl_path):
        """Load and display STL model"""
        try:
            # Load mesh using trimesh
            self.mesh = trimesh.load(stl_path)
            
            # Clear previous plot
            self.ax.clear()
            
            # Get vertices
            vertices = np.asarray(self.mesh.vertices)
            faces = np.asarray(self.mesh.faces)
            
            # Plot the mesh using plot_trisurf
            self.ax.plot_trisurf(
                vertices[:, 0],      # X coordinates
                vertices[:, 1],      # Y coordinates
                vertices[:, 2],      # Z coordinates
                triangles=faces,     # Triangle indices
                color=(0.9, 0.5, 0.1),
                alpha=0.8,
                edgecolor='darkgray',
                linewidth=0.1
            )
            
            # Set labels
            self.ax.set_xlabel('X (mm)', fontsize=9)
            self.ax.set_ylabel('Y (mm)', fontsize=9)
            self.ax.set_zlabel('Z (mm)', fontsize=9)
            self.ax.set_title('3D Model (Drag to Rotate | Scroll to Zoom)', fontsize=10)
            
            # Set view angle
            self.ax.view_init(elev=20, azim=45)
            
            self.canvas.draw()
            return True
        except Exception as e:
            print(f"Error loading model: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def add_slice_planes(self, z_min, z_max, layer_height, num_slices=10):
        """Add slice plane indicators as horizontal grid lines"""
        if not self.mesh:
            return
        
        # Get bounds
        vertices = np.asarray(self.mesh.vertices)
        x_min, x_max = vertices[:, 0].min(), vertices[:, 0].max()
        y_min, y_max = vertices[:, 1].min(), vertices[:, 1].max()
        
        # Show every 5th slice plane for clarity
        step = max(1, int(num_slices / 5))
        z_values = np.arange(z_min, z_max, step * layer_height)
        
        # Draw horizontal lines at each slice height
        for z in z_values:
            # Draw a circle at this Z height to show the slice plane
            theta = np.linspace(0, 2*np.pi, 20)
            radius = max(x_max - x_min, y_max - y_min) / 3
            cx = (x_min + x_max) / 2
            cy = (y_min + y_max) / 2
            
            x = cx + radius * np.cos(theta)
            y = cy + radius * np.sin(theta)
            z_arr = np.ones_like(x) * z
            
            self.ax.plot(x, y, z_arr, 'g--', alpha=0.4, linewidth=1)
        
        self.canvas.draw()


class GcodeViewer:
    """3D Toolpath Visualization from Gcode"""
    
    def __init__(self, parent_frame):
        self.frame = parent_frame
        
        # Create figure
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Embed in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def show_gcode(self, gcode_file):
        """Visualize gcode toolpath"""
        try:
            # Parse gcode
            parser = GcodeParser(gcode_file)
            movements = parser.movements
            
            if not movements:
                self.ax.clear()
                self.ax.text(0.5, 0.5, 'No movements found in gcode', 
                           ha='center', va='center', transform=self.ax.transAxes)
                self.canvas.draw()
                return
            
            # Clear previous plot
            self.ax.clear()
            
            # Extract coordinates and B angles
            x_coords = [m['x'] for m in movements]
            y_coords = [m['y'] for m in movements]
            z_coords = [m['z'] for m in movements]
            b_angles = [m['b'] for m in movements]
            
            # Plot toolpath with color based on B angle
            scatter = self.ax.scatter(x_coords, y_coords, z_coords, 
                                     c=b_angles, cmap='coolwarm', 
                                     s=10, alpha=0.6)
            
            # Connect with lines to show path
            self.ax.plot(x_coords, y_coords, z_coords, 'b-', alpha=0.3, linewidth=0.5)
            
            # Add colorbar showing B angle
            cbar = plt.colorbar(scatter, ax=self.ax, pad=0.1, shrink=0.8)
            cbar.set_label('B-Angle (°)', fontsize=9)
            
            # Labels
            self.ax.set_xlabel('X (mm)', fontsize=9)
            self.ax.set_ylabel('Y (mm)', fontsize=9)
            self.ax.set_zlabel('Z (mm)', fontsize=9)
            self.ax.set_title(f'Toolpath Visualization ({len(movements)} moves)', fontsize=10)
            
            # Set view
            self.ax.view_init(elev=20, azim=45)
            
            self.canvas.draw()
            
            # Print stats
            print(f"Gcode loaded: {len(movements)} movements")
            print(f"X range: {min(x_coords):.1f} - {max(x_coords):.1f}")
            print(f"Y range: {min(y_coords):.1f} - {max(y_coords):.1f}")
            print(f"Z range: {min(z_coords):.1f} - {max(z_coords):.1f}")
            print(f"B range: {min(b_angles):.1f} - {max(b_angles):.1f}°")
            
            return True
        except Exception as e:
            print(f"Error displaying gcode: {e}")
            import traceback
            traceback.print_exc()
            return False


class SlicerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("4-Axis Slicer - 3D Model Viewer")
        self.root.geometry("1600x900")
        
        self.stl_path = None
        self.slicer = None
        
        self.setup_ui()
    
    def setup_ui(self):
        """Create the user interface"""
        
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # LEFT SIDE: Controls (narrow)
        left_frame = ttk.Frame(main_frame, width=350)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=5)
        left_frame.pack_propagate(False)
        
        # File selection
        file_frame = ttk.LabelFrame(left_frame, text="1. Load Model", padding=10)
        file_frame.pack(fill=tk.X, pady=5)
        
        self.file_label = ttk.Label(file_frame, text="No file selected", foreground="red", wraplength=300)
        self.file_label.pack(fill=tk.X)
        
        ttk.Button(file_frame, text="Browse STL File", command=self.browse_file).pack(fill=tk.X, pady=5)
        
        # Model Info
        info_frame = ttk.LabelFrame(left_frame, text="Model Info", padding=10)
        info_frame.pack(fill=tk.X, pady=5)
        
        self.info_text = tk.Text(info_frame, height=6, width=40, state=tk.DISABLED, wrap=tk.WORD)
        self.info_text.pack(fill=tk.BOTH)
        
        # Settings
        settings_frame = ttk.LabelFrame(left_frame, text="2. Settings", padding=10)
        settings_frame.pack(fill=tk.X, pady=5)
        
        # Layer Height
        ttk.Label(settings_frame, text="Layer Height (mm):").grid(row=0, column=0, sticky=tk.W, pady=3)
        self.layer_height_var = tk.DoubleVar(value=cfg.LAYER_HEIGHT)
        layer_height_spinbox = ttk.Spinbox(
            settings_frame, from_=0.1, to=0.5, increment=0.05,
            textvariable=self.layer_height_var, width=10,
            command=self.update_preview
        )
        layer_height_spinbox.grid(row=0, column=1, sticky=tk.W)
        
        # Max Rotation
        ttk.Label(settings_frame, text="Max Rotation (°):").grid(row=1, column=0, sticky=tk.W, pady=3)
        self.max_rotation_var = tk.DoubleVar(value=cfg.MAX_ROTATION)
        max_rotation_spinbox = ttk.Spinbox(
            settings_frame, from_=10, to=90, increment=5,
            textvariable=self.max_rotation_var, width=10,
            command=self.update_preview
        )
        max_rotation_spinbox.grid(row=1, column=1, sticky=tk.W)
        
        # Rotation Strategy
        ttk.Label(settings_frame, text="Strategy:").grid(row=2, column=0, sticky=tk.W, pady=3)
        self.strategy_var = tk.StringVar(value=cfg.ROTATION_STRATEGY)
        strategy_combo = ttk.Combobox(
            settings_frame, textvariable=self.strategy_var,
            values=['linear', 'smooth', 'adaptive'], width=15, state='readonly'
        )
        strategy_combo.grid(row=2, column=1, sticky=tk.W)
        strategy_combo.bind('<<ComboboxSelected>>', lambda e: self.update_preview())
        
        # Nozzle Temp
        ttk.Label(settings_frame, text="Nozzle Temp (°C):").grid(row=3, column=0, sticky=tk.W, pady=3)
        self.nozzle_temp_var = tk.IntVar(value=cfg.NOZZLE_TEMP)
        ttk.Spinbox(
            settings_frame, from_=150, to=260, increment=5,
            textvariable=self.nozzle_temp_var, width=10
        ).grid(row=3, column=1, sticky=tk.W)
        
        # Bed Temp
        ttk.Label(settings_frame, text="Bed Temp (°C):").grid(row=4, column=0, sticky=tk.W, pady=3)
        self.bed_temp_var = tk.IntVar(value=cfg.BED_TEMP)
        ttk.Spinbox(
            settings_frame, from_=20, to=120, increment=5,
            textvariable=self.bed_temp_var, width=10
        ).grid(row=4, column=1, sticky=tk.W)
        
        # Slice button
        button_frame = ttk.LabelFrame(left_frame, text="3. Generate", padding=10)
        button_frame.pack(fill=tk.X, pady=5)
        
        self.slice_button = ttk.Button(
            button_frame, text="SLICE & GENERATE GCODE",
            command=self.slice_model, state=tk.DISABLED
        )
        self.slice_button.pack(fill=tk.X, pady=5)
        
        # Status
        self.status_label = ttk.Label(left_frame, text="Ready", foreground="blue", wraplength=300)
        self.status_label.pack(fill=tk.X, pady=5)
        
        # Progress bar
        self.progress = ttk.Progressbar(left_frame, mode='indeterminate')
        self.progress.pack(fill=tk.X, pady=5)
        
        # RIGHT SIDE: 3D Viewer and Graph
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        
        # Notebook for tabs
        notebook = ttk.Notebook(right_frame)
        notebook.pack(fill=tk.BOTH, expand=True)
        
        # Tab 1: 3D Model
        self.viewer_frame = ttk.Frame(notebook)
        notebook.add(self.viewer_frame, text="3D Model Viewer")
        
        viewer_label = ttk.Label(self.viewer_frame, text="3D Model (Rotate: drag | Zoom: scroll)")
        viewer_label.pack(pady=5)
        
        # Create 3D model viewer
        self.model_viewer = Model3DViewer(self.viewer_frame)
        
        # Tab 2: B-Axis Preview
        graph_frame = ttk.Frame(notebook)
        notebook.add(graph_frame, text="B-Axis Rotation Preview")
        
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Tab 3: Gcode Viewer
        self.gcode_viewer_frame = ttk.Frame(notebook)
        notebook.add(self.gcode_viewer_frame, text="Toolpath Preview")
        
        self.gcode_viewer = GcodeViewer(self.gcode_viewer_frame)
        
        # Initialize preview
        self.update_preview()
    
    def browse_file(self):
        """Browse and select STL file"""
        file_path = filedialog.askopenfilename(
            title="Select STL File",
            filetypes=[("STL files", "*.stl"), ("All files", "*.*")]
        )
        
        if file_path:
            self.stl_path = file_path
            self.file_label.config(text=f"✓ {Path(file_path).name}", foreground="green")
            self.load_model_info()
            self.slice_button.config(state=tk.NORMAL)
            self.status_label.config(text="Ready to slice", foreground="blue")
            self.update_preview()
    
    def load_model_info(self):
        """Load and display model information"""
        try:
            self.slicer = FourAxisSlicer(
                self.stl_path,
                layer_height=self.layer_height_var.get()
            )
            
            # Load 3D model into viewer
            self.model_viewer.show_model(self.stl_path)
            
            # Add slice planes
            num_layers = int(np.ceil(self.slicer.height / self.layer_height_var.get()))
            self.model_viewer.add_slice_planes(
                self.slicer.z_min, 
                self.slicer.z_max, 
                self.layer_height_var.get(),
                num_layers
            )
            
            # Display info
            info = f"""File: {Path(self.stl_path).name}

Dimensions:
X: {self.slicer.bounds[0][0]:.1f}-{self.slicer.bounds[1][0]:.1f}
Y: {self.slicer.bounds[0][1]:.1f}-{self.slicer.bounds[1][1]:.1f}
Z: {self.slicer.bounds[0][2]:.1f}-{self.slicer.bounds[1][2]:.1f}

Height: {self.slicer.height:.1f} mm
Layers: {num_layers}

Center: ({self.slicer.bed_center[0]:.0f}, {self.slicer.bed_center[1]:.0f})
            """
            
            self.info_text.config(state=tk.NORMAL)
            self.info_text.delete('1.0', tk.END)
            self.info_text.insert('1.0', info)
            self.info_text.config(state=tk.DISABLED)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load model: {e}")
    
    def update_preview(self):
        """Update the B-angle preview graph"""
        if not self.slicer:
            self.ax.clear()
            self.ax.set_xlabel("Layer")
            self.ax.set_ylabel("B-Angle (degrees)")
            self.ax.set_title("B-Axis Rotation Schedule")
            self.fig.tight_layout()
            self.canvas.draw()
            return
        
        num_layers = int(np.ceil(self.slicer.height / self.layer_height_var.get()))
        max_rotation = self.max_rotation_var.get()
        strategy = self.strategy_var.get()
        
        layers = np.arange(num_layers)
        angles = []
        
        for layer_idx in layers:
            progress = layer_idx / (num_layers - 1) if num_layers > 1 else 0
            
            if strategy == 'linear':
                angle = progress * max_rotation
            elif strategy == 'smooth':
                smooth = 3 * progress**2 - 2 * progress**3
                angle = smooth * max_rotation
            else:
                angle = progress * max_rotation
            
            angles.append(angle)
        
        self.ax.clear()
        self.ax.plot(layers, angles, 'b-', linewidth=2.5)
        self.ax.fill_between(layers, angles, alpha=0.3)
        self.ax.set_xlabel("Layer", fontsize=11)
        self.ax.set_ylabel("B-Angle (degrees)", fontsize=11)
        self.ax.set_title(f"B-Axis Rotation Schedule ({strategy})", fontsize=12)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_ylim(0, max_rotation * 1.1)
        
        stats_text = f"Layers: {num_layers}\nMax: {max_rotation}°"
        self.ax.text(0.98, 0.97, stats_text, transform=self.ax.transAxes,
                    verticalalignment='top', horizontalalignment='right',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7),
                    fontsize=10)
        
        self.fig.tight_layout()
        self.canvas.draw()
    
    def slice_model(self):
        """Slice the model in background thread"""
        if not self.stl_path:
            messagebox.showwarning("Warning", "Please select an STL file first")
            return
        
        thread = threading.Thread(target=self._slice_thread)
        thread.daemon = True
        thread.start()
    
    def _slice_thread(self):
        """Background thread for slicing"""
        try:
            self.status_label.config(text="Slicing... please wait", foreground="orange")
            self.progress.start()
            self.slice_button.config(state=tk.DISABLED)
            self.root.update()
            
            slicer = FourAxisSlicer(
                self.stl_path,
                layer_height=self.layer_height_var.get()
            )
            
            num_layers = len(slicer.slice())
            max_rotation = self.max_rotation_var.get()
            strategy = self.strategy_var.get()
            
            for idx, layer in enumerate(slicer.layers):
                progress = idx / (num_layers - 1) if num_layers > 1 else 0
                
                if strategy == 'linear':
                    b_angle = progress * max_rotation
                elif strategy == 'smooth':
                    smooth = 3 * progress**2 - 2 * progress**3
                    b_angle = smooth * max_rotation
                else:
                    b_angle = progress * max_rotation
                
                layer['b_angle'] = b_angle
            
            slicer.generate_toolpaths()
            output_file = 'output/output.gcode'
            os.makedirs('output', exist_ok=True)
            slicer.generate_gcode(output_file)
            
            self.progress.stop()
            self.status_label.config(
                text=f"✓ Success! Gcode saved",
                foreground="green"
            )
            self.slice_button.config(state=tk.NORMAL)
            
            # Load gcode viewer
            self.gcode_viewer.show_gcode(output_file)
            
            messagebox.showinfo(
                "Success!",
                f"Slicing complete!\n\n"
                f"Layers: {num_layers}\n"
                f"Rotation: 0° → {max_rotation}°\n\n"
                f"Gcode saved to:\noutput/output.gcode\n\n"
                f"Check the 'Toolpath Preview' tab to visualize!"
            )
        
        except Exception as e:
            self.progress.stop()
            self.status_label.config(text=f"Error: {str(e)}", foreground="red")
            self.slice_button.config(state=tk.NORMAL)
            messagebox.showerror("Error", f"Slicing failed:\n{str(e)}")


def main():
    """Main entry point"""
    root = tk.Tk()
    gui = SlicerGUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()
