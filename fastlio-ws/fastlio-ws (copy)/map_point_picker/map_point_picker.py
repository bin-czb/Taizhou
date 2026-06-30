import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import yaml
import sys
import os
import math  # === 新增：用于计算角度 ===

# ================= Configuration =================
# DEFAULT_PCD = "map.pcd"
# DEFAULT_POSE = "pose.txt"
DEFAULT_PCD = "../lab_full/map.pcd"
DEFAULT_POSE = "../lab_full/pose.txt"
OUTPUT_YAML = "lab_full.yaml"
# DEFAULT_PCD = "../corridor/map.pcd"
# DEFAULT_POSE = "../corridor/pose.txt"
# OUTPUT_YAML = "corridor_2d_waypoints.yaml"

# Height Filter (meters)
Z_MIN = -0.5  
Z_MAX = 1.0   

# Display Downsample Ratio
DOWN_SAMPLE_RATIO = 0.8
# ===============================================

def save_yaml(points, filename):
    """Save points to YAML with automatic Yaw calculation"""
    data = {
        'frame_id': 'map',
        'z_height': 0.0, 
        'waypoints': []
    }
    
    for i, pt in enumerate(points):
        # 1. 强制类型转换 (numpy -> python float)
        x_val = float(pt[0])
        y_val = float(pt[1])
        
        # 2. === 核心修改：计算 Yaw ===
        yaw_val = 0.0 # 默认第一个点为 0
        
        if i > 0:
            # 获取前一个点
            prev_pt = points[i-1]
            dx = x_val - float(prev_pt[0])
            dy = y_val - float(prev_pt[1])
            
            # 使用 atan2 计算向量角度 (结果为弧度 -pi 到 pi)
            yaw_val = math.atan2(dy, dx)

        data['waypoints'].append({
            'id': i + 1,
            'x': round(x_val, 4),
            'y': round(y_val, 4),
            'yaw': round(yaw_val, 4) 
        })
    
    with open(filename, 'w') as f:
        yaml.dump(data, f, sort_keys=False, default_flow_style=None)
    print(f"\n[Success] Saved {len(points)} waypoints to: {filename}")

class MapPicker:
    def __init__(self, pcd_path, pose_path):
        self.points = []
        self.fig, self.ax = plt.subplots(figsize=(12, 9))
        self.pcd_path = pcd_path
        self.pose_path = pose_path
        self.show_history = True 

    def load_and_process_data(self):
        if not os.path.exists(self.pcd_path):
            print(f"Error: File not found {self.pcd_path}")
            sys.exit(1)
            
        print("Loading point cloud...")
        pcd = o3d.io.read_point_cloud(self.pcd_path)
        pts = np.asarray(pcd.points)

        # Filter by height
        mask = (pts[:, 2] > Z_MIN) & (pts[:, 2] < Z_MAX)
        pts_filtered = pts[mask]
        
        # Downsample
        if len(pts_filtered) > 0:
            indices = np.random.choice(len(pts_filtered), 
                                     int(len(pts_filtered) * DOWN_SAMPLE_RATIO), 
                                     replace=False)
            self.map_x = pts_filtered[indices, 0]
            self.map_y = pts_filtered[indices, 1]
        else:
            self.map_x, self.map_y = [], []

        # Load history pose
        self.traj_x, self.traj_y = [], []
        if os.path.exists(self.pose_path):
            try:
                data = []
                with open(self.pose_path, 'r') as f:
                    for line in f:
                        if line.startswith('#') or not line.strip(): continue
                        parts = line.replace(',', ' ').split()
                        if len(parts) > 3: 
                            if len(parts) >= 8: 
                                data.append([float(parts[1]), float(parts[2])])
                            else: 
                                data.append([float(parts[0]), float(parts[1])])
                if data:
                    data = np.array(data)
                    self.traj_x = data[:, 0]
                    self.traj_y = data[:, 1]
            except Exception:
                pass

    # --- Core Logic ---
    def add_point(self, x, y):
        self.points.append((x, y))
        self.redraw_canvas()
        print(f"Added Point {len(self.points)}: ({x:.3f}, {y:.3f})")

    def undo_point(self):
        if self.points:
            removed = self.points.pop()
            print(f"Undo Point: ({removed[0]:.3f}, {removed[1]:.3f})")
            self.redraw_canvas()
        else:
            print("No points to undo.")

    def clear_all(self):
        if self.points:
            self.points = []
            print("Cleared all points.")
            self.redraw_canvas()

    def toggle_history(self):
        self.show_history = not self.show_history
        self.redraw_canvas()

    # --- Events ---
    def on_click(self, event):
        if event.inaxes != self.ax: return

        if event.button == 1 and event.key == 'shift': 
            self.add_point(event.xdata, event.ydata)
        elif event.button == 3: 
            self.undo_point()

    def on_key(self, event):
        sys.stdout.flush() 
        if event.key == '1':
            self.undo_point()
        elif event.key == '2':
            save_yaml(self.points, OUTPUT_YAML)
            original_title = self.ax.get_title()
            self.ax.set_title(f"*** Saved {len(self.points)} points to YAML! ***", color='red', fontweight='bold')
            self.fig.canvas.draw()
            plt.pause(0.5) 
            self.redraw_canvas() 
        elif event.key == 'c':
            self.clear_all()
        elif event.key == 'h':
            self.toggle_history()
        elif event.key == 'r':
            self.ax.autoscale()
            self.fig.canvas.draw()
            print("View reset")

    def redraw_canvas(self):
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        
        self.ax.clear()
        
        self.ax.set_title(f"Map Waypoint Picker - Points: {len(self.points)}", fontsize=12)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, linestyle=':', alpha=0.6)

        # 1. Draw Map
        self.ax.scatter(self.map_x, self.map_y, s=1.0, c='k', alpha=0.6, label='Map')

        # 2. Draw History
        if self.show_history and len(self.traj_x) > 0:
            self.ax.plot(self.traj_x, self.traj_y, 'g--', linewidth=1, alpha=0.4, label='History Pose')

        # 3. Draw Selected Points with Arrows
        if self.points:
            pts_np = np.array(self.points)
            self.ax.plot(pts_np[:,0], pts_np[:,1], 'ro-', markersize=6, linewidth=1.5, label='Waypoints')
            
            for i, pt in enumerate(self.points):
                # 绘制数字标签
                self.ax.text(pt[0], pt[1], str(i+1), 
                             fontsize=12, color='blue', fontweight='bold',
                             bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=0.5))
                
                # === 新增：绘制 Yaw 箭头 ===
                if i > 0:
                    prev_pt = self.points[i-1]
                    dx = pt[0] - prev_pt[0]
                    dy = pt[1] - prev_pt[1]
                    # 计算角度
                    angle = math.atan2(dy, dx)
                    
                    # 在当前点画一个小箭头，长度为 0.5 米（可根据地图比例调整）
                    arrow_len = 0.5 
                    self.ax.arrow(pt[0], pt[1], 
                                  arrow_len * math.cos(angle), 
                                  arrow_len * math.sin(angle),
                                  head_width=0.2, head_length=0.2, fc='r', ec='r')

        # === Instructions Panel ===
        instructions = (
            "[ Controls ]\n"
            "Shift + Click : Add Point\n"
            "Key 1 / R-Click: Undo\n"
            "Key 2        : Save YAML\n"
            "Key c        : Clear All\n"
            "Key h        : Toggle History\n"
            "Key r        : Reset View"
        )
        self.ax.text(0.02, 0.98, instructions, transform=self.ax.transAxes,
                     fontsize=10, verticalalignment='top', fontfamily='monospace',
                     bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'))

        if xlim != (0.0, 1.0) and ylim != (0.0, 1.0): 
            self.ax.set_xlim(xlim)
            self.ax.set_ylim(ylim)
        else:
            self.ax.axis('equal')

        self.ax.legend(loc='upper right')
        self.fig.canvas.draw()

    def run(self):
        self.load_and_process_data()
        self.redraw_canvas()
        self.ax.axis('equal') 

        try:
            mng = plt.get_current_fig_manager()
            mng.resize(*mng.window.maxsize())
        except:
            pass

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
        print("\n" + "="*50)
        print("Controls:")
        print("  [Shift + Click] : Add Point")
        print("  [1]             : Undo")
        print("  [2]             : Save YAML")
        print("  [c]             : Clear All")
        print("  [h]             : Show/Hide History")
        print("  [r]             : Reset View")
        print("="*50 + "\n")

        plt.show()
        return self.points

if __name__ == "__main__":
    pcd_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PCD
    
    picker = MapPicker(pcd_file, DEFAULT_POSE)
    waypoints = picker.run()
    
    if waypoints:
        save_yaml(waypoints, OUTPUT_YAML)
    else:
        print("No points selected.")