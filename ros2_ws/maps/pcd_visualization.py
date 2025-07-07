import open3d as o3d
import argparse
import os
import numpy as np

# Parse the arguments
parser = argparse.ArgumentParser()
parser.add_argument("pcd",  help="Path to the PCD file", type=str)
args = parser.parse_args()
path_to_pcd = args.pcd

global_map_path = os.path.join(path_to_pcd, "Cleaned_GlobalMap.pcd")
if not os.path.exists(global_map_path):
    print(f"Cleaned_GlobalMap.pcd not found in {path_to_pcd}, trying GlobalMap.pcd")
    global_map_path = os.path.join(path_to_pcd, "GlobalMap.pcd")
if not os.path.exists(global_map_path):
    raise FileNotFoundError(f"Neither 'Cleaned_GlobalMap.pcd' nor 'GlobalMap.pcd' found in {path_to_pcd}")

trajectory_path = os.path.join(path_to_pcd, "trajectory.pcd")
if not os.path.exists(trajectory_path):
    raise FileNotFoundError(f"trajectory.pcd not found in {path_to_pcd}")

rtk_gps_path = os.path.join(path_to_pcd, "gps_markers.pcd")
if not os.path.exists(rtk_gps_path):
    print(f"RTK GPS path not found: {rtk_gps_path}. Skipping RTK GPS path")
    use_rtk = False
else:
    use_rtk = True

# Load the PCD file

global_pcd = o3d.io.read_point_cloud(global_map_path)
trajectory_pcd = o3d.io.read_point_cloud(trajectory_path)
if use_rtk:
    rtk_pcd = o3d.io.read_point_cloud(rtk_gps_path)

trajectory_pcd.paint_uniform_color([1, 1, 1])  # Red, RGB in [0, 1]
# pcd.paint_uniform_color([1, 0, 0])  # Red, RGB in [0, 1]
# points = np.asarray(pcd.points)
# z_vals = points[:, 2]
# colors = plt.get_cmap("viridis")((z_vals - z_vals.min()) / (z_vals.max() - z_vals.min()))[:, :3]
# pcd.colors = o3d.utility.Vector3dVector(colors)

# Find normals for the global map
# print("Estimating normals for the global map...")
# global_pcd.estimate_normals(
#         o3d.geometry.KDTreeSearchParamHybrid(radius=2.0, max_nn=30))

# Visualize
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
# o3d.visualization.draw([point_cloud, frame])
# o3d.visualization.draw_geometries([pcd])

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(frame)
vis.add_geometry(trajectory_pcd)
vis.add_geometry(global_pcd)
if use_rtk:
    vis.add_geometry(rtk_pcd)

opt = vis.get_render_option()
opt.point_size = 3.0  # Try values like 3–10 for visibility
opt.background_color = [0, 0, 0]  # Optional: black background

vis.run()
vis.destroy_window()
# geometries = [
#     {"name": "Global Map", "geometry": global_pcd, "point_size": 1.0},
#     {"name": "Trajectory", "geometry": trajectory_pcd, "point_size": 5.0},
#     {"name": "Frame", "geometry": frame},
# ]

# if use_rtk:
#     geometries.append({"name": "RTK GPS", "geometry": rtk_pcd, "point_size": 7.0})

# o3d.visualization.draw(geometries, bg_color=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32))

