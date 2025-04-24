import open3d as o3d
import argparse
import os

# Parse the arguments
parser = argparse.ArgumentParser()
parser.add_argument("pcd",  help="Path to the PCD file", type=str)
args = parser.parse_args()
path_to_pcd = args.pcd

global_map = os.path.join(path_to_pcd, "Cleaned_GlobalMap.pcd")
if not os.path.exists(global_map):
    print(f"Cleaned_GlobalMap.pcd not found in {path_to_pcd}, trying GlobalMap.pcd")
    global_map = os.path.join(path_to_pcd, "GlobalMap.pcd")
if not os.path.exists(global_map):
    raise FileNotFoundError(f"Neither 'Cleaned_GlobalMap.pcd' nor 'GlobalMap.pcd' found in {path_to_pcd}")

trajectory = os.path.join(path_to_pcd, "trajectory.pcd")
if not os.path.exists(trajectory):
    raise FileNotFoundError(f"trajectory.pcd not found in {path_to_pcd}")

# Load the PCD file

global_pcd = o3d.io.read_point_cloud(global_map)
trajectory_pcd = o3d.io.read_point_cloud(trajectory)

trajectory_pcd.paint_uniform_color([1, 0, 0])  # Red, RGB in [0, 1]
# pcd.paint_uniform_color([1, 0, 0])  # Red, RGB in [0, 1]
# points = np.asarray(pcd.points)
# z_vals = points[:, 2]
# colors = plt.get_cmap("viridis")((z_vals - z_vals.min()) / (z_vals.max() - z_vals.min()))[:, :3]
# pcd.colors = o3d.utility.Vector3dVector(colors)


# Visualize
# o3d.visualization.draw_geometries([pcd])
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(trajectory_pcd)
vis.add_geometry(global_pcd)

opt = vis.get_render_option()
opt.point_size = 3.0  # Try values like 3–10 for visibility
opt.background_color = [0, 0, 0]  # Optional: black background

vis.run()
vis.destroy_window()

