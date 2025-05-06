import open3d as o3d
import argparse
import os

# Define a set of 6 distinct colors (RGB in [0, 1])
distinct_colors = [
    [1.0, 0.0, 0.0],  # Red
    [0.0, 1.0, 0.0],  # Green
    [0.0, 0.0, 1.0],  # Blue
    [1.0, 1.0, 0.0],  # Yellow
    [1.0, 0.0, 1.0],  # Magenta
    [0.0, 1.0, 1.0]   # Cyan
]

def visualize_point_clouds(paths):
    """
    Visualizes point clouds from multiple directories, each with a unique color (up to 6).

    Args:
        paths (list): A list of directory paths containing PCD files.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    opt = vis.get_render_option()
    opt.point_size = 3.0
    opt.background_color = [0, 0, 0]

    # Visualize initial pointclouds
    for i, path_to_pcd in enumerate(paths):
        global_map_path = os.path.join(path_to_pcd, "Cleaned_GlobalMap.pcd")
        if not os.path.exists(global_map_path):
            print(f"Cleaned_GlobalMap.pcd not found in {path_to_pcd}, trying GlobalMap.pcd")
            global_map_path = os.path.join(path_to_pcd, "GlobalMap.pcd")
        if not os.path.exists(global_map_path):
            print(f"Warning: Neither 'Cleaned_GlobalMap.pcd' nor 'GlobalMap.pcd' found in {path_to_pcd}. Skipping directory.")
            continue

        trajectory_path = os.path.join(path_to_pcd, "trajectory.pcd")
        if not os.path.exists(trajectory_path):
            print(f"Warning: trajectory.pcd not found in {path_to_pcd}. Skipping trajectory for this directory.")
            trajectory_pcd = None
        else:
            trajectory_pcd = o3d.io.read_point_cloud(trajectory_path)

        global_pcd = o3d.io.read_point_cloud(global_map_path)

        # Assign a color based on the index, cycling if more than 6 paths
        color_index = i % len(distinct_colors)
        current_color = distinct_colors[color_index]

        global_pcd.paint_uniform_color(current_color)
        vis.add_geometry(global_pcd)

        if trajectory_pcd is not None:
            trajectory_pcd.paint_uniform_color(current_color)
            vis.add_geometry(trajectory_pcd)

    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize PCD files from one or more directories with distinct colors (up to 6).")
    parser.add_argument("paths", nargs='+', help="One or more paths to directories containing PCD files.")
    args = parser.parse_args()

    visualize_point_clouds(args.paths)