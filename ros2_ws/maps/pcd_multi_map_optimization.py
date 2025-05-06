import open3d as o3d
import argparse
import os
import time

# Define a set of 6 distinct colors (RGB in [0, 1])
distinct_colors = [
    [1.0, 0.0, 0.0],  # Red
    [0.0, 1.0, 0.0],  # Green
    [0.0, 0.0, 1.0],  # Blue
    [1.0, 1.0, 0.0],  # Yellow
    [1.0, 0.0, 1.0],  # Magenta
    [0.0, 1.0, 1.0]   # Cyan
]

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

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

    combined_pcd = o3d.geometry.PointCloud()
    has_loaded_cloud = False

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
        print(f"Estimating normals for {global_map_path}...")
        global_pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))

        # Assign a color based on the index, cycling if more than 6 paths
        color_index = i % len(distinct_colors)
        current_color = distinct_colors[color_index]

        global_pcd.paint_uniform_color(current_color)
        vis.add_geometry(global_pcd)

        combined_pcd += global_pcd
        has_loaded_cloud = True

        if trajectory_pcd is not None:
            trajectory_pcd.paint_uniform_color(current_color)
            vis.add_geometry(trajectory_pcd)
            combined_pcd += trajectory_pcd
            

    print("Visualizing point clouds...")
    vis.run()
    vis.destroy_window()

    if has_loaded_cloud:
        print("Combining point clouds...")
        return combined_pcd
    else:
        print("No point clouds loaded. Exiting.")
        return None

def save_point_cloud(pcd):
    save_file = input("Save the visualized point cloud? (y/[n]): ").lower()
    if save_file == 'y':
        save_path = input("Enter the path to save the point cloud: ")
        file_name = input("Enter the file name (without extension): ")
        save_format = input("Save point cloud as? (ply/pcd/obj): ").lower()
        if save_format in ("ply", "pcd", "obj"):
            os.makedirs(save_path, exist_ok=True)
            save_path = os.path.join(save_path, f"{file_name}.{save_format}")
            o3d.io.write_point_cloud(save_path, pcd)
            print(f"Saved registered point cloud as {save_path}")
            return True
        else:
            print("Invalid save format. Not saving. Try again")
            return False    
    else:
        print("Point cloud not saved.")
        return True
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize PCD files from one or more directories with distinct colors (up to 6).")
    parser.add_argument("paths", nargs='+', help="One or more paths to directories containing PCD files.")
    args = parser.parse_args()

    combined_pcd = visualize_point_clouds(args.paths)
    if combined_pcd:
        while not save_point_cloud(combined_pcd):
            pass
    else:
        print("No point clouds to visualize.")