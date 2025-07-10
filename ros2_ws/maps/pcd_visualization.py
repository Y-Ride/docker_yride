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

rtk_gps_points_path = os.path.join(path_to_pcd, "gps_path_points.pcd")
if not os.path.exists(rtk_gps_points_path):
    print(f"RTK GPS points path not found: {rtk_gps_points_path}. Skipping RTK GPS poinsts")
    use_rtk = False
else:
    use_rtk = True

rtk_gps_cov_path = os.path.join(path_to_pcd, "gps_covariance_lines.ply")
if not os.path.exists(rtk_gps_cov_path):
    print(f"RTK GPS covariance path not found: {rtk_gps_cov_path}. Skipping RTK GPS covariance")
    use_rtk_cov = False
else:
    use_rtk_cov = True

# Load the PCD file

global_pcd = o3d.io.read_point_cloud(global_map_path)
trajectory_pcd = o3d.io.read_point_cloud(trajectory_path)

def read_ply_lines_manual(filepath):
    """
    Manually parses an ASCII PLY file to extract vertices, vertex colors, and edges
    and returns an Open3D LineSet object.
    It expects 'vertex' and 'edge' elements in the header and their respective data.
    """
    vertices = []
    vertex_colors = [] # To store vertex colors
    lines = []
    header_end = False
    num_vertices = 0
    num_edges = 0
    vertex_properties = [] # Stores (name, type) for each property

    try:
        with open(filepath, 'r') as f:
            lines_content = f.readlines()

        line_idx = 0
        # --- Parse Header ---
        while line_idx < len(lines_content):
            line = lines_content[line_idx].strip()
            if line == "end_header":
                header_end = True
                break
            elif line.startswith("element vertex"):
                num_vertices = int(line.split()[-1])
            elif line.startswith("element edge"):
                num_edges = int(line.split()[-1])
            elif line.startswith("property"):
                parts = line.split()
                if parts[1] != "list": # Only care about non-list properties for vertices here
                    vertex_properties.append((parts[-1], parts[1])) # (name, type)
            line_idx += 1

        if not header_end:
            print("Error: 'end_header' not found in PLY file.")
            return None

        # Determine column indices for x, y, z, red, green, blue based on header order
        prop_indices = {name: i for i, (name, _) in enumerate(vertex_properties)}
        x_col = prop_indices.get('x')
        y_col = prop_indices.get('y')
        z_col = prop_indices.get('z')
        red_col = prop_indices.get('red')
        green_col = prop_indices.get('green')
        blue_col = prop_indices.get('blue')

        # --- Parse Vertex Data ---
        vertex_data_start_idx = line_idx + 1 # Start reading vertex data after end_header
        for i in range(num_vertices):
            line = lines_content[vertex_data_start_idx + i].strip()
            parts = line.split()
            
            try:
                # Extract XYZ
                x = float(parts[x_col])
                y = float(parts[y_col])
                z = float(parts[z_col])
                vertices.append([x, y, z])

                # Extract RGB (and normalize to 0-1)
                if red_col is not None and green_col is not None and blue_col is not None:
                    r = int(parts[red_col]) / 255.0
                    g = int(parts[green_col]) / 255.0
                    b = int(parts[blue_col]) / 255.0
                    vertex_colors.append([r, g, b])
                else:
                    vertex_colors.append([0.5, 0.5, 0.5]) # Default grey if no color in PLY
            except (ValueError, IndexError) as e:
                print(f"Error parsing vertex line {vertex_data_start_idx + i}: '{line}' - {e}")
                # Append dummy data to maintain list length if parsing fails
                vertices.append([0.0, 0.0, 0.0])
                vertex_colors.append([0.0, 0.0, 0.0])


        # --- Parse Edge Data ---
        edge_data_start_idx = vertex_data_start_idx + num_vertices
        for i in range(num_edges):
            line = lines_content[edge_data_start_idx + i].strip()
            parts = [int(p) for p in line.split()]
            if len(parts) >= 3 and parts[0] == 2: # Expecting '2 v1_idx v2_idx'
                lines.append([parts[1], parts[2]])
            else:
                print(f"Warning: Malformed edge line {edge_data_start_idx + i}: '{line}'")

    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during PLY manual parsing: {e}")
        return None

    if not vertices or not lines:
        print("Warning: No vertices or lines found after manual parsing.")
        return None

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.asarray(vertices))
    line_set.lines = o3d.utility.Vector2iVector(np.asarray(lines))

    # Assign colors to lines. Each line needs a color.
    # Since covariance axes from one point share the same color,
    # we can use the color of the first vertex of each line segment.
    line_colors_for_lineset = []
    for line_indices in lines:
        if line_indices[0] < len(vertex_colors):
            line_colors_for_lineset.append(vertex_colors[line_indices[0]])
        else:
            line_colors_for_lineset.append([0.5, 0.5, 0.5]) # Fallback to grey

    if line_colors_for_lineset:
        line_set.colors = o3d.utility.Vector3dVector(np.asarray(line_colors_for_lineset))

    return line_set


if use_rtk:
    rtk_pose = o3d.io.read_point_cloud(rtk_gps_points_path)
# if use_rtk_cov:
# #     rtk_cov = o3d.io.read_point_cloud(rtk_gps_cov_path)
#     rtk_cov = o3d.io.read_line_set(rtk_gps_cov_path)
#     # Check if the LineSet was loaded successfully
#     if not rtk_cov.has_points() or not rtk_cov.has_lines():
#         print(f"Warning: Could not load lines from {rtk_gps_cov_path}. It might only contain points or be malformed.")
#         use_rtk_cov = False # Revert to false if line set is empty
# Attempt to load the .ply file. Prioritize native, then fallback to manual.
rtk_cov = None
if use_rtk_cov:
    try:
        print(f"Attempting manual parse using file: {rtk_gps_cov_path}")
        rtk_cov = read_ply_lines_manual(rtk_gps_cov_path)
    except Exception as e:
        print(f"Error with native Open3D reader ({e}). Attempting manual parse.")
        rtk_cov = read_ply_lines_manual(rtk_gps_cov_path)

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
    vis.add_geometry(rtk_pose)
if use_rtk_cov:
    vis.add_geometry(rtk_cov)


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

