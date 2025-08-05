import open3d as o3d
import argparse
import os
import copy

# Parse the arguments
parser = argparse.ArgumentParser()
parser.add_argument("pcd", help="Path to the pcd file to be cleaned up", type=str)
args = parser.parse_args()
path_to_pcd = args.pcd

# Construct full path to the PCD file
filename = os.path.basename(path_to_pcd)  # 'my.pcd'
dirname = os.path.dirname(path_to_pcd)    # './test/folder'
cleaned_filename = f"Cleaned_{filename}"      # 'cleaned_my.pcd'
cleaned_path = os.path.join(dirname, cleaned_filename)
# Load the point cloud
print("Loading point cloud...")
global_pcd = o3d.io.read_point_cloud(path_to_pcd)

# Apply Statistical Outlier Removal
print("Starting outlier removal...")
# clean_pcd, ind = global_pcd.remove_statistical_outlier(nb_neighbors=5, std_ratio=50.0)
clean_pcd, ind = global_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=10.0)


# Separate inliers and outliers
inlier_cloud = global_pcd.select_by_index(ind)
outlier_cloud = global_pcd.select_by_index(ind, invert=True)

# Color the inliers and outliers differently
colored_inlier_cloud = copy.copy(inlier_cloud)
colored_inlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])  # Light gray for inliers
outlier_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # Red for outliers

# Visualize the result
o3d.visualization.draw_geometries([colored_inlier_cloud, outlier_cloud])

response = input("Save the cleaned point cloud? (y/[n]): ")
if response.lower() == 'y':
    # Save the cleaned point cloud
    o3d.io.write_point_cloud(cleaned_path, inlier_cloud)
    print(f"Cleaned point cloud saved to {cleaned_path}")
else:
    print("Cleaned point cloud not saved.")
