import copy
import open3d as o3d
import argparse
import os
import numpy as np
import time

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])
    
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    source = o3d.io.read_point_cloud("./gps_ref_infront_EB/marb_ref_set_internal/navsatfix/Cleaned_GlobalMap.pcd")
    target = o3d.io.read_point_cloud("./gps_ref_infront_EB/localize_into_marb_ref_set_internal/navsatfix/Cleaned_GlobalMap.pcd")
    trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], 
                             [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0], 
                             [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, initial_transformation, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-06,  # Default value
        relative_rmse=1e-06,     # Default value
        max_iteration=2000       # Increased iterations
    )
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, initial_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=criteria)
    return result


if __name__ == "__main__":
    np.set_printoptions(suppress=True)
    voxel_size = 0.5  # means 2m for the dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)
    response = input("Was an initial transformation provided? (y/[n]): ")
    if response.lower() == 'y':
        initial_transformation = np.identity(4)
        # initial_transformation = np.array([[ 0.99945164,  0.01702687,  0.02839886,  0.01219619],
        #                                     [-0.01723957,  0.99982502,  0.00726164, -6.94633902],
        #                                     [-0.02827025, -0.00774724,  0.99957029, -5.92616439],
        #                                     [ 0.,         -0.,         -0.,          1.        ]])
        print("Skipping global registration, using provided initial transformation.")
        print("Initial ICP transformation is:")
        print(initial_transformation)
    else:
        # initial_transformation = np.identity(4)

        start = time.time()
        result_fast = execute_fast_global_registration(source_down, target_down,
                                                    source_fpfh, target_fpfh,
                                                    voxel_size)
        print("Fast global registration took %.3f sec.\n" % (time.time() - start))
        print(result_fast)
        print("Final transformation from global registration is:")
        print(result_fast.transformation)
        initial_transformation = result_fast.transformation

    draw_registration_result(source_down, target_down, initial_transformation)

    result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh, initial_transformation, voxel_size)
    print(result_icp)
    print("Refined transformation is:")
    print(result_icp.transformation)
    draw_registration_result(source_down, target_down, result_icp.transformation)

