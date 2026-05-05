import open3d as o3d
import numpy as np
import copy


# -------------------------------------------------------
# CONFIG
# -------------------------------------------------------

SCENE_FILE = "WALLR3.pcd"  # input scene containing the car
TEMPLATE_FILE = "rc_car_template.pcd"

VOXEL_SIZE = 0.2  # meters (adjust for your data)


# -------------------------------------------------------
# HELPERS
# -------------------------------------------------------

def preprocess_point_cloud(pcd, voxel_size):
    """
    Downsample and compute features.
    """

    print("Downsampling...")
    pcd_down = pcd.voxel_down_sample(voxel_size)

    print("Estimating normals...")
    radius_normal = voxel_size * 2

    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_normal,
            max_nn=30
        )
    )

    print("Computing FPFH features...")
    radius_feature = voxel_size * 5

    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_feature,
            max_nn=100
        )
    )

    return pcd_down, fpfh


def execute_global_registration(
    source_down,
    target_down,
    source_fpfh,
    target_fpfh,
    voxel_size
):
    """
    Coarse alignment using RANSAC feature matching.
    """

    distance_threshold = voxel_size * 1.5

    print("Running RANSAC global registration...")

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=distance_threshold,

        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),

        ransac_n=4,

        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold
            )
        ],

        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            4000000,
            500
        )
    )

    return result


def refine_registration(source, target, initial_transform, voxel_size):
    """
    Refine alignment using ICP.
    """

    distance_threshold = voxel_size * 0.4

    print("Running ICP refinement...")

    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        distance_threshold,
        initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    return result


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)

    source_temp.paint_uniform_color([1, 0, 0])  # red
    target_temp.paint_uniform_color([0, 1, 0])  # green

    source_temp.transform(transformation)

    o3d.visualization.draw_geometries([source_temp, target_temp])


# -------------------------------------------------------
# MAIN
# -------------------------------------------------------

def main():

    print("Loading point clouds...")

    scene = o3d.io.read_point_cloud(SCENE_FILE)
    template = o3d.io.read_point_cloud(TEMPLATE_FILE)

    print(f"Scene points: {len(scene.points)}")
    print(f"Template points: {len(template.points)}")

    # Preprocess
    scene_down, scene_fpfh = preprocess_point_cloud(scene, VOXEL_SIZE)
    template_down, template_fpfh = preprocess_point_cloud(template, VOXEL_SIZE)

    # Global registration
    result_ransac = execute_global_registration(
        template_down,
        scene_down,
        template_fpfh,
        scene_fpfh,
        VOXEL_SIZE
    )

    print("\nInitial transformation:")
    print(result_ransac.transformation)

    print(f"RANSAC fitness: {result_ransac.fitness}")
    print(f"RANSAC RMSE: {result_ransac.inlier_rmse}")

    # ICP refinement
    result_icp = refine_registration(
        template,
        scene,
        result_ransac.transformation,
        VOXEL_SIZE
    )

    print("\nRefined transformation:")
    print(result_icp.transformation)

    print(f"ICP fitness: {result_icp.fitness}")
    print(f"ICP RMSE: {result_icp.inlier_rmse}")

    # Visualize
    draw_registration_result(
        template,
        scene,
        result_icp.transformation
    )


if __name__ == "__main__":
    main()