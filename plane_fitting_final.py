import pyzed.sl as sl
import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree
import os
from tqdm import tqdm

def knn_search(pc_np, k):
    tree = cKDTree(pc_np)
    _, indices = tree.query(pc_np, k=k+1)
    return indices[:, 1:]

def smooth_point_cloud_numpy(points, k=20, iterations=1):
    smoothed = points.copy()
    for _ in range(iterations):
        knns = knn_search(smoothed, k)
        smoothed = np.mean(smoothed[knns], axis=1)
    return smoothed


def process_point_cloud(pc_data, distance_threshold=0.005, pothole_distance=0.05, outlier_percentile=7):
    pc_data = pc_data.get_data()
    pc_data = np.array(pc_data, copy=True)
    points = pc_data[..., :3].reshape(-1, 3)

    # Filter out invalid points
    valid_mask = np.isfinite(points).all(axis=1)
    points = points[valid_mask]

    # Create point cloud and downsample
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd = pcd.voxel_down_sample(voxel_size=0.005)

    # Remove outliers
    pcd, ind = pcd.remove_radius_outlier(nb_points=20, radius=0.03)

    # Smooth
    #print("smoothing..")
    smoothed = smooth_point_cloud_numpy(np.asarray(pcd.points), k=10, iterations=5)
    pcd.points = o3d.utility.Vector3dVector(smoothed)

    # RANSAC plane fitting
    #print("plane fitting..")
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                             ransac_n=3,
                                             num_iterations=1000)
    a, b, c, d = plane_model
    normal = np.array([a, b, c], dtype=float)
    norm = np.linalg.norm(normal)
    normal /= norm
    d /= norm

    # Compute distances from plane
    points_np = np.asarray(pcd.points)
    #distances = np.abs(points_np @ normal + d)
    distances =points_np @ (-normal) + d # <<minus>> because i noticed that the plane was fitted upside down 
    distances -= np.mean(distances)   # shift so avg = 0

    # Statistics
    min_d = np.min(distances)
    max_d = np.max(distances)
    avg_d = np.mean(distances)

    # Thresholds for lowest and highest points
    pothole_threshold = np.percentile(distances, outlier_percentile)      # low points
    bump_threshold = np.percentile(distances, 100 - outlier_percentile)   # high points

    # stricter Thresholds to decide wether there is a bump/pothole
    pothole_threshold4 = np.percentile(distances, 4)      # low points
    bump_threshold4 = np.percentile(distances, 100 - 4)   # high points

    # Differences from mean (optional, for your printout)
    diff_pothole = avg_d - pothole_threshold4
    diff_bump = avg_d - bump_threshold4

    # Print stats
    '''
    print("Min Distance:", min_d)
    print("Max Distance:", max_d)
    print("Avg Distance:", avg_d)
    print("Threshold for pothole (distance):", pothole_threshold4)
    print("Threshold for bump (distance):", bump_threshold4)
    print("Difference (avg - pothole threshold):", diff_pothole)
    print("Difference (avg - bump threshold):", diff_bump)'''
    

    # Standard deviation for detection
    std = np.std(distances)
    #print("std :", std)

    # Check for potholes and bumps
    pothole_points_mask4 = distances <= pothole_threshold4
    bump_points_mask4 = distances >= bump_threshold4

    # final mask after using the stricter mask for check
    pothole_points_mask = distances <= pothole_threshold
    bump_points_mask = distances >= bump_threshold


    # Default = all gray
    status = "flat"
    colors_gray = np.tile([0.6, 0.6, 0.6], (len(points_np), 1))

    # If std is too small, skip detection (everything gray)
    if std < 0.01:
        merged_points = points_np
        merged_colors = colors_gray
    elif std > 0.1:
        merged_points = points_np
        merged_colors = colors_gray
    else:
        # Check for potholes and bumps
        pothole_points_mask4 = distances <= pothole_threshold4
        bump_points_mask4 = distances >= bump_threshold4

        # final mask after using the stricter mask for check
        pothole_points_mask = distances <= pothole_threshold
        bump_points_mask = distances >= bump_threshold

        has_pothole = len(distances[pothole_points_mask4]) > 0 and abs(np.mean(distances[pothole_points_mask4])) > 2.2 * std
        has_bump = len(distances[bump_points_mask4]) > 0 and abs(np.mean(distances[bump_points_mask4])) > 2.5 * std

        if has_pothole and not has_bump:
            status = "pothole"
        elif has_bump and not has_pothole:
            status = "bump"
        elif has_pothole and has_bump:
            status = "both"

        #print("status: ", status)

        # Assign colors
        pothole_points = points_np[pothole_points_mask] if has_pothole else np.empty((0, 3))
        bump_points = points_np[bump_points_mask] if has_bump else np.empty((0, 3))
        background_mask = np.ones_like(distances, dtype=bool)

        if has_pothole:
            background_mask[pothole_points_mask] = False
        if has_bump:
            background_mask[bump_points_mask] = False

        background_points = points_np[background_mask]

        # remove outliers
        if len(pothole_points) > 0:
            pothole_pcd = o3d.geometry.PointCloud()
            pothole_pcd.points = o3d.utility.Vector3dVector(pothole_points)
            pothole_pcd, _ = pothole_pcd.remove_radius_outlier(nb_points=70, radius=0.025)
            pothole_points = np.asarray(pothole_pcd.points)

        if len(bump_points) > 0:
            bump_pcd = o3d.geometry.PointCloud()
            bump_pcd.points = o3d.utility.Vector3dVector(bump_points)
            bump_pcd, _ = bump_pcd.remove_radius_outlier(nb_points=70, radius=0.025)
            bump_points = np.asarray(bump_pcd.points)

        # Colors
        colors_red = np.tile([1, 0, 0], (len(pothole_points), 1))
        colors_blue = np.tile([0, 0, 1], (len(bump_points), 1))
        colors_gray = np.tile([0.6, 0.6, 0.6], (len(background_points), 1))

        merged_points = np.vstack([pothole_points, bump_points, background_points])
        merged_colors = np.vstack([colors_red, colors_blue, colors_gray])

    # Final result point cloud
    result_pcd = o3d.geometry.PointCloud()
    result_pcd.points = o3d.utility.Vector3dVector(merged_points)
    result_pcd.colors = o3d.utility.Vector3dVector(merged_colors)

    return result_pcd


def main():
    svo_path = "Villanova_potholes_dataset/HD2K_SN39967967_08-46-22.svo2"
    output_folder = "processed_frames"
    process_frame_sequence(svo_path, start_frame=64, end_frame=217, output_folder=output_folder)
    print("✅ All frames processed and saved as PLYs.")




def process_frame_sequence(svo_path, start_frame, end_frame, output_folder):
    # Make sure output folder exists
    os.makedirs(output_folder, exist_ok=True)

    zed = sl.Camera()
    input_type = sl.InputType()
    input_type.set_from_svo_file(svo_path)
    init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_units = sl.UNIT.METER

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open {svo_path}")
        return

    runtime_params = sl.RuntimeParameters()
    point_cloud = sl.Mat()

    total_frames = zed.get_svo_number_of_frames()
    print(f"Total frames in SVO: {total_frames}")
    print(f"Processing frames {start_frame} to {end_frame}")

    # Progress bar
    for frame_idx in tqdm(range(start_frame, end_frame + 1), desc="Processing frames"):
        zed.set_svo_position(frame_idx)

        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            # Process and save
            pcd = process_point_cloud(point_cloud)
            output_path = os.path.join(output_folder, f"frame_{frame_idx:04d}.ply")
            o3d.io.write_point_cloud(output_path, pcd)

    zed.close()


if __name__ == "__main__":
    main()



