import pyzed.sl as sl
import cv2
import open3d as o3d
import numpy as np
import os
from tqdm import tqdm

def project_points_to_image(points, intrinsics):
    """
    Converts 3D points (X, Y, Z) to 2D pixels (u, v)
    using the Pinhole Camera Model.
    """
    fx = intrinsics.get_camera_information().camera_configuration.calibration_parameters.left_cam.fx
    fy = intrinsics.get_camera_information().camera_configuration.calibration_parameters.left_cam.fy
    cx = intrinsics.get_camera_information().camera_configuration.calibration_parameters.left_cam.cx
    cy = intrinsics.get_camera_information().camera_configuration.calibration_parameters.left_cam.cy

    # Unpacking
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Avoid division by zero
    z[z == 0] = 0.001

    # Pinhole equations
    u = (x * fx / z) + cx
    v = (y * fy / z) + cy

    return np.stack([u, v], axis=1).astype(int)

def create_video_from_plys(svo_path, ply_folder, output_video, start_frame, end_frame):
    # 1. Initialize ZED (only to retrieve Intrinsics and original images)
    zed = sl.Camera()
    input_type = sl.InputType()
    input_type.set_from_svo_file(svo_path)
    init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)
    init_params.depth_mode = sl.DEPTH_MODE.NONE  # We don't need depth from ZED, we have it in the PLYs
    
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open SVO")
        return

    # Get dimensions and Video Writer
    image_size = zed.get_camera_information().camera_configuration.resolution
    width, height = image_size.width, image_size.height
    
    # --- CHANGE 1: Half speed ---
    # Set to 10 fps instead of 30 to make it slower and better visualize detection
    fps = 10
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    image_zed = sl.Mat()
    
    print(f"Processing frames {start_frame} to {end_frame} using PLYs from: {ply_folder}")

    for frame_idx in tqdm(range(start_frame, end_frame + 1)):
        # --- A. Retrieve original image from SVO ---
        zed.set_svo_position(frame_idx)
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            frame_bgra = image_zed.get_data()
            frame_bgr = frame_bgra[:, :, :3].copy() # Convert to BGR for OpenCV
            frame_bgr = np.ascontiguousarray(frame_bgr)
        else:
            continue

        # --- B. Retrieve corresponding PLY ---
        ply_name = f"frame_{frame_idx:04d}.ply"
        ply_path = os.path.join(ply_folder, ply_name)

        if not os.path.exists(ply_path):
            # If PLY is missing for a frame, write raw video and continue
            out.write(frame_bgr)
            continue

        # Load Point Cloud
        pcd = o3d.io.read_point_cloud(ply_path)
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)

        if len(points) == 0:
            out.write(frame_bgr)
            continue

        # --- C. Filtering: Keep ONLY the colored points ---
        color_variance = np.var(colors, axis=1)
        mask_colorful = color_variance > 0.01

        points_to_draw = points[mask_colorful]
        colors_to_draw = colors[mask_colorful]

        if len(points_to_draw) > 0:
            # --- D. Reprojection ---
            pixels = project_points_to_image(points_to_draw, zed)
            colors_bgr = (colors_to_draw[:, [2, 1, 0]] * 255).astype(np.uint8)

            # --- E. Drawing on Frame (Mask Overlay) ---
            overlay = frame_bgr.copy()
            
            valid_pixels = (pixels[:, 0] >= 0) & (pixels[:, 0] < width) & \
                           (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
            
            final_pixels = pixels[valid_pixels]
            final_colors = colors_bgr[valid_pixels]

            # --- CHANGE 2: More intense and dense marking ---
            # Increased radius from 3 to 6 to fill gaps
            for (u, v), c in zip(final_pixels, final_colors):
                cv2.circle(overlay, (u, v), 6, (int(c[0]), int(c[1]), int(c[2])), -1)

            # --- CHANGE 3: More intense colors ---
            # Changed weight (alpha) from 0.4 to 0.5 to make color appear more clearly
            cv2.addWeighted(overlay, 0.5, frame_bgr, 0.5, 0, frame_bgr)

        # Write to video
        out.write(frame_bgr)

    out.release()
    zed.close()
    print(f"Video saved as {output_video}")

if __name__ == "__main__":
    svo_file = "Villanova_potholes_dataset/HD2K_SN39967967_08-46-22.svo2"
    ply_folder_path = "processed_frames"
    output_filename = "output_overlayed.mp4"
    
    create_video_from_plys(svo_file, ply_folder_path, output_filename, start_frame=64, end_frame=217)