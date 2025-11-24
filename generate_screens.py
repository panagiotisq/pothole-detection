import os
import open3d as o3d
import numpy as np

def generate_screenshots_from_plys(ply_folder, img_folder, angle_deg=180):
    os.makedirs(img_folder, exist_ok=True)

    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)  # offscreen

    ply_files = sorted([f for f in os.listdir(ply_folder) if f.endswith(".ply")])
    if not ply_files:
        print("No PLY files found!")
        return

    # --- Rotation around X-axis ---
    theta = np.deg2rad(angle_deg)  # convert to radians
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta), -np.sin(theta)],
                    [0, np.sin(theta),  np.cos(theta)]])

    # --- Load first PLY to set camera ---
    first_ply = os.path.join(ply_folder, ply_files[0])
    pcd = o3d.io.read_point_cloud(first_ply)
    pcd.rotate(R_x, center=(0,0,0))  # apply rotation

    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    ctr = vis.get_view_control()
    parameters = ctr.convert_to_pinhole_camera_parameters()  # save camera params
    vis.remove_geometry(pcd)

    # --- Loop through all frames ---
    for idx, ply_file in enumerate(ply_files):
        ply_path = os.path.join(ply_folder, ply_file)
        pcd = o3d.io.read_point_cloud(ply_path)
        pcd.rotate(R_x, center=(0,0,0))  # rotate each frame

        vis.add_geometry(pcd)
        ctr.convert_from_pinhole_camera_parameters(parameters)
        vis.poll_events()
        vis.update_renderer()

        img_path = os.path.join(img_folder, f"frame_{idx:04d}.png")
        vis.capture_screen_image(img_path)
        vis.remove_geometry(pcd)
        print(f"Saved {img_path}")

    vis.destroy_window()

if __name__ == "__main__":
    ply_folder = "processed_frames"
    img_folder = "screenshots"
    generate_screenshots_from_plys(ply_folder, img_folder, angle_deg=160)  # try 90° for partial flip
