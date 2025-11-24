import open3d as o3d
import numpy as np
'''
def main():
    # Load the saved PLY file
    # ply_file = "processed_frame95_colored.ply"
    ply_file = "processed_frame261_colored.ply"

    print(f"Loading point cloud from: {ply_file}")
    pcd = o3d.io.read_point_cloud(ply_file)

    if not pcd.has_points():
        print("Error: No points found in the file.")
        return

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    print("Original point cloud has", len(points), "points")

    # Identify red points (potholes)
    red_mask = (colors[:, 0] > 0.9) & (colors[:, 1] < 0.1) & (colors[:, 2] < 0.1)
    red_points = points[red_mask]
    red_colors = colors[red_mask]

    gray_points = points[~red_mask]
    gray_colors = colors[~red_mask]

    print("Red points before cleaning:", len(red_points))

    # Create point cloud for red points
    red_pcd = o3d.geometry.PointCloud()
    red_pcd.points = o3d.utility.Vector3dVector(red_points)
    red_pcd.colors = o3d.utility.Vector3dVector(red_colors)

    # --- Outlier removal only on red subset ---
    #cleaned_red_pcd, _ = red_pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=0.2)
    cleaned_red_pcd, _ = red_pcd.remove_radius_outlier(nb_points=70, radius=0.025)

    cleaned_red_points = np.asarray(cleaned_red_pcd.points)
    cleaned_red_colors = np.asarray(cleaned_red_pcd.colors)

    print("Red points after cleaning:", len(cleaned_red_points))

    # Merge cleaned red + untouched gray
    merged_points = np.vstack([cleaned_red_points, gray_points])
    merged_colors = np.vstack([cleaned_red_colors, gray_colors])

    cleaned_pcd = o3d.geometry.PointCloud()
    cleaned_pcd.points = o3d.utility.Vector3dVector(merged_points)
    cleaned_pcd.colors = o3d.utility.Vector3dVector(merged_colors)

    # Save result
    output_file = ply_file.replace(".ply", "_red_cleaned.ply")
    o3d.io.write_point_cloud(output_file, cleaned_pcd)
    print(f"Saved cleaned point cloud to: {output_file}")

    # Visualize comparison
    o3d.visualization.draw_geometries([pcd], window_name="Original Point Cloud")
    o3d.visualization.draw_geometries([cleaned_pcd], window_name="Red-Only Cleaned Point Cloud")
'''

def main():
    # Load the saved PLY file
    #ply_file = "processed_frame95_colored.ply"
    #ply_file = "processed_frame261_colored.ply"
    #ply_file = "processed_frame200_colored.ply"
    #ply_file = "processed_frame269_colored.ply"
    ply_file = "processed_frame64_colored.ply"
    #ply_file = "processed_frame249_colored.ply"
    ply_file = "processed_frame216_colored.ply"



    print(f"Loading point cloud from: {ply_file}")
    pcd = o3d.io.read_point_cloud(ply_file)

    # Check if it's loaded properly
    if not pcd.has_points():
        print("Error: No points found in the file.")
        return
    
    print(pcd.has_normals())  # Should return True


    # Visualize it
    o3d.visualization.draw_geometries([pcd],
                                      window_name="PLY Viewer",
                                      width=1280,
                                      height=720,
                                      point_show_normal=False)
    
if __name__ == "__main__":
    main()
