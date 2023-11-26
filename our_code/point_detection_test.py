import open3d as o3d
import numpy as np

def load_point_cloud(file_path):
    """Load a point cloud file."""
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def preprocess_point_cloud(pcd, voxel_size):
    """Preprocess the point cloud by downsampling using a voxel grid."""
    pcd_down = pcd.voxel_down_sample(voxel_size)
    return pcd_down

def cluster_point_cloud(pcd, eps=0.2, min_points=10):
    """Cluster the point cloud to detect separate objects."""
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
    return labels

def visualize_clusters(pcd, labels):
    """Visualize the clustered point cloud."""
    max_label = labels.max()
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0  # set noise to black
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd])

def generate_bounding_boxes(pcd, labels):
    """Generate bounding boxes for each cluster."""
    bounding_boxes = []
    for i in range(labels.max() + 1):
        points = np.asarray(pcd.points)[labels == i]
        if points.shape[0] < 3:
            continue
        bbox = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(points))
        bounding_boxes.append(bbox)
    return bounding_boxes

def main():
    pcd = load_point_cloud("path_to_your_lidar_data.ply")
    pcd_down = preprocess_point_cloud(pcd, voxel_size=0.05)
    labels = cluster_point_cloud(pcd_down)
    visualize_clusters(pcd_down, labels)
    bboxes = generate_bounding_boxes(pcd_down, labels)
    for bbox in bboxes:
        print("Bounding box:", bbox.get_extent())

if __name__ == "__main__":
    main()
