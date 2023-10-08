import pcl

def extract_person_from_point_cloud(cloud):
    """
    When we first tackled the problem of extracting a person from a point cloud, we realized that 
    the height of a person could be a good initial filter. So, this function first filters out points 
    based on height (z-axis) and then clusters them to identify individual objects. Our hope is to find
    the person as the largest cluster.
    """
    
    # We decided to use a passthrough filter because we noticed many irrelevant points
    # in the scene (like the ground, and sky). By focusing on a certain height range, 
    # we hoped to eliminate a lot of noise.
    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name("z")
    passthrough.set_filter_limits(0.5, 2.5)  # These limits worked for most adults in our initial tests.
    cloud_filtered = passthrough.filter()

    # Clustering is powerful. By using a kd-tree to organize the data and then segmenting
    # the point cloud into individual clusters, we can treat each cluster as a potential person.
    tree = cloud_filtered.make_kdtree()
    ec = cloud_filtered.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)  # We found 2cm to be a good balance between precision and performance.
    ec.set_MinClusterSize(100)  
    ec.set_MaxClusterSize(25000)  
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    
    # The idea here is simple: the person is likely to be the biggest object standing. 
    # So, we try to find the largest cluster to identify the person.
    max_size = 0
    largest_cluster = None
    for j, indices in enumerate(cluster_indices):
        if len(indices) > max_size:
            max_size = len(indices)
            largest_cluster = indices

    person_cloud = cloud_filtered.extract(largest_cluster)

    return person_cloud

def track_person_in_next_frame(previous_person_cloud, next_frame_cloud):
    """
    Tracking was a bit tricky. Instead of using any advanced tracking algorithm, 
    we chose a simpler approach. By comparing the center of the detected person in the 
    last frame with the clusters in the current frame, we can make a good guess of 
    where the person moved.
    """
    
    # Getting the centroid helps in identifying where the person was in the last frame.
    centroid = np.mean(previous_person_cloud.to_array(), axis=0)

    # Using the same clustering technique, we can identify potential objects in the current frame.
    tree = next_frame_cloud.make_kdtree()
    ec = next_frame_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Now, we try to find the closest cluster in the current frame to where the person was in the last frame.
    closest_cluster = None
    min_distance = float('inf')
    for indices in cluster_indices:
        cluster_cloud = next_frame_cloud.extract(indices)
        cluster_centroid = np.mean(cluster_cloud.to_array(), axis=0)
        distance = np.linalg.norm(centroid - cluster_centroid)
        if distance < min_distance:
            min_distance = distance
            closest_cluster = indices

    next_person_cloud = next_frame_cloud.extract(closest_cluster)

    return next_person_cloud

# To use this code, load your point cloud, call the 'extract_person_from_point_cloud' function first 
# and then the 'track_person_in_next_frame' function for subsequent frames.
