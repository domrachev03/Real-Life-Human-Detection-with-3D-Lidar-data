import numpy as np
import cv2

def find_3d_position_fisheye(img1_points, img2_points, mtx_l, dist_l, mtx_r, dist_r, R, T):
    img1_points = np.array(img1_points, dtype='float32').reshape(-1, 1, 2)
    img2_points = np.array(img2_points, dtype='float32').reshape(-1, 1, 2)

    img1_points_undistorted = cv2.fisheye.undistortPoints(img1_points, mtx_l, dist_l, None, mtx_l)
    img2_points_undistorted = cv2.fisheye.undistortPoints(img2_points, mtx_r, dist_r, None, mtx_r)

    # Compute the projection matrices directly
    P1 = mtx_l @ np.hstack((np.eye(3), np.zeros((3, 1))))
    P2 = mtx_r @ np.hstack((R, T))

    points_4d_hom = cv2.triangulatePoints(P1, P2, img1_points_undistorted, img2_points_undistorted)

    # Convert from homogeneous coordinates to 3D
    points_3d = points_4d_hom / points_4d_hom[3,:]

    return points_3d[:3].T

# Example usage for fisheye cameras with assumed distortion:
# Define camera matrices, distortion coefficients, rotation matrix, and translation vector
# These should be obtained through the fisheye camera calibration process
mtx_l = np.array([[1000, 0, 320],
                  [0, 1000, 240],
                  [0, 0, 1]], dtype='float64')  # Intrinsic matrix of the left fisheye camera
dist_l = np.array([-0.2, 0.1, 0.01, -0.01], dtype='float64')  # Assumed distortion coefficients for the left camera

mtx_r = np.array([[1000, 0, 320],
                  [0, 1000, 240],
                  [0, 0, 1]], dtype='float64')  # Intrinsic matrix of the right fisheye camera
dist_r = np.array([-0.15, 0.09, 0.005, -0.005], dtype='float64')  # Assumed distortion coefficients for the right camera

R = np.eye(3)  # Identity matrix for rotation, assuming cameras are aligned
T = np.array([[0.1], [0], [0]])  # Translation of 10 cm along the x-axis

# Define points in pixel coordinates on the left and right images
img1_points = [(300, 200)]  # Points in the left image
img2_points = [(305, 200)]  # Points in the right image, with a slight shift to the right

# Compute the 3D position with distortion
object_3d_position_fisheye = find_3d_position_fisheye(img1_points, img2_points, mtx_l, dist_l, mtx_r, dist_r, R, T)
print("The object's 3D position with fisheye distortion is:", object_3d_position_fisheye)