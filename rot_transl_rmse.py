import numpy as np
from scipy.spatial.transform import Rotation as R

import numpy as np
#import cv2

def calculate_errors(source_points, target_points):
    # Calculate the centroid of the source and target points
    source_centroid = np.mean(source_points, axis=0)
    target_centroid = np.mean(target_points, axis=0)

    # Calculate the covariance matrix of the source and target points
    source_covariance = np.cov(source_points.T)
    target_covariance = np.cov(target_points.T)

    # Calculate the singular value decomposition of the covariance matrices
    U_source, S_source, Vt_source = np.linalg.svd(source_covariance)
    U_target, S_target, Vt_target = np.linalg.svd(target_covariance)

    # Calculate the rotation matrix
    R = Vt_target.T @ U_source.T

    # Calculate the translation vector
    t = target_centroid - R @ source_centroid

    # Calculate the mean squared error
    mse = np.mean(np.sum((target_points - (source_points @ R.T + t))**2, axis=1))

    return R, t, mse

# Example usage:
source_points = np.random.rand(1000, 3)
target_points = np.random.rand(5000, 3)

R, t, mse = calculate_errors(source_points, target_points)

print(f"Rotation matrix:\n{R}")
print(f"Translation vector:\n{t}")
print(f"Mean squared error: {mse}")


def compute_registration_error(source, target, transformation):
    # Apply transformation to source point cloud
    source_transformed = np.dot(transformation[:3, :3], source.T).T + transformation[:3, 3]
    # Compute the difference between the transformed source and target point clouds
    diff = target - source_transformed
    # Compute the root mean square error (RMSE) of the difference
    rmse = np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))
    # Compute the rotational error using quaternions
    r = R.from_matrix(transformation[:3, :3])
    q = r.as_quat()
    q_target = R.from_matrix(np.identity(3)).as_quat()
    rot_err = np.arccos(np.abs(np.dot(q, q_target))) * 180 / np.pi
    # Compute the translational error
    trans_err = np.linalg.norm(transformation[:3, 3] - np.array([0, 0, 0]))
    return rmse, rot_err, trans_err

'''This code uses NumPy and SciPy libraries to calculate 
the root mean square error (RMSE), rotational error 
and translational error between two point clouds.
The function compute_registration_error takes 
three arguments: source, target, and transformation. 
The source and target arguments are NumPy arrays 
representing the two point clouds. 
The transformation argument is a 4x4 homogeneous transformation matrix 
that maps points from the source point cloud to the target point cloud.

The RMSE is calculated as the square root of the mean of the sum 
of squared differences between corresponding points in 
the transformed source and target point clouds. 
The rotational error is calculated using quaternions. 
The translational error is calculated as the Euclidean distance 
between the translation vector in the transformation matrix 
and the origin.'''