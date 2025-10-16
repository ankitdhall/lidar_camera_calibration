from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Tuple, Any

def estimate_transform(camera_points: list[Any], lidar_points: list[Any]) -> Tuple[np.ndarray, np.ndarray]:
    """
    Estimate the rigid transformation (rotation and translation) that aligns lidar_points to camera_points.

    Args:
        camera_points (list[Any]): List of 3D points in the camera frame.
        lidar_points (list[Any]): List of 3D points in the LiDAR frame (corresponding to camera_points).

    Returns:
        tuple[np.ndarray, np.ndarray]:
            - R_est (np.ndarray): Estimated 3x3 rotation matrix.
            - t_est (np.ndarray): Estimated 3x1 translation vector.
    """
    assert len(camera_points) == len(lidar_points)
    camera_points = np.asarray(camera_points)
    lidar_points = np.asarray(lidar_points)
    centroid_camera = np.mean(camera_points, axis=0)
    centroid_lidar = np.mean(lidar_points, axis=0)
    camera_centered = camera_points - centroid_camera
    lidar_centered = lidar_points - centroid_lidar
    H = lidar_centered.T @ camera_centered
    U, S, Vt = np.linalg.svd(H)
    R_est = Vt.T @ U.T
    if np.linalg.det(R_est) < 0:
        Vt[-1, :] *= -1
        R_est = Vt.T @ U.T
    t_est = centroid_camera - R_est @ centroid_lidar
    return R_est, t_est

def average_quaternions(quaternions: list[np.ndarray]) -> np.ndarray:
    """
    Average a list of quaternions (Nx4 numpy array).

    Args:
        quaternions (list[np.ndarray]): List of quaternions to average (each as a 4-element array).

    Returns:
        np.ndarray: The average quaternion as a 4-element array.
    """
    Q = np.array(quaternions)
    M = Q.T @ Q
    eigvals, eigvecs = np.linalg.eigh(M)
    avg_quat = eigvecs[:, np.argmax(eigvals)]
    return avg_quat
