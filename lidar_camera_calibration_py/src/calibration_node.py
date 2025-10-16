from __future__ import annotations
import rospy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import cv2
from lidar_camera_calibration_py.src.config_utils import parse_config_file, parse_marker_coordinates
from lidar_camera_calibration_py.src.calibration_math import estimate_transform
import os
from typing import Any, Tuple, Optional

class LidarCameraCalibrationNode:
    def __init__(self) -> None:
        """
        Initialize the LiDAR-Camera Calibration Node, load configuration, and subscribe to topics.
        """
        self.bridge = CvBridge()
        # Load configuration files
        config_path = os.path.join(os.path.dirname(__file__), '../../conf/config_file.txt')
        marker_path = os.path.join(os.path.dirname(__file__), '../../conf/marker_coordinates.txt')
        self.config = parse_config_file(config_path)
        self.marker_boards = parse_marker_coordinates(marker_path)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)
        rospy.loginfo('LidarCameraCalibrationNode initialized.')
        # TODO: Add calibration logic, marker/corner detection, transformation estimation

    def image_callback(self, msg: Image) -> None:
        """
        Callback for camera image topic. Detects ArUco markers in the image.

        Args:
            msg (Image): ROS Image message.
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Example: Detect ArUco markers (replace with your board/marker detection logic)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=aruco_params)
        if ids is not None:
            rospy.loginfo(f'Detected ArUco markers: {ids.flatten()}')
            cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
        else:
            rospy.loginfo('No ArUco markers detected.')
        # TODO: Use corners for calibration, visualize or store as needed

    def lidar_callback(self, msg: PointCloud2) -> None:
        """
        Callback for LiDAR point cloud topic. Filters and processes point cloud data.

        Args:
            msg (PointCloud2): ROS PointCloud2 message.
        """
        # Convert PointCloud2 to numpy array
        import sensor_msgs.point_cloud2 as pc2
        points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        filtered_points = [p for p in points if self.config['x_min'] <= p[0] <= self.config['x_max'] and
                                             self.config['y_min'] <= p[1] <= self.config['y_max'] and
                                             self.config['z_min'] <= p[2] <= self.config['z_max'] and
                                             p[3] >= self.config['cloud_intensity_threshold']]
        rospy.loginfo(f'Received lidar point cloud. Filtered points: {len(filtered_points)}')
        # TODO: Use filtered_points for calibration

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """
        Callback for camera info topic. Stores camera intrinsic parameters.

        Args:
            msg (CameraInfo): ROS CameraInfo message.
        """
        # TODO: Store camera intrinsic parameters
        rospy.loginfo('Received camera info.')

    def perform_calibration(self, camera_points: list[Any], lidar_points: list[Any]) -> Tuple[Optional[Any], Optional[Any]]:
        """
        Estimate the transformation between camera and LiDAR frames using corresponding points.

        Args:
            camera_points (list[Any]): List of 3D points in the camera frame.
            lidar_points (list[Any]): List of 3D points in the LiDAR frame.

        Returns:
            tuple[Optional[Any], Optional[Any]]: Estimated rotation matrix and translation vector, or (None, None) if insufficient points.
        """
        if len(camera_points) != len(lidar_points) or len(camera_points) == 0:
            rospy.logwarn('Insufficient or mismatched points for calibration.')
            return None, None
        R_est, t_est = estimate_transform(camera_points, lidar_points)
        rospy.loginfo(f'Estimated rotation:\n{R_est}')
        rospy.loginfo(f'Estimated translation: {t_est}')
        # TODO: Save or publish results as needed
        return R_est, t_est

if __name__ == '__main__':
    rospy.init_node('lidar_camera_calibration_py_node')
    node = LidarCameraCalibrationNode()
    rospy.spin()
