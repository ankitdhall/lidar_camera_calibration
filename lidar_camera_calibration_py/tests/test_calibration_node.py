"""
Unit tests for LidarCameraCalibrationNode in src.

Uses mocking to test initialization and callback stubs.
"""
from __future__ import annotations
import pytest
from unittest.mock import patch, MagicMock
import sys

@pytest.fixture(autouse=True)
def patch_ros(monkeypatch):
    monkeypatch.setitem(sys.modules, 'rospy', MagicMock())
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', MagicMock())
    monkeypatch.setitem(sys.modules, 'cv_bridge', MagicMock())
    monkeypatch.setitem(sys.modules, 'cv2', MagicMock())
    monkeypatch.setitem(sys.modules, 'sensor_msgs.point_cloud2', MagicMock())
    monkeypatch.setitem(sys.modules, 'src.config_utils', MagicMock())
    monkeypatch.setitem(sys.modules, 'src.calibration_math', MagicMock())
    yield

def test_node_init():
    """
    Test LidarCameraCalibrationNode initializes without error.
    """
    from lidar_camera_calibration_py.src.calibration_node import LidarCameraCalibrationNode
    node = LidarCameraCalibrationNode()
    assert node is not None

def test_callbacks():
    """
    Test that callbacks can be called without error (with mocks).
    """
    from lidar_camera_calibration_py.src.calibration_node import LidarCameraCalibrationNode
    node = LidarCameraCalibrationNode()
    node.image_callback(MagicMock())
    node.lidar_callback(MagicMock())
    node.camera_info_callback(MagicMock())

