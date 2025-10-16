"""
Unit tests for calibration_math module in src.

Covers transformation estimation and quaternion averaging.
"""
from __future__ import annotations
import numpy as np
import pytest
from lidar_camera_calibration_py.src import calibration_math

def test_estimate_transform_identity() -> None:
    """
    Test estimate_transform returns identity rotation and zero translation for identical point sets.
    """
    points = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    R, t = calibration_math.estimate_transform(points, points)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-7)
    np.testing.assert_allclose(t, np.zeros(3), atol=1e-7)

def test_estimate_transform_translation() -> None:
    """
    Test estimate_transform returns correct translation for shifted point sets.
    """
    cam = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    shift = np.array([10, 0, 0])
    lidar = cam + shift
    R, t = calibration_math.estimate_transform(cam, lidar)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-7)
    np.testing.assert_allclose(t, -shift, atol=1e-7)

def test_average_quaternions_identical() -> None:
    """
    Test average_quaternions returns the same quaternion if all inputs are identical.
    """
    q = np.array([1, 0, 0, 0])
    avg = calibration_math.average_quaternions([q, q, q])
    np.testing.assert_allclose(np.abs(avg), np.abs(q), atol=1e-7)

def test_average_quaternions_zeros() -> None:
    """
    Test average_quaternions handles all-zero input gracefully.
    """
    q = np.zeros(4)
    avg = calibration_math.average_quaternions([q, q, q])
    assert avg.shape == (4,)
    assert np.all(np.isnan(avg)) or np.allclose(avg, 0)

