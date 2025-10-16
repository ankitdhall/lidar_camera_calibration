"""
Unit tests for config_utils module in src.

Tests configuration file and marker coordinates parsing, including error handling.
"""
from __future__ import annotations
import pytest
from lidar_camera_calibration_py.src import config_utils
import os

def test_parse_config_file() -> None:
    """
    Test parsing of the main configuration file.
    """
    test_path = os.path.join(os.path.dirname(__file__), '../../conf/config_file.txt')
    params = config_utils.parse_config_file(test_path)
    assert 'image_width' in params
    assert 'number_of_markers' in params
    assert isinstance(params['image_width'], float)
    assert isinstance(params['number_of_markers'], int)

def test_parse_marker_coordinates() -> None:
    """
    Test parsing of the marker coordinates file.
    """
    test_path = os.path.join(os.path.dirname(__file__), '../../conf/marker_coordinates.txt')
    boards = config_utils.parse_marker_coordinates(test_path)
    assert isinstance(boards, list)
    assert len(boards) > 0
    assert 'length' in boards[0]
    assert isinstance(boards[0]['length'], float)

def test_parse_config_file_missing() -> None:
    """
    Test parse_config_file raises FileNotFoundError for missing file.
    """
    with pytest.raises(FileNotFoundError):
        config_utils.parse_config_file('nonexistent_config.txt')

def test_parse_marker_coordinates_malformed(tmp_path) -> None:
    """
    Test parse_marker_coordinates handles malformed file gracefully.
    """
    malformed = tmp_path / "malformed.txt"
    malformed.write_text("not a number\nfoo\nbar\n")
    with pytest.raises(Exception):
        config_utils.parse_marker_coordinates(str(malformed))
