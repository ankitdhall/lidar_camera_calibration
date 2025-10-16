from __future__ import annotations
from typing import Any, Dict

def parse_config_file(config_path: str) -> dict[str, Any]:
    """Parse the main configuration file for calibration parameters.

    Args:
        config_path (str): Path to the configuration file.

    Returns:
        dict[str, Any]: Dictionary of configuration parameters.
    """
    params: Dict[str, Any] = {}
    with open(config_path, 'r') as f:
        lines = [line.strip() for line in f if line.strip() and not line.startswith('#')]
    # Parse lines according to README.md spec
    params['image_width'], params['image_height'] = map(float, lines[0].split())
    params['x_min'], params['x_max'] = map(float, lines[1].split())
    params['y_min'], params['y_max'] = map(float, lines[2].split())
    params['z_min'], params['z_max'] = map(float, lines[3].split())
    params['cloud_intensity_threshold'] = float(lines[4])
    params['number_of_markers'] = int(lines[5])
    params['use_camera_info_topic'] = int(lines[6])
    params['fx'], _, params['cx'], _ = map(float, lines[7].split())
    _, params['fy'], params['cy'], _ = map(float, lines[8].split())
    _, _, _, _ = map(float, lines[9].split()) # 3x3 matrix, only first row used
    params['MAX_ITERS'] = int(lines[10])
    params['initial_rot_x'], params['initial_rot_y'], params['initial_rot_z'] = map(float, lines[11].split())
    params['lidar_type'] = int(lines[12])
    return params

def parse_marker_coordinates(marker_path: str) -> list[dict[str, float]]:
    """Parse the marker coordinates file for board definitions.

    Args:
        marker_path (str): Path to the marker coordinates file.

    Returns:
        list[dict[str, float]]: List of board definitions with dimensions and properties.
    """
    boards: List[Dict[str, float]] = []
    with open(marker_path, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]
    N = int(lines[0])
    for i in range(N):
        idx = 1 + i*5
        board = {
            'length': float(lines[idx]),
            'breadth': float(lines[idx+1]),
            'border_width_length': float(lines[idx+2]),
            'border_width_breadth': float(lines[idx+3]),
            'aruco_edge_length': float(lines[idx+4]),
        }
        boards.append(board)
    return boards
