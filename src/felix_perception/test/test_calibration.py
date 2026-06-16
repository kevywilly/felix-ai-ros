"""Pure-logic tests for headless calibration helpers (no ROS).

board_params and write_camera_info live in calibrate_camera; that module imports
cv2 + numpy at top (both available without ROS) but NOT rclpy at import time
beyond the node class, so importing the helpers is safe here.
"""
import numpy as np

from felix_perception.calibrate_camera import board_params, write_camera_info
from felix_perception import geometry


def _synthetic_board(bw, bh, w, h, s):
    """Row-major corners (j*bw + i) for a centered, axis-aligned board."""
    cx0 = w / 2 - (bw - 1) * s / 2
    cy0 = h / 2 - (bh - 1) * s / 2
    pts = [[cx0 + i * s, cy0 + j * s] for j in range(bh) for i in range(bw)]
    return np.array(pts, np.float32).reshape(-1, 1, 2)


def test_board_params_centered_frontal():
    corners = _synthetic_board(8, 6, 640, 360, 30)
    x, y, size, skew = board_params(corners, 640, 360, 8, 6)
    assert abs(x - 0.5) < 1e-6 and abs(y - 0.5) < 1e-6
    assert abs(skew) < 1e-6          # axis-aligned -> no skew
    assert 0.0 < size < 1.0


def test_board_params_off_center():
    corners = _synthetic_board(8, 6, 640, 360, 30)
    corners[:, 0, 0] += 100          # shift right
    x, _, _, _ = board_params(corners, 640, 360, 8, 6)
    assert x > 0.5


def test_write_camera_info_roundtrips(tmp_path):
    K = np.array([[530.0, 0.0, 320.0],
                  [0.0, 525.0, 180.0],
                  [0.0, 0.0, 1.0]])
    D = np.array([0.1, -0.05, 0.001, 0.002, 0.0])
    path = str(tmp_path / "ci.yaml")
    write_camera_info(path, 640, 360, K, D, name="felix_csi")

    intr = geometry.parse_camera_info_yaml(path)
    assert intr.width == 640 and intr.height == 360
    assert intr.name == "felix_csi"
    assert np.allclose(intr.K, K)
    assert np.allclose(intr.D, D)
