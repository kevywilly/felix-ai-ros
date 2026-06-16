"""Pure-logic tests for back-projection + rigid transforms (U6). No ROS."""
import math

import numpy as np

from felix_perception import geometry

# fx = fy = 500, principal point at (320, 180); no distortion.
K = np.array([[500.0, 0.0, 320.0],
              [0.0, 500.0, 180.0],
              [0.0, 0.0, 1.0]])
D = np.zeros(5)


def test_center_column_zero_bearing():
    tmin, tmax = geometry.bbox_bearings(320.0, 320.0, 180.0, K, D)
    assert abs(tmin) < 1e-9 and abs(tmax) < 1e-9


def test_symmetric_bbox_symmetric_bearings():
    tmin, tmax = geometry.bbox_bearings(220.0, 420.0, 180.0, K, D)
    # columns +/-100px from cx -> bearings atan(100/500) each side, symmetric.
    assert math.isclose(tmin, -tmax, abs_tol=1e-9)
    assert math.isclose(tmax, math.atan(100 / 500), abs_tol=1e-6)


def test_one_focal_length_offset_is_45_degrees():
    # a column fx pixels right of cx -> atan(1) = 45 deg.
    _, tmax = geometry.bbox_bearings(320.0, 820.0, 180.0, K, D)
    assert math.isclose(tmax, math.pi / 4, abs_tol=1e-6)


def test_optical_bearing_sign():
    assert geometry.optical_bearing(0.0, 1.0) == 0.0
    assert geometry.optical_bearing(1.0, 1.0) > 0      # +x right -> positive
    assert geometry.optical_bearing(-1.0, 1.0) < 0


def test_transform_points_identity_is_translation():
    pts = [[1.0, 2.0, 0.0], [3.0, 4.0, 0.0]]
    out = geometry.transform_points(pts, (10.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert np.allclose(out, [[11.0, 2.0, 0.0], [13.0, 4.0, 0.0]])


def test_transform_points_yaw_90():
    # +90 deg about z: (1,0,0) -> (0,1,0).
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    out = geometry.transform_points([[1.0, 0.0, 0.0]], (0.0, 0.0, 0.0), q)
    assert np.allclose(out[0], [0.0, 1.0, 0.0], atol=1e-9)
