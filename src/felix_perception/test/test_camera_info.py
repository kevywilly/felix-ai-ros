"""Pure-logic tests for camera_info parsing (U3). No ROS required."""
import textwrap

import numpy as np

from felix_perception import geometry


def _write(tmp_path, body):
    p = tmp_path / "ci.yaml"
    p.write_text(textwrap.dedent(body))
    return str(p)


def test_parse_camera_info_happy(tmp_path):
    path = _write(tmp_path, """
        image_width: 640
        image_height: 360
        camera_name: felix_csi
        camera_matrix:
          rows: 3
          cols: 3
          data: [530.0, 0.0, 320.0, 0.0, 525.0, 180.0, 0.0, 0.0, 1.0]
        distortion_coefficients:
          rows: 1
          cols: 5
          data: [0.1, -0.05, 0.0, 0.0, 0.0]
    """)
    intr = geometry.parse_camera_info_yaml(path)
    assert intr.width == 640
    assert intr.height == 360
    assert intr.name == "felix_csi"
    assert intr.fx == 530.0
    assert intr.fy == 525.0
    assert intr.cx == 320.0
    assert intr.cy == 180.0
    assert intr.D.shape == (5,)
    assert intr.K.shape == (3, 3)


def test_parse_camera_info_missing_distortion(tmp_path):
    # No distortion block at all -> empty D, not a crash.
    path = _write(tmp_path, """
        image_width: 320
        image_height: 240
        camera_matrix:
          rows: 3
          cols: 3
          data: [100.0, 0.0, 160.0, 0.0, 100.0, 120.0, 0.0, 0.0, 1.0]
    """)
    intr = geometry.parse_camera_info_yaml(path)
    assert intr.D.size == 0
    assert intr.width == 320


def test_committed_placeholder_parses():
    # The shipped placeholder must parse and be self-consistent (cx == width/2).
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    intr = geometry.parse_camera_info_yaml(
        os.path.join(here, "config", "camera_info.yaml"))
    assert intr.width == 640 and intr.height == 360
    assert intr.cx == intr.width / 2
    assert np.allclose(intr.D, 0.0)
