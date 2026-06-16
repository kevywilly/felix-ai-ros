"""Pure-logic helpers for felix_perception.

ROS-free by design (numpy / pyyaml at import time; cv2 imported lazily) so the
unit tests run without a sourced ROS environment. The nodes (detector_node,
fusion_node) wrap these functions in ROS message I/O. See the plan's
"pytest for pure logic only" test posture.

Contents:
  - CameraIntrinsics / parse_camera_info_yaml / load_camera_info  (U3)
  - engine_cache_path                                             (U4)
  - detections_to_records                                         (U5)
  - bbox_bearings / wedge_indices / cluster_nearest_range         (U6)
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import yaml


# --------------------------------------------------------------------------- #
# U3: camera intrinsics
# --------------------------------------------------------------------------- #
@dataclass
class CameraIntrinsics:
    """Parsed camera_info: K (3x3), D (n,), and image size."""
    width: int
    height: int
    K: np.ndarray
    D: np.ndarray
    name: str = ""

    @property
    def fx(self) -> float:
        return float(self.K[0, 0])

    @property
    def fy(self) -> float:
        return float(self.K[1, 1])

    @property
    def cx(self) -> float:
        return float(self.K[0, 2])

    @property
    def cy(self) -> float:
        return float(self.K[1, 2])


def parse_camera_info_yaml(path: str) -> CameraIntrinsics:
    """Parse a standard ROS camera_info YAML into a CameraIntrinsics.

    Tolerates a missing/empty distortion block (-> zero-length D)."""
    with open(path) as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"], dtype=float).reshape(3, 3)
    dist = data.get("distortion_coefficients") or {}
    D = np.array(dist.get("data") or [], dtype=float)
    return CameraIntrinsics(
        width=int(data["image_width"]),
        height=int(data["image_height"]),
        K=K,
        D=D,
        name=str(data.get("camera_name", "")),
    )


def load_camera_info(yaml_path: str, optical_frame: str):
    """Build a sensor_msgs/CameraInfo from a calibration YAML, stamped with the
    optical frame_id. Returns (CameraInfo, CameraIntrinsics).

    ROS import is lazy so the pure-logic tests don't need a sourced ROS env."""
    from sensor_msgs.msg import CameraInfo  # lazy: ROS-only

    intr = parse_camera_info_yaml(yaml_path)
    msg = CameraInfo()
    msg.header.frame_id = optical_frame
    msg.width = intr.width
    msg.height = intr.height
    msg.distortion_model = "plumb_bob"
    msg.k = intr.K.flatten().tolist()
    msg.d = intr.D.tolist()
    msg.r = np.eye(3).flatten().tolist()
    proj = np.zeros((3, 4))
    proj[:3, :3] = intr.K
    msg.p = proj.flatten().tolist()
    return msg, intr


# --------------------------------------------------------------------------- #
# U4: TensorRT engine cache location
# --------------------------------------------------------------------------- #
def engine_cache_path(weights, engine_dir: Optional[str] = None) -> str:
    """Where the built .engine for `weights` lives.

    Precedence for the directory: explicit `engine_dir` > $FELIX_ENGINE_DIR >
    ~/.cache/felix. The filename is the weights stem with a .engine suffix
    (e.g. 'yolo11n.pt' or '/models/yolo11n.pt' -> '<dir>/yolo11n.engine')."""
    stem = os.path.splitext(os.path.basename(str(weights)))[0]
    if engine_dir is None:
        engine_dir = os.environ.get("FELIX_ENGINE_DIR") \
            or os.path.expanduser(os.path.join("~", ".cache", "felix"))
    return os.path.join(engine_dir, stem + ".engine")


# --------------------------------------------------------------------------- #
# U5: YOLO outputs -> flat detection records
# --------------------------------------------------------------------------- #
@dataclass
class DetectionRecord:
    """A single detection as plain numbers (pixel bbox), ROS-free.

    The detector node maps this onto vision_msgs/Detection2D. Keeping the field
    math here lets the contract (string class_id, pixel center/size) be unit
    tested without a sourced ROS environment."""
    class_id: str
    score: float
    cx: float
    cy: float
    size_x: float
    size_y: float


def detections_to_records(xyxy, conf, cls, names=None) -> List["DetectionRecord"]:
    """Map raw YOLO outputs (xyxy pixels, conf, integer class) to records.

    `class_id` is ALWAYS a string (vision_msgs 4.x convention): the `names`
    lookup when available, else the stringified integer class."""
    xyxy = np.asarray(xyxy, dtype=float).reshape(-1, 4)
    conf = np.asarray(conf, dtype=float).reshape(-1)
    cls = np.asarray(cls).reshape(-1)
    records: List[DetectionRecord] = []
    for i in range(xyxy.shape[0]):
        x1, y1, x2, y2 = xyxy[i]
        c = int(cls[i])
        label = None
        if names is not None:
            try:
                label = names[c]
            except (KeyError, IndexError, TypeError):
                label = None
        if label is None:
            label = str(c)
        records.append(DetectionRecord(
            class_id=str(label),
            score=float(conf[i]),
            cx=(x1 + x2) / 2.0,
            cy=(y1 + y2) / 2.0,
            size_x=(x2 - x1),
            size_y=(y2 - y1),
        ))
    return records


# --------------------------------------------------------------------------- #
# U6: bearing-wedge lidar fusion (back-projection, transforms, clustering)
# --------------------------------------------------------------------------- #
def optical_bearing(x: float, z: float) -> float:
    """Horizontal bearing (rad) of an optical-frame point: +x right, +z forward.
    Zero looks straight ahead, positive to the right."""
    return math.atan2(x, z)


def bbox_bearings(x_left, x_right, y_row, K, D) -> Tuple[float, float]:
    """Horizontal bearings (rad, optical frame) of the bbox left/right columns,
    undistorted with K/D. Returns (theta_min, theta_max). cv2 is imported lazily
    so non-fusion code paths don't require it."""
    import cv2  # lazy: only the fusion path needs OpenCV's undistort
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    D = np.asarray(D, dtype=np.float64).reshape(-1)
    pts = np.array([[[float(x_left), float(y_row)]],
                    [[float(x_right), float(y_row)]]], dtype=np.float64)
    norm = cv2.undistortPoints(pts, K, D).reshape(-1, 2)
    thetas = np.arctan2(norm[:, 0], 1.0)
    return float(thetas.min()), float(thetas.max())


def quat_to_matrix(quaternion) -> np.ndarray:
    """3x3 rotation matrix from a (x, y, z, w) quaternion (assumed normalized)."""
    x, y, z, w = (float(v) for v in quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def transform_points(points_xyz, translation, quaternion) -> np.ndarray:
    """Apply a rigid transform (translation (3,), quaternion (x,y,z,w)) to an
    Nx3 array of points. Returns Nx3."""
    rot = quat_to_matrix(quaternion)
    pts = np.asarray(points_xyz, dtype=float).reshape(-1, 3)
    return (rot @ pts.T).T + np.asarray(translation, dtype=float).reshape(1, 3)


def select_wedge(bearings, theta_min, theta_max, margin=0.0) -> np.ndarray:
    """Indices of `bearings` falling within [theta_min, theta_max] (+/- margin)."""
    bearings = np.asarray(bearings, dtype=float)
    return np.where((bearings >= theta_min - margin) &
                    (bearings <= theta_max + margin))[0]


def cluster_nearest_range(ranges, gap=0.15, min_points=1) -> Optional[float]:
    """Median range of the NEAREST substantial cluster.

    Sorts the ranges, splits into clusters wherever consecutive ranges jump by
    more than `gap`, and returns the median of the nearest cluster with at least
    `min_points` members. The median (not mean) and nearest-cluster choice reject
    background returns seen beside/through the object. Returns None when there are
    no usable ranges or no cluster meets `min_points` (the empty-wedge / too-few
    case -- the caller emits no marker)."""
    vals = sorted(float(r) for r in ranges
                  if r is not None and np.isfinite(r) and r > 0)
    if not vals:
        return None
    clusters: List[List[float]] = [[vals[0]]]
    for v in vals[1:]:
        if v - clusters[-1][-1] > gap:
            clusters.append([v])
        else:
            clusters[-1].append(v)
    for cluster in clusters:  # ascending -> nearest first
        if len(cluster) >= min_points:
            return float(np.median(cluster))
    return None
