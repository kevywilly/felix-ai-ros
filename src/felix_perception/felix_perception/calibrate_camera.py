"""Headless camera calibration for felix_perception.

A GUI-free replacement for `cameracalibrator` (which needs an X display this
robot doesn't have). Subscribes to the camera, detects the checkerboard each
frame, auto-collects well-spread views, runs cv2.calibrateCamera, prints
progress + final RMS reprojection error to the terminal, and writes a standard
camera_info YAML.

  ros2 run felix_perception calibrate-camera --square 0.025
  ros2 run felix_perception calibrate-camera --size 8x6 --square 0.025 \
      --target 40 --out src/felix_perception/config/camera_info.yaml

Watch progress headlessly two ways:
  * the terminal prints "captured N/target" + per-axis coverage spans, and
  * an annotated feed is published on /calibration/annotated/compressed --
    add an Image panel on that topic in Foxglove to see detected corners live.

Move the board across the WHOLE frame (corners, near/far, tilts) until the
coverage spans are wide and it reaches the target; it then calibrates and writes
the YAML. Ctrl-C early also calibrates with whatever was collected (>= --min).

NOTE: calibrate at the SAME resolution the detector runs at (the camera's
bringup default is 640x360), and verify the printed square size with a ruler.
"""
import argparse
import math
import sys

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage

SENSOR_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)
_SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
_FIND_FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
               | cv2.CALIB_CB_FAST_CHECK)


def board_params(corners, w, h, bw, bh):
    """cameracalibrator-style view signature in [0,1]: (X, Y, size, skew)."""
    pts = corners.reshape(-1, 2)
    x = float(pts[:, 0].mean() / w)
    y = float(pts[:, 1].mean() / h)
    ul, ur = pts[0], pts[bw - 1]
    dl, dr = pts[bw * (bh - 1)], pts[-1]
    quad = np.array([ul, ur, dr, dl])
    area = 0.5 * abs(np.dot(quad[:, 0], np.roll(quad[:, 1], -1))
                     - np.dot(quad[:, 1], np.roll(quad[:, 0], -1)))
    size = math.sqrt(area / (w * h))

    def angle(a, b, c):
        ba, bc = a - b, c - b
        cos = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-9)
        return math.acos(max(-1.0, min(1.0, cos)))

    skew = min(1.0, 2.0 * abs(math.pi / 2 - angle(ul, ur, dr)))
    return np.array([x, y, size, skew])


class CalibrateNode(Node):
    def __init__(self, bw, bh, square, target, min_views, min_dist, out_path):
        super().__init__("calibrate_camera")
        self.bw, self.bh = bw, bh
        self.target, self.min_views, self.min_dist = target, min_views, min_dist
        self.out_path = out_path
        self.done = False
        self.size_wh = None

        # 3D object points for one board (Z=0), scaled by the square size.
        objp = np.zeros((bw * bh, 3), np.float32)
        objp[:, :2] = np.mgrid[0:bw, 0:bh].T.reshape(-1, 2) * square
        self.objp = objp

        self.obj_points = []     # list of objp
        self.img_points = []     # list of refined corners
        self.params = []         # list of view signatures

        self.pub = self.create_publisher(
            CompressedImage, "/calibration/annotated/compressed", 1)
        self.sub = self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed",
            self._on_image, SENSOR_QOS)
        self.get_logger().info(
            f"calibrating: board {bw}x{bh} corners, target {target} views. "
            "Move the board across the whole frame. Watch "
            "/calibration/annotated/compressed in Foxglove.")

    def _on_image(self, msg):
        if self.done:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        h, w = frame.shape[:2]
        if self.size_wh is None:
            self.size_wh = (w, h)
        elif self.size_wh != (w, h):
            return  # all views must share one resolution

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, (self.bw, self.bh), _FIND_FLAGS)
        captured = False
        if found:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), _SUBPIX)
            p = board_params(corners, w, h, self.bw, self.bh)
            if self._accept(p):
                self.obj_points.append(self.objp)
                self.img_points.append(corners)
                self.params.append(p)
                captured = True
                self._report(len(self.params))
            cv2.drawChessboardCorners(frame, (self.bw, self.bh), corners, found)

        self._annotate(frame, found, captured)
        self._publish(frame, msg.header)

        if len(self.params) >= self.target:
            self.finish()

    def _accept(self, p):
        if not self.params:
            return True
        d = min(float(np.linalg.norm(p - q)) for q in self.params)
        return d > self.min_dist

    def _spans(self):
        if not self.params:
            return [0.0, 0.0, 0.0, 0.0]
        a = np.array(self.params)
        return (a.max(axis=0) - a.min(axis=0)).tolist()

    def _report(self, n):
        sx, sy, ss, sk = self._spans()
        self.get_logger().info(
            f"captured {n}/{self.target}  coverage  "
            f"X:{sx:.2f} Y:{sy:.2f} Size:{ss:.2f} Skew:{sk:.2f}")

    def _annotate(self, frame, found, captured):
        n = len(self.params)
        msg = f"views {n}/{self.target}"
        color = (0, 200, 0) if found else (0, 0, 200)
        if captured:
            msg += "  CAPTURED"
        cv2.putText(frame, msg, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    color, 2, cv2.LINE_AA)

    def _publish(self, frame, header):
        ok, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            out = CompressedImage()
            out.header = header
            out.format = "jpeg"
            out.data = enc.tobytes()
            self.pub.publish(out)

    def finish(self):
        if self.done:
            return
        self.done = True
        n = len(self.params)
        if n < self.min_views:
            self.get_logger().error(
                f"only {n} views (< --min {self.min_views}); not calibrating.")
            return
        self.get_logger().info(f"calibrating from {n} views...")
        w, h = self.size_wh
        rms, K, D, _, _ = cv2.calibrateCamera(
            self.obj_points, self.img_points, (w, h), None, None)
        self.get_logger().info(f"RMS reprojection error: {rms:.4f} px "
                               f"({'good' if rms < 0.6 else 'HIGH -- recapture'})")
        write_camera_info(self.out_path, w, h, K, D.flatten(),
                          name="felix_csi")
        self.get_logger().info(f"wrote {self.out_path}")
        print("\ncamera_matrix K =\n", np.array2string(K, precision=2))
        print("distortion D =", np.array2string(D.flatten(), precision=4))


def write_camera_info(path, w, h, K, D, name="camera"):
    K = np.asarray(K, float).reshape(3, 3)
    D = np.asarray(D, float).reshape(-1)
    P = np.zeros((3, 4))
    P[:3, :3] = K

    def grid(a, cols):
        flat = ", ".join(f"{v:.8g}" for v in np.asarray(a).flatten())
        return flat

    text = f"""# Generated by felix_perception calibrate-camera. Resolution {w}x{h}.
image_width: {w}
image_height: {h}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{grid(K, 3)}]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: {D.size}
  data: [{grid(D, D.size)}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [{grid(P, 4)}]
"""
    with open(path, "w") as f:
        f.write(text)


def _default_out():
    try:
        from ament_index_python.packages import get_package_share_directory
        import os
        return os.path.join(
            get_package_share_directory("felix_perception"),
            "config", "camera_info.yaml")
    except Exception:
        return "camera_info.yaml"


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    ap = argparse.ArgumentParser(description="Headless camera calibration.")
    ap.add_argument("--size", default="8x6", help="interior corners WxH")
    ap.add_argument("--square", type=float, default=0.025,
                    help="square size in METERS (measure your print!)")
    ap.add_argument("--target", type=int, default=40,
                    help="views to collect before calibrating")
    ap.add_argument("--min", type=int, default=12,
                    help="minimum views to calibrate on Ctrl-C")
    ap.add_argument("--min-distance", type=float, default=0.15,
                    help="how different a view must be to be collected")
    ap.add_argument("--out", default=None,
                    help="camera_info YAML output (default: package share copy, "
                         "which symlinks back to source under --symlink-install)")
    args, _ = ap.parse_known_args(argv)
    bw, bh = (int(v) for v in args.size.lower().split("x"))
    out = args.out or _default_out()

    rclpy.init()
    node = CalibrateNode(bw, bh, args.square, args.target, args.min,
                         args.min_distance, out)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("stopped -- calibrating with collected views.")
        node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
