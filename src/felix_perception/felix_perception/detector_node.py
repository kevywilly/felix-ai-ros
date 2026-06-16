"""felix_perception detector node.

Consumes the camera's published JPEG stream (the camera is the sole argus
consumer -- we never open the sensor), runs Ultralytics YOLO on a TensorRT FP16
engine (or the committed .pt as a fallback), and publishes:

  * /perception/detections            vision_msgs/Detection2DArray (optical frame)
  * /camera/camera_info               sensor_msgs/CameraInfo (matches each frame)
  * /perception/status                std_msgs/String: backend + inference ms (latched)
  * /perception/annotated/compressed  sensor_msgs/CompressedImage (FPV boxes)

Inference runs on a worker thread on the latest frame only (depth-1 sensor QoS),
so it never blocks the executor and never queues stale frames. See the plan's
pixel-path / QoS / engine Key Technical Decisions.
"""
import os
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

from felix_perception import geometry

# Depth-1 best-effort: a frame arriving mid-inference replaces the queued one.
# Best-effort subscriber is compatible with the camera's reliable publisher.
SENSOR_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)
# Latched so a late Foxglove/diagnostic subscriber still sees the active backend.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        share = get_package_share_directory("felix_perception")

        self.declare_parameter("weights", "yolo11n.pt")
        self.declare_parameter("engine_dir", "")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("optical_frame", "camera_optical_link")
        self.declare_parameter(
            "camera_info_yaml", os.path.join(share, "config", "camera_info.yaml"))
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("require_engine", False)
        self.declare_parameter("health_timeout", 10.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.conf = float(g("conf"))
        self.imgsz = int(g("imgsz"))
        self.optical_frame = g("optical_frame")
        self.publish_annotated = bool(g("publish_annotated"))
        self._health_timeout = float(g("health_timeout"))

        # CameraInfo built once from the calibration YAML; re-stamped per frame.
        self._cam_info, self._intr = geometry.load_camera_info(
            g("camera_info_yaml"), self.optical_frame)

        self.pub_det = self.create_publisher(
            Detection2DArray, "/perception/detections", 10)
        self.pub_info = self.create_publisher(
            CameraInfo, "/camera/camera_info", 10)
        self.pub_status = self.create_publisher(
            String, "/perception/status", LATCHED_QOS)
        self.pub_annot = self.create_publisher(
            CompressedImage, "/perception/annotated/compressed", 10) \
            if self.publish_annotated else None

        # Model: engine if built, else .pt fallback (made observable on /status).
        self._backend, self._model = self._load_model(
            g("weights"), g("engine_dir"), bool(g("require_engine")))
        self._publish_status(backend_only=True)
        self._warmup()

        self._lock = threading.Lock()
        self._latest = None         # (stamp, bgr frame)
        self._last_rx = None
        self._dims_checked = False
        self._running = True
        self._start = time.monotonic()

        self.sub = self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed",
            self._on_image, SENSOR_QOS)
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()
        self.create_timer(2.0, self._health_check)
        self.get_logger().info(
            f"detector up (backend={self._backend}, optical_frame={self.optical_frame})")

    # ----- model loading ---------------------------------------------------- #
    def _load_model(self, weights, engine_dir, require_engine):
        from ultralytics import YOLO
        engine_path = geometry.engine_cache_path(weights, engine_dir or None)
        if os.path.exists(engine_path):
            self.get_logger().info(f"loading TensorRT engine {engine_path}")
            # task is explicit: a bare .engine carries no task metadata to guess.
            return "engine", YOLO(engine_path, task="detect")
        note = (f"engine {engine_path} not found; run "
                "'ros2 run felix_perception build-engine'")
        if require_engine:
            raise RuntimeError(note + " (require_engine=true)")
        self.get_logger().warning(
            note + " -- FALLING BACK to the .pt (slower torch path). "
            "Watch /perception/status.")
        share_w = os.path.join(
            get_package_share_directory("felix_perception"), "models", weights)
        wpath = weights if os.path.exists(weights) else (
            share_w if os.path.exists(share_w) else weights)
        return "pt", YOLO(wpath)

    def _warmup(self):
        try:
            self._model(np.zeros((self.imgsz, self.imgsz, 3), np.uint8),
                        imgsz=self.imgsz, verbose=False)
        except Exception as exc:  # pragma: no cover - device-only
            self.get_logger().warning(f"warmup failed: {exc}")

    # ----- frame ingest + worker loop --------------------------------------- #
    def _on_image(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self._lock:
            self._latest = (msg.header.stamp, frame)
            self._last_rx = time.monotonic()

    def _loop(self):
        while self._running and rclpy.ok():
            with self._lock:
                item, self._latest = self._latest, None
            if item is None:
                time.sleep(0.002)
                continue
            try:
                self._process(*item)
            except Exception as exc:  # pragma: no cover - device-only
                self.get_logger().error(f"inference error: {exc}")

    def _process(self, stamp, frame):
        h, w = frame.shape[:2]
        if not self._dims_checked:
            self._dims_checked = True
            if (w, h) != (self._intr.width, self._intr.height):
                self.get_logger().error(
                    f"image {w}x{h} != camera_info "
                    f"{self._intr.width}x{self._intr.height}: back-projection and "
                    "fusion will be wrong. Calibrate at the run resolution (R13).")

        t0 = time.monotonic()
        res = self._model(frame, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]
        infer_ms = (time.monotonic() - t0) * 1000.0

        b = res.boxes
        if b is not None and len(b):
            xyxy, conf, cls = (b.xyxy.cpu().numpy(),
                               b.conf.cpu().numpy(), b.cls.cpu().numpy())
        else:
            xyxy, conf, cls = np.zeros((0, 4)), np.zeros((0,)), np.zeros((0,))
        records = geometry.detections_to_records(xyxy, conf, cls, self._model.names)

        arr = Detection2DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self.optical_frame
        for r in records:
            det = Detection2D()
            det.header = arr.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = r.class_id     # vision_msgs 4.x: string
            hyp.hypothesis.score = r.score
            det.results.append(hyp)
            det.bbox.center.position.x = r.cx        # vision_msgs/Pose2D/Point2D
            det.bbox.center.position.y = r.cy
            det.bbox.size_x = r.size_x
            det.bbox.size_y = r.size_y
            arr.detections.append(det)
        self.pub_det.publish(arr)

        self._cam_info.header.stamp = stamp
        self.pub_info.publish(self._cam_info)
        self._publish_status(infer_ms=infer_ms, n=len(records))

        if self.pub_annot is not None:
            ok, enc = cv2.imencode(
                ".jpg", res.plot(), [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                out = CompressedImage()
                out.header = arr.header
                out.format = "jpeg"
                out.data = enc.tobytes()
                self.pub_annot.publish(out)

    def _publish_status(self, backend_only=False, infer_ms=None, n=None):
        st = String()
        if backend_only:
            st.data = f"backend={self._backend} (initializing)"
        else:
            st.data = (f"backend={self._backend} infer_ms={infer_ms:.1f} "
                       f"detections={n}")
        self.pub_status.publish(st)

    def _health_check(self):
        with self._lock:
            last = self._last_rx
        if last is None and time.monotonic() - self._start > self._health_timeout:
            self.get_logger().warning(
                "no camera frames yet -- is camera.launch.py running and is "
                "/camera/image_raw/compressed publishing?")

    def destroy_node(self):
        self._running = False
        t = getattr(self, "_worker", None)
        if t is not None:
            t.join(timeout=2)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
