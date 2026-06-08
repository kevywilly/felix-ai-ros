#!/usr/bin/env python3
"""CSI IMX219 camera -> ROS 2 CompressedImage (JPEG) for Foxglove / perception.

Ports the GStreamer capture from the pre-ROS felix-ai video_agent: nvargus CSI
source, hardware-accelerated downscale + 180-degree flip (the camera is mounted
upside down), into an OpenCV BGR appsink. A background thread keeps only the
latest frame (drop=1 max-buffers=1) and publishes it JPEG-compressed, so a slow
network consumer can never build a latency backlog.

CompressedImage (not raw Image) is deliberate: raw 960x540 BGR is ~1.5 MB/frame
(~45 MB/s at 30 fps), which chokes the Foxglove WebSocket over WiFi; JPEG is
~50-100x smaller and Foxglove decodes it natively.

Note: the CSI camera allows only ONE argus consumer at a time. If the old
felix-ai WebRTC stack (or anything else) holds the camera, opening here fails --
stop the other consumer first.
"""
import threading

import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CompressedImage


def build_gst_pipeline(sensor_id, in_w, in_h, fps, out_w, out_h, flip_method):
    """nvargus CSI -> downscaled, flipped BGR appsink (matches felix-ai)."""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={in_w}, height={in_h}, "
        f"format=NV12, framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={out_w}, height={out_h}, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink drop=1 max-buffers=1"
    )


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Capture geometry. Defaults match felix-ai: sensor mode 4 (1280x720@59)
        # downscaled to 960x540 (16:9, no stretch); flip 2 = 180deg (upside down).
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('capture_width', 1280)
        self.declare_parameter('capture_height', 720)
        self.declare_parameter('framerate', 59)
        self.declare_parameter('output_width', 960)
        self.declare_parameter('output_height', 540)
        self.declare_parameter('flip_method', 2)
        self.declare_parameter('jpeg_quality', 80)      # 0-100; 80 is a good size/quality knob
        self.declare_parameter('max_fps', 0.0)          # cap publish rate; 0 = every captured frame
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('open_attempts', 5)
        self.declare_parameter('open_backoff', 2.0)

        g = lambda n: self.get_parameter(n).value
        self.jpeg_quality = int(g('jpeg_quality'))
        self.frame_id = g('frame_id')
        max_fps = float(g('max_fps'))
        # Minimum seconds between published frames. We still READ every captured
        # frame (drains the appsink so the latest stays fresh) but only encode +
        # publish when due -- throttling here cuts both WiFi bandwidth and the
        # CPU spent on JPEG, which is what actually reduces perceived lag.
        self._min_period = (1.0 / max_fps) if max_fps > 0.0 else 0.0

        self.pipeline = build_gst_pipeline(
            g('sensor_id'), g('capture_width'), g('capture_height'), g('framerate'),
            g('output_width'), g('output_height'), g('flip_method'))
        self.get_logger().info(f"GStreamer pipeline:\n{self.pipeline}")

        self.pub = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)

        self._cap = self._open(int(g('open_attempts')), float(g('open_backoff')))
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            "Publishing camera/image_raw/compressed (JPEG q=%d)" % self.jpeg_quality)

    def _open(self, attempts, backoff):
        """Open the CSI camera, retrying transient argus contention.

        Argus allows one consumer and needs a moment to release the sensor after
        a prior process exits; retry with backoff so that window is tolerated."""
        import time
        for attempt in range(1, attempts + 1):
            cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                return cap
            cap.release()
            if attempt < attempts:
                self.get_logger().warning(
                    f"CSI camera busy (attempt {attempt}/{attempts}), retrying in {backoff:.1f}s "
                    "(is another process holding the camera?)")
                time.sleep(backoff)
        raise RuntimeError(
            f"Could not open CSI camera after {attempts} attempts. Pipeline:\n{self.pipeline}")

    def _loop(self):
        import time
        next_pub = 0.0
        while self._running and rclpy.ok():
            ok, frame = self._cap.read()
            if not ok:
                self.get_logger().warning("empty frame from camera")
                continue
            # Throttle: drop frames that arrive before the next publish slot
            # (we already drained the read above to keep the sensor current).
            now = time.monotonic()
            if now < next_pub:
                continue
            next_pub = now + self._min_period
            ok, buf = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self.pub.publish(msg)

    def destroy_node(self):
        self._running = False
        t = getattr(self, '_thread', None)
        if t is not None:
            t.join(timeout=2)
        cap = getattr(self, '_cap', None)
        if cap is not None:
            cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
