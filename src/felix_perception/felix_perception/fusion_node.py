"""felix_perception fusion node: bearing-wedge 2D-lidar fusion.

For each YOLO detection it back-projects the bbox horizontal extent to a bearing
wedge (optical frame), selects RPLIDAR returns whose bearing falls in the wedge,
takes the nearest substantial range cluster's median, and publishes the object's
position as a map-frame marker on /perception/objects.

Limitations are by design (see the plan): the RPLIDAR is a single horizontal
plane, so only objects crossing it (floor-standing) get a range; off-plane
objects and the unlocalized case publish NO marker and are NOT errors. Accuracy
is bounded by the camera<->lidar extrinsic (from the URDF mounts) and the
publish-time image stamp.
"""
import math
import os

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

import message_filters
import tf2_ros
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import do_transform_point
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from felix_perception import geometry


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion_node")
        self.declare_parameter("optical_frame", "camera_optical_link")
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_info_yaml", "")
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("range_gap", 0.15)
        self.declare_parameter("min_cluster_points", 1)
        self.declare_parameter("wedge_margin", 0.0)
        self.declare_parameter("marker_z", 0.1)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.optical_frame = g("optical_frame")
        self.laser_frame = g("laser_frame")
        self.map_frame = g("map_frame")
        self.range_gap = float(g("range_gap"))
        self.min_pts = int(g("min_cluster_points"))
        self.wedge_margin = float(g("wedge_margin"))
        self.marker_z = float(g("marker_z"))

        ci = g("camera_info_yaml")
        if not ci:
            from ament_index_python.packages import get_package_share_directory
            ci = os.path.join(
                get_package_share_directory("felix_perception"),
                "config", "camera_info.yaml")
        self._intr = geometry.parse_camera_info_yaml(ci)
        self._warned_unlocalized = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(MarkerArray, "/perception/objects", 10)

        det_sub = message_filters.Subscriber(
            self, Detection2DArray, "/perception/detections")
        scan_sub = message_filters.Subscriber(self, LaserScan, "/scan")
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [det_sub, scan_sub], queue_size=10, slop=float(g("sync_slop")))
        self.sync.registerCallback(self._on_pair)
        self.get_logger().info("fusion up (bearing-wedge lidar fusion)")

    # ----- synchronized callback ------------------------------------------- #
    def _on_pair(self, detections, scan):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if not detections.detections:
            self.pub.publish(markers)
            return

        # laser -> optical is fixed; bearings of every scan return in the optical
        # frame let us test wedge membership without transforming each wedge.
        try:
            t_lo = self.tf_buffer.lookup_transform(
                self.optical_frame, self.laser_frame, Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warning(f"no laser->optical TF yet: {exc}")
            self.pub.publish(markers)
            return

        laser_xy, ranges = self._valid_scan_points(scan)
        if laser_xy.shape[0] == 0:
            self.pub.publish(markers)
            return
        bearings = self._scan_bearings(laser_xy, t_lo)

        # laser -> map at the image stamp; absent => not localized => no markers.
        try:
            t_lm = self.tf_buffer.lookup_transform(
                self.map_frame, self.laser_frame, detections.header.stamp)
        except tf2_ros.TransformException:
            try:
                t_lm = self.tf_buffer.lookup_transform(
                    self.map_frame, self.laser_frame, Time())
            except tf2_ros.TransformException:
                if not self._warned_unlocalized:
                    self.get_logger().warning(
                        "no map->laser TF (robot not localized); publishing "
                        "detections without map placement.")
                    self._warned_unlocalized = True
                self.pub.publish(markers)
                return

        mid = 0
        for det in detections.detections:
            placed = self._place(det, laser_xy, ranges, bearings, t_lm, mid)
            if placed is not None:
                markers.markers.extend(placed)
                mid += 2
        self.pub.publish(markers)

    # ----- helpers ---------------------------------------------------------- #
    def _valid_scan_points(self, scan):
        """Nx2 laser-frame (x, y) and Nx range for finite, in-range returns."""
        xy, rng = [], []
        ang = scan.angle_min
        for r in scan.ranges:
            a, ang = ang, ang + scan.angle_increment
            if not math.isfinite(r) or r <= scan.range_min or r >= scan.range_max:
                continue
            xy.append((r * math.cos(a), r * math.sin(a)))
            rng.append(float(r))
        return np.asarray(xy, dtype=float).reshape(-1, 2), np.asarray(rng, float)

    def _scan_bearings(self, laser_xy, t_lo):
        tr = t_lo.transform.translation
        q = t_lo.transform.rotation
        pts = np.column_stack([laser_xy, np.zeros(laser_xy.shape[0])])
        opt = geometry.transform_points(
            pts, (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        return np.arctan2(opt[:, 0], opt[:, 2])

    def _place(self, det, laser_xy, ranges, bearings, t_lm, mid):
        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        half = det.bbox.size_x / 2.0
        theta_min, theta_max = geometry.bbox_bearings(
            cx - half, cx + half, cy, self._intr.K, self._intr.D)
        idx = geometry.select_wedge(bearings, theta_min, theta_max, self.wedge_margin)
        if idx.size == 0:
            return None
        med = geometry.cluster_nearest_range(
            ranges[idx], gap=self.range_gap, min_points=self.min_pts)
        if med is None:
            return None
        # Representative laser point: mean of in-wedge points near the median range.
        near = idx[np.abs(ranges[idx] - med) <= self.range_gap]
        lx, ly = laser_xy[near].mean(axis=0)

        p = PointStamped()
        p.header.frame_id = self.laser_frame
        p.point.x, p.point.y, p.point.z = float(lx), float(ly), 0.0
        mp = do_transform_point(p, t_lm).point

        label = det.results[0].hypothesis.class_id if det.results else "?"
        return [self._sphere(mp, mid, label), self._text(mp, mid + 1, label)]

    def _sphere(self, mp, mid, label):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.ns = "perception_objects"
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y = mp.x, mp.y
        m.pose.position.z = self.marker_z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.2, 0.9
        m.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        return m

    def _text(self, mp, mid, label):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.ns = "perception_labels"
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y = mp.x, mp.y
        m.pose.position.z = self.marker_z + 0.18
        m.pose.orientation.w = 1.0
        m.scale.z = 0.15
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = label
        m.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
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
