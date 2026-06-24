#!/usr/bin/env python3
"""home_scan_assist.py -- capture a lidar "fingerprint" at the current pose and
later replay it as a fixed "ghost" scan you drive back onto in Foxglove.

This is the zero-friction re-homing aid: park at p0, SAVE the scan, drive away,
then PLAY it back as a stationary ghost and manually drive until the live /scan
overlaps the ghost. No colcon build -- run directly after sourcing ROS:

    # at p0 (on the dock), with the stack running so the 'map' frame exists:
    python3 home_scan_assist.py save                 # writes home_scan.json

    # later, after driving away, with the stack running again:
    python3 home_scan_assist.py play                 # publishes the ghost
    # in Foxglove (3D panel, Fixed frame = map): add /home_scan (e.g. red) and
    # /scan (e.g. green); drive until the two scans overlap -> you're back at p0.

Why a whole frame and not just a bag replay: a replayed /scan keeps frame_id
'laser', so Foxglove draws it at the LIVE (moving) laser pose -- it would track
the robot, not mark p0. So we pin the ghost to a NEW static frame 'home_laser'
fixed to where the laser was at capture time (in 'map' by default). The ghost
then stays put while the robot -- and the live /scan in the moving 'laser'
frame -- moves around it.

Use --frame odom if you are NOT running SLAM/AMCL (no 'map' frame). Note odom
drifts over a run and resets on restart, so 'map' is better for returning
"later"; odom is fine for a quick same-session there-and-back.

Captures the full scan (all beams), which is far more robust than comparing
front/left/right distances -- those are rotationally ambiguous in boxy/symmetric
rooms (the same 90-degree-flip failure that bites slam_toolbox's wide match).
"""
import argparse
import json
import sys

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster

DEFAULT_FILE = "home_scan.json"
GHOST_FRAME = "home_laser"


def do_save(args):
    rclpy.init()
    node = rclpy.create_node("home_scan_save")
    buf = Buffer()
    TransformListener(buf, node)

    holder = {}
    node.create_subscription(LaserScan, "/scan",
                             lambda m: holder.setdefault("scan", m), 10)

    node.get_logger().info("waiting for a /scan ...")
    while rclpy.ok() and "scan" not in holder:
        rclpy.spin_once(node, timeout_sec=0.2)
    scan = holder["scan"]
    src = scan.header.frame_id  # 'laser'

    node.get_logger().info(f"got scan in '{src}', looking up {args.frame} -> {src} ...")
    tf = None
    for _ in range(50):
        try:
            tf = buf.lookup_transform(args.frame, src, rclpy.time.Time())
            break
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.2)
    if tf is None:
        node.get_logger().error(
            f"no transform {args.frame} -> {src}. Is the stack up and the "
            f"'{args.frame}' frame published? (try --frame odom if no SLAM/AMCL)")
        node.destroy_node(); rclpy.shutdown(); return 1

    t, q = tf.transform.translation, tf.transform.rotation
    data = {
        "frame": args.frame,
        "tf": {"x": t.x, "y": t.y, "z": t.z,
               "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w},
        "scan": {
            "angle_min": scan.angle_min, "angle_max": scan.angle_max,
            "angle_increment": scan.angle_increment,
            "time_increment": scan.time_increment, "scan_time": scan.scan_time,
            "range_min": scan.range_min, "range_max": scan.range_max,
            "ranges": list(scan.ranges), "intensities": list(scan.intensities),
        },
    }
    with open(args.file, "w") as f:
        json.dump(data, f)
    node.get_logger().info(
        f"saved {len(data['scan']['ranges'])} beams + pose to {args.file} "
        f"(laser at {args.frame}: x={t.x:.3f} y={t.y:.3f})")
    node.destroy_node(); rclpy.shutdown(); return 0


def do_play(args):
    with open(args.file) as f:
        data = json.load(f)
    rclpy.init()
    node = rclpy.create_node("home_scan_play")

    # Pin the ghost frame to where the laser was at capture time (latched static TF).
    stb = StaticTransformBroadcaster(node)
    tfm = TransformStamped()
    tfm.header.stamp = node.get_clock().now().to_msg()
    tfm.header.frame_id = data["frame"]
    tfm.child_frame_id = GHOST_FRAME
    d = data["tf"]
    (tfm.transform.translation.x, tfm.transform.translation.y,
     tfm.transform.translation.z) = d["x"], d["y"], d["z"]
    (tfm.transform.rotation.x, tfm.transform.rotation.y,
     tfm.transform.rotation.z, tfm.transform.rotation.w) = (d["qx"], d["qy"],
                                                            d["qz"], d["qw"])
    stb.sendTransform(tfm)

    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL  # latched for late subscribers
    pub = node.create_publisher(LaserScan, "/home_scan", qos)

    s = data["scan"]

    def publish():
        msg = LaserScan()
        msg.header.stamp = node.get_clock().now().to_msg()  # keep it "fresh"
        msg.header.frame_id = GHOST_FRAME
        msg.angle_min, msg.angle_max = s["angle_min"], s["angle_max"]
        msg.angle_increment = s["angle_increment"]
        msg.time_increment, msg.scan_time = s["time_increment"], s["scan_time"]
        msg.range_min, msg.range_max = s["range_min"], s["range_max"]
        msg.ranges = [float(r) for r in s["ranges"]]
        msg.intensities = [float(i) for i in s["intensities"]]
        pub.publish(msg)

    node.create_timer(0.25, publish)
    node.get_logger().info(
        f"ghost on /home_scan in '{GHOST_FRAME}' (pinned to {data['frame']}). "
        f"Foxglove: 3D panel Fixed frame = {data['frame']}; add /home_scan + /scan. "
        f"Ctrl-C to stop.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown(); return 0


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["save", "play"])
    p.add_argument("--file", default=DEFAULT_FILE, help="fingerprint JSON file")
    p.add_argument("--frame", default="map",
                   help="anchor frame for the ghost (default map; use odom if no SLAM/AMCL)")
    args = p.parse_args()
    return do_save(args) if args.mode == "save" else do_play(args)


if __name__ == "__main__":
    sys.exit(main())
