#!/usr/bin/env python3
"""Teach the robot's current spot as a named place.

    ros2 run felix_llm teach kitchen

Drive Felix where you want with teleop, then run this to capture its current
pose into the locations file. Needs localization up (the composed felix_bringup
localization/navigation launch) so the map -> base_link TF exists.

Pose comes from TF (map -> base_link), not /amcl_pose: AMCL only publishes
/amcl_pose on a filter update (i.e. when the robot moves), so a stationary robot
never emits it -- but the map->odom->base_link transform is published
continuously, so TF works whether or not the robot is moving.
"""
import argparse
import os
import sys

import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory

from felix_llm.locations import LocationStore, quat_to_yaw


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    default_file = os.path.join(
        get_package_share_directory('felix_llm'), 'config', 'locations.yaml')
    parser = argparse.ArgumentParser(description="Save the current pose as a named place.")
    parser.add_argument('name', help="place name, e.g. kitchen")
    parser.add_argument('--file', default=default_file, help="locations YAML path")
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help="seconds to wait for the map->base_link TF")
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node('felix_llm_teach')
    store = LocationStore(args.file)
    buf = Buffer()
    TransformListener(buf, node)

    node.get_logger().info(
        f"waiting for {args.map_frame} -> {args.base_frame} TF "
        f"(is localization running and seeded?) ...")
    end = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    tf = None
    while rclpy.ok():
        try:
            tf = buf.lookup_transform(args.map_frame, args.base_frame,
                                      rclpy.time.Time())
            break
        except TransformException:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds > end:
                node.get_logger().error(
                    f"no {args.map_frame} -> {args.base_frame} TF -- start "
                    f"localization (felix_bringup localization.launch.py) and "
                    f"make sure AMCL is seeded (2D Pose Estimate in Foxglove, or "
                    f"set_initial_pose).")
                node.destroy_node()
                rclpy.shutdown()
                return 1

    t = tf.transform.translation
    q = tf.transform.rotation
    yaw = quat_to_yaw(q.z, q.w)
    store.save(args.name, t.x, t.y, yaw)
    node.get_logger().info(
        f"saved '{args.name.strip().lower()}' at x={t.x:.2f} y={t.y:.2f} "
        f"yaw={yaw:.2f} -> {store.path}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
