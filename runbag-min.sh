#!/bin/bash
cd bags && ros2 bag record -o felix_$(date +%Y%m%d_%H%M%S) \
  /cmd_vel /perception/objects /perception/detections \
  /odometry/filtered /amcl_pose /scan /tf /tf_static \
  /camera/camera_info
