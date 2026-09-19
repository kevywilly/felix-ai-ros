#!/bin/bash
ros2 bag record -o felix_run_02 --include-unpublished-topics \
  /cmd_vel /perception/objects /perception/detections /perception/status \
  /odometry/filtered /amcl_pose /scan /tf /tf_static \
  /camera/camera_info /goal_pose \
  /llm/command /llm/response /felix_llm/behavior
