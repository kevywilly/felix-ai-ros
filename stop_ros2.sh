#!/usr/bin/env bash
# Stop ALL Felix ROS 2 nodes and launches.
#
# Use when nodes orphan -- e.g. `ros2 node list` shows duplicates like three
# /tof_node. That happens when a `ros2 launch` is backgrounded (or its terminal
# is closed) without a foreground Ctrl-C, so launch never tears down its child
# nodes and they reparent to init and keep running (and fight over the serial
# port). This force-stops them. The ros2 CLI daemon is left alone.
#
# Usage:  ./stop_ros2.sh
set -u

SELF=$$

# Match the node executables we run + the launch files, by path fragments that
# do NOT appear in this script's own path (so we never kill ourselves).
patterns=(
  "felix_base/lib/felix_base/"                  # bridge, tof, teleop
  "felix_camera/lib/felix_camera/"              # camera
  "robot_state_publisher/robot_state_publisher"
  "robot_localization/ekf_node"
  "slam_toolbox/async_slam_toolbox_node"
  "rplidar_ros/rplidar"
  "foxglove_bridge/foxglove_bridge"
  "tf2_ros/static_transform_publisher"
  "nav2_map_server/map_server"                  # localization + navigation stacks
  "nav2_amcl/amcl"
  "nav2_lifecycle_manager/lifecycle_manager"
  "nav2_controller/controller_server"
  "nav2_planner/planner_server"
  "nav2_behaviors/behavior_server"
  "nav2_bt_navigator/bt_navigator"
  "felix.launch.py"
  "slam.launch.py"
  "mapping.launch.py"
  "localization.launch.py"
  "navigation.launch.py"
  "camera.launch.py"
  "ekf.launch.py"
  "description.launch.py"
)

sweep() {  # $1 = signal
  for pat in "${patterns[@]}"; do
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
      [ "$pid" = "$SELF" ] && continue
      kill "$1" "$pid" 2>/dev/null
    done
  done
}

sweep -TERM      # ask launches to shut down children cleanly
sleep 1
sweep -KILL      # force any stragglers

# Refresh the (now stale) discovery graph so `ros2 node list` is accurate.
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
set -u
ros2 daemon stop >/dev/null 2>&1 || true

echo "Stopped all Felix ROS nodes. Verify with: ros2 node list"
