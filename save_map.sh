#!/usr/bin/env bash
# Save the Felix map in BOTH formats. Run this while a mapping session
# (felix_slam slam.launch.py) is still LIVE -- the serialize step is a service
# call to the running slam_toolbox node.
#
#   1) occupancy grid  (.pgm + .yaml)  -- what nav2_map_server + AMCL load for
#      localization. This is a one-way export; slam_toolbox CANNOT reload it.
#   2) pose-graph      (.posegraph + .data)  -- slam_toolbox's own format,
#      REQUIRED to later resume/extend mapping (see slam.launch.py map_file_name).
#      Skipping this is why an existing map can't be appended to afterwards.
#
# Usage:  ./save_map.sh [basename]   (default: /felix-ai-ros/maps/felix_map)
set -u

MAP="${1:-/felix-ai-ros/maps/felix_map}"

# 1) occupancy grid for localization.
#    --mode scale + a low --free preserves the live map's 0-100 probability
#    gradient instead of the default trinary, which snaps every mid-confidence
#    cell (free_thresh..occupied_thresh) to grey "unknown" -- that snap is why a
#    map that looks good live looks eroded (thin/missing walls) once Nav2/AMCL
#    load it. PNG, not PGM: scale mode needs the alpha channel to carry unknown.
ros2 run nav2_map_server map_saver_cli -f "$MAP" \
  --mode scale --occ 0.65 --free 0.196 --fmt png

# 2) pose-graph for resuming/extending mapping
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: \"$MAP\"}"

# The in-container (root) slam_toolbox node writes .posegraph/.data as root;
# hand them back to the host user so host-side git can manage them.
chown 1000:1000 "${MAP}".png "${MAP}".pgm "${MAP}".yaml \
                "${MAP}".posegraph "${MAP}".data 2>/dev/null || true

echo "Saved: ${MAP}.png/.yaml (grid, scale mode) + ${MAP}.posegraph/.data (pose-graph)"
