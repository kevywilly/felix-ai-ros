#!/usr/bin/env bash
# Resume/extend an existing slam_toolbox map (continue mapping on top of it).
#
# Run this AFTER `mapping.launch.py` is up. Two modes:
#
#   ./resume_map.sh [basename]                 # GIVEN-POSE at origin (default)
#   ./resume_map.sh [basename] <x> <y> <theta> # GIVEN-POSE at x,y,theta (map frame)
#   ./resume_map.sh [basename] dock            # dock auto-match (START_AT_FIRST_NODE)
#
# DEFAULT is START_AT_GIVEN_POSE (match_type 2) seeded at the robot's pose in the
# map frame -- origin (0,0,0) if you're parked where the original map began. This
# seeds the pose and only scan-matches LOCALLY (~+/-20 deg), so it will NOT flip.
#
# The `dock` mode uses START_AT_FIRST_NODE (match_type 1 -> karto ProcessAtDock),
# which scan-matches your first scan against the saved first node over a WIDE
# orientation search. In rectangular / right-angled spaces that match can snap to
# a 90 deg local optimum and build the continuation rotated -- avoid it unless the
# space is distinctive. Prefer given-pose.
#
# WHY a service call and not a launch arg: the mapping node
# (async_slam_toolbox_node) does NOT read map_file_name/map_start_pose/
# map_start_at_dock at startup -- slam_toolbox only honours those in LOCALIZATION
# mode. Continuing to MAP from a saved graph must go through deserialize_map.
#
# initial_pose is map -> base_link (x, y, theta). theta in radians.
set -u

MAP="${1:-/felix-ai-ros/maps/felix_map}"

if [ ! -f "${MAP}.posegraph" ]; then
  echo "ERROR: ${MAP}.posegraph not found -- nothing to resume." >&2
  echo "       (A .pgm/.yaml grid cannot be resumed; you need the pose-graph.)" >&2
  exit 1
fi

if [ "${2:-}" = "dock" ]; then
  MATCH=1                       # START_AT_FIRST_NODE
  X=0.0; Y=0.0; THETA=0.0
  MODE="dock (START_AT_FIRST_NODE, wide match -- may flip 90 in square rooms)"
else
  MATCH=2                       # START_AT_GIVEN_POSE
  X="${2:-0.0}"; Y="${3:-0.0}"; THETA="${4:-0.0}"
  MODE="given-pose (${X}, ${Y}, ${THETA})"
fi

echo "Resuming ${MAP} -> ${MODE}"
ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: \"${MAP}\", match_type: ${MATCH}, initial_pose: {x: ${X}, y: ${Y}, theta: ${THETA}}}"

echo "Done. Verify the old map appears in /map AND the live scan lands on top of"
echo "the old walls before driving; then extend it and ./save_map.sh."
