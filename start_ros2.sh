#!/usr/bin/env bash
#
# Run the felix_robot ROS 2 package: hardware bridge + ToF nodes (background, via
# ros2 launch) plus keyboard teleop (foreground). This is the colcon/launch-based
# replacement for the old start.sh.
#
# Prerequisite (once, and after any setup.py / package.xml change):
#   colcon build --symlink-install
#
# Usage:
#   ./start_ros2.sh [serial_port]
#
# Example:
#   ./start_ros2.sh /dev/myserial
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source the ROS 2 environment and this workspace's overlay. ROS setup scripts
# reference unset variables, so relax `nounset` while sourcing (same reason as
# the original start.sh).
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found." >&2
    exit 1
fi
if [ ! -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    echo "ERROR: ${SCRIPT_DIR}/install/setup.bash not found." >&2
    echo "Build the workspace first:  colcon build --symlink-install" >&2
    exit 1
fi
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/install/setup.bash"
set -u

SERIAL_PORT="${1:-/dev/myserial}"

# Start the hardware bridge + ToF nodes via launch (owns their lifecycle).
echo "Launching bridge + tof on ${SERIAL_PORT} ..."
ros2 launch felix_bringup felix.launch.py port:="${SERIAL_PORT}" &
LAUNCH_PID=$!

# On exit/interrupt, SIGINT the launch process so it shuts both nodes down
# cleanly (motors stopped in bridge destroy_node), then wait for it.
cleanup() {
    echo
    echo "Shutting down ..."
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# Give launch a moment to bring the bridge up (serial connect).
sleep 3
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: ros2 launch exited early. Check the serial port and the build." >&2
    exit 1
fi

# Keyboard teleop in the foreground (owns the terminal for keystrokes).
echo "Starting keyboard teleop ..."
ros2 run felix_base teleop
