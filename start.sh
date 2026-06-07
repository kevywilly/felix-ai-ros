#!/usr/bin/env bash
#
# Start the ROSMASTER bridge node and the keyboard teleop node together.
#
# The bridge node (nodes/rosmaster_bridge_node.py) subscribes to /cmd_vel and
# drives the Yahboom board. It runs in the background. The keyboard teleop
# (main.py) runs in the foreground because it needs the terminal to read
# keystrokes. Pressing CTRL-C (or quitting teleop) shuts both down.
#
# Usage:
#   ./start.sh [serial_port]
#
# Example:
#   ./start.sh /dev/myserial
#
set -euo pipefail

# Run everything from the repository root so `from lib.rosmaster import ...`
# and `python3 nodes/...` resolve correctly regardless of where this is called.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source the ROS 2 environment (override with ROS_DISTRO if needed).
# ROS setup scripts reference unset variables, so relax `nounset` while sourcing.
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
else
    echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found." >&2
    echo "Set ROS_DISTRO to your installed distro and retry." >&2
    exit 1
fi

# Make the local packages (lib/, nodes/) importable.
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"

SERIAL_PORT="${1:-/dev/myserial}"

# Start the hardware bridge in the background.
echo "Starting rosmaster_bridge_node on ${SERIAL_PORT} ..."
python3 nodes/rosmaster_bridge_node.py --ros-args -p "port:=${SERIAL_PORT}" &
BRIDGE_PID=$!

# Ensure the bridge is stopped when teleop exits or this script is interrupted.
cleanup() {
    echo
    echo "Shutting down ..."
    if kill -0 "$BRIDGE_PID" 2>/dev/null; then
        kill "$BRIDGE_PID" 2>/dev/null || true
        wait "$BRIDGE_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# Give the bridge a moment to connect to the board.
sleep 2

# Bail out early if the bridge failed to start (e.g. serial port missing).
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo "ERROR: rosmaster_bridge_node failed to start. Check the serial port." >&2
    exit 1
fi

# Run the keyboard teleop in the foreground (owns the terminal for keystrokes).
echo "Starting keyboard teleop ..."
python3 main.py
