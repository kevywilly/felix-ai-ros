#!/usr/bin/env bash
# cd /nano-control && /usr/bin/git pull
# /etc/init.d/nginx start

source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
export RCUTILS_COLORIZED_OUTPUT=1

cd /felix-ai-ros
source install/setup.sh
service avahi-daemon stop
#tail -f /dev/null