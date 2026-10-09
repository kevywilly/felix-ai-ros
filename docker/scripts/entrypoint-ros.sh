#!/usr/bin/env bash
# cd /nano-control && /usr/bin/git pull
# /etc/init.d/nginx start

source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
export RCUTILS_COLORIZED_OUTPUT=1

cd /felix-ai-ros
source install/setup.sh
service avahi-daemon stop

# Auto-start the LLM server (llama-server) once, in the background. Sourced per
# shell, so the pgrep guard keeps the 2nd+ shell from fighting over port 8080,
# and nohup/& keeps it off the TTY (llm_server.sh ends in `exec`, which would
# otherwise hijack the shell). Toggle via FELIX_AUTOSTART_LLM in run-interactive.sh.
if [ "${FELIX_AUTOSTART_LLM:-0}" = "1" ] && ! pgrep -f llama-server >/dev/null 2>&1; then
  echo "Starting LLM server in background → /felix-ai-ros/llm_server.log"
  nohup /felix-ai-ros/llm_server.sh >/felix-ai-ros/llm_server.log 2>&1 &
fi
#tail -f /dev/null