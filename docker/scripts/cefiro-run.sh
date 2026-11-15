#!/usr/bin/env bash
# A ROS-aware wrapper for docker compose exec (exec bypasses ENTRYPOINT).
set -e
source /opt/ros/jazzy/setup.bash
if [[ -f /cefiro_ws/install/setup.bash ]]; then
  source /cefiro_ws/install/setup.bash
fi
cd /cefiro_ws
exec "$@"
