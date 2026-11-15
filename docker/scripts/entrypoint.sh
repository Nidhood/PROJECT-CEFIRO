#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
if [[ -f /cefiro_ws/install/setup.bash ]]; then
  source /cefiro_ws/install/setup.bash
fi
exec "$@"
