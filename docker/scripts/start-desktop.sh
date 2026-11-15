#!/usr/bin/env bash
# Browser desktop. Portable, but intentionally software-rendered (Mesa).
set -euo pipefail
export DISPLAY=:99
Xvfb :99 -screen 0 1600x900x24 +extension GLX +render -noreset -nolisten tcp &
XVFB=$!
trap 'kill "$XVFB" "${VNC:-}" "${WEB:-}" 2>/dev/null || true' EXIT
sleep 2
openbox &
xsetroot -solid '#191d26' || true
xterm -fa Monospace -fs 11 -title 'CEFIRO ROS 2 terminal' -e bash -lc 'source /opt/ros/jazzy/setup.bash; if [[ -f /cefiro_ws/install/setup.bash ]]; then source /cefiro_ws/install/setup.bash; fi; cd /cefiro_ws; exec bash' &
x11vnc -display :99 -rfbport 5900 -localhost -shared -forever -nopw -quiet &
VNC=$!
websockify --web=/usr/share/novnc 0.0.0.0:6080 localhost:5900 &
WEB=$!
echo 'CEFIRO desktop: http://127.0.0.1:6080/vnc.html?autoconnect=1'
wait -n "$XVFB" "$VNC" "$WEB"
