#!/usr/bin/env bash
# Start once the enrolled adapter is present. The CEFIRO node handles reconnects.
set -eo pipefail
: "${CEFIRO_WS:?Missing workspace}" "${CEFIRO_SERIAL:?Missing serial path}"
: "${CEFIRO_ENABLE_TX:?Missing TX configuration}" "${CEFIRO_DSHOT_MAX:?Missing DShot cap}"
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$CEFIRO_WS/install/setup.bash" ]]; then
    echo 'ROS 2 Jazzy or the compiled CEFIRO workspace is missing.' >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$CEFIRO_WS/install/setup.bash"
cd "$CEFIRO_WS"
if [[ ! -c "$CEFIRO_SERIAL" ]]; then
    echo "Waiting for enrolled CEFIRO adapter: $CEFIRO_SERIAL"
fi
while [[ ! -c "$CEFIRO_SERIAL" ]]; do
    sleep 1
done
echo "CEFIRO adapter detected: $(readlink -f "$CEFIRO_SERIAL")"
exec ros2 launch cefiro_hardware_interface hardware.launch.py \
    "port:=$CEFIRO_SERIAL" protocol_mode:=cf1 \
    "enable_command_tx:=$CEFIRO_ENABLE_TX" \
    "rpm_to_dshot_enabled:=$CEFIRO_ENABLE_TX" \
    "dshot_max:=$CEFIRO_DSHOT_MAX"
