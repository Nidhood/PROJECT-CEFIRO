
#!/usr/bin/env bash
# CEFIRO Xpra seamless server
set -euo pipefail

if [[ -z "${XPRA_PASSWORD:-}" ]]; then
    echo "XPRA_PASSWORD is missing" >&2
    exit 1
fi

mkdir -p /tmp/runtime-cefiro
chmod 700 /tmp/runtime-cefiro

export XDG_RUNTIME_DIR=/tmp/runtime-cefiro
export XPRA_IBUS=0
export XPRA_EXPOSE_IBUS_LAYOUTS=0

unset DISPLAY
unset DBUS_SESSION_BUS_ADDRESS

echo "[CEFIRO] Starting Xpra on :100"

exec xpra seamless :100 \
    --xvfb='Xvfb -screen 0 1920x1080x24 +extension GLX +render -noreset -nolisten tcp' \
    '--bind-tcp=0.0.0.0:14500,auth=env' \
    --start-child=xterm \
    --input-method=none \
    --dbus-launch=no \
    --dbus-control=no \
    --notifications=no \
    --system-tray=no \
    --mdns=no \
    --pulseaudio=no \
    --printing=no \
    --webcam=no \
    --daemon=no
