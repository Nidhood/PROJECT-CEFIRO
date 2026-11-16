#!/usr/bin/env bash
# Launch a separate Xpra display (:100) for seamless native Windows GUI windows.
# The existing noVNC Xvfb display (:99) is unaffected.
set -euo pipefail

if [[ -z "${XPRA_PASSWORD:-}" ]]; then
  echo "XPRA_PASSWORD must be set in docker/.env" >&2
  exit 1
fi

mkdir -p /tmp/runtime-cefiro
chmod 700 /tmp/runtime-cefiro
export XDG_RUNTIME_DIR=/tmp/runtime-cefiro
unset DISPLAY

echo 'Starting Xpra seamless on display :100 (Windows localhost:14500)'
exec xpra seamless :100 \
  --xvfb='Xvfb -screen 0 1920x1080x24 +extension GLX +render -noreset -nolisten tcp' \
  '--bind-tcp=0.0.0.0:14500,auth=env' \
  --start-child=xterm \
  --mdns=no \
  --pulseaudio=no \
  --printing=no \
  --webcam=no \
  --daemon=no
