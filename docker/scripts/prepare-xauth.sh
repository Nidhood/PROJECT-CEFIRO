#!/usr/bin/env bash
# Ubuntu Linux host only; avoids xhost + and forwards just this X11 cookie.
set -euo pipefail
: "${DISPLAY:?DISPLAY must be set on Ubuntu desktop}"
command -v xauth >/dev/null || { echo 'Install: sudo apt install xauth'; exit 1; }
XAUTH_FILE="${XAUTHORITY_FILE:-/tmp/cefiro-docker.xauth}"
touch "$XAUTH_FILE"
chmod 600 "$XAUTH_FILE"
# Wild-family auth permits container hostnames to use the current display cookie.
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH_FILE" nmerge -
if [[ ! -s "$XAUTH_FILE" ]]; then
  echo 'No cookie exported; verify X11/XWayland and xauth first.' >&2
  exit 1
fi
printf 'Created %s\n' "$XAUTH_FILE"
