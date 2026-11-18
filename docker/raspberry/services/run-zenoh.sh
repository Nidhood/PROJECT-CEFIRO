#!/usr/bin/env bash
# Reuse the existing tested Pi configuration; systemd restarts on failure.
set -euo pipefail
: "${CEFIRO_ZENOH_BIN:?Missing Zenoh executable}" "${CEFIRO_ZENOH_CONFIG:?Missing configuration}"
[[ -x "$CEFIRO_ZENOH_BIN" && -r "$CEFIRO_ZENOH_CONFIG" ]] || {
    echo 'Zenoh executable or configuration unavailable.' >&2; exit 1;
}
exec "$CEFIRO_ZENOH_BIN" -c "$CEFIRO_ZENOH_CONFIG"
