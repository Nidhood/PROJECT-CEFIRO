#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"
PID=''
trap 'if [[ -n "$PID" ]]; then kill "$PID" 2>/dev/null || true; wait "$PID" 2>/dev/null || true; fi; rm -rf "$TMP"' EXIT
mkdir "$TMP/bin"
# Disable the NSS fallback to exercise UDP alone. Run only in an isolated test LAN.
printf '#!/bin/sh\nexit 2\n' > "$TMP/bin/getent"
chmod +x "$TMP/bin/getent"
export PATH="$TMP/bin:$PATH"
c++ -std=c++20 -Wall -Wextra -Wpedantic -Werror -pthread -I "$ROOT/include" "$ROOT/src/wifi_manager.cpp" "$ROOT/tools/test_udp_discovery.cpp" -o "$TMP/test"
python3 - "$TMP/ready" <<'PY' &
import socket,sys,re
from pathlib import Path
with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
    s.bind(('0.0.0.0',9001));Path(sys.argv[1]).touch();s.settimeout(10)
    while True:
        data, address=s.recvfrom(1024)
        m=re.fullmatch(rb'CEFIRO_DISCOVER_V1 ([0-9a-f]{16})',data)
        if m:s.sendto(b'CEFIRO_FMU_V3 '+m.group(1)+b' 9000',address)
PY
PID=$!
for _ in $(seq 1 50); do [[ -f "$TMP/ready" ]] && break; sleep .05; done
[[ -f "$TMP/ready" ]]
"$TMP/test"
