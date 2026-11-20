#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
c++ -std=c++20 -Wall -Wextra -Wpedantic -Werror -pthread -I "$ROOT/include" \
  "$ROOT/src/tcp_port.cpp" "$ROOT/src/serial_port.cpp" "$ROOT/src/protocol.cpp" \
  "$ROOT/tools/test_tcp_transport.cpp" -o "$TMP/test_tcp"
"$TMP/test_tcp"
