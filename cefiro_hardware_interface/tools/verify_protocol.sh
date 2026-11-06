#!/usr/bin/env bash
# Standalone C++ quick check. Does not require ROS or the STM32 on UART.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
c++ -std=c++20 -Wall -Wextra -Wpedantic -Werror \
  -I"$HERE/include" \
  "$HERE/src/protocol.cpp" \
  "$HERE/src/motor_command.cpp" \
  "$HERE/tools/verify_protocol.cpp" \
  -o "$TMP/verify_protocol"
"$TMP/verify_protocol"
