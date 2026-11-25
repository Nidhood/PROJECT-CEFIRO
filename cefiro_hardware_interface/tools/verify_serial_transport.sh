#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
bin=$(mktemp)
trap 'rm -f "$bin"' EXIT
g++ -std=c++20 -Wall -Wextra -Werror -Iinclude src/serial_port.cpp tools/test_serial_transport.cpp -o "$bin"
"$bin"
