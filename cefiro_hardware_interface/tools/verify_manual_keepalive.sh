#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXE="$(mktemp /tmp/cefiro-manual-keepalive-XXXXXX)"
trap 'rm -f "$EXE"' EXIT
c++ -std=c++20 -Wall -Wextra -Wpedantic -Werror \
  -I "$ROOT/include" "$ROOT/tools/test_manual_keepalive.cpp" -o "$EXE"
"$EXE"
