#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir "$TMP/bin"
cp "$ROOT/tools/fake_nmcli.py" "$TMP/bin/nmcli"
cat > "$TMP/bin/ip" <<'IP'
#!/usr/bin/env bash
printf '192.168.4.1 dev wlan1 src 192.168.4.2\n'
IP
cat > "$TMP/bin/getent" <<'GET'
#!/usr/bin/env bash
set -eu
[[ "$*" == 'ahostsv4 lab-cefiro.local' ]]
[[ -f "$CF_WIFI_TEST_DIR/resolve" ]] || exit 2
printf '%s STREAM test\n' "$(cat "$CF_WIFI_TEST_DIR/resolve")"
GET
chmod +x "$TMP/bin/"*
export CF_WIFI_TEST_DIR="$TMP" CEFIRO_WIFI_PASSWORD='TestPassword123!'
export PATH="$TMP/bin:$PATH"
c++ -std=c++20 -Wall -Wextra -Wpedantic -Werror -pthread -I "$ROOT/include" "$ROOT/src/wifi_manager.cpp" "$ROOT/tools/test_lan_discovery.cpp" -o "$TMP/test"
"$TMP/test"
