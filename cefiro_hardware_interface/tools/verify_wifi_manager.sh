#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir "$TMP/bin"
cp "$ROOT/tools/fake_nmcli.py" "$TMP/bin/nmcli"
chmod +x "$TMP/bin/nmcli"
cat > "$TMP/bin/ip" <<'IP'
#!/usr/bin/env bash
set -eu
[[ "$*" == '-4 route get 192.168.4.1' ]]
printf '192.168.4.1 dev %s src 192.168.4.2\n' "$(cat "$CF_WIFI_TEST_DIR/route")"
IP
chmod +x "$TMP/bin/ip"
printf 'wlan1' > "$TMP/route"
export CF_WIFI_TEST_DIR="$TMP" CEFIRO_WIFI_PASSWORD='TestPassword123!'
export PATH="$TMP/bin:$PATH"
c++ -std=c++20 -Wall -Wextra -Wpedantic -Werror -pthread -I "$ROOT/include" \
  "$ROOT/src/wifi_manager.cpp" "$ROOT/tools/test_wifi_manager.cpp" -o "$TMP/test_wifi"
"$TMP/test_wifi"
[[ ! -e "$(cat "$TMP/password_path")" ]]
echo 'Password file cleanup: PASS'
