#!/usr/bin/env bash
# Run as the normal Raspberry Pi user. Enroll one known serial adapter.
set -euo pipefail
shopt -s nullglob
fail() { echo "ERROR: $*" >&2; exit 1; }
usage() {
    cat <<'HELP'
Usage: bash install-services.sh [--device /dev/ttyUSB0 | --serial-path /dev/serial/by-id/...] [--dshot-max 300|2047] [--telemetry-only] [--workspace PATH] [--domain 42] [--zenoh-config PATH] [--no-start]
If no device is specified, exactly one connected USB/ACM adapter is required.
--serial-path can enroll a known persistent path even while disconnected.
Default TX is enabled with a conservative DShot cap of 300; no auto-arm.
HELP
}
[[ $EUID -ne 0 ]] || fail 'Run without sudo; the installer invokes sudo only for system files.'
user=$(id -un)
[[ $user =~ ^[a-zA-Z_][a-zA-Z0-9_-]*$ ]] || fail 'Unsupported username.'
ws="$HOME/cefiro_ws"; domain=42; zenoh_config="$HOME/zenoh-pi.json5"
max=300; tx=true; start=true; device=; serial=
while (($#)); do
    case "$1" in
        --device|--serial-path|--dshot-max|--workspace|--domain|--zenoh-config)
            (($# >= 2)) || fail "Missing value for $1"
            case "$1" in
                --device) device=$2;; --serial-path) serial=$2;; --dshot-max) max=$2;;
                --workspace) ws=$2;; --domain) domain=$2;; --zenoh-config) zenoh_config=$2;;
            esac; shift 2;;
        --telemetry-only) tx=false; shift;;
        --no-start) start=false; shift;;
        -h|--help) usage; exit 0;;
        *) fail "Unknown option: $1";;
    esac
done
[[ $max =~ ^[0-9]{1,4}$ ]] && ((10#$max >= 48 && 10#$max <= 2047)) || fail 'DShot max must be 48..2047.'
[[ $domain =~ ^[0-9]{1,3}$ ]] && ((10#$domain <= 232)) || fail 'Invalid ROS domain.'
max=$((10#$max)); domain=$((10#$domain))
[[ -z $device || -z $serial ]] || fail 'Choose --device or --serial-path, not both.'
[[ -f /opt/ros/jazzy/setup.bash && -f "$ws/install/setup.bash" ]] || fail 'ROS 2 Jazzy and the built workspace are required.'
[[ -r $zenoh_config ]] || fail "Missing $zenoh_config"
zenoh=$(command -v zenoh-bridge-ros2dds) || fail 'Install zenoh-bridge-ros2dds first.'
[[ $zenoh = /* && -x $zenoh ]] || fail 'Zenoh must resolve to an executable path.'
getent group dialout >/dev/null || fail 'Missing dialout group.'
command -v systemctl >/dev/null || fail 'systemd is required.'
# Source ROS under the normal user, without nounset (ROS setup is not nounset-safe).
( set +u; source /opt/ros/jazzy/setup.bash; source "$ws/install/setup.bash"; ros2 pkg prefix cefiro_hardware_interface >/dev/null ) || fail 'CEFIRO package not found; build the workspace first.'
if [[ -z $serial ]]; then
    if [[ -z $device ]]; then
        devices=(/dev/ttyUSB[0-9]* /dev/ttyACM[0-9]*)
        candidates=(); for d in "${devices[@]}"; do [[ -c $d ]] && candidates+=("$d"); done
        ((${#candidates[@]} == 1)) || fail 'Connect just the CEFIRO adapter or specify --device /dev/ttyUSB0. No arbitrary USB selection.'
        device=${candidates[0]}
    fi
    [[ $device =~ ^/dev/tty(USB|ACM)[0-9]+$ && -c $device ]] || fail "Not a connected USB serial device: $device"
    target=$(readlink -f "$device")
    # A USB serial number is preferable. Otherwise bind to the same USB socket.
    props=$(udevadm info --query=property --name="$device" 2>/dev/null || true)
    links=()
    if grep -q '^ID_SERIAL_SHORT=.' <<< "$props"; then links+=(/dev/serial/by-id/*); fi
    links+=(/dev/serial/by-path/*)
    for link in "${links[@]}"; do
        if [[ -L $link && $(readlink -f "$link") = "$target" ]]; then serial=$link; break; fi
    done
    [[ -n $serial ]] || fail 'No stable /dev/serial/by-id or by-path link; inspect udev before proceeding.'
else
    [[ $serial == /dev/serial/by-id/* || $serial == /dev/serial/by-path/* ]] || fail 'Use a persistent /dev/serial/by-id or by-path path.'
    [[ $(dirname "$serial") == /dev/serial/by-id || $(dirname "$serial") == /dev/serial/by-path ]] || fail 'Invalid serial path.'
fi
ws=$(realpath "$ws"); zenoh_config=$(realpath "$zenoh_config")
# Write literal values compatible with systemd EnvironmentFile.
for value in "$ws" "$serial" "$zenoh" "$zenoh_config"; do
    [[ $value != *$'\n'* && $value != *$'\r'* && $value != *'"'* && $value != *'\'* && $value != *'$'* && $value != *'`'* ]] || fail 'Unsupported characters in configuration path.'
done
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
tmp=$(mktemp -d); trap 'rm -rf -- "$tmp"' EXIT
for svc in cefiro-hardware cefiro-zenoh; do sed "s/@USER@/$user/g" "$here/$svc.service" > "$tmp/$svc.service"; done
cat > "$tmp/pi-services.env" <<ENV
ROS_DOMAIN_ID=$domain
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
CEFIRO_WS="$ws"
CEFIRO_SERIAL="$serial"
CEFIRO_ENABLE_TX=$tx
CEFIRO_DSHOT_MAX=$max
CEFIRO_ZENOH_BIN="$zenoh"
CEFIRO_ZENOH_CONFIG="$zenoh_config"
ENV
printf 'Adapter: %s\nDomain: %s\nMotor TX: %s (DShot cap %s)\n' "$serial" "$domain" "$tx" "$max"
echo 'Stop any manually launched hardware and Zenoh processes before continuing. No automatic arming is performed.'
sudo -v
sudo install -d -m 0755 /etc/cefiro /usr/local/lib/cefiro
sudo install -m 0644 "$tmp/pi-services.env" /etc/cefiro/pi-services.env
sudo install -m 0755 "$here/run-hardware.sh" "$here/run-zenoh.sh" /usr/local/lib/cefiro/
sudo install -m 0644 "$tmp/cefiro-hardware.service" "$tmp/cefiro-zenoh.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable cefiro-zenoh.service cefiro-hardware.service
if [[ $start == true ]]; then
    sudo systemctl restart cefiro-zenoh.service cefiro-hardware.service
    systemctl status cefiro-zenoh.service cefiro-hardware.service --no-pager || true
else
    echo 'Enabled at boot; start manually when ready.'
fi
