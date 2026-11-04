#!/usr/bin/env bash
# Clean rebuild and verify the correct ROS 2 Jazzy hardware_node executable.
# Run after unzipping this package into ~/cefiro_ws/src/.
set -Eeuo pipefail
WS="${1:-$HOME/cefiro_ws}"
PKG="$WS/src/cefiro_hardware_interface"
if [[ ! -f "$PKG/CMakeLists.txt" || ! -f "$PKG/src/hardware_node.cpp" ]]; then
  echo "ERROR: package not found at $PKG (unzip the archive to $WS/src first)" >&2
  exit 2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ERROR: ROS 2 Jazzy is not installed at /opt/ros/jazzy' >&2
  exit 2
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo 'ERROR: colcon not found. Install ROS 2 colcon tooling first.' >&2
  exit 2
fi
if pgrep -fa '(/hardware_node|cefiro_hardware_interface/hardware_node)' >/dev/null 2>&1; then
  echo 'ERROR: hardware_node still running. Stop the ros2 launch before rebuilding.' >&2
  exit 2
fi
# Preserve the user's existing workspace and the other ROS packages.
cd "$WS"
source /opt/ros/jazzy/setup.bash
if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash"
fi
printf '\n[1/3] Clean old cefiro_hardware_interface binary and install folder\n'
rm -rf "$WS/build/cefiro_hardware_interface" "$WS/install/cefiro_hardware_interface"
printf '\n[2/3] Build package and its workspace dependencies\n'
colcon build --symlink-install --packages-up-to cefiro_hardware_interface \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source "$WS/install/setup.bash"
printf '\n[3/3] Verify the installed binary (not just the source)\n'
BIN="$WS/install/cefiro_hardware_interface/lib/cefiro_hardware_interface/hardware_node"
if [[ ! -x "$BIN" ]]; then
  echo "ERROR: binary missing at $BIN" >&2
  exit 3
fi
VERSION="$("$BIN" --version)"
echo "Binary: $BIN"
echo "Version: $VERSION"
if [[ "$VERSION" != 'cefiro_hardware_interface 0.4.1' ]]; then
  echo 'ERROR: stale binary detected; please check the overlay and package paths.' >&2
  exit 3
fi
PREFIX="$(ros2 pkg prefix cefiro_hardware_interface)"
echo "ROS package prefix: $PREFIX"
if [[ "$PREFIX" != "$WS/install/cefiro_hardware_interface" ]]; then
  echo "WARNING: ROS resolved a different prefix; open a fresh terminal and source $WS/install/setup.bash" >&2
fi
printf '\nOK: hardware interface 0.4.1 installed. Start in receive-only mode first.\n'
