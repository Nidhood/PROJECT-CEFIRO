#!/usr/bin/env bash
# Optional: install ROS 2 hardware-interface dependencies on an Ubuntu 24.04 Pi.
# Safe: compile only. Does not open serial ports or send motor commands.
set -euo pipefail

WS="${1:-$HOME/cefiro_ws}"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is missing. Install the official ROS 2 Jazzy Ubuntu packages first.' >&2
  exit 1
fi
if [[ ! -d "$WS/src/PROJECT-CEFIRO/cefiro_hardware_interface" ]]; then
  echo "Missing $WS/src/PROJECT-CEFIRO/cefiro_hardware_interface" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions python3-rosdep \
    ros-jazzy-rclcpp ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs ros-jazzy-std-srvs ros-jazzy-launch-ros \
    ros-jazzy-rosidl-default-generators

source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --base-paths src --packages-up-to cefiro_hardware_interface \
    --symlink-install --parallel-workers 2
source install/setup.bash
ros2 pkg prefix cefiro_hardware_interface

echo 'CEFIRO Pi hardware interface dependencies installed; NO hardware node has been started.'
echo 'To view topics: ros2 topic list | grep ^/real/'
echo 'Use enable_command_tx:=false for telemetry-only testing.'
