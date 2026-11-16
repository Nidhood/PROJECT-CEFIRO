#!/usr/bin/env bash
# Install Zenoh Bridge (run directly on Raspberry Pi Ubuntu/Debian).
# Comments in English. No Docker installation is needed on the Pi.
set -euo pipefail
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key \
   | sudo gpg --dearmor --yes -o /etc/apt/keyrings/zenoh-public-key.gpg
echo 'deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /' \
   | sudo tee /etc/apt/sources.list.d/zenoh.list >/dev/null
sudo apt-get update
sudo apt-get install -y zenoh-bridge-ros2dds
echo "Installed zenoh-bridge-ros2dds."
echo "Start bridge with: ROS_DOMAIN_ID=30 zenoh-bridge-ros2dds -c ~/zenoh-pi.json5"
