# CEFIRO GUI v1.5.0 — nominal RPM / real DShot request display

- Retains Qt6 control dashboard and 4 independent motor sliders + MASTER request.
- GUI `0..12000` remains a **nominal RPM request scale**, NOT motor RPM sensing or closed-loop speed regulation.
- New subscription to `/real/cefiro/hardware/status` reports `configured_dshot_max` (300 default, 2047 protocol ceiling), UART connectivity, and last requested DShot code on each motor. Labels show requested values, not measured RPM or a guarantee of MCU ACK.
- Removal of the ineffective "Reverse" option; the ESC command stream remains normal on M1..M4. Swapping 2 motor phase wires with power OFF determines physical direction on M1 and M4.
- GUI does not itself bypass limits. Set the Raspberry cap with `dshot_max:=...` at the hardware launch; the GUI learns it from host telemetry.
- No change to safe-stop, emergency stop, ARM or periodic GUI output.

To rebuild on ASUS (after extracting in `~/cefiro_ws/src/`):
```bash
cd ~/cefiro_ws
source /opt/ros/jazzy/setup.bash
rm -rf build/cefiro_gui install/cefiro_gui
colcon build --symlink-install --packages-up-to cefiro_gui --cmake-clean-cache
source install/setup.bash
ros2 launch cefiro_hardware_interface gui_real.launch.py
```
Make sure the correct package overlay is sourced. Qt6/ROS 2 Jazzy compilation must be verified on the target ASUS.
