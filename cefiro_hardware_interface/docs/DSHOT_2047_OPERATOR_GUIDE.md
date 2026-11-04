# CEFIRO hardware interface v0.5.0 — configurable 0 / 48..2047

The STM32H743 accepts DShot **throttle codes** 0 (stop) and 48..2047 (active); codes 1..47 are ESC special commands and are never emitted by the manual-RPM adapter. DShot300 is the serial protocol **bitrate** (300 kbit/s), NOT the throttle ceiling.

The Raspberry bridge's separate `dshot_max` ceiling **defaults to 300** to protect commissioning. It can be changed **only at node startup** using `dshot_max:=...` (integer 48..2047, no propellers, measured current/temperature, restrained frame). The GUI slider's 0..12000 "RPM" is *nominal uncalibrated command scale*, never measured RPM. `12000 nominal` maps to configured `dshot_max`.

## Receive-only smoke test
```bash
ros2 launch cefiro_hardware_interface hardware.launch.py \
  port:=/dev/ttyUSB1 protocol_mode:=cf1 \
  enable_command_tx:=false rpm_to_dshot_enabled:=false
```

## Supervised low-limit bench mode (default 300)
```bash
ros2 launch cefiro_hardware_interface hardware.launch.py \
  port:=/dev/ttyUSB1 protocol_mode:=cf1 \
  enable_command_tx:=true rpm_to_dshot_enabled:=true \
  dshot_max:=300
```

## Raising after deliberate verification
```bash
ros2 launch cefiro_hardware_interface hardware.launch.py \
  port:=/dev/ttyUSB1 protocol_mode:=cf1 \
  enable_command_tx:=true rpm_to_dshot_enabled:=true \
  dshot_max:=2047
```
Use intermediate caps first, and stop at any overheating, overload, poor ESC response, vibration or battery sag. Maximum DShot command is **not** maximum rotor RPM. Emergency stop, explicit ARM, serial failure recovery, host GUI lease (600 ms), STM32 host watchdog (300 ms), ESC failsafe (500 ms) and power interlocks remain active. The bridge does not auto-rearm after faults.

Check runtime configuration:
```bash
ros2 topic echo /real/cefiro/hardware/status --full-length --once
```
Expected JSON fields: `hardware_interface_version: "0.5.0"`, `configured_dshot_max`, `last_commanded_dshot`. `last_commanded_dshot` is a **command request**, not RPM measurement or ESC acceptance guarantee. Check `last_command_ack` and power states independently.

Install `cefiro_msgs` first; clean-build package from within the sourced ROS 2 Jazzy workspace:
```bash
cd ~/cefiro_ws
bash src/cefiro_hardware_interface/tools/install_and_verify.sh
```
