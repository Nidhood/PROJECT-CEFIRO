# Direct Wi-Fi transport (v0.6.0)

The same C++ ROS node supports both the original POSIX serial transport and a
native, nonblocking TCP stream. The ESP32-C3 forwards the existing CF1 frames;
no PTY, socat, Python bridge, or ROS topic remapping is required. The default
endpoint is `192.168.4.1:9000` (CEFIRO_FMU_V3 access point).

## Host network setup (one time)

Connect the Linux host running `hardware_node` to CEFIRO_FMU_V3. On a Raspberry
Pi using NetworkManager, run locally (changing Wi-Fi may interrupt SSH):

```bash
nmcli device wifi list
nmcli --ask device wifi connect CEFIRO_FMU_V3
nmcli connection modify CEFIRO_FMU_V3 connection.autoconnect yes
ip route get 192.168.4.1
```

Use the configured ESP32 password when prompted. If the saved profile has a
 different name, use that name in the second `nmcli connection` command. The
node does not change OS Wi-Fi profiles or store Wi-Fi credentials. TCP connects
on launch, and reconnects automatically when the network becomes reachable.
If using Docker, the container must have a route to the AP; on Linux native
host networking is one option. Do not run the Python GUI/monitor and ROS
hardware node simultaneously: the ESP32 supports one TCP peer. Do not run
serial and Wi-Fi hardware nodes under the same ROS namespace simultaneously.

## Build

Replace the old package in `~/cefiro_ws/src/cefiro_hardware_interface` (not
another copy in the workspace). Stop the previously running node first.

```bash
cd ~/cefiro_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to cefiro_hardware_interface
source install/setup.bash
ros2 run cefiro_hardware_interface hardware_node --version
```

Version should be `cefiro_hardware_interface 0.6.0`.

## Launch

```bash
# Telemetry-only, automatic TCP connection/reconnect
ros2 launch cefiro_hardware_interface hardware_wifi.launch.py

# Original serial path remains available (legacy_text or cf1 as applicable)
ros2 launch cefiro_hardware_interface hardware.launch.py port:=/dev/ttyUSB0 protocol_mode:=cf1

# Existing GUI, if installed, in a separate terminal
ros2 launch cefiro_hardware_interface gui_real.launch.py
```

After validation with propellers removed and ESC power isolated as appropriate,
opt in to command TX and nominal-RPM mapping on a secured bench:

```bash
ros2 launch cefiro_hardware_interface hardware_wifi.launch.py \
  enable_command_tx:=true rpm_to_dshot_enabled:=true dshot_max:=300
```

This is not measured RPM control. Existing firmware guards, ACK-gated manual
arm, 40 ms refresh, 600 ms GUI lease and fault-reset service are preserved.
Connection loss clears host-side queued commands and arm state; a new GUI
enable/arm action is required by the existing state machine. TCP is not a
hard-real-time control link, and delivery of a stop cannot be guaranteed over a
failed connection. The independent STM32 watchdog remains mandatory.

| Launch parameter | Default |
|---|---|
| namespace | real |
| wifi_host (numeric IPv4) | 192.168.4.1 |
| wifi_port | 9000 |
| reconnect_s | 2.0 |
| wifi_connect_timeout_s | 3.0 |
| wifi_rx_timeout_s | 3.0 |
| enable_command_tx | false |
| rpm_to_dshot_enabled | false |
| allow_direct_dshot | false |
| dshot_max | 300 |

TCP connection attempts are nonblocking. EOF, I/O errors and the absence of
CRC-valid CF1 frames for `wifi_rx_timeout_s` reset the connection. The TCP
socket status alone is not a sensor health indicator. The status JSON keeps
`serial_connected` as a legacy alias for the active transport so existing GUI
consumers continue to work; `transport`, `transport_connected`, `wifi_connected`
and `transport_errors` explicitly describe the current link.

## ROS graph

All existing publishers, subscriptions and service names are preserved;
namespace `real` by default. The node only publishes actual received data,
with the original conversion units and ROS receive timestamps. CF1 `BAT` and
`PWR` from the V4 firmware add:

* `/real/cefiro/sensors/battery` (`sensor_msgs/msg/BatteryState`): valid voltage,
  fraction 0..1 if valid, negative discharge current if calibrated; unknown
  values are NaN. `present` reflects the firmware voltage-valid flag, not a
  physical battery presence detector. Inspect status flags for invalid data.
* `/real/cefiro/hardware/power_status` (`std_msgs/msg/String`): original PWR CSV
  `adc_v_raw,adc_i_raw,bat_pin_mV,current_pin_mV,esc_current_mV,adc_fresh,init_stage,dma_errors,adc_sequence,bench_mode,esc_armed,sample_ms`.
* Hardware status JSON adds battery/power freshness and the raw BAT CSV
  `voltage_v,current_a,soc_percent,level,flags,sample_ms`. The flags are integer
  bit values transmitted as decimal floats by the firmware (1 voltage valid,
  2 current calibrated, 4 SOC valid, 8 ADC fresh, 16 bench mode).

```bash
ros2 topic list
ros2 topic echo /real/cefiro/hardware/status
ros2 topic echo /real/cefiro/sensors/battery
ros2 topic echo /real/cefiro/hardware/power_status
ros2 topic hz /real/bmp581/air_pressure
```

No battery frames are invented if an older firmware is in use. The V4 Wi-Fi
firmware already supports TCP CF1; no MCU reflash is required for this change.
The original serial launch is retained for firmware exposing CF1 or legacy
telemetry over UART; V4 Wi-Fi firmware may reserve its USB-UART console for
maintenance instead of CF1, so the serial launch does not magically change the
MCU's selected link.

## Tests without ROS

```bash
bash tools/verify_protocol.sh
bash tools/verify_manual_keepalive.sh
bash tools/verify_tcp_transport.sh
```

The transport test uses a real local TCP socket, verifies fragmented and merged
CF1 frames (including BAT/PWR), command/ACK, EOF, reconnection and failure to
connect. Full ROS Jazzy/cefiro_msgs compilation and physical device testing
must be performed in the target workspace.
