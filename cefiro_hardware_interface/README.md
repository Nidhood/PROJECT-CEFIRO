# cefiro_hardware_interface — C++20 / ROS 2 Jazzy / CEFIRO FMU V3

This package bridges the **actual** CEFIRO STM32H743 modular firmware to the
CEFIRO Qt6 HMI using Linux USB-to-USART (115200 8N1). Simulation remains isolated
from the real plant under `/real`.

## Verified integration contract (from `CEFIRO_FMU_V3_Modular_Main_Explicito.zip`)

* `cf_protocol.c`, `cf_link.c`: CF1 ASCII with CRC16-CCITT and `@CF1|seq|KIND|a,b*FFFF\n`.
* `cf_fmu.c`: `CMD_MODE MANUAL`, `CMD_ARM 1/0`, `CMD_DSHOT 4 values`, `CMD_STOP`,
  `CMD_ESTOP`, `CMD_RESET_FAULT`, `CMD_LQR` (18 values) and `CMD_ATT` subject to
  firmware interlocks.
* **`CMD_RPM` is explicitly `UNSUPPORTED`**. There is no measured motor RPM.
* **`CMD_DIR` and `CMD_PID` are explicitly `UNSUPPORTED`**.
* `CF_MANUAL_MAX_DSHOT=300`; accepted throttle codes: `0` or `48..300`.
* `CF_HOST_COMMAND_TTL_MS=300`: the STM32 autonomously stops manual output after
  loss of commands. The Raspberry Pi uses **200 ms** command watchdog by default.
* `CF_ENABLE_AUTOMATIC_OUTPUT=0`: automatic attitude-control actuation is blocked
  in the firmware. **Do not bypass this protection in the host bridge**.
* `IMU`, `ATT`, `MAG`, `BARO`, `STAT`, `ACK` are emitted in CF1. `ATT` contains a
  quaternion from the STM32 estimator (yaw drifts without calibrated mag yaw).

## Compatibility with the existing C++ GUI

| GUI topic / message | Bridge action | STM32 CF1 action |
|---|---|---|
| `/real/cefiro/control/enable` (`std_msgs/Bool`) | GUI output gate | **NOT** forwarded as `CMD_ENABLE=1` (auto inhibited) |
| `/real/cefiro_actuators/armed` (`std_msgs/Bool`) | Explicit ARM request only after GUI output enable, MCU STAT, and CMD_MODE ACK | `CMD_ARM,1` → matching `ACK` |
| `/real/cefiro_actuators/target_rpm` (`cefiro_msgs/MotorRpm`) | Optional, explicitly enabled **open-loop** scaling; per motor, preserves MASTER and independent selection | `CMD_DSHOT,d1,d2,d3,d4` |
| `/real/cefiro_actuators/dshot_throttle` (`cefiro_msgs/MotorDshot`) | Direct bounded manual bench command after ARM ACK; also needs `allow_direct_dshot=true` | `CMD_DSHOT` |
| `/real/cefiro_actuators/motor_direction` (`std_msgs/Int8MultiArray`) | `+1` tolerated; reverse/invalid values rejected, with stop if active | `CMD_DIR` **never** sent |
| `/real/cefiro/control/emergency_stop` (`std_msgs/Bool`) | If `true`, latch emergency stop | `CMD_ESTOP` (requires explicit fault reset) |
| GUI SAFE STOP / `enable=false` / `armed=false` | Stop, disarm, clear stale commands | `CMD_STOP`, `CMD_ARM,0` |
| `/real/cefiro/control/attitude_reference` | Telemetry/reference interface only | **No motor output** while firmware auto output is inhibited |
| `/real/cefiro/control/lqr_gain` | Configure only when GUI output OFF and motors disarmed | `CMD_LQR` |

The GUI's numeric RPM **is only a nominal position on the GUI scale**, NOT
measured RPM. Default explicit mapping for a nonzero nominal GUI request is:

```
DShot = 48 + round((GUI_RPM - 25) / (12000 - 25) * (300 - 48))
GUI_RPM <= 25 -> DShot 0
```

This mapping assumes **no actual RPM calibration**, cannot regulate RPM,
and must never be reported as motor-speed feedback. It is not enabled by default.
The exact upper bound and lower bound are parameters, and must remain within
firmware `48..300`. The host never emits special DShot commands `1..47`.

## Build only the real hardware package

```bash
cd ~/cefiro_ws
source /opt/ros/jazzy/setup.bash
rm -rf build/cefiro_hardware_interface install/cefiro_hardware_interface
colcon build --symlink-install --packages-up-to cefiro_hardware_interface \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

This package uses **C++20, `rclcpp`, `ament_cmake`, and `termios`**. There is no
`rclpy` or `pyserial`; Python is used only by ROS launch files.

## Stage A — verify firmware CF1 telemetry, no motor output

**Remove propellers, isolate ESC power as appropriate, secure the gimbal.**
Use the actual persistent CP2102 device name from `ls -l /dev/serial/by-id/`,
not a guessed tty index. Example:

```bash
ros2 launch cefiro_hardware_interface hardware.launch.py \
  port:=/dev/ttyUSB0 protocol_mode:=cf1 \
  enable_command_tx:=false rpm_to_dshot_enabled:=false
```

Separate terminal:

```bash
source ~/cefiro_ws/install/setup.bash
ros2 topic echo /real/cefiro/hardware/status
ros2 topic echo /real/imu/data
ros2 topic echo /real/cefiro/control/status
```

Expect `mcu_status` beginning with `FMU_V3`, `rx_valid_frames` increasing and
`orientation_available=true` once `ATT` frames arrive. If valid frames stay at
zero but plain IMU text exists, **the old firmware is still running**: reflash
the modular STM32 firmware; do not enable CF1 command TX.

To continue using the old text-only STM32 binary, instead launch
`protocol_mode:=legacy_text` with TX disabled.

## Stage B — bench-only manual GUI command interface

This stage is **only for a supervised bench test with propellers removed and
confirmed firmware power/ESC interlocks**. Do **not** enable TX in flight or
with exposed spinning propellers. Start from all four GUI sliders at zero.

After successful Stage A and confirming that the modular firmware is flashed:

```bash
ros2 launch cefiro_hardware_interface hardware.launch.py \
  port:=/dev/ttyUSB0 protocol_mode:=cf1 \
  enable_command_tx:=true rpm_to_dshot_enabled:=true
```

In a **different terminal** on a machine with a GUI display:

```bash
source ~/cefiro_ws/install/setup.bash
ros2 launch cefiro_hardware_interface gui_real.launch.py
```

In CEFIRO GUI select `MANUAL`, `Output enabled`, then `START TEST`. The GUI
publishes periodic 4-motor setpoints. The Raspberry Pi will not send throttle
until the STM32 acknowledges manual mode and arming. If startup outputs
`DENIED`, check battery validity, FMU `STAT`, attached ESC, and firmware.

When manually adjusting one motor, the other three retain their individual
values. The GUI's `Link all motors to master` synchronizes requests; after
unlinking it retains each channel's last setting. Reverse direction is
**not supported** by this firmware, and selecting it stops output.

Observe live diagnostics:

```bash
ros2 topic echo /real/cefiro/hardware/status
ros2 topic echo /real/cefiro/hardware/command_ack
ros2 topic echo /real/cefiro/hardware/commanded_dshot
```

The `commanded_dshot` message is **requested**, not measured motor feedback;
`measured_rpm` and `telemetry` remain silent until a real telemetry source
is implemented on the STM32.

**If EMERGENCY ZERO was requested**, the STM32 deliberately enters latched
FMU FAULT. After resolving the cause, stopping the GUI and verifying motors
are stopped, explicitly request reset:

```bash
ros2 service call /real/cefiro/hardware/reset_fault std_srvs/srv/Trigger '{}'
ros2 topic echo /real/cefiro/hardware/command_ack
```

The service only confirms that the reset command was queued; an `ACK OK` and
a subsequent `STAT` without FAULT are required before trying GUI ARM again.

## Real topics

**Sensor publishers:**
`/real/imu/data`, `/real/magnetometer/data`, `/real/bmp581/air_pressure`,
`/real/cefiro/sensors/imu_temperature`,
`/real/cefiro/sensors/baro_temperature`,
`/real/cefiro/sensors/baro/isa_altitude`,
`/real/cefiro/sensors/baro/relative_altitude`.

**Status:**
`/real/cefiro/hardware/status` (JSON in `std_msgs/String`),
`/real/cefiro/hardware/command_ack` (CF1 ACK summary),
`/real/cefiro/hardware/commanded_dshot` (`MotorDshot` request),
`/real/cefiro/control/status` (human-readable MCU state).

**Reserved, NOT fabricated:**
`/real/cefiro_actuators/measured_rpm`,
`/real/cefiro_actuators/telemetry`.

## Additional safety notes

* `hardware.launch.py` defaults to TX OFF, nominal RPM conversion OFF,
  and direct DShot input OFF.
* On port reconnect, output gate and ARM intent are cleared; no stale commands
  are resent. A new explicit GUI action is required.
* The firmware independently enforces 300 ms timeout, battery limits and
  emergency latch. Host timestamps use a steady clock for timeouts.
* The `CMD_STOP`/`CMD_ARM,0` path does **not** latch FMU FAULT, unlike `CMD_ESTOP`.
* No attempt is made to activate automatic attitude control while the firmware
  configuration explicitly inhibits it.
* Only one process can open `/dev/ttyUSB0`. Close `minicom` before launching.

Test the ROS-independent codec without a hardware device:

```bash
bash tools/verify_protocol.sh
```

See [`docs/PROTOCOL.md`](docs/PROTOCOL.md) for the command/ACK contract.
