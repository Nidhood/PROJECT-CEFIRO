# cefiro_hardware_interface

ROS 2 Jazzy package (`ament_python`) for a **Raspberry Pi connected by USB-to-UART** to `CEFIRO_FMU_V3`, based on the provided STM32H743 `main.c`. All hardware ROS topics are in **`/real`**, leaving Gazebo simulation topics untouched.

## What works with the uploaded firmware, today

`main.c` currently prints IMU/MAG/BARO human-readable text via **USART2 PA2/PA3, 115200 baud, 8N1**. The package supports this directly via `protocol_mode=legacy_text`, **read-only**: it does not need you to flash a new MCU image.

ROS conversions:

- IMU accel m/s² unchanged, gyro **degrees/s -> radians/s**.
- Magnetic field **µT -> Tesla**.
- Pressure **hPa -> Pa**, using current printed (filtered) `P=` value.
- Barometer `Alt_ISA` and `Z_REL` as supplemental Float64 topics in m.
- IMU/MAG/BARO temperature as supplemental sensor_msgs/Temperature topics in °C.
- Current `main.c` has **no quaternion**. `Imu.orientation_covariance[0]=-1` advertises that the orientation is unavailable. Your GUI's roll/pitch/yaw 3D view will **not** be driven until the MCU or another estimator provides attitude. No orientation data are fabricated.
- There is no real ESC/RPM telemetry in current `main.c`, so motor measurement topics remain silent until firmware supports them.

## Topics (under /real by default)

| ROS topic | Type | Direction relative to Raspberry Pi | Actual firmware support |
|---|---|---|---|
| `/real/imu/data` | `sensor_msgs/msg/Imu` | publish | legacy RX |
| `/real/magnetometer/data` | `sensor_msgs/msg/MagneticField` | publish | legacy RX |
| `/real/bmp581/air_pressure` | `sensor_msgs/msg/FluidPressure` | publish | legacy RX (actual sensor LPS22HB; topic alias retained for compatibility) |
| `/real/cefiro/sensors/imu_temperature` | `sensor_msgs/msg/Temperature` | publish | legacy RX |
| `/real/cefiro/sensors/baro_temperature` | `sensor_msgs/msg/Temperature` | publish | legacy RX |
| `/real/cefiro/sensors/baro/isa_altitude` | `std_msgs/msg/Float64` | publish | legacy RX |
| `/real/cefiro/sensors/baro/relative_altitude` | `std_msgs/msg/Float64` | publish | legacy RX |
| `/real/cefiro/hardware/status` | `std_msgs/msg/String` (JSON) | publish | both modes |
| `/real/cefiro/control/status` | `std_msgs/msg/String` | publish | CF1 future |
| `/real/cefiro_actuators/measured_rpm` | `cefiro_msgs/msg/MotorRpm` | publish | CF1 future |
| `/real/cefiro_actuators/telemetry` | `cefiro_msgs/msg/MotorTelemetry` | publish | CF1 `MOTOR_FULL` future |
| `/real/cefiro_actuators/target_rpm` | `cefiro_msgs/msg/MotorRpm` | subscribe | CF1 future |
| `/real/cefiro_actuators/dshot_throttle` | `cefiro_msgs/msg/MotorDshot` | subscribe | CF1 future |
| `/real/cefiro_actuators/armed` | `std_msgs/msg/Bool` | subscribe | CF1 future |
| `/real/cefiro_actuators/motor_direction` | `std_msgs/msg/Int8MultiArray` | subscribe | CF1 future, opt-in |
| `/real/cefiro/control/attitude_reference` | `geometry_msgs/msg/Vector3Stamped` | subscribe | CF1 future |
| `/real/cefiro/control/enable`, `/real/cefiro/control/emergency_stop` | `std_msgs/msg/Bool` | subscribe | CF1 future |
| `/real/cefiro/control/mode`, `/real/cefiro/control/strategy` | `std_msgs/msg/String` | subscribe | CF1 future |
| `/real/cefiro/control/pid_gains`, `/real/cefiro/control/lqr_gain` | `std_msgs/msg/Float64MultiArray` | subscribe | CF1 future |

**Names are compatibility aliases**: `/real/bmp581/air_pressure` deliberately uses the existing `cefiro_gui` subscriber's topic even though `main.c` actually uses an **LPS22HB**. This does not mean the physical device is a BMP581. Frame IDs should be mapped to your real CAD/TF frames before using navigation.

## Install on Raspberry Pi (Ubuntu 24.04 / ROS 2 Jazzy)

The RPi and the PC running the existing GUI can join the same ROS domain if you configure the ROS network appropriately. Build this package and `cefiro_msgs`; **do not run the simulated `cefiro_actuators` process for the real plant**.

```bash
sudo apt update
sudo apt install python3-serial

cd ~/cefiro_ws/src
# Extract the ZIP here, yielding src/cefiro_hardware_interface/
cd ~/cefiro_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select cefiro_msgs cefiro_hardware_interface
source install/setup.bash
```

If you do not have `cefiro_msgs` yet, copy its existing package from your development workspace (the message definitions must be identical on PC and Pi).

Inspect the actual adapter path:

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
# Raspberry Pi user must have read/write access to the serial device.
sudo usermod -aG dialout "$USER"
# Re-login so supplementary group membership is refreshed.
```

**Wiring**: STM32 USART2 TX -> USB-UART RX, STM32 USART2 RX <- USB-UART TX (needed for future commands), common GND; use **3.3V UART logic**. Connect USB-UART USB to Pi. Power the STM32 separately and correctly; **do not connect 5V USB-UART TTL output to STM32 3.3V pin**. Confirm CP2102/adapter voltage selection and grounds. Only one process can open the serial device at a time.

### Run current `main.c` (real sensor telemetry)

```bash
ros2 launch cefiro_hardware_interface hardware.launch.py port:=/dev/ttyUSB0

# Separate terminal (source the same workspace):
ros2 topic echo /real/imu/data
ros2 topic echo /real/magnetometer/data
ros2 topic echo /real/bmp581/air_pressure
ros2 topic echo /real/cefiro/hardware/status
```

Optional GUI for the *real* plant, **only if cefiro_gui is installed on that computer**:

```bash
ros2 launch cefiro_hardware_interface gui_real.launch.py
```

The GUI's default simulation launch still uses `/imu/data`, `/cefiro_actuators/...` and `use_sim_time=true`. `gui_real.launch.py` changes only **its parameters** to `/real/...` and `use_sim_time=false`; simulation and real topics do not collide. The real GUI's motor command buttons will be disabled by the hardware adapter while in `legacy_text` mode (you can interact visually, but the stock MCU cannot accept those commands). The GUI itself may still show a locally requested "enabled" state: use `/real/cefiro/hardware/status` and **MCU acknowledgment** as the source of truth.

### Future bidirectional firmware

```bash
# Only after the STM32 CF1 parser, ACK, ESC interlocks and MCU watchdog exist:
ros2 launch cefiro_hardware_interface hardware.launch.py \
  port:=/dev/ttyUSB0 protocol_mode:=cf1 enable_command_tx:=true
```

Both the Raspberry Pi and the STM32 must enforce the watchdog; command TX is **off by default**. `CMD_ARM` must be ACKed by firmware before motor requests can be forwarded. `CMD_DIR` is disabled by default and blocked by host when armed or when any requested RPM is nonzero; the real ESC must separately support reversal. The `MotorRpm` request alone does **not** guarantee actual regulated RPM without ESC/firmware feedback control.

## Checks (no `test/` dependency in colcon)

```bash
python3 ~/cefiro_ws/src/cefiro_hardware_interface/tools/protocol_selfcheck.py
python3 -m compileall -q ~/cefiro_ws/src/cefiro_hardware_interface/cefiro_hardware_interface
```

See [docs/PROTOCOL.md](docs/PROTOCOL.md) for the proposed MCU packet types, CRC, ACK and the phased firmware implementation. No changes to the original user-provided `main.c` are included in this package.

## Optional isolated serial smoke test (NOT hardware)

For a workstation without an attached STM32, you can create a temporary **simulated** pseudo-serial source:

```bash
python3 ~/cefiro_ws/src/cefiro_hardware_interface/tools/virtual_legacy_fmu.py
# The program prints a /dev/pts/N path. In a different terminal:
ros2 launch cefiro_hardware_interface hardware.launch.py port:=/dev/pts/N
```

This is a deliberately fixed fixture for testing the ROS 2 plumbing only; it **must not** be used to calibrate the real IMU, control the physical drone, or validate motor safety.
