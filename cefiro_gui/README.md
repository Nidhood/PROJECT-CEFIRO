# CEFIRO · Qt6 Attitude HMI (ROS 2 Jazzy / Gazebo Harmonic)

Native **C++20 / Qt6 Widgets / Qt6 Charts / rclcpp** desktop HMI inspired by the open-source QUBE-Servo 2 GUI workflow: dashboard, reference & control deck, system status, six selectable themes, oscilloscope controls, waveform preview, CSV recorder, live statistics and experiment configuration. The instrument design and plot operations follow the reference; all CEFIRO source code is written for the drone's three attitude axes.

## Install · Ubuntu 24.04

```bash
sudo apt update
sudo apt install qt6-base-dev libqt6charts6-dev ros-jazzy-rclcpp \
  ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-std-msgs

# Extract cefiro_gui in ~/cefiro_ws/src/ (the folder must contain package.xml)
cd ~/cefiro_ws
colcon build --symlink-install --packages-select cefiro_gui
source install/setup.bash
ros2 launch cefiro_gui cefiro_gui.launch.py
```

Run the Gazebo launch separately in another sourced terminal:

```bash
ros2 launch cefiro_gz start_gz.launch.py
```

**No dependency on Gazebo or `cefiro_description` is needed to build this HMI.** Graphs populate automatically when the configured ROS sensor topics exist; otherwise the display explicitly indicates that data is missing or stale. No flight dynamics are simulated in the GUI. The simplified drone is a lightweight axonometric 3D rendering rotated by the *IMU's own orientation quaternion*.

## Pages

| Page | Instruments |
| --- | --- |
| Dashboard | Roll, pitch, yaw (reference vs measured); body angular rate p/q/r; acceleration ax/ay/az; magnetic vector Bx/By/Bz |
| Signal Generator | Strategy selection; MANUAL / AUTOMATIC; nine waveforms, parameters and preview; control topology, live monitored axis and drone orientation; PID / LQR gain panels; test and recording controls |
| System | ROS sensor freshness; controller acknowledgment; frames and ROS topic contract; sensor values including pressure-derived relative height |

Each scope uses the same conventional presentation as the QUBE HMI: time and vertical zoom (`T−`, `T+`, `Y−`, `Y+`), visible-data `FIT`, cursor tooltips, live window, measured/reference traces and dashed reference styling. Traces retained for approximately 70 seconds, with 5,000 points per series maximum. Plot redraw at 20 Hz, Qt UI refresh at ~30 Hz; ROS sensor callbacks run on a separate executor thread. The CSV recorder processes all buffered IMU samples, not only the samples rendered.

Six themes: **Tron Ares** (default), MATLAB / Lab, Light, Dark, Tron Evolution and Cyberpunk Neon.

## CEFIRO sensor topics (existing `cefiro_gz` bridge)

| Direction | Topic | Type |
| --- | --- | --- |
| Subscribe | `/imu/data` | `sensor_msgs/msg/Imu` |
| Subscribe | `/magnetometer/data` | `sensor_msgs/msg/MagneticField` |
| Subscribe | `/bmp581/air_pressure` | `sensor_msgs/msg/FluidPressure` |

ROS topics can be changed using `config/gui_config.yaml`. ROS 2 `use_sim_time` is enabled by default. The Yaw/Roll/Pitch display is obtained from the IMU quaternion. Gyroscope signals use angular velocity directly (not differentiated Euler angles). Magnetometer measurements are displayed in microtesla; **no magnetometer-based orientation fusion is implemented**. Relative altitude is computed from the first valid pressure sample; it is *not* absolute altitude.

## Controller interface: important integration boundary

`cefiro_control` currently has no controller implementation. This HMI is fully capable of monitoring, generating signals, publishing control requests and recording experiments, but **publishing a command is not the same as actuating CEFIRO**. No ROS 2 controller feedback means the UI cannot truthfully claim a PID/LQR is running. The GUI warns the operator before enabling the output request without controller feedback.

| Direction | Topic | ROS message | Contract |
| --- | --- | --- | --- |
| Publish | `/cefiro/control/attitude_reference` | `geometry_msgs/msg/Vector3Stamped` | `vector.x=roll`, `vector.y=pitch`, `vector.z=yaw`, **radians**, `frame_id=drone_body_link` |
| Publish | `/cefiro/control/enable` | `std_msgs/msg/Bool` | `true` requests controller enable; false inhibits it |
| Publish | `/cefiro/control/emergency_stop` | `std_msgs/msg/Bool` | true requests emergency stop, false on new enable |
| Publish | `/cefiro/control/mode` | `std_msgs/msg/String` | `ATTITUDE` |
| Publish | `/cefiro/control/strategy` | `std_msgs/msg/String` | `OPEN LOOP`, `PID`, `LQR`, `CUSTOM` |
| Publish | `/cefiro/control/pid_gains` | `std_msgs/msg/Float64MultiArray` | nine values, **3×3 row-major**, rows roll/pitch/yaw, columns Kp/Ki/Kd |
| Publish | `/cefiro/control/lqr_gain` | `std_msgs/msg/Float64MultiArray` | eighteen values, **3×6 row-major**, rows torque axes, columns φ θ ψ p q r |
| Subscribe | `/cefiro/control/status` | `std_msgs/msg/String` | Text feedback from external controller (e.g. `ACTIVE:PID` / `INACTIVE` / `FAULT`) |

**Safety:** `SAFE STOP` disables output and requests a zero reference; `EMERGENCY ZERO` additionally publishes the emergency stop signal. These are *software requests* and are **not guaranteed to stop any external actuators** unless the controller is implemented to honor them. The external controller must own watchdogs, saturation limits, arming logic, emergency stop and actuator safety independently of the GUI. No actuation is implemented here; the GUI never sends motor commands directly.

## Recording and analysis

Click `DATA RECORDER` and choose a CSV path. A companion `_summary.txt` contains roll/pitch/yaw RMSE, MAE and peak absolute errors over **recorded samples**. CSV includes ROS timestamps, sequence number, all attitude angles, references, wrapped errors, angular rates, accelerometer, magnetometer, barometric measurements, validity flags, strategy, and manual/automatic source.

Note: the error metrics use the shortest 360° angular difference, including yaw wrap. They summarize tracking error, **not** closed-loop stability, step settling time, or a guarantee of controller performance. For step response analysis, additional offline processing of the CSV is recommended.

## Quick verification

```bash
ros2 topic hz /imu/data
ros2 topic echo /cefiro/control/attitude_reference --once
ros2 topic echo /cefiro/control/status --once
```

With Gazebo paused, the HMI remains responsive. Press Play to receive IMU data.

## Testing status

Pure C++ math tests under `test/test_models.cpp` cover quaternion Euler conversion, yaw wrap, waveform generation, and RMSE; these can be run without ROS or Qt:

```bash
g++ -std=c++20 -Iinclude test/test_models.cpp -o /tmp/test_cefiro_gui && /tmp/test_cefiro_gui
```

Full Qt6/ROS 2 compilation and end-to-end Gazebo integration must be verified on Ubuntu 24.04 with ROS 2 Jazzy; they cannot be validated by math-only tests.
