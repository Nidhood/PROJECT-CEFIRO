# CEFIRO GUI v1.3.0 — Verified Interface Deployment

**Important:** Screenshots displaying `REFERENCE COMMAND` and a standalone
`CONTROLLER PROFILES / EXPERIMENT CONFIGURATION` correspond to an older
executable, not this source. The v1.3.0 title bar and operations-page subtitle
include a version identifier so an outdated ROS 2 installation can be detected.

Expected layout:
- **ATTITUDE COMMAND & ANALYSIS** contains five tabs: Command, Preview,
  Topology, Gains, and Metrics. There is **no third row** for
  controller profiles.
- MANUAL reveals the draggable **3-axis reference gimbal** and numeric
  roll, pitch, yaw fields. AUTOMATIC reveals the waveform configuration.
- The IMU 3D view and live response receive a 64% horizontal stretch.
- Scope popups follow the mouse inside the chart viewport, while the sampled
  values are updated at most ~30 Hz to reduce CPU work.

To install, see `INSTALL_V1_3.md`. A GUI display running an older window
version is a sign of stale binaries or a different workspace being sourced.

---

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

**No dependency on Gazebo or `cefiro_description` is needed to build this HMI.** Graphs populate automatically when the configured ROS sensor topics exist; otherwise the display explicitly indicates that data is missing or stale. No flight dynamics are simulated in the GUI. The simplified drone is a **true 3D geometry rendered via orthographic Qt6 projection**, rotated by the *IMU's own orientation quaternion*. It includes a volumetric FMU, arm struts, motors and rotor discs. Camera rotation is separate from body attitude; **drag with the left mouse button** to orbit the view and **double-click to reset**. The view automatically fits its panel without any zoom controls. Body axes use **X red, Y green, Z blue** for every theme.

## Pages

| Page | Instruments |
| --- | --- |
| Dashboard | Roll, pitch, yaw (reference vs measured); body angular rate p/q/r; acceleration ax/ay/az; magnetic vector Bx/By/Bz |
| Signal Generator | Strategy selection; MANUAL / AUTOMATIC; nine waveforms, parameters and preview; control topology, live monitored axis and drone orientation; PID / LQR gain panels; test and recording controls |
| System | ROS sensor freshness; controller acknowledgment; frames and ROS topic contract; sensor values including pressure-derived relative height |

Each scope uses the same conventional presentation as the QUBE HMI: time and vertical zoom (`T−`, `T+`, `Y−`, `Y+`), visible-data `FIT`, **crosshair cursor, color-coded per-series markers and interpolated on-hover values**, live window, measured/reference traces and dashed reference styling. The Dashboard retains spacious chart panels (minimum 360 px of plot height, 430 px overall per panel), inside a scrollable two-column layout. Traces retained for approximately 70 seconds, with 5,000 points per series maximum. Plot redraw at 20 Hz, Qt UI refresh at ~30 Hz; ROS sensor callbacks run on a separate executor thread. The CSV recorder processes all buffered IMU samples, not only the samples rendered.

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

Click **`DATA RECORDER` to open the dedicated modeless acquisition window**. Set the output directory, file prefix, experiment notes, metadata option and one of four capture modes:

- **Manual / continuous:** records until you stop.
- **Time window:** waits for an optional start delay and stops after the specified duration (wall-clock seconds).
- **N waveform periods:** with AUTOMATIC reference mode, captures for a wall-clock duration of `period_count / reference_frequency` after the start delay. Frequency is frozen at capture start; this is not an assertion that an external controller is tracking the signal.
- **Threshold events:** detects local maxima (optionally on the absolute value) of measured roll, pitch or yaw, subject to a threshold and minimum separation; stops at the specified event count.

The minimum save interval is evaluated against **ROS IMU measurement timestamps**, preserving sample resolution across GUI refresh batches. Set it to 0 ms to save all received samples. The dialog shows recording state, saved rows, elapsed wall time, detected peaks and file path. If the metadata option is enabled, a companion `_metadata.yaml` and `_summary.txt` are created. The companion `_summary.txt` contains roll/pitch/yaw RMSE, MAE and peak absolute errors over **recorded samples**. CSV includes ROS timestamps, sequence number, all attitude angles, references, wrapped errors, angular rates, accelerometer, magnetometer, barometric measurements, validity flags, strategy, and manual/automatic source.

Note: the error metrics use the shortest 360° angular difference, including yaw wrap. They summarize tracking error, **not** closed-loop stability, step settling time, or a guarantee of controller performance. The `_summary.txt` also includes the latest qualifying step for each axis **if its command occurred during that recording**. A separate **Response** tab now analyzes a detected step of at least 1° on any of the three commanded axes. It reports amplitude, peak, peak time, overshoot percentage, 5%-threshold dead time, 10–90% rise time, ±2% settling time (minimum 0.5 s continuously inside tolerance), ringing crossings, late-window mean signed error, RMSE, MAE and maximum absolute error. Undefined or unsupported measurements show an em dash rather than a fabricated value. These values describe only the observed reference/IMU response; they do **not** confirm controller activation, closed-loop performance or actuator safety. A new step on a given axis replaces that axis's previous step analysis. For formal experiments, CSV export and independently validated offline analysis remain recommended.

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

## Version 1.1 HMI restoration notes

- Existing Qt6 themes, ScopePlot rendering style, reference publishing, SAFE STOP and EMERGENCY ZERO are preserved.
- `ScopePlot` now draws an in-chart cursor/markers and interpolates each series for accurate hover readouts (not simply the nearest stored sample).
- The interactive camera does **not** affect ROS orientation or send commands to actuators. Mouse wheel is consumed to avoid scaling the model.
- Recorder configuration is independent of controller state. `Auto record` uses the last settings selected in the Data Recorder dialog. CSV writing runs on the Qt loop, receiving all queued IMU samples; update rate and UI drawing do not determine the data sample rate.
- Timed capture uses **wall-clock elapsed time**, whereas optional CSV downsampling uses **IMU header stamps**. They differ if Gazebo's real-time factor is not 1.0.

### Validation boundaries

The project package, Python launch syntax and platform-independent C++ unit tests were checked while preparing this archive. The local build environment lacks ROS 2 Jazzy/Qt6 development headers, so compilation and runtime integration of the Qt6 application must be confirmed on your Ubuntu system.

## Version 1.2: compact Attitude Operations layout

- **Signal Generator has no whole-page scroll container.** Two independently sized columns reserve about 38% of the available width for control and **62% for live attitude instrumentation**, including the 3D drone and response scope. The minimum window height has been relaxed to 700 px; maximize the application for full presentation. The Dashboard still scrolls because six large time-series charts cannot reasonably fit into a single viewport.
- The main heading is **ATTITUDE CONTROL & EXPERIMENTS**. The **ATTITUDE COMMAND & ANALYSIS** panel contains compact tabs: **Setup, Preview, Topology, Gains and Response**; the gains table no longer occupies a third row.
- **MANUAL** shows a colored, concentric 3-axis command instrument with red roll, green pitch and blue yaw. Click and drag a ring for the respective reference, use the mouse wheel for 1° changes, or double-click to zero its angle. Numerical entry stays available for exact setpoints, including pitch's ±89° limits. **AUTOMATIC** shows waveform parameters instead; these two modes never compete for vertical space. Neither source transmits commands until the output is enabled and the operator activates the command request.
- Each oscilloscope now uses a styled **tooltip label anchored to its viewport** instead of the platform-native tooltip. Its position follows every mouse movement, while data interpolation and chart markers are throttled to 25 Hz. The label stays inside the panel, flipping sides near edges.
- Six themes, 3D camera orbit, recorder subwindow and ROS topics are retained.

### Tested here

The ROS package structure, Python launch, existing waveform/quaternion tests, and the new step-response tests were checked with Python/GCC. Full `colcon` compilation with Qt6 and ROS 2 Jazzy is not available in this build environment and still requires your Ubuntu installation.


## v1.4 — Motor manual control and recorder

- AUTO shows the interactive attitude gimbal and waveform generator.
- MANUAL uses four vertical RPM sliders (0–12000 RPM), with a four-channel command published at 20 Hz to `/cefiro_actuators/target_rpm`. Arm/disarm on `/cefiro_actuators/armed`; emergency stop publishes zeros and disarms.
- Direction selection publishes requests on `/cefiro_actuators/motor_direction` (`std_msgs/Int8MultiArray`: +1 / -1). **The current actuator node does not subscribe to this direction topic and does not implement reverse rotation.** Direction control only takes effect after support is implemented in `cefiro_actuators`, including signed speed and torque handling.
- Hover cursor uses full viewport repaint to avoid persistent guide lines.
- Recorder offers selectable data groups, time readout units, and a progress bar for time-window, waveform-period, and peak-count captures. Continuous captures remain indeterminate.
- The build requires the existing `cefiro_msgs` package.


## Build only the GUI (v1.4.1)

This GUI depends on **cefiro_msgs**, not on **cefiro_actuators**. The `test/`
directory is not required and is not included in this release.

```bash
cd ~/cefiro_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select cefiro_msgs
source install/setup.bash
colcon build --symlink-install --packages-select cefiro_gui --cmake-clean-cache
source install/setup.bash
ros2 launch cefiro_gui cefiro_gui.launch.py
```

If building every package with `colcon build`, an obsolete CMake cache of
`cefiro_actuators` may still try to compile `test/test_actuator_model.cpp`.
This is **not a GUI compile error**. Clean `build/cefiro_actuators` and
`install/cefiro_actuators`, and install a `cefiro_actuators/CMakeLists.txt`
without that test target. Alternatively, use `-DBUILD_TESTING=OFF` for builds
that still support that flag (after cleaning the stale build cache).

Motor direction requests are published separately, but effective reverse
rotation requires an actuator subscriber which implements the direction topic.
