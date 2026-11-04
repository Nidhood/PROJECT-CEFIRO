# CEFIRO GUI v1.3.0 — installation and binary verification

## Replace the source package

Stop any running `cefiro_gui` process. Replace the entire `cefiro_gui`
folder in `~/cefiro_ws/src` with this ZIP's `cefiro_gui` directory.
Do not copy only a subset of its source files.

## Compile the actual workspace

```bash
cd ~/cefiro_ws
source /opt/ros/jazzy/setup.bash
# Remove stale output of only this package. Do not delete the other packages.
rm -rf build/cefiro_gui install/cefiro_gui
find src/cefiro_gui -type f -exec touch {} +
colcon build --symlink-install --packages-select cefiro_gui --cmake-clean-cache
source install/setup.bash
```

For merged installs, `install/cefiro_gui` is not a separate directory; use
`colcon build --cmake-clean-cache` and check the resolved binary below.

## Verify the executable ROS 2 is actually launching

```bash
ros2 pkg prefix cefiro_gui
BIN="$(ros2 pkg prefix cefiro_gui)/lib/cefiro_gui/cefiro_gui_node"
ls -l "$BIN"
strings "$BIN" | grep -F 'CEFIRO GUI v1.3.0'
ros2 launch cefiro_gui cefiro_gui.launch.py
```

**Expected:** the window's title includes `CEFIRO GUI v1.3.0` and the
operations header includes `CEFIRO GUI v1.3.0 · ATTITUDE COMMAND & ANALYSIS`.
If the marker is absent, stop and check `which ros2`, `echo "$AMENT_PREFIX_PATH"`,
and whether another workspace has been sourced *after* `~/cefiro_ws/install/setup.bash`.

## Visual acceptance checks

1. Select Signal Generator: no standalone `CONTROLLER PROFILES` row.
2. Click Gains: it appears INSIDE Attitude Command & Analysis.
3. Click MANUAL: the colored-ring gimbal replaces the waveform fields;
   drag the red/green/blue rings and observe angle spin boxes change.
4. Click AUTOMATIC: the waveform parameters replace the manual instrument.
5. Hover over a chart curve and move the cursor: the label follows the
   cursor; a vertical cursor and interpolated sample markers are shown.

The GUI publishes requests only. An external attitude controller must
implement the ROS topics before commands cause physical or simulated motion.
