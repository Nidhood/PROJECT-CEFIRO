# CEFIRO GUI v2.3 — Adaptive layout / Tron Ares console

This is the complete `cefiro_gui` package based on the previously delivered v2.2 GUI. No changes to `cefiro_control` or `cefiro_gz` are required for the styling/layout update.

## Responsive behaviour

- Dashboard uses one column below 920 px of available viewport, two columns from 920 px and three columns from 2100 px. Panel order is kept consistent, and scopes resize with their containers.
- Signal Generator becomes a scrollable work area; the architecture/reference stack and the attitude/response stack switch from two columns to one below 1050 px. Manual rotor columns adapt from 4 to 2 to 1; master command fields and waveform fields also reflow. ROS publishers, telemetry subscriptions, references and persistent chart history are unaffected.
- System screen stacks its two panels under 850 px. The top toolbar shortens tab labels and hides secondary text when the window narrows; SIM and REAL connection indicators remain visible.
- Main window minimum size is now 610 x 480 logical pixels, rather than 1180 x 700. Data recorder dialog minimum size reduced to 500 x 480.
- Panel styling respects the currently selected theme; custom neon framing is enabled only on `Tron Ares`.

## Theme changes

- Ares red / charcoal palette, subtle perspective grid, neon-lit corner rails, stronger terminal-style labels, contrast-safe controls and vibrant motor telemetry plots.
- No CSS `box-shadow`: Qt style sheets do not implement that property. Instead native QPainter draws lightweight layered neon edges, and static titles use a soft QGraphicsDropShadowEffect.
- Other installed themes continue to work without the Ares panel overlays.

## Build (ROS 2 Jazzy / Qt6)

```bash
cd ~/cefiro_ws
colcon build --packages-select cefiro_gui --symlink-install
source install/setup.bash
ros2 launch cefiro_gui cefiro_gui.launch.py
```

Build/run cannot be verified in this environment because ROS 2 Jazzy and Qt6 C++ development libraries are not installed. Static source checks and package checks were run.

## Safety and existing behavior

This update modifies layout and theme only. It does **not** change real/sim command routing, flight control limits, DShot authorization, output gating, or emergency-stop logic. Always bench-test with propellers removed before commissioning real hardware.

## v2.3.1: Qt resize crash hardening

The v2.3 UI called `QGridLayout::removeWidget` and `addWidget` synchronously
from viewport `Resize` event filters. Qt can deliver recursive resize notifications
while it is negotiating geometry, causing re-entrant layout modification.

The GUI now uses a 35 ms single-shot timer to coalesce resize notifications,
only applies reflows after all UI pages are initialized, and applies hysteresis
around column-switch thresholds so scrollbars cannot cause oscillating layouts.
The source/control telemetry behavior is unchanged.

The original crash log only reports exit code -11 (SIGSEGV), not a backtrace.
This addresses the most likely source, but must be validated by launching
under ROS 2 Jazzy/Qt 6 on the target machine. If SIGSEGV persists:

```bash
cd ~/cefiro_ws
colcon build --packages-select cefiro_gui --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
gdb --args install/cefiro_gui/lib/cefiro_gui/cefiro_gui_node \
  --ros-args -r __node:=cefiro_hmi \
  --params-file install/cefiro_gui/share/cefiro_gui/config/gui_config.yaml
# In GDB: run
# After crash: thread apply all bt 20
```

To disable adaptive rearrangement temporarily for diagnosis while keeping
Tron Ares and the normal control interfaces:

```bash
CEFIRO_GUI_DISABLE_REFLOW=1 ros2 launch cefiro_gui cefiro_gui.launch.py
```

Use the normal launch command again to restore responsive behavior.
