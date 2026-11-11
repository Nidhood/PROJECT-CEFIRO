# CEFIRO GUI v2.3.2 -- diagnostic and safe paint mode

The initial GUI configuration uses `equilibrium_world_yaw_deg: 151.318` (unquoted float). If a ROS launch override provides a string, optional `deg`/`rad` suffixes are parsed, and unsupported trailing characters are logged with an explicit explanation. This warning is not evidence that the parameter caused the SIGSEGV.

## Start in conservative mode

The Tron Ares QSS remains active. Custom QPainter glow and QGraphicsDropShadowEffect decorations are opt-in because certain graphics driver and Qt combinations fail during GUI composition.

```bash
CEFIRO_GUI_TRACE_STARTUP=1 ros2 launch cefiro_gui cefiro_gui.launch.py
```

To separately rule out the runtime relayout engine:

```bash
CEFIRO_GUI_TRACE_STARTUP=1 CEFIRO_GUI_DISABLE_REFLOW=1 ros2 launch cefiro_gui cefiro_gui.launch.py
```

Optional advanced glow (only after GUI starts reliably): `CEFIRO_GUI_ENABLE_ARES_FX=1`.

If the process still exits with -11, get an actual backtrace using gdb:

```bash
sudo apt install gdb
gdb -q --args ~/cefiro_ws/install/cefiro_gui/lib/cefiro_gui/cefiro_gui_node --ros-args -r __node:=cefiro_hmi --params-file ~/cefiro_ws/install/cefiro_gui/share/cefiro_gui/config/gui_config.yaml
```

At the `(gdb)` prompt: `run`, then `thread apply all bt 20`. Ensure `source ~/cefiro_ws/install/setup.bash` in the parent shell before invoking gdb. Running outside ros2 launch skips the temporary extra ROS params file; for exact launch behavior use a launch prefix with gdb or copy the launch command from ROS logs.

The package cannot be runtime-compiled here because ROS 2 Jazzy and Qt6Charts are not installed in this environment. Static checks do not validate crash fixes. **Never test armed motors while diagnosing GUI faults.**
