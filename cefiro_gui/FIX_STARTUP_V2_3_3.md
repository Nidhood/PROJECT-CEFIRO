# CEFIRO GUI v2.3.3 - SIGSEGV/locale fix

- Root cause: Qt `findChildren<Subclass*>()` uses the Qt metaobject. The custom
  AresPanel, AresPage and ScopePlot classes do **not** declare `Q_OBJECT`.
  Their inherited metaobjects can cause invalid casts (e.g. a QLabel treated
  as AresPage), corrupting memory at initial theme application / resize.
- Fix: enumerate QWidget children and use C++ `dynamic_cast` to check runtime
  type before invoking custom widget methods. No Qt MOC changes required.
- Fix: parse string yaw overrides using locale-independent `std::from_chars`.
  Previously `std::stod("151.318")` could consume `151` under a locale expecting
  comma decimals, incorrectly treating `.318` as a suffix.
- Keep default lightweight Tron Ares appearance; optional overlay effects stay
  disabled unless `CEFIRO_GUI_ENABLE_ARES_FX` is explicitly set.
- Existing safety and motor publishing logic unchanged.

Build:

```bash
cd ~/cefiro_ws
rm -rf build/cefiro_gui install/cefiro_gui
colcon build --packages-select cefiro_gui --symlink-install
source install/setup.bash
CEFIRO_GUI_TRACE_STARTUP=1 ros2 launch cefiro_gui cefiro_gui.launch.py
```

Expected: '[CEFIRO startup] Theme widget types verified', then 'Theme charts
and attitude updated', 'Theme applied', and 'MainWindow constructor completed'.

**Environment note**: Source checks can be run here; the ROS 2/Qt6 runtime
must be verified on the target Ubuntu machine. If SIGSEGV persists, obtain
`gdb` backtrace to identify another fault, rather than assuming this fix is
sufficient. No files were pushed to the repository.
