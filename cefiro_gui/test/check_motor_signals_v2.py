#!/usr/bin/env python3
"""Motor plot contract. Static tests do not claim ROS / ESC validation."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
bridge=(root/'src/ros_bridge.cpp').read_text()
header=(root/'include/cefiro_gui/ros_bridge.hpp').read_text()
win=(root/'src/main_window.cpp').read_text()
win_header=(root/'include/cefiro_gui/main_window.hpp').read_text()
config=(root/'config/gui_config.yaml').read_text()
for topic in (
    '/cefiro_actuators/target_rpm', '/real/cefiro_actuators/target_rpm',
    '/cefiro_actuators/telemetry', '/real/cefiro_actuators/telemetry',
    '/cefiro_actuators/measured_rpm', '/real/cefiro_actuators/measured_rpm',
    '/cefiro_actuators/dshot_throttle', '/real/cefiro_actuators/dshot_throttle',
    '/real/cefiro/hardware/commanded_dshot',
):
    assert topic in config, topic
for method in ('onMotorRpm', 'onMotorDshot', 'onMotorTelemetry', 'drainMotor', 'motorFreshFor'):
    assert method in bridge and method in header, method
for code in ('event.command_source==1', 'event.command_source==2',
             'event.dshot', 'event.measured_rpm', 'event.requested_rpm',
             'last_motor_telemetry_s_', 'expireMotorGroups(motor_time)',
             'const auto motor_events=bridge_.drainMotor()',
             'void MainWindow::clearMotorGroup(bool real,int group)',
             'if(rpm_target_plot_)rpm_target_plot_->clear()',
             'if(rpm_feedback_plot_)rpm_feedback_plot_->clear()'):
    assert code in win, code
assert 'ScopePlot* rpm_target_plot_' in win_header
assert 'ScopePlot* rpm_feedback_plot_' in win_header
assert 'add_rpm_panel("RPM / REQUESTED REFERENCE"' in win
assert 'add_rpm_panel("RPM / MEASURED FEEDBACK"' in win
assert 'grid->addWidget(panel,5,column)' in win
assert 'last_motor_group_s_' in win_header
assert 'onMotorEvent(fallback,motor_time)' in win
assert 'std::array<std::array<std::size_t,5>,2> latest_motor' in win
assert 'Sending a command alone does not mean the plant is connected.' in bridge
assert 'msg.command_source' in bridge
assert '#include <cefiro_msgs/msg/motor_telemetry.hpp>' in header
assert '#include <cefiro_msgs/msg/motor_dshot.hpp>' in header
assert 'real?' in bridge
assert 'src/ros_bridge.cpp' in (root/'CMakeLists.txt').read_text()
print('Motor signal routing, semantics, and 12-panel layout checks passed')
