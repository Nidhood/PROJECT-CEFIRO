#!/usr/bin/env python3
"""Static UI contract checks for CEFIRO GUI v1.8 (no ROS/Qt needed)."""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
main = (root/'src/main_window.cpp').read_text()
bridge = (root/'src/ros_bridge.cpp').read_text()
plot = (root/'src/scope_plot.cpp').read_text()
config = (root/'config/gui_config.yaml').read_text()
body = main[main.index('void MainWindow::buildControl()'):main.index('void MainWindow::buildSystem()')]
assert 'CEFIRO GUI v2.3' in main
assert 'ATTITUDE COMMAND & ANALYSIS' in body
assert 'new AttitudeCommandPad(gimbal_tab)' in body
assert 'publishMotorCommand' in main
assert 'motor_sliders_' in body
assert 'positionHoverLabel(hover_position_)' in plot
assert 'hover_label_->setAttribute(Qt::WA_TransparentForMouseEvents)' in plot
# Every source and sensor must have its own ROS callback and telemetry lifecycle.
for code in ('onImu(*m,false)', 'onImu(*m,true)',
             'onMag(*m,false)', 'onMag(*m,true)',
             'onBaro(*m,false)', 'onBaro(*m,true)',
             'SampleKind::Magnetometer', 'SampleKind::Barometer',
             'real_status_rx_'):
    assert code in bridge, code
for code in ('statusBar()->addWidget(status_sim_connection_)',
             'statusBar()->addWidget(status_real_connection_)',
             'void MainWindow::clearSourceHistory(bool real)',
             'void MainWindow::clearSensorHistory(bool real,SampleKind sensor)',
             'last_event[s.from_real ? 1 : 0]',
             'plant_selector_->setEnabled(false)',
             'plant_selector_->setEnabled(true)',
             'new ScopePlot("","m",QStringList{"SIM altitude","REAL altitude"}',
             'void MainWindow::setMonitorAxis(int idx)'):
    assert code in main, code
assert 'void ScopePlot::clearSeries(int index)' in plot
assert 'live_->clear();live_->setYRange' not in main
for param in ('real_imu_topic', 'real_magnetometer_topic',
              'real_barometer_topic', 'real_status_topic'):
    assert param in config
assert 'VERSION 2.3.3' in (root/'CMakeLists.txt').read_text()
assert '<version>2.3.3</version>' in (root/'package.xml').read_text()
print('CEFIRO v1.8 DUAL SOURCE / UI STATIC CHECKS PASSED')
