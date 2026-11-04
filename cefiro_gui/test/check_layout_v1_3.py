"""Static integration checks for CEFIRO Qt6 GUI v1.4.0.

Run with python3 test/check_layout_v1_3.py from the package root.
This does not replace a native ROS 2/Qt6 build and a manual UI smoke test.
"""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
main = (root / 'src/main_window.cpp').read_text()
plot = (root / 'src/scope_plot.cpp').read_text()
pad = (root / 'src/attitude_command_pad.cpp').read_text()
body = main[main.index('void MainWindow::buildControl()'): main.index('void MainWindow::buildSystem()')]
assert 'CEFIRO GUI v1.4.0' in main
assert 'ATTITUDE COMMAND & ANALYSIS' in body
assert 'CONTROLLER PROFILES' not in body
assert 'REFERENCE COMMAND' not in body
assert 'ref_tabs_->addTab(gains_tab,"Gains")' in body
assert 'ref_tabs_->addTab(metrics_tab,"Metrics")' in body
assert 'ref_tabs_->addTab(reference_tab, "Command")' in body
assert 'columns->addWidget(right_widget, 64)' in body
assert 'new AttitudeCommandPad(auto_page)' in body
assert 'motor_sliders_' in body
assert 'publishMotorCommand' in main
assert 'new QScrollArea(page)' not in body
assert 'if (ref_tabs_) ref_tabs_->setCurrentIndex(0)' in main
assert 'command_source_stack_->setCurrentIndex(yes ? 1 : 0)' in main
assert 'positionHoverLabel(hover_position_)' in plot
assert 'hover_label_->move(left, top)' in plot
assert 'hover_label_->setAttribute(Qt::WA_TransparentForMouseEvents)' in plot
assert 'updateCursor(hover_position_)' in plot
for method in ('mousePressEvent', 'mouseMoveEvent', 'mouseDoubleClickEvent', 'wheelEvent'):
    assert method in pad
assert 'VERSION 1.4.0' in (root / 'CMakeLists.txt').read_text()
assert '<version>1.4.0</version>' in (root / 'package.xml').read_text()
print('CEFIRO GUI 1.4 LAYOUT, HOVER AND GIMBAL CONTRACT CHECKS PASSED')
