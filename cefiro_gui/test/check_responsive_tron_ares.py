#!/usr/bin/env python3
"""Static package/contract checks for adaptive Ares UI (no Qt runtime required)."""
from pathlib import Path
import re
root=Path(__file__).resolve().parents[1]
main=(root/'src/main_window.cpp').read_text()
header=(root/'include/cefiro_gui/main_window.hpp').read_text()
chrome_h=(root/'include/cefiro_gui/ares_chrome.hpp').read_text()
chrome_cpp=(root/'src/ares_chrome.cpp').read_text()
qss=(root/'resources/theme/tron_ares.qss').read_text()
cmake=(root/'CMakeLists.txt').read_text()
ctrl=(root/'src/attitude_command_pad.cpp').read_text()
assert 'src/ares_chrome.cpp' in cmake
assert 'AresPanel::paintEvent' in chrome_cpp
assert 'AresPage::paintEvent' in chrome_cpp
assert 'setAresEnabled(index==0)' in main
assert 'setAresTheme(index==0)' in main
assert 'setAresTheme(bool enabled)' in ctrl
assert 'QMainWindow::resizeEvent(event)' in main
assert 'reflowDashboard(dashboard_scroll_->viewport()->width())' in main
assert 'reflowControl(control_scroll_->viewport()->width())' in main
assert 'reflowSystem(system_scroll_->viewport()->width())' in main
assert 'dashboard_columns_==3' in main and 'available_width>=2160?3' in main
assert 'control_columns_mode_==2' in main and 'available_width<1020?1:2' in main
assert 'system_columns_mode_==2' in main and 'available_width<820?1:2' in main
assert 'reflowMotorControls(w)' in main
assert 'reflowMasterControl(w)' in main
assert 'reflowTrajectoryFields(w)' in main
assert 'motor_columns_mode_==4' in main and 'available_width>980?4' in main
assert 'masterMotorSlider' in (root/'resources/theme/tron_ares.qss').read_text()
assert 'status_sim_connection_' in main and 'status_real_connection_' in main
assert 'setMinimumSize(610,480)' in main
assert 'setMinimumSize(500, 480)' in (root/'src/recording_dialog.cpp').read_text()
assert 'std::array<ScopePlot*,9> dash_' in header
assert 'rpm_target_plot_' in main and 'rpm_feedback_plot_' in main
assert 'qlineargradient' in qss
assert 'chartAutoFitButton' in qss
assert 'QFrame#panel' in qss
assert '#ff304c' in main
assert re.search(r'class AresPanel final',chrome_h)
assert re.search(r'class AresPage final',chrome_h)
assert 'responsive_timer_->setSingleShot(true)' in main
assert 'CEFIRO_GUI_DISABLE_REFLOW' in main
assert 'scheduleResponsiveLayout();' in main
assert 'reflow_in_progress_' in main
assert 'void MainWindow::applyResponsiveLayout()' in main
assert 'ui_ready_=true;' in main
# Resize callbacks must only enqueue UI reflow; never manipulate a layout inline.
resize_section=main[main.index('bool MainWindow::eventFilter('):main.index('void MainWindow::reflowDashboard(')]
assert 'scheduleResponsiveLayout()' in resize_section
assert 'reflowDashboard(' in resize_section  # calls happen only in applyResponsiveLayout
assert resize_section.index('void MainWindow::applyResponsiveLayout()') > resize_section.index('void MainWindow::resizeEvent(')
print('RESPONSIVE TRON ARES UI STATIC CHECKS PASSED')
