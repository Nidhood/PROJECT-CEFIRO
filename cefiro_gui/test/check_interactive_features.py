#!/usr/bin/env python3
"""Static package checks, runnable without Qt or ROS."""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
plot = (root/'src/scope_plot.cpp').read_text()
attitude = (root/'src/attitude_view.cpp').read_text()
recorder = (root/'src/recording_dialog.cpp').read_text()
mainwindow = (root/'src/main_window.cpp').read_text()
cmake = (root/'CMakeLists.txt').read_text()
assert 'updateCursor' in plot and 'cursor_line_' in plot
assert 'lower_bound' in plot and 'hover_timer_' in plot
assert 'mouseMoveEvent' in attitude and 'azimuth_' in attitude
assert 'wheelEvent' in attitude and 'event->accept()' in attitude
for color in ('#ff5252', '#4ee58c', '#448cff'):
    assert color in attitude
for text in ('Manual / continuous', 'Time window', 'N waveform periods', 'Threshold events'):
    assert text in recorder
assert 'new RecordingDialog(this)' in mainwindow
assert 'sample.ros_time_s' in mainwindow
assert 'recording_should_stop_' in mainwindow
assert 'src/recording_dialog.cpp' in cmake
print('CEFIRO INTERACTIVE GUI STATIC CHECKS PASSED')
