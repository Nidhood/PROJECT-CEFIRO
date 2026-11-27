#!/usr/bin/env python3
"""Static integration checks, not a substitute for Qt rendering tests."""
from pathlib import Path
import re
root=Path(__file__).resolve().parents[1]
main=(root/'src/main_window.cpp').read_text()
plot=(root/'src/scope_plot.cpp').read_text()
rec=(root/'src/recording_dialog.cpp').read_text()
view=(root/'src/attitude_view.cpp').read_text()
pad=(root/'src/attitude_command_pad.cpp').read_text()
assert 'effectiveUiScale(width(),height(),ui_scale_percent_)' in main
assert 'base_theme_qss_' in main and 'globalMatch(base_theme_qss_)' in main
assert 'std::abs(next-ui_scale_)<0.001' in main
assert 'setStyleSheet(scaled)' in main
assert 'DISPLAY / UI SCALE' in main and 'Ctrl+0' in main
assert 'bar->setMaximumHeight(48)' not in main
assert 'nav_->setMaximumHeight(32)' not in main
assert 'available_width=layoutWidth(available_width,ui_scale_)' in main
assert 'if(dashboard_columns_==cols)return;' not in main
assert 'scaledPixels(card_height,ui_scale_)' in main
assert 'setLabelsFont(labels)' in plot and 'setTitleFont(titles)' in plot
assert 'legend()->setFont(labels)' in plot
assert 'b->setFixedHeight(22)' not in plot
assert 'RecordingDialog::setTypographyScale' in rec and 'reflowTypography()' in rec
assert 'WrapLongRows' in rec and 'WrapLongRows' in main
assert 'outer->addWidget(emergency)' in main
assert 'QFont("Sans", 9' not in view and 'QFontMetrics' in view
assert 'setPixelSize(10)' not in pad and '2.0*center().x()' in pad
for f in (root/'resources/theme').glob('*.qss'):
    qss=f.read_text()
    for role,size in [('pageTitle',24),('metricValue',15),('panelTitle',14)]:
        assert re.search(r'QLabel#'+role+r'[^{}]*\{[^}]*font-size:'+str(size)+r'px',qss), (f,role)
print('PASS: adaptive typography integration, all themes, chart fonts, dialog reflow, accessible reset')
