#!/usr/bin/env python3
"""Static regression checks for independent per-ScopePlot AUTO Y toggles.

ROS 2/Qt compilation and interactive runtime tests require an installed GUI environment.
"""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
header = (root/'include/cefiro_gui/scope_plot.hpp').read_text()
scope = (root/'src/scope_plot.cpp').read_text()
window = (root/'src/main_window.cpp').read_text()
assert 'QToolButton* auto_fit_button_{nullptr};' in header
assert 'auto_fit_button_=make("AUTO Y: OFF"' in scope
assert 'auto_fit_button_->setCheckable(true)' in scope
assert 'QToolButton::toggled' in scope
assert 'setAutoFitY(checked,minimum_y_span_)' in scope
assert 'const QSignalBlocker block(auto_fit_button_)' in scope
assert 'auto_fit_button_->setText(enabled?"AUTO Y: ON":"AUTO Y: OFF")' in scope
assert 'setAutoFitY(false,minimum_y_span_)' in scope
assert 'connect(fit,&QToolButton::clicked,this,[this]{fitY();});' in scope
assert 'if(auto_y_)fitY();' in scope
assert 'live_->setAutoFitY(true,4.0);' in window
assert window.count('live_->setAutoFitY(true,4.0);') == 1
assert 'preview_->setYRange(-190,190);' in window
assert 'preview_->setWindowSeconds(std::clamp(total,1.0,60.0));' in window
for qss in (root/'resources/theme').glob('*.qss'):
    theme=qss.read_text()
    assert 'QToolButton#chartAutoFitButton:checked' in theme, qss
print('Independent AUTO Y toggle contract passed')
