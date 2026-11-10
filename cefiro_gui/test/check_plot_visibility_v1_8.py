#!/usr/bin/env python3
"""Checks the dynamic-series display contract without ROS or Qt dependencies."""
from pathlib import Path

root = Path(__file__).resolve().parents[1]
scope = (root / 'src/scope_plot.cpp').read_text()
header = (root / 'include/cefiro_gui/scope_plot.hpp').read_text()
window = (root / 'src/main_window.cpp').read_text()
for needle in (
    'void ScopePlot::syncSeriesVisibility()',
    'const bool has_samples=!series.points.empty()',
    'series.line->setVisible(has_samples)',
    'chart_->legend()->markers(series.line)',
    'marker->setVisible(has_samples)',
    'chart_->legend()->setVisible(visible_count>0)',
    'if(!series.line->isVisible()) continue;',
    'connect(fit,&QToolButton::clicked,this,[this]{fitY();});',
    'const double span=std::max(high-low,minimum_y_span_)',
    'if(!data_[i].line->isVisible())',
):
    assert needle in scope, needle
assert 'void setAutoFitY(bool enabled=true,double minimumSpan=2.0);' in header
for needle in (
    'dash_[size_t(i)]->setAutoFitY(true,minimum_span[i])',
    'altitude_plot_->setAutoFitY(true,2.0)',
    'live_->setAutoFitY(true,4.0)',
    'void MainWindow::clearSensorHistory(bool real,SampleKind sensor)',
    'void MainWindow::updateSourceConnections()',
    'source_online_[size_t(index)]=connected',
    'statusBar()->addWidget(status_sim_connection_)',
    'statusBar()->addWidget(status_real_connection_)',
):
    assert needle in window, needle
print('CEFIRO v1.8 VISIBLE SERIES / AUTO FIT STATIC CHECKS PASSED')
