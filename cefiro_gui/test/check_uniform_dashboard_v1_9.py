#!/usr/bin/env python3
"""Static regression test: twelve independent, equally sized dashboard panels."""
from pathlib import Path
main=(Path(__file__).resolve().parents[1]/'src/main_window.cpp').read_text()
a=main.index('void MainWindow::buildDashboard()')
b=main.index('void MainWindow::buildControl()',a)
layout=main[a:b]
assert 'for(int i=0;i<9;++i)' in layout
assert 'grid->addWidget(panel,row,column);' in layout
assert 'const int row=i<=6 ? i/2 : 4;' in layout
assert 'const int column=i<=6 ? i%2 : i-7;' in layout
assert 'grid->addWidget(altitude_panel,3,1);' in layout
assert 'grid->addWidget(panel,5,column);' in layout
assert 'RPM / REQUESTED REFERENCE' in layout
assert 'RPM / MEASURED FEEDBACK' in layout
assert 'titledPanel("RELATIVE ALTITUDE",altitude_inner,container)' in layout
assert 'altitude_plot_->setMinimumHeight(235);' in layout
assert 'altitude_panel->setMinimumHeight(308);' in layout
assert 'panel->setMinimumHeight(308);' in layout
assert 'setMinimumHeight(650)' not in layout
assert 'setMinimumHeight(205)' not in layout
assert 'altitude_inner->addWidget(altitude_plot_,1);' in layout
assert 'altitude_value_->setText(f(s.relative_alt_m,2)+" m")' in main
assert 'pressure_value_->setText(f(s.pressure_hPa,2)+" hPa")' in main
assert 'magnitude_value_->setText(f(field_magnitude,2)+" µT")' in main
print('Uniform 12-panel responsive dashboard checks passed')
