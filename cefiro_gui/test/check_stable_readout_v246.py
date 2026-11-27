#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=(root/'src/main_window.cpp').read_text()
h=(root/'include/cefiro_gui/stable_readout.hpp').read_text()
assert 'setWordWrap(false)' in h and 'setFixedHeight(height)' in h and 'setFixedWidth(width+' in h
assert 'QSizePolicy::Ignored' in h and 'if(readout->text()!=text)' in h
assert 'updateMotorLabel' not in s and 'updateMasterLabel' not in s
assert 'motor_labels_[i],QString("Req %1 RPM\\nDShot %2\\nEq %3 RPM")' in s
assert 'if(manual_mode_)refreshMotorReadouts();' in s
assert 'reserveReadout(readout,3,true' in s
assert 'reserveReadout(master_motor_label_,3);' in s
assert 'reserveReadout(command_value_,2);' in s
assert 'const QString code=host?' in s
assert 'map.inverse(host->codes)' in s
assert 'Qt::AlignHCenter' in s
assert 'setReadoutText(master_motor_label_,master,reason)' in s
print('PASS: stable readout geometry, single writer, explicit rows, source validity and typography reapplication')
