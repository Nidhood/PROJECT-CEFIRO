"""Static contract for SIM model-limited roll/pitch and full-heading yaw."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=(root/'src/main_window.cpp').read_text()
p=(root/'src/attitude_command_pad.cpp').read_text()
r=(root/'src/ros_bridge.cpp').read_text()
h=(root/'include/cefiro_gui/ros_bridge.hpp').read_text()
for token in ('void MainWindow::refreshReferenceLimits()',
              'void MainWindow::applyReferenceLimits(const Vec3& requested)',
              'bridge_.simReferenceEnvelope(limits)',
              'command_pad_->setLimits(limits)',
              'std::clamp(command_deg_[i],-limits[i],limits[i])',
              'applyReferenceLimits({5.0,5.0,10.0})'):
    assert token in s,token
assert 'kReferenceLimitsDeg' not in s
assert 'kSafeLimits' not in p
assert 'void AttitudeCommandPad::setLimits(const Vec3& limits)' in p
assert 'transient_local()' in r
assert 'onReferenceEnvelope' in r and 'sub_reference_envelope_' in h
assert '/cefiro/control/reference_envelope_deg' in r
print('GUI SIM/REAL envelope and full-heading yaw contracts PASSED')
