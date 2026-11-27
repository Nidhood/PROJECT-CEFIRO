"""Static wiring checks; not a Qt/ROS integration test."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=(root/'src/main_window.cpp').read_text()
def body(a,b):return s.split('void MainWindow::'+a,1)[1].split('void MainWindow::'+b,1)[0]
start=body('startEmbeddedRoll(){','serviceEmbeddedRoll(){')
service=body('serviceEmbeddedRoll(){','finishEmbeddedRoll(')
finish=body('finishEmbeddedRoll(','startOutput(){')
for token in ('startRecording(&config)','publishSwingTuning(true,{kp,kd,duration,-1.,double(embedded_roll_token_),1.})','stat[2]=="1"','stat[20]=="1"','QMessageBox::No','swing_session_active','external_supply_policy_ready','firmware_fault_latched'):
    assert token in start,token
assert start.index('startRecording(&config)')<start.index('publishSwingTuning(true,{kp')
for block in (start,service,finish):
    for forbidden in ('publishReference(', 'publishMotorCommand(', 'setMotorArmed(', 'bridge_.setEnabled(true)', 'publishGains('):
        assert forbidden not in block,forbidden
for token in ('embedded_roll_lease_.elapsed()>=80','embedded_roll_feedback_.elapsed()>450','publishSwingTuning(true,{double(embedded_roll_token_)})','finishEmbeddedRoll(raw)','recording_stream_.status()','transport_connected'):
    assert token in service,token
assert 'publishSwingTuning(true,{})' in finish and 'if(emergency)bridge_.emergencyStop()' in finish
assert 'if(embedded_roll_active_){finishEmbeddedRoll' in body('stopOutput(bool emergency){','tickWaveform(){')
assert 'if(!command_active_||e_stop_||embedded_roll_active_)return;' in body('publishCommand(){','publishMotorCommand(){')
assert 'startEmbeddedRoll();return;' in body('startOutput(){','stopOutput(bool emergency){')
assert 'if (embedded_roll_active_) finishEmbeddedRoll("GUI closed",false);' in s
print('Embedded strategy static routing, recording and stop contracts: PASS')
