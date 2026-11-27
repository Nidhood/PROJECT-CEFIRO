#include "cefiro_gui/recording_join.hpp"
#include <cassert>
#include <iostream>
using namespace cefiro::gui;
using namespace std::chrono_literals;
int main(){
    RecordingJoin join;
    const auto t=RecordingJoin::Clock::now();
    Telemetry s; s.from_real=true; s.orientation_valid=true; s.attitude_deg={12.,0.,0.};
    IdentificationFields f; f.fmu_dshot_valid=true; f.fmu_dshot={48,60,70,80};f.orientation_valid=true;
    join.pushImu(1,s,t);assert(join.drain(t+10ms).empty());
    join.pushId(1,f,t+20ms);auto rows=join.drain(t+21ms);assert(rows.size()==1&&rows[0].fmu_dshot_valid&&rows[0].fmu_dshot[3]==80);
    join.pushId(2,f,t+30ms);join.pushImu(2,s,t+40ms);rows=join.drain(t+41ms);assert(rows.size()==1&&rows[0].fmu_dshot_valid);
    join.pushImu(3,s,t+50ms);join.pushId(4,f,t+60ms);assert(join.drain(t+100ms).empty());
    rows=join.drain(t+180ms);assert(rows.size()==1&&!rows[0].fmu_dshot_valid&&rows[0].orientation_valid&&rows[0].attitude_deg[0]==12.);
    join.pushImu(5,s,t+200ms);join.pushId(5,f,t+330ms);rows=join.drain(t+331ms);assert(rows.size()==1&&!rows[0].fmu_dshot_valid);
    join.pushImu(0,s,t+350ms);join.pushId(0,f,t+351ms);rows=join.drain(t+352ms,true);assert(rows.size()==1&&!rows[0].fmu_dshot_valid);
    join.pushImu(6,s,t+360ms);join.clear();assert(join.drain(t+500ms).empty());
    join.pushImu(7,s,t+510ms);f.orientation_valid=false;join.pushId(7,f,t+511ms);rows=join.drain(t+512ms);assert(rows.size()==1&&!rows[0].orientation_valid&&rows[0].fmu_dshot_valid);
    assert(join.drain(t+530ms).empty());
    std::cout<<"PASS: IMU-only, both arrival orders, no unmatched/late/zero stamp pairing, reset, strict orientation, no duplicate rows\n";
}
