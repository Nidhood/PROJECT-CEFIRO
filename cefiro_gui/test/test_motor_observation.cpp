#include "cefiro_gui/recording_join.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace cefiro::gui;
using namespace std::chrono_literals;
int main(){
    const NominalMotorMap m{48,300,12000,25};
    auto r=m.inverse({0,48,174,300});
    assert(r && (*r)[0]==0 && (*r)[1]==25 && (*r)[2]==6012.5 && (*r)[3]==12000);
    assert(!m.inverse({0,47,48,300}));
    assert(!m.inverse({0,48,301,300}));
    assert(!NominalMotorMap{}.inverse({0,0,0,0}));
    const NominalMotorMap full{48,2047,12000,25};
    assert(full.inverse({0,48,200,2047})->at(3)==12000);
    const auto t=MotorClock::now();
    const HostDshotSample a{{48,60,80,100},HostOrigin::Topic,t,0};
    const HostDshotSample b{{48,90,100,120},HostOrigin::Status,t+100ms,0.01};
    assert(chooseHost(a,b,t+110ms)->codes[3]==120);
    assert(chooseHost(a,{},t+40ms)->codes[1]==60);
    assert(!chooseHost(a,b,t+500ms));
    const HostDshotSample zero{{0,0,0,0},HostOrigin::Topic,t+120ms,0};
    assert(chooseHost(zero,b,t+130ms)->codes[3]==0);
    RecordingJoin j;
    Telemetry s; s.host_dshot_valid=true;s.host_dshot=a.codes;s.host_origin=a.origin;s.nominal_map=m;s.host_dshot_age_s=.02;
    j.pushImu(1,s,t);auto rows=j.drain(t+130ms);
    assert(rows.size()==1 && rows[0].host_dshot_valid && !rows[0].fmu_dshot_valid);
    j.pushImu(2,s,t+140ms);IdentificationFields f;f.fmu_dshot_valid=true;f.fmu_dshot={0,48,174,300};
    j.pushId(2,f,t+150ms);rows=j.drain(t+150ms);
    assert(rows.size()==1 && rows[0].host_dshot_valid && rows[0].fmu_dshot_valid && rows[0].host_origin==HostOrigin::Topic);
    f.host_dshot_valid=true;f.host_dshot=b.codes;f.host_dshot_age_s=.03;
    j.pushImu(3,s,t+160ms);j.pushId(3,f,t+170ms);rows=j.drain(t+170ms);
    assert(rows[0].host_origin==HostOrigin::Identification && rows[0].host_dshot==b.codes);
    f.host_dshot_age_s=.5;j.pushImu(4,s,t+180ms);j.pushId(4,f,t+190ms);rows=j.drain(t+190ms);
    assert(rows[0].host_origin==HostOrigin::Topic && rows[0].host_dshot==a.codes);
    std::cout<<"PASS: nominal inverse, both caps, invalid codes, source priority, age, explicit zero, host-only recording, exact FMU join and fallback retention\n";
}
