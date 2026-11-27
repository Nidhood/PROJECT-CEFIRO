#include "cefiro_gui/recording_time.hpp"
#include "cefiro_gui/recording_join.hpp"
#include <cassert>
#include <iostream>
using namespace cefiro::gui;
using namespace std::chrono_literals;
int main(){
    assert(recordingUnitSeconds(0)*500==.5);
    assert(recordingUnitSeconds(1)*30==30);
    assert(recordingUnitSeconds(2)*2==120);
    assert(recordingUnitSeconds(3)*.5==1800);
    assert(!captureTimeContains(.999,1.,2.));assert(captureTimeContains(1.,1.,2.));assert(!captureTimeContains(2.,1.,2.));
    assert(captureTimeContains(100000.,0.,-1.));
    assert(captureIntervalAllows(.001,.001,0));assert(!captureIntervalAllows(.010,0.,20));assert(captureIntervalAllows(.020,0.,20));
    RecordingJoin join;Telemetry s;auto t=RecordingJoin::Clock::now();
    // A render stall must not turn 1000 distinct samples into one row.
    for(int i=0;i<1000;++i){s.sequence=i;s.capture_elapsed_s=i*.005;join.pushImu(0,s,t+i*5ms);}
    auto rows=join.drain(t+6s);assert(rows.size()==1000);assert(join.dropped()==0);
    for(int i=0;i<1000;++i){assert(rows[i].sequence==i);assert(rows[i].capture_elapsed_s==i*.005);}
    join.clear();for(std::size_t i=0;i<RecordingJoin::capacity+4;++i)join.pushImu(0,s,t);
    assert(join.dropped()==4);assert(join.drain(t,true).size()==RecordingJoin::capacity);
    join.clear();assert(join.dropped()==0);
    s.from_real=true;join.pushImu(55,s,t);assert(join.drain(t+121ms).size()==1);assert(join.unmatched()==1);
    std::cout<<"PASS: duration conversion, capture boundaries, full rate, interval, render stall, overflow accounting and unmatched rows\n";
}
