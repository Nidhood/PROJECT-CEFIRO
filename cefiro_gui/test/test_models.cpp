#include "cefiro_gui/models.hpp"
#include "cefiro_gui/waveform.hpp"
#include <cassert>
#include <cmath>
using namespace cefiro::gui;
int main(){
 Quaternion q{};assert(q.normalize());auto a=q.eulerDeg();assert(std::abs(a[0])<1.e-8);
 Quaternion yaw90{std::sqrt(.5),0,0,std::sqrt(.5)};
 assert(yaw90.normalize());assert(std::abs(yaw90.eulerDeg()[2]-90.)<1.e-6);
 assert(std::abs(angleErrorDeg(-179,179)-2)<1.e-9);
 WaveformConfig c;c.type=WaveformType::Sine;c.amplitude=10.;c.frequency_hz=.25;
 assert(std::abs(waveformValue(c,1.0)-10.)<1.e-10);
 c.type=WaveformType::Step;c.delay_s=1.0;c.initial_deg=-10.;c.final_deg=15.;
 assert(waveformValue(c,0.5)==-10. && waveformValue(c,1.5)==15.);
 c.type=WaveformType::Ramp;c.rise_s=2.0;
 assert(std::abs(waveformValue(c,2.0)-2.5)<1.e-9);
 Telemetry s;s.orientation_valid=true;s.setpoint_deg={10,0,-179};s.attitude_deg={0,0,179};
 ErrorStats st;st.add(s);assert(st.count==1 && std::abs(st.rmse(0)-10)<1.e-8 && std::abs(st.rmse(2)-2.)<1.e-8);
 return 0;
}
