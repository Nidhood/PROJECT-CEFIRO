#include "cefiro_gui/step_response.hpp"
#include <cassert>
#include <cmath>
using namespace cefiro::gui;
int main(){
    StepResponseAnalyzer a;
    assert(!a.metrics().valid);
    a.start(12.,0.,10.,0.);
    for(int i=0;i<=500;++i){
        const double t=double(i)*.01;
        double y=0;
        if(t>.2) y=12.*(1.-std::exp(-(t-.2)*2.));
        if(t>2.2) y=10.+(y-10.)*std::exp(-(t-2.2)*4.);
        a.add(12.+t,y);
    }
    const auto m=a.metrics();
    assert(m.valid && m.amplitude_deg==10.);
    assert(std::isfinite(m.dead_time_s) && m.dead_time_s>=.2);
    assert(std::isfinite(m.rise_time_s) && m.rise_time_s>0.);
    assert(m.overshoot_percent>0.);
    assert(m.peak_time_s>m.dead_time_s);
    assert(m.mae_deg>=0 && m.rmse_deg>=m.mae_deg-1e-10);
    a.reset();assert(!a.metrics().valid);
    return 0;
}
