#include "cefiro_gui/predictive_roll.hpp"
#include <cassert>
#include <iostream>
using namespace cefiro::gui;
int main(int argc,char** argv){
    autotune::Model m;m.a=18.8138378;m.b=.7587913045;m.cw=19.95237227;m.ccw=19.91520814;m.delay=.28;
    auto k=predictive_roll::discreteLqr(m,1000.);
    assert(std::abs(k.kp-1.30068923)<1e-5&&std::abs(k.kd-.27243876)<1e-5);
    assert(predictive_roll::linearScreen(m,k,14,14));
    assert(!predictive_roll::linearScreen(m,k,14,8));
    if(argc==3)m=autotune::fit({autotune::read(argv[1]),autotune::read(argv[2])});
    auto candidates=predictive_roll::search(m);assert(candidates.size()==234);
    int robust=0,all=0,maxs=0,maxr=0,maxt=0;
    for(auto& c:candidates){assert(c.cases==24&&std::isfinite(c.cost));robust+=c.linearPass==c.linearCases;all+=c.linearPass==c.linearCases&&c.trackPass==c.cases;maxs=std::max(maxs,c.swingPass);maxr=std::max(maxr,c.regPass);maxt=std::max(maxt,c.trackPass);}
    auto c=candidates.front();auto nominal=predictive_roll::simulate(m,c,int(std::lround(m.delay/predictive_roll::h)),1,1,1,.02);
    std::cout<<"candidates="<<candidates.size()<<" robust="<<robust<<" all="<<all<<" max swing/reg/track="<<maxs<<'/'<<maxr<<'/'<<maxt<<"\n";
    std::cout<<"best: "<<c.energy<<' '<<c.swingGain<<' '<<c.duration<<' '<<c.direction<<" predictor="<<c.pred<<" K="<<c.k.kp<<','<<c.k.kd<<" linear="<<c.linearPass<<'/'<<c.linearCases<<" phases="<<c.swingPass<<','<<c.regPass<<','<<c.trackPass<<" upper="<<c.upperRegPass<<','<<c.upperTrackPass<<" nominal="<<nominal.swing<<','<<nominal.reg<<','<<nominal.track<<'\n';
}
