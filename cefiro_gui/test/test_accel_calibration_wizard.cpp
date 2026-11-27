#include "cefiro_gui/accel_calibration_wizard.hpp"
#include <cassert>
#include <limits>
using namespace cefiro::gui;
using State=AccelCalibrationWizard::State;
int main(){
    AccelCalibrationWizard w;
    const CalVec bias{.12,-.09,.18},scale{1.04,.97,1.11},still{.1,-.2,.3};
    auto sample=[&](unsigned f){CalVec a=bias;a[f/2]+=(f%2?-9.80665:9.80665)/scale[f/2];return a;};
    double t=1.;
    assert(w.expectedFace()==4);
    assert(w.update(sample(5),still,t)==State::WrongFace);
    assert(w.update(sample(4),{20,0,0},t+=.04)==State::Motion);
    assert(w.update(sample(4),still,t+=.04)==State::Stabilizing);
    assert(w.update(sample(4),still,t)==State::InvalidTime);
    assert(w.count()==0);
    assert(w.update(sample(4),still,t+=1.)==State::InvalidTime);
    assert(w.count()==0);
    assert(w.update({0,0,std::numeric_limits<double>::quiet_NaN()},still,t+=.04)==State::WrongFace);
    for(unsigned step=0;step<6;++step){
        const unsigned face=kWizardFaces[step];
        assert(w.expectedFace()==int(face));
        bool green=false;
        for(unsigned i=0;i<150 && w.step()==step;++i){
            auto state=w.update(sample(face),still,t+=.04);
            if(state==State::Ready)green=true;
        }
        assert(green);assert(w.step()==step+1);
        assert(bool(w.result())==(step==5));
        if(step<5)assert(w.update(sample(face),still,t+=.04)==State::WrongFace);
    }
    assert(w.result());
    for(unsigned j=0;j<3;++j){assert(std::abs(w.result()->offset[j]-bias[j])<1e-9);assert(std::abs(w.result()->scale[j]-scale[j])<1e-9);}
    w={};
    for(int i=0;i<58;++i)w.update(sample(4),still,t+=.04);
    w.interrupt();assert(w.count()==0 && w.step()==0);
    // A face that looks axis-aligned but violates the full six-face residual is not exported.
    for(unsigned step=0;step<6;++step){
        auto a=sample(kWizardFaces[step]);if(step==2)a[1]+=.8;
        for(int i=0;i<150&&w.step()==step;++i)w.update(a,still,t+=.04);
    }
    assert(w.step()==6 && !w.result());
    // Alternating noisy samples must never become green or captured.
    w={};for(int i=0;i<300;++i){auto a=sample(4);a[0]+=(i%2?.5:-.5);auto st=w.update(a,still,t+=.04);assert(st!=State::Ready&&st!=State::Captured);}assert(w.step()==0);
}
