#include "cefiro_gui/models.hpp"
#include <cassert>
#include <iostream>
#include <random>
using namespace cefiro::gui;
static Vec3 euler(const DisplayAttitude& a){return Quaternion{a.q[0],a.q[1],a.q[2],a.q[3]}.eulerDeg();}
int main(){
    DisplayAttitudeFilter filter;
    std::int64_t time=1000000000LL;
    auto step=[&](Vec3 a,Vec3 g,bool valid=true,bool fmu=false){time+=40000000;return filter.update(time,a,g,valid,fmu);};
    const Vec3 acc{1.0851,-1.078,-8.7051};
    const Vec3 bias{0.014660765716752368,-0.01832595714594046,-0.0024434609527920616};
    std::mt19937 rng(42);std::normal_distribution<double> noise(0.,0.065),gyro_noise(0.,0.001);
    DisplayAttitude sample;
    double raw_sq=0,filtered_sq=0;unsigned n=0;
    const double expected=std::atan2(acc[1],acc[2])*rad2deg;
    for(int i=0;i<750;++i){Vec3 a=acc,g=bias;for(int j=0;j<3;++j){a[j]+=noise(rng);g[j]+=gyro_noise(rng);}
      sample=step(a,g);
      if(i>100){assert(sample.valid);const double raw=angleErrorDeg(expected,std::atan2(a[1],a[2])*rad2deg);const double err=angleErrorDeg(expected,euler(sample)[0]);raw_sq+=raw*raw;filtered_sq+=err*err;++n;}}
    assert(std::abs(angleErrorDeg(expected,euler(sample)[0]))<0.5);
    assert(filtered_sq<raw_sq*0.20);
    std::cout<<"Stationary synthetic noise RMS raw="<<std::sqrt(raw_sq/n)<<" filtered="<<std::sqrt(filtered_sq/n)<<" deg\n";
    // Moving plant: simulate a pure body roll with gravity rotating consistently.
    const Vec3 initial=euler(sample);const double rotation_rate=0.35;
    for(int i=1;i<=75;++i){const double ang=i*.04*rotation_rate;Vec3 a{acc[0],std::cos(ang)*acc[1]+std::sin(ang)*acc[2],-std::sin(ang)*acc[1]+std::cos(ang)*acc[2]};sample=step(a,{bias[0]+rotation_rate,bias[1],bias[2]});assert(sample.valid);}
    assert(std::abs(angleErrorDeg(initial[0]+rotation_rate*3*rad2deg,euler(sample)[0]))<4.0);
    // FMU quaternion always wins; fallback can never claim validity on its behalf.
    assert(!step(acc,bias,true,true).valid);
    assert(!step(acc,bias).valid);
    for(int i=0;i<40;++i)sample=step(acc,bias);
    assert(sample.valid);
    time+=1000000000LL;assert(!step(acc,bias).valid);
    assert(!step(acc,bias,false).valid);
    for(int i=0;i<100;++i)assert(!step(acc,{.5,0.,0.}).valid); // no initialization in motion
    std::cout<<"PASS: movement, FMU priority, gap and invalid sample rejection, stationary start\n";
}
