#include "cefiro_gui/gimbal_accel_calibration.hpp"
#include <cassert>
#include <iostream>
using namespace cefiro::gui;
int main(){
 GimbalAccelCalibration w;double t=1.;CalVec bias{.12,-.09,.18},gain{1.04,.97,1.11};
 auto add=[&](CalVec d,bool corrupt=false){double n=std::hypot(d[0],d[1],d[2]);for(unsigned k=0;k<24;++k){CalVec a{};for(unsigned j=0;j<3;++j)a[j]=bias[j]+9.80665*d[j]/n/gain[j]+.008*std::sin(k+j);if(corrupt)a[0]+=.75;w.update(a,{3.,1.,.5},t+=.04);}};
 // Six exact or near-axis samples are not enough; calibration accepts oblique orientations.
 for(unsigned j=0;j<3;++j)for(int sign:{-1,1}){CalVec d{.08,.07,.09};d[j]=sign;add(d);}
 assert(!w.result());
 for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z)if(x||y||z)add({x+.08,y-.04,z+.07});
 assert(w.result());for(unsigned j=0;j<3;++j){assert(std::abs(w.result()->offset[j]-bias[j])<.03);assert(std::abs(w.result()->scale[j]-gain[j])<.005);}
 std::cout<<"coverage "<<w.count()<<" RMS "<<w.rms()<<'\n';
 w={};for(int i=0;i<100;++i)add({1.,0.,0.});assert(!w.result()&&w.count()==1);
 w={};for(int i=0;i<100;++i)w.update({0.,0.,9.8},{20.,0.,0.},t+=.04);assert(w.count()==0);
 w={};for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)if(x||y)add({double(x),double(y),0.});assert(!w.result());
 w={};for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z)if(x||y||z)add({double(x),double(y),double(z)},(x+y+z)%2==0);assert(!w.result());
 // Once the six broad faces are covered, the guidance must request an unused bin.
 w={};for(unsigned j=0;j<3;++j)for(int sign:{-1,1}){CalVec d{};d[j]=sign;add(d);}
 assert(w.count()==6);assert(!w.result());
 for(unsigned k=0;k<20&&!w.result();++k){
   const auto i=w.suggestedBin();assert(i<GimbalAccelCalibration::bins_count);assert(!w.hasDirection(i));
   const auto before=w.count();const auto d=w.direction(i);add(d);assert(w.count()>before);
 }
 assert(w.result());assert(w.count()>=GimbalAccelCalibration::minimum_bins);
 std::cout<<"guided calibration: "<<w.count()<<" directions, RMS "<<w.rms()<<'\n';
}
