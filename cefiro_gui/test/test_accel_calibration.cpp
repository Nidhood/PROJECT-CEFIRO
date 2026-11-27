#include "cefiro_gui/accel_calibration.hpp"
#include <cassert>
int main(){using namespace cefiro::gui;std::array<CalVec,6> f{};CalVec bias{.12,-.09,.18},gain{1.04,.97,1.11};
 for(unsigned k=0;k<6;k++){AccelFaceCapture cap;for(unsigned j=0;j<3;j++)f[k][j]=bias[j]+(j==k/2?(k%2?-9.80665:9.80665)/gain[j]:0.);for(unsigned i=0;i<60;i++)assert(cap.add(f[k],{0.1,-.2,.3},k));assert(cap.stable());}
 auto c=fitAccelCalibration(f);assert(c);for(unsigned j=0;j<3;j++){assert(std::abs(c->offset[j]-bias[j])<1e-10);assert(std::abs(c->scale[j]-gain[j])<1e-10);}f[3][0]+=1.0;assert(!fitAccelCalibration(f));AccelFaceCapture cap;assert(!cap.add({0,0,9.8},{10,0,0},4));assert(!cap.add({0,0,9.8},{0,0,0},0));}
