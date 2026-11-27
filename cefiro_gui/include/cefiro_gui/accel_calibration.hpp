#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
namespace cefiro::gui {
using CalVec=std::array<double,3>;
struct AccelFaceCapture {
    unsigned n{};CalVec mean{},m2{};
    static constexpr unsigned required=60;
    bool add(const CalVec& a,const CalVec& gyro_deg_s,unsigned face){
        if(face>=6)return false;
        for(unsigned j=0;j<3;j++)if(!std::isfinite(a[j])||!std::isfinite(gyro_deg_s[j]))return false;
        const double mag=std::hypot(a[0],a[1],a[2]),rate=std::hypot(gyro_deg_s[0],gyro_deg_s[1],gyro_deg_s[2]);
        if(mag<7.5||mag>12.5||rate>5.0||(face%2? -1.:1.)*a[face/2]<7.0)return false;
        for(unsigned j=0;j<3;j++)if(j!=face/2&&std::abs(a[j])>2.0)return false;
        ++n;for(unsigned j=0;j<3;j++){double d=a[j]-mean[j];mean[j]+=d/n;m2[j]+=d*(a[j]-mean[j]);}return true;
    }
    bool stable()const {if(n<required)return false;for(double v:m2)if(std::sqrt(v/(n-1))>0.18)return false;return true;}
};
struct AccelCalibration {CalVec offset{},scale{};double max_residual{};};
inline std::optional<AccelCalibration> fitAccelCalibration(const std::array<CalVec,6>& faces){
    constexpr double g=9.80665;AccelCalibration c;
    for(auto& a:faces)for(double x:a)if(!std::isfinite(x))return {};
    for(unsigned i=0;i<3;i++){
        const double hi=faces[2*i][i],lo=faces[2*i+1][i];
        if(hi<7.0||lo> -7.0)return {};
        c.offset[i]=(hi+lo)/2;c.scale[i]=2*g/(hi-lo);
        if(std::abs(c.offset[i])>1.5||c.scale[i]<0.8||c.scale[i]>1.25)return {};
    }
    for(unsigned f=0;f<6;f++){
        CalVec e{};for(unsigned i=0;i<3;i++)e[i]=(faces[f][i]-c.offset[i])*c.scale[i]-(i==f/2?(f%2?-g:g):0.);
        const double err=std::hypot(e[0],e[1],e[2]);if(err>0.35)return {};
        if(err>c.max_residual)c.max_residual=err;
    }
    return c;
}
}
