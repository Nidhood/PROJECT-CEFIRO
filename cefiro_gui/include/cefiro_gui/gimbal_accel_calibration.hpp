#pragma once
#include "cefiro_gui/accel_calibration.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace cefiro::gui {
// Axis-diagonal ellipsoid calibration, compatible with the existing FMU header.
// No assumed exact pose or attitude estimate enters the fit.
class GimbalAccelCalibration {
public:
    enum class State { Collecting, Motion, InvalidTime, Captured, Complete, NeedCoverage, FitFailed };
    static constexpr unsigned bins_count=26, minimum_bins=14;
    unsigned count() const {unsigned n=0;for(const auto& b:bins_)n+=b.valid;return n;}
    unsigned sectorCount(unsigned face) const {unsigned n=0;for(const auto& b:bins_)if(b.valid&&sector(b.a)==face)++n;return n;}
    const std::optional<AccelCalibration>& result() const {return result_;}
    double progress() const {return result_?1.:std::min(.99,double(count())/minimum_bins);}
    double rms() const {return rms_;}
    void interrupt(){window_.clear();last_=-1.;}
    // Guide to an actual unvisited bin, not just one of six broad sectors.
    bool hasDirection(unsigned i) const {return i<bins_count && bins_[i].valid;}
    static CalVec direction(unsigned i) {return directions().at(i);}
    unsigned suggestedBin() const {
        const auto dirs=directions();
        const CalVec reference=last_direction_;
        unsigned best=bins_count;double score=-10.;
        for(unsigned i=0;i<bins_count;++i){
            if(bins_[i].valid)continue;
            const auto& d=dirs[i];
            // First request missing axis coverage; otherwise choose a nearby new direction.
            const double priority=sectorCount(sector(d))==0?4.:0.;
            const double q=priority+dot(reference,d);
            if(q>score){score=q;best=i;}
        }
        return best;
    }
    unsigned suggestedFace() const {const auto i=suggestedBin();return i<bins_count?sector(direction(i)):sector(last_direction_);}
    State update(const CalVec& a,const CalVec& gyro,double stamp){
        if(result_)return State::Complete;
        if(!std::isfinite(stamp)||stamp<=0.||(last_>=0.&&(stamp<=last_||stamp-last_>.25))){interrupt();return State::InvalidTime;}
        last_=stamp;
        for(unsigned i=0;i<3;++i)if(!std::isfinite(a[i])||!std::isfinite(gyro[i])){window_.clear();return State::Motion;}
        const double n=norm(a);
        if(n<7.5||n>12.5||norm(gyro)>12.){window_.clear();return State::Motion;}
        window_.push_back({a,stamp});
        while(window_.size()>1&&stamp-window_.front().t>.45)window_.erase(window_.begin());
        if(window_.size()<6||stamp-window_.front().t<.24)return State::Collecting;
        CalVec mean{};double minmag=12.5,maxmag=0.;
        for(const auto& s:window_){for(unsigned j=0;j<3;++j)mean[j]+=s.a[j]/window_.size();minmag=std::min(minmag,norm(s.a));maxmag=std::max(maxmag,norm(s.a));}
        // Small averaging interval accepts gentle rotation, but excludes abrupt linear acceleration.
        if(maxmag-minmag>.28||dot(unit(window_.front().a),unit(window_.back().a))<.993){window_.clear();return State::Motion;}
        const auto d=unit(mean);unsigned bin=0;double score=-2.;const auto dirs=directions();
        for(unsigned i=0;i<bins_count;++i){const double q=dot(d,dirs[i]);if(q>score){score=q;bin=i;}}
        auto& b=bins_[bin];
        if(b.valid&&score<=b.score+.004)return State::NeedCoverage;
        // Prevent effectively duplicate directions from inflating coverage.
        for(unsigned i=0;i<bins_count;++i)if(i!=bin&&bins_[i].valid&&dot(d,unit(bins_[i].a))>.985)return State::NeedCoverage;
        const bool new_direction=!b.valid;
        b={mean,score,true};if(new_direction)last_direction_=d;window_.clear();
        if(count()<minimum_bins)return new_direction?State::Captured:State::NeedCoverage;
        for(unsigned f=0;f<6;++f)if(!sectorCount(f))return State::NeedCoverage;
        // Require coverage in both signs of each axis, not merely six labels.
        for(unsigned j=0;j<3;++j){double lo=1.,hi=-1.;for(const auto& v:bins_)if(v.valid){const auto q=unit(v.a);lo=std::min(lo,q[j]);hi=std::max(hi,q[j]);}if(lo>-.72||hi<.72)return State::NeedCoverage;}
        std::vector<CalVec> data;for(const auto& v:bins_)if(v.valid)data.push_back(v.a);
        auto c=fit(data,-1);if(!c)return State::FitFailed;
        // Leave-one-direction-out validation. Do not export a fit based solely on training residuals.
        double sum=0.,worst=0.;
        for(unsigned i=0;i<data.size();++i){const auto cv=fit(data,int(i));if(!cv)return State::FitFailed;const double e=residual(*cv,data[i]);sum+=e*e;worst=std::max(worst,e);}
        rms_=std::sqrt(sum/data.size());
        if(rms_>.20||worst>.45||c->max_residual>.35)return State::FitFailed;
        c->max_residual=std::max(c->max_residual,worst);result_=c;return State::Complete;
    }
private:
    struct Sample{CalVec a;double t;};struct Bin{CalVec a{};double score{};bool valid{};};
    std::array<Bin,bins_count> bins_{};std::vector<Sample> window_;std::optional<AccelCalibration> result_;double last_{-1.},rms_{};CalVec last_direction_{0.,0.,1.};
    static double norm(const CalVec& a){return std::hypot(a[0],a[1],a[2]);}
    static double dot(const CalVec& a,const CalVec& b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
    static CalVec unit(CalVec a){const double n=norm(a);for(double& x:a)x/=n;return a;}
    static unsigned sector(const CalVec& a){unsigned j=0;for(unsigned i=1;i<3;++i)if(std::abs(a[i])>std::abs(a[j]))j=i;return 2*j+(a[j]<0.);}
    static std::array<CalVec,bins_count> directions(){std::array<CalVec,bins_count> out{};unsigned i=0;for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z)if(x||y||z)out[i++]=unit({double(x),double(y),double(z)});return out;}
    static double residual(const AccelCalibration& c,const CalVec& a){CalVec corrected{};for(unsigned j=0;j<3;++j)corrected[j]=(a[j]-c.offset[j])*c.scale[j];return std::abs(norm(corrected)-9.80665);}
    static std::optional<AccelCalibration> fit(const std::vector<CalVec>& data,int exclude){
        constexpr double g=9.80665;double mat[6][7]{};
        // In normalized coordinates: sum(A_i*x_i^2+B_i*x_i)=1.
        for(unsigned k=0;k<data.size();++k){if(int(k)==exclude)continue;double row[6]{};for(unsigned j=0;j<3;++j){const double x=data[k][j]/g;row[j]=x*x;row[j+3]=x;}for(unsigned i=0;i<6;++i){for(unsigned j=0;j<6;++j)mat[i][j]+=row[i]*row[j];mat[i][6]+=row[i];}}
        double minp=std::numeric_limits<double>::infinity(),maxp=0.;
        for(unsigned i=0;i<6;++i){unsigned p=i;for(unsigned j=i+1;j<6;++j)if(std::abs(mat[j][i])>std::abs(mat[p][i]))p=j;const double pivot=std::abs(mat[p][i]);if(pivot<1e-8)return {};minp=std::min(minp,pivot);maxp=std::max(maxp,pivot);for(unsigned j=0;j<7;++j)std::swap(mat[i][j],mat[p][j]);const double d=mat[i][i];for(unsigned j=i;j<7;++j)mat[i][j]/=d;for(unsigned k=0;k<6;++k)if(k!=i){const double f=mat[k][i];for(unsigned j=i;j<7;++j)mat[k][j]-=f*mat[i][j];}}
        if(minp/maxp<.015)return {};
        AccelCalibration c;double constant=1.;for(unsigned j=0;j<3;++j){const double a=mat[j][6],b=mat[j+3][6];if(!std::isfinite(a)||!std::isfinite(b)||a<=0.)return {};const double center=-b/(2*a);c.offset[j]=g*center;constant+=a*center*center;}
        for(unsigned j=0;j<3;++j){c.scale[j]=std::sqrt(mat[j][6]/constant);if(!std::isfinite(c.scale[j])||std::abs(c.offset[j])>1.5||c.scale[j]<.8||c.scale[j]>1.25)return {};}
        for(unsigned k=0;k<data.size();++k)if(int(k)!=exclude)c.max_residual=std::max(c.max_residual,residual(c,data[k]));
        return c;
    }
};
}
