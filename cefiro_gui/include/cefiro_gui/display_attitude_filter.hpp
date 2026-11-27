#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace cefiro::gui {
// Independent six-axis display estimator. Never used as FMU attitude or for arming.
struct DisplayAttitude {
    bool valid{false};
    std::array<double,4> q{1.,0.,0.,0.}; // Hamilton body-to-world
};
class DisplayAttitudeFilter {
    using V = std::array<double,3>;
    using Q = std::array<double,4>;
    static double norm(const V& v) { return std::hypot(v[0],v[1],v[2]); }
    static V diff(const V& a,const V& b) {return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
    static V cross(const V& a,const V& b) {return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
    static Q multiply(const Q& a,const Q& b) {
        return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
                a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
                a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
    }
    static bool finite(const V& v) {return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});}
    bool rotate(const V& omega,double dt) {
        const double speed=norm(omega);
        if(speed>1.e-12){
            const double h=0.5*speed*dt, k=std::sin(h)/speed;
            q_=multiply(q_,{std::cos(h),k*omega[0],k*omega[1],k*omega[2]});
        }
        const double n=std::sqrt(q_[0]*q_[0]+q_[1]*q_[1]+q_[2]*q_[2]+q_[3]*q_[3]);
        if(!std::isfinite(n)||n<1.e-9)return false;
        for(auto& v:q_)v/=n;
        return true;
    }
    void clearQuiet(){quiet_time_=0.;count_=0;acc_sum_={};gyro_sum_={};}
public:
    static constexpr double max_gap_s=0.25;
    void reset(){*this={};}
    DisplayAttitude update(std::int64_t stamp_ns,const V& acc,const V& gyro,bool raw_valid,bool fmu_valid) {
        // Authoritative orientation always takes priority. No fallback data is returned.
        if(fmu_valid || !raw_valid || stamp_ns<=0 || !finite(acc) || !finite(gyro) || norm(gyro)>8.5){reset();return {};}
        if(last_ns_==0){last_ns_=stamp_ns;return {};}
        const double dt=double(stamp_ns-last_ns_)*1.e-9;
        if(dt<=0. || dt>max_gap_s){reset();last_ns_=stamp_ns;return {};}
        // Very short intervals can occur when CF1 packets arrive in a burst.
        // Accumulate their time into the next usable update, rather than divide by it.
        if(dt<0.002)return ready_?DisplayAttitude{true,q_}:DisplayAttitude{};
        last_ns_=stamp_ns;
        const double n=norm(acc);
        if(!ready_){
            // Only a stationary diagnostic start. The local norm may have a scale error;
            // accepting it here does NOT relax the firmware's calibration gate.
            const bool quiet=n>=7.5 && n<=12.0 && norm(gyro)<0.08;
            if(!quiet){clearQuiet();return {};}
            if(count_ && (norm(diff(acc,anchor_acc_))>0.45 || norm(diff(gyro,anchor_gyro_))>0.045))clearQuiet();
            if(!count_){anchor_acc_=acc;anchor_gyro_=gyro;}
            ++count_;quiet_time_+=dt;
            for(int i=0;i<3;++i){acc_sum_[i]+=acc[i];gyro_sum_[i]+=gyro[i];}
            if(quiet_time_<1.2 || count_<20)return {};
            for(int i=0;i<3;++i){filtered_acc_[i]=acc_sum_[i]/count_;bias_[i]=gyro_sum_[i]/count_;}
            baseline_=norm(filtered_acc_);
            const double roll=std::atan2(filtered_acc_[1],filtered_acc_[2]);
            const double pitch=std::atan2(-filtered_acc_[0],std::hypot(filtered_acc_[1],filtered_acc_[2]));
            const double cr=std::cos(roll/2),sr=std::sin(roll/2),cp=std::cos(pitch/2),sp=std::sin(pitch/2);
            q_={cr*cp,sr*cp,cr*sp,-sr*sp}; // yaw=0, relative to this initialization
            ready_=true;return {true,q_};
        }
        V omega=diff(gyro,bias_);
        const double rate=norm(omega);
        // Gyro propagation, slow gravity correction. Reject dynamic acceleration as a tilt reference.
        if(n>1.e-6 && std::abs(n-baseline_)<1.5 && rate<2.5){
            const double alpha=-std::expm1(-dt/0.22);
            for(int i=0;i<3;++i)filtered_acc_[i]+=alpha*(acc[i]-filtered_acc_[i]);
            const double nf=norm(filtered_acc_);
            if(nf>1.e-6){
                const V m={filtered_acc_[0]/nf,filtered_acc_[1]/nf,filtered_acc_[2]/nf};
                const V p={2*(q_[1]*q_[3]-q_[0]*q_[2]),2*(q_[2]*q_[3]+q_[0]*q_[1]),1-2*(q_[1]*q_[1]+q_[2]*q_[2])};
                const V e=cross(m,p);
                const double dot=m[0]*p[0]+m[1]*p[1]+m[2]*p[2];
                const double weight=std::clamp(1.-std::abs(n-baseline_)/1.5,0.,1.)*std::clamp(1.-rate/2.5,0.,1.);
                // Do not pull the attitude toward a large contradictory acceleration.
                if(dot>0.7)for(int i=0;i<3;++i)omega[i]+=1.2*weight*e[i];
            }
        }
        if(!rotate(omega,dt)){reset();return {};}
        return {true,q_};
    }
private:
    std::int64_t last_ns_{};
    bool ready_{};
    double quiet_time_{},baseline_{};
    unsigned count_{};
    V acc_sum_{},gyro_sum_{},anchor_acc_{},anchor_gyro_{},filtered_acc_{},bias_{};
    Q q_{1.,0.,0.,0.};
};
}
