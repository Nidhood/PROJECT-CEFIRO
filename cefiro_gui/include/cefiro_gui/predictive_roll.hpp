#pragma once
// Offline-only predictor design. No transport, actuator or arm APIs.
#include "cefiro_gui/autotune.hpp"
#include <deque>
namespace cefiro::gui::predictive_roll {
using autotune::Model;
constexpr double h=.02, rad=autotune::rad, pi=autotune::pi;
struct State { double x{},p{}; };
struct Mat {double a{},b{},c{},d{};};
struct Zoh {Mat A;State B;};
inline State mul(Mat m,State x){return {m.a*x.x+m.b*x.p,m.c*x.x+m.d*x.p};}
inline Zoh zoh(double a,double b,double g){
    const double c=std::sqrt(b*b/4+a), e=std::exp(-b*h/2), sh=std::sinh(c*h)/c,ch=std::cosh(c*h);
    Mat A{e*(ch+b*sh/2),e*sh,e*a*sh,e*(ch-b*sh/2)};
    return {A,{(A.a-1)*g/a,A.c*g/a}};
}
struct Gain {double kp{},kd{},r{};};
inline Gain discreteLqr(const Model& m,double r){
    const auto z=zoh(m.a*std::cos(m.offset),m.b,1/m.j);const auto A=z.A;const auto B=z.B;
    const double q1=1/std::pow(5*rad,2),q2=1/std::pow(30*rad,2);
    double p11=q1,p12=0,p22=q2; Gain K;
    for(int i=0;i<10000;++i){
        const double v1=p11*B.x+p12*B.p,v2=p12*B.x+p22*B.p;
        const double s=r+B.x*v1+B.p*v2,t1=v1*A.a+v2*A.c,t2=v1*A.b+v2*A.d;
        K={t1/s,t2/s,r};
        const double n11=q1+A.a*(p11*A.a+p12*A.c)+A.c*(p12*A.a+p22*A.c)-t1*t1/s;
        const double n12=A.a*(p11*A.b+p12*A.d)+A.c*(p12*A.b+p22*A.d)-t1*t2/s;
        const double n22=q2+A.b*(p11*A.b+p12*A.d)+A.d*(p12*A.b+p22*A.d)-t2*t2/s;
        const double err=std::max({std::abs(n11-p11),std::abs(n12-p12),std::abs(n22-p22)});
        p11=n11;p12=n12;p22=n22;
        if(!std::isfinite(err))throw std::runtime_error("Discrete Riccati iteration diverged");
        if(err<1e-8*std::max(1.,std::abs(p11)))return K;
    }
    throw std::runtime_error("Discrete Riccati iteration did not converge");
}
inline State linearStep(Zoh z,State x,double u){auto s=mul(z.A,x);s.x+=z.B.x*u;s.p+=z.B.p*u;return s;}
// A sufficient discrete stability test for each sampled plant: ||M^N||_inf < 1.
// This is not a certificate over the continuous uncertainty set.
inline bool linearScreen(const Model& m,Gain k,int pred,int actual,double af=1.,double bf=1.,double uf=1.){
    const auto nominal=zoh(m.a*std::cos(m.offset),m.b,1/m.j);
    const auto plant=zoh(m.a*std::cos(m.offset)*af,m.b*bf,uf/m.j);
    const int n=2+std::max(pred,actual), q=n-2;
    std::vector<double> M(n*n),P(n*n),T(n*n);
    for(int c=0;c<n;++c){
        std::vector<double> v(n);v[c]=1.;State x{v[0],v[1]},xp=x;
        for(int j=q-pred;j<q;++j)xp=linearStep(nominal,xp,v[2+j]);
        const double u=-k.kp*xp.x-k.kd*xp.p;
        auto next=linearStep(plant,x,actual?v[n-actual]:u);
        M[c]=next.x;M[n+c]=next.p;
        for(int j=0;j<q-1;++j)M[(2+j)*n+c]=v[3+j];
        if(q)M[(n-1)*n+c]=u;
        P[c*n+c]=1.;
    }
    // Repeated squaring computes M^256 without an eigensolver.
    for(int i=0;i<8;++i){
        const auto& S=i?P:M;std::fill(T.begin(),T.end(),0.);
        for(int a=0;a<n;++a)for(int b=0;b<n;++b)for(int c=0;c<n;++c)T[a*n+c]+=S[a*n+b]*S[b*n+c];
        P.swap(T);
    }
    double norm=0.;for(int a=0;a<n;++a){double sum=0.;for(int b=0;b<n;++b)sum+=std::abs(P[a*n+b]);if(!std::isfinite(sum))return false;norm=std::max(norm,sum);}
    return norm<.99;
}
struct Params {Gain k; double swingGain{},swingKd{},duration{};int direction{1},pred{};bool energy{};int linearPass{},linearCases{},swingPass{},regPass{},trackPass{},upperRegPass{},upperTrackPass{},cases{};double cost{};};
struct Trace {double t,roll,p,ref,requested,applied;int phase;};
struct Result {bool swing{},reg{},track{};double cost{},peak{},capture{-1.};};
// State is absolute, input is torque in Nm. Integrate with four substeps.
inline State evolve(const Model& m,State x,double u,double af=1.,double bf=1.,double uf=1.){
    for(int i=0;i<4;++i){const double step=h/4.;x.p+=step*(af*m.a*std::sin(x.x-m.offset)-bf*m.b*x.p+uf*u/m.j);x.x+=step*x.p;}return x;
}
inline Result simulate(const Model& m,const Params& c,int actual,double af,double bf,double uf,double perturb,std::vector<Trace>* trace=nullptr,bool startUpper=false){
    // The voltage-aware fit is evaluated at the lowest observed voltage, not an extrapolated higher one.
    const double vf=m.voltage_normalized?std::pow(m.voltage_min/m.voltage_ref,2):1.;
    const double pos=std::min(1.1,m.j*m.cw*std::pow(m.maxcw/1000.,2)*vf),neg=std::min(1.1,m.j*m.ccw*std::pow(m.maxccw/1000.,2)*vf);
    const double target=c.direction>0?0.:-2*pi,start=m.offset-pi+perturb;
    State x{startUpper?target+perturb:start,0.};std::deque<double> hist(size_t(std::max(c.pred,actual)),0.);
    const double kg=m.j*m.a;Result o;int phase=startUpper?1:0;double since=0.,dwell=0.,trackDwell=0.,regDwell=0.; std::array<bool,3> stepPass{}; int previousStep=-1;
    for(int i=0;i<2000;++i){
        const double t=i*h;State xp=x;
        for(int j=int(hist.size())-c.pred;j<int(hist.size());++j)xp=evolve(m,xp,hist[size_t(j)]);
        const double err=autotune::wrap(x.x-target),predErr=autotune::wrap(xp.x-target);
        if(phase==0){
            if(std::abs(err)<9*rad&&std::abs(x.p)<25*rad&&std::abs(predErr)<9*rad&&std::abs(xp.p)<25*rad)dwell+=h;else dwell=0.;
            if(dwell>=.2){phase=1;since=0.;o.swing=true;o.capture=t;}
            if(t>25.)break;
        }
        if(phase==1&&since>=4.){if(regDwell<1.5)break;o.reg=true;phase=2;since=0.;}
        if(phase==2&&since>=9.){o.track=std::all_of(stepPass.begin(),stepPass.end(),[](bool v){return v;});break;}
        double ref=target,raw=0.;
        if(phase==0){
            if(c.energy){
                const double E=.5*m.j*xp.p*xp.p+kg*std::cos(xp.x-m.offset),Ed=kg*std::cos(m.offset);
                raw=m.j*m.b*xp.p+c.swingGain*(Ed-E)*xp.p;
                if(std::abs(xp.p)<.12&&std::abs(predErr)>1.)raw+=c.direction*.12;
            }else{
                const double z=std::clamp((t+c.pred*h)/c.duration,0.,1.),z2=z*z,z3=z*z2,z4=z*z3,z5=z*z4,d=target-start;
                ref=start+d*(10*z3-15*z4+6*z5);
                const double rate=d*(30*z2-60*z3+30*z4)/c.duration,acc=d*(60*z-180*z2+120*z3)/(c.duration*c.duration);
                raw=m.j*(acc+m.b*rate)-kg*std::sin(ref-m.offset)-c.swingGain*(xp.x-ref)-c.swingKd*(xp.p-rate);
            }
        }else{
            // Small reference steps, assessed only after equilibrium hold.
            if(phase==2)ref+=since<3.?5*rad:since<6.?-5*rad:0.;
            raw=-kg*std::sin(ref-m.offset)-c.k.kp*autotune::wrap(xp.x-ref)-c.k.kd*xp.p;
        }
        const double cmd=std::clamp(raw,-neg,pos),applied=actual?hist[hist.size()-size_t(actual)]:cmd;
        if(trace)trace->push_back({t,x.x,x.p,ref,cmd,applied,phase});
        if(phase==1){if(std::abs(err)<5*rad&&std::abs(x.p)<15*rad)regDwell+=h;else regDwell=0.;}
        if(phase==2){
            const int step=std::min(2,int(since/3.));
            if(step!=previousStep){trackDwell=0.;previousStep=step;}
            if(std::abs(autotune::wrap(x.x-ref))<3*rad&&std::abs(x.p)<10*rad)trackDwell+=h;else trackDwell=0.;
            // Each reference must have a full second of settled tracking at its end.
            stepPass[size_t(step)]=trackDwell>=1.;
        }
        o.cost+=h*(std::pow(autotune::wrap(x.x-ref),2)+.05*x.p*x.p+.5*cmd*cmd+std::abs(raw-cmd));
        o.peak=std::max(o.peak,std::abs(x.p));
        if(!hist.empty()){hist.pop_front();hist.push_back(cmd);}
        x=evolve(m,x,applied,af,bf,uf);since+=h;
        if(!std::isfinite(x.x)||!std::isfinite(x.p)||std::abs(x.p)>120*rad||std::abs(x.x-start)>4*pi)break;
        if(phase>=1&&(std::abs(autotune::wrap(x.x-target))>35*rad||std::abs(x.p)>100*rad))break;
    }
    o.cost+=(!o.swing&&!startUpper?1000.:0.)+(!o.reg?500.:0.)+(!o.track?250.:0.);return o;
}
inline std::vector<Params> search(const Model& m){
    const int actual=std::clamp(int(std::lround(m.delay/h)),0,20);
    std::vector<Params> out;
    for(double r:{100.,1000.,10000.})for(int pred:{std::max(0,actual-2),actual,std::min(20,actual+2)}){
        Params base;base.k=discreteLqr(m,r);base.pred=pred;
        // Host-timestamp models have a wider uncertainty range than local measurements.
        int lo=std::max(0,actual-(m.fmu?2:6)),hi=std::min(24,actual+2);
        for(int d=lo;d<=hi;++d)for(double a:{.9,1.1})for(double b:{.8,1.2})for(double u:{.9,1.1}){++base.linearCases;base.linearPass+=linearScreen(m,base.k,pred,d,a,b,u);}
        auto eval=[&](Params c){
            for(int d:{lo,actual,hi})for(double a:{.9,1.1})for(double s:{.9,1.1})for(double p:{-.02,.02}){
                auto o=simulate(m,c,d,a,s,s,p);++c.cases;c.swingPass+=o.swing;c.regPass+=o.reg;c.trackPass+=o.track;c.cost+=o.cost;
                const auto upper=simulate(m,c,d,a,s,s,p,nullptr,true);c.upperRegPass+=upper.reg;c.upperTrackPass+=upper.track;
            }
            c.cost/=c.cases;out.push_back(c);
        };
        for(int dir:{-1,1}){
            for(double k:{.25,.5,1.,2.}){auto c=base;c.energy=true;c.swingGain=k;c.direction=dir;c.duration=12.;eval(c);}
            for(double k:{.6,1.1,1.6})for(double dur:{8.,12.,16.}){auto c=base;c.swingGain=k;c.swingKd=.5;c.duration=dur;c.direction=dir;eval(c);}
        }
    }
    std::sort(out.begin(),out.end(),[](const auto& a,const auto& b){
        const bool ar=a.linearPass==a.linearCases,br=b.linearPass==b.linearCases;if(ar!=br)return ar>br;
        if(a.trackPass!=b.trackPass)return a.trackPass>b.trackPass;
        if(a.regPass!=b.regPass)return a.regPass>b.regPass;
        if(a.swingPass!=b.swingPass)return a.swingPass>b.swingPass;
        if(a.upperTrackPass!=b.upperTrackPass)return a.upperTrackPass>b.upperTrackPass;
        return a.cost<b.cost;
    });return out;
}
}
