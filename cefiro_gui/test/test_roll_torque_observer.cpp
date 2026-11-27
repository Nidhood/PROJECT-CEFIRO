#include "cefiro_gui/roll_torque_observer.hpp"
#include "cefiro_gui/recording_join.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using namespace cefiro::gui;
static constexpr std::int64_t t0=1700000000000000000LL, step=40000000LL;
int main(){
    RollTorqueObserver o;
    auto update=[&](int i,double roll,double pitch,double p,double q=0.,double r=0.,bool valid=true){
        return o.update(t0+i*step,valid,roll,pitch,p,q,r);
    };
    assert(update(0,-173.5,-8,0).quality==TorqueQuality::WarmingUp);
    assert(update(1,-173.5,-8,0).quality==TorqueQuality::WarmingUp);
    auto e=update(2,-173.5,-8,0);
    assert(e.valid() && std::abs(e.external_nm)<1e-12);
    e=update(3,-83.5,0,0);
    assert(e.valid() && std::abs(e.gravity_nm+RollTorqueObserver::gravity_coefficient)<1e-12);
    assert(std::abs(e.external_nm-RollTorqueObserver::gravity_coefficient)<1e-12);
    assert(update(4,0,16,0).quality==TorqueQuality::OutsideRollModel);
    assert(update(5,0,0,0,31).quality==TorqueQuality::OutsideRollModel);
    assert(update(6,0,0,0,0,31).quality==TorqueQuality::OutsideRollModel);
    assert(update(7,0,0,0,0,0,false).quality==TorqueQuality::InvalidOrientation);
    assert(update(8,0,0,std::numeric_limits<double>::quiet_NaN()).quality==TorqueQuality::InvalidOrientation);
    assert(update(9,0,0,0).quality==TorqueQuality::WarmingUp);
    assert(update(9,0,0,0).quality==TorqueQuality::InvalidTime);
    assert(update(10,0,0,0).quality==TorqueQuality::WarmingUp);
    assert(update(20,0,0,0).quality==TorqueQuality::InvalidTime);
    assert(update(21,0,0,4).quality==TorqueQuality::WarmingUp);
    e=update(22,0,0,8);
    assert(e.valid() && std::abs(e.acceleration_rad_s2-100*RollTorqueObserver::to_rad)<1e-10);
    assert(std::abs(e.external_nm-(RollTorqueObserver::inertia*e.acceleration_rad_s2-e.gravity_nm+e.damping_nm))<1e-12);
    // Signed damping and the roll wrap around +/-180 degrees remain continuous.
    o.reset();update(0,179.99,0,-10);update(1,179.99,0,-10);e=update(2,179.99,0,-10);
    auto e2=update(3,-180.01,0,-10);
    assert(e.valid() && e2.valid() && e.damping_nm<0 && std::abs(e.external_nm-e2.external_nm)<1e-10);
    // Co-sampled strict orientation can invalidate only the added estimate.
    Telemetry s;s.from_real=true;s.roll_torque=e;
    IdentificationFields id;id.orientation_valid=false;id.apply(s);
    assert(!s.roll_torque.valid() && !s.orientation_valid);
    o.reset();assert(update(0,0,0,0).quality==TorqueQuality::WarmingUp);
    // Fallback is read-only and explicitly identified; changing sources resets derivative.
    o.reset();
    for(int i=0;i<3;i++)e=o.update(t0+i*step,true,-173.5,-7.,0.,0.,0.,TorqueSource::GuiFusion);
    assert(e.valid() && e.source==TorqueSource::GuiFusion);
    Telemetry fallback;fallback.from_real=true;fallback.display_attitude.valid=true;fallback.roll_torque=e;
    id.apply(fallback);assert(fallback.roll_torque.valid()&&!fallback.orientation_valid);
    assert(o.update(t0+3*step,true,-173.5,-7.,0.,0.,0.,TorqueSource::FmuAhrs).quality==TorqueQuality::WarmingUp);
    assert(!o.update(t0+4*step,false,0,0,0,0,0,TorqueSource::None).valid());
}
