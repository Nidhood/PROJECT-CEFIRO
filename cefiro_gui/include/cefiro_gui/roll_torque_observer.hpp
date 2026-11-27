#pragma once
#include <cmath>
#include <cstdint>

namespace cefiro::gui {
// Read-only, provisional one-axis inverse dynamics model. Not an actuator map.
enum class TorqueQuality { Waiting, InvalidOrientation, OutsideRollModel, InvalidTime, WarmingUp, Valid, Stale };
enum class TorqueSource { None, FmuAhrs, GuiFusion };
struct RollTorqueEstimate {
    TorqueQuality quality{TorqueQuality::Waiting};
    double roll_rad{}, rate_rad_s{}, acceleration_rad_s2{}, gravity_nm{}, damping_nm{}, external_nm{};
    TorqueSource source{TorqueSource::None};
    bool valid() const noexcept { return quality == TorqueQuality::Valid; }
};
class RollTorqueObserver {
public:
    static constexpr double inertia = 0.03378;
    static constexpr double gravity_coefficient = 0.820915;
    static constexpr double damping = 0.020867;
    static constexpr double gravity_offset_deg = 6.5;
    static constexpr double filter_tau_s = 0.12;
    static constexpr double max_pitch_deg = 15.0;
    static constexpr double max_cross_rate_deg_s = 30.0;
    static constexpr double min_dt_s = 0.001;
    static constexpr double max_dt_s = 0.15;
    static constexpr double stale_s = 0.25;
    static constexpr double to_rad = 0.017453292519943295769;

    void reset() noexcept { *this = {}; }
    RollTorqueEstimate update(std::int64_t stamp_ns, bool orientation_valid,
                               double roll_deg, double pitch_deg,
                               double p_deg_s, double q_deg_s, double r_deg_s,
                               TorqueSource source=TorqueSource::FmuAhrs) noexcept {
        if(source!=source_){reset();source_=source;}
        if(source==TorqueSource::None)return reject(TorqueQuality::InvalidOrientation);
        if (!orientation_valid || !std::isfinite(roll_deg) || !std::isfinite(pitch_deg) ||
            !std::isfinite(p_deg_s) || !std::isfinite(q_deg_s) || !std::isfinite(r_deg_s))
            return reject(TorqueQuality::InvalidOrientation);
        if (std::abs(pitch_deg)>max_pitch_deg || std::abs(q_deg_s)>max_cross_rate_deg_s ||
            std::abs(r_deg_s)>max_cross_rate_deg_s)
            return reject(TorqueQuality::OutsideRollModel);
        if (stamp_ns<=0) return reject(TorqueQuality::InvalidTime);
        const double p=p_deg_s*to_rad;
        if (!have_previous_) {
            seed(stamp_ns,p);
            return {TorqueQuality::WarmingUp};
        }
        if (stamp_ns<=previous_ns_) return reject(TorqueQuality::InvalidTime);
        const double dt=static_cast<double>(stamp_ns-previous_ns_)*1.e-9;
        if (dt<min_dt_s || dt>max_dt_s) {
            reset(); source_=source; seed(stamp_ns,p);
            return {TorqueQuality::InvalidTime};
        }
        const double raw=(p-previous_p_)/dt;
        seed(stamp_ns,p);
        if (!std::isfinite(raw)) return reject(TorqueQuality::InvalidOrientation);
        if (!have_filter_) {
            filtered_acceleration_=raw;
            have_filter_=true;
            return {TorqueQuality::WarmingUp};
        }
        const double alpha=-std::expm1(-dt/filter_tau_s);
        filtered_acceleration_+=alpha*(raw-filtered_acceleration_);
        const double phi=roll_deg*to_rad;
        const double gravity=gravity_coefficient*std::sin(phi-gravity_offset_deg*to_rad);
        const double friction=damping*p;
        const double external=inertia*filtered_acceleration_-gravity+friction;
        if (!std::isfinite(external)) return reject(TorqueQuality::InvalidOrientation);
        return {TorqueQuality::Valid,phi,p,filtered_acceleration_,gravity,friction,external,source};
    }
private:
    RollTorqueEstimate reject(TorqueQuality quality) noexcept { reset(); return {quality}; }
    void seed(std::int64_t stamp_ns,double p) noexcept { previous_ns_=stamp_ns;previous_p_=p;have_previous_=true; }
    TorqueSource source_{TorqueSource::None};
    std::int64_t previous_ns_{};
    double previous_p_{},filtered_acceleration_{};
    bool have_previous_{},have_filter_{};
};
}
