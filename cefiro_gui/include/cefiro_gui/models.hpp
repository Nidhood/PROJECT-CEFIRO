#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace cefiro::gui {
constexpr double pi = 3.14159265358979323846;
constexpr double rad2deg = 180.0 / pi;
constexpr double deg2rad = pi / 180.0;
using Vec3 = std::array<double, 3>; // roll, pitch, yaw
inline double clampFinite(double v, double lo, double hi) {
    return std::isfinite(v) ? std::clamp(v, lo, hi) : 0.0;
}
inline double wrapDeg(double angle) {
    double a = std::remainder(angle, 360.0);
    return a == -180.0 ? 180.0 : a;
}
inline double angleErrorDeg(double reference, double measured) {
    return wrapDeg(reference - measured);
}
struct Quaternion {
    double w{1.0}, x{0.0}, y{0.0}, z{0.0};
    bool normalize() {
        const double n2 = w*w + x*x + y*y + z*z;
        if (!std::isfinite(n2) || n2 < 1.e-12) return false;
        const double inv = 1.0 / std::sqrt(n2);
        w*=inv; x*=inv; y*=inv; z*=inv; return true;
    }
    Vec3 eulerDeg() const {
        const double sinp = std::clamp(2.0*(w*y - z*x), -1.0, 1.0);
        return {std::atan2(2.0*(w*x+y*z), 1.0-2.0*(x*x+y*y))*rad2deg,
                std::asin(sinp)*rad2deg,
                std::atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))*rad2deg};
    }
};
struct Telemetry {
    double ros_time_s{0.0};
    std::int64_t sequence{0};
    Quaternion orientation{};
    bool orientation_valid{false};
    Vec3 attitude_deg{};
    Vec3 angular_rate_deg_s{};
    Vec3 accel_m_s2{};
    Vec3 mag_uT{};
    Vec3 setpoint_deg{};
    bool magnetometer_valid{false};
    bool barometer_valid{false};
    double pressure_hPa{0.0};
    double relative_alt_m{0.0};
    std::string imu_frame;
    std::string mag_frame;
    std::string baro_frame;
};
struct ErrorStats {
    std::uint64_t count{0};
    std::array<double,3> sum_sq{};
    std::array<double,3> abs_sum{};
    std::array<double,3> peak{};
    void reset() { *this = {}; }
    void add(const Telemetry& s) {
        if(!s.orientation_valid) return;
        ++count;
        for(int i=0;i<3;++i) {
            const double e = angleErrorDeg(s.setpoint_deg[i],s.attitude_deg[i]);
            sum_sq[i] += e*e; abs_sum[i] += std::abs(e);
            peak[i]=std::max(peak[i],std::abs(e));
        }
    }
    double rmse(int i) const {return count ? std::sqrt(sum_sq[i]/double(count)) : 0.0;}
    double mae(int i) const {return count ? abs_sum[i]/double(count) : 0.0;}
};
}
