#pragma once
#include "cefiro_gui/models.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>

namespace cefiro::gui {

struct StepMetrics {
    bool valid{false};
    double amplitude_deg{0.0};
    double peak_deg{0.0};
    double peak_time_s{0.0};
    double overshoot_percent{0.0};
    double dead_time_s{std::numeric_limits<double>::quiet_NaN()};
    double rise_time_s{std::numeric_limits<double>::quiet_NaN()};
    double settling_time_s{std::numeric_limits<double>::quiet_NaN()};
    double steady_error_deg{std::numeric_limits<double>::quiet_NaN()};
    double rmse_deg{0.0};
    double mae_deg{0.0};
    double peak_error_deg{0.0};
    int ringing_crossings{0};
};

// Measures a step-like reference change using timestamped angular measurements.
// A metric remains unavailable until its associated event has been observed.
class StepResponseAnalyzer {
public:
    void start(double time_s, double from_ref_deg, double to_ref_deg, double measured_deg) {
        const double step = angleErrorDeg(to_ref_deg, from_ref_deg);
        if (!std::isfinite(time_s) || std::abs(step) < 1.0 || !std::isfinite(measured_deg)) return;
        samples_.clear();
        t0_ = time_s;
        initial_measured_ = measured_deg;
        amplitude_ = step;
        target_ = to_ref_deg;
        enabled_ = true;
    }
    void add(double time_s, double measured_deg) {
        if (!enabled_ || !std::isfinite(time_s) || !std::isfinite(measured_deg)) return;
        if (time_s < t0_) return;
        if (!samples_.empty() && time_s <= samples_.back().t) return;
        const double elapsed = time_s-t0_;
        if (!samples_.empty() && elapsed-samples_.back().t < 0.005) return;
        samples_.push_back({elapsed, measured_deg});
        // 120-second rolling bound. Preserve the beginning of the step and
        // decimate later samples rather than growing memory indefinitely.
        if (samples_.size() > 15000) samples_.pop_back();
    }
    StepMetrics metrics() const {
        StepMetrics m;
        if (!enabled_ || samples_.empty()) return m;
        m.valid = true;
        const double sign = amplitude_ >= 0.0 ? 1.0 : -1.0;
        const double a = std::abs(amplitude_);
        m.amplitude_deg = amplitude_;
        double max_progress = -std::numeric_limits<double>::infinity();
        double t10 = std::numeric_limits<double>::quiet_NaN();
        double t90 = std::numeric_limits<double>::quiet_NaN();
        double last_outside = std::numeric_limits<double>::quiet_NaN();
        const double band = std::max(0.5,0.02*a);
        double sum_sq = 0, sum_abs = 0;
        int previous_side = 0;
        for (const auto& s : samples_) {
            const double progress = sign * angleErrorDeg(s.measured,initial_measured_);
            const double error = angleErrorDeg(target_,s.measured);
            if (progress > max_progress) {
                max_progress = progress;
                m.peak_time_s = s.t;
            }
            m.peak_error_deg = std::max(m.peak_error_deg,std::abs(error));
            sum_sq += error*error;
            sum_abs += std::abs(error);
            if (std::isnan(m.dead_time_s) && progress >= a*0.05) m.dead_time_s = s.t;
            if (std::isnan(t10) && progress >= a*0.10) t10 = s.t;
            if (std::isnan(t90) && progress >= a*0.90) t90 = s.t;
            if (std::abs(error) > band) last_outside = s.t;
            if (progress >= a*0.50) {
                const int side = progress > a+band ? 1 : (progress < a-band ? -1 : 0);
                if (side != 0) {
                    if (previous_side != 0 && previous_side != side) ++m.ringing_crossings;
                    previous_side = side;
                }
            }
        }
        m.peak_deg = wrapDeg(max_progress*sign + initial_measured_);
        m.overshoot_percent = std::max(0.0, (max_progress-a)/a*100.0);
        if (std::isfinite(t10) && std::isfinite(t90) && t90 >= t10) m.rise_time_s = t90-t10;
        const double t_end = samples_.back().t;
        // An observed stable interval of at least 0.5 s is necessary.
        if ((std::isnan(last_outside) && t_end >= 0.5) ||
            (std::isfinite(last_outside) && t_end-last_outside >= 0.5)) {
            m.settling_time_s = std::isnan(last_outside) ? 0.0 : last_outside;
        }
        const double window_start = std::max(0.0, t_end-0.5);
        double error_total=0.0;std::size_t n=0;
        for (const auto& s : samples_) {
            if (s.t >= window_start) { error_total += angleErrorDeg(target_,s.measured); ++n; }
        }
        if (t_end >= 0.5 && n > 1) m.steady_error_deg=error_total/double(n);
        m.rmse_deg=std::sqrt(sum_sq/double(samples_.size()));
        m.mae_deg=sum_abs/double(samples_.size());
        return m;
    }
    bool active() const noexcept { return enabled_; }
    double startTime() const noexcept { return t0_; }
    void reset() { enabled_=false;samples_.clear(); }
private:
    struct Sample { double t; double measured; };
    bool enabled_{false};
    double t0_{0.0},initial_measured_{0.0},amplitude_{0.0},target_{0.0};
    std::deque<Sample> samples_;
};

} // namespace cefiro::gui
