#pragma once
#include "cefiro_gui/accel_calibration.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace cefiro::gui {
// Physical FLU body convention used by the existing 3D view: +X nose,
// +Y left, +Z top. These are specific-force signs, not gravity arrows.
inline constexpr std::array<unsigned,6> kWizardFaces{4,5,0,1,2,3};
inline int detectedAccelFace(const CalVec& a) {
    for(double x:a) if(!std::isfinite(x)) return -1;
    const double norm=std::hypot(a[0],a[1],a[2]);
    if(norm<7.5 || norm>12.5) return -1;
    unsigned axis=0;
    for(unsigned j=1;j<3;++j) if(std::abs(a[j])>std::abs(a[axis])) axis=j;
    if(std::abs(a[axis])<7.0) return -1;
    for(unsigned j=0;j<3;++j) if(j!=axis && std::abs(a[j])>2.0) return -1;
    return int(2*axis+(a[axis]<0.0?1:0));
}
class AccelCalibrationWizard {
public:
    enum class State { Waiting, WrongFace, Motion, Stabilizing, Ready, Captured, Complete, FitFailed, InvalidTime };
    static constexpr double hold_seconds=0.6;
    static constexpr double stable_seconds=1.8;
    unsigned step() const { return step_; }
    unsigned count() const { return stats_.n; }
    double progress() const { return progress_; }
    const std::optional<AccelCalibration>& result() const { return result_; }
    int expectedFace() const { return step_<6?int(kWizardFaces[step_]):-1; }
    void resetWindow() { stats_={}; first_=-1.; green_=-1.; progress_=0.; }
    void interrupt() { resetWindow(); last_=-1.; }
    State update(const CalVec& a,const CalVec& gyro,double stamp) {
        if(step_==6) return result_?State::Complete:State::FitFailed;
        if(!std::isfinite(stamp)||stamp<=0.) { interrupt(); return State::InvalidTime; }
        if(last_>=0. && (stamp<=last_ || stamp-last_>0.25)) {
            resetWindow(); last_=stamp; return State::InvalidTime;
        }
        last_=stamp;
        const auto face=unsigned(expectedFace());
        if(detectedAccelFace(a)!=int(face)) { resetWindow(); return State::WrongFace; }
        if(!stats_.add(a,gyro,face)) { resetWindow(); return State::Motion; }
        if(stats_.n==1) first_=stamp;
        const double span=stamp-first_;
        // Do not indefinitely average across a moving or noisy interval.
        if(span>4.5 && !stats_.stable()) { resetWindow(); return State::Motion; }
        progress_=0.75*std::min({1.,double(stats_.n)/AccelFaceCapture::required,span/stable_seconds});
        if(span<stable_seconds || !stats_.stable()) { green_=-1.; return State::Stabilizing; }
        if(green_<0.) green_=stamp;
        progress_=0.75+0.25*std::clamp((stamp-green_)/hold_seconds,0.,1.);
        if(stamp-green_<hold_seconds) return State::Ready;
        faces_[face]=stats_.mean;
        ++step_; resetWindow();
        if(step_==6) { result_=fitAccelCalibration(faces_); return result_?State::Complete:State::FitFailed; }
        return State::Captured;
    }
private:
    unsigned step_{};
    std::array<CalVec,6> faces_{};
    AccelFaceCapture stats_{};
    std::optional<AccelCalibration> result_{};
    double last_{-1.},first_{-1.},green_{-1.},progress_{};
};
}
