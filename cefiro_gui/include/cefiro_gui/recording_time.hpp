#pragma once
#include <cstdint>
#include <cmath>
#include <algorithm>
namespace cefiro::gui {
inline constexpr double recordingUnitSeconds(int unit) noexcept {
    switch(unit){case 0:return .001;case 2:return 60.;case 3:return 3600.;default:return 1.;}
}
inline bool captureTimeContains(double elapsed,double delay,double end) noexcept {
    return std::isfinite(elapsed) && elapsed>=delay && (end<0. || elapsed<end);
}
inline bool captureIntervalAllows(double elapsed,double last,int interval_ms) noexcept {
    return interval_ms<=0 || elapsed-last+1.e-9>=double(interval_ms)/1000.;
}
}
