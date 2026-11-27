#pragma once
#include <algorithm>
#include <cmath>

namespace cefiro::gui {
// Work in Qt logical pixels; Qt already compensates for monitor DPI.
// Quantization limits stylesheet repolishing during interactive resize.
inline double automaticUiScale(int width, int height) noexcept {
    const double ratio = std::min(width / 1280.0, height / 740.0);
    return std::clamp(std::round(ratio * 10.0) / 10.0, 1.0, 2.0);
}
inline double effectiveUiScale(int width, int height, int manual_percent) noexcept {
    return manual_percent > 0 ? std::clamp(manual_percent / 100.0, 1.0, 2.0)
                              : automaticUiScale(width, height);
}
inline int scaledPixels(int pixels, double scale) noexcept {
    return static_cast<int>(std::lround(pixels * scale));
}
inline int layoutWidth(int width, double scale) noexcept {
    return static_cast<int>(std::floor(width / std::max(1.0, scale)));
}
} // namespace cefiro::gui
