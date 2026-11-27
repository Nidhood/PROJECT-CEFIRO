#pragma once
#include <charconv>
#include <cmath>
#include <optional>
#include <string_view>
#include <system_error>

namespace cefiro::gui {
// ROS YAML may supply a decimal as a string under a non-C locale.
// Parse the complete scalar without using the process locale.
inline std::optional<double> parseNonnegativeParameter(std::string_view raw) noexcept {
    constexpr std::string_view whitespace = " \t\r\n";
    const auto first = raw.find_first_not_of(whitespace);
    if (first == std::string_view::npos) return std::nullopt;
    raw = raw.substr(first, raw.find_last_not_of(whitespace) - first + 1);
    double value = 0.0;
    const auto result = std::from_chars(raw.data(), raw.data() + raw.size(), value,
                                       std::chars_format::general);
    if (result.ec != std::errc{} || result.ptr != raw.data() + raw.size() ||
        !std::isfinite(value) || value < 0.0) return std::nullopt;
    return value;
}
} // namespace cefiro::gui
