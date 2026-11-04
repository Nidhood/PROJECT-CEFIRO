#include "cefiro_hardware_interface/motor_command.hpp"
#include <algorithm>
#include <cmath>

namespace cefiro::hardware {

bool validManualMotorMap(const ManualMotorMap& map) noexcept {
  return std::isfinite(map.reference_max_rpm) &&
         std::isfinite(map.rpm_deadband) &&
         map.reference_max_rpm > 0.0 &&
         map.rpm_deadband >= 0.0 &&
         map.rpm_deadband < map.reference_max_rpm &&
         map.dshot_min >= 48 && map.dshot_min <= map.dshot_max &&
         map.dshot_max <= 300;
}

std::optional<std::uint16_t> mapGuiRpmToDshot(
    double gui_rpm, const ManualMotorMap& map) noexcept {
  if (!validManualMotorMap(map) || !std::isfinite(gui_rpm) ||
      gui_rpm < 0.0 || gui_rpm > map.reference_max_rpm) {
    return std::nullopt;
  }
  if (gui_rpm <= map.rpm_deadband) return std::uint16_t{0};
  const double fraction = std::clamp(
      (gui_rpm - map.rpm_deadband) /
          (map.reference_max_rpm - map.rpm_deadband),
      0.0, 1.0);
  const auto code = static_cast<std::uint16_t>(std::lround(
      map.dshot_min + (map.dshot_max - map.dshot_min) * fraction));
  return code;
}

std::optional<std::array<std::uint16_t, 4>> mapGuiRpmToDshot(
    const std::array<double, 4>& gui_rpm, const ManualMotorMap& map) noexcept {
  std::array<std::uint16_t, 4> result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    const auto code = mapGuiRpmToDshot(gui_rpm[i], map);
    if (!code) return std::nullopt;
    result[i] = *code;
  }
  return result;
}

}  // namespace cefiro::hardware
