#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace cefiro::hardware {

// The CEFIRO FMU V3 currently has NO measured-RPM feedback or closed-loop RPM
// actuator. This mapping is only an explicit, open-loop GUI-command adapter.
struct ManualMotorMap {
  double reference_max_rpm{12000.0};
  double rpm_deadband{25.0};
  std::uint16_t dshot_min{48};
  std::uint16_t dshot_max{300};
};

bool validManualMotorMap(const ManualMotorMap& map) noexcept;
std::optional<std::uint16_t> mapGuiRpmToDshot(
    double gui_rpm, const ManualMotorMap& map) noexcept;
std::optional<std::array<std::uint16_t, 4>> mapGuiRpmToDshot(
    const std::array<double, 4>& gui_rpm, const ManualMotorMap& map) noexcept;

}  // namespace cefiro::hardware
