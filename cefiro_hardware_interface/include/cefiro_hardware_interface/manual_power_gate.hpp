#pragma once
#include <string_view>

namespace cefiro::hardware {
// Firmware BAT flags and PWR fields are authoritative. Never infer motor
// permission from an ARM ACK, a host DShot cap, or the USB power supply.
struct ManualPowerGate {
  bool battery_valid{}, battery_adc_fresh{}, battery_bench{};
  bool power_adc_fresh{}, power_bench{}, bench_denial{};
  [[nodiscard]] bool benchZeroOnly() const noexcept {
    return battery_bench || power_bench || bench_denial;
  }
  [[nodiscard]] std::string_view reason(bool battery_fresh, bool power_fresh) const noexcept {
    if (benchZeroOnly()) return "BENCH_ZERO_ONLY";
    if (!battery_fresh || !power_fresh) return "BAT/PWR telemetry missing or stale";
    if (!battery_adc_fresh || !power_adc_fresh) return "Battery ADC not fresh";
    if (!battery_valid) return "Battery voltage invalid";
    return {};
  }
};
}  // namespace cefiro::hardware
