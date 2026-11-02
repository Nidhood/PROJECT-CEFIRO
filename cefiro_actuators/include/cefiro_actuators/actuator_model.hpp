#pragma once

#include <array>
#include <cstdint>

namespace cefiro::actuators {

constexpr std::size_t kMotorCount = 4;
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kRpmToRadS = 2.0 * kPi / 60.0;
constexpr double kRadSToRpm = 60.0 / (2.0 * kPi);

// DShot300 encodes a 16-bit packet at 300 kbit/s. This library models
// the packet and the actuator response, not the GPIO signal waveform.
struct Dshot {
  static constexpr std::uint16_t kStop = 0;
  static constexpr std::uint16_t kThrottleMin = 48;
  static constexpr std::uint16_t kThrottleMax = 2047;
  static constexpr double kBitsPerSecond = 300000.0;
  static constexpr double kFrameSeconds = 16.0 / kBitsPerSecond;

  static bool validThrottle(std::uint16_t command) noexcept;
  static std::uint16_t encode(std::uint16_t command,
                              bool request_telemetry = false) noexcept;
  static double fraction(std::uint16_t command) noexcept;
  static std::uint16_t fromFraction(double fraction) noexcept;
};

// Simplified equivalent BLDC / ESC model. Values below are STARTING
// HYPOTHESES ONLY until the actual CEFIRO motors and propellers are measured.
struct MotorParameters {
  double kv_rpm_per_volt{750.0};
  double resistance_ohm{0.30};
  double inductance_h{0.0001};
  double rotor_prop_inertia_kg_m2{0.00004};
  double current_limit_a{35.0};
  double no_load_current_a{0.65};
  double viscous_friction_nm_per_rad_s{0.000004};
  double propeller_diameter_m{0.254};
  double thrust_coefficient_ct{0.10};
  double power_torque_coefficient_cq{0.015};
  double air_density_kg_m3{1.225};
  double esc_time_constant_s{0.025};
  double minimum_spinning_duty{0.035};
  double max_rpm{20000.0};
  bool active_braking{false};

  void validate() const;
  [[nodiscard]] double backEmfConstant() const noexcept;
  [[nodiscard]] double torqueConstant() const noexcept;
  [[nodiscard]] double thrustCoefficientSI() const noexcept;
  [[nodiscard]] double dragCoefficientSI() const noexcept;
};

struct MotorState {
  double rpm{0.0};
  double omega_rad_s{0.0};
  double current_a{0.0};
  double duty{0.0};
  double voltage_v{0.0};
  double electromagnetic_torque_nm{0.0};
  double aerodynamic_drag_nm{0.0};
  double thrust_n{0.0};
};

class MotorModel final {
public:
  explicit MotorModel(MotorParameters parameters = {});
  void reset() noexcept;
  void step(double dt_s, std::uint16_t dshot_command, double bus_voltage_v);
  [[nodiscard]] const MotorState& state() const noexcept { return state_; }
  [[nodiscard]] const MotorParameters& parameters() const noexcept { return params_; }
  // Open-loop steady-state inverse; NOT a closed-loop RPM regulator.
  [[nodiscard]] std::uint16_t estimateDshotForRpm(double rpm,
                                                   double bus_voltage_v) const noexcept;

private:
  MotorParameters params_;
  MotorState state_{};
};

class QuadMotorModel final {
public:
  explicit QuadMotorModel(MotorParameters parameters, double nominal_voltage_v,
                          double battery_resistance_ohm = 0.0);
  void reset() noexcept;
  void step(double dt_s, const std::array<std::uint16_t, kMotorCount>& dshot,
            bool outputs_enabled);
  [[nodiscard]] const std::array<MotorModel, kMotorCount>& motors() const noexcept { return motors_; }
  [[nodiscard]] double busVoltage() const noexcept { return bus_voltage_v_; }
  [[nodiscard]] double totalThrust() const noexcept;
  [[nodiscard]] double reactionYawTorque() const noexcept;
  [[nodiscard]] std::array<std::uint16_t, kMotorCount> estimateDshotForRpm(
      const std::array<double, kMotorCount>& desired_rpm) const noexcept;

private:
  std::array<MotorModel, kMotorCount> motors_;
  double nominal_voltage_v_{};
  double battery_resistance_ohm_{};
  double bus_voltage_v_{};
};

}  // namespace cefiro::actuators
