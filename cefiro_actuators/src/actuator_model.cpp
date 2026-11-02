#include "cefiro_actuators/actuator_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cefiro::actuators {

namespace {
double clampFinite(double x, double lo, double hi) noexcept {
  return std::isfinite(x) ? std::clamp(x, lo, hi) : lo;
}
}  // namespace

bool Dshot::validThrottle(std::uint16_t command) noexcept {
  return command == kStop || (command >= kThrottleMin && command <= kThrottleMax);
}

std::uint16_t Dshot::encode(std::uint16_t command, bool request_telemetry) noexcept {
  if (!validThrottle(command)) return 0;
  const std::uint16_t payload = static_cast<std::uint16_t>((command << 1U) |
                                                           (request_telemetry ? 1U : 0U));
  const std::uint16_t crc = static_cast<std::uint16_t>(
      (payload ^ (payload >> 4U) ^ (payload >> 8U)) & 0x0FU);
  return static_cast<std::uint16_t>((payload << 4U) | crc);
}

double Dshot::fraction(std::uint16_t command) noexcept {
  if (command < kThrottleMin || command > kThrottleMax) return 0.0;
  return static_cast<double>(command - kThrottleMin) /
         static_cast<double>(kThrottleMax - kThrottleMin);
}

std::uint16_t Dshot::fromFraction(double fraction) noexcept {
  if (!std::isfinite(fraction) || fraction <= 0.0) return kStop;
  const double normalized = std::clamp(fraction, 0.0, 1.0);
  return static_cast<std::uint16_t>(kThrottleMin +
      std::lround(normalized * (kThrottleMax - kThrottleMin)));
}

void MotorParameters::validate() const {
  auto positive = [](double value, const char* name) {
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument(name);
  };
  positive(kv_rpm_per_volt, "kv_rpm_per_volt must be positive");
  positive(resistance_ohm, "resistance_ohm must be positive");
  positive(inductance_h, "inductance_h must be positive");
  positive(rotor_prop_inertia_kg_m2, "rotor_prop_inertia_kg_m2 must be positive");
  positive(current_limit_a, "current_limit_a must be positive");
  positive(propeller_diameter_m, "propeller_diameter_m must be positive");
  positive(air_density_kg_m3, "air_density_kg_m3 must be positive");
  positive(max_rpm, "max_rpm must be positive");
  if (!(std::isfinite(no_load_current_a) && no_load_current_a >= 0.0) ||
      !(std::isfinite(viscous_friction_nm_per_rad_s) && viscous_friction_nm_per_rad_s >= 0.0) ||
      !(std::isfinite(thrust_coefficient_ct) && thrust_coefficient_ct >= 0.0) ||
      !(std::isfinite(power_torque_coefficient_cq) && power_torque_coefficient_cq >= 0.0) ||
      !(std::isfinite(esc_time_constant_s) && esc_time_constant_s >= 0.0) ||
      !(std::isfinite(minimum_spinning_duty) && minimum_spinning_duty >= 0.0 &&
        minimum_spinning_duty <= 1.0)) {
    throw std::invalid_argument("Invalid non-negative motor/propeller parameter");
  }
}

double MotorParameters::backEmfConstant() const noexcept {
  return 1.0 / (kv_rpm_per_volt * kRpmToRadS);
}

double MotorParameters::torqueConstant() const noexcept {
  // Effective SI Kt = Ke approximation for a brushless commutated motor.
  return backEmfConstant();
}

double MotorParameters::thrustCoefficientSI() const noexcept {
  return thrust_coefficient_ct * air_density_kg_m3 *
         std::pow(propeller_diameter_m, 4) / (4.0 * kPi * kPi);
}

double MotorParameters::dragCoefficientSI() const noexcept {
  return power_torque_coefficient_cq * air_density_kg_m3 *
         std::pow(propeller_diameter_m, 5) / (4.0 * kPi * kPi);
}

MotorModel::MotorModel(MotorParameters parameters) : params_(parameters) {
  params_.validate();
}

void MotorModel::reset() noexcept { state_ = {}; }

std::uint16_t MotorModel::estimateDshotForRpm(double rpm,
                                              double bus_voltage_v) const noexcept {
  if (!std::isfinite(rpm) || rpm <= 0.0 ||
      !std::isfinite(bus_voltage_v) || bus_voltage_v <= 0.0) return Dshot::kStop;
  const double omega = std::min(rpm, params_.max_rpm) * kRpmToRadS;
  const double drag = params_.dragCoefficientSI() * omega * omega;
  const double friction = params_.viscous_friction_nm_per_rad_s * omega +
                          params_.torqueConstant() * params_.no_load_current_a;
  const double current = (drag + friction) / params_.torqueConstant();
  const double voltage = params_.backEmfConstant() * omega +
                         params_.resistance_ohm * current;
  const double duty = voltage / bus_voltage_v;
  // Commands below the ESC spin threshold cannot create useful torque.
  if (duty < params_.minimum_spinning_duty) return Dshot::kStop;
  return Dshot::fromFraction(duty);
}

void MotorModel::step(double dt_s, std::uint16_t dshot_command,
                      double bus_voltage_v) {
  if (!(std::isfinite(dt_s) && dt_s > 0.0)) return;
  if (!Dshot::validThrottle(dshot_command)) dshot_command = Dshot::kStop;
  const double requested = Dshot::fraction(dshot_command);
  const double effective_target = requested >= params_.minimum_spinning_duty ? requested : 0.0;
  const double tau = params_.esc_time_constant_s;
  const double alpha = tau > 0.0 ? -std::expm1(-dt_s / tau) : 1.0;
  state_.duty = clampFinite(state_.duty + alpha * (effective_target - state_.duty), 0.0, 1.0);
  const double vbus = std::max(0.0, bus_voltage_v);
  state_.voltage_v = state_.duty * vbus;

  const double omega = state_.omega_rad_s;
  const double back_emf = params_.backEmfConstant() * omega;
  const double i_equilibrium = (state_.voltage_v - back_emf) / params_.resistance_ohm;
  const double i_alpha = -std::expm1(-params_.resistance_ohm * dt_s / params_.inductance_h);
  state_.current_a += (i_equilibrium - state_.current_a) * i_alpha;
  const double i_min = params_.active_braking ? -params_.current_limit_a : 0.0;
  state_.current_a = clampFinite(state_.current_a, i_min, params_.current_limit_a);
  // When throttle is zero, assume freewheeling/coasting rather than regenerative braking.
  if (!params_.active_braking && state_.duty < 1e-5) state_.current_a = 0.0;

  state_.electromagnetic_torque_nm = params_.torqueConstant() * state_.current_a;
  const double prop_drag = params_.dragCoefficientSI() * omega * omega;
  const double friction = params_.viscous_friction_nm_per_rad_s * omega +
      params_.torqueConstant() * params_.no_load_current_a * std::tanh(omega / 10.0);
  const double acceleration = (state_.electromagnetic_torque_nm - prop_drag - friction) /
                              params_.rotor_prop_inertia_kg_m2;
  const double omega_max = params_.max_rpm * kRpmToRadS;
  state_.omega_rad_s = clampFinite(omega + acceleration * dt_s, 0.0, omega_max);
  state_.rpm = state_.omega_rad_s * kRadSToRpm;
  state_.aerodynamic_drag_nm = params_.dragCoefficientSI() *
                                state_.omega_rad_s * state_.omega_rad_s;
  state_.thrust_n = params_.thrustCoefficientSI() *
                    state_.omega_rad_s * state_.omega_rad_s;
}

QuadMotorModel::QuadMotorModel(MotorParameters parameters, double nominal_voltage_v,
                               double battery_resistance_ohm)
    : motors_{MotorModel(parameters), MotorModel(parameters),
              MotorModel(parameters), MotorModel(parameters)},
      nominal_voltage_v_(nominal_voltage_v),
      battery_resistance_ohm_(battery_resistance_ohm),
      bus_voltage_v_(nominal_voltage_v) {
  if (!std::isfinite(nominal_voltage_v) || nominal_voltage_v <= 0.0 ||
      !std::isfinite(battery_resistance_ohm) || battery_resistance_ohm < 0.0) {
    throw std::invalid_argument("Invalid battery voltage or resistance");
  }
}

void QuadMotorModel::reset() noexcept {
  for (auto& motor : motors_) motor.reset();
  bus_voltage_v_ = nominal_voltage_v_;
}

void QuadMotorModel::step(double dt_s,
                          const std::array<std::uint16_t, kMotorCount>& dshot,
                          bool outputs_enabled) {
  if (!(std::isfinite(dt_s) && dt_s > 0.0)) return;
  double current_sum = 0.0;
  for (const auto& motor : motors_) {
    current_sum += std::max(0.0, motor.state().current_a);
  }
  bus_voltage_v_ = std::max(0.0, nominal_voltage_v_ -
                               battery_resistance_ohm_ * current_sum);
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    motors_[i].step(dt_s, outputs_enabled ? dshot[i] : Dshot::kStop, bus_voltage_v_);
  }
}

double QuadMotorModel::totalThrust() const noexcept {
  double value = 0.0;
  for (const auto& motor : motors_) value += motor.state().thrust_n;
  return value;
}

double QuadMotorModel::reactionYawTorque() const noexcept {
  // CAD rotor order: CW, CCW, CW, CCW. Reaction acts opposite rotor rotation.
  constexpr std::array<double, kMotorCount> kReactionSigns{+1.0, -1.0, +1.0, -1.0};
  double torque = 0.0;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    torque += kReactionSigns[i] * motors_[i].state().aerodynamic_drag_nm;
  }
  return torque;
}

std::array<std::uint16_t, kMotorCount> QuadMotorModel::estimateDshotForRpm(
    const std::array<double, kMotorCount>& desired_rpm) const noexcept {
  std::array<std::uint16_t, kMotorCount> result{};
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    result[i] = motors_[i].estimateDshotForRpm(desired_rpm[i], bus_voltage_v_);
  }
  return result;
}

}  // namespace cefiro::actuators
