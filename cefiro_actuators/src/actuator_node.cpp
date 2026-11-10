#include "cefiro_actuators/actuator_model.hpp"

#include <cefiro_msgs/msg/motor_dshot.hpp>
#include <cefiro_msgs/msg/motor_rpm.hpp>
#include <cefiro_msgs/msg/motor_telemetry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace cefiro::actuators
{

  class ActuatorNode final : public rclcpp::Node
  {
  public:
    ActuatorNode() : rclcpp::Node("cefiro_actuators")
    {
      MotorParameters parameters;
      parameters.kv_rpm_per_volt = declare_parameter("motor.kv_rpm_per_volt", 750.0);
      parameters.resistance_ohm = declare_parameter("motor.resistance_ohm", 0.30);
      parameters.inductance_h = declare_parameter("motor.inductance_h", 0.0001);
      parameters.rotor_prop_inertia_kg_m2 = declare_parameter("motor.rotor_prop_inertia_kg_m2", 0.00004);
      parameters.current_limit_a = declare_parameter("motor.current_limit_a", 35.0);
      parameters.no_load_current_a = declare_parameter("motor.no_load_current_a", 0.65);
      parameters.viscous_friction_nm_per_rad_s =
          declare_parameter("motor.viscous_friction_nm_per_rad_s", 0.000004);
      parameters.propeller_diameter_m = declare_parameter("propeller.diameter_m", 0.254);
      parameters.thrust_coefficient_ct = declare_parameter("propeller.ct", 0.10);
      parameters.power_torque_coefficient_cq = declare_parameter("propeller.cq", 0.015);
      parameters.air_density_kg_m3 = declare_parameter("environment.air_density_kg_m3", 1.225);
      parameters.esc_time_constant_s = declare_parameter("esc.time_constant_s", 0.025);
      parameters.minimum_spinning_duty = declare_parameter("esc.minimum_spinning_duty", 0.035);
      parameters.max_rpm = declare_parameter("motor.max_rpm", 20000.0);
      parameters.active_braking = declare_parameter("esc.active_braking", false);
      const double voltage = declare_parameter("battery.nominal_voltage_v", 12.63);
      const double battery_resistance = declare_parameter("battery.internal_resistance_ohm", 0.03);
      watchdog_s_ = declare_parameter("command_watchdog_s", 0.5);
      const double dynamics_hz = declare_parameter("dynamics_rate_hz", 1000.0);
      const double telemetry_hz = declare_parameter("telemetry_rate_hz", 100.0);
      const auto frame_id = declare_parameter("frame_id", std::string("drone_body_link"));
      frame_id_ = frame_id;

      if (!(std::isfinite(watchdog_s_) && watchdog_s_ > 0.0) ||
          !(std::isfinite(dynamics_hz) && dynamics_hz >= 50.0 && dynamics_hz <= 4000.0) ||
          !(std::isfinite(telemetry_hz) && telemetry_hz >= 1.0 && telemetry_hz <= dynamics_hz))
      {
        throw std::invalid_argument("Invalid dynamics/telemetry frequency or command watchdog");
      }
      plant_ = std::make_unique<QuadMotorModel>(parameters, voltage, battery_resistance);
      max_rpm_ = parameters.max_rpm;
      nominal_voltage_ = voltage;

      const auto sub_qos = rclcpp::QoS(10).reliable();
      sub_rpm_ = create_subscription<cefiro_msgs::msg::MotorRpm>(
          "~/target_rpm", sub_qos,
          [this](cefiro_msgs::msg::MotorRpm::ConstSharedPtr msg)
          { onRpm(*msg); });
      sub_dshot_ = create_subscription<cefiro_msgs::msg::MotorDshot>(
          "~/dshot_throttle", sub_qos,
          [this](cefiro_msgs::msg::MotorDshot::ConstSharedPtr msg)
          { onDshot(*msg); });
      sub_arm_ = create_subscription<std_msgs::msg::Bool>(
          "~/armed", sub_qos,
          [this](std_msgs::msg::Bool::ConstSharedPtr msg)
          { onArm(msg->data); });
      pub_rpm_ = create_publisher<cefiro_msgs::msg::MotorRpm>("~/measured_rpm", 10);
      pub_state_ = create_publisher<cefiro_msgs::msg::MotorTelemetry>("~/telemetry", 10);
      for (std::size_t i = 0; i < kMotorCount; ++i)
      {
        const std::string topic = "/cefiro/gz/motor_" + std::to_string(i + 1);
        gz_publishers_[i] = create_publisher<geometry_msgs::msg::Vector3>(topic, 10);
      }

      // All callbacks run in the same executor: no cross-thread shared-state race.
      using namespace std::chrono;
      const auto dynamics_period = duration_cast<nanoseconds>(duration<double>(1.0 / dynamics_hz));
      const auto telemetry_period = duration_cast<nanoseconds>(duration<double>(1.0 / telemetry_hz));
      last_step_ = steady_clock::now();
      dynamics_timer_ = create_wall_timer(dynamics_period, [this]
                                          { step(); });
      telemetry_timer_ = create_wall_timer(telemetry_period, [this]
                                           { publish(); });

      RCLCPP_INFO(get_logger(),
                  "Standalone actuator model ready; 4 channels; DShot300 modeled logically; "
                  "Kv=%.1f rpm/V; voltage=%.2f V; physics=%.0f Hz; outputs DISARMED",
                  parameters.kv_rpm_per_volt, voltage, dynamics_hz);
      RCLCPP_WARN(get_logger(),
                  "Motor/propeller coefficients are provisional. RPM commands use open-loop "
                  "feedforward; DShot is not RPM feedback control.");
    }

  private:
    using SteadyClock = std::chrono::steady_clock;
    using SteadyTime = SteadyClock::time_point;
    enum class CommandSource : std::uint8_t
    {
      None = 0,
      Rpm = 1,
      Dshot = 2
    };

    void onRpm(const cefiro_msgs::msg::MotorRpm &msg)
    {
      std::array<double, kMotorCount> requested{};
      for (std::size_t i = 0; i < kMotorCount; ++i)
      {
        if (!std::isfinite(msg.data[i]) || msg.data[i] < 0.0 || msg.data[i] > max_rpm_)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Rejected RPM array: all four entries must be finite and in [0,max_rpm]");
          return;
        }
        requested[i] = msg.data[i];
      }
      requested_rpm_ = requested;
      requested_dshot_ = plant_->estimateDshotForRpm(requested_rpm_);
      source_ = CommandSource::Rpm;
      command_received_ = true;
      last_command_ = SteadyClock::now();
    }

    void onDshot(const cefiro_msgs::msg::MotorDshot &msg)
    {
      for (std::size_t i = 0; i < kMotorCount; ++i)
      {
        if (!Dshot::validThrottle(msg.data[i]))
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Rejected DShot array: use 0 or throttle 48..2047; 1..47 are reserved");
          return;
        }
      }
      for (std::size_t i = 0; i < kMotorCount; ++i)
      {
        requested_dshot_[i] = msg.data[i];
        requested_rpm_[i] = 0.0; // RPM setpoint is unknown for raw DShot commands.
      }
      source_ = CommandSource::Dshot;
      command_received_ = true;
      last_command_ = SteadyClock::now();
    }

    void onArm(bool requested)
    {
      armed_ = requested;
      if (!armed_)
      {
        requested_dshot_.fill(Dshot::kStop);
        requested_rpm_.fill(0.0);
        command_received_ = false;
        source_ = CommandSource::None;
        RCLCPP_INFO(get_logger(), "DISARMED: commands cleared; rotors coast to rest");
      }
      else
      {
        // Arming itself never energizes motors. Require a new fresh command.
        command_received_ = false;
        requested_dshot_.fill(Dshot::kStop);
        requested_rpm_.fill(0.0);
        source_ = CommandSource::None;
        RCLCPP_INFO(get_logger(), "ARMED: awaiting a fresh four-channel command");
      }
    }

    void step()
    {
      const auto now = SteadyClock::now();
      const double elapsed = std::chrono::duration<double>(now - last_step_).count();
      last_step_ = now;
      // Cap the integration catch-up. A stalled ROS executor should not create a huge step.
      const double dt = std::clamp(elapsed, 0.0, 0.02);
      const bool fresh = command_received_ &&
                         std::chrono::duration<double>(now - last_command_).count() <= watchdog_s_;
      watchdog_ok_ = fresh;
      const bool enabled = armed_ && fresh;
      std::array<std::uint16_t, kMotorCount> applied{};
      if (enabled)
        applied = requested_dshot_;
      applied_dshot_ = applied;

      // Small bounded substeps keep the electrical/mechanical integration stable.
      const int steps = std::max(1, static_cast<int>(std::ceil(dt / 0.0005)));
      const double h = dt / steps;
      for (int k = 0; k < steps; ++k)
        plant_->step(h, applied, enabled);
    }

    void publish()
    {
      cefiro_msgs::msg::MotorTelemetry state;
      state.header.stamp = get_clock()->now();
      state.header.frame_id = frame_id_;
      state.requested_rpm = requested_rpm_;
      state.bus_voltage_v = plant_->busVoltage();
      state.total_thrust_n = plant_->totalThrust();
      state.total_reaction_yaw_nm = plant_->reactionYawTorque();
      state.armed = armed_;
      state.watchdog_ok = watchdog_ok_;
      state.command_source = static_cast<std::uint8_t>(source_);

      cefiro_msgs::msg::MotorRpm rpm;
      constexpr std::array<double, kMotorCount> reaction_sign{1.0, -1.0, 1.0, -1.0};
      for (std::size_t i = 0; i < kMotorCount; ++i)
      {
        const auto &motor = plant_->motors()[i].state();
        rpm.data[i] = motor.rpm;
        state.measured_rpm[i] = motor.rpm;
        state.dshot_value[i] = applied_dshot_[i];
        state.dshot_packet[i] = Dshot::encode(applied_dshot_[i]);
        state.throttle_fraction[i] = motor.duty;
        state.thrust_n[i] = motor.thrust_n;
        state.aerodynamic_torque_nm[i] = motor.aerodynamic_drag_nm;
        state.reaction_torque_nm[i] = reaction_sign[i] * motor.aerodynamic_drag_nm;
        state.electromagnetic_torque_nm[i] = motor.electromagnetic_torque_nm;
        state.current_a[i] = motor.current_a;
        state.applied_voltage_v[i] = motor.voltage_v;
        geometry_msgs::msg::Vector3 output;
        // Physical rotor speed, not reduced to visual animation speed.
        // The Gazebo plugin applies a documented visual slowdown.
        output.x = motor.omega_rad_s;
        output.y = motor.thrust_n;
        output.z = state.reaction_torque_nm[i];
        gz_publishers_[i]->publish(output);
      }
      pub_rpm_->publish(rpm);
      pub_state_->publish(state);
    }

    std::unique_ptr<QuadMotorModel> plant_;
    double watchdog_s_{0.5};
    double max_rpm_{20000.0};
    double nominal_voltage_{12.63};
    std::string frame_id_;
    bool armed_{false};
    bool command_received_{false};
    bool watchdog_ok_{false};
    CommandSource source_{CommandSource::None};
    SteadyTime last_step_{};
    SteadyTime last_command_{};
    std::array<double, kMotorCount> requested_rpm_{};
    std::array<std::uint16_t, kMotorCount> requested_dshot_{};
    std::array<std::uint16_t, kMotorCount> applied_dshot_{};
    rclcpp::Subscription<cefiro_msgs::msg::MotorRpm>::SharedPtr sub_rpm_;
    rclcpp::Subscription<cefiro_msgs::msg::MotorDshot>::SharedPtr sub_dshot_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_arm_;
    rclcpp::Publisher<cefiro_msgs::msg::MotorRpm>::SharedPtr pub_rpm_;
    rclcpp::Publisher<cefiro_msgs::msg::MotorTelemetry>::SharedPtr pub_state_;
    std::array<rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr, kMotorCount>
        gz_publishers_{};
    rclcpp::TimerBase::SharedPtr dynamics_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
  };

} // namespace cefiro::actuators

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<cefiro::actuators::ActuatorNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("cefiro_actuators"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
