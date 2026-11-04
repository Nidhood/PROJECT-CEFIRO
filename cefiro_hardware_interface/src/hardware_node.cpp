#include "cefiro_hardware_interface/protocol.hpp"
#include "cefiro_hardware_interface/motor_command.hpp"
#include "cefiro_hardware_interface/manual_keepalive.hpp"
#include "cefiro_hardware_interface/serial_port.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cefiro_msgs/msg/motor_rpm.hpp>
#include <cefiro_msgs/msg/motor_dshot.hpp>
#include <cefiro_msgs/msg/motor_telemetry.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
namespace cefiro::hardware {
namespace {
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
constexpr double kRadiansPerDegree = 0.017453292519943295;
constexpr double kMagneticTeslaPerMicrotesla = 1.0e-6;
constexpr std::size_t kCommandQueueMax = 24;
constexpr const char* kHardwareVersion = "0.4.1";

std::string number(double value, int precision = 5) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}
std::string jsonQuote(std::string_view source) {
  std::string result = "\"";
  for (char ch : source) {
    if (ch == '"' || ch == '\\') { result.push_back('\\'); result.push_back(ch); }
    else if (ch == '\n') result += "\\n";
    else if (ch == '\r') result += "\\r";
    else if (static_cast<unsigned char>(ch) < 0x20) result.push_back(' ');
    else result.push_back(ch);
  }
  result.push_back('"');
  return result;
}
std::optional<std::vector<double>> numbers(const std::vector<std::string>& fields,
                                           std::size_t begin, std::size_t count) {
  if (begin > fields.size() || count > fields.size() - begin) return std::nullopt;
  std::vector<double> values;
  values.reserve(count);
  for (std::size_t i = begin; i < begin + count; ++i) {
    const auto v = parseFinite(fields[i]);
    if (!v) return std::nullopt;
    values.push_back(*v);
  }
  return values;
}
template<std::size_t N>
std::vector<std::string> floatFields(const std::array<double,N>& values) {
  std::vector<std::string> fields;
  fields.reserve(N);
  for (const double value : values) fields.push_back(number(value, 3));
  return fields;
}
std::vector<std::string> floatFields(const std::vector<double>& values) {
  std::vector<std::string> fields;
  fields.reserve(values.size());
  for (const double value : values) fields.push_back(number(value, 6));
  return fields;
}
}  // namespace

class HardwareNode final : public rclcpp::Node {
public:
  HardwareNode() : Node("cefiro_hardware_interface") {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    protocol_mode_ = declare_parameter<std::string>("protocol_mode", "legacy_text");
    enable_command_tx_ = declare_parameter<bool>("enable_command_tx", false);
    // Explicit opt-in: this is NOT measured-RPM closed-loop control. The MCU
    // supports manual DShot 0 / 48..300, but rejects CMD_RPM and CMD_DIR.
    rpm_to_dshot_enabled_ = declare_parameter<bool>("rpm_to_dshot_enabled", false);
    allow_direct_dshot_ = declare_parameter<bool>("allow_direct_dshot", false);
    manual_map_.reference_max_rpm = declare_parameter<double>("rpm_full_scale", 12000.0);
    manual_map_.rpm_deadband = declare_parameter<double>("rpm_deadband", 25.0);
    const int dshot_min_param = declare_parameter<int>("dshot_min", 48);
    const int dshot_max_param = declare_parameter<int>("dshot_max", 300);
    if (dshot_min_param < 48 || dshot_max_param > 300 ||
        dshot_max_param < dshot_min_param) {
      throw std::invalid_argument("Firmware manual DShot allowed: zero or 48..300");
    }
    manual_map_.dshot_min = static_cast<std::uint16_t>(dshot_min_param);
    manual_map_.dshot_max = static_cast<std::uint16_t>(dshot_max_param);
    allow_direction_commands_ = declare_parameter<bool>("allow_direction_commands", false);
    reconnect_s_ = declare_parameter<double>("serial_reconnect_s", 2.0);
    sensor_stale_s_ = declare_parameter<double>("sensor_stale_s", 1.5);
    // Maximum GUI input gap; Raspberry independently refreshes already-received
    // DShot every 40 ms. This is a bounded lease, NOT a disabled failsafe.
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.60);
    manual_keepalive_period_s_ = declare_parameter<double>("manual_keepalive_period_s", 0.040);
    orientation_timeout_s_ = declare_parameter<double>("orientation_timeout_s", 0.2);
    max_command_rpm_ = declare_parameter<double>("max_command_rpm", 12000.0);
    imu_frame_ = declare_parameter<std::string>("imu_frame_id", "cefiro_fmu_imu");
    mag_frame_ = declare_parameter<std::string>("mag_frame_id", "cefiro_fmu_mag");
    baro_frame_ = declare_parameter<std::string>("baro_frame_id", "cefiro_fmu_baro");
    body_frame_ = declare_parameter<std::string>("body_frame_id", "drone_body_link");
    if (protocol_mode_ != "legacy_text" && protocol_mode_ != "cf1") {
      throw std::invalid_argument("protocol_mode must be legacy_text or cf1");
    }
    if (baudrate_ <= 0 || reconnect_s_ <= 0.0 || sensor_stale_s_ <= 0.0 ||
        command_timeout_s_ <= 0.0 || orientation_timeout_s_ <= 0.0 ||
        !std::isfinite(reconnect_s_) || !std::isfinite(sensor_stale_s_) ||
        !std::isfinite(command_timeout_s_) || !std::isfinite(orientation_timeout_s_) ||
        !std::isfinite(max_command_rpm_) || max_command_rpm_ <= 0.0 ||
        !std::isfinite(manual_keepalive_period_s_)) {
      throw std::invalid_argument("Invalid UART/timing/motor parameter");
    }
    if (!validManualMotorMap(manual_map_) ||
        command_timeout_s_ > 0.90 || command_timeout_s_ < 0.15 ||
        manual_keepalive_period_s_ < 0.020 ||
        manual_keepalive_period_s_ > 0.100 ||
        manual_keepalive_period_s_ * 3.0 > command_timeout_s_) {
      throw std::invalid_argument(
          "Invalid manual motor configuration: DShot=0 or 48..300; "
          "command_timeout_s (GUI lease) must be 0.15..0.90 s, "
          "manual_keepalive_period_s 0.020..0.100 s and at least 3 periods "
          "within the GUI lease. Received lease=" + number(command_timeout_s_, 3) +
          "s refresh=" + number(manual_keepalive_period_s_, 3) + "s");
    }
    manual_stream_.configure(manual_keepalive_period_s_, command_timeout_s_);
    can_tx_ = protocol_mode_ == "cf1" && enable_command_tx_;
    // All names are relative; default launch namespace 'real' isolates the real plant.
    const auto sensor_qos = rclcpp::SensorDataQoS();
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", sensor_qos);
    pub_mag_ = create_publisher<sensor_msgs::msg::MagneticField>("magnetometer/data", sensor_qos);
    pub_baro_ = create_publisher<sensor_msgs::msg::FluidPressure>("bmp581/air_pressure", sensor_qos);
    pub_imu_temp_ = create_publisher<sensor_msgs::msg::Temperature>("cefiro/sensors/imu_temperature", 10);
    pub_baro_temp_ = create_publisher<sensor_msgs::msg::Temperature>("cefiro/sensors/baro_temperature", 10);
    pub_alt_isa_ = create_publisher<std_msgs::msg::Float64>("cefiro/sensors/baro/isa_altitude", 10);
    pub_alt_rel_ = create_publisher<std_msgs::msg::Float64>("cefiro/sensors/baro/relative_altitude", 10);
    pub_rpm_ = create_publisher<cefiro_msgs::msg::MotorRpm>("cefiro_actuators/measured_rpm", 10);
    pub_motor_telemetry_ = create_publisher<cefiro_msgs::msg::MotorTelemetry>("cefiro_actuators/telemetry", 10);
    pub_status_ = create_publisher<std_msgs::msg::String>("cefiro/hardware/status", 10);
    pub_controller_ = create_publisher<std_msgs::msg::String>("cefiro/control/status", 10);
    pub_ack_ = create_publisher<std_msgs::msg::String>("cefiro/hardware/command_ack", 10);
    pub_commanded_dshot_ = create_publisher<cefiro_msgs::msg::MotorDshot>("cefiro/hardware/commanded_dshot", 10);

    sub_rpm_ = create_subscription<cefiro_msgs::msg::MotorRpm>("cefiro_actuators/target_rpm", 10,
      [this](cefiro_msgs::msg::MotorRpm::ConstSharedPtr m) { onRpm(*m); });
    sub_dshot_ = create_subscription<cefiro_msgs::msg::MotorDshot>("cefiro_actuators/dshot_throttle", 10,
      [this](cefiro_msgs::msg::MotorDshot::ConstSharedPtr m) { onDshot(*m); });
    sub_armed_ = create_subscription<std_msgs::msg::Bool>("cefiro_actuators/armed", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { onArmed(m->data); });
    sub_direction_ = create_subscription<std_msgs::msg::Int8MultiArray>("cefiro_actuators/motor_direction", 10,
      [this](std_msgs::msg::Int8MultiArray::ConstSharedPtr m) { onDirection(*m); });
    sub_enable_ = create_subscription<std_msgs::msg::Bool>("cefiro/control/enable", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        // In this GUI 'enable' is the output gate for BOTH manual and automatic.
        // Firmware CF_ENABLE_AUTOMATIC_OUTPUT=0: never forward CMD_ENABLE=1.
        // Only the additional GUI armed=true intent can authorize manual DShot.
        if (!m->data) disarm("GUI output disabled");
        else { controller_enabled_ = true; maybeArm(); }
      });
    sub_estop_ = create_subscription<std_msgs::msg::Bool>("cefiro/control/emergency_stop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { if (m->data) emergencyStop(); });
    sub_attitude_ = create_subscription<geometry_msgs::msg::Vector3Stamped>("cefiro/control/attitude_reference", 10,
      [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr m) { onAttitude(*m); });
    sub_mode_ = create_subscription<std_msgs::msg::String>("cefiro/control/mode", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        // GUI currently publishes "ATTITUDE" even on safe-stop; it is NOT a
        // firmware CMD_MODE enum. Mode MANUAL is selected at link startup.
        if (m->data != "ATTITUDE" && m->data != "MANUAL" && m->data != "AUTO") {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "GUI mode ignored: %s", m->data.c_str());
        }
      });
    sub_strategy_ = create_subscription<std_msgs::msg::String>("cefiro/control/strategy", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        // Automatic motor output is firmware-inhibited; keep strategy as GUI metadata.
        (void)m;
      });
    sub_pid_ = create_subscription<std_msgs::msg::Float64MultiArray>("cefiro/control/pid_gains", 10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
        (void)m; // STM32 firmware returns UNSUPPORTED for CMD_PID.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "PID gains ignored: firmware only supports 3x6 LQR K in inactive state");
      });
    sub_lqr_ = create_subscription<std_msgs::msg::Float64MultiArray>("cefiro/control/lqr_gain", 10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) { onGains("CMD_LQR", m->data, 18); });

    reset_fault_service_ = create_service<std_srvs::srv::Trigger>(
      "cefiro/hardware/reset_fault",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!can_tx_ || !serial_.connected() || controller_enabled_ ||
            arm_requested_ || arm_confirmed_) {
          response->success = false;
          response->message = "Not allowed: require CF1 TX, connected, GUI output OFF, and disarmed";
          return;
        }
        reset_fault_ack_sequence_ = queueCommand("CMD_RESET_FAULT", {});
        response->success = reset_fault_ack_sequence_.has_value();
        response->message = response->success
          ? "Reset request queued; verify MCU ACK=OK and next STAT state!=FAULT"
          : "Unable to queue reset request";
      });
    io_timer_ = create_wall_timer(10ms, [this] { pollSerial(); });
    status_timer_ = create_wall_timer(500ms, [this] { publishStatus(); });
    RCLCPP_INFO(get_logger(), "CEFIRO hardware interface v%s (manual keepalive + 0.60 s GUI lease)",
      kHardwareVersion);
    RCLCPP_INFO(get_logger(),
      "CEFIRO USART %s at %d 8N1, namespace=%s, protocol=%s, command TX=%s",
      port_.c_str(), baudrate_, get_namespace(), protocol_mode_.c_str(), can_tx_ ? "ENABLED" : "DISABLED");
    RCLCPP_INFO(get_logger(),
      "GUI MANUAL /real interface: TX=%s, nominal-RPM-to-DSHOT=%s, DShot zero or %u..%u; no measured RPM",
      can_tx_ ? "enabled" : "disabled",
      rpm_to_dshot_enabled_ ? "enabled" : "disabled",
      unsigned(manual_map_.dshot_min), unsigned(manual_map_.dshot_max));
    if (!can_tx_) RCLCPP_WARN(get_logger(), "CF1 TX OFF. Sensor telemetry only; old legacy_text firmware can never drive motors");
  }

  ~HardwareNode() override {
    // Best-effort safety request only; independent MCU watchdog is mandatory.
    if (can_tx_ && serial_.connected()) {
      const auto stop = encodeFrame(++sequence_, "CMD_STOP", {});
      const auto disarm = encodeFrame(++sequence_, "CMD_ARM", {"0"});
      if (stop) (void)serial_.writeSome(reinterpret_cast<const std::uint8_t*>(stop->data()), stop->size());
      if (disarm) (void)serial_.writeSome(reinterpret_cast<const std::uint8_t*>(disarm->data()), disarm->size());
    }
    serial_.close();
  }

private:
  void resetConnection(std::string_view reason) {
    if (serial_.connected()) RCLCPP_WARN(get_logger(),
      "Serial link lost: %.*s; a NEW GUI enable/arm action will be required",
      static_cast<int>(reason.size()), reason.data());
    serial_.close();
    lines_.clear();
    tx_queue_.clear();
    pending_tx_.clear();
    pending_offset_ = 0;
    arm_requested_ = false;
    arm_confirmed_ = false;
    controller_enabled_ = false;
    manual_mode_confirmed_ = false;
    mcu_status_seen_ = false;
    arm_ack_sequence_.reset();
    mode_ack_sequence_.reset();
    reset_fault_ack_sequence_.reset();
    arm_confirmed_at_.reset();
    manual_stream_.reset();
    last_mcu_stat_.reset();
    last_imu_.reset(); last_mag_.reset(); last_baro_.reset(); last_motor_.reset();
    orientation_.reset();
    requested_rpm_.fill(0.0);
    last_dshot_.fill(0);
    reconnect_at_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(reconnect_s_));
  }
  void connectPort() {
    if (Clock::now() < reconnect_at_) return;
    std::string error;
    if (!serial_.open(port_, baudrate_, error)) {
      ++serial_errors_;
      reconnect_at_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(reconnect_s_));
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for UART %s: %s", port_.c_str(), error.c_str());
      return;
    }
    // Never restore ARM / GUI setpoint after a physical link reconnect.
    arm_requested_ = false;
    arm_confirmed_ = false;
    controller_enabled_ = false;
    manual_mode_confirmed_ = false;
    mcu_status_seen_ = false;
    arm_ack_sequence_.reset();
    mode_ack_sequence_.reset();
    arm_confirmed_at_.reset();
    manual_stream_.reset();
    last_mcu_stat_.reset();
    tx_queue_.clear();
    pending_tx_.clear();
    pending_offset_ = 0;
    lines_.clear();
    RCLCPP_INFO(get_logger(), "Serial connected: %s", port_.c_str());
    if (can_tx_) {
      // SAFE STOP, NOT E-STOP: the latter latches FMU FAULT and requires an
      // explicit reset. Configure manual mode while still disarmed.
      (void)queueCommand("CMD_STOP", {});
      (void)queueCommand("CMD_ARM", {"0"});
      mode_ack_sequence_ = queueCommand("CMD_MODE", {"MANUAL"});
    }
  }
  void pollSerial() {
    if (!serial_.connected()) { connectPort(); return; }
    std::array<std::uint8_t, 4096> buffer{};
    for (int attempt = 0; attempt < 6; ++attempt) {
      const ssize_t n = serial_.readSome(buffer.data(), buffer.size());
      if (n < 0) { ++serial_errors_; resetConnection("read error"); return; }
      if (n == 0) break;
      const std::string_view chunk(reinterpret_cast<const char*>(buffer.data()),
                                   static_cast<std::size_t>(n));
      for (const auto& line : lines_.feed(chunk)) {
        if (protocol_mode_ == "legacy_text") onLegacy(line);
        else onFrame(line);
      }
    }
    if (can_tx_) {
      const auto current = Clock::now();
      if (arm_ack_sequence_ && current >= arm_ack_deadline_) {
        disarm("STM32 ARM ACK timeout");
      }
      // The STM32 has an INDEPENDENT 300 ms DShot watchdog. Refresh its
      // command from the Raspberry on a steady ROS timer, independent of the
      // Qt event/render thread. Never refresh from a stale GUI command.
      if (arm_confirmed_) {
        if (manual_stream_.expired(current, arm_confirmed_at_.value_or(current))) {
          ++gui_timeout_stops_;
          RCLCPP_WARN(get_logger(),
            "GUI setpoint stream expired (last GUI age %.3f s, limit %.3f s); SAFE STOP",
            manual_stream_.guiAgeSeconds(current), command_timeout_s_);
          disarm("GUI setpoint timeout");
        } else if (manual_stream_.due(current)) {
          if (const auto* codes = manual_stream_.setpoint()) sendMotorDshot(*codes);
        }
      }
      if (arm_confirmed_ &&
          (!last_mcu_stat_ ||
           std::chrono::duration<double>(current - *last_mcu_stat_).count() > 1.25)) {
        disarm("MCU status heartbeat lost");
      }
      flushTx();
    }
  }
  std::optional<std::uint16_t> queueCommand(std::string_view kind,
                                             const std::vector<std::string>& payload) {
    if (!can_tx_ || !serial_.connected()) return std::nullopt;
    const auto seq = ++sequence_;
    const auto data = encodeFrame(seq, kind, payload);
    if (!data) { RCLCPP_WARN(get_logger(), "Rejected invalid outgoing CF1 command"); return std::nullopt; }
    if (kind == "CMD_RPM" || kind == "CMD_DSHOT") {
      // Latest setpoint wins; stale queued throttle must never play back later.
      std::erase_if(tx_queue_, [](const std::string& s) {
        return s.find("|CMD_RPM|") != std::string::npos ||
               s.find("|CMD_DSHOT|") != std::string::npos;
      });
    }
    if (tx_queue_.size() >= kCommandQueueMax) {
      tx_queue_.clear();
      pending_tx_.clear();
      pending_offset_ = 0;
      arm_requested_ = false;
      arm_confirmed_ = false;
      arm_ack_sequence_.reset();
      const auto estop = encodeFrame(++sequence_, "CMD_ESTOP", {});
      if (estop) tx_queue_.push_back(*estop);
      RCLCPP_ERROR(get_logger(), "UART TX queue overflow: commands dropped, emergency stop queued");
      return std::nullopt;
    }
    tx_queue_.push_back(*data);
    return seq;
  }
  void flushTx() {
    for (int i=0; i<6; ++i) {
      if (pending_tx_.empty()) {
        if (tx_queue_.empty()) return;
        pending_tx_ = std::move(tx_queue_.front());
        tx_queue_.pop_front();
        pending_offset_ = 0;
      }
      const auto n = serial_.writeSome(
        reinterpret_cast<const std::uint8_t*>(pending_tx_.data()) + pending_offset_,
        pending_tx_.size() - pending_offset_);
      if (n < 0) { ++serial_errors_; resetConnection("write error"); return; }
      if (n == 0) return;
      pending_offset_ += static_cast<std::size_t>(n);
      if (pending_offset_ == pending_tx_.size()) { pending_tx_.clear(); pending_offset_ = 0; }
    }
  }
  void disarm(std::string_view reason) {
    const bool was_active = arm_requested_ || arm_confirmed_ || controller_enabled_;
    arm_requested_ = false;
    arm_confirmed_ = false;
    controller_enabled_ = false;
    arm_ack_sequence_.reset();
    arm_confirmed_at_.reset();
    manual_stream_.reset();
    requested_rpm_.fill(0.0);
    last_dshot_.fill(0);
    // Stale queued DShot requests must never execute after STOP.
    tx_queue_.clear();
    // Partially transmitted UART frame cannot be interrupted mid-frame.
    if (pending_offset_ == 0) pending_tx_.clear();
    manual_mode_confirmed_ = false;
    mode_ack_sequence_.reset();
    if (can_tx_ && serial_.connected()) {
      if (fault_latched_) {
        // GUI STOP publishes multiple topics without cross-topic ordering.
        // A late enable=false / armed=false MUST NOT erase an earlier ESTOP.
        // Keep the MCU FAULT latched until explicit reset service + ACK.
        (void)queueCommand("CMD_ESTOP", {});
      } else {
        (void)queueCommand("CMD_STOP", {});
        (void)queueCommand("CMD_ARM", {"0"});
        mode_ack_sequence_ = queueCommand("CMD_MODE", {"MANUAL"});
      }
    }
    if (was_active) RCLCPP_WARN(get_logger(), "SAFE STOP / disarm: %.*s",
      static_cast<int>(reason.size()), reason.data());
  }
  void emergencyStop() {
    disarm("GUI EMERGENCY ZERO");
    if (can_tx_ && serial_.connected()) {
      tx_queue_.clear();
      mode_ack_sequence_.reset();
      manual_mode_confirmed_ = false;
      (void)queueCommand("CMD_ESTOP", {});
      fault_latched_ = true;
    }
    RCLCPP_ERROR(get_logger(),
      "GUI EMERGENCY ZERO: STM32 FMU FAULT will remain latched until explicit reset");
  }
  void maybeArm() {
    if (!can_tx_ || !serial_.connected() || !controller_enabled_ ||
        !arm_requested_ || arm_confirmed_ || arm_ack_sequence_ ||
        !(rpm_to_dshot_enabled_ || allow_direct_dshot_) ||
        !manual_mode_confirmed_ || !mcu_status_seen_ || fault_latched_ ||
        reverse_requested_ || !last_mcu_stat_ ||
        std::chrono::duration<double>(Clock::now() - *last_mcu_stat_).count() > 1.25) {
      return;
    }
    arm_ack_sequence_ = queueCommand("CMD_ARM", {"1"});
    if (arm_ack_sequence_) {
      arm_ack_deadline_ = Clock::now() + 1000ms;
      RCLCPP_INFO(get_logger(), "GUI ARM requested: waiting for matching STM32 ACK");
    }
  }
  void onArmed(bool armed) {
    if (!armed) { disarm("GUI requested disarm"); return; }
    if (!can_tx_ || !serial_.connected()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "ARM rejected: CF1 command TX is disabled or disconnected");
      return;
    }
    if (fault_latched_ || reverse_requested_) {
      RCLCPP_WARN(get_logger(), "ARM rejected: FMU fault or unsupported reverse direction");
      return;
    }
    if (arm_requested_ && (arm_confirmed_ || arm_ack_sequence_)) return;
    if (!(rpm_to_dshot_enabled_ || allow_direct_dshot_)) {
      RCLCPP_WARN(get_logger(), "ARM ignored: neither GUI RPM mapping nor direct DShot bench mode enabled");
      return;
    }
    arm_requested_ = true;
    arm_confirmed_ = false;
    arm_confirmed_at_.reset();
    manual_stream_.reset();
    maybeArm();
  }
  void sendMotorDshot(const std::array<std::uint16_t, 4>& codes) {
    if (!can_tx_ || !serial_.connected() || !controller_enabled_ ||
        !arm_requested_ || !arm_confirmed_ || reverse_requested_ || fault_latched_) return;
    std::vector<std::string> fields;
    for (const std::uint16_t code : codes) {
      if (code != 0 && (code < manual_map_.dshot_min || code > manual_map_.dshot_max)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Manual DShot outside configured firmware bench limits");
        return;
      }
      fields.push_back(std::to_string(code));
    }
    if (!queueCommand("CMD_DSHOT", fields)) return;
    manual_stream_.transmitted(Clock::now());
    ++manual_refresh_sent_;
    last_dshot_ = codes;
    cefiro_msgs::msg::MotorDshot msg;
    msg.data = codes;
    pub_commanded_dshot_->publish(msg); // Command REQUEST, not actual motor telemetry.
  }
  void onRpm(const cefiro_msgs::msg::MotorRpm& msg) {
    // The GUI publishes nominal RPMs; firmware CMD_RPM always UNSUPPORTED.
    // Conversion is strictly an opt-in OPEN-LOOP bench command.
    if (!rpm_to_dshot_enabled_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "GUI RPM command ignored: enable rpm_to_dshot_enabled for explicit open-loop DShot mapping");
      return;
    }
    if (std::any_of(msg.data.begin(), msg.data.end(),
          [this](double rpm) { return !std::isfinite(rpm) || rpm > max_command_rpm_; })) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "GUI nominal RPM exceeds max_command_rpm safety ceiling");
      return;
    }
    const auto codes = mapGuiRpmToDshot(msg.data, manual_map_);
    if (!codes) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "GUI RPM outside nominal open-loop mapping range; command rejected");
      return;
    }
    if (!can_tx_ || !controller_enabled_ || !arm_requested_ || reverse_requested_) return;
    requested_rpm_ = msg.data;
    manual_stream_.update(*codes, Clock::now());
    ++gui_setpoints_received_;
    // Do not send immediately for every GUI message. The 40-ms Raspberry
    // refresh loop provides a smooth, bounded DShot stream instead.
  }
  void onDshot(const cefiro_msgs::msg::MotorDshot& msg) {
    if (!allow_direct_dshot_) return;
    for (const auto code : msg.data) {
      if (code != 0 && (code < manual_map_.dshot_min || code > manual_map_.dshot_max)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "DShot outside allowed 0 or %u..%u", unsigned(manual_map_.dshot_min),
          unsigned(manual_map_.dshot_max));
        return;
      }
    }
    if (!can_tx_ || !controller_enabled_ || !arm_requested_ || reverse_requested_) return;
    manual_stream_.update(msg.data, Clock::now());
    ++gui_setpoints_received_;
    // Periodic refresh is owned by the Raspberry, not by GUI event scheduling.
  }
  void onDirection(const std_msgs::msg::Int8MultiArray& msg) {
    if (msg.data.size() != 4) return;
    const bool unsupported = std::any_of(msg.data.begin(), msg.data.end(),
       [](std::int8_t v) { return v != 1; });
    reverse_requested_ = unsupported;
    if (unsupported) {
      if (arm_requested_ || arm_confirmed_) disarm("reverse direction unsupported by CEFIRO FMU");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Reverse/invalid direction rejected; STM32 CMD_DIR is UNSUPPORTED");
    } else {
      maybeArm();
    }
  }
  void onAttitude(const geometry_msgs::msg::Vector3Stamped& m) {
    (void)m;
    // Firmware CF_ENABLE_AUTOMATIC_OUTPUT=0. Never route the automatic GUI's
    // attitude reference into ESC outputs via the manual throttle path.
    if (controller_enabled_ && !arm_requested_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 6000,
        "Automatic GUI reference not sent to motors: firmware CF_ENABLE_AUTOMATIC_OUTPUT=0");
    }
  }
  void onGains(std::string_view kind, const std::vector<double>& values,
               std::size_t expected) {
    if (!can_tx_ || arm_requested_ || arm_confirmed_ ||
        controller_enabled_ || values.size() != expected) return;
    if (!std::all_of(values.begin(), values.end(),
          [](double v) { return std::isfinite(v) && std::abs(v) < 1e7; })) return;
    (void)queueCommand(kind, floatFields(values));
  }

  void onLegacy(const std::string& line) {
    const auto record = parseLegacy(line);
    if (!record) { if (line.starts_with("IMU") || line.starts_with("MAG") || line.starts_with("BARO")) ++bad_frames_; return; }
    switch (record->kind) {
      case LegacyKind::Imu: publishImu(record->imu); break;
      case LegacyKind::Mag: publishMag(record->mag); break;
      case LegacyKind::Baro: publishBaro(record->baro); break;
      case LegacyKind::SensorError: {
        last_sensor_error_ = record->sensor + ": " + record->error;
        break;
      }
      default: break;
    }
    ++rx_frames_;
  }
  void onFrame(const std::string& line) {
    const auto frame = decodeFrame(line);
    if (!frame) {
      ++bad_frames_;
      if (bad_frames_ % 50 == 1) RCLCPP_WARN(get_logger(), "Dropped invalid CF1 frame (CRC/framing)");
      return;
    }
    if (handleFrame(*frame)) ++rx_frames_;
    else ++bad_frames_;
  }
  bool handleFrame(const Frame& frame) {
    const auto& fields = frame.fields;
    if (frame.kind == "IMU" && (fields.size() == 7 || fields.size() == 8)) {
      const auto v = numbers(fields, 0, 7);
      if (!v) return false;
      ImuReading s;
      std::copy_n(v->begin(), 3, s.acceleration_ms2.begin());
      std::copy_n(v->begin()+3, 3, s.gyro_dps.begin());
      s.temperature_c = (*v)[6];
      publishImu(s);
      return true;
    }
    if (frame.kind == "MAG" && (fields.size() == 3 || fields.size() == 4)) {
      const auto v = numbers(fields, 0, 3);
      if (!v) return false;
      MagReading s;
      std::copy_n(v->begin(), 3, s.magnetic_uT.begin());
      publishMag(s);
      return true;
    }
    if (frame.kind == "BARO" && (fields.size() == 5 || fields.size() == 6)) {
      const auto v = numbers(fields, 0, 5);
      if (!v || (*v)[0] <= 0.0 || (*v)[3] <= 0.0) return false;
      publishBaro({(*v)[0], (*v)[1], (*v)[2], (*v)[3], (*v)[4]});
      return true;
    }
    if (frame.kind == "ATT" && (fields.size() == 4 || fields.size() == 5)) {
      const auto v = numbers(fields, 0, 4);
      if (!v) return false;
      double squared = 0.0;
      for (double x : *v) squared += x*x;
      const double norm = std::sqrt(squared);
      if (norm <= 0.5 || norm >= 1.5) return false;
      std::array<double,4> q{};
      for (std::size_t i=0; i<4; ++i) q[i] = (*v)[i] / norm;
      orientation_ = q;
      orientation_received_ = Clock::now();
      return true;
    }
    if (frame.kind == "MOTOR" && fields.size() == 11) {
      const auto v = numbers(fields, 0, 9);
      if (!v) return false;
      for (std::size_t i=0;i<4;++i) if ((*v)[i] < 0.0) return false;
      if ((*v)[8] < 0.0) return false;
      const auto armed = parseInteger(fields[9], 0, 1);
      const auto watchdog = parseInteger(fields[10], 0, 1);
      if (!armed || !watchdog) return false;
      cefiro_msgs::msg::MotorRpm msg;
      for (std::size_t i = 0; i < 4; ++i) msg.data[i] = (*v)[i];
      pub_rpm_->publish(msg);
      last_motor_ = Clock::now();
      if (!*armed) arm_confirmed_ = false;
      last_mcu_status_ = "MOTOR bus=" + number((*v)[8], 2) + "V, armed=" + std::to_string(*armed) +
                         ", watchdog=" + std::to_string(*watchdog);
      // Do not fake unmeasured thrust/current/torque fields of MotorTelemetry.
      return true;
    }
    if (frame.kind == "MOTOR_FULL") return onMotorFull(fields);
    if (frame.kind == "ACK" && fields.size() == 3) {
      const auto ack = parseInteger(fields[0], 0, 65535);
      if (!ack) return false;
      last_ack_ = std::to_string(*ack) + "," + fields[1] + "," + fields[2];
      std_msgs::msg::String ack_msg;
      ack_msg.data = last_ack_;
      pub_ack_->publish(ack_msg);
      if (mode_ack_sequence_ && *ack == *mode_ack_sequence_ &&
          fields[1] == "CMD_MODE") {
        manual_mode_confirmed_ = fields[2] == "OK";
        mode_ack_sequence_.reset();
        if (!manual_mode_confirmed_) RCLCPP_ERROR(get_logger(),
          "STM32 rejected CMD_MODE MANUAL: %s", fields[2].c_str());
        else maybeArm();
      }
      if (arm_ack_sequence_ && *ack == *arm_ack_sequence_ &&
          fields[1] == "CMD_ARM") {
        const bool accepted = arm_requested_ && fields[2] == "OK";
        arm_ack_sequence_.reset();
        arm_confirmed_ = accepted;
        if (!accepted) {
          RCLCPP_WARN(get_logger(), "STM32 ARM rejected: %s", fields[2].c_str());
          disarm("STM32 ARM rejected");
        } else {
          arm_confirmed_at_ = Clock::now();
          RCLCPP_INFO(get_logger(), "STM32 ACK accepted GUI MANUAL ARM");
          if (manual_stream_.fresh(Clock::now())) {
            if (const auto* codes = manual_stream_.setpoint()) sendMotorDshot(*codes);
          }
        }
      }
      if (reset_fault_ack_sequence_ && *ack == *reset_fault_ack_sequence_ &&
          fields[1] == "CMD_RESET_FAULT") {
        if (fields[2] == "OK") {
          fault_latched_ = false;
          manual_mode_confirmed_ = false;
          mode_ack_sequence_ = queueCommand("CMD_MODE", {"MANUAL"});
        }
        reset_fault_ack_sequence_.reset();
        RCLCPP_INFO(get_logger(), "STM32 CMD_RESET_FAULT: %s", fields[2].c_str());
      }
      if (fields[1] == "CMD_DSHOT" && fields[2] != "OK") {
        ++dshot_denials_;
        RCLCPP_ERROR(get_logger(),
          "STM32 rejected CMD_DSHOT (seq=%s, reason=%s, GUI age=%.3fs, "
          "last DShot TX age=%.3fs, MCU state=%d). "
          "Firmware DENIED may mean its independent 300ms watchdog "
          "disarmed, ESC timeout, or power/mode interlock.",
          fields[0].c_str(), fields[2].c_str(),
          manual_stream_.guiAgeSeconds(Clock::now()),
          manual_stream_.txAgeSeconds(Clock::now()), mcu_fmu_state_);
        disarm("STM32 DShot rejected");
      }
      return true;
    }
    if (frame.kind == "STAT") {
      // Verified format in modular cf_fmu.c:
      // FMU_V3,state,calibrated,k_valid,auto_enabled,bad_packets,faults,imu_dma_err,link_drops
      if (fields.size() < 9 || fields[0] != "FMU_V3") return false;
      const auto state = parseInteger(fields[1], 0, 4);
      const auto auto_enabled = parseInteger(fields[4], 0, 1);
      if (!state || !auto_enabled) return false;
      const bool entering_fault = *state == 4 && !fault_latched_;
      mcu_fmu_state_ = static_cast<int>(*state);
      mcu_auto_enabled_ = (*auto_enabled != 0);
      mcu_status_seen_ = true;
      last_mcu_stat_ = Clock::now();
      if (*state == 4) {
        fault_latched_ = true;
        if (entering_fault || arm_requested_ || arm_confirmed_) disarm("STM32 FMU reports FAULT");
      }
      last_mcu_status_.clear();
      for (std::size_t i=0;i<fields.size();++i) {
        if(i)last_mcu_status_ += ',';
        last_mcu_status_ += fields[i];
      }
      std_msgs::msg::String msg;
      msg.data = "FMU state=" + std::to_string(*state) +
        " | auto=" + (*auto_enabled ? "enabled" : "inhibited") +
        " | ARM_ACK=" + (arm_confirmed_ ? "yes" : "no") +
        " | CF1=" + last_mcu_status_;
      pub_controller_->publish(msg);
      maybeArm();
      return true;
    }
    if (frame.kind == "ERROR") {
      last_mcu_status_ = "MCU ERROR";
      for (const auto& field : fields) last_mcu_status_ += " " + field;
      RCLCPP_WARN(get_logger(), "%s", last_mcu_status_.c_str());
      return true;
    }
    return false;
  }
  bool onMotorFull(const std::vector<std::string>& fields) {
    if (fields.size() != 50) return false;
    auto v = numbers(fields, 0, 47);
    if (!v) return false;
    const auto armed = parseInteger(fields[47], 0, 1);
    const auto watchdog = parseInteger(fields[48], 0, 1);
    const auto source = parseInteger(fields[49], 0, 2);
    if (!armed || !watchdog || !source) return false;
    auto msg = cefiro_msgs::msg::MotorTelemetry();
    msg.header.stamp = now();
    msg.header.frame_id = body_frame_;
    for (std::size_t i=0;i<4;++i) {
      if ((*v)[4+i] < 0) return false;
      const auto dshot = parseInteger(fields[8+i], 0, 65535);
      const auto packet = parseInteger(fields[12+i], 0, 65535);
      if (!dshot || !packet) return false;
      msg.requested_rpm[i] = (*v)[i];
      msg.measured_rpm[i] = (*v)[4+i];
      msg.dshot_value[i] = static_cast<std::uint16_t>(*dshot);
      msg.dshot_packet[i] = static_cast<std::uint16_t>(*packet);
      msg.throttle_fraction[i] = (*v)[16+i];
      msg.thrust_n[i] = (*v)[20+i];
      msg.aerodynamic_torque_nm[i] = (*v)[24+i];
      msg.reaction_torque_nm[i] = (*v)[28+i];
      msg.electromagnetic_torque_nm[i] = (*v)[32+i];
      msg.current_a[i] = (*v)[36+i];
      msg.applied_voltage_v[i] = (*v)[40+i];
    }
    msg.bus_voltage_v = (*v)[44];
    msg.total_thrust_n = (*v)[45];
    msg.total_reaction_yaw_nm = (*v)[46];
    msg.armed = *armed == 1;
    msg.watchdog_ok = *watchdog == 1;
    msg.command_source = static_cast<std::uint8_t>(*source);
    cefiro_msgs::msg::MotorRpm rpm;
    rpm.data = msg.measured_rpm;
    pub_rpm_->publish(rpm);
    pub_motor_telemetry_->publish(msg);
    last_motor_ = Clock::now();
    if (!msg.armed) arm_confirmed_ = false;
    return true;
  }
  void publishImu(const ImuReading& reading) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = imu_frame_;
    msg.linear_acceleration.x = reading.acceleration_ms2[0];
    msg.linear_acceleration.y = reading.acceleration_ms2[1];
    msg.linear_acceleration.z = reading.acceleration_ms2[2];
    msg.angular_velocity.x = reading.gyro_dps[0] * kRadiansPerDegree;
    msg.angular_velocity.y = reading.gyro_dps[1] * kRadiansPerDegree;
    msg.angular_velocity.z = reading.gyro_dps[2] * kRadiansPerDegree;
    if (orientation_ &&
        std::chrono::duration<double>(Clock::now() - orientation_received_).count() <= orientation_timeout_s_) {
      msg.orientation.w = (*orientation_)[0];
      msg.orientation.x = (*orientation_)[1];
      msg.orientation.y = (*orientation_)[2];
      msg.orientation.z = (*orientation_)[3];
    } else {
      msg.orientation.w = 1.0;  // Placeholder only; covariance -1 means unavailable.
      msg.orientation_covariance[0] = -1.0;
    }
    pub_imu_->publish(msg);
    sensor_msgs::msg::Temperature temperature;
    temperature.header = msg.header;
    temperature.temperature = reading.temperature_c;
    pub_imu_temp_->publish(temperature);
    last_imu_ = Clock::now();
    last_sensor_error_.clear();
  }
  void publishMag(const MagReading& reading) {
    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = now();
    msg.header.frame_id = mag_frame_;
    msg.magnetic_field.x = reading.magnetic_uT[0] * kMagneticTeslaPerMicrotesla;
    msg.magnetic_field.y = reading.magnetic_uT[1] * kMagneticTeslaPerMicrotesla;
    msg.magnetic_field.z = reading.magnetic_uT[2] * kMagneticTeslaPerMicrotesla;
    pub_mag_->publish(msg);
    last_mag_ = Clock::now();
  }
  void publishBaro(const BaroReading& reading) {
    sensor_msgs::msg::FluidPressure msg;
    msg.header.stamp = now();
    msg.header.frame_id = baro_frame_;
    msg.fluid_pressure = reading.pressure_hpa * 100.0;
    pub_baro_->publish(msg);
    sensor_msgs::msg::Temperature temperature;
    temperature.header = msg.header;
    temperature.temperature = reading.temperature_c;
    pub_baro_temp_->publish(temperature);
    std_msgs::msg::Float64 isa;
    isa.data = reading.altitude_isa_m;
    pub_alt_isa_->publish(isa);
    std_msgs::msg::Float64 relative;
    relative.data = reading.altitude_relative_m;
    pub_alt_rel_->publish(relative);
    last_baro_ = Clock::now();
  }
  void publishStatus() {
    const auto age = [](const std::optional<Time>& received) {
      if (!received) return -1.0;
      return std::chrono::duration<double>(Clock::now() - *received).count();
    };
    const double imu_age = age(last_imu_);
    const double mag_age = age(last_mag_);
    const double baro_age = age(last_baro_);
    const double motor_age = age(last_motor_);
    const bool att = orientation_ &&
        std::chrono::duration<double>(Clock::now() - orientation_received_).count() < orientation_timeout_s_;
    std::ostringstream stream;
    stream << std::boolalpha << '{'
      << "\"hardware_interface_version\":" << jsonQuote(kHardwareVersion) << ','
      << "\"serial_connected\":" << serial_.connected() << ','
      << "\"device\":" << jsonQuote(port_) << ','
      << "\"protocol\":" << jsonQuote(protocol_mode_) << ','
      << "\"command_tx_enabled\":" << can_tx_ << ','
      << "\"armed_requested\":" << arm_requested_ << ','
      << "\"armed_acknowledged\":" << arm_confirmed_ << ','
      << "\"gui_output_enabled\":" << controller_enabled_ << ','
      << "\"rpm_to_dshot_enabled\":" << rpm_to_dshot_enabled_ << ','
      << "\"allow_direct_dshot\":" << allow_direct_dshot_ << ','
      << "\"manual_mode_acknowledged\":" << manual_mode_confirmed_ << ','
      << "\"firmware_fault_latched\":" << fault_latched_ << ','
      << "\"mcu_fmu_state\":" << mcu_fmu_state_ << ','
      << "\"mcu_auto_enabled\":" << mcu_auto_enabled_ << ','
      << "\"last_command_ack\":" << jsonQuote(last_ack_) << ','
      << "\"gui_setpoint_age_s\":" << manual_stream_.guiAgeSeconds(Clock::now()) << ','
      << "\"last_dshot_tx_age_s\":" << manual_stream_.txAgeSeconds(Clock::now()) << ','
      << "\"gui_setpoint_timeout_s\":" << command_timeout_s_ << ','
      << "\"manual_keepalive_period_s\":" << manual_keepalive_period_s_ << ','
      << "\"gui_setpoints_received\":" << gui_setpoints_received_ << ','
      << "\"manual_keepalive_sent\":" << manual_refresh_sent_ << ','
      << "\"gui_timeout_stops\":" << gui_timeout_stops_ << ','
      << "\"dshot_denials\":" << dshot_denials_ << ','
      << "\"orientation_available\":" << att << ','
      << "\"imu_online\":" << (imu_age >= 0 && imu_age <= sensor_stale_s_) << ','
      << "\"mag_online\":" << (mag_age >= 0 && mag_age <= sensor_stale_s_) << ','
      << "\"baro_online\":" << (baro_age >= 0 && baro_age <= sensor_stale_s_) << ','
      << "\"last_imu_age_s\":" << imu_age << ','
      << "\"last_mag_age_s\":" << mag_age << ','
      << "\"last_baro_age_s\":" << baro_age << ','
      << "\"last_motor_age_s\":" << motor_age << ','
      << "\"rx_valid_frames\":" << rx_frames_ << ','
      << "\"rx_invalid_frames\":" << bad_frames_ + lines_.dropped() << ','
      << "\"serial_errors\":" << serial_errors_ << ','
      << "\"sensor_error\":" << jsonQuote(last_sensor_error_) << ','
      << "\"mcu_status\":" << jsonQuote(last_mcu_status_) << ','
      << "\"last_commanded_dshot\":[" << last_dshot_[0] << ','
      << last_dshot_[1] << ',' << last_dshot_[2] << ',' << last_dshot_[3] << "],"
      << "\"timestamp_source\":\"Raspberry Pi ROS receive time\"}";
    std_msgs::msg::String msg;
    msg.data = stream.str();
    pub_status_->publish(msg);
  }

  std::string port_;
  int baudrate_{115200};
  std::string protocol_mode_;
  bool enable_command_tx_{false};
  bool allow_direction_commands_{false};  // kept for existing YAML, firmware ignores reversal
  bool rpm_to_dshot_enabled_{false};
  bool allow_direct_dshot_{false};
  ManualMotorMap manual_map_{};
  bool manual_mode_confirmed_{false};
  bool mcu_status_seen_{false};
  bool fault_latched_{false};
  bool reverse_requested_{false};
  bool mcu_auto_enabled_{false};
  int mcu_fmu_state_{-1};
  bool can_tx_{false};
  double reconnect_s_{2.0};
  double sensor_stale_s_{1.5};
  double command_timeout_s_{0.60};
  double manual_keepalive_period_s_{0.040};
  double orientation_timeout_s_{0.2};
  double max_command_rpm_{12000.0};
  std::string imu_frame_, mag_frame_, baro_frame_, body_frame_;
  SerialPort serial_;
  LineBuffer lines_;
  Time reconnect_at_{};
  std::deque<std::string> tx_queue_;
  std::string pending_tx_;
  std::size_t pending_offset_{0};
  std::uint16_t sequence_{0};
  bool arm_requested_{false};
  bool arm_confirmed_{false};
  bool controller_enabled_{false};
  std::optional<std::uint16_t> arm_ack_sequence_;
  std::optional<std::uint16_t> mode_ack_sequence_;
  std::optional<std::uint16_t> reset_fault_ack_sequence_;
  std::optional<Time> arm_confirmed_at_, last_mcu_stat_;
  ManualKeepalive manual_stream_{};
  std::array<std::uint16_t,4> last_dshot_{};
  std::string last_ack_{"NONE"};
  Time arm_ack_deadline_{};
  std::optional<Time> last_imu_, last_mag_, last_baro_, last_motor_;
  std::optional<std::array<double,4>> orientation_;
  Time orientation_received_{};
  std::array<double,4> requested_rpm_{};
  std::uint64_t rx_frames_{0}, bad_frames_{0}, serial_errors_{0};
  std::uint64_t gui_setpoints_received_{0}, manual_refresh_sent_{0};
  std::uint64_t gui_timeout_stops_{0}, dshot_denials_{0};
  std::string last_sensor_error_, last_mcu_status_{"NO MCU STATUS"};

  rclcpp::TimerBase::SharedPtr io_timer_, status_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_baro_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_imu_temp_, pub_baro_temp_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_alt_isa_, pub_alt_rel_;
  rclcpp::Publisher<cefiro_msgs::msg::MotorRpm>::SharedPtr pub_rpm_;
  rclcpp::Publisher<cefiro_msgs::msg::MotorDshot>::SharedPtr pub_commanded_dshot_;
  rclcpp::Publisher<cefiro_msgs::msg::MotorTelemetry>::SharedPtr pub_motor_telemetry_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_, pub_controller_, pub_ack_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
  rclcpp::Subscription<cefiro_msgs::msg::MotorRpm>::SharedPtr sub_rpm_;
  rclcpp::Subscription<cefiro_msgs::msg::MotorDshot>::SharedPtr sub_dshot_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_armed_, sub_enable_, sub_estop_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr sub_direction_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_attitude_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_, sub_strategy_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pid_, sub_lqr_;
};
}  // namespace cefiro::hardware

int main(int argc, char** argv) {
  if (argc == 2 && std::string_view(argv[1]) == "--version") {
    std::cout << "cefiro_hardware_interface 0.4.1\n";
    return 0;
  }
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<cefiro::hardware::HardwareNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("cefiro_hardware_interface"), "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
