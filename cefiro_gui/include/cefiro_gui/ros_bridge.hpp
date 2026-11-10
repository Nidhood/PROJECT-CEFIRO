#pragma once
#include "cefiro_gui/models.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <chrono>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <cefiro_msgs/msg/motor_rpm.hpp>
#include <cefiro_msgs/msg/motor_dshot.hpp>
#include <cefiro_msgs/msg/motor_telemetry.hpp>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

namespace cefiro::gui {
// Command/setpoint, transport request, and measurement are deliberately distinct.
enum class MotorEventKind { TargetRpm=0, DirectDshot=1, MeasuredRpm=2,
                             Telemetry=3, HostCommandedDshot=4 };
struct MotorEvent {
    MotorEventKind kind{MotorEventKind::TargetRpm};
    bool from_real{false};
    std::array<double,4> requested_rpm{};
    std::array<double,4> measured_rpm{};
    std::array<std::uint16_t,4> dshot{};
    std::uint8_t command_source{0};
};
class RosBridge final {
public:
    RosBridge();
    ~RosBridge();
    RosBridge(const RosBridge&)=delete;
    RosBridge& operator=(const RosBridge&)=delete;
    void start();
 void selectReal(bool real);
 bool isReal() const;
    bool imuFreshFor(bool real, double timeout_s=1.5) const;
    bool magFreshFor(bool real, double timeout_s=2.5) const;
    bool baroFreshFor(bool real, double timeout_s=2.5) const;
    bool sourceFresh(bool real) const;
    bool simReferenceEnvelope(Vec3& limits_deg) const;
    void resetTelemetry(bool real);
    void stop();
    std::vector<Telemetry> drain();
    std::vector<MotorEvent> drainMotor();
    bool motorFreshFor(bool real,double timeout_s=2.0) const;
    bool imuFresh(double timeout_s=1.5) const;
    bool magFresh(double timeout_s=2.5) const;
    bool baroFresh(double timeout_s=2.5) const;
    void publishReference(const Vec3& deg, const std::string& mode, const std::string& strategy);
    void publishMotorCommand(const std::array<double,4>& rpm, const std::array<int,4>& direction);
    void setMotorArmed(bool armed);
    void setEnabled(bool enabled);
    void publishGains(const std::vector<double>& pid, const std::vector<double>& lqr);
    void emergencyStop();
    std::string controllerStatus() const;
    bool feedbackFresh(double timeout_s=2.0) const;
    bool hardwareStatusFresh(double timeout_s=2.0) const;
    std::string hardwareStatus() const;
    std::string imuTopic() const {return imu_topic_;}
    std::string magTopic() const {return mag_topic_;}
    std::string baroTopic() const {return baro_topic_;}
    std::string refTopic() const {return reference_topic_;}
private:
    using Imu = sensor_msgs::msg::Imu;
    using Mag = sensor_msgs::msg::MagneticField;
    using Baro = sensor_msgs::msg::FluidPressure;
    using Ref = geometry_msgs::msg::Vector3Stamped;
    using MotorRpm = cefiro_msgs::msg::MotorRpm;
    using MotorDshot = cefiro_msgs::msg::MotorDshot;
    using MotorTelemetry = cefiro_msgs::msg::MotorTelemetry;
    void onMotorRpm(const MotorRpm& msg,bool real,bool measured);
    void onMotorDshot(const MotorDshot& msg,bool real,bool host_confirmed);
    void onMotorTelemetry(const MotorTelemetry& msg,bool real);
    void pushMotor(MotorEvent event);
    void onImu(const Imu& msg,bool real);
    void onMag(const Mag& msg,bool real);
    void onBaro(const Baro& msg,bool real);
    void onStatus(const std_msgs::msg::String& msg, bool real);
    void onHardwareStatus(const std_msgs::msg::String& msg);
    void onReferenceEnvelope(const std_msgs::msg::Float64MultiArray& msg);
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Subscription<Imu>::SharedPtr sub_imu_,sub_real_imu_;
    rclcpp::Subscription<Mag>::SharedPtr sub_mag_,sub_real_mag_;
    rclcpp::Subscription<Baro>::SharedPtr sub_baro_,sub_real_baro_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_real_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_hardware_status_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_reference_envelope_;
    std::array<rclcpp::Subscription<MotorRpm>::SharedPtr,2> sub_target_rpm_{},sub_measured_rpm_{};
    std::array<rclcpp::Subscription<MotorDshot>::SharedPtr,2> sub_direct_dshot_{};
    rclcpp::Subscription<MotorDshot>::SharedPtr sub_real_commanded_dshot_;
    std::array<rclcpp::Subscription<MotorTelemetry>::SharedPtr,2> sub_motor_telemetry_{};
    rclcpp::Publisher<Ref>::SharedPtr pub_reference_,pub_real_reference_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enabled_,pub_real_enabled_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop_,pub_real_stop_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_,pub_real_mode_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_strategy_,pub_real_strategy_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pid_,pub_real_pid_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_lqr_,pub_real_lqr_;
    rclcpp::Publisher<cefiro_msgs::msg::MotorRpm>::SharedPtr pub_motor_rpm_,pub_real_motor_rpm_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr pub_motor_direction_,pub_real_motor_direction_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_motor_armed_,pub_real_motor_armed_;
    std::thread worker_;
    mutable std::mutex mutex_;
    std::deque<Telemetry> queue_;
    std::deque<MotorEvent> motor_queue_;
    std::array<std::chrono::steady_clock::time_point,2> motor_rx_{};
    Telemetry latest_{},latest_real_{};
 bool real_selected_{false};
 double real_initial_pressure_{0};
 std::chrono::steady_clock::time_point real_imu_rx_{},real_mag_rx_{},real_baro_rx_{};
    std::array<Vec3,2> latest_reference_{};
    double first_pressure_pa_{0.0};
    std::chrono::steady_clock::time_point imu_rx_{},mag_rx_{},baro_rx_{},status_rx_{},hardware_status_rx_{},real_status_rx_{};
    std::string status_{"NO CONTROLLER FEEDBACK"};
    std::string real_status_{"NO REAL CONTROLLER FEEDBACK"};
    std::string hardware_status_{""};
    Vec3 sim_reference_envelope_deg_{{5.0,5.0,10.0}};
    bool have_sim_reference_envelope_{false};
    std::string imu_topic_,mag_topic_,baro_topic_,reference_topic_;
    std::string motor_rpm_topic_,motor_direction_topic_;
    bool display_relative_attitude_{false};
    double equilibrium_world_yaw_deg_{151.318};
    bool running_{false};
};
}
