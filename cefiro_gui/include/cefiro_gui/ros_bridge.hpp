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
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

namespace cefiro::gui {
class RosBridge final {
public:
    RosBridge();
    ~RosBridge();
    RosBridge(const RosBridge&)=delete;
    RosBridge& operator=(const RosBridge&)=delete;
    void start();
    void stop();
    std::vector<Telemetry> drain();
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
    void onImu(const Imu& msg);
    void onMag(const Mag& msg);
    void onBaro(const Baro& msg);
    void onStatus(const std_msgs::msg::String& msg);
    void onHardwareStatus(const std_msgs::msg::String& msg);
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<Mag>::SharedPtr sub_mag_;
    rclcpp::Subscription<Baro>::SharedPtr sub_baro_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_hardware_status_;
    rclcpp::Publisher<Ref>::SharedPtr pub_reference_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enabled_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_strategy_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pid_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_lqr_;
    rclcpp::Publisher<cefiro_msgs::msg::MotorRpm>::SharedPtr pub_motor_rpm_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr pub_motor_direction_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_motor_armed_;
    std::thread worker_;
    mutable std::mutex mutex_;
    std::deque<Telemetry> queue_;
    Telemetry latest_{};
    Vec3 latest_reference_{};
    double first_pressure_pa_{0.0};
    std::chrono::steady_clock::time_point imu_rx_{},mag_rx_{},baro_rx_{},status_rx_{},hardware_status_rx_{};
    std::string status_{"NO CONTROLLER FEEDBACK"};
    std::string hardware_status_{""};
    std::string imu_topic_,mag_topic_,baro_topic_,reference_topic_;
    std::string motor_rpm_topic_,motor_direction_topic_;
    bool running_{false};
};
}
