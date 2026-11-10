#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "cefiro_msgs/msg/motor_rpm.hpp"

namespace cefiro::control
{

    class CefiroAttitudeController final : public controller_interface::ControllerInterface
    {
    private:
        std::vector<std::string> joints_{};
        std::string interface_name_{"rpm"};
        std::string reference_topic_{"~/reference"};
        std::string imu_sensor_name_{"imu_sensor"};
        std::string magnetometer_sensor_name_{"magnetometer_sensor"};
        int k_rows_{0};
        int k_cols_{0};
        int l_rows_{0};
        int l_cols_{0};
        double min_output_{0.0};
        double max_output_{0.0};
        double nominal_rpm_{0.0};
        double max_torque_{0.0};
        double jx_{0.00};
        double jy_{0.00};
        double jz_{0.00};
        double arm_length_{0.00};
        double thrust_coeff_{0.00};
        double drag_coeff_{0.00};
        std::string frame_type_{"cross"};
        double arm_angle_rad_{0.00};
        std::vector<double> motor_signs_{};

        Eigen::Matrix<double, 3, 6> k_matrix_{Eigen::Matrix<double, 3, 6>::Zero()};
        Eigen::Matrix<double, 6, 6> l_matrix_{Eigen::Matrix<double, 6, 6>::Zero()};
        Eigen::Matrix<double, 6, 6> a_matrix_{Eigen::Matrix<double, 6, 6>::Zero()};
        Eigen::Matrix<double, 6, 3> b_matrix_{Eigen::Matrix<double, 6, 3>::Zero()};
        Eigen::Matrix<double, 6, 6> c_matrix_{Eigen::Matrix<double, 6, 6>::Zero()};
        Eigen::Matrix<double, 4, 4> inverse_mixer_matrix_{Eigen::Matrix<double, 4, 4>::Zero()};

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr reference_subscriber_;
        realtime_tools::RealtimeBuffer<std::vector<double>> reference_buffer_;

        std::vector<double> estimated_state_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        bool observer_initialized_{false};

    public:
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    };

} // namespace cefiro::control