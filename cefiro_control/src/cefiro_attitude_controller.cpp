#include "cefiro_attitude_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <numbers>

#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/logging.hpp>

namespace cefiro::control {

controller_interface::InterfaceConfiguration
CefiroAttitudeController::command_interface_configuration () const {
	controller_interface::InterfaceConfiguration config;

	config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.clear();

	for (const auto & joint : joints_) {
        config.names.push_back(joint + "/" + interface_name_);
    }

	return config;
}

controller_interface::InterfaceConfiguration
CefiroAttitudeController::state_interface_configuration () const {
    controller_interface::InterfaceConfiguration config;

    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.clear();

    const std::vector<std::string> imu_interfaces = {
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
        "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"
    };
    for (const auto & interface : imu_interfaces) {
        config.names.push_back(imu_sensor_name_ + "/" + interface);
    }

    const std::vector<std::string> mag_interfaces = {
        "magnetic_field.x", "magnetic_field.y", "magnetic_field.z"
    };
    for (const auto & interface : mag_interfaces) {
        config.names.push_back(magnetometer_sensor_name_ + "/" + interface);
    }

    return config;
}

controller_interface::CallbackReturn
CefiroAttitudeController::on_init () {
    try {
        // --- 1. Declaración de parámetros estándar del controlador ---
        auto_declare<std::vector<std::string>> ( "joints", {} );
        auto_declare<std::string> ( "interface_name", "rpm" );
        auto_declare<std::string> ( "reference_topic", "~/reference" );

        auto_declare<std::string> ( "imu_sensor_name", "imu_sensor" );
        auto_declare<std::string> ( "magnetometer_sensor_name", "magnetometer_sensor" );

        auto_declare<std::vector<double>> ( "gains.control_gains.k_matrix", {} );
        auto_declare<int> ( "gains.control_gains.k_rows", 0 );
        auto_declare<int> ( "gains.control_gains.k_cols", 0 );

        auto_declare<std::vector<double>> ( "gains.observer_gains.l_matrix", {} );
        auto_declare<int> ( "gains.observer_gains.l_rows", 0 );
        auto_declare<int> ( "gains.observer_gains.l_cols", 0 );

        auto_declare<double> ( "limits.min_output", 320.0 );
        auto_declare<double> ( "limits.max_output", 1600.0 );
        auto_declare<double> ( "limits.max_torque", 0.01 );
        auto_declare<double> ( "nominal_rpm", 500.0 );

        // --- 2. Declaración de parámetros físicos del dron (Sin masa) ---
        auto_declare<double> ( "inertia.jx", 0.01 );
        auto_declare<double> ( "inertia.jy", 0.01 );
        auto_declare<double> ( "inertia.jz", 0.02 );

        auto_declare<double> ( "mixer.arm_length", 0.25 );
        auto_declare<double> ( "mixer.thrust_coeff", 1.2e-6 );
        auto_declare<double> ( "mixer.drag_coeff", 2.5e-8 );
        auto_declare<std::string> ( "mixer.frame_type", "cross" );
        auto_declare<double> ( "mixer.arm_angle_deg", 45.0 );

        // Signos del mixer por motor [Roll, Pitch, Yaw]
        auto_declare<std::vector<double>> ( "mixer.motor_matrix_signs.motor_1", {} );
        auto_declare<std::vector<double>> ( "mixer.motor_matrix_signs.motor_2", {} );
        auto_declare<std::vector<double>> ( "mixer.motor_matrix_signs.motor_3", {} );
        auto_declare<std::vector<double>> ( "mixer.motor_matrix_signs.motor_4", {} );

        reference_buffer_.writeFromNonRT ( std::vector<double>{ 0.0, 0.0, 0.0 } );
    } catch ( const std::exception &exception ) {
        RCLCPP_ERROR ( get_node ()->get_logger (), "Failed to initialize CEFIRO: %s", exception.what () );
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CefiroAttitudeController::on_configure ( const rclcpp_lifecycle::State & /* previous_state */ ) {
    
    // --- 1. Extracción de parámetros estándar ---
    joints_ = get_node ()->get_parameter ( "joints" ).as_string_array ();
    interface_name_ = get_node ()->get_parameter ( "interface_name" ).as_string ();
    reference_topic_ = get_node ()->get_parameter ( "reference_topic" ).as_string ();

    imu_sensor_name_ = get_node ()->get_parameter ( "imu_sensor_name" ).as_string ();
    magnetometer_sensor_name_ = get_node ()->get_parameter ( "magnetometer_sensor_name" ).as_string ();

    k_rows_ = static_cast<int> ( get_node ()->get_parameter ( "gains.control_gains.k_rows" ).as_int () );
    k_cols_ = static_cast<int> ( get_node ()->get_parameter ( "gains.control_gains.k_cols" ).as_int () );
    l_rows_ = static_cast<int> ( get_node ()->get_parameter ( "gains.observer_gains.l_rows" ).as_int () );
    l_cols_ = static_cast<int> ( get_node ()->get_parameter ( "gains.observer_gains.l_cols" ).as_int () );

    min_output_ = get_node ()->get_parameter ( "limits.min_output" ).as_double ();
    max_output_ = get_node ()->get_parameter ( "limits.max_output" ).as_double ();
    max_torque_ = get_node ()->get_parameter ( "limits.max_torque" ).as_double ();
    nominal_rpm_ = get_node ()->get_parameter ( "nominal_rpm" ).as_double ();

    // Lectura temporal de ganancias desde YAML
    std::vector<double> k_vec = get_node ()->get_parameter ( "gains.control_gains.k_matrix" ).as_double_array ();
    std::vector<double> l_vec = get_node ()->get_parameter ( "gains.observer_gains.l_matrix" ).as_double_array ();

    // --- 2. Extracción de parámetros físicos ---
    jx_ = get_node ()->get_parameter ( "inertia.jx" ).as_double ();
    jy_ = get_node ()->get_parameter ( "inertia.jy" ).as_double ();
    jz_ = get_node ()->get_parameter ( "inertia.jz" ).as_double ();

    arm_length_ = get_node ()->get_parameter ( "mixer.arm_length" ).as_double ();
    thrust_coeff_ = get_node ()->get_parameter ( "mixer.thrust_coeff" ).as_double ();
    drag_coeff_ = get_node ()->get_parameter ( "mixer.drag_coeff" ).as_double ();
    frame_type_ = get_node ()->get_parameter ( "mixer.frame_type" ).as_string ();
    double arm_angle_deg = get_node ()->get_parameter ( "mixer.arm_angle_deg" ).as_double ();
    arm_angle_rad_ = arm_angle_deg * ( std::numbers::pi / 180.0 );

    std::vector<double> signs_m1 = get_node ()->get_parameter ( "mixer.motor_matrix_signs.motor_1" ).as_double_array ();
    std::vector<double> signs_m2 = get_node ()->get_parameter ( "mixer.motor_matrix_signs.motor_2" ).as_double_array ();
    std::vector<double> signs_m3 = get_node ()->get_parameter ( "mixer.motor_matrix_signs.motor_3" ).as_double_array ();
    std::vector<double> signs_m4 = get_node ()->get_parameter ( "mixer.motor_matrix_signs.motor_4" ).as_double_array ();

    // --- 3. Validaciones de Seguridad ---
    if ( joints_.size () != 4U || interface_name_ != "rpm" || reference_topic_.empty () ) {
        RCLCPP_ERROR ( get_node ()->get_logger (), "Basic interface or joint validation failed." );
        return controller_interface::CallbackReturn::FAILURE;
    }

    if ( k_rows_ != 3 || k_cols_ != 6 || k_vec.size () != 18U ) {
        RCLCPP_ERROR ( get_node ()->get_logger (), "Control gain matrix K must be 3x6 (18 elements)." );
        return controller_interface::CallbackReturn::FAILURE;
    }

    if ( l_rows_ != 6 || l_cols_ != 6 || l_vec.size () != 36U ) {
        RCLCPP_ERROR ( get_node ()->get_logger (), "Observer gain matrix L must be 6x6 (36 elements)." );
        return controller_interface::CallbackReturn::FAILURE;
    }

    if ( !std::isfinite ( min_output_ ) || !std::isfinite ( max_output_ ) || min_output_ >= max_output_ ) {
        RCLCPP_ERROR ( get_node ()->get_logger (), "Output limits are invalid." );
        return controller_interface::CallbackReturn::FAILURE;
    }

    if ( !std::isfinite ( max_torque_ ) || max_torque_ <= 0.0 ) {
        RCLCPP_ERROR ( get_node ()->get_logger (), "'limits.max_torque' must be positive and finite." );
        return controller_interface::CallbackReturn::FAILURE;
    }

    motor_signs_.clear ();
    for ( const auto *signs : { &signs_m1, &signs_m2, &signs_m3, &signs_m4 } ) {
        if ( signs->size () != 3U ) return controller_interface::CallbackReturn::FAILURE;
        motor_signs_.insert ( motor_signs_.end (), signs->begin (), signs->end () );
    }

    // --- 4. Inicialización de Matrices de Eigen ---
    // Mapeo directo de K y L desde los vectores temporales del YAML
    k_matrix_ = Eigen::Map<const Eigen::Matrix<double, 3, 6, Eigen::RowMajor>>( k_vec.data() );
    l_matrix_ = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>( l_vec.data() );

    // Matriz A (Cinemática: derivadas de ángulos son velocidades angulares)
    a_matrix_.setZero();
    a_matrix_(0, 3) = 1.0;
    a_matrix_(1, 4) = 1.0;
    a_matrix_(2, 5) = 1.0;

    // Matriz B (Dinámica: torques afectados por la inercia)
    b_matrix_.setZero();
    b_matrix_(3, 0) = 1.0 / jx_;
    b_matrix_(4, 1) = 1.0 / jy_;
    b_matrix_(5, 2) = 1.0 / jz_;

    // Matriz C (Observación directa de los 6 estados)
    c_matrix_.setIdentity();

    // --- 5. Construcción e Inversión de la Matriz Mixer ---
    Eigen::Matrix4d mixer_matrix;
    double k = arm_length_ * thrust_coeff_;
    if ( frame_type_ == "cross" ) k *= std::sin ( arm_angle_rad_ );

    for ( int i = 0; i < 4; ++i ) {
        int sign_idx = i * 3;
        mixer_matrix ( 0, i ) = thrust_coeff_;                              
        mixer_matrix ( 1, i ) = motor_signs_[sign_idx] * k;                 
        mixer_matrix ( 2, i ) = motor_signs_[sign_idx + 1] * k;             
        mixer_matrix ( 3, i ) = motor_signs_[sign_idx + 2] * drag_coeff_;   
    }

    inverse_mixer_matrix_ = mixer_matrix.inverse ();

    // --- 6. Suscriptor de Referencias ---
    reference_subscriber_ = get_node ()->create_subscription<std_msgs::msg::Float64MultiArray> (
        reference_topic_, 10, [this] ( const std_msgs::msg::Float64MultiArray::SharedPtr message ) {
            if ( message->data.size () == 3U ) {
                reference_buffer_.writeFromNonRT ( message->data );
            }
        } );

    reference_buffer_.writeFromNonRT ( std::vector<double>{ 0.0, 0.0, 0.0 } );

    RCLCPP_INFO ( get_node ()->get_logger (), "CefiroAttitudeController successfully configured." );
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CefiroAttitudeController::on_activate ( const rclcpp_lifecycle::State & /* previous_state */ ) {

    // 1. Verificación de interfaces de comando (4 motores)
    if ( command_interfaces_.size () != 4U ) {
        RCLCPP_ERROR ( get_node ()->get_logger (), "Expected 4 command interfaces, got %zu.", command_interfaces_.size () );
        return controller_interface::CallbackReturn::ERROR;
    }

    // 2. Verificación de interfaces de estado (IMU:6 + Mag:3 = 9) - ¡Modificado por eliminación de Baro!
    if ( state_interfaces_.size () != 9U ) {
        RCLCPP_ERROR ( get_node ()->get_logger (), "Expected exactly 9 state interfaces, got %zu.", state_interfaces_.size () );
        return controller_interface::CallbackReturn::ERROR;
    }

    // 3. Reinicio de la memoria del controlador
    observer_initialized_ = false;
    std::fill ( estimated_state_.begin (), estimated_state_.end (), 0.0 );
    reference_buffer_.writeFromNonRT ( std::vector<double>{ 0.0, 0.0, 0.0 } );

    // 4. Inicialización segura de los comandos al régimen base
    for ( size_t i = 0; i < command_interfaces_.size (); ++i ) {
        if ( !command_interfaces_[i].set_value ( nominal_rpm_ ) ) {
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO ( get_node ()->get_logger (), "CefiroAttitudeController successfully activated." );
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
CefiroAttitudeController::update ( const rclcpp::Time & /* time */, const rclcpp::Duration &period ) {

    // --- 1. Control de periodo ---
    const double dt = period.seconds ();
    if ( !std::isfinite ( dt ) || dt <= 0.0 ) {
        return controller_interface::return_type::OK;
    }

    // --- 2. Lectura del buffer de referencias (Roll, Pitch, Yaw) ---
    const std::vector<double> *reference = reference_buffer_.readFromRT ();
    if ( reference == nullptr || reference->size () != 3U ) {
        return controller_interface::return_type::ERROR;
    }

    // Vector de referencia deseado r = [phi_ref, theta_ref, psi_ref, p_ref, q_ref, r_ref]
    Eigen::VectorXd r_ref(6);
    r_ref << (*reference)[0], (*reference)[1], (*reference)[2], 0.0, 0.0, 0.0;

    // --- 3. Extracción de mediciones de hardware ---
    // Interfaces de estado: IMU [0..5], Mag [6..8]
    const auto p_opt  = state_interfaces_[0].get_optional<double> ();
    const auto q_opt  = state_interfaces_[1].get_optional<double> ();
    const auto r_opt  = state_interfaces_[2].get_optional<double> ();
    const auto ax_opt = state_interfaces_[3].get_optional<double> ();
    const auto ay_opt = state_interfaces_[4].get_optional<double> ();
    const auto az_opt = state_interfaces_[5].get_optional<double> ();

    const auto mx_opt = state_interfaces_[6].get_optional<double> ();
    const auto my_opt = state_interfaces_[7].get_optional<double> ();
    const auto mz_opt = state_interfaces_[8].get_optional<double> ();

    if ( !p_opt || !q_opt || !r_opt || !ax_opt || !ay_opt || !az_opt || !mx_opt || !my_opt || !mz_opt ) {
        RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000, // Imprime como máximo UNA VEZ cada 1000 ms (1 segundo)
        "Sensors not ready yet. Skipping update cycle..."
        );
        return controller_interface::return_type::OK; // Sensores no listos aún
    }

    const double p  = *p_opt;
    const double q  = *q_opt;
    const double r_gyro = *r_opt;
    const double ax = *ax_opt;
    const double ay = *ay_opt;
    const double az = *az_opt;
    const double mx = *mx_opt;
    const double my = *my_opt;
    const double mz = *mz_opt;

    // --- 4. Trigonometría y Estimación de la Actitud (Vector y_sensors) ---
    const double roll = std::atan2 ( ay, az );
    const double pitch = std::atan2 ( -ax, std::hypot ( ay, az ) );

    const double cos_roll = std::cos ( roll );
    const double sin_roll = std::sin ( roll );
    const double cos_pitch = std::cos ( pitch );
    const double sin_pitch = std::sin ( pitch );

    const double mag_x = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
    const double mag_y = my * cos_roll - mz * sin_roll;
    const double yaw = std::atan2 ( -mag_y, mag_x );

    Eigen::VectorXd y_sensors(6);
    y_sensors << roll, pitch, yaw, p, q, r_gyro;

    // --- 5. Inicialización de la memoria del Observador de Luenberger ---
    Eigen::VectorXd x_hat(6);
    for ( int i = 0; i < 6; ++i ) {
        x_hat(i) = estimated_state_[i];
    }

    if ( !observer_initialized_ ) {
        x_hat = y_sensors;
        for ( int i = 0; i < 6; ++i ) {
            estimated_state_[i] = x_hat(i);
        }
        observer_initialized_ = true;
    }

    // --- 6. Ley de Control: u = -K * (x_hat - r) ---
    Eigen::VectorXd error = x_hat - r_ref;
    error(2) = std::remainder ( error(2), 2.0 * std::numbers::pi_v<double> );

    Eigen::VectorXd torques = -k_matrix_ * error;

    // Limitación de torque
    for ( int i = 0; i < 3; ++i ) {
        torques(i) = std::clamp ( torques(i), -max_torque_, max_torque_ );
    }

    // --- 7. Mixer: Conversión de Torques a Velocidades de Motor ---
    Eigen::VectorXd wrench(4);
    wrench << 0.0, torques(0), torques(1), torques(2);

    Eigen::VectorXd delta_w_sq = inverse_mixer_matrix_ * wrench;
    const double rpm_base_sq = nominal_rpm_ * nominal_rpm_;

    std::vector<double> rpm_commands(4, 0.0);
    for ( int i = 0; i < 4; ++i ) {
        double final_w_sq = rpm_base_sq + delta_w_sq(i);
        double rpm = std::sqrt ( std::max ( 0.0, final_w_sq ) );
        rpm_commands[i] = std::clamp ( rpm, min_output_, max_output_ );
    }

    // --- 8. Propagación del Observador de Luenberger ---
    Eigen::VectorXd y_hat = c_matrix_ * x_hat;
    Eigen::VectorXd innovation = y_sensors - y_hat;
    innovation(2) = std::remainder ( innovation(2), 2.0 * std::numbers::pi_v<double> );

    // x_dot = A * x_hat + B * u + L * (y - C * x_hat)
    Eigen::VectorXd x_dot = a_matrix_ * x_hat + b_matrix_ * torques + l_matrix_ * innovation;

    x_hat += x_dot * dt;
    x_hat(2) = std::remainder ( x_hat(2), 2.0 * std::numbers::pi_v<double> );

    for ( int i = 0; i < 6; ++i ) {
        estimated_state_[i] = x_hat(i);
    }

    // --- 9. Envío de Comandos a las Interfaces de Actuación ---
    for ( size_t i = 0; i < 4U; ++i ) {
        if ( !command_interfaces_[i].set_value ( rpm_commands[i] ) ) {
            return controller_interface::return_type::OK;
        }
    }

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
CefiroAttitudeController::on_deactivate ( const rclcpp_lifecycle::State & /* previous_state */ ) {
    
    // 1. Apagar los 4 motores de forma segura
    for ( size_t i = 0; i < command_interfaces_.size (); ++i ) {
        if ( !command_interfaces_[i].set_value ( 0.0 ) ) {
            RCLCPP_ERROR (
                get_node ()->get_logger (),
                "Failed to set motor %zu to 0 RPM during deactivation.", i + 1 );

            return controller_interface::CallbackReturn::ERROR;
        }
    }

    // 2. Limpiar el buffer de referencias de actitud
    reference_buffer_.writeFromNonRT ( std::vector<double>{ 0.0, 0.0, 0.0 } );

    // 3. Reiniciar la memoria del observador de Luenberger
    observer_initialized_ = false;
    std::fill ( estimated_state_.begin (), estimated_state_.end (), 0.0 );

    RCLCPP_INFO ( get_node ()->get_logger (), "CefiroAttitudeController successfully deactivated." );

    return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace cefiro::control

PLUGINLIB_EXPORT_CLASS (
	cefiro::control::CefiroAttitudeController, controller_interface::ControllerInterface )