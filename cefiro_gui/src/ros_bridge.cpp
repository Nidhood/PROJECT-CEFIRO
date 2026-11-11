#include "cefiro_gui/ros_bridge.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <cctype>
#include <charconv>
#include <system_error>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
using namespace std::chrono_literals;
namespace cefiro::gui {
namespace {
double stampSeconds(const builtin_interfaces::msg::Time& t) { return double(t.sec)+double(t.nanosec)*1.e-9; }
bool fresh(std::chrono::steady_clock::time_point t, double timeout) {
    return t.time_since_epoch().count()!=0 && std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count()<timeout;
}
}
RosBridge::RosBridge() {
    node_=std::make_shared<rclcpp::Node>("cefiro_hmi");
    display_relative_attitude_=node_->declare_parameter<bool>("display_relative_attitude",false);
    // Support both numeric and quoted-string YAML values without crashing the GUI.
    // The parameter is an angle in DEGREES, not an actuator command.
    rcl_interfaces::msg::ParameterDescriptor yaw_descriptor;
    yaw_descriptor.description = "Nominal world yaw in degrees for relative attitude display";
    yaw_descriptor.dynamic_typing = true;
    node_->declare_parameter(
        "equilibrium_world_yaw_deg", rclcpp::ParameterValue(151.318), yaw_descriptor);
    const auto yaw_parameter = node_->get_parameter("equilibrium_world_yaw_deg");
    try {
        if (yaw_parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            equilibrium_world_yaw_deg_ = yaw_parameter.as_double();
        } else if (yaw_parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            equilibrium_world_yaw_deg_ = static_cast<double>(yaw_parameter.as_int());
        } else if (yaw_parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            // When ROS launch supplies an angle as a string, reject unknown
            // units rather than silently accepting an invalid configuration.
            // The YAML shipped with the GUI uses an unquoted numeric value.
            // std::stod follows the process C locale, which may use a comma
            // decimal separator. Under es_CO it can parse "151.318" as 151
            // and incorrectly report ".318" as a unit suffix.
            // from_chars is independent of the process locale.
            std::string raw = yaw_parameter.as_string();
            const auto is_space = [](unsigned char ch) { return std::isspace(ch) != 0; };
            raw.erase(raw.begin(),
                std::find_if_not(raw.begin(), raw.end(), is_space));
            const char* begin = raw.data();
            const char* end = begin + raw.size();
            double value = 0.0;
            const auto parsed = std::from_chars(begin, end, value,
                                                std::chars_format::general);
            if(parsed.ec != std::errc{} || parsed.ptr == begin) {
                throw std::invalid_argument("invalid numeric yaw parameter");
            }
            std::string suffix(parsed.ptr, end);
            suffix.erase(suffix.begin(),
                std::find_if_not(suffix.begin(), suffix.end(), is_space));
            suffix.erase(std::find_if_not(suffix.rbegin(), suffix.rend(), is_space).base(),
                suffix.end());
            if (suffix == "rad" || suffix == "radian" || suffix == "radians") {
                value *= rad2deg;
            } else if (!suffix.empty() && suffix != "deg" && suffix != "degree" &&
                       suffix != "degrees" && suffix != "°") {
                throw std::invalid_argument("unknown suffix in yaw parameter: '" +
                                            suffix + "'");
            }
            equilibrium_world_yaw_deg_ = value;
            RCLCPP_INFO(node_->get_logger(),
                "equilibrium_world_yaw_deg = %.3f deg (string override)",
                equilibrium_world_yaw_deg_);
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "equilibrium_world_yaw_deg has unsupported type; using 151.318 deg.");
        }
        if (!std::isfinite(equilibrium_world_yaw_deg_)) {
            RCLCPP_WARN(node_->get_logger(),
                "equilibrium_world_yaw_deg must be finite; using 151.318 deg.");
            equilibrium_world_yaw_deg_ = 151.318;
        }
    } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
            "Invalid equilibrium_world_yaw_deg override: %s; using 151.318 deg. "
            "Check other --params-file entries and launch overrides.", e.what());
        equilibrium_world_yaw_deg_ = 151.318;
    }
    imu_topic_=node_->declare_parameter<std::string>("imu_topic","/imu/data");
    mag_topic_=node_->declare_parameter<std::string>("magnetometer_topic","/magnetometer/data");
    baro_topic_=node_->declare_parameter<std::string>("barometer_topic","/bmp581/air_pressure");
    reference_topic_=node_->declare_parameter<std::string>("reference_topic","/cefiro/control/attitude_reference");
    auto enable_topic=node_->declare_parameter<std::string>("enable_topic","/cefiro/control/enable");
    auto stop_topic=node_->declare_parameter<std::string>("stop_topic","/cefiro/control/emergency_stop");
    auto mode_topic=node_->declare_parameter<std::string>("mode_topic","/cefiro/control/mode");
    auto strategy_topic=node_->declare_parameter<std::string>("strategy_topic","/cefiro/control/strategy");
    auto status_topic=node_->declare_parameter<std::string>("status_topic","/cefiro/control/status");
    sub_imu_=node_->create_subscription<Imu>(imu_topic_,rclcpp::SensorDataQoS(),[this](const Imu::SharedPtr m){onImu(*m,false);});
 auto real_imu_topic=node_->declare_parameter<std::string>("real_imu_topic","/real/imu/data");
 sub_real_imu_=node_->create_subscription<Imu>(real_imu_topic,rclcpp::SensorDataQoS(),[this](const Imu::SharedPtr m){onImu(*m,true);});
    sub_mag_=node_->create_subscription<Mag>(mag_topic_,rclcpp::SensorDataQoS(),[this](const Mag::SharedPtr m){onMag(*m,false);});
 auto real_mag_topic=node_->declare_parameter<std::string>("real_magnetometer_topic","/real/magnetometer/data");
 sub_real_mag_=node_->create_subscription<Mag>(real_mag_topic,rclcpp::SensorDataQoS(),[this](const Mag::SharedPtr m){onMag(*m,true);});
    sub_baro_=node_->create_subscription<Baro>(baro_topic_,rclcpp::SensorDataQoS(),[this](const Baro::SharedPtr m){onBaro(*m,false);});
 auto real_baro_topic=node_->declare_parameter<std::string>("real_barometer_topic","/real/bmp581/air_pressure");
 sub_real_baro_=node_->create_subscription<Baro>(real_baro_topic,rclcpp::SensorDataQoS(),[this](const Baro::SharedPtr m){onBaro(*m,true);});
    sub_status_=node_->create_subscription<std_msgs::msg::String>(status_topic,10,
        [this](const std_msgs::msg::String::SharedPtr m){onStatus(*m,false);});
    const auto real_status_topic=node_->declare_parameter<std::string>(
        "real_status_topic", "/real/cefiro/control/status");
    sub_real_status_=node_->create_subscription<std_msgs::msg::String>(real_status_topic,10,
        [this](const std_msgs::msg::String::SharedPtr m){onStatus(*m,true);});
    auto hardware_status_topic=node_->declare_parameter<std::string>("hardware_status_topic","/real/cefiro/hardware/status");
    sub_hardware_status_=node_->create_subscription<std_msgs::msg::String>(hardware_status_topic,10,[this](const std_msgs::msg::String::SharedPtr m){onHardwareStatus(*m);});
    // Only the SIM balance node publishes a model-based envelope. There is
    // no corresponding real-plant certification in this release.
    const auto reference_envelope_topic=node_->declare_parameter<std::string>(
        "sim_reference_envelope_topic","/cefiro/control/reference_envelope_deg");
    rclcpp::QoS envelope_qos(1);
    envelope_qos.reliable().transient_local();
    sub_reference_envelope_=node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        reference_envelope_topic,envelope_qos,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){onReferenceEnvelope(*msg);});
    pub_reference_=node_->create_publisher<Ref>(reference_topic_,10);
 pub_real_reference_=node_->create_publisher<Ref>("/real/cefiro/control/attitude_reference",10);
    pub_enabled_=node_->create_publisher<std_msgs::msg::Bool>(enable_topic,10);
 pub_real_enabled_=node_->create_publisher<std_msgs::msg::Bool>("/real/cefiro/control/enable",10);
    pub_stop_=node_->create_publisher<std_msgs::msg::Bool>(stop_topic,10);
 pub_real_stop_=node_->create_publisher<std_msgs::msg::Bool>("/real/cefiro/control/emergency_stop",10);
    pub_mode_=node_->create_publisher<std_msgs::msg::String>(mode_topic,10);
 pub_real_mode_=node_->create_publisher<std_msgs::msg::String>("/real/cefiro/control/mode",10);
    pub_strategy_=node_->create_publisher<std_msgs::msg::String>(strategy_topic,10);
 pub_real_strategy_=node_->create_publisher<std_msgs::msg::String>("/real/cefiro/control/strategy",10);
    auto pid_topic=node_->declare_parameter<std::string>("pid_gains_topic","/cefiro/control/pid_gains");
    auto lqr_topic=node_->declare_parameter<std::string>("lqr_gain_topic","/cefiro/control/lqr_gain");
    motor_rpm_topic_=node_->declare_parameter<std::string>("motor_rpm_topic","/cefiro_actuators/target_rpm");
    motor_direction_topic_=node_->declare_parameter<std::string>("motor_direction_topic","/cefiro_actuators/motor_direction");
    pub_pid_=node_->create_publisher<std_msgs::msg::Float64MultiArray>(pid_topic,10);
 pub_real_pid_=node_->create_publisher<std_msgs::msg::Float64MultiArray>("/real/cefiro/control/pid_gains",10);
    pub_lqr_=node_->create_publisher<std_msgs::msg::Float64MultiArray>(lqr_topic,10);
 pub_real_lqr_=node_->create_publisher<std_msgs::msg::Float64MultiArray>("/real/cefiro/control/lqr_gain",10);
    pub_motor_rpm_=node_->create_publisher<cefiro_msgs::msg::MotorRpm>(motor_rpm_topic_,10);
 pub_real_motor_rpm_=node_->create_publisher<cefiro_msgs::msg::MotorRpm>("/real/cefiro_actuators/target_rpm",10);
    pub_motor_direction_=node_->create_publisher<std_msgs::msg::Int8MultiArray>(motor_direction_topic_,10);
 pub_real_motor_direction_=node_->create_publisher<std_msgs::msg::Int8MultiArray>("/real/cefiro_actuators/motor_direction",10);
    auto motor_armed_topic=node_->declare_parameter<std::string>("motor_armed_topic","/cefiro_actuators/armed");
    pub_motor_armed_=node_->create_publisher<std_msgs::msg::Bool>(motor_armed_topic,10);
 pub_real_motor_armed_=node_->create_publisher<std_msgs::msg::Bool>("/real/cefiro_actuators/armed",10);
    // Observe the actual motor command and feedback graph; do not synthesize ESC RPM.
    const auto sim_target=node_->declare_parameter<std::string>("sim_target_rpm_topic",motor_rpm_topic_);
    const auto real_target=node_->declare_parameter<std::string>("real_target_rpm_topic","/real/cefiro_actuators/target_rpm");
    const auto sim_direct=node_->declare_parameter<std::string>("sim_direct_dshot_topic","/cefiro_actuators/dshot_throttle");
    const auto real_direct=node_->declare_parameter<std::string>("real_direct_dshot_topic","/real/cefiro_actuators/dshot_throttle");
    const auto sim_feedback=node_->declare_parameter<std::string>("sim_measured_rpm_topic","/cefiro_actuators/measured_rpm");
    const auto real_feedback=node_->declare_parameter<std::string>("real_measured_rpm_topic","/real/cefiro_actuators/measured_rpm");
    const auto sim_telemetry=node_->declare_parameter<std::string>("sim_motor_telemetry_topic","/cefiro_actuators/telemetry");
    const auto real_telemetry=node_->declare_parameter<std::string>("real_motor_telemetry_topic","/real/cefiro_actuators/telemetry");
    const auto real_host_dshot=node_->declare_parameter<std::string>("real_host_commanded_dshot_topic","/real/cefiro/hardware/commanded_dshot");
    for(int src=0;src<2;++src){
        const bool real=src==1;
        sub_target_rpm_[src]=node_->create_subscription<MotorRpm>(real?real_target:sim_target,10,
            [this,real](MotorRpm::ConstSharedPtr msg){onMotorRpm(*msg,real,false);});
        sub_measured_rpm_[src]=node_->create_subscription<MotorRpm>(real?real_feedback:sim_feedback,10,
            [this,real](MotorRpm::ConstSharedPtr msg){onMotorRpm(*msg,real,true);});
        sub_direct_dshot_[src]=node_->create_subscription<MotorDshot>(real?real_direct:sim_direct,10,
            [this,real](MotorDshot::ConstSharedPtr msg){onMotorDshot(*msg,real,false);});
        sub_motor_telemetry_[src]=node_->create_subscription<MotorTelemetry>(real?real_telemetry:sim_telemetry,10,
            [this,real](MotorTelemetry::ConstSharedPtr msg){onMotorTelemetry(*msg,real);});
    }
    sub_real_commanded_dshot_=node_->create_subscription<MotorDshot>(real_host_dshot,10,
        [this](MotorDshot::ConstSharedPtr msg){onMotorDshot(*msg,true,true);});
    executor_.add_node(node_);
}
RosBridge::~RosBridge(){stop();}
void RosBridge::start(){ if(running_)return; running_=true; worker_=std::thread([this]{executor_.spin();}); }
void RosBridge::stop(){ if(!running_)return; running_=false;executor_.cancel();if(worker_.joinable())worker_.join();executor_.remove_node(node_); }
void RosBridge::selectReal(bool v){std::lock_guard<std::mutex>g(mutex_);real_selected_=v;}
bool RosBridge::isReal()const{std::lock_guard<std::mutex>g(mutex_);return real_selected_;}
void RosBridge::pushMotor(MotorEvent event){
    const auto index=event.from_real?1u:0u;
    std::lock_guard<std::mutex> guard(mutex_);
    // Sending a command alone does not mean the plant is connected.
    if(event.kind==MotorEventKind::MeasuredRpm || event.kind==MotorEventKind::Telemetry){
        motor_rx_[index]=std::chrono::steady_clock::now();
    }
    if(motor_queue_.size()>=2000)motor_queue_.pop_front();
    motor_queue_.push_back(std::move(event));
}
void RosBridge::onMotorRpm(const MotorRpm& msg,bool real,bool measured){
    MotorEvent event;
    event.from_real=real;
    event.kind=measured?MotorEventKind::MeasuredRpm:MotorEventKind::TargetRpm;
    for(std::size_t i=0;i<4;++i){
        if(!std::isfinite(msg.data[i])||msg.data[i]<0.0)return;
        (measured?event.measured_rpm:event.requested_rpm)[i]=msg.data[i];
    }
    pushMotor(std::move(event));
}
void RosBridge::onMotorDshot(const MotorDshot& msg,bool real,bool host_confirmed){
    MotorEvent event;
    event.from_real=real;
    event.kind=host_confirmed?MotorEventKind::HostCommandedDshot:MotorEventKind::DirectDshot;
    for(std::size_t i=0;i<4;++i){
        const auto v=msg.data[i];
        if(v!=0 && (v<48 || v>2047))return;
        event.dshot[i]=v;
    }
    pushMotor(std::move(event));
}
void RosBridge::onMotorTelemetry(const MotorTelemetry& msg,bool real){
    MotorEvent event;
    event.from_real=real;
    event.kind=MotorEventKind::Telemetry;
    event.command_source=msg.command_source;
    for(std::size_t i=0;i<4;++i){
        if(!std::isfinite(msg.measured_rpm[i])||msg.measured_rpm[i]<0.0 ||
           !std::isfinite(msg.requested_rpm[i])||msg.requested_rpm[i]<0.0)return;
        event.requested_rpm[i]=msg.requested_rpm[i];
        event.measured_rpm[i]=msg.measured_rpm[i];
        const auto code=msg.dshot_value[i];
        if(code!=0 && (code<48 || code>2047))return;
        event.dshot[i]=code;
    }
    pushMotor(std::move(event));
}
std::vector<MotorEvent> RosBridge::drainMotor(){
    std::lock_guard<std::mutex> guard(mutex_);
    std::vector<MotorEvent> result;
    result.reserve(motor_queue_.size());
    while(!motor_queue_.empty()){
        result.push_back(std::move(motor_queue_.front()));
        motor_queue_.pop_front();
    }
    return result;
}
bool RosBridge::motorFreshFor(bool real,double timeout_s) const{
    std::lock_guard<std::mutex> guard(mutex_);
    return fresh(motor_rx_[real?1u:0u],timeout_s);
}
void RosBridge::onImu(const Imu& msg,bool real){
    Telemetry s;
    {
        std::lock_guard<std::mutex> g(mutex_);
        s=real?latest_real_:latest_;
        s.from_real=real;
        s.kind=SampleKind::Imu;
        ++s.sequence;
        s.setpoint_deg=latest_reference_[real ? 1 : 0];
        (real?real_imu_rx_:imu_rx_)=std::chrono::steady_clock::now();
    }
    s.ros_time_s=stampSeconds(msg.header.stamp);
    s.imu_frame=msg.header.frame_id;
    s.orientation={msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z};
    s.orientation_valid=msg.orientation_covariance[0]!=-1.0 && s.orientation.normalize();
    if(s.orientation_valid) {
        // In Gazebo the nominal gimbal orientation is world yaw 1.07 rad +
        // gyro_to_frame yaw 1.571 rad. GUI setpoints of zero denote
        // deviations about THIS attitude, not absolute world yaw=0.
        if(display_relative_attitude_&&!real) {
            const double half=-0.5*equilibrium_world_yaw_deg_*deg2rad;
            const double c=std::cos(half), v=std::sin(half);
            const Quaternion a=s.orientation;
            Quaternion relative{c*a.w-v*a.z,
                                c*a.x-v*a.y,
                                c*a.y+v*a.x,
                                c*a.z+v*a.w};
            if(relative.normalize()) s.orientation=relative;
        }
        s.attitude_deg=s.orientation.eulerDeg();
    }
    s.angular_rate_deg_s={msg.angular_velocity.x*rad2deg,msg.angular_velocity.y*rad2deg,msg.angular_velocity.z*rad2deg};
    s.accel_m_s2={msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z};
    std::lock_guard<std::mutex> g(mutex_);
    const auto& current=real?latest_real_:latest_;
    s.mag_uT=current.mag_uT;
    s.magnetometer_valid=fresh(real ? real_mag_rx_ : mag_rx_, 2.5);
    s.mag_frame=current.mag_frame;
    s.barometer_valid=fresh(real ? real_baro_rx_ : baro_rx_, 2.5);
    s.baro_frame=current.baro_frame;
    s.pressure_hPa=current.pressure_hPa;
    s.relative_alt_m=current.relative_alt_m;
    (real ? latest_real_ : latest_)=s;
    if(queue_.size()>=5000)queue_.pop_front();
    queue_.push_back(std::move(s));
}
void RosBridge::onMag(const Mag& msg,bool real){
    if (!std::isfinite(msg.magnetic_field.x) ||
        !std::isfinite(msg.magnetic_field.y) ||
        !std::isfinite(msg.magnetic_field.z)) return;
    std::lock_guard<std::mutex> g(mutex_);
    (real ? real_mag_rx_ : mag_rx_)=std::chrono::steady_clock::now();
    auto& latest=real ? latest_real_ : latest_;
    latest.mag_uT={msg.magnetic_field.x*1.e6,
                   msg.magnetic_field.y*1.e6,
                   msg.magnetic_field.z*1.e6};
    latest.magnetometer_valid=true;
    latest.mag_frame=msg.header.frame_id;
    Telemetry sample=latest;
    sample.from_real=real;
    sample.kind=SampleKind::Magnetometer;
    if (queue_.size()>=5000) queue_.pop_front();
    queue_.push_back(std::move(sample));
}

void RosBridge::onBaro(const Baro& msg,bool real){
    const double p=msg.fluid_pressure;
    if (!std::isfinite(p) || p<=0.0) return;
    std::lock_guard<std::mutex> g(mutex_);
    auto& timestamp=real ? real_baro_rx_ : baro_rx_;
    auto& baseline=real ? real_initial_pressure_ : first_pressure_pa_;
    if (!fresh(timestamp,2.5)) baseline=0.0; // New session: reset relative altitude.
    timestamp=std::chrono::steady_clock::now();
    auto& latest=real ? latest_real_ : latest_;
    if (baseline<=0.0) {
        baseline=p;
    }
    latest.pressure_hPa=p/100.0; // ROS FluidPressure is Pa; display is hPa.
    latest.relative_alt_m=44330.0*(1.0-std::pow(p/baseline,0.190294957));
    latest.barometer_valid=true;
    latest.baro_frame=msg.header.frame_id;
    Telemetry sample=latest;
    sample.from_real=real;
    sample.kind=SampleKind::Barometer;
    if (queue_.size()>=5000) queue_.pop_front();
    queue_.push_back(std::move(sample));
}

void RosBridge::onStatus(const std_msgs::msg::String& msg,bool real){
    std::lock_guard<std::mutex> g(mutex_);
    (real ? real_status_ : status_)=msg.data;
    (real ? real_status_rx_ : status_rx_)=std::chrono::steady_clock::now();
}
void RosBridge::onHardwareStatus(const std_msgs::msg::String& msg){std::lock_guard<std::mutex> g(mutex_);hardware_status_=msg.data;hardware_status_rx_=std::chrono::steady_clock::now();}
bool RosBridge::hardwareStatusFresh(double t)const{std::lock_guard<std::mutex>g(mutex_);return fresh(hardware_status_rx_,t);}
std::string RosBridge::hardwareStatus()const{std::lock_guard<std::mutex>g(mutex_);return hardware_status_;}
std::vector<Telemetry> RosBridge::drain(){
    std::lock_guard<std::mutex> g(mutex_);
    std::vector<Telemetry> out;out.reserve(queue_.size());
    while(!queue_.empty()){out.push_back(std::move(queue_.front()));queue_.pop_front();}
    return out;
}
bool RosBridge::imuFreshFor(bool real,double t)const{
    std::lock_guard<std::mutex> g(mutex_);
    return fresh(real ? real_imu_rx_ : imu_rx_, t);
}
bool RosBridge::magFreshFor(bool real,double t)const{
    std::lock_guard<std::mutex> g(mutex_);
    return fresh(real ? real_mag_rx_ : mag_rx_, t);
}
bool RosBridge::baroFreshFor(bool real,double t)const{
    std::lock_guard<std::mutex> g(mutex_);
    return fresh(real ? real_baro_rx_ : baro_rx_, t);
}
bool RosBridge::sourceFresh(bool real)const{
    std::lock_guard<std::mutex> g(mutex_);
    return fresh(real ? real_imu_rx_ : imu_rx_, 1.5) ||
           fresh(real ? real_mag_rx_ : mag_rx_, 2.5) ||
           fresh(real ? real_baro_rx_ : baro_rx_, 2.5);
}
void RosBridge::resetTelemetry(bool real){
    std::lock_guard<std::mutex> g(mutex_);
    auto& latest=real ? latest_real_ : latest_;
    latest=Telemetry{};
    latest.from_real=real;
    (real ? real_initial_pressure_ : first_pressure_pa_)=0.0;
}
bool RosBridge::imuFresh(double t)const{return imuFreshFor(isReal(),t);}
bool RosBridge::magFresh(double t)const{return magFreshFor(isReal(),t);}
bool RosBridge::baroFresh(double t)const{return baroFreshFor(isReal(),t);}
bool RosBridge::feedbackFresh(double t)const{
    std::lock_guard<std::mutex> g(mutex_);
    return fresh(real_selected_ ? real_status_rx_ : status_rx_,t);
}
std::string RosBridge::controllerStatus()const{
    std::lock_guard<std::mutex> g(mutex_);
    return real_selected_ ? real_status_ : status_;
}
void RosBridge::onReferenceEnvelope(const std_msgs::msg::Float64MultiArray& msg){
    if(msg.data.size()!=3)return;
    const Vec3 limits{{msg.data[0],msg.data[1],msg.data[2]}};
    if(!std::all_of(limits.begin(),limits.end(),[](double x){return std::isfinite(x)&&x>0.;}))return;
    if(limits[0]>30.01||limits[1]>30.01||limits[2]>180.01)return;
    std::lock_guard<std::mutex> g(mutex_);
    sim_reference_envelope_deg_=limits;
    have_sim_reference_envelope_=true;
}
bool RosBridge::simReferenceEnvelope(Vec3& limits_deg)const{
    std::lock_guard<std::mutex> g(mutex_);
    if(!have_sim_reference_envelope_)return false;
    limits_deg=sim_reference_envelope_deg_;
    return true;
}
void RosBridge::publishReference(const Vec3& degrees,const std::string& mode,const std::string& strategy){
    Ref msg;msg.header.stamp=node_->now();msg.header.frame_id="drone_body_link";
    msg.vector.x=clampFinite(degrees[0],-180.,180.)*deg2rad;
    msg.vector.y=clampFinite(degrees[1],-90.,90.)*deg2rad;
    msg.vector.z=clampFinite(degrees[2],-180.,180.)*deg2rad;
    {std::lock_guard<std::mutex>g(mutex_);latest_reference_[real_selected_ ? 1 : 0]=degrees;}
    (isReal()?pub_real_reference_:pub_reference_)->publish(msg);
    std_msgs::msg::String m; m.data=mode;(isReal()?pub_real_mode_:pub_mode_)->publish(m);
    m.data=strategy;(isReal()?pub_real_strategy_:pub_strategy_)->publish(m);
}

void RosBridge::publishMotorCommand(const std::array<double,4>& rpm,const std::array<int,4>& direction){
    cefiro_msgs::msg::MotorRpm rpm_msg;
    std_msgs::msg::Int8MultiArray dir_msg;
    dir_msg.data.resize(4);
    for(std::size_t i=0;i<4;++i){
        rpm_msg.data[i]=std::max(0.0, std::isfinite(rpm[i]) ? rpm[i] : 0.0);
        dir_msg.data[i]=direction[i] >= 0 ? 1 : -1;
    }
    (isReal()?pub_real_motor_rpm_:pub_motor_rpm_)->publish(rpm_msg);
    (isReal()?pub_real_motor_direction_:pub_motor_direction_)->publish(dir_msg);
}

void RosBridge::setMotorArmed(bool armed){
    std_msgs::msg::Bool msg;
    msg.data=armed;
    (isReal()?pub_real_motor_armed_:pub_motor_armed_)->publish(msg);
}

void RosBridge::publishGains(const std::vector<double>& pid,const std::vector<double>& lqr){
    std_msgs::msg::Float64MultiArray msg;msg.data=pid;(isReal()?pub_real_pid_:pub_pid_)->publish(msg);
    msg.data=lqr;(isReal()?pub_real_lqr_:pub_lqr_)->publish(msg);
}
void RosBridge::setEnabled(bool e){std_msgs::msg::Bool m;m.data=e;(isReal()?pub_real_enabled_:pub_enabled_)->publish(m);if(e){m.data=false;(isReal()?pub_real_stop_:pub_stop_)->publish(m);}}
void RosBridge::emergencyStop(){std_msgs::msg::Bool m;m.data=false;pub_enabled_->publish(m);pub_real_enabled_->publish(m);m.data=true;pub_stop_->publish(m);pub_real_stop_->publish(m);}
}
