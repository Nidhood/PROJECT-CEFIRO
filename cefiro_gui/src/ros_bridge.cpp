#include "cefiro_gui/ros_bridge.hpp"
#include <chrono>
#include <cmath>
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
    imu_topic_=node_->declare_parameter<std::string>("imu_topic","/imu/data");
    mag_topic_=node_->declare_parameter<std::string>("magnetometer_topic","/magnetometer/data");
    baro_topic_=node_->declare_parameter<std::string>("barometer_topic","/bmp581/air_pressure");
    reference_topic_=node_->declare_parameter<std::string>("reference_topic","/cefiro/control/attitude_reference");
    auto enable_topic=node_->declare_parameter<std::string>("enable_topic","/cefiro/control/enable");
    auto stop_topic=node_->declare_parameter<std::string>("stop_topic","/cefiro/control/emergency_stop");
    auto mode_topic=node_->declare_parameter<std::string>("mode_topic","/cefiro/control/mode");
    auto strategy_topic=node_->declare_parameter<std::string>("strategy_topic","/cefiro/control/strategy");
    auto status_topic=node_->declare_parameter<std::string>("status_topic","/cefiro/control/status");
    sub_imu_=node_->create_subscription<Imu>(imu_topic_,rclcpp::SensorDataQoS(),[this](const Imu::SharedPtr m){onImu(*m);});
    sub_mag_=node_->create_subscription<Mag>(mag_topic_,rclcpp::SensorDataQoS(),[this](const Mag::SharedPtr m){onMag(*m);});
    sub_baro_=node_->create_subscription<Baro>(baro_topic_,rclcpp::SensorDataQoS(),[this](const Baro::SharedPtr m){onBaro(*m);});
    sub_status_=node_->create_subscription<std_msgs::msg::String>(status_topic,10,[this](const std_msgs::msg::String::SharedPtr m){onStatus(*m);});
    pub_reference_=node_->create_publisher<Ref>(reference_topic_,10);
    pub_enabled_=node_->create_publisher<std_msgs::msg::Bool>(enable_topic,10);
    pub_stop_=node_->create_publisher<std_msgs::msg::Bool>(stop_topic,10);
    pub_mode_=node_->create_publisher<std_msgs::msg::String>(mode_topic,10);
    pub_strategy_=node_->create_publisher<std_msgs::msg::String>(strategy_topic,10);
    auto pid_topic=node_->declare_parameter<std::string>("pid_gains_topic","/cefiro/control/pid_gains");
    auto lqr_topic=node_->declare_parameter<std::string>("lqr_gain_topic","/cefiro/control/lqr_gain");
    pub_pid_=node_->create_publisher<std_msgs::msg::Float64MultiArray>(pid_topic,10);
    pub_lqr_=node_->create_publisher<std_msgs::msg::Float64MultiArray>(lqr_topic,10);
    executor_.add_node(node_);
}
RosBridge::~RosBridge(){stop();}
void RosBridge::start(){ if(running_)return; running_=true; worker_=std::thread([this]{executor_.spin();}); }
void RosBridge::stop(){ if(!running_)return; running_=false;executor_.cancel();if(worker_.joinable())worker_.join();executor_.remove_node(node_); }
void RosBridge::onImu(const Imu& msg){
    Telemetry s;
    {
        std::lock_guard<std::mutex> g(mutex_);
        s=latest_;
        s.sequence=latest_.sequence+1;
        s.setpoint_deg=latest_reference_;
        imu_rx_=std::chrono::steady_clock::now();
    }
    s.ros_time_s=stampSeconds(msg.header.stamp);
    s.imu_frame=msg.header.frame_id;
    s.orientation={msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z};
    s.orientation_valid=msg.orientation_covariance[0]!=-1.0 && s.orientation.normalize();
    if(s.orientation_valid) s.attitude_deg=s.orientation.eulerDeg();
    s.angular_rate_deg_s={msg.angular_velocity.x*rad2deg,msg.angular_velocity.y*rad2deg,msg.angular_velocity.z*rad2deg};
    s.accel_m_s2={msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z};
    std::lock_guard<std::mutex> g(mutex_);
    s.mag_uT=latest_.mag_uT; s.magnetometer_valid=latest_.magnetometer_valid; s.mag_frame=latest_.mag_frame;
    s.barometer_valid=latest_.barometer_valid; s.baro_frame=latest_.baro_frame;
    s.pressure_hPa=latest_.pressure_hPa; s.relative_alt_m=latest_.relative_alt_m;
    latest_=s;
    if(queue_.size()>=5000)queue_.pop_front();
    queue_.push_back(std::move(s));
}
void RosBridge::onMag(const Mag& msg){
    std::lock_guard<std::mutex> g(mutex_);
    mag_rx_=std::chrono::steady_clock::now();
    latest_.mag_uT={msg.magnetic_field.x*1.e6,msg.magnetic_field.y*1.e6,msg.magnetic_field.z*1.e6};
    latest_.magnetometer_valid=true;latest_.mag_frame=msg.header.frame_id;
}
void RosBridge::onBaro(const Baro& msg){
    const double p=msg.fluid_pressure;
    if(!std::isfinite(p)||p<=0.0)return;
    std::lock_guard<std::mutex> g(mutex_);
    baro_rx_=std::chrono::steady_clock::now();
    if(first_pressure_pa_<=0.0)first_pressure_pa_=p;
    latest_.pressure_hPa=p/100.0;
    latest_.relative_alt_m=44330.0*(1.0-std::pow(p/first_pressure_pa_,0.190294957));
    latest_.barometer_valid=true;latest_.baro_frame=msg.header.frame_id;
}
void RosBridge::onStatus(const std_msgs::msg::String& msg){std::lock_guard<std::mutex> g(mutex_);status_=msg.data;status_rx_=std::chrono::steady_clock::now();}
std::vector<Telemetry> RosBridge::drain(){
    std::lock_guard<std::mutex> g(mutex_);
    std::vector<Telemetry> out;out.reserve(queue_.size());
    while(!queue_.empty()){out.push_back(std::move(queue_.front()));queue_.pop_front();}
    return out;
}
bool RosBridge::imuFresh(double t)const{std::lock_guard<std::mutex>g(mutex_);return fresh(imu_rx_,t);}
bool RosBridge::magFresh(double t)const{std::lock_guard<std::mutex>g(mutex_);return fresh(mag_rx_,t);}
bool RosBridge::baroFresh(double t)const{std::lock_guard<std::mutex>g(mutex_);return fresh(baro_rx_,t);}
bool RosBridge::feedbackFresh(double t)const{std::lock_guard<std::mutex>g(mutex_);return fresh(status_rx_,t);}
std::string RosBridge::controllerStatus()const{std::lock_guard<std::mutex>g(mutex_);return status_;}
void RosBridge::publishReference(const Vec3& degrees,const std::string& mode,const std::string& strategy){
    Ref msg;msg.header.stamp=node_->now();msg.header.frame_id="drone_body_link";
    msg.vector.x=clampFinite(degrees[0],-180.,180.)*deg2rad;
    msg.vector.y=clampFinite(degrees[1],-90.,90.)*deg2rad;
    msg.vector.z=clampFinite(degrees[2],-180.,180.)*deg2rad;
    {std::lock_guard<std::mutex>g(mutex_);latest_reference_=degrees;}
    pub_reference_->publish(msg);
    std_msgs::msg::String m; m.data=mode;pub_mode_->publish(m);
    m.data=strategy;pub_strategy_->publish(m);
}
void RosBridge::publishGains(const std::vector<double>& pid,const std::vector<double>& lqr){
    std_msgs::msg::Float64MultiArray msg;msg.data=pid;pub_pid_->publish(msg);
    msg.data=lqr;pub_lqr_->publish(msg);
}
void RosBridge::setEnabled(bool e){std_msgs::msg::Bool m;m.data=e;pub_enabled_->publish(m);if(e){m.data=false;pub_stop_->publish(m);}}
void RosBridge::emergencyStop(){std_msgs::msg::Bool m;m.data=false;pub_enabled_->publish(m);m.data=true;pub_stop_->publish(m);}
}
