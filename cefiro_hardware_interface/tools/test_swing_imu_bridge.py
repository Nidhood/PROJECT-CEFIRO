#!/usr/bin/env python3
"""Compile the production swing callback and IMU decoder against small test doubles.
No ROS runtime, serial port or motor output is required. Not a ROS integration test.
"""
from pathlib import Path
import subprocess, tempfile
root=Path(__file__).resolve().parents[1]
s=(root/'src/hardware_node.cpp').read_text()
cb=s.split('sub_shadow_ = create_subscription<std_msgs::msg::Float64MultiArray>("cefiro/control/swing_tuning", 10,\n',1)[1].split('\n      });',1)[0]+'\n      }'
parser=s[s.index('  cefiro_msgs::msg::IdentificationSample parseImuMetadata('):s.index('    if (frame.kind == "MAG"')]+ '\n    return false;\n  }\n'
helpers=s[s.index('std::string number('):s.index('std::string jsonQuote(')]+s[s.index('std::optional<std::vector<double>> numbers('):s.index('}  // namespace\n\nclass HardwareNode')]
code=r'''
#include "cefiro_hardware_interface/protocol.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
using namespace std::chrono_literals;
using namespace cefiro::hardware;
using Clock=std::chrono::steady_clock;
using Time=Clock::time_point;
template<class... Args> void testLog(Args&&...) {}
#define RCLCPP_INFO(...) testLog(__VA_ARGS__)
#define RCLCPP_WARN(...) testLog(__VA_ARGS__)
namespace std_msgs::msg {
struct String {std::string data;};
struct Float64MultiArray {using ConstSharedPtr=std::shared_ptr<const Float64MultiArray>;std::vector<double> data;};
}
namespace cefiro_msgs::msg {
struct IdentificationSample {bool fmu_dshot_valid{},fmu_armed{};std::uint8_t fmu_state{};std::uint32_t fmu_sample_ms{};std::array<std::uint16_t,4> fmu_dshot{};};
}
'''+helpers+r'''
struct FakePublisher {std::string last;void publish(const std_msgs::msg::String& m){last=m.data;}};
struct Harness {
  FakePublisher pub; FakePublisher* pub_shadow_=&pub;
  bool can_tx_=true,connected=true,controller_enabled_=false,arm_requested_=false,arm_confirmed_=false,fault_latched_=false,manual_mode_confirmed_=true,mcu_status_seen_=true,swing_confirmed_=false;
  bool allow_experimental_roll_=false;int get_logger(){return 0;}
  unsigned motor_map_revision_=3;std::string swing_result_,fmu_fault_reason_;
  unsigned swing_token_=0;int mcu_fmu_state_=0;int stopped=0;
  std::optional<std::uint16_t> swing_start_sequence_;
  Time swing_last_gui_{},swing_last_feedback_{},orientation_received_=Clock::now();
  std::optional<Time> last_valid_frame_=Clock::now(),last_mcu_stat_=Clock::now(),last_imu_=Clock::now();
  std::optional<int> orientation_=1;double orientation_timeout_s_=.2;
  struct {bool external_manual=true;} power_gate_;
  std::deque<std::string> tx_queue_;std::string pending_tx_;std::size_t pending_offset_=0;
  std::string command;std::vector<std::string> payload;
  bool sessionReady(){return connected;}
  std::string manualPowerReason(Time){return {};}
  void emergencyStop(){++stopped;swing_token_=0;}
  std::optional<std::uint16_t> queueCommand(std::string_view k,const std::vector<std::string>& v){command=k;payload=v;return 17;}
  void send(std::vector<double> v){command.clear();pub.last.clear();auto m=std::make_shared<std_msgs::msg::Float64MultiArray>();m->data=std::move(v);
    auto callback = '''+cb+r''';callback(m);}
  int imu_count=0;ImuReading raw{};cefiro_msgs::msg::IdentificationSample meta{};std::optional<std::uint32_t> tick;bool ext=false;
  std::optional<cefiro_msgs::msg::IdentificationSample> pending_imu_metadata_;Time pending_imu_metadata_at_{};
  void publishImu(const ImuReading& r,cefiro_msgs::msg::IdentificationSample m,std::optional<std::uint32_t> t,bool e){++imu_count;raw=r;meta=m;tick=t;ext=e;}
'''+parser+r'''
};
int main(){
 Harness h;
 h.send({1.1,.4,12.,1.});assert(h.command=="CMD_SWING_SHADOW"&&h.payload.size()==4&&h.payload[0]=="1.100000");
 h.controller_enabled_=true;h.send({1.1,.4,12.,1.});assert(h.command.empty()&&h.pub.last.starts_with("REJECTED"));h.controller_enabled_=false;
 h.send({1.1,.4,12.,0.});assert(h.command.empty());
 h.orientation_.reset();h.send({1.1,.4,12.,1.});assert(h.command.empty()&&h.pub.last.find("AHRS")!=std::string::npos);h.orientation_=1;
 h.send({1.1,.4,12.,1.,77.});assert(h.command.empty()&&h.pub.last.starts_with("REJECTED")); // External supply is not a powered profile.
 h.send({.7,.4,11.5,-1.,77.,1.});assert(h.command.empty()&&h.pub.last.starts_with("REJECTED"));
 h.allow_experimental_roll_=true;
 h.motor_map_revision_=0;h.send({.7,.4,11.5,-1.,77.,1.});assert(h.command.empty()&&h.pub.last.find("map revision")!=std::string::npos);h.motor_map_revision_=2;h.send({.7,.4,11.5,-1.,77.,1.});assert(h.command.empty()&&h.pub.last.find("map revision")!=std::string::npos);h.motor_map_revision_=3;
 h.send({.7,.4,11.5,-1.,77.,0.});assert(h.command.empty()&&h.pub.last.starts_with("REJECTED"));
 h.send({.7,.4,11.5,1.,77.,1.});assert(h.command.empty()&&h.pub.last.starts_with("REJECTED"));
 h.send({.7,.4,11.5,-1.,77.,1.});assert(h.command=="CMD_SWING_EXP"&&h.payload.size()==5&&h.payload[4]=="77"&&h.swing_token_==77);
 h.send({77.});assert(h.command.empty());h.swing_confirmed_=true;
 h.send({77.});assert(h.command=="CMD_SWING_LEASE");
 h.send({});assert(h.command=="CMD_SWING_STOP"&&h.swing_token_==0);
 h.fault_latched_=true;h.send({.7,.4,11.5,-1.,77.,1.});assert(h.command.empty()&&h.pub.last.starts_with("REJECTED"));h.fault_latched_=false;
 auto test=[&](std::string kind,std::string csv){auto f=encodeFrame(11,kind,splitCsv(csv));assert(f);auto d=decodeFrame(*f);assert(d);return h.handleFrame(*d);};
 const std::string raw="1.2538,-0.9440,-8.6285,0.8400,-1.1900,-0.7000,29.3750";
 assert(test("IMU",raw));assert(h.imu_count==1&&!h.meta.fmu_dshot_valid);
 assert(test("IMU",raw+",645984"));assert(h.imu_count==2&&h.tick==645984&&!h.meta.fmu_dshot_valid);
 assert(test("IMU",raw+",0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,645984"));assert(h.imu_count==3&&h.meta.fmu_dshot_valid&&h.meta.fmu_sample_ms==645984);
 assert(test("IMU",raw+",12,0,0,0,1,2,645985"));assert(h.imu_count==4&&!h.meta.fmu_dshot_valid);
 assert(test("IMX","48,0,0,0,1,2,645986"));assert(test("IMU",raw+",645986"));assert(h.imu_count==5&&h.meta.fmu_dshot_valid&&h.meta.fmu_dshot[0]==48);
 assert(test("IMU",raw+",645987"));assert(h.imu_count==6&&!h.meta.fmu_dshot_valid);
 assert(!test("IMU","invalid,0,0,0,0,0,0"));assert(h.imu_count==6);
 std::cout<<"Production swing callback + IMU/IMX parser (test doubles): PASS\n";
}
'''
with tempfile.TemporaryDirectory() as td:
    p=Path(td)/'test.cpp';p.write_text(code)
    exe=Path(td)/'test'
    subprocess.run(['c++','-std=c++20','-Wall','-Wextra','-Wpedantic','-Werror','-I'+str(root/'include'),str(p),str(root/'src/protocol.cpp'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
