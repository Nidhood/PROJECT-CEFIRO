// Standalone (no ROS required) quick verification of the host CF1 codec and
// the firmware-safe open-loop GUI command mapping.
#include "cefiro_hardware_interface/protocol.hpp"
#include "cefiro_hardware_interface/motor_command.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace cefiro::hardware;
int main() {
  ManualMotorMap map{};
  assert(validManualMotorMap(map));
  assert(mapGuiRpmToDshot(0, map).value() == 0);
  assert(mapGuiRpmToDshot(25, map).value() == 0);
  assert(mapGuiRpmToDshot(26, map).value() >= 48);
  assert(mapGuiRpmToDshot(12000, map).value() == 300);
  assert(!mapGuiRpmToDshot(-1, map));
  assert(!mapGuiRpmToDshot(12001, map));
  assert(!mapGuiRpmToDshot(std::nan(""), map));
  const auto four = mapGuiRpmToDshot({0, 2000, 6000, 12000}, map);
  assert(four && (*four)[0] == 0 && (*four)[3] == 300);
  for (std::uint16_t d : *four) assert(d == 0 || (d >= 48 && d <= 300));
  const auto cmd = encodeFrame(101, "CMD_DSHOT", {"0","48","150","300"});
  assert(cmd && cmd->starts_with("@CF1|101|CMD_DSHOT|"));
  const auto parsed = decodeFrame(*cmd);
  assert(parsed && parsed->sequence == 101 && parsed->fields.size() == 4);
  auto tampered = *cmd;
  tampered[11] = tampered[11] == 'A' ? 'B' : 'A';
  assert(!decodeFrame(tampered));
  const auto stat = encodeFrame(44, "STAT", {"FMU_V3","0","0","0","0","0","0","0","0"});
  assert(stat && decodeFrame(*stat) && decodeFrame(*stat)->fields.size() == 9);
  const auto imu = encodeFrame(45, "IMU", {"0.1","-0.2","9.8","1.0","0","-1.0","27.2","5000"});
  assert(imu && decodeFrame(*imu) && decodeFrame(*imu)->fields.size() == 8);
  const auto att = encodeFrame(46, "ATT", {"1.0000000","0.0000000","0.0000000","0.0000000","5001"});
  assert(att && decodeFrame(*att) && decodeFrame(*att)->fields.size() == 5);
  const auto ack = encodeFrame(47, "ACK", {"101","CMD_DSHOT","OK"});
  assert(ack && decodeFrame(*ack) && decodeFrame(*ack)->fields[2] == "OK");
  LineBuffer buffer;
  auto partial = buffer.feed(cmd->substr(0,cmd->size()-1));
  assert(partial.empty());
  auto complete = buffer.feed(cmd->substr(cmd->size()-1));
  assert(complete.size() == 1 && decodeFrame(complete.front()));
  std::cout << "CF1 host codec, CRC, GUI mapping, framing: OK\n";
}
