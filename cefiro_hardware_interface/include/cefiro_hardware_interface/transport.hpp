#pragma once
#include "cefiro_hardware_interface/serial_port.hpp"
#include "cefiro_hardware_interface/tcp_port.hpp"
#include <utility>

namespace cefiro::hardware {
// One stream abstraction for both transports; no duplicated protocol or ROS graph.
class Transport {
public:
  using State = TcpPort::State;
  void configure(bool wifi, std::string endpoint, int speed, double timeout_s) {
    close(); wifi_ = wifi; endpoint_ = std::move(endpoint); speed_ = speed; timeout_s_ = timeout_s;
  }
  State connectStep(std::string& error) {
    if (wifi_) return tcp_.connectStep(endpoint_, speed_, timeout_s_, error);
    return serial_.open(endpoint_, speed_, error) ? State::Connected : State::Failed;
  }
  void close() noexcept { tcp_.close(); serial_.close(); }
  bool connected() const noexcept { return wifi_ ? tcp_.connected() : serial_.connected(); }
  ssize_t readSome(std::uint8_t* data, std::size_t n) { return wifi_ ? tcp_.readSome(data,n) : serial_.readSome(data,n); }
  ssize_t writeSome(const std::uint8_t* data, std::size_t n) { return wifi_ ? tcp_.writeSome(data,n) : serial_.writeSome(data,n); }
private:
  bool wifi_{false};
  std::string endpoint_;
  int speed_{0};
  double timeout_s_{3.0};
  SerialPort serial_;
  TcpPort tcp_;
};
}  // namespace cefiro::hardware
