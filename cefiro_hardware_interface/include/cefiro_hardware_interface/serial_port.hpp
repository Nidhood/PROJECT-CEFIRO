#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/types.h>

namespace cefiro::hardware {
class SerialPort {
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;
  bool open(const std::string& port, int baudrate, std::string& error);
  void close() noexcept;
  [[nodiscard]] bool connected() const noexcept { return fd_ >= 0; }
  // Returns -1 on I/O error; 0 when no bytes are available (nonblocking).
  ssize_t readSome(std::uint8_t* output, std::size_t capacity);
  ssize_t writeSome(const std::uint8_t* input, std::size_t size);
private:
  int fd_{-1};
};
}  // namespace cefiro::hardware
