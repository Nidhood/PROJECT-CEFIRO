#pragma once
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/types.h>

namespace cefiro::hardware {
// Nonblocking IPv4 TCP stream. All calls belong to the node's IO executor.
class TcpPort {
public:
  enum class State { Pending, Connected, Failed };
  TcpPort() = default;
  ~TcpPort() { close(); }
  TcpPort(const TcpPort&) = delete;
  TcpPort& operator=(const TcpPort&) = delete;
  State connectStep(const std::string& host, int port, double timeout_s, std::string& error);
  void close() noexcept;
  bool connected() const noexcept { return fd_ >= 0 && !pending_; }
  ssize_t readSome(std::uint8_t* output, std::size_t capacity);
  ssize_t writeSome(const std::uint8_t* input, std::size_t size);
private:
  int fd_{-1};
  bool pending_{false};
  std::chrono::steady_clock::time_point deadline_{};
};
}  // namespace cefiro::hardware
