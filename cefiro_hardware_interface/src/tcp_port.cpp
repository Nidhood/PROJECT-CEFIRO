#include "cefiro_hardware_interface/tcp_port.hpp"
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace cefiro::hardware {
void TcpPort::close() noexcept {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  pending_ = false;
}
TcpPort::State TcpPort::connectStep(const std::string& host, int port,
                                    double timeout_s, std::string& error) {
  using Clock = std::chrono::steady_clock;
  auto fail = [&](const std::string& why) { error = why; close(); return State::Failed; };
  if (connected()) return State::Connected;
  if (fd_ < 0) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    if (port < 1 || port > 65535 || ::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
      return fail("Expected an IPv4 address and TCP port 1..65535");
    addr.sin_port = htons(static_cast<std::uint16_t>(port));
    fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, IPPROTO_TCP);
    if (fd_ < 0) return fail(std::strerror(errno));
    int one = 1, idle = 3, interval = 1, probes = 2;
    (void)::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    (void)::setsockopt(fd_, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    (void)::setsockopt(fd_, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    (void)::setsockopt(fd_, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
    (void)::setsockopt(fd_, IPPROTO_TCP, TCP_KEEPCNT, &probes, sizeof(probes));
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0)
      return State::Connected;
    if (errno != EINPROGRESS && errno != EINTR) return fail(std::strerror(errno));
    pending_ = true;
    deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_s));
  }
  pollfd p{fd_, POLLOUT, 0};
  const int ready = ::poll(&p, 1, 0);
  if (ready > 0) {
    int code = 0; socklen_t len = sizeof(code);
    if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &code, &len) < 0) return fail(std::strerror(errno));
    if (code) return fail(std::strerror(code));
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) return fail("TCP connect interrupted");
    if (p.revents & POLLOUT) { pending_ = false; return State::Connected; }
  } else if (ready < 0 && errno != EINTR) return fail(std::strerror(errno));
  if (Clock::now() >= deadline_) return fail("TCP connect timeout");
  return State::Pending;
}
ssize_t TcpPort::readSome(std::uint8_t* output, std::size_t capacity) {
  if (!connected()) return -1;
  if (!capacity) return 0;
  const auto n = ::recv(fd_, output, capacity, MSG_DONTWAIT);
  if (n == 0) return -1;  // peer EOF, not idle
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
  return n;
}
ssize_t TcpPort::writeSome(const std::uint8_t* input, std::size_t size) {
  if (!connected()) return -1;
  const auto n = ::send(fd_, input, size, MSG_DONTWAIT | MSG_NOSIGNAL);
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
  return n;
}
}  // namespace cefiro::hardware
