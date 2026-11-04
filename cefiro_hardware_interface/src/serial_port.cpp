#include "cefiro_hardware_interface/serial_port.hpp"
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace cefiro::hardware {
namespace {
speed_t baudToFlag(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
  }
}
}  // namespace
SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& port, int baudrate, std::string& error) {
  close();
  const speed_t speed = baudToFlag(baudrate);
  if (!speed) { error = "Unsupported baudrate"; return false; }
  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) { error = "open failed: " + std::to_string(errno); return false; }
  termios opts{};
  if (::tcgetattr(fd, &opts) < 0) {
    error = "tcgetattr failed: " + std::to_string(errno);
    ::close(fd); return false;
  }
  ::cfmakeraw(&opts);
  opts.c_cflag |= CLOCAL | CREAD;
  opts.c_cflag &= ~CSTOPB;
  opts.c_cflag &= ~PARENB;
  opts.c_cflag &= ~CSIZE;
  opts.c_cflag |= CS8;
  opts.c_iflag &= ~(IXON | IXOFF | IXANY);
  opts.c_cc[VMIN] = 0;
  opts.c_cc[VTIME] = 0;
  if (::cfsetispeed(&opts, speed) < 0 || ::cfsetospeed(&opts, speed) < 0 ||
      ::tcsetattr(fd, TCSANOW, &opts) < 0) {
    error = "termios configuration failed: " + std::to_string(errno);
    ::close(fd); return false;
  }
  (void)::tcflush(fd, TCIFLUSH);
  fd_ = fd;
  return true;
}

void SerialPort::close() noexcept {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

ssize_t SerialPort::readSome(std::uint8_t* output, std::size_t capacity) {
  if (fd_ < 0) return -1;
  const ssize_t n = ::read(fd_, output, capacity);
  if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
  return n;
}
ssize_t SerialPort::writeSome(const std::uint8_t* input, std::size_t size) {
  if (fd_ < 0) return -1;
  const ssize_t n = ::write(fd_, input, size);
  if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
  return n;
}
}  // namespace cefiro::hardware
