#include "cefiro_hardware_interface/serial_port.hpp"
#include <cassert>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <string>
#include <unistd.h>
#include <termios.h>
using cefiro::hardware::SerialPort;
int main() {
  const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
  const std::string path = ptsname(master);
  const int slave = ::open(path.c_str(), O_RDWR | O_NOCTTY);
  assert(slave >= 0);
  termios opts{}; assert(tcgetattr(slave, &opts) == 0);
#ifdef CRTSCTS
  opts.c_cflag |= CRTSCTS;
#endif
  opts.c_iflag |= IXON | IXOFF | IXANY;
  assert(tcsetattr(slave, TCSANOW, &opts) == 0);
  SerialPort a,b; std::string err;
  assert(a.open(path,115200,err)); assert(!b.open(path,115200,err));
  assert(tcgetattr(slave, &opts) == 0);
#ifdef CRTSCTS
  assert(!(opts.c_cflag & CRTSCTS));
#endif
  assert(!(opts.c_iflag & (IXON | IXOFF | IXANY)));
  assert(cfgetispeed(&opts) == B115200 && cfgetospeed(&opts) == B115200);
  assert((opts.c_cflag & CSIZE) == CS8 && !(opts.c_cflag & (PARENB | CSTOPB)));
  close(slave);
  const std::uint8_t cmd[] = {'@','C','F','1','\n'};
  assert(a.writeSome(cmd,sizeof cmd)==sizeof cmd);
  pollfd p{master,POLLIN,0}; assert(poll(&p,1,1000)>0);
  std::uint8_t buf[32]{}; assert(read(master,buf,sizeof buf)==sizeof cmd);
  assert(std::string(reinterpret_cast<char*>(buf),sizeof cmd)=="@CF1\n");
  assert(write(master,"ACK\n",4)==4);
  ssize_t n=0; for(int i=0;i<100 && n==0;++i){usleep(1000);n=a.readSome(buf,sizeof buf);}
  assert(n==4 && std::string(reinterpret_cast<char*>(buf),4)=="ACK\n");
  a.close();assert(b.open(path,115200,err));b.close();close(master);
  std::cout << "PASS: USB/USART via PTY, bidirectional bytes, exclusive open, reopen\n";
}
