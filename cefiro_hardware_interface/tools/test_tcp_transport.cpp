#include "cefiro_hardware_interface/transport.hpp"
#include "cefiro_hardware_interface/protocol.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <array>
using namespace cefiro::hardware;
using namespace std::chrono_literals;
int main() {
  int listener = socket(AF_INET, SOCK_STREAM, 0); assert(listener >= 0);
  sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  assert(bind(listener, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
  socklen_t len = sizeof(addr); assert(getsockname(listener, reinterpret_cast<sockaddr*>(&addr), &len) == 0);
  assert(listen(listener, 2) == 0);
  const int port = ntohs(addr.sin_port);
  Transport link; link.configure(true, "127.0.0.1", port, 0.5);
  auto connect = [&](){
    std::string error;
    for(int i=0;i<100;++i){
      auto state = link.connectStep(error);
      assert(state != Transport::State::Failed);
      if(state == Transport::State::Connected) return;
      std::this_thread::sleep_for(5ms);
    }
    assert(false);
  };
  auto receive = [&](LineBuffer& lines, std::size_t count){
    std::vector<std::string> result; std::array<std::uint8_t,17> buf{};
    for(int i=0;i<200 && result.size()<count;++i){
      auto n = link.readSome(buf.data(),buf.size()); assert(n>=0);
      if(n){auto out=lines.feed({reinterpret_cast<char*>(buf.data()),static_cast<std::size_t>(n)}); result.insert(result.end(),out.begin(),out.end());}
      else std::this_thread::sleep_for(2ms);
    }
    assert(result.size()==count); return result;
  };
  connect(); int peer=accept(listener,nullptr,nullptr); assert(peer>=0);
  std::uint8_t byte{}; assert(link.readSome(&byte,1)==0);
  auto baro=*encodeFrame(3,"BARO",{"675.5064","32.09","3291.71","675.4","-103.918","429410"});
  auto bat=*encodeFrame(4,"BAT",{"12.600","-1.000","100.000","3.000","13.000","429410"});
  auto pwr=*encodeFrame(5,"PWR",{"51066","0","2571","0","0","1","10","0","42","0","0","429410"});
  assert(send(peer,baro.data(),9,0)==9);
  auto tail=baro.substr(9)+bat+pwr; assert(send(peer,tail.data(),tail.size(),0)==static_cast<ssize_t>(tail.size()));
  LineBuffer lines; auto received=receive(lines,3);
  assert(decodeFrame(received[0])->kind=="BARO");
  assert(decodeFrame(received[1])->fields[4]=="13.000");
  assert(decodeFrame(received[2])->fields.size()==12);
  auto ping=*encodeFrame(42,"CMD_PING",{});
  assert(link.writeSome(reinterpret_cast<const std::uint8_t*>(ping.data()),ping.size())==static_cast<ssize_t>(ping.size()));
  std::string input(ping.size(),'\0'); assert(recv(peer,input.data(),input.size(),MSG_WAITALL)==static_cast<ssize_t>(input.size()));
  assert(input==ping);
  auto ack=*encodeFrame(6,"ACK",{"42","CMD_PING","OK"});
  assert(send(peer,ack.data(),ack.size(),0)==static_cast<ssize_t>(ack.size()));
  assert(decodeFrame(receive(lines,1)[0])->fields[2]=="OK");
  close(peer);
  bool eof=false; for(int i=0;i<100;++i){if(link.readSome(&byte,1)<0){eof=true;break;}std::this_thread::sleep_for(5ms);} assert(eof);
  link.close(); lines.clear(); connect(); peer=accept(listener,nullptr,nullptr); assert(peer>=0);
  assert(send(peer,baro.data(),baro.size(),0)==static_cast<ssize_t>(baro.size()));
  assert(decodeFrame(receive(lines,1)[0])->sequence==3);
  close(peer); link.close(); close(listener);
  TcpPort tcp; std::string err; assert(tcp.connectStep("invalid",9000,0.1,err)==TcpPort::State::Failed);
  bool failed=false; for(int i=0;i<100;++i){auto state=tcp.connectStep("127.0.0.1",port,0.2,err); if(state==TcpPort::State::Failed){failed=true;break;}std::this_thread::sleep_for(5ms);} assert(failed);
  std::cout << "PASS: nonblocking TCP, fragmented/coalesced CF1, BAT/PWR, command/ACK, EOF, reconnect, refused connection\n";
}
