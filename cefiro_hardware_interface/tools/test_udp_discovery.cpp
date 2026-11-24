#include "cefiro_hardware_interface/wifi_manager.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
using namespace cefiro::hardware;
int main(){
  WifiConfig cfg;cfg.mode="network";cfg.retry_s=.5;
  WifiManager manager(cfg);
  const auto end=std::chrono::steady_clock::now()+5s;
  while(std::chrono::steady_clock::now()<end){
    auto s=manager.status();
    if(s.ready && s.state=="network_discovered"){
      std::cout<<"UDP discovery from local simulated peer: "<<s.host<<" PASS\n";return 0;
    }
    std::this_thread::sleep_for(20ms);
  }
  std::cerr<<"UDP simulation unavailable or failed\n";return 1;
}
