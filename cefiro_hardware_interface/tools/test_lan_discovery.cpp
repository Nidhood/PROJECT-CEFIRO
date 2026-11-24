#include "cefiro_hardware_interface/wifi_manager.hpp"
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
using namespace cefiro::hardware;
namespace fs=std::filesystem;
static void put(const fs::path& p,const std::string& text){std::ofstream f(p);f<<text;}
static bool wait(WifiManager& m,const std::string& state,const std::string& host={}){
  auto end=std::chrono::steady_clock::now()+7s;
  while(std::chrono::steady_clock::now()<end){auto s=m.status();if(s.state==state && s.host==host)return true;std::this_thread::sleep_for(20ms);}
  auto s=m.status();std::cerr<<"expected "<<state<<" / "<<host<<" got "<<s.state<<" / "<<s.host<<'\n';return false;
}
int main(){
  fs::path dir=std::getenv("CF_WIFI_TEST_DIR");
  WifiConfig cfg;cfg.retry_s=.5;cfg.mode="auto";cfg.discovery_grace_s=0;cfg.network_host="lab-cefiro.local";
  put(dir/"busy","yes");
  {
    WifiManager m(cfg);
    assert(wait(m,"lan_not_found_preserving_existing_wifi"));assert(!fs::exists(dir/"profile"));
    put(dir/"resolve","192.168.43.20");assert(wait(m,"network_discovered","192.168.43.20"));
    m.setLinkActive(true);put(dir/"resolve","192.168.43.21");std::this_thread::sleep_for(1500ms);
    assert(m.status().host=="192.168.43.20");m.setLinkActive(false);
    assert(wait(m,"network_discovered","192.168.43.21"));
    fs::remove(dir/"busy");fs::remove(dir/"resolve");
    assert(wait(m,"waiting_for_cefiro_on_lan"));assert(!fs::exists(dir/"profile")); // no migration after LAN chosen
  }
  put(dir/"ap","yes");
  { WifiManager m(cfg);assert(wait(m,"associated","192.168.4.1")); }
  assert(!fs::exists(dir/"profile"));
  cfg.mode="network";cfg.network_host="192.168.43.50";
  {WifiManager m(cfg);assert(wait(m,"network_discovered","192.168.43.50"));assert(!fs::exists(dir/"profile"));}
  cfg.mode="direct";cfg.manage_ap=false;
  {WifiManager m(cfg);assert(wait(m,"direct_external","192.168.4.1"));}
  std::cout<<"LAN selection, IP refresh, active session pinning, non-disruptive AP fallback: PASS\n";
}
