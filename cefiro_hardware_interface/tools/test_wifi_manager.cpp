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
namespace fs = std::filesystem;
static void put(const fs::path& p, const std::string& text) {
  std::ofstream f(p); f << text;
}
static bool waitFor(WifiManager& manager, const std::string& state, bool ready) {
  const auto until = std::chrono::steady_clock::now() + 8s;
  while (std::chrono::steady_clock::now() < until) {
    const auto s = manager.status();
    if (s.state == state && s.ready == ready) return true;
    std::this_thread::sleep_for(20ms);
  }
  std::cerr << "Expected " << state << ", actual " << manager.status().state << '\n';
  return false;
}
int main() {
  const fs::path dir = std::getenv("CF_WIFI_TEST_DIR");
  WifiConfig cfg; cfg.retry_s = 0.5;
  {
    WifiManager manager(cfg);
    assert(waitFor(manager, "waiting_for_ap", false));
    put(dir/"ap", "yes");
    assert(waitFor(manager, "associated", true));
    assert(manager.status().interface == "wlan1");
    put(dir/"route", "eth0");
    assert(waitFor(manager, "target_route_not_wifi", false));
    put(dir/"route", "wlan1");
    assert(waitFor(manager, "associated", true));
    put(dir/"ap", "no");
    fs::remove(dir/"active");
    assert(waitFor(manager, "waiting_for_ap", false));
    put(dir/"ap", "yes");
    assert(waitFor(manager, "associated", true));
  }
  assert(!fs::exists(dir/"active"));
  assert(!fs::exists(dir/"profile"));
  put(dir/"deny", "yes");
  {
    WifiManager manager(cfg);
    assert(waitFor(manager, "networkmanager_permission_denied", false));
  }
  fs::remove(dir/"deny");
  cfg.disconnect_on_exit = false;
  {
    WifiManager manager(cfg);
    assert(waitFor(manager, "associated", true));
  }
  assert(fs::exists(dir/"active"));
  assert(fs::exists(dir/"profile"));
  std::cout << "Wi-Fi manager fake-NM integration: PASS\n";
}
