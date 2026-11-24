#pragma once
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace cefiro::hardware {
struct WifiConfig {
  std::string ssid{"CEFIRO_FMU_V3"};
  std::string interface{"auto"};
  std::string host{"192.168.4.1"};
  std::string mode{"direct"};
  std::string network_host{"cefiro.local"};
  int port{9000};
  bool manage_ap{true};
  double discovery_grace_s{15.0};
  bool disconnect_on_exit{true};
  double retry_s{2.0};
};
struct WifiStatus {
  bool ready{false};
  std::string state{"starting"};
  std::string interface;
  std::string host;
};
// Owns only a transient NetworkManager profile. No network operations on ROS executor.
class WifiManager {
public:
  explicit WifiManager(WifiConfig config);
  ~WifiManager();
  WifiManager(const WifiManager&) = delete;
  WifiManager& operator=(const WifiManager&) = delete;
  WifiStatus status() const;
  void setLinkActive(bool active) noexcept { link_active_ = active; }
private:
  void run();
  void publish(bool ready, std::string state, const std::string& iface = {}, const std::string& host = {});
  bool runNetwork();
  WifiConfig config_;
  std::atomic<bool> stop_{false}, link_active_{false};
  mutable std::mutex mutex_;
  WifiStatus status_;
  std::thread worker_;
};
}  // namespace cefiro::hardware
