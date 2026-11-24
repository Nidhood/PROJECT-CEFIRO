#include "cefiro_hardware_interface/wifi_manager.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cerrno>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <array>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <sys/socket.h>
#include <poll.h>
#include <iomanip>
#include <random>
#include <set>
#include <string_view>

extern char** environ;
using namespace std::chrono_literals;
namespace cefiro::hardware {
namespace {
using Args = std::vector<std::string>;
using Clock = std::chrono::steady_clock;
struct Result { int code{-1}; std::string out; };
// argv only (no shell). Drain stdout/stderr, bound output and wall time, reap children.
Result runCommand(Args args, const std::atomic<bool>* stop, int timeout_s) {
  if (stop && *stop) return {};
  int pipefd[2];
  if (::pipe2(pipefd, O_CLOEXEC) != 0) return {};
  const int flags = ::fcntl(pipefd[0], F_GETFL);
  if (flags < 0 || ::fcntl(pipefd[0], F_SETFL, flags | O_NONBLOCK) < 0) {
    ::close(pipefd[0]); ::close(pipefd[1]); return {};
  }
  // env sets the command locale without mutating the process's global environment.
  args.insert(args.begin(), {"env", "LC_ALL=C"});
  std::vector<char*> argv;
  for (auto& s : args) argv.push_back(s.data());
  argv.push_back(nullptr);
  posix_spawn_file_actions_t fa;
  if (posix_spawn_file_actions_init(&fa) != 0) {
    ::close(pipefd[0]); ::close(pipefd[1]); return {};
  }
  int rc = posix_spawn_file_actions_addopen(&fa, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
  if (!rc) rc = posix_spawn_file_actions_adddup2(&fa, pipefd[1], STDOUT_FILENO);
  if (!rc) rc = posix_spawn_file_actions_adddup2(&fa, pipefd[1], STDERR_FILENO);
  if (!rc) rc = posix_spawn_file_actions_addclose(&fa, pipefd[0]);
  if (!rc) rc = posix_spawn_file_actions_addclose(&fa, pipefd[1]);
  pid_t pid{};
  if (!rc) rc = posix_spawnp(&pid, argv[0], &fa, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&fa);
  ::close(pipefd[1]);
  if (rc) { ::close(pipefd[0]); return {}; }
  Result result;
  const auto deadline = Clock::now() + std::chrono::seconds(timeout_s);
  const auto drain = [&] {
    std::array<char, 4096> buf{};
    for (int i = 0; i < 32; ++i) {
      const auto n = ::read(pipefd[0], buf.data(), buf.size());
      if (n <= 0) break;
      if (result.out.size() < 65536) result.out.append(buf.data(), static_cast<std::size_t>(n));
    }
  };
  int child_status{};
  for (;;) {
    drain();
    const auto w = ::waitpid(pid, &child_status, WNOHANG);
    if (w == pid) { if (WIFEXITED(child_status)) result.code = WEXITSTATUS(child_status); break; }
    if (w < 0 && errno != EINTR) break;
    if ((stop && *stop) || Clock::now() >= deadline) {
      ::kill(pid, SIGKILL);
      while (::waitpid(pid, &child_status, 0) < 0 && errno == EINTR) {}
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  drain(); ::close(pipefd[0]);
  return result;
}
Result nm(Args args, const std::atomic<bool>* stop, int timeout_s = 6) {
  args.insert(args.begin(), {"nmcli", "--wait", std::to_string(std::max(1, timeout_s - 1))});
  return runCommand(std::move(args), stop, timeout_s);
}
std::vector<std::string> lines(const std::string& s) {
  std::istringstream in(s);
  std::vector<std::string> result;
  for (std::string line; std::getline(in, line);) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    result.push_back(std::move(line));
  }
  return result;
}
std::string chooseWifi(const std::string& text, const std::string& preferred, bool freeOnly = false) {
  std::string best; int rank = 999;
  for (const auto& line : lines(text)) {
    const auto a = line.find(':');
    const auto b = a == std::string::npos ? a : line.find(':', a + 1);
    if (b == std::string::npos || line.substr(a + 1, b - a - 1) != "wifi") continue;
    const auto device = line.substr(0, a); const auto state = line.substr(b + 1);
    if (preferred != "auto" && preferred != device) continue;
    if (freeOnly && state != "disconnected") continue;
    if (state == "unmanaged" || state == "unavailable" || state == "unknown") continue;
    // Prefer a free adapter to one currently connected to another Wi-Fi network.
    const int score = state == "disconnected" ? 0 : 1;
    if (score < rank || (score == rank && device < best)) { best = device; rank = score; }
  }
  return best;
}
bool containsLine(const std::string& text, const std::string& target) {
  for (const auto& l : lines(text)) if (l == target) return true;
  return false;
}
bool routeMatches(const std::string& text, const std::string& iface) {
  std::istringstream in(text);
  for (std::string token; in >> token;) if (token == "dev") { in >> token; return token == iface; }
  return false;
}
bool ipv4(const std::string& address) {
  in_addr ip{}; return ::inet_pton(AF_INET, address.c_str(), &ip) == 1 && ip.s_addr != 0;
}
// Directed broadcasts on existing local IPv4 interfaces. Nonce is not authentication.
std::string discoverUdp(const WifiConfig& cfg, const std::atomic<bool>& stop) {
  const int fd=::socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0);
  if(fd<0)return {};
  int one=1;
  if(::setsockopt(fd,SOL_SOCKET,SO_BROADCAST,&one,sizeof(one))<0){::close(fd);return {};}
  std::random_device rd;
  std::ostringstream nonce;nonce<<std::hex<<std::setfill('0')<<std::setw(8)<<uint32_t(rd())<<std::setw(8)<<uint32_t(rd());
  const std::string request="CEFIRO_DISCOVER_V1 "+nonce.str();
  const std::string response="CEFIRO_FMU_V3 "+nonce.str()+" "+std::to_string(cfg.port);
  ifaddrs* interfaces=nullptr;bool sent=false;
  if(::getifaddrs(&interfaces)==0){
    for(auto* it=interfaces;it;it=it->ifa_next){
      if(!it->ifa_addr || !it->ifa_broadaddr || it->ifa_addr->sa_family!=AF_INET ||
        !(it->ifa_flags&IFF_UP) || !(it->ifa_flags&IFF_BROADCAST) || (it->ifa_flags&IFF_LOOPBACK))continue;
      auto address=*reinterpret_cast<sockaddr_in*>(it->ifa_broadaddr);address.sin_port=htons(9001);
      if(::sendto(fd,request.data(),request.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&address),sizeof(address))==static_cast<ssize_t>(request.size()))sent=true;
    }
    ::freeifaddrs(interfaces);
  }
  std::set<std::string> candidates;
  const auto deadline=Clock::now()+400ms;
  while(sent && !stop && Clock::now()<deadline){
    pollfd p{fd,POLLIN,0};if(::poll(&p,1,40)<=0)continue;
    char buf[128];sockaddr_in src{};socklen_t len=sizeof(src);
    const auto n=::recvfrom(fd,buf,sizeof(buf),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&src),&len);
    if(n!=static_cast<ssize_t>(response.size()) || std::string_view(buf,response.size())!=response || ntohs(src.sin_port)!=9001)continue;
    char ip[INET_ADDRSTRLEN]{};
    if(::inet_ntop(AF_INET,&src.sin_addr,ip,sizeof(ip)) && cfg.host!=ip)candidates.insert(ip);
  }
  ::close(fd);
  if(candidates.size()>1)return "MULTIPLE";
  return candidates.empty()?std::string{}:*candidates.begin();
}
std::string resolveNetwork(const WifiConfig& cfg,const std::atomic<bool>& stop){
  if(ipv4(cfg.network_host))return cfg.network_host;
  if(cfg.network_host=="cefiro.local"){
    const auto reply=discoverUdp(cfg,stop);if(!reply.empty())return reply;
  }
  const auto r=runCommand({"getent","ahostsv4",cfg.network_host},&stop,3);
  if(r.code)return {};
  std::set<std::string> addresses;
  for(const auto& line:lines(r.out)){
    std::istringstream in(line);std::string ip;in>>ip;
    if(ipv4(ip) && ip!=cfg.host)addresses.insert(ip);
  }
  if(addresses.size()>1)return "MULTIPLE";
  return addresses.empty()?std::string{}:*addresses.begin();
}
std::string createUuid() {
  std::ifstream in("/proc/sys/kernel/random/uuid");
  std::string uuid; in >> uuid; return uuid;
}
// Secrets never appear in child argv or ROS parameters/logs. File is mode 0600.
class PasswordFile {
public:
  explicit PasswordFile(const std::string& password) {
    char name[] = "/tmp/cefiro-wifi-XXXXXX";
    const int fd = ::mkstemp(name);
    if (fd < 0) return;
    path = name;
    const std::string content = "802-11-wireless-security.psk:" + password + "\n";
    std::size_t sent = 0;
    while (sent < content.size()) {
      const auto n = ::write(fd, content.data() + sent, content.size() - sent);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) break;
      sent += static_cast<std::size_t>(n);
    }
    ::close(fd);
    if (sent != content.size()) { ::unlink(path.c_str()); path.clear(); }
  }
  ~PasswordFile() { if (!path.empty()) ::unlink(path.c_str()); }
  PasswordFile(const PasswordFile&) = delete;
  std::string path;
};
std::string failure(const Result& r, const std::string& stage) {
  // Don't propagate raw subprocess output: it may contain private network details.
  if (r.code == 127) return "missing_nmcli_or_ip";
  if (r.code == 8) return "networkmanager_not_running";
  if (r.out.find("not authorized") != std::string::npos || r.out.find("permission") != std::string::npos ||
      r.out.find("Not authorized") != std::string::npos) return "networkmanager_permission_denied";
  if (r.code < 0 || r.code == 3) return stage + "_timeout";
  return stage + "_failed_rc_" + std::to_string(r.code);
}
}  // namespace
WifiManager::WifiManager(WifiConfig config) : config_(std::move(config)) {
  in_addr addr{};
  if ((config_.mode != "direct" && config_.mode != "network" && config_.mode != "auto") ||
      config_.network_host.empty() || config_.network_host[0]=='-' || config_.port < 1 || config_.port > 65535 ||
      !std::isfinite(config_.discovery_grace_s) || config_.discovery_grace_s < 0 || config_.discovery_grace_s > 120 ||
      ::inet_pton(AF_INET, config_.host.c_str(), &addr) != 1 || config_.ssid.empty() || config_.ssid.size() > 32 || config_.ssid.find_first_of("\r\n") != std::string::npos ||
      config_.interface.empty() || !std::isfinite(config_.retry_s) || config_.retry_s < 0.5 || config_.retry_s > 120)
    throw std::invalid_argument("Invalid managed Wi-Fi configuration");
  worker_ = std::thread([this] { run(); });
}
WifiManager::~WifiManager() { stop_ = true; if (worker_.joinable()) worker_.join(); }
WifiStatus WifiManager::status() const { std::lock_guard lock(mutex_); return status_; }
void WifiManager::publish(bool ready, std::string state, const std::string& iface, const std::string& host) {
  std::lock_guard lock(mutex_); status_ = {ready, std::move(state), iface, ready ? (host.empty()?config_.host:host) : std::string{}};
}
// Choose once per launch. After LAN selection, rediscover on disconnect but do not roam.
bool WifiManager::runNetwork(){
  const auto started=Clock::now();bool selected=false;
  while(!stop_){
    if(link_active_){std::this_thread::sleep_for(100ms);continue;}
    const auto host=resolveNetwork(config_,stop_);
    if(stop_)return true;
    if(link_active_)continue;
    if(host=="MULTIPLE")publish(false,"multiple_cefiro_addresses_use_wifi_host");
    else if(!host.empty()){selected=true;publish(true,"network_discovered",{},host);}
    else{
      publish(false,"waiting_for_cefiro_on_lan");
      if(!selected && config_.mode=="auto" && config_.manage_ap &&
         Clock::now()-started>=std::chrono::duration<double>(config_.discovery_grace_s)){
        const auto devices=nm({"-t","-e","no","-f","DEVICE,TYPE,STATE","device","status"},&stop_);
        if(!devices.code && !chooseWifi(devices.out,config_.interface,true).empty())return false;
        publish(false,"lan_not_found_preserving_existing_wifi");
      }
    }
    const auto end=Clock::now()+std::chrono::duration<double>(config_.retry_s);
    while(!stop_ && Clock::now()<end)std::this_thread::sleep_for(50ms);
  }
  return true;
}
void WifiManager::run() {
  try {
    if(config_.mode!="direct" && runNetwork()){publish(false,"stopped");return;}
    if(!config_.manage_ap){publish(true,"direct_external");while(!stop_)std::this_thread::sleep_for(100ms);return;}
  }catch(...){publish(false,"network_discovery_exception");return;}
  std::string uuid, iface;
  bool profile_created = false;
  const auto pause = [this] {
    const auto end = Clock::now() + std::chrono::duration<double>(config_.retry_s);
    while (!stop_ && Clock::now() < end) std::this_thread::sleep_for(50ms);
  };
  // Only this session's profile is deleted; never delete/mutate Ethernet or saved Wi-Fi profiles.
  const auto cleanup = [&] {
    if (!uuid.empty()) (void)nm({"connection", "delete", "uuid", uuid}, nullptr, 4);
    uuid.clear(); profile_created = false; iface.clear();
  };
  try {
    const char* supplied = std::getenv("CEFIRO_WIFI_PASSWORD");
    const std::string password = supplied ? supplied : "Cefiro2026!";  // Lab default; override for other APs.
    if (password.size() < 8 || password.size() > 64 || password.find_first_of("\r\n") != std::string::npos) {
      publish(false, "invalid_wifi_password"); return;
    }
    while (!stop_) {
      if (profile_created) {
        const auto active = nm({"-t", "-e", "no", "-f", "UUID,DEVICE", "connection", "show", "--active"}, &stop_);
        if (!active.code && containsLine(active.out, uuid + ":" + iface)) {
          // Refuse to dial TCP if another interface owns the target route (overlap/VPN).
          const auto route = runCommand({"ip", "-4", "route", "get", config_.host}, &stop_, 3);
          if (!route.code && routeMatches(route.out, iface)) publish(true, "associated", iface);
          else publish(false, route.code == 127 ? "missing_iproute2" : "target_route_not_wifi", iface);
          pause(); continue;
        }
        publish(false, "wifi_link_lost", iface);
        if (stop_) break;
        cleanup(); // re-discover if adapter was removed or NM was restarted
      }
      const auto devices = nm({"-t", "-e", "no", "-f", "DEVICE,TYPE,STATE", "device", "status"}, &stop_);
      if (devices.code) { publish(false, failure(devices, "device_discovery")); pause(); continue; }
      iface = chooseWifi(devices.out, config_.interface, config_.mode=="auto");
      if (iface.empty()) { publish(false, "waiting_for_wifi_adapter"); pause(); continue; }
      publish(false, "searching_ap", iface);
      const auto scan = nm({"-t", "-e", "no", "-f", "SSID", "device", "wifi", "list", "ifname", iface, "--rescan", "yes"}, &stop_, 12);
      if (scan.code) { publish(false, failure(scan, "wifi_scan"), iface); pause(); continue; }
      if (!containsLine(scan.out, config_.ssid)) { publish(false, "waiting_for_ap", iface); pause(); continue; }
      if (stop_) break;
      uuid = createUuid();
      if (uuid.empty()) { publish(false, "uuid_unavailable", iface); pause(); continue; }
      const auto add = nm({"connection", "add", "save", "no", "type", "wifi", "ifname", iface,
        "con-name", "cefiro-session-" + uuid, "ssid", config_.ssid, "connection.uuid", uuid,
        "connection.autoconnect", "no", "ipv4.method", "auto", "ipv4.never-default", "yes",
        "ipv4.ignore-auto-dns", "yes", "ipv4.ignore-auto-routes", "yes", "ipv4.may-fail", "no",
        "ipv6.method", "disabled", "ipv6.never-default", "yes", "ipv6.ignore-auto-dns", "yes",
        "wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk-flags", "2"}, &stop_);
      if (add.code) { publish(false, failure(add, "profile_create"), iface); cleanup(); pause(); continue; }
      profile_created = true;
      publish(false, "authenticating", iface);
      {
        PasswordFile secret(password);
        if (secret.path.empty()) { publish(false, "password_file_failed", iface); cleanup(); pause(); continue; }
        const auto up = nm({"connection", "up", "uuid", uuid, "ifname", iface, "passwd-file", secret.path}, &stop_, 25);
        if (up.code) { publish(false, failure(up, "association"), iface); cleanup(); pause(); continue; }
      }
      // Check active profile and route at top of loop before TCP can connect.
    }
  } catch (...) { publish(false, "wifi_manager_exception", iface); }
  if (config_.disconnect_on_exit || !profile_created) cleanup();
  publish(false, "stopped");
}
}  // namespace cefiro::hardware
