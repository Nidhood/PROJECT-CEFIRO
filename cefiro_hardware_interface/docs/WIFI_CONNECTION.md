# Wi-Fi AP + STA / native CF1 TCP (v0.8.0)

The ROS package uses the same node, topics, parser, watchdog and transports as v0.7.0. STA mode assumes the host already has a LAN connection, e.g. to the phone hotspot; no Wi-Fi reconfiguration is necessary then. A discovery worker broadcasts a nonce-tagged request to local directed IPv4 broadcasts on UDP 9001 and uses the unique reply IP, or resolves `cefiro.local` via `getent ahostsv4`. TCP stream uses port 9000. If UDP multicast/broadcast is filtered, specify the ESP DHCP IP directly. Discovery is not authentication. For DNS resolution install `avahi-daemon libnss-mdns` if required on Ubuntu; directed UDP or explicit IP works without them. No cloud or Internet relay is involved.

Build: `colcon build --symlink-install --packages-up-to cefiro_hardware_interface` in your sourced ROS 2 Jazzy workspace, then `source install/setup.bash`.

| Parameter | Default | Meaning |
|---|---|---|
| wifi_mode | auto | LAN first, direct AP fallback only if free adapter; network = LAN only, direct = force AP |
| wifi_host | cefiro.local | LAN hostname or explicit numeric IPv4 |
| wifi_direct_host | 192.168.4.1 | Direct AP numeric IPv4 |
| wifi_discovery_grace_s | 15.0 | Initial grace before auto fallback |
| wifi_ssid | CEFIRO_FMU_V3 | Direct AP, not phone hotspot |
| wifi_auto_connect | true | NetworkManager management of direct AP only |
| wifi_interface | auto | Direct AP adapter selection |
| wifi_disconnect_on_exit | true | Remove owned temporary AP profile on normal exit |
| wifi_port | 9000 | TCP port |
| reconnect_s | 2.0 | TCP retries |
| wifi_connect_timeout_s | 3.0 | TCP connection timeout |
| wifi_rx_timeout_s | 3.0 | CF1 receive timeout |
| enable_command_tx | false | Explicit opt-in only |
| rpm_to_dshot_enabled | false | Explicit opt-in only |
| dshot_max | 2047 | Manual protocol ceiling; lower cap configurable |

Examples:

```
ros2 launch cefiro_hardware_interface hardware_wifi.launch.py
ros2 launch cefiro_hardware_interface hardware_wifi.launch.py wifi_mode:=network
ros2 launch cefiro_hardware_interface hardware_wifi.launch.py wifi_mode:=network wifi_host:=192.168.43.20
ros2 launch cefiro_hardware_interface hardware_wifi.launch.py wifi_mode:=direct
ros2 launch cefiro_hardware_interface hardware_wifi.launch.py wifi_mode:=direct wifi_auto_connect:=false
ros2 launch cefiro_hardware_interface hardware.launch.py port:=/dev/ttyUSB0 protocol_mode:=cf1
```

The explicit IP is illustrative. `auto` does not replace any active Wi-Fi association, even if LAN discovery fails. It will attempt to create a direct AP profile only if a free managed Wi-Fi adapter exists. A selected mode is pinned for the launch; LAN IP can be rediscovered after disconnect but no migration to/from direct occurs until relaunch. For an already-associated direct AP choose `direct wifi_auto_connect:=false`. Direct mode with management may replace the existing association on the selected adapter. Ethernet route/DNS is not modified. AP profile uses WPA-PSK, no default routes, no DHCP DNS, no extra routes; route to 192.168.4.1 checked. Direct mode needs nmcli, iproute2, NetworkManager/PolicyKit permissions. `CEFIRO_WIFI_PASSWORD` overrides the private direct AP default, not STA credentials. STA credentials are stored on ESP, never needed by ROS.

Docker on Linux: host network recommended for broadcast/mDNS, or explicit STA IP reachable from container. LAN mode needs no access to NetworkManager or host D-Bus. Managed direct mode requires host network and permitted NetworkManager D-Bus access. In Docker Desktop use host OS Wi-Fi configuration and explicit STA IP or manual AP mode. Do not blindly grant privileged access.

One TCP client (close UI/monitor first). `/real/cefiro/hardware/status` retains all fields including `transport_connected`, `wifi_connected`, `wifi_network_state`, `wifi_network_interface`; network state `network_discovered` means candidate IPv4 found, not guaranteed connected. Topic publishers and subscribers unchanged, including BAT/PWR. Control TX defaults disabled. In v0.8.2 a matching CMD_PING ACK is required for transport_connected; PING is read-only and always enabled. USB–USART uses hardware.launch.py, see the archive connection guide. Disconnect clears queued commands and host arm state; requires new explicit enable/arm. Independent STM32 watchdog required. No network identity/authentication or TLS, and CRC does not authenticate. Private LAN only; no port forwarding.

Host tests: `bash tools/verify_protocol.sh`, `bash tools/verify_manual_keepalive.sh`, `bash tools/verify_tcp_transport.sh`, `bash tools/verify_wifi_manager.sh`, `bash tools/verify_lan_discovery.sh`, `bash tools/verify_udp_discovery.sh` (isolated test LAN). Full ROS build and physical testing are pending.
