# 0.8.2 — Companion USB–USART / verified CF1 sessions

- Both Wi-Fi modes preserved; default auto, explicit network/direct.
- Serial launch defaults to CF1, configurable baudrate and port.
- Read-only PING every 200 ms and matching ACK before state-changing commands; no auto re-arm.
- Exclusive serial descriptor lock and MCU session arbitration.

# Changelog

## 0.8.1

- Default to LAN-only network mode to preserve the mobile hotspot connection. Explicit auto/direct modes remain available.
- No CF1, topic, actuator or watchdog change.

## 0.8.0

- LAN-first UDP and mDNS/NSS discovery, with explicit numeric IP override.
- `auto`, `network`, `direct`; automatic AP fallback only on unused adapters.
- Pin mode for each launch; rediscover LAN IP on reconnect; no control replay.
- Retain all existing topics, CF1 parser and safety state handling.

## 0.7.0
- Automatic Wi-Fi discovery and WPA-PSK association in a dedicated C++ worker using NetworkManager's nmcli client.
- Session-only profile without default route or DHCP DNS; pre-TCP route validation, reconnect and optional cleanup.
- New launch arguments and link status fields; no changes to the existing ROS publishers/subscriptions, CF1 codec or manual control watchdog.
- Private lab default credential can be overridden with CEFIRO_WIFI_PASSWORD. No secret in ROS parameter values or child arguments.
- Added fake-NetworkManager integration test for association, missing AP, wrong route, loss/reconnect, permissions and cleanup.
