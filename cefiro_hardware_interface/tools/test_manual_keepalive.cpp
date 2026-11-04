#include "cefiro_hardware_interface/manual_keepalive.hpp"
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <optional>
using namespace std::chrono_literals;
using cefiro::hardware::ManualKeepalive;

int main() {
  ManualKeepalive hold;
  hold.configure(0.040, 0.600);
  const auto origin = ManualKeepalive::Time{} + 10s;
  const ManualKeepalive::MotorCodes motors{48, 0, 120, 240};
  assert(!hold.due(origin));
  assert(!hold.expired(origin + 590ms, origin));
  assert(hold.expired(origin + 610ms, origin)); // No GUI setpoint after ARM.
  hold.update(motors, origin);
  assert(hold.fresh(origin + 500ms));
  assert(hold.due(origin));
  hold.transmitted(origin);
  assert(!hold.due(origin + 30ms));
  assert(hold.due(origin + 40ms));
  hold.transmitted(origin + 40ms);
  assert(!hold.due(origin + 50ms));
  assert(hold.due(origin + 80ms));
  assert(!hold.expired(origin + 300ms, origin));
  assert(hold.expired(origin + 601ms, origin));
  assert(!hold.fresh(origin + 601ms));
  assert(!hold.due(origin + 601ms));
  // A fresh GUI command starts a new bounded window.
  hold.update({0,0,0,0}, origin + 550ms);
  assert(hold.fresh(origin + 850ms));
  assert(hold.due(origin + 850ms));
  assert(hold.setpoint() && *hold.setpoint() == ManualKeepalive::MotorCodes({0,0,0,0}));
  // Hardware node must call reset() on STOP, reconnection or new ARM request.
  hold.reset();
  assert(!hold.fresh(origin + 850ms));
  assert(!hold.due(origin + 850ms));
  assert(hold.expired(origin + 1460ms, origin + 850ms));
  // Integration-like deterministic simulation: Qt goes quiet for 460 ms
  // while the Raspberry maintains 40 ms DShot refresh; MCU watchdog=300 ms.
  ManualKeepalive stream;
  stream.configure(0.040, 0.600);
  const auto base = ManualKeepalive::Time{} + 100s;
  const ManualKeepalive::MotorCodes setpoint{80, 0, 0, 0};
  stream.update(setpoint, base);
  std::optional<ManualKeepalive::Time> mcu_last_rx;
  int frames_sent = 0;
  for (int ms=0;ms<=520;ms+=10) {
    const auto now = base + std::chrono::milliseconds(ms);
    if (ms==520) stream.update(setpoint, now);
    if (stream.due(now)) {
      stream.transmitted(now);
      mcu_last_rx = now;
      ++frames_sent;
    }
    assert(mcu_last_rx);
    assert(now - *mcu_last_rx < 300ms);
  }
  assert(frames_sent >= 12);
  // If GUI dies, the bridge MUST stop refreshing after 600 ms.
  for (int ms=530;ms<=1130;ms+=10) {
    const auto now = base + std::chrono::milliseconds(ms);
    if (stream.due(now)) stream.transmitted(now);
  }
  assert(stream.expired(base + 1130ms, base));
  assert(!stream.due(base + 1130ms));
  std::cout << "Manual keepalive, 40ms refresh / 600ms GUI lease: PASS\n";
}
