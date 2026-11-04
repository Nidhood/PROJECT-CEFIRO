#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

namespace cefiro::hardware {

// A bounded, independently scheduled manual DShot stream.
// It bridges brief GUI/DDS scheduling gaps, but NEVER keeps a stale GUI
// command alive forever. The STM32 retains its own independent 300 ms TTL.
class ManualKeepalive {
 public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  using MotorCodes = std::array<std::uint16_t, 4>;

  void configure(double period_s, double gui_timeout_s) noexcept {
    period_ = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(period_s));
    timeout_ = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(gui_timeout_s));
  }

  void reset() noexcept {
    codes_.reset();
    last_gui_.reset();
    last_tx_.reset();
  }

  void update(const MotorCodes& codes, Time now) noexcept {
    codes_ = codes;
    last_gui_ = now;
  }

  [[nodiscard]] bool fresh(Time now) const noexcept {
    return codes_.has_value() && last_gui_.has_value() &&
           now >= *last_gui_ && now - *last_gui_ <= timeout_;
  }

  // If the first setpoint has not arrived after ARM, enforce the same
  // deadline from the ARM ACK; never hold an armed ESC indefinitely at zero.
  [[nodiscard]] bool expired(Time now, Time armed_at) const noexcept {
    if (last_gui_) return now >= *last_gui_ && now - *last_gui_ > timeout_;
    return now >= armed_at && now - armed_at > timeout_;
  }

  [[nodiscard]] bool due(Time now) const noexcept {
    return fresh(now) &&
           (!last_tx_ || (now >= *last_tx_ && now - *last_tx_ >= period_));
  }

  [[nodiscard]] const MotorCodes* setpoint() const noexcept {
    return codes_ ? &*codes_ : nullptr;
  }

  void transmitted(Time now) noexcept { last_tx_ = now; }

  [[nodiscard]] double guiAgeSeconds(Time now) const noexcept {
    return last_gui_ ? std::chrono::duration<double>(now - *last_gui_).count() : -1.0;
  }

  [[nodiscard]] double txAgeSeconds(Time now) const noexcept {
    return last_tx_ ? std::chrono::duration<double>(now - *last_tx_).count() : -1.0;
  }

 private:
  Clock::duration period_{std::chrono::milliseconds(40)};
  Clock::duration timeout_{std::chrono::milliseconds(600)};
  std::optional<MotorCodes> codes_;
  std::optional<Time> last_gui_;
  std::optional<Time> last_tx_;
};

}  // namespace cefiro::hardware
