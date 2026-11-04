#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace cefiro::hardware {

constexpr std::size_t kMaxLineBytes = 2048;

struct ImuReading {
  std::array<double, 3> acceleration_ms2{};
  std::array<double, 3> gyro_dps{};
  double temperature_c{0.0};
};
struct MagReading { std::array<double, 3> magnetic_uT{}; };
struct BaroReading {
  double pressure_hpa{0.0};
  double temperature_c{0.0};
  double altitude_isa_m{0.0};
  double pressure_raw_hpa{0.0};
  double altitude_relative_m{0.0};
};

enum class LegacyKind { None, Imu, Mag, Baro, SensorError };
struct LegacyReading {
  LegacyKind kind{LegacyKind::None};
  ImuReading imu{};
  MagReading mag{};
  BaroReading baro{};
  std::string sensor;
  std::string error;
};

std::optional<LegacyReading> parseLegacy(std::string_view line);

struct Frame {
  std::uint16_t sequence{0};
  std::string kind;
  std::vector<std::string> fields;
};

std::uint16_t crc16(std::string_view data);
std::optional<Frame> decodeFrame(std::string_view line);
std::optional<std::string> encodeFrame(std::uint16_t sequence,
                                       std::string_view kind,
                                       const std::vector<std::string>& fields);
std::vector<std::string> splitCsv(std::string_view text);
std::optional<double> parseFinite(std::string_view text);
std::optional<std::int64_t> parseInteger(std::string_view text,
                                          std::int64_t min,
                                          std::int64_t max);

class LineBuffer {
public:
  // Returns complete lines only, discards any overlength line safely.
  std::vector<std::string> feed(std::string_view chunk);
  void clear();
  std::uint64_t dropped() const noexcept { return dropped_; }
private:
  std::string partial_;
  bool dropping_{false};
  std::uint64_t dropped_{0};
};

}  // namespace cefiro::hardware
