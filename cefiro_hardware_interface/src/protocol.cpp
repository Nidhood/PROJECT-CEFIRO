#include "cefiro_hardware_interface/protocol.hpp"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>

namespace cefiro::hardware {
namespace {
constexpr const char* kNumber = R"(([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?))";

const std::regex& imuPattern() {
  static const std::regex pattern(
      std::string(R"(^IMU\s*\|\s*A\[m/s2\]\s*X=)") + kNumber +
      R"(\s+Y=)" + kNumber + R"(\s+Z=)" + kNumber +
      R"(\s*\|\s*G\[deg/s\]\s*X=)" + kNumber +
      R"(\s+Y=)" + kNumber + R"(\s+Z=)" + kNumber +
      R"(\s*\|\s*T=)" + kNumber + R"(\s*C\s*$)");
  return pattern;
}
const std::regex& magPattern() {
  static const std::regex pattern(
      std::string(R"(^MAG\s*\|\s*B\[uT\]\s*X=)") + kNumber +
      R"(\s+Y=)" + kNumber + R"(\s+Z=)" + kNumber + R"(\s*$)");
  return pattern;
}
const std::regex& baroPattern() {
  static const std::regex pattern(
      std::string(R"(^BARO\s*\|\s*P=)") + kNumber +
      R"(\s+hPa\s*\|\s*T=)" + kNumber +
      R"(\s+C\s*\|\s*Alt_ISA=)" + kNumber +
      R"(\s+m\s*\|\s*P_RAW=)" + kNumber +
      R"(\s+hPa\s*\|\s*Z_REL=)" + kNumber + R"(\s+m\s*$)");
  return pattern;
}
std::string_view trim(std::string_view s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string_view::npos) return {};
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}
bool validKind(std::string_view kind) {
  if (kind.empty() || kind.size() > 32 || kind[0] < 'A' || kind[0] > 'Z') return false;
  for (char c : kind) {
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_')) return false;
  }
  return true;
}
bool numericMatches(const std::smatch& match, std::size_t count, std::vector<double>& out) {
  if (match.size() != count + 1) return false;
  out.clear();
  out.reserve(count);
  for (std::size_t i = 1; i <= count; ++i) {
    const auto number = parseFinite(match[i].str());
    if (!number) return false;
    out.push_back(*number);
  }
  return true;
}
}  // namespace

std::optional<double> parseFinite(std::string_view text) {
  if (text.empty() || text.size() > 96) return std::nullopt;
  std::string tmp(text);
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(tmp.c_str(), &end);
  if (end == tmp.c_str() || *end != '\0' || errno == ERANGE || !std::isfinite(v)) return std::nullopt;
  return v;
}

std::optional<std::int64_t> parseInteger(std::string_view text, std::int64_t min, std::int64_t max) {
  if (text.empty() || text.size() > 20) return std::nullopt;
  std::int64_t v{0};
  const auto result = std::from_chars(text.data(), text.data() + text.size(), v);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || v < min || v > max) return std::nullopt;
  return v;
}

std::optional<LegacyReading> parseLegacy(std::string_view line) {
  line = trim(line);
  if (line.empty() || line.size() > kMaxLineBytes) return std::nullopt;
  std::string s(line);
  LegacyReading result;
  for (const auto name : {"IMU", "MAG", "BARO"}) {
    const auto label = s.find(name);
    const auto pipe = s.find('|');
    if (label == 0 && pipe != std::string::npos) {
      const auto rest = trim(std::string_view(s).substr(pipe + 1));
      if (rest.starts_with("ERROR:") || rest.starts_with("CALIBRATING:")) {
        result.kind = LegacyKind::SensorError;
        result.sensor = name;
        result.error = std::string(rest);
        return result;
      }
    }
  }
  std::smatch match;
  std::vector<double> values;
  if (std::regex_match(s, match, imuPattern()) && numericMatches(match, 7, values)) {
    result.kind = LegacyKind::Imu;
    std::copy_n(values.begin(), 3, result.imu.acceleration_ms2.begin());
    std::copy_n(values.begin() + 3, 3, result.imu.gyro_dps.begin());
    result.imu.temperature_c = values[6];
    return result;
  }
  if (std::regex_match(s, match, magPattern()) && numericMatches(match, 3, values)) {
    result.kind = LegacyKind::Mag;
    std::copy_n(values.begin(), 3, result.mag.magnetic_uT.begin());
    return result;
  }
  if (std::regex_match(s, match, baroPattern()) && numericMatches(match, 5, values)) {
    if (values[0] <= 0.0 || values[3] <= 0.0) return std::nullopt;
    result.kind = LegacyKind::Baro;
    result.baro = {values[0], values[1], values[2], values[3], values[4]};
    return result;
  }
  return std::nullopt;
}

std::uint16_t crc16(std::string_view data) {
  std::uint16_t crc = 0xFFFF;
  for (unsigned char ch : data) {
    crc ^= static_cast<std::uint16_t>(ch) << 8U;
    for (int i = 0; i < 8; ++i) {
      crc = (crc & 0x8000U) ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                            : static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

std::vector<std::string> splitCsv(std::string_view s) {
  std::vector<std::string> result;
  if (s.empty()) return result;
  while (true) {
    const auto sep = s.find(',');
    result.emplace_back(s.substr(0, sep));
    if (sep == std::string_view::npos) return result;
    s.remove_prefix(sep + 1);
  }
}

std::optional<Frame> decodeFrame(std::string_view line) {
  line = trim(line);
  if (line.size() > kMaxLineBytes || line.size() < 13 || line[0] != '@') return std::nullopt;
  const auto star = line.find('*');
  if (star == std::string_view::npos || line.find('*', star + 1) != std::string_view::npos || star + 5 != line.size()) return std::nullopt;
  const auto core = line.substr(1, star - 1);
  const auto crc_text = line.substr(star + 1, 4);
  unsigned int expected = 0;
  const auto parsed = std::from_chars(crc_text.data(), crc_text.data() + crc_text.size(), expected, 16);
  if (parsed.ec != std::errc{} || parsed.ptr != crc_text.data() + 4 || crc16(core) != expected) return std::nullopt;
  const auto v1 = core.find('|');
  const auto v2 = core.find('|', v1 == std::string_view::npos ? 0 : v1 + 1);
  const auto v3 = core.find('|', v2 == std::string_view::npos ? 0 : v2 + 1);
  if (v1 == std::string_view::npos || v2 == std::string_view::npos || v3 == std::string_view::npos) return std::nullopt;
  if (core.substr(0, v1) != "CF1") return std::nullopt;
  const auto sequence = parseInteger(core.substr(v1 + 1, v2 - v1 - 1), 0, 65535);
  if (!sequence) return std::nullopt;
  const auto kind = core.substr(v2 + 1, v3 - v2 - 1);
  if (!validKind(kind)) return std::nullopt;
  Frame frame;
  frame.sequence = static_cast<std::uint16_t>(*sequence);
  frame.kind = std::string(kind);
  frame.fields = splitCsv(core.substr(v3 + 1));
  for (const auto& f : frame.fields) {
    if (f.size() > 96 || f.find_first_of("\r\n|*@") != std::string::npos) return std::nullopt;
  }
  return frame;
}

std::optional<std::string> encodeFrame(std::uint16_t sequence,
                                       std::string_view kind,
                                       const std::vector<std::string>& fields) {
  if (!validKind(kind)) return std::nullopt;
  for (const auto& value : fields) {
    if (value.size() > 96 || value.find_first_of("\r\n|*,@") != std::string::npos) return std::nullopt;
  }
  std::ostringstream base;
  base << "CF1|" << sequence << '|' << kind << '|';
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i) base << ',';
    base << fields[i];
  }
  const auto core = base.str();
  std::ostringstream frame;
  frame << '@' << core << '*' << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << crc16(core) << '\n';
  if (frame.str().size() > kMaxLineBytes) return std::nullopt;
  return frame.str();
}

std::vector<std::string> LineBuffer::feed(std::string_view chunk) {
  std::vector<std::string> complete;
  for (char ch : chunk) {
    if (ch == '\n') {
      if (!dropping_ && !partial_.empty()) {
        if (partial_.back() == '\r') partial_.pop_back();
        if (!partial_.empty()) complete.push_back(std::move(partial_));
      }
      partial_.clear();
      dropping_ = false;
      continue;
    }
    if (dropping_) continue;
    if (partial_.size() >= kMaxLineBytes) {
      partial_.clear();
      dropping_ = true;
      ++dropped_;
      continue;
    }
    partial_.push_back(ch);
  }
  return complete;
}
void LineBuffer::clear() { partial_.clear(); dropping_ = false; }

}  // namespace cefiro::hardware
