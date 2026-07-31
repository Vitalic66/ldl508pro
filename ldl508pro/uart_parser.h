#pragma once

#include <cstdint>
#include <string>

namespace esphome {
namespace ldl508pro {

enum class ParsedLineType : uint8_t {
  UNKNOWN = 0,
  MEASUREMENT,
  CLI_RESPONSE,
};

struct Measurement {
  float distance_m{0.0f};
  float speed_kmh{0.0f};
  uint32_t timestamp_ms{0};
};

class UARTParser {
 public:
  UARTParser() = default;

  ParsedLineType parse_line(const std::string &line, uint32_t timestamp_ms, Measurement &measurement) const;
  ParsedLineType classify_line(const std::string &line) const;

  static void trim(std::string &value);

 protected:
  bool parse_measurement_(const std::string &line, uint32_t timestamp_ms, Measurement &measurement) const;
};

}  // namespace ldl508pro
}  // namespace esphome
