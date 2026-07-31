#include "uart_parser.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>

namespace esphome {
namespace ldl508pro {

ParsedLineType UARTParser::parse_line(const std::string &line, uint32_t timestamp_ms,
                                      Measurement &measurement) const {
  if (this->parse_measurement_(line, timestamp_ms, measurement)) {
    return ParsedLineType::MEASUREMENT;
  }

  if (!line.empty()) {
    return ParsedLineType::CLI_RESPONSE;
  }

  return ParsedLineType::UNKNOWN;
}

ParsedLineType UARTParser::classify_line(const std::string &line) const {
  Measurement ignored;
  return this->parse_line(line, 0, ignored);
}

bool UARTParser::parse_measurement_(const std::string &line, uint32_t timestamp_ms,
                                    Measurement &measurement) const {
  const char *cursor = line.c_str();

  while (*cursor == ' ' || *cursor == '\t') {
    cursor++;
  }

  if (*cursor != 'R') {
    return false;
  }
  cursor++;

  if (*cursor != ' ' && *cursor != '\t') {
    return false;
  }

  while (*cursor == ' ' || *cursor == '\t') {
    cursor++;
  }

  errno = 0;
  char *end = nullptr;
  const float distance = std::strtof(cursor, &end);
  if (end == cursor || errno == ERANGE || !std::isfinite(distance)) {
    return false;
  }
  cursor = end;

  if (*cursor != ' ' && *cursor != '\t') {
    return false;
  }
  while (*cursor == ' ' || *cursor == '\t') {
    cursor++;
  }

  errno = 0;
  const float speed = std::strtof(cursor, &end);
  if (end == cursor || errno == ERANGE || !std::isfinite(speed)) {
    return false;
  }
  cursor = end;

  while (*cursor == ' ' || *cursor == '\t') {
    cursor++;
  }
  if (*cursor != '\0') {
    return false;
  }

  // The radar's valid target lines contain a positive distance. A zero or
  // negative distance is treated as a non-measurement/diagnostic line.
  if (distance <= 0.0f || distance > 1000.0f || std::fabs(speed) > 1000.0f) {
    return false;
  }

  measurement.distance_m = distance;
  measurement.speed_kmh = speed;
  measurement.timestamp_ms = timestamp_ms;
  return true;
}

void UARTParser::trim(std::string &value) {
  const std::string::size_type first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    value.clear();
    return;
  }

  const std::string::size_type last = value.find_last_not_of(" \t\r\n");
  value = value.substr(first, last - first + 1);
}

}  // namespace ldl508pro
}  // namespace esphome
