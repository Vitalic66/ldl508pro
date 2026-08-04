#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <string>

namespace esphome {
namespace ldl508pro {

enum class RadarParameter : uint8_t {
  CFAR,
  MAX_FRAMERATE,
  STATIC_DETECTION,
  SPEED_LIMIT_ENABLED,
  SPEED_LIMIT_HIGH,
  SPEED_LIMIT_LOW,
  POWER_MODE,
  DOPPLER_FILTER,
  DISTANCE_LIMIT_HIGH,
  DISTANCE_LIMIT_LOW,
  SPEED_THRESHOLD,
  DURATION,
  SNR_FILTER,
  COUNT,
};

// ESPHome code generation may emit enum values in the component namespace.
// These constexpr aliases preserve strong enum typing while keeping generated
// code compatible across ESPHome versions.
inline constexpr RadarParameter CFAR = RadarParameter::CFAR;
inline constexpr RadarParameter MAX_FRAMERATE = RadarParameter::MAX_FRAMERATE;
inline constexpr RadarParameter STATIC_DETECTION = RadarParameter::STATIC_DETECTION;
inline constexpr RadarParameter SPEED_LIMIT_ENABLED = RadarParameter::SPEED_LIMIT_ENABLED;
inline constexpr RadarParameter SPEED_LIMIT_HIGH = RadarParameter::SPEED_LIMIT_HIGH;
inline constexpr RadarParameter SPEED_LIMIT_LOW = RadarParameter::SPEED_LIMIT_LOW;
inline constexpr RadarParameter POWER_MODE = RadarParameter::POWER_MODE;
inline constexpr RadarParameter DOPPLER_FILTER = RadarParameter::DOPPLER_FILTER;
inline constexpr RadarParameter DISTANCE_LIMIT_HIGH = RadarParameter::DISTANCE_LIMIT_HIGH;
inline constexpr RadarParameter DISTANCE_LIMIT_LOW = RadarParameter::DISTANCE_LIMIT_LOW;
inline constexpr RadarParameter SPEED_THRESHOLD = RadarParameter::SPEED_THRESHOLD;
inline constexpr RadarParameter DURATION = RadarParameter::DURATION;
inline constexpr RadarParameter SNR_FILTER = RadarParameter::SNR_FILTER;

enum class ParameterValueType : uint8_t {
  NUMBER,
  INTEGER,
  BOOLEAN,
  STATIC_STATUS,
};

struct ParameterDefinition {
  RadarParameter id;
  const char *name;
  const char *get_command;
  const char *set_command;
  ParameterValueType value_type;
};

struct ParameterValue {
  bool valid{false};
  float numeric{0.0f};
};

class ConfigManagerListener {
 public:
  virtual ~ConfigManagerListener() = default;
  virtual void config_send_command(const std::string &command) = 0;
  virtual void config_value_received(RadarParameter parameter, float value) = 0;
  virtual void config_sync_changed(bool synchronized) = 0;
  virtual void config_error(const std::string &message) = 0;
};

class ConfigManager {
 public:
  explicit ConfigManager(ConfigManagerListener *listener) : listener_(listener) {}

  void set_command_gap(uint32_t value) { this->command_gap_ms_ = value; }
  void set_command_timeout(uint32_t value) { this->command_timeout_ms_ = value; }
  void set_retries(uint8_t value) { this->max_retries_ = value; }

  void loop(uint32_t now_ms);
  bool process_line(const std::string &line, uint32_t now_ms);

  void queue_read_all();
  void queue_write(RadarParameter parameter, float value);
  void queue_raw(const std::string &command);

  bool is_busy() const { return this->active_ || !this->queue_.empty(); }
  bool is_synchronized() const { return this->synchronized_; }

  const ParameterValue &value(RadarParameter parameter) const;
  std::string configuration_json() const;

  static const std::array<ParameterDefinition, static_cast<size_t>(RadarParameter::COUNT)> &definitions();
  static const ParameterDefinition &definition(RadarParameter parameter);

 protected:
  struct PendingCommand {
    RadarParameter parameter{RadarParameter::CFAR};
    std::string tx;
    bool expects_response{true};
    uint8_t retries_left{0};
  };

  void start_next_(uint32_t now_ms);
  void send_active_(uint32_t now_ms);
  void finish_active_(uint32_t now_ms);
  void finish_active_(float value, uint32_t now_ms);
  void fail_active_(const std::string &reason, uint32_t now_ms);
  void set_synchronized_(bool state);
  void add_sync_commands_(size_t count);

  bool is_cli_echo_(const std::string &line) const;
  bool parse_named_response_(const std::string &line, float &value) const;
  bool parse_response_payload_(const std::string &payload, float &value) const;
  bool parse_legacy_response_(const std::string &line, float &value) const;

  static bool parse_first_number_(const std::string &line, float &value);
  static std::string lowercase_(const std::string &value);
  static std::string format_set_value_(RadarParameter parameter, float value);

  ConfigManagerListener *listener_{nullptr};
  std::array<ParameterValue, static_cast<size_t>(RadarParameter::COUNT)> values_{};
  std::deque<PendingCommand> queue_{};
  PendingCommand active_command_{};

  bool active_{false};
  bool echo_seen_{false};
  bool synchronized_{false};
  bool sync_cycle_active_{false};
  size_t sync_commands_remaining_{0};
  uint32_t sent_at_ms_{0};
  uint32_t last_finished_ms_{0};
  uint32_t command_gap_ms_{200};
  uint32_t command_timeout_ms_{1200};
  uint8_t max_retries_{1};
};

}  // namespace ldl508pro
}  // namespace esphome
