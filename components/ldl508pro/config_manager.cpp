#include "config_manager.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace esphome {
namespace ldl508pro {

static const std::array<ParameterDefinition, static_cast<size_t>(RadarParameter::COUNT)> PARAMETER_DEFINITIONS{{
    {RadarParameter::CFAR, "cfar", "RfeCfarGet", "RfeCfarSet", ParameterValueType::NUMBER},
    {RadarParameter::MAX_FRAMERATE, "max_framerate", "RfeMaxFramerateGet", "RfeMaxFramerateSet", ParameterValueType::NUMBER},
    {RadarParameter::STATIC_DETECTION, "static_detection", "RfeStaticStatus", nullptr, ParameterValueType::STATIC_STATUS},
    {RadarParameter::SPEED_LIMIT_ENABLED, "speed_limit_enabled", "RfeSpdLimitEnGet", "RfeSpdLimitEnSet", ParameterValueType::BOOLEAN},
    {RadarParameter::SPEED_LIMIT_HIGH, "speed_limit_high", "RfeSpdLimitHiGet", "RfeSpdLimitHiSet", ParameterValueType::NUMBER},
    {RadarParameter::SPEED_LIMIT_LOW, "speed_limit_low", "RfeSpdLimitLoGet", "RfeSpdLimitLoSet", ParameterValueType::NUMBER},
    {RadarParameter::POWER_MODE, "power_mode", "RfePwrModeGet", "RfePwrModeSet", ParameterValueType::INTEGER},
    {RadarParameter::DOPPLER_FILTER, "doppler_filter", "RfeTargetDopFilterGet", "RfeTargetDopFilterSet", ParameterValueType::INTEGER},
    {RadarParameter::DISTANCE_LIMIT_HIGH, "distance_limit_high", "RfeDistanceLimitHiGet", "RfeDistanceLimitHiSet", ParameterValueType::NUMBER},
    {RadarParameter::DISTANCE_LIMIT_LOW, "distance_limit_low", "RfeDistanceLimitLoGet", "RfeDistanceLimitLoSet", ParameterValueType::NUMBER},
    {RadarParameter::SPEED_THRESHOLD, "speed_threshold", "RfeSpeedThresholdGet", "RfeSpeedThresholdSet", ParameterValueType::INTEGER},
    {RadarParameter::DURATION, "duration", "RfeDurationGet", "RfeDurationSet", ParameterValueType::INTEGER},
    {RadarParameter::SNR_FILTER, "snr_filter", "RfeSnrFiltrationGet", "RfeSnrFiltrationSet", ParameterValueType::NUMBER},
}};

const std::array<ParameterDefinition, static_cast<size_t>(RadarParameter::COUNT)> &ConfigManager::definitions() {
  return PARAMETER_DEFINITIONS;
}

const ParameterDefinition &ConfigManager::definition(RadarParameter parameter) {
  return PARAMETER_DEFINITIONS.at(static_cast<size_t>(parameter));
}

const ParameterValue &ConfigManager::value(RadarParameter parameter) const {
  return this->values_.at(static_cast<size_t>(parameter));
}

void ConfigManager::queue_read_all() {
  this->queue_.clear();
  this->active_ = false;
  this->echo_seen_ = false;
  this->sync_cycle_active_ = true;
  this->sync_commands_remaining_ = 0;
  this->set_synchronized_(false);

  for (const auto &parameter : PARAMETER_DEFINITIONS) {
    // RfeDurationGet is not implemented by all LDL508PRO firmware variants.
    // Keep the optional entity/setter available, but do not let this command
    // abort the complete boot synchronization cycle.
    if (parameter.id == RadarParameter::DURATION) continue;
    this->queue_.push_back({parameter.id, parameter.get_command, true, this->max_retries_});
    this->sync_commands_remaining_++;
  }
}

void ConfigManager::queue_write(RadarParameter parameter, float value) {
  const auto &definition = ConfigManager::definition(parameter);
  std::string set_command;
  if (parameter == RadarParameter::STATIC_DETECTION) {
    set_command = value != 0.0f ? "RfeStaticEnable" : "RfeStaticDisable";
  } else {
    if (definition.set_command == nullptr) {
      return;
    }
    set_command = std::string(definition.set_command) + ":{" + format_set_value_(parameter, value) + "}";
  }

  this->add_sync_commands_(2);
  this->queue_.push_back({parameter, set_command, false, 0});
  this->queue_.push_back({parameter, definition.get_command, true, this->max_retries_});
}

void ConfigManager::queue_raw(const std::string &command) {
  this->queue_.push_back({RadarParameter::CFAR, command, false, 0});
}

void ConfigManager::add_sync_commands_(size_t count) {
  this->set_synchronized_(false);
  this->sync_cycle_active_ = true;
  this->sync_commands_remaining_ += count;
}

void ConfigManager::loop(uint32_t now_ms) {
  if (this->active_) {
    if (!this->active_command_.expects_response) {
      if (static_cast<uint32_t>(now_ms - this->sent_at_ms_) >= this->command_gap_ms_) {
        this->finish_active_(now_ms);
      }
      return;
    }

    if (static_cast<uint32_t>(now_ms - this->sent_at_ms_) >= this->command_timeout_ms_) {
      if (this->active_command_.retries_left > 0) {
        this->active_command_.retries_left--;
        this->send_active_(now_ms);
      } else {
        this->fail_active_("Timeout bei " + this->active_command_.tx, now_ms);
      }
    }
    return;
  }

  if (!this->queue_.empty() &&
      static_cast<uint32_t>(now_ms - this->last_finished_ms_) >= this->command_gap_ms_) {
    this->start_next_(now_ms);
  }
}

void ConfigManager::start_next_(uint32_t now_ms) {
  this->active_command_ = this->queue_.front();
  this->queue_.pop_front();
  this->active_ = true;
  this->echo_seen_ = false;
  this->send_active_(now_ms);
}

void ConfigManager::send_active_(uint32_t now_ms) {
  this->echo_seen_ = false;
  this->sent_at_ms_ = now_ms;
  if (this->listener_ != nullptr) {
    this->listener_->config_send_command(this->active_command_.tx);
  }
}

bool ConfigManager::process_line(const std::string &line, uint32_t now_ms) {
  if (!this->active_) {
    return false;
  }

  const std::string lower = lowercase_(line);
  if (lower.find("bad command") != std::string::npos || lower.find("parameter error") != std::string::npos) {
    this->fail_active_(line, now_ms);
    return true;
  }

  if (!this->active_command_.expects_response) {
    return this->is_cli_echo_(line);
  }

  float parsed_value = 0.0f;
  if (this->parse_named_response_(line, parsed_value)) {
    this->finish_active_(parsed_value, now_ms);
    return true;
  }

  if (this->is_cli_echo_(line)) {
    this->echo_seen_ = true;
    return true;
  }

  if (this->echo_seen_ && this->parse_legacy_response_(line, parsed_value)) {
    this->finish_active_(parsed_value, now_ms);
    return true;
  }

  return false;
}

bool ConfigManager::is_cli_echo_(const std::string &line) const {
  return line.find("CLI:") != std::string::npos && line.find(this->active_command_.tx) != std::string::npos;
}

bool ConfigManager::parse_named_response_(const std::string &line, float &value) const {
  const auto parameter = this->active_command_.parameter;
  const std::string marker = std::string(ConfigManager::definition(parameter).get_command) + ":{";
  const size_t marker_pos = line.find(marker);
  if (marker_pos != std::string::npos) {
    const size_t payload_begin = marker_pos + marker.size();
    const size_t payload_end = line.find('}', payload_begin);
    if (payload_end == std::string::npos || payload_end <= payload_begin) {
      return false;
    }
    return this->parse_response_payload_(line.substr(payload_begin, payload_end - payload_begin), value);
  }

  if (parameter == RadarParameter::STATIC_DETECTION && line.find("RfeStatic:") != std::string::npos) {
    return this->parse_response_payload_(line.substr(line.find(':') + 1), value);
  }

  return false;
}

bool ConfigManager::parse_response_payload_(const std::string &payload, float &value) const {
  const auto type = definition(this->active_command_.parameter).value_type;
  const std::string lower = lowercase_(payload);

  if (type == ParameterValueType::STATIC_STATUS || type == ParameterValueType::BOOLEAN) {
    if (lower.find("enable") != std::string::npos || lower.find("true") != std::string::npos) {
      value = 1.0f;
      return true;
    }
    if (lower.find("disable") != std::string::npos || lower.find("false") != std::string::npos) {
      value = 0.0f;
      return true;
    }
  }

  if (!parse_first_number_(payload, value)) {
    return false;
  }

  if (type == ParameterValueType::BOOLEAN || type == ParameterValueType::STATIC_STATUS) {
    value = std::lround(value) == 0 ? 0.0f : 1.0f;
  } else if (type == ParameterValueType::INTEGER) {
    value = static_cast<float>(std::lround(value));
  }
  return true;
}

bool ConfigManager::parse_legacy_response_(const std::string &line, float &value) const {
  const std::string lower = lowercase_(line);
  if (lower.find("framerate:") != std::string::npos || lower.find("pointnum:") != std::string::npos ||
      lower.find("cli:") != std::string::npos) {
    return false;
  }
  return this->parse_response_payload_(line, value);
}

bool ConfigManager::parse_first_number_(const std::string &line, float &value) {
  const char *cursor = line.c_str();
  while (*cursor != '\0') {
    const bool possible_start = (*cursor >= '0' && *cursor <= '9') || *cursor == '-' || *cursor == '+' || *cursor == '.';
    if (!possible_start) {
      cursor++;
      continue;
    }

    errno = 0;
    char *end = nullptr;
    const float candidate = std::strtof(cursor, &end);
    if (end != cursor && errno != ERANGE && std::isfinite(candidate)) {
      value = candidate;
      return true;
    }
    cursor++;
  }
  return false;
}

void ConfigManager::finish_active_(uint32_t now_ms) {
  this->active_ = false;
  this->echo_seen_ = false;
  this->last_finished_ms_ = now_ms;

  if (this->sync_cycle_active_ && this->sync_commands_remaining_ > 0) {
    this->sync_commands_remaining_--;
    if (this->sync_commands_remaining_ == 0) {
      this->sync_cycle_active_ = false;
      this->set_synchronized_(true);
    }
  }
}

void ConfigManager::finish_active_(float value, uint32_t now_ms) {
  const RadarParameter parameter = this->active_command_.parameter;
  auto &stored = this->values_.at(static_cast<size_t>(parameter));
  stored.valid = true;
  stored.numeric = value;

  if (this->listener_ != nullptr) {
    this->listener_->config_value_received(parameter, value);
  }
  this->finish_active_(now_ms);
}

void ConfigManager::fail_active_(const std::string &reason, uint32_t now_ms) {
  if (this->listener_ != nullptr) {
    this->listener_->config_error(reason);
  }
  this->active_ = false;
  this->echo_seen_ = false;
  this->last_finished_ms_ = now_ms;
  this->queue_.clear();
  this->sync_cycle_active_ = false;
  this->sync_commands_remaining_ = 0;
  this->set_synchronized_(false);
}

void ConfigManager::set_synchronized_(bool state) {
  if (this->synchronized_ == state) {
    return;
  }
  this->synchronized_ = state;
  if (this->listener_ != nullptr) {
    this->listener_->config_sync_changed(state);
  }
}

std::string ConfigManager::configuration_json() const {
  std::ostringstream out;
  out << "{";
  bool first = true;
  for (const auto &definition : PARAMETER_DEFINITIONS) {
    const auto &stored = this->values_.at(static_cast<size_t>(definition.id));
    if (!stored.valid) {
      continue;
    }
    if (!first) {
      out << ",";
    }
    first = false;
    out << "\"" << definition.name << "\":";
    if (definition.value_type == ParameterValueType::BOOLEAN ||
        definition.value_type == ParameterValueType::STATIC_STATUS) {
      out << (stored.numeric != 0.0f ? "true" : "false");
    } else {
      out << stored.numeric;
    }
  }
  out << "}";
  return out.str();
}

std::string ConfigManager::lowercase_(const std::string &value) {
  std::string result(value);
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
    if (c >= 'A' && c <= 'Z') {
      return static_cast<char>(c - 'A' + 'a');
    }
    return static_cast<char>(c);
  });
  return result;
}

std::string ConfigManager::format_set_value_(RadarParameter parameter, float value) {
  const auto type = ConfigManager::definition(parameter).value_type;
  std::ostringstream out;
  if (type == ParameterValueType::INTEGER || type == ParameterValueType::BOOLEAN ||
      type == ParameterValueType::STATIC_STATUS) {
    out << static_cast<int>(std::lround(value));
  } else {
    out << std::fixed << std::setprecision(2) << value;
    std::string result = out.str();
    while (!result.empty() && result.back() == '0') result.pop_back();
    if (!result.empty() && result.back() == '.') result.pop_back();
    return result;
  }
  return out.str();
}

}  // namespace ldl508pro
}  // namespace esphome
