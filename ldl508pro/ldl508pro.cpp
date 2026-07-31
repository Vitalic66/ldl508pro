#include "ldl508pro.h"

#include <inttypes.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ldl508pro {

static const char *const TAG = "ldl508pro";

std::string LDL508PROComponent::escape_raw_bytes_(const std::string &value) {
  std::string out;
  out.reserve(value.size() * 2);
  char escaped[5];
  for (const unsigned char byte : value) {
    if (byte >= 0x20 && byte <= 0x7E && byte != '\\' && byte != '"') {
      out.push_back(static_cast<char>(byte));
    } else if (byte == '\\') {
      out += "\\\\";
    } else {
      std::snprintf(escaped, sizeof(escaped), "\\x%02X", byte);
      out += escaped;
    }
  }
  return out;
}

bool LDL508PROComponent::is_safe_text_line_(const std::string &value) {
  for (const unsigned char byte : value) {
    if (byte == '\t') continue;
    if (byte < 0x20 || byte > 0x7E) return false;
  }
  return true;
}

void LDLNumber::control(float value) { this->parent_->set_numeric_parameter(this->parameter_, value); }
void LDLLEDNumber::control(float value) { this->parent_->set_led_setting(this->setting_, value); }
void LDLSwitch::write_state(bool state) { this->parent_->set_switch_parameter(this->parameter_, state); }
void LDLSelect::control(const std::string &value) { this->parent_->set_select_parameter(this->parameter_, value); }
void LDLButton::press_action() {
  if (this->action_ == LDLButtonAction::FACTORY_RESET) {
    this->parent_->factory_reset();
  } else if (this->action_ == LDLButtonAction::REQUEST_MULTI_TARGET_SNAPSHOT) {
    this->parent_->request_multi_target_snapshot();
  } else if (this->action_ == LDLButtonAction::TARGET_MODE_0) {
    this->parent_->request_target_mode(0);
  } else if (this->action_ == LDLButtonAction::TARGET_MODE_1) {
    this->parent_->request_target_mode(1);
  } else if (this->action_ == LDLButtonAction::START_RAW_CAPTURE) {
    this->parent_->start_raw_capture();
  } else {
    this->parent_->refresh_configuration();
  }
}

void LDL508PROComponent::setup() {
  this->setup_ms_ = millis();
  this->rx_buffer_.reserve(96);
  if (this->red_output_pin_ != nullptr) {
    this->red_output_pin_->setup();
    this->red_output_pin_->digital_write(false);
  }
  if (this->green_output_pin_ != nullptr) {
    this->green_output_pin_->setup();
    this->green_output_pin_->digital_write(false);
  }
  this->led_idle_started_ms_ = this->setup_ms_;
  this->publish_detection_(false);
  if (this->config_synchronized_sensor_ != nullptr) {
    this->config_synchronized_sensor_->publish_state(false);
  }
  this->publish_error_("Kein Fehler");
  if (this->vehicle_tracking_sensor_ != nullptr) this->vehicle_tracking_sensor_->publish_state(false);
  if (this->vehicle_direction_sensor_ != nullptr) this->vehicle_direction_sensor_->publish_state("Unbekannt");
  if (this->vehicle_count_sensor_ != nullptr) this->vehicle_count_sensor_->publish_state(0);
  if (this->target_count_sensor_ != nullptr) {
    this->target_count_sensor_->publish_state(0);
    this->last_published_target_count_ = 0;
  }
  if (this->max_simultaneous_targets_sensor_ != nullptr) {
    this->max_simultaneous_targets_sensor_->publish_state(0);
    this->last_published_max_targets_ = 0;
  }
  if (this->multi_target_active_sensor_ != nullptr) {
    this->multi_target_active_sensor_->publish_state(false);
    this->last_published_multi_active_ = false;
    this->multi_active_has_state_ = true;
  }
  if (this->multi_target_status_sensor_ != nullptr) {
    this->last_multi_target_status_ = this->multi_target_polling_ ? "Polling aktiv – experimentell" : "Polling deaktiviert";
    this->multi_target_status_sensor_->publish_state(this->last_multi_target_status_);
  }
  if (this->target_mode_status_sensor_ != nullptr) this->target_mode_status_sensor_->publish_state("Unverändert / unbekannt");
  if (this->raw_capture_status_sensor_ != nullptr) this->raw_capture_status_sensor_->publish_state("Bereit");
  auto red_hold = this->led_numbers_.find(LEDSetting::RED_AFTERGLOW);
  if (red_hold != this->led_numbers_.end()) red_hold->second->publish_state(this->led_red_afterglow_ms_ / 1000.0f);
  auto standby = this->led_numbers_.find(LEDSetting::STANDBY_TIMEOUT);
  if (standby != this->led_numbers_.end()) standby->second->publish_state(this->led_standby_timeout_ms_ / 1000.0f);
  ESP_LOGCONFIG(TAG, "Initializing LDL508PRO stable single-target build with LED afterglow, standby and fault blink");
}

void LDL508PROComponent::loop() {
  while (this->available() > 0) {
    uint8_t byte;
    if (!this->read_byte(&byte)) {
      break;
    }
    this->process_byte_(byte);
  }

  const uint32_t now = millis();
  if (!this->boot_mode_normalized_ && static_cast<uint32_t>(now - this->setup_ms_) >= 250) {
    this->boot_mode_normalized_ = true;
    ESP_LOGI(TAG, "Normalizing radar to target mode 1 before configuration sync");
    this->send_target_mode_command_(1);
    this->multi_target_stream_mode_ = false;
  }

  if (!this->boot_read_started_ && this->boot_mode_normalized_ &&
      static_cast<uint32_t>(now - this->setup_ms_) >= this->boot_read_delay_ms_) {
    this->boot_read_started_ = true;
    ESP_LOGI(TAG, "Queueing complete radar configuration read");
    this->config_manager_.queue_read_all();
  }

  this->config_manager_.loop(now);

  // Phase 7.1 starts the selected diagnostic mode only after the normal ASCII
  // configuration synchronization has completed. This leaves the stable boot
  // path unchanged and makes "off" identical to stable 1.0.
  if (!this->multitarget_debug_started_ && this->boot_read_started_ &&
      this->config_manager_.is_synchronized() && !this->config_manager_.is_busy() &&
      this->multitarget_debug_mode_ != "off") {
    this->multitarget_debug_started_ = true;
    this->start_multitarget_debug_();
  }

  // Phase 7.1.3: flush the exact RX byte stream after a short UART idle gap.
  // This recorder runs independently from all ASCII/HEX frame parsers.
  if (this->multitarget_debug_started_ && this->multitarget_byte_block_length_ > 0 &&
      static_cast<uint32_t>(now - this->multitarget_byte_last_byte_ms_) >= 60) {
    this->flush_multitarget_byte_block_();
  }

  if (this->multitarget_debug_mode_ == "hex" && this->multitarget_hex_mqtt_block_length_ > 0 &&
      static_cast<uint32_t>(now - this->multitarget_hex_last_byte_ms_) >= 100) {
    std::string payload;
    char part[4];
    for (uint8_t i = 0; i < this->multitarget_hex_mqtt_block_length_; i++) {
      std::snprintf(part, sizeof(part), "%02X", this->multitarget_hex_mqtt_block_[i]);
      if (!payload.empty()) payload += " ";
      payload += part;
    }
    ESP_LOGI(TAG, "RAW7.1-HEX: %s", payload.c_str());
    this->publish_multitarget_raw_mqtt_(payload, true);
    this->multitarget_hex_mqtt_block_length_ = 0;
  }

  // The legacy mode-1 timeout must never clear a detection owned by the
  // continuous HEADA state machine. Exactly one parser owns the entity.
  if (!this->multi_target_stream_mode_ && this->target_detected_ &&
      static_cast<uint32_t>(now - this->last_target_ms_) >= this->target_timeout_ms_) {
    this->publish_detection_(false);
    this->finish_vehicle_event_(now);
  }

  if (!this->multi_target_candidate_confirmed_ && this->multi_target_candidate_frames_ > 0 &&
      static_cast<uint32_t>(now - this->multi_target_candidate_first_ms_) > this->multi_target_confirmation_window_ms_) {
    if (this->debug_uart_) {
      ESP_LOGD(TAG, "Discarding unconfirmed HEADA candidate after %u frame(s)",
               static_cast<unsigned>(this->multi_target_candidate_frames_));
    }
    this->multi_target_candidate_frames_ = 0;
    this->multi_target_candidate_first_ms_ = 0;
    this->multi_target_candidate_last_ms_ = 0;
    this->multi_target_candidate_first_distance_ = 0.0f;
  }

  // Phase 7.3 tracks expire independently. A short HEADA0 gap therefore does
  // not erase all vehicles at once, while a stopped UART stream still times out.
  if (this->multi_target_stream_mode_) {
    const uint8_t before = this->multi_target_tracker_.active_count();
    this->multi_target_tracker_.expire(now);
    if (before != this->multi_target_tracker_.active_count()) {
      this->publish_tracked_snapshot_(0, now);
      this->publish_detection_(this->multi_target_tracker_.confirmed_count() > 0);
    }
  }

  // RfeTargetListGet is the documented ASCII snapshot command for up to nine
  // simultaneous targets. Poll only after boot synchronization and while the
  // configuration queue is idle so normal configuration traffic stays intact.
  if (!this->multi_target_stream_mode_ && this->multi_target_polling_ && this->boot_read_started_ && this->config_manager_.is_synchronized() &&
      !this->config_manager_.is_busy() && this->multi_target_poll_interval_ms_ > 0 &&
      static_cast<uint32_t>(now - this->last_multi_target_poll_ms_) >= this->multi_target_poll_interval_ms_) {
    this->request_target_list_(now);
  }

  if (this->raw_capture_active_ && static_cast<uint32_t>(now - this->raw_capture_started_ms_) >= this->raw_capture_duration_ms_) {
    this->finish_raw_capture_(now);
  }

  // LED timers and fault blinking continue even when no new radar frame arrives.
  this->update_status_outputs_();

  // Some firmware revisions omit PointNum or the final HEADA summary. Close a
  // partially received snapshot after a short quiet period.
  if (this->target_list_receiving_ && this->target_list_last_row_ms_ != 0 &&
      static_cast<uint32_t>(now - this->target_list_last_row_ms_) >= 180) {
    this->publish_target_snapshot_(now);
  }
}

void LDL508PROComponent::process_byte_(uint8_t byte) {
  const uint32_t now_ms = millis();
  if (this->multitarget_debug_started_) this->capture_multitarget_byte_(byte, now_ms);
  if (this->multitarget_debug_mode_ == "hex" && this->multitarget_debug_started_) {
    this->multitarget_hex_mqtt_block_[this->multitarget_hex_mqtt_block_length_++] = byte;
    this->multitarget_hex_last_byte_ms_ = now_ms;
    if (this->multitarget_hex_mqtt_block_length_ >= this->multitarget_hex_mqtt_block_.size()) {
      std::string payload;
      char part[4];
      for (uint8_t i = 0; i < this->multitarget_hex_mqtt_block_length_; i++) {
        std::snprintf(part, sizeof(part), "%02X", this->multitarget_hex_mqtt_block_[i]);
        if (!payload.empty()) payload += " ";
        payload += part;
      }
      ESP_LOGI(TAG, "RAW7.1-HEX: %s", payload.c_str());
      this->publish_multitarget_raw_mqtt_(payload, true);
      this->multitarget_hex_mqtt_block_length_ = 0;
    }
    return;
  }
  if (this->raw_capture_active_ && this->raw_capture_hex_mode_) {
    this->capture_hex_byte_(byte, now_ms);
    return;
  }

  this->ingress_buffer_.push_back(static_cast<char>(byte));
  if (this->ingress_buffer_.size() > MAX_INGRESS_LENGTH) {
    ESP_LOGW(TAG, "UART ingress buffer exceeded %u bytes; resynchronizing", static_cast<unsigned>(MAX_INGRESS_LENGTH));
    this->ingress_buffer_.erase(0, this->ingress_buffer_.size() - 5);
    this->ingress_resyncs_++;
  }
  this->process_ingress_buffer_(now_ms);
}

void LDL508PROComponent::process_ingress_buffer_(uint32_t now_ms) {
  static const std::string header = "HEADA";

  while (!this->ingress_buffer_.empty()) {
    const size_t pos = this->ingress_buffer_.find(header);
    if (pos == 0) {
      if (this->ingress_buffer_.size() < 6) return;
      const char count_char = this->ingress_buffer_[5];
      if (count_char < '0' || count_char > '9') {
        this->process_text_byte_(static_cast<uint8_t>(this->ingress_buffer_.front()));
        this->ingress_buffer_.erase(0, 1);
        this->ingress_resyncs_++;
        continue;
      }
      const uint8_t count = static_cast<uint8_t>(count_char - '0');
      const size_t frame_length = 7U + static_cast<size_t>(count) * 4U;
      if (this->ingress_buffer_.size() < frame_length) return;
      const std::string frame = this->ingress_buffer_.substr(0, frame_length);
      this->ingress_buffer_.erase(0, frame_length);
      this->ingress_heada_frames_++;
      this->process_multi_target_frame_(frame, now_ms);
      continue;
    }

    if (pos != std::string::npos) {
      for (size_t i = 0; i < pos; i++)
        this->process_text_byte_(static_cast<uint8_t>(this->ingress_buffer_[i]));
      this->ingress_text_bytes_ += pos;
      this->ingress_buffer_.erase(0, pos);
      continue;
    }

    // No complete header is present. Preserve the longest suffix that could be
    // the beginning of HEADA and deliver everything else to the text parser.
    size_t keep = 0;
    const size_t max_prefix = std::min(header.size() - 1, this->ingress_buffer_.size());
    for (size_t n = max_prefix; n > 0; n--) {
      if (this->ingress_buffer_.compare(this->ingress_buffer_.size() - n, n, header, 0, n) == 0) {
        keep = n;
        break;
      }
    }
    const size_t flush = this->ingress_buffer_.size() - keep;
    for (size_t i = 0; i < flush; i++)
      this->process_text_byte_(static_cast<uint8_t>(this->ingress_buffer_[i]));
    this->ingress_text_bytes_ += flush;
    this->ingress_buffer_.erase(0, flush);
    return;
  }
}

void LDL508PROComponent::process_text_byte_(uint8_t byte) {
  if (byte == '\n') {
    if (this->discard_until_newline_) {
      this->discard_until_newline_ = false;
      this->rx_buffer_.clear();
      return;
    }
    std::string line;
    line.swap(this->rx_buffer_);
    this->process_line_(std::move(line));
    return;
  }

  if (byte == '\r') return;
  if (this->discard_until_newline_) return;
  if (this->rx_buffer_.size() >= MAX_LINE_LENGTH) {
    ESP_LOGW(TAG, "UART text line exceeded %u bytes; discarding until newline", static_cast<unsigned>(MAX_LINE_LENGTH));
    this->rx_buffer_.clear();
    this->discard_until_newline_ = true;
    return;
  }
  this->rx_buffer_.push_back(static_cast<char>(byte));
}


uint8_t LDL508PROComponent::xor_checksum_(const std::string &data) {
  uint8_t checksum = 0;
  for (const unsigned char value : data) checksum ^= value;
  return checksum;
}

void LDL508PROComponent::process_multi_target_byte_(uint8_t byte, uint32_t now_ms) {
  this->multi_target_stream_buffer_.push_back(static_cast<char>(byte));

  // Synchronize on the fixed header. Command echoes may precede the stream.
  while (true) {
    const size_t header = this->multi_target_stream_buffer_.find("HEADA");
    if (header == std::string::npos) {
      // Keep only a possible partial header suffix.
      if (this->multi_target_stream_buffer_.size() > 4)
        this->multi_target_stream_buffer_.erase(0, this->multi_target_stream_buffer_.size() - 4);
      return;
    }
    if (header > 0) this->multi_target_stream_buffer_.erase(0, header);
    if (this->multi_target_stream_buffer_.size() < 6) return;

    const char count_char = this->multi_target_stream_buffer_[5];
    if (count_char < '0' || count_char > '9') {
      this->multi_target_stream_buffer_.erase(0, 1);
      continue;
    }
    const uint8_t count = static_cast<uint8_t>(count_char - '0');
    const size_t frame_length = 7U + static_cast<size_t>(count) * 4U;
    if (this->multi_target_stream_buffer_.size() < frame_length) return;

    const std::string frame = this->multi_target_stream_buffer_.substr(0, frame_length);
    this->multi_target_stream_buffer_.erase(0, frame_length);
    this->process_multi_target_frame_(frame, now_ms);
  }
}

void LDL508PROComponent::process_multi_target_frame_(const std::string &frame, uint32_t now_ms) {
  const std::string escaped_frame = escape_raw_bytes_(frame);
  if (this->multitarget_debug_mode_ == "ascii") {
    ESP_LOGI(TAG, "RAW7.3-ASCII: %s", escaped_frame.c_str());
    this->publish_multitarget_raw_mqtt_(frame, false);
  }

  if (frame.size() < 7 || frame.rfind("HEADA", 0) != 0) return;
  const char count_char = frame[5];
  if (count_char < '0' || count_char > '9') return;

  const uint8_t count = static_cast<uint8_t>(count_char - '0');
  const size_t expected_length = 7U + static_cast<size_t>(count) * 4U;
  if (frame.size() != expected_length) {
    ESP_LOGW(TAG, "Invalid HEADA length: count=%u expected=%u received=%u frame='%s'",
             static_cast<unsigned>(count), static_cast<unsigned>(expected_length),
             static_cast<unsigned>(frame.size()), escaped_frame.c_str());
    return;
  }

  const uint8_t expected_checksum = xor_checksum_(frame.substr(0, frame.size() - 1));
  const uint8_t received_checksum = static_cast<uint8_t>(frame.back());
  if (expected_checksum != received_checksum) {
    this->multi_target_checksum_errors_++;
    ESP_LOGW(TAG, "Multi-target checksum error: frame='%s' expected 0x%02X received 0x%02X",
             escaped_frame.c_str(), expected_checksum, received_checksum);
    return;
  }

  std::array<float, MultiTargetTracker::MAX_TRACKS> distances{};
  for (uint8_t i = 0; i < count; i++) {
    const size_t offset = 6U + static_cast<size_t>(i) * 4U;
    unsigned value = 0;
    for (size_t digit = 0; digit < 4; digit++) {
      const char ch = frame[offset + digit];
      if (ch < '0' || ch > '9') {
        ESP_LOGW(TAG, "Invalid HEADA distance field %u in frame: %s",
                 static_cast<unsigned>(i), escaped_frame.c_str());
        return;
      }
      value = value * 10U + static_cast<unsigned>(ch - '0');
    }
    distances[i] = static_cast<float>(value);
  }

  this->multi_target_valid_frames_++;
  if (count == 0) this->multi_target_empty_frames_++;
  else this->multi_target_target_frames_++;

  if (!this->multi_target_stream_mode_) {
    this->multi_target_stream_mode_ = true;
    ESP_LOGI(TAG, "Detected active HEADA stream; assigning detection ownership to mode 0");
    if (this->target_mode_status_sensor_ != nullptr)
      this->target_mode_status_sensor_->publish_state("Modus 0 automatisch anhand HEADA erkannt");
  }

  // Phase 7.3: raw slots are associated with persistent tracks. HEADA0 does
  // not immediately delete tracks; each track expires independently.
  this->multi_target_tracker_.update(distances, count, now_ms);
  if (count > 0) {
    this->multi_target_last_seen_ms_ = now_ms;
    this->last_target_ms_ = now_ms;
  }

  this->publish_tracked_snapshot_(count, now_ms);
  this->publish_detection_(this->multi_target_tracker_.confirmed_count() > 0 || count > 0);

  if (this->debug_uart_) {
    ESP_LOGD(TAG, "Multi-target frame #%" PRIu32 ": raw=%u active=%u confirmed=%u checksum OK: %s",
             this->multi_target_valid_frames_, static_cast<unsigned>(count),
             static_cast<unsigned>(this->multi_target_tracker_.active_count()),
             static_cast<unsigned>(this->multi_target_tracker_.confirmed_count()), escaped_frame.c_str());
  }
}

void LDL508PROComponent::publish_tracked_snapshot_(uint8_t raw_count, uint32_t now_ms) {
  const uint8_t active_count = this->multi_target_tracker_.active_count();
  const uint8_t confirmed_count = this->multi_target_tracker_.confirmed_count();
  if (active_count > this->max_simultaneous_targets_) this->max_simultaneous_targets_ = active_count;

  if (this->target_count_sensor_ != nullptr && this->last_published_target_count_ != confirmed_count) {
    this->target_count_sensor_->publish_state(confirmed_count);
    this->last_published_target_count_ = confirmed_count;
  }
  if (this->max_simultaneous_targets_sensor_ != nullptr &&
      this->last_published_max_targets_ != this->max_simultaneous_targets_) {
    this->max_simultaneous_targets_sensor_->publish_state(this->max_simultaneous_targets_);
    this->last_published_max_targets_ = this->max_simultaneous_targets_;
  }
  const bool multiple_active = confirmed_count > 1;
  if (this->multi_target_active_sensor_ != nullptr &&
      (!this->multi_active_has_state_ || this->last_published_multi_active_ != multiple_active)) {
    this->multi_target_active_sensor_->publish_state(multiple_active);
    this->last_published_multi_active_ = multiple_active;
    this->multi_active_has_state_ = true;
  }

  std::string snapshot = "{\"phase\":\"7.3\",\"raw_count\":" + std::to_string(raw_count) +
                         ",\"active_count\":" + std::to_string(active_count) +
                         ",\"confirmed_count\":" + std::to_string(confirmed_count) +
                         ",\"tracks\":[";
  bool first = true;
  char item[220];
  for (const auto &track : this->multi_target_tracker_.tracks()) {
    if (!track.active) continue;
    const uint32_t age_ms = now_ms - track.first_seen_ms;
    std::snprintf(item, sizeof(item),
                  "%s{\"id\":%u,\"distance_m\":%.1f,\"speed_kmh\":%.1f,"
                  "\"direction\":\"%s\",\"confirmed\":%s,\"hits\":%u,"
                  "\"misses\":%u,\"age_ms\":%" PRIu32 "}",
                  first ? "" : ",", static_cast<unsigned>(track.id), track.distance_m,
                  track.speed_kmh, MultiTargetTracker::direction_to_string(track.direction),
                  track.confirmed ? "true" : "false", static_cast<unsigned>(track.hits),
                  static_cast<unsigned>(track.misses), age_ms);
    snapshot += item;
    first = false;
  }
  snapshot += "]}";

  this->publish_multitarget_parsed_mqtt_(snapshot);
  if (this->multi_target_snapshot_sensor_ != nullptr && snapshot != this->last_published_snapshot_) {
    this->multi_target_snapshot_sensor_->publish_state(snapshot);
    this->last_published_snapshot_ = snapshot;
  }
}

bool LDL508PROComponent::is_artifact_measurement_(const Measurement &measurement) {
  if (!this->artifact_filter_enabled_) return false;
  // This check intentionally runs immediately after decoding the Mode-1 frame,
  // before publishing sensors, changing LED state or touching the vehicle tracker.
  if (!this->ghost_filter_.matches(measurement, this->artifact_distance_m_,
                                   this->artifact_distance_tolerance_m_, this->artifact_speed_kmh_,
                                   this->artifact_speed_tolerance_kmh_)) return false;

  this->artifact_filtered_count_++;
  const uint32_t now_ms = millis();
  if (this->last_artifact_log_ms_ == 0 || now_ms - this->last_artifact_log_ms_ >= 5000) {
    ESP_LOGW(TAG, "Filtered known ghost measurement: %.1f m / %.1f km/h (total %" PRIu32 ")",
             measurement.distance_m, measurement.speed_kmh, this->artifact_filtered_count_);
    this->last_artifact_log_ms_ = now_ms;
  }
  return true;
}

void LDL508PROComponent::process_line_(std::string line) {
  while (!line.empty() && (line.back() == ' ' || line.back() == '\t')) line.pop_back();
  size_t first = 0;
  while (first < line.size() && (line[first] == ' ' || line[first] == '\t')) first++;
  if (first > 0) line.erase(0, first);
  if (line.empty()) return;

  const std::string escaped_line = escape_raw_bytes_(line);
  if (this->debug_uart_) ESP_LOGD(TAG, "RX: %s", escaped_line.c_str());
  if (this->multitarget_debug_mode_ == "ascii" && this->multitarget_debug_started_) {
    ESP_LOGI(TAG, "RAW7.1.1-ASCII-LINE: %s", escaped_line.c_str());
    this->publish_multitarget_raw_mqtt_(line, false);
  }
  if (this->raw_capture_active_) {
    this->raw_capture_line_count_++;
    ESP_LOGI(TAG, "RAW6.8-TEXT[%" PRIu32 "]: %s", this->raw_capture_line_count_, escaped_line.c_str());
  }

  // Radar/UART bytes are untrusted. Never forward non-ASCII bytes into an
  // ESPHome text_sensor, because the native API requires valid UTF-8. Raw data
  // has already been preserved above in escaped ASCII form over MQTT.
  if (!is_safe_text_line_(line)) {
    ESP_LOGW(TAG, "Discarding binary-contaminated text line after raw capture: %s", escaped_line.c_str());
    return;
  }

  const uint32_t now_ms = millis();
  if (this->process_target_list_line_(line, now_ms)) return;

  Measurement measurement{};
  if (this->uart_parser_.parse_line(line, now_ms, measurement) == ParsedLineType::MEASUREMENT) {
    if (this->is_artifact_measurement_(measurement)) return;
    if (this->distance_sensor_ != nullptr) this->distance_sensor_->publish_state(measurement.distance_m);
    if (this->speed_sensor_ != nullptr) this->speed_sensor_->publish_state(measurement.speed_kmh);
    const bool was_tracking = this->vehicle_tracker_.tracking();
    this->vehicle_tracker_.add_measurement(measurement);
    if (!was_tracking) {
      ESP_LOGI(TAG, "Vehicle #%" PRIu32 " started at %.1f m", this->vehicle_tracker_.current_id(), measurement.distance_m);
      if (this->vehicle_tracking_sensor_ != nullptr) this->vehicle_tracking_sensor_->publish_state(true);
      if (this->vehicle_id_sensor_ != nullptr) this->vehicle_id_sensor_->publish_state(this->vehicle_tracker_.current_id());
    }
    if (this->vehicle_direction_sensor_ != nullptr) {
      this->vehicle_direction_sensor_->publish_state(vehicle_direction_to_string(this->vehicle_tracker_.current_direction()));
    }
    this->last_target_ms_ = measurement.timestamp_ms;
    this->publish_detection_(true);
    return;
  }

  this->config_manager_.process_line(line, millis());
}


bool LDL508PROComponent::parse_target_row_(const std::string &raw, uint8_t &id, float &distance_m,
                                            float &speed_kmh, float &snr) {
  std::string line = raw;
  if (line.rfind("[INFO]", 0) == 0) {
    line.erase(0, 6);
    while (!line.empty() && line.front() == ' ') line.erase(0, 1);
  }

  // Ignore timestamped CLI and FrameRate/header lines. Target rows start with
  // a two-digit ID followed by distance, speed and SNR. scanf also accepts the
  // documented compact form "029.5-042.1" without an intervening space.
  unsigned parsed_id = 0;
  float parsed_distance = 0.0f;
  float parsed_speed = 0.0f;
  float parsed_snr = 0.0f;
  int consumed = 0;
  if (std::sscanf(line.c_str(), "%2u %f %f %f%n", &parsed_id, &parsed_distance, &parsed_speed,
                  &parsed_snr, &consumed) != 4) {
    return false;
  }
  if (parsed_id > 8 || !std::isfinite(parsed_distance) || !std::isfinite(parsed_speed) ||
      !std::isfinite(parsed_snr)) {
    return false;
  }
  id = static_cast<uint8_t>(parsed_id);
  distance_m = parsed_distance;
  speed_kmh = parsed_speed;
  snr = parsed_snr;
  return true;
}

bool LDL508PROComponent::process_target_list_line_(const std::string &line, uint32_t now_ms) {
  if (line.find("CLI: RfeTargetListGet") != std::string::npos) {
    this->target_list_receiving_ = true;
    this->target_list_started_ms_ = now_ms;
    this->target_list_last_row_ms_ = 0;
    this->target_list_expected_ = 0;
    this->target_list_count_ = 0;
    for (auto &point : this->target_points_) point.valid = false;
    return true;
  }

  const size_t point_num = line.find("PointNum:");
  if (point_num != std::string::npos) {
    unsigned count = 0;
    if (std::sscanf(line.c_str() + point_num, "PointNum:%u", &count) == 1) {
      this->target_list_expected_ = static_cast<uint8_t>(count > 9 ? 9 : count);
      this->target_list_receiving_ = true;
      if (this->target_list_expected_ == 0) this->publish_target_snapshot_(now_ms);
    }
    return true;
  }

  if (line.find("T_ID") != std::string::npos && line.find("R(m)") != std::string::npos) return true;

  uint8_t id = 0;
  float distance_m = 0.0f;
  float speed_kmh = 0.0f;
  float snr = 0.0f;
  if (this->parse_target_row_(line, id, distance_m, speed_kmh, snr)) {
    if (!this->target_list_receiving_) {
      this->target_list_receiving_ = true;
      this->target_list_started_ms_ = now_ms;
      this->target_list_count_ = 0;
      for (auto &point : this->target_points_) point.valid = false;
    }
    TargetPoint &point = this->target_points_[id];
    if (!point.valid) this->target_list_count_++;
    point.valid = true;
    point.id = id;
    point.distance_m = distance_m;
    point.speed_kmh = speed_kmh;
    point.snr = snr;
    this->target_list_last_row_ms_ = now_ms;
    if (this->target_list_expected_ > 0 && this->target_list_count_ >= this->target_list_expected_) {
      this->publish_target_snapshot_(now_ms);
    }
    return true;
  }

  // HEADA is the compact summary emitted after the detailed ASCII rows.
  if (this->target_list_receiving_ && line.rfind("HEADA", 0) == 0) {
    this->publish_target_snapshot_(now_ms);
    return true;
  }
  return false;
}

void LDL508PROComponent::request_target_list_(uint32_t now_ms) {
  this->last_multi_target_poll_ms_ = now_ms;
  this->target_list_receiving_ = false;
  this->target_list_last_row_ms_ = 0;
  this->config_send_command("RfeTargetListGet");
}

void LDL508PROComponent::publish_target_snapshot_(uint32_t now_ms) {
  if (!this->target_list_receiving_) return;
  this->target_list_receiving_ = false;
  const uint8_t count = this->target_list_count_;
  if (count > this->max_simultaneous_targets_) this->max_simultaneous_targets_ = count;

  if (this->target_count_sensor_ != nullptr && this->last_published_target_count_ != count) {
    this->target_count_sensor_->publish_state(count);
    this->last_published_target_count_ = count;
  }
  if (this->max_simultaneous_targets_sensor_ != nullptr &&
      this->last_published_max_targets_ != this->max_simultaneous_targets_) {
    this->max_simultaneous_targets_sensor_->publish_state(this->max_simultaneous_targets_);
    this->last_published_max_targets_ = this->max_simultaneous_targets_;
  }
  const bool multiple_active = count > 1;
  if (this->multi_target_active_sensor_ != nullptr &&
      (!this->multi_active_has_state_ || this->last_published_multi_active_ != multiple_active)) {
    this->multi_target_active_sensor_->publish_state(multiple_active);
    this->last_published_multi_active_ = multiple_active;
    this->multi_active_has_state_ = true;
  }

  std::string snapshot = "{\"count\":" + std::to_string(count) + ",\"targets\":[";
  bool first = true;
  char item[80];
  for (const auto &point : this->target_points_) {
    if (!point.valid) continue;
    if (std::isfinite(point.speed_kmh) && std::isfinite(point.snr)) {
      std::snprintf(item, sizeof(item), "%s{\"id\":%u,\"r\":%.1f,\"v\":%.1f,\"snr\":%.1f}",
                    first ? "" : ",", static_cast<unsigned>(point.id), point.distance_m, point.speed_kmh, point.snr);
    } else {
      std::snprintf(item, sizeof(item), "%s{\"id\":%u,\"r\":%.1f}",
                    first ? "" : ",", static_cast<unsigned>(point.id), point.distance_m);
    }
    snapshot += item;
    first = false;
  }
  snapshot += "]}";
  this->publish_multitarget_parsed_mqtt_(snapshot);
  if (this->multi_target_snapshot_sensor_ != nullptr && snapshot != this->last_published_snapshot_) {
    this->multi_target_snapshot_sensor_->publish_state(snapshot);
    this->last_published_snapshot_ = snapshot;
  }

  std::string status;
  if (count == 0 && this->target_detected_) {
    status = "Inkonsistent: Einzelziel aktiv, Mehrzielliste leer";
  } else if (count == 0) {
    status = "Mehrzielmodus: kein Ziel";
  } else if (count == 1) {
    status = "Mehrzielmodus: ein Ziel";
  } else {
    status = std::to_string(count) + " Ziele im Mehrzielmodus";
  }
  if (this->multi_target_status_sensor_ != nullptr && status != this->last_multi_target_status_) {
    this->multi_target_status_sensor_->publish_state(status);
    this->last_multi_target_status_ = status;
  }

  if (this->target_list_manual_request_ || count > 0 || (count == 0 && this->target_detected_)) {
    ESP_LOGI(TAG, "Multi-target snapshot: %u target(s), maximum observed %u%s", static_cast<unsigned>(count),
             static_cast<unsigned>(this->max_simultaneous_targets_),
             (count == 0 && this->target_detected_) ? "; inconsistent with active single-target stream" : "");
  }
  this->target_list_manual_request_ = false;
  (void) now_ms;
}

void LDL508PROComponent::start_raw_capture_(uint32_t now_ms, const char *reason) {
  this->raw_capture_active_ = true;
  this->raw_capture_hex_mode_ = false;
  this->raw_capture_auto_restore_mode_1_ = false;
  this->raw_capture_started_ms_ = now_ms;
  this->raw_capture_line_count_ = 0;
  this->raw_capture_byte_count_ = 0;
  this->raw_capture_block_count_ = 0;
  this->raw_hex_block_length_ = 0;
  ESP_LOGW(TAG, "Phase 6.7 text UART capture started for %" PRIu32 " ms (%s)", this->raw_capture_duration_ms_, reason);
  if (this->raw_capture_status_sensor_ != nullptr) {
    this->raw_capture_status_sensor_->publish_state(std::string("Textaufzeichnung aktiv: ") + reason);
  }
}

void LDL508PROComponent::start_hex_capture_(uint32_t now_ms, const char *reason, bool auto_restore_mode_1) {
  this->rx_buffer_.clear();
  this->discard_until_newline_ = false;
  this->raw_capture_active_ = true;
  this->raw_capture_hex_mode_ = true;
  this->raw_capture_auto_restore_mode_1_ = auto_restore_mode_1;
  this->raw_capture_started_ms_ = now_ms;
  this->raw_capture_line_count_ = 0;
  this->raw_capture_byte_count_ = 0;
  this->raw_capture_block_count_ = 0;
  this->raw_hex_block_length_ = 0;
  ESP_LOGW(TAG, "Phase 6.4 HEX capture started: max %" PRIu32 " ms / %" PRIu32 " bytes (%s)",
           this->raw_capture_duration_ms_, this->raw_capture_max_bytes_, reason);
  if (this->raw_capture_status_sensor_ != nullptr) {
    this->raw_capture_status_sensor_->publish_state(std::string("HEX-Aufzeichnung aktiv: ") + reason);
  }
}

void LDL508PROComponent::flush_hex_block_() {
  if (this->raw_hex_block_length_ == 0) return;
  char hex[32 * 3 + 1];
  char ascii[33];
  size_t pos = 0;
  for (uint8_t i = 0; i < this->raw_hex_block_length_; i++) {
    const uint8_t value = this->raw_hex_block_[i];
    const int written = std::snprintf(hex + pos, sizeof(hex) - pos, "%02X%s", value,
                                      i + 1 < this->raw_hex_block_length_ ? " " : "");
    if (written > 0) pos += static_cast<size_t>(written);
    ascii[i] = (value >= 32 && value <= 126) ? static_cast<char>(value) : '.';
  }
  hex[pos] = '\0';
  ascii[this->raw_hex_block_length_] = '\0';
  this->raw_capture_block_count_++;
  ESP_LOGI(TAG, "RAW6.4-HEX[%" PRIu32 "] @%" PRIu32 ": %s |%s|",
           this->raw_capture_block_count_, this->raw_capture_byte_count_ - this->raw_hex_block_length_, hex, ascii);
  this->raw_hex_block_length_ = 0;
}

void LDL508PROComponent::capture_hex_byte_(uint8_t byte, uint32_t now_ms) {
  this->raw_hex_block_[this->raw_hex_block_length_++] = byte;
  this->raw_capture_byte_count_++;
  if (this->raw_hex_block_length_ >= this->raw_hex_block_.size()) this->flush_hex_block_();
  if (this->raw_capture_max_bytes_ > 0 && this->raw_capture_byte_count_ >= this->raw_capture_max_bytes_) {
    this->finish_raw_capture_(now_ms);
  }
}

void LDL508PROComponent::send_target_mode_command_(uint8_t mode) {
  char command[40];
  std::snprintf(command, sizeof(command), "RfeTargetModeSwitch:{%u}", mode);
  ESP_LOGW(TAG, "EXPERIMENTAL TX: %s", command);
  this->write_str(command);
  this->write_str("\r\n");
  if (this->last_cli_command_sensor_ != nullptr) this->last_cli_command_sensor_->publish_state(command);
}

void LDL508PROComponent::finish_raw_capture_(uint32_t now_ms) {
  if (!this->raw_capture_active_) return;
  if (this->raw_capture_hex_mode_) this->flush_hex_block_();
  const bool was_hex = this->raw_capture_hex_mode_;
  const bool restore_mode_1 = this->raw_capture_auto_restore_mode_1_;
  this->raw_capture_active_ = false;
  this->raw_capture_hex_mode_ = false;
  this->raw_capture_auto_restore_mode_1_ = false;
  this->rx_buffer_.clear();
  this->discard_until_newline_ = false;
  const uint32_t elapsed = static_cast<uint32_t>(now_ms - this->raw_capture_started_ms_);
  if (was_hex) {
    ESP_LOGW(TAG, "Phase 6.4 HEX capture finished: %" PRIu32 " bytes in %" PRIu32 " blocks / %" PRIu32 " ms",
             this->raw_capture_byte_count_, this->raw_capture_block_count_, elapsed);
  } else {
    ESP_LOGW(TAG, "Phase 6.4 text capture finished: %" PRIu32 " lines in %" PRIu32 " ms",
             this->raw_capture_line_count_, elapsed);
  }
  if (this->raw_capture_status_sensor_ != nullptr) {
    char status[120];
    if (was_hex) {
      std::snprintf(status, sizeof(status), "Fertig: %" PRIu32 " Bytes / %" PRIu32 " Blöcke / %.1f s",
                    this->raw_capture_byte_count_, this->raw_capture_block_count_, elapsed / 1000.0f);
    } else {
      std::snprintf(status, sizeof(status), "Fertig: %" PRIu32 " Zeilen / %.1f s",
                    this->raw_capture_line_count_, elapsed / 1000.0f);
    }
    this->raw_capture_status_sensor_->publish_state(status);
  }
  if (restore_mode_1) {
    ESP_LOGW(TAG, "Phase 6.4 safety restore: switching radar back to ASCII mode 1");
    this->requested_target_mode_ = 1;
    this->send_target_mode_command_(1);
    if (this->target_mode_status_sensor_ != nullptr) {
      this->target_mode_status_sensor_->publish_state("Modus 1 automatisch wiederhergestellt (ASCII)");
    }
  }
}

void LDL508PROComponent::start_raw_capture() {
  this->start_raw_capture_(millis(), "manuell");
}

void LDL508PROComponent::request_target_mode(uint8_t mode) {
  if (mode > 1) return;
  if (!this->boot_read_started_ || !this->config_manager_.is_synchronized() || this->config_manager_.is_busy()) {
    ESP_LOGW(TAG, "Target mode %u request ignored while configuration is not ready", mode);
    if (this->target_mode_status_sensor_ != nullptr)
      this->target_mode_status_sensor_->publish_state("Nicht gesendet: Konfiguration beschäftigt");
    return;
  }

  this->requested_target_mode_ = static_cast<int8_t>(mode);
  this->rx_buffer_.clear();
  this->multi_target_stream_buffer_.clear();
  this->discard_until_newline_ = false;

  if (mode == 0) {
    this->multi_target_stream_mode_ = true;
    this->multi_target_last_seen_ms_ = 0;
    this->multi_target_valid_frames_ = 0;
    this->multi_target_target_frames_ = 0;
    this->multi_target_empty_frames_ = 0;
    this->multi_target_checksum_errors_ = 0;
    this->send_target_mode_command_(0);
    if (this->target_mode_status_sensor_ != nullptr)
      this->target_mode_status_sensor_->publish_state("Modus 0 aktiv: Mehrziel-ASCII wird ausgewertet");
    if (this->multi_target_status_sensor_ != nullptr) {
      this->last_multi_target_status_ = "Mehrzielmodus aktiv – warte auf HEADA-Frames";
      this->multi_target_status_sensor_->publish_state(this->last_multi_target_status_);
    }
    ESP_LOGI(TAG, "Experimental HEADA mode enabled");
  } else {
    if (this->multi_target_stream_mode_) {
      ESP_LOGI(TAG, "Multi-target statistics: valid=%" PRIu32 ", positive=%" PRIu32
                    ", empty=%" PRIu32 ", checksum_errors=%" PRIu32,
               this->multi_target_valid_frames_, this->multi_target_target_frames_,
               this->multi_target_empty_frames_, this->multi_target_checksum_errors_);
    }
    this->multi_target_stream_mode_ = false;
    this->send_target_mode_command_(1);
    if (this->target_mode_status_sensor_ != nullptr)
      this->target_mode_status_sensor_->publish_state("Modus 1 aktiv: ASCII-Einzelziel");
    ESP_LOGI(TAG, "Phase 6.7 single-target mode restored");
  }
}

void LDL508PROComponent::request_multi_target_snapshot() {
  if (this->multi_target_stream_mode_) {
    if (this->multi_target_status_sensor_ != nullptr)
      this->multi_target_status_sensor_->publish_state("Live-Mehrzielmodus aktiv; Snapshot wird laufend aktualisiert");
    return;
  }
  if (!this->boot_read_started_ || !this->config_manager_.is_synchronized() || this->config_manager_.is_busy()) {
    ESP_LOGW(TAG, "Multi-target snapshot request ignored while radar configuration is not ready");
    if (this->multi_target_status_sensor_ != nullptr) {
      const std::string status = "Anfrage verschoben: Konfiguration nicht bereit";
      if (status != this->last_multi_target_status_) {
        this->multi_target_status_sensor_->publish_state(status);
        this->last_multi_target_status_ = status;
      }
    }
    return;
  }
  this->target_list_manual_request_ = true;
  this->request_target_list_(millis());
}

void LDL508PROComponent::publish_detection_(bool detected) {
  const uint32_t now = millis();
  const bool changed = this->target_detected_ != detected;
  const bool sensor_needs_initial_state = this->detected_sensor_ != nullptr && !this->detected_sensor_->has_state();

  if (this->detection_state_initialized_ && changed && !detected) {
    this->led_afterglow_active_ = this->led_red_afterglow_ms_ > 0;
    this->led_afterglow_started_ms_ = now;
    if (!this->led_afterglow_active_) this->led_idle_started_ms_ = now;
  } else if (detected) {
    this->led_afterglow_active_ = false;
    this->led_idle_started_ms_ = now;
  }

  this->target_detected_ = detected;
  this->detection_state_initialized_ = true;
  if (this->detected_sensor_ != nullptr && (changed || sensor_needs_initial_state)) {
    this->detected_sensor_->publish_state(detected);
  }
  this->update_status_outputs_();
}

void LDL508PROComponent::set_led_setting(LEDSetting setting, float seconds) {
  if (!std::isfinite(seconds)) return;
  const uint32_t milliseconds = static_cast<uint32_t>(std::max(0.0f, seconds) * 1000.0f);
  if (setting == LEDSetting::RED_AFTERGLOW) {
    this->led_red_afterglow_ms_ = milliseconds;
    if (milliseconds == 0 && this->led_afterglow_active_) {
      this->led_afterglow_active_ = false;
      this->led_idle_started_ms_ = millis();
    }
  } else {
    this->led_standby_timeout_ms_ = milliseconds;
  }
  auto it = this->led_numbers_.find(setting);
  if (it != this->led_numbers_.end()) it->second->publish_state(milliseconds / 1000.0f);
  this->update_status_outputs_();
}

void LDL508PROComponent::set_led_fault_(bool active) {
  if (this->led_fault_active_ == active) return;
  this->led_fault_active_ = active;
  this->led_fault_blink_state_ = false;
  this->led_fault_blink_ms_ = millis();
  ESP_LOGW(TAG, "LED fault indication: %s", active ? "ACTIVE (red blinking)" : "cleared");
  this->update_status_outputs_();
}

void LDL508PROComponent::update_status_outputs_() {
  const uint32_t now = millis();
  bool red = false;
  bool green = false;

  // A configuration fault has highest priority: red blinks at 1 Hz, green is off.
  if (this->led_fault_active_) {
    if (static_cast<uint32_t>(now - this->led_fault_blink_ms_) >= 500) {
      this->led_fault_blink_ms_ = now;
      this->led_fault_blink_state_ = !this->led_fault_blink_state_;
    }
    red = this->led_fault_blink_state_;
  } else if (this->target_detected_) {
    red = true;
  } else {
    if (this->led_afterglow_active_ &&
        static_cast<uint32_t>(now - this->led_afterglow_started_ms_) >= this->led_red_afterglow_ms_) {
      this->led_afterglow_active_ = false;
      this->led_idle_started_ms_ = now;
    }
    if (this->led_afterglow_active_) {
      red = true;
    } else {
      // A standby value of 0 disables standby and keeps green on continuously.
      green = this->led_standby_timeout_ms_ == 0 ||
              static_cast<uint32_t>(now - this->led_idle_started_ms_) < this->led_standby_timeout_ms_;
    }
  }

  // The external LED driver is active HIGH and switches the channel's GND.
  if (this->red_output_pin_ != nullptr) this->red_output_pin_->digital_write(red);
  if (this->green_output_pin_ != nullptr) this->green_output_pin_->digital_write(green);
}

void LDL508PROComponent::finish_vehicle_event_(uint32_t now_ms) {
  VehicleEvent event{};
  if (!this->vehicle_tracker_.finish(now_ms, event)) return;
  this->completed_vehicle_count_++;
  ESP_LOGI(TAG, "Vehicle #%" PRIu32 " completed: %s, %.1f km/h max, %.1f km/h average, %" PRIu32 " samples",
           event.id, vehicle_direction_to_string(event.direction), event.max_speed_kmh, event.average_speed_kmh,
           event.sample_count);
  this->publish_vehicle_event_(event);
}

void LDL508PROComponent::publish_vehicle_event_(const VehicleEvent &event) {
  if (this->vehicle_tracking_sensor_ != nullptr) this->vehicle_tracking_sensor_->publish_state(false);
  if (this->vehicle_direction_sensor_ != nullptr)
    this->vehicle_direction_sensor_->publish_state(vehicle_direction_to_string(event.direction));
  if (this->vehicle_id_sensor_ != nullptr) this->vehicle_id_sensor_->publish_state(event.id);
  if (this->vehicle_count_sensor_ != nullptr) this->vehicle_count_sensor_->publish_state(this->completed_vehicle_count_);
  if (this->vehicle_max_speed_sensor_ != nullptr) this->vehicle_max_speed_sensor_->publish_state(event.max_speed_kmh);
  if (this->vehicle_average_speed_sensor_ != nullptr) this->vehicle_average_speed_sensor_->publish_state(event.average_speed_kmh);
  if (this->vehicle_start_distance_sensor_ != nullptr) this->vehicle_start_distance_sensor_->publish_state(event.first_distance_m);
  if (this->vehicle_end_distance_sensor_ != nullptr) this->vehicle_end_distance_sensor_->publish_state(event.last_distance_m);
  if (this->vehicle_min_distance_sensor_ != nullptr) this->vehicle_min_distance_sensor_->publish_state(event.minimum_distance_m);
  if (this->vehicle_duration_sensor_ != nullptr) this->vehicle_duration_sensor_->publish_state(event.duration_ms / 1000.0f);
  if (this->vehicle_samples_sensor_ != nullptr) this->vehicle_samples_sensor_->publish_state(event.sample_count);

  char json[512];
  std::snprintf(json, sizeof(json),
                "{\"id\":%" PRIu32 ",\"direction\":\"%s\",\"start_distance_m\":%.1f,"
                "\"end_distance_m\":%.1f,\"minimum_distance_m\":%.1f,\"max_speed_kmh\":%.1f,"
                "\"average_speed_kmh\":%.1f,\"duration_s\":%.1f,\"samples\":%" PRIu32 ","
                "\"ghosts_filtered\":%" PRIu32 ",\"firmware\":\"stable-1.0-mqtt\"}",
                event.id, vehicle_direction_to_string(event.direction), event.first_distance_m, event.last_distance_m,
                event.minimum_distance_m, event.max_speed_kmh, event.average_speed_kmh, event.duration_ms / 1000.0f,
                event.sample_count, this->artifact_filtered_count_);

  if (this->last_vehicle_event_sensor_ != nullptr) this->last_vehicle_event_sensor_->publish_state(json);
  this->publish_vehicle_event_mqtt_(json);
}

std::string LDL508PROComponent::resolve_mqtt_topic_(const std::string &configured, const char *suffix) const {
#ifdef USE_MQTT
  if (!configured.empty()) return configured;
  if (mqtt::global_mqtt_client != nullptr) return mqtt::global_mqtt_client->get_topic_prefix() + suffix;
#endif
  return std::string();
}

void LDL508PROComponent::capture_multitarget_byte_(uint8_t byte, uint32_t now_ms) {
  // Direct UART tap: called immediately after read_byte(), before every parser.
  if (this->multitarget_byte_block_length_ >= this->multitarget_byte_block_.size())
    this->flush_multitarget_byte_block_();

  const uint32_t now_us = micros();
  const uint16_t index = this->multitarget_byte_block_length_;
  if (index == 0) {
    this->multitarget_byte_first_us_ = now_us;
    this->multitarget_byte_delta_us_[index] = 0;
  } else {
    this->multitarget_byte_delta_us_[index] = static_cast<uint32_t>(now_us - this->multitarget_byte_previous_us_);
  }
  this->multitarget_byte_previous_us_ = now_us;
  this->multitarget_byte_block_[index] = byte;
  this->multitarget_byte_block_length_++;
  this->multitarget_byte_last_byte_ms_ = now_ms;

  if (this->multitarget_byte_block_length_ >= this->multitarget_byte_block_.size())
    this->flush_multitarget_byte_block_();
}

void LDL508PROComponent::flush_multitarget_byte_block_() {
  if (this->multitarget_byte_block_length_ == 0) return;

  std::string hex;
  std::string ascii;
  std::string delta_us;
  hex.reserve(this->multitarget_byte_block_length_ * 3U);
  ascii.reserve(this->multitarget_byte_block_length_ * 4U);
  delta_us.reserve(this->multitarget_byte_block_length_ * 7U);
  char part[4];
  for (uint16_t i = 0; i < this->multitarget_byte_block_length_; i++) {
    const uint8_t value = this->multitarget_byte_block_[i];
    std::snprintf(part, sizeof(part), "%02X", value);
    if (!hex.empty()) hex += " ";
    hex += part;
    if (value >= 0x20 && value <= 0x7E && value != '\\' && value != '"') {
      ascii.push_back(static_cast<char>(value));
    } else {
      char escaped[5];
      std::snprintf(escaped, sizeof(escaped), "\\x%02X", value);
      ascii += escaped;
    }
    if (i > 0) delta_us += ",";
    delta_us += std::to_string(this->multitarget_byte_delta_us_[i]);
  }

  const uint32_t sequence = ++this->multitarget_byte_sequence_;
  const uint32_t span_us = static_cast<uint32_t>(this->multitarget_byte_previous_us_ - this->multitarget_byte_first_us_);
  std::string payload = "{\"source\":\"uart_rx_before_parser\",\"seq\":" + std::to_string(sequence) +
                        ",\"first_us\":" + std::to_string(this->multitarget_byte_first_us_) +
                        ",\"span_us\":" + std::to_string(span_us) +
                        ",\"len\":" + std::to_string(this->multitarget_byte_block_length_) +
                        ",\"hex\":\"" + hex + "\",\"ascii\":\"" + ascii +
                        "\",\"delta_us\":[" + delta_us + "]}";
  ESP_LOGI(TAG, "RAW7.1.3-UART #%" PRIu32 " len=%u span=%" PRIu32 "us: %s", sequence,
           static_cast<unsigned>(this->multitarget_byte_block_length_), span_us, hex.c_str());

#ifdef USE_MQTT
  if (this->multitarget_raw_mqtt_enabled_ && mqtt::global_mqtt_client != nullptr &&
      mqtt::global_mqtt_client->is_connected()) {
    const std::string topic = this->resolve_mqtt_topic_(this->multitarget_byte_mqtt_topic_, "/debug/raw_bytes");
    if (!topic.empty()) mqtt::global_mqtt_client->publish(topic, payload, this->multitarget_mqtt_qos_, false);
  }
#endif
  this->multitarget_byte_block_length_ = 0;
  this->multitarget_byte_first_us_ = 0;
  this->multitarget_byte_previous_us_ = 0;
}

void LDL508PROComponent::publish_multitarget_raw_mqtt_(const std::string &payload, bool hex) {
  if (!this->multitarget_raw_mqtt_enabled_) return;
#ifdef USE_MQTT
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) return;
  const std::string topic = this->resolve_mqtt_topic_(this->multitarget_raw_mqtt_topic_,
                                                      hex ? "/debug/raw_hex" : "/debug/raw_ascii");
  if (!topic.empty()) {
    const std::string safe_payload = escape_raw_bytes_(payload);
    mqtt::global_mqtt_client->publish(topic, safe_payload, this->multitarget_mqtt_qos_, false);
  }
#else
  (void) payload;
  (void) hex;
#endif
}

void LDL508PROComponent::publish_multitarget_parsed_mqtt_(const std::string &payload) {
  if (this->multitarget_debug_mode_ == "off") return;
#ifdef USE_MQTT
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) return;
  const std::string topic = this->resolve_mqtt_topic_(this->multitarget_parsed_mqtt_topic_, "/debug/parsed");
  if (!topic.empty()) mqtt::global_mqtt_client->publish(topic, payload, this->multitarget_mqtt_qos_, false);
#else
  (void) payload;
#endif
}

void LDL508PROComponent::send_hex_target_mode_command_(uint8_t mode) {
  const uint8_t command[] = {0xAA, 0xAA, 0x00, 0x48, mode, 0x55, 0x55};
  ESP_LOGW(TAG, "PHASE7.1 HEX TX: AA AA 00 48 %02X 55 55", mode);
  this->write_array(command, sizeof(command));
}

void LDL508PROComponent::start_multitarget_debug_() {
  this->rx_buffer_.clear();
  this->ingress_buffer_.clear();
  this->multi_target_stream_buffer_.clear();
  this->discard_until_newline_ = false;
  this->multitarget_byte_block_length_ = 0;
  this->multitarget_byte_first_us_ = 0;
  this->multitarget_byte_previous_us_ = 0;
  this->multitarget_byte_sequence_ = 0;
  if (this->multitarget_debug_mode_ == "ascii") {
    ESP_LOGW(TAG, "Phase 7.1.3 ASCII multi-target byte recorder starting (parser remains active)");
    this->request_target_mode(0);
    this->set_timeout("phase71_headw", 300, [this]() {
      ESP_LOGW(TAG, "PHASE7.1 TX: HEADW4k");
      this->write_str("HEADW4k\r\n");
    });
  } else if (this->multitarget_debug_mode_ == "hex") {
    ESP_LOGW(TAG, "Phase 7.1.3 HEX multi-target byte recorder starting; parser disabled");
    this->multi_target_stream_mode_ = false;
    this->send_hex_target_mode_command_(0);
  }
}

void LDL508PROComponent::publish_vehicle_event_mqtt_(const char *json) {
  if (!this->mqtt_event_enabled_) return;
#ifdef USE_MQTT
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
    ESP_LOGW(TAG, "MQTT vehicle event skipped: client not connected");
    return;
  }

  std::string topic = this->mqtt_event_topic_;
  if (topic.empty()) topic = mqtt::global_mqtt_client->get_topic_prefix() + "/vehicle/event";

  const bool published = mqtt::global_mqtt_client->publish(topic, json, this->mqtt_event_qos_, this->mqtt_event_retain_);
  if (published) {
    ESP_LOGI(TAG, "MQTT vehicle event published: %s", topic.c_str());
  } else {
    ESP_LOGW(TAG, "MQTT vehicle event publish failed: %s", topic.c_str());
  }
#else
  ESP_LOGW(TAG, "MQTT vehicle event enabled, but MQTT is not configured");
#endif
}

void LDL508PROComponent::config_send_command(const std::string &command) {
  ESP_LOGD(TAG, "TX: %s", command.c_str());
  if (this->last_cli_command_sensor_ != nullptr) this->last_cli_command_sensor_->publish_state(command);
  this->write_str(command.c_str());
  this->write_str("\r\n");
}

void LDL508PROComponent::config_value_received(RadarParameter parameter, float value) {
  ESP_LOGI(TAG, "Config %s = %.3f", ConfigManager::definition(parameter).name, value);

  auto number_it = this->numbers_.find(parameter);
  if (number_it != this->numbers_.end()) number_it->second->publish_state(value);

  auto switch_it = this->switches_.find(parameter);
  if (switch_it != this->switches_.end()) switch_it->second->publish_state(value != 0.0f);

  auto select_it = this->selects_.find(parameter);
  if (select_it != this->selects_.end()) {
    const char *option = select_option_(parameter, static_cast<int>(std::lround(value)));
    if (option != nullptr) select_it->second->publish_state(option);
  }

  this->publish_configuration_();
}

void LDL508PROComponent::config_sync_changed(bool synchronized) {
  ESP_LOGI(TAG, "Configuration synchronized: %s", synchronized ? "YES" : "NO");
  if (this->config_synchronized_sensor_ != nullptr) this->config_synchronized_sensor_->publish_state(synchronized);
  if (synchronized) {
    this->set_led_fault_(false);
    this->publish_error_("Kein Fehler");
    if (this->auto_enable_multi_target_after_sync_ && !this->auto_mode0_requested_) {
      this->auto_mode0_requested_ = true;
      ESP_LOGI(TAG, "Configuration complete; enabling target mode 0 automatically");
      this->request_target_mode(0);
    }
  }
}

void LDL508PROComponent::config_error(const std::string &message) {
  ESP_LOGW(TAG, "Configuration error: %s", message.c_str());
  this->set_led_fault_(true);
  this->publish_error_(message);
}

void LDL508PROComponent::publish_configuration_() {
  if (this->configuration_sensor_ != nullptr) {
    this->configuration_sensor_->publish_state(this->config_manager_.configuration_json());
  }
}

void LDL508PROComponent::publish_error_(const std::string &message) {
  if (this->last_config_error_sensor_ != nullptr) this->last_config_error_sensor_->publish_state(message);
}

void LDL508PROComponent::set_numeric_parameter(RadarParameter parameter, float value) {
  std::string error;
  if (!this->validate_numeric_(parameter, value, error)) {
    this->config_error(error);
    return;
  }
  this->config_manager_.queue_write(parameter, value);
}

void LDL508PROComponent::set_switch_parameter(RadarParameter parameter, bool value) {
  this->config_manager_.queue_write(parameter, value ? 1.0f : 0.0f);
}

void LDL508PROComponent::set_select_parameter(RadarParameter parameter, const std::string &value) {
  int raw = -1;
  if (parameter == RadarParameter::POWER_MODE) {
    if (value == "Volle Geschwindigkeit") raw = 0;
    else if (value == "Normal") raw = 1;
    else if (value == "Ultra-Low-Power") raw = 2;
  } else if (parameter == RadarParameter::DOPPLER_FILTER) {
    if (value == "Beide Richtungen") raw = 0;
    else if (value == "Nur kommende Fahrzeuge") raw = 1;
    else if (value == "Nur sich entfernende Fahrzeuge") raw = 2;
  }
  if (raw < 0) {
    this->config_error("Unbekannte Select-Auswahl: " + value);
    return;
  }
  this->config_manager_.queue_write(parameter, static_cast<float>(raw));
}

void LDL508PROComponent::refresh_configuration() {
  ESP_LOGI(TAG, "Manual configuration refresh requested");
  this->config_manager_.queue_read_all();
}

void LDL508PROComponent::factory_reset() {
  ESP_LOGW(TAG, "Factory reset requested");
  this->config_manager_.queue_raw("RfeLoadDefaultConfiguration");
  this->set_timeout("ldl508pro_factory_refresh", 1800, [this]() { this->refresh_configuration(); });
}

bool LDL508PROComponent::validate_numeric_(RadarParameter parameter, float value, std::string &error) const {
  float min_value = 0.0f;
  float max_value = 100.0f;
  switch (parameter) {
    case RadarParameter::CFAR: min_value = 0.0f; max_value = 100.0f; break;
    case RadarParameter::MAX_FRAMERATE: min_value = 1.0f; max_value = 100.0f; break;
    case RadarParameter::SPEED_LIMIT_HIGH:
    case RadarParameter::SPEED_LIMIT_LOW:
    case RadarParameter::SPEED_THRESHOLD: min_value = 0.0f; max_value = 180.0f; break;
    case RadarParameter::DISTANCE_LIMIT_HIGH:
    case RadarParameter::DISTANCE_LIMIT_LOW: min_value = 0.0f; max_value = 140.0f; break;
    case RadarParameter::DURATION: min_value = 0.0f; max_value = 60.0f; break;
    case RadarParameter::SNR_FILTER: min_value = 0.0f; max_value = 100.0f; break;
    default: return true;
  }
  if (!std::isfinite(value) || value < min_value || value > max_value) {
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "%s: %.2f liegt außerhalb %.2f bis %.2f",
                  ConfigManager::definition(parameter).name, value, min_value, max_value);
    error = buffer;
    return false;
  }

  if (parameter == RadarParameter::DISTANCE_LIMIT_LOW) {
    const auto &high = this->config_manager_.value(RadarParameter::DISTANCE_LIMIT_HIGH);
    if (high.valid && value > high.numeric) {
      error = "Entfernung Minimum darf nicht größer als Entfernung Maximum sein";
      return false;
    }
  }
  if (parameter == RadarParameter::DISTANCE_LIMIT_HIGH) {
    const auto &low = this->config_manager_.value(RadarParameter::DISTANCE_LIMIT_LOW);
    if (low.valid && value < low.numeric) {
      error = "Entfernung Maximum darf nicht kleiner als Entfernung Minimum sein";
      return false;
    }
  }
  if (parameter == RadarParameter::SPEED_LIMIT_LOW) {
    const auto &high = this->config_manager_.value(RadarParameter::SPEED_LIMIT_HIGH);
    if (high.valid && value > high.numeric) {
      error = "Geschwindigkeit Minimum darf nicht größer als Geschwindigkeit Maximum sein";
      return false;
    }
  }
  if (parameter == RadarParameter::SPEED_LIMIT_HIGH) {
    const auto &low = this->config_manager_.value(RadarParameter::SPEED_LIMIT_LOW);
    if (low.valid && value < low.numeric) {
      error = "Geschwindigkeit Maximum darf nicht kleiner als Geschwindigkeit Minimum sein";
      return false;
    }
  }
  return true;
}

const char *LDL508PROComponent::select_option_(RadarParameter parameter, int raw) {
  if (parameter == RadarParameter::POWER_MODE) {
    switch (raw) {
      case 0: return "Volle Geschwindigkeit";
      case 1: return "Normal";
      case 2: return "Ultra-Low-Power";
      default: return nullptr;
    }
  }
  if (parameter == RadarParameter::DOPPLER_FILTER) {
    switch (raw) {
      case 0: return "Beide Richtungen";
      case 1: return "Nur kommende Fahrzeuge";
      case 2: return "Nur sich entfernende Fahrzeuge";
      default: return nullptr;
    }
  }
  return nullptr;
}

void LDL508PROComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "LDL508PRO:");
  ESP_LOGCONFIG(TAG, "  Build stage: Stable 1.0 + early ghost filter + direct MQTT vehicle events");
  ESP_LOGCONFIG(TAG, "  UART debug: %s", this->debug_uart_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Target timeout: %" PRIu32 " ms", this->target_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Ghost filter: %s (%.1f +/- %.1f m, |speed| %.1f +/- %.1f km/h)",
                this->artifact_filter_enabled_ ? "ENABLED" : "DISABLED", this->artifact_distance_m_,
                this->artifact_distance_tolerance_m_, this->artifact_speed_kmh_,
                this->artifact_speed_tolerance_kmh_);
  ESP_LOGCONFIG(TAG, "  Boot read delay: %" PRIu32 " ms", this->boot_read_delay_ms_);
  ESP_LOGCONFIG(TAG, "  Command gap: %" PRIu32 " ms", this->command_gap_ms_);
  ESP_LOGCONFIG(TAG, "  Command timeout: %" PRIu32 " ms", this->command_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Command retries: %u", this->command_retries_);
  ESP_LOGCONFIG(TAG, "  Vehicle tracker: enabled");
  ESP_LOGCONFIG(TAG, "  MQTT vehicle events: %s", this->mqtt_event_enabled_ ? "ENABLED" : "DISABLED");
  ESP_LOGCONFIG(TAG, "  MQTT event topic: %s", this->mqtt_event_topic_.empty() ? "<topic_prefix>/vehicle/event" : this->mqtt_event_topic_.c_str());
  ESP_LOGCONFIG(TAG, "  MQTT event QoS/retain: %u/%s", this->mqtt_event_qos_, this->mqtt_event_retain_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Phase 7.1.3 multi-target debug mode: %s", this->multitarget_debug_mode_.c_str());
  ESP_LOGCONFIG(TAG, "  Phase 7.1 raw MQTT: %s", this->multitarget_raw_mqtt_enabled_ ? "ENABLED" : "DISABLED");
  ESP_LOGCONFIG(TAG, "  Phase 7.1.3 byte topic: %s", this->multitarget_byte_mqtt_topic_.empty() ? "<topic_prefix>/debug/raw_bytes" : this->multitarget_byte_mqtt_topic_.c_str());
  ESP_LOGCONFIG(TAG, "  Phase 7.1 raw topic: %s", this->multitarget_raw_mqtt_topic_.empty() ? "<topic_prefix>/debug/raw_ascii|raw_hex" : this->multitarget_raw_mqtt_topic_.c_str());
  ESP_LOGCONFIG(TAG, "  Phase 7.1 parsed topic: %s", this->multitarget_parsed_mqtt_topic_.empty() ? "<topic_prefix>/debug/parsed" : this->multitarget_parsed_mqtt_topic_.c_str());
  ESP_LOGCONFIG(TAG, "  Multi-target polling: %s", this->multi_target_polling_ ? "ENABLED (experimental)" : "DISABLED");
  ESP_LOGCONFIG(TAG, "  Multi-target poll interval: %" PRIu32 " ms", this->multi_target_poll_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Multi-target hold time: %" PRIu32 " ms", this->multi_target_hold_time_ms_);
  ESP_LOGCONFIG(TAG, "  Confirmation: %u frames / %" PRIu32 " ms / %.1f m tolerance / %.1f m minimum movement",
                static_cast<unsigned>(this->multi_target_confirmation_frames_),
                this->multi_target_confirmation_window_ms_, this->multi_target_confirmation_tolerance_m_,
                this->multi_target_min_confirmation_movement_m_);
  ESP_LOGCONFIG(TAG, "  Auto mode 0 after sync: %s", this->auto_enable_multi_target_after_sync_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Raw capture duration: %" PRIu32 " ms", this->raw_capture_duration_ms_);
  LOG_PIN("  Red status output: ", this->red_output_pin_);
  LOG_PIN("  Green status output: ", this->green_output_pin_);
  ESP_LOGCONFIG(TAG, "  Red LED afterglow: %.1f s", this->led_red_afterglow_ms_ / 1000.0f);
  ESP_LOGCONFIG(TAG, "  Green LED standby: %.1f s (0 = disabled)", this->led_standby_timeout_ms_ / 1000.0f);
  ESP_LOGCONFIG(TAG, "  Fault indication: red blinking at 1 Hz on configuration error");
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_SENSOR("  ", "Speed", this->speed_sensor_);
  LOG_BINARY_SENSOR("  ", "Detected", this->detected_sensor_);
  LOG_BINARY_SENSOR("  ", "Configuration synchronized", this->config_synchronized_sensor_);
  LOG_TEXT_SENSOR("  ", "Configuration JSON", this->configuration_sensor_);
  LOG_TEXT_SENSOR("  ", "Last configuration error", this->last_config_error_sensor_);
  LOG_TEXT_SENSOR("  ", "Last CLI command", this->last_cli_command_sensor_);
  LOG_BINARY_SENSOR("  ", "Vehicle tracking", this->vehicle_tracking_sensor_);
  LOG_TEXT_SENSOR("  ", "Vehicle direction", this->vehicle_direction_sensor_);
  LOG_TEXT_SENSOR("  ", "Last vehicle event", this->last_vehicle_event_sensor_);
  LOG_SENSOR("  ", "Vehicle ID", this->vehicle_id_sensor_);
  LOG_SENSOR("  ", "Vehicle count", this->vehicle_count_sensor_);
  LOG_SENSOR("  ", "Current target count", this->target_count_sensor_);
  LOG_SENSOR("  ", "Maximum simultaneous targets", this->max_simultaneous_targets_sensor_);
  LOG_BINARY_SENSOR("  ", "Multiple targets active", this->multi_target_active_sensor_);
  LOG_TEXT_SENSOR("  ", "Multi-target snapshot", this->multi_target_snapshot_sensor_);
  LOG_TEXT_SENSOR("  ", "Multi-target status", this->multi_target_status_sensor_);
  LOG_TEXT_SENSOR("  ", "Target mode status", this->target_mode_status_sensor_);
  LOG_TEXT_SENSOR("  ", "Raw capture status", this->raw_capture_status_sensor_);
}

}  // namespace ldl508pro
}  // namespace esphome
