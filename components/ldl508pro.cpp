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

static const char *const FIRMWARE_VERSION =
    "stable-1.1.1-dual-mode";

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
void LDLOperatingModeSelect::control(const std::string &value) { this->parent_->set_operating_mode(value); }
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

  this->status_lights_.setup(this->setup_ms_);

  this->carport_presence_.setup(this->setup_ms_);

  this->driveway_controller_.setup(this->setup_ms_);

  this->warning_light_.setup();

  if (this->carport_departure_sensor_ != nullptr) {
    this->carport_departure_sensor_->publish_state(false);
  }

this->update_carport_sensors_(
    this->setup_ms_);

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
  if (red_hold != this->led_numbers_.end()) {
    red_hold->second->publish_state(
        this->status_lights_.red_afterglow_ms() /
        1000.0f);
  }

  auto standby = this->led_numbers_.find(LEDSetting::STANDBY_TIMEOUT);

  if (standby != this->led_numbers_.end()) {
    standby->second->publish_state(
        this->status_lights_.standby_timeout_ms() /
        1000.0f);
  }

  if (this->operating_mode_select_ != nullptr) {
    this->operating_mode_select_->publish_state(
        "Mehrziel (Empfohlen)");
  }

  ESP_LOGCONFIG(
      TAG,
      "Initializing LDL508PRO %s with selectable single-/multi-target operation",
      FIRMWARE_VERSION);
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

  // Mehrzieltracks auch dann abschließen, wenn nach dem Fahrzeug
  // kein neuer Radar-Batch mehr eintrifft.
  if (this->operating_mode_ ==
          RadarOperatingMode::HEX_MULTI_TARGET &&
      this->runtime_config_state_ ==
          RuntimeConfigState::IDLE_HEX) {

    for (auto &track : this->multi_target_tracks_) {
      if (!track.active)
        continue;

      const uint32_t age_ms =
          static_cast<uint32_t>(now - track.last_seen_ms);

      if (age_ms <= 3000)
        continue;

      this->publish_completed_track_(track, now);

      ESP_LOGI(
          TAG,
          "TRACK-MGR expired track %u after %" PRIu32
          " ms without requiring a new batch",
          static_cast<unsigned>(track.id),
          age_ms);

      if (this->primary_vehicle_track_id_ == track.id) {
        this->primary_vehicle_track_id_ = 0;
      }

      track.active = false;
    }

    this->update_driveway_traffic_state_(now);
  }

  if (this->operating_mode_ ==
          RadarOperatingMode::HEX_MULTI_TARGET &&
      this->runtime_config_state_ ==
          RuntimeConfigState::IDLE_HEX &&
      this->pending_target_batch_active_ &&
      static_cast<uint32_t>(
          now - this->pending_target_batch_started_ms_) >= 300) {
    this->flush_multi_target_batch_(now);
  }

  if (!this->boot_mode_normalized_ &&
    static_cast<uint32_t>(now - this->setup_ms_) >= 250) {
  this->boot_mode_normalized_ = true;
  this->multi_target_stream_mode_ = false;

  const uint8_t switch_to_ascii[] = {
      0xAA, 0xAA, 0x77, 0x77, 0x55, 0x55
  };

  ESP_LOGI(TAG, "Switching radar communication protocol to ASCII for configuration sync");
  this->write_array(switch_to_ascii, sizeof(switch_to_ascii));
  }

  if (!this->boot_read_started_ && this->boot_mode_normalized_ &&
      static_cast<uint32_t>(now - this->setup_ms_) >= this->boot_read_delay_ms_) {
    this->boot_read_started_ = true;
    ESP_LOGI(TAG, "Queueing complete radar configuration read");
    this->config_manager_.queue_read_all();
  }

  this->config_manager_.loop(now);

  // Laufzeit-Konfiguration:
  // HEX -> ASCII -> schreiben/prüfen -> HEX
  if (this->runtime_config_state_ ==
          RuntimeConfigState::SWITCHING_TO_ASCII &&
      static_cast<uint32_t>(
          now - this->runtime_config_state_started_ms_) >= 300) {

    ESP_LOGI(TAG, "ASCII protocol ready");

    this->runtime_config_state_ =
        RuntimeConfigState::RUNNING_ASCII;
    this->runtime_config_state_started_ms_ = now;

    if (this->operating_mode_change_pending_ &&
        this->requested_operating_mode_ ==
            RadarOperatingMode::ASCII_SINGLE_TARGET) {

      ESP_LOGI(TAG, "Operating mode: enabling ASCII mode 1");

      this->send_target_mode_command_(1);
      this->runtime_config_command_queued_ = true;

    } else if (this->runtime_config_write_pending_) {

      this->config_manager_.queue_write(
          this->runtime_config_parameter_,
          this->runtime_config_value_);

      this->runtime_config_command_queued_ = true;

      ESP_LOGI(TAG, "CONFIG: runtime write queued");
    }
  }

  if (this->runtime_config_state_ ==
          RuntimeConfigState::RUNNING_ASCII &&
      this->operating_mode_change_pending_ &&
      this->requested_operating_mode_ ==
          RadarOperatingMode::ASCII_SINGLE_TARGET &&
      this->runtime_config_command_queued_ &&
      static_cast<uint32_t>(
          now - this->runtime_config_state_started_ms_) >= 300) {

    this->runtime_config_command_queued_ = false;
    this->runtime_config_state_ =
        RuntimeConfigState::IDLE_ASCII;
    this->runtime_config_state_started_ms_ = now;

    this->rx_buffer_.clear();
    this->ingress_buffer_.clear();

    this->apply_ascii_single_target_mode_();
  }

  if (this->runtime_config_state_ ==
          RuntimeConfigState::RUNNING_ASCII &&
      this->runtime_config_write_pending_ &&
      this->runtime_config_command_queued_ &&
      !this->config_manager_.is_busy() &&
      static_cast<uint32_t>(
          now - this->runtime_config_state_started_ms_) >= 500) {

    ESP_LOGI(TAG, "CONFIG: runtime write completed");

    this->runtime_config_write_pending_ = false;
    this->runtime_config_command_queued_ = false;

    // Erfolgreiche Transaktion löscht eine eventuell gesetzte Fehleranzeige.
    this->set_led_fault_(false);
    this->publish_error_("Kein Fehler");

    if (this->operating_mode_ ==
        RadarOperatingMode::ASCII_SINGLE_TARGET) {

      this->runtime_config_state_ =
          RuntimeConfigState::IDLE_ASCII;
      this->runtime_config_state_started_ms_ = now;

      ESP_LOGI(TAG, "CONFIG: remaining in ASCII single-target mode");

    } else {

      this->restore_runtime_hex_mode_();
    }
  }

  if (this->runtime_config_state_ ==
          RuntimeConfigState::SWITCHING_TO_HEX &&
      !this->runtime_config_command_queued_ &&
      static_cast<uint32_t>(
          now - this->runtime_config_state_started_ms_) >= 300) {

    ESP_LOGI(TAG, "CONFIG: HEX protocol ready, enabling mode 2");

    this->send_hex_target_mode_command_(2);

    this->runtime_config_command_queued_ = true;
    this->runtime_config_state_started_ms_ = now;
  }

  if (this->runtime_config_state_ ==
          RuntimeConfigState::SWITCHING_TO_HEX &&
      this->runtime_config_command_queued_ &&
      static_cast<uint32_t>(
          now - this->runtime_config_state_started_ms_) >= 300) {

    this->runtime_config_state_ =
        RuntimeConfigState::IDLE_HEX;

    this->runtime_config_state_started_ms_ = now;
    this->runtime_config_command_queued_ = false;

    this->rx_buffer_.clear();
    this->ingress_buffer_.clear();
    this->multi_target_stream_buffer_.clear();
    this->multitarget_hex_mqtt_block_length_ = 0;

    if (this->operating_mode_change_pending_ &&
        this->requested_operating_mode_ ==
            RadarOperatingMode::HEX_MULTI_TARGET) {
      this->apply_hex_multi_target_mode_();
    }

    ESP_LOGI(TAG, "CONFIG: HEX mode 2 ready");
  }

// Den konfigurierten Radar-Datenmodus erst starten, nachdem die initiale
// ASCII-Konfigurationssynchronisation abgeschlossen ist. Dadurch bleiben
// Konfigurationsverkehr und kontinuierlicher Messdatenstrom sauber getrennt.
  if (!this->multitarget_debug_started_ && this->boot_read_started_ &&
      this->config_manager_.is_synchronized() && !this->config_manager_.is_busy() &&
      this->multitarget_debug_mode_ != "off") {
    this->multitarget_debug_started_ = true;
    this->start_multitarget_debug_();
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
    ESP_LOGD(TAG, "MODE2 RAW HEX: %s", payload.c_str());
    this->publish_multitarget_raw_mqtt_(payload, true);
    this->multitarget_hex_mqtt_block_length_ = 0;
  }

  const bool mode2_hex_active =
      this->operating_mode_ ==
          RadarOperatingMode::HEX_MULTI_TARGET &&
      this->multitarget_debug_started_;

  if (mode2_hex_active) {
    bool detected_from_active_tracks = false;

    for (const auto &track : this->multi_target_tracks_) {
      if (!track.active)
        continue;

      const uint32_t track_age_ms =
          static_cast<uint32_t>(now - track.last_seen_ms);

      // Für die Erkennung gilt die interne Track-Lebensdauer.
      // Die JSON-Anzeige darf weiterhin bereits nach 1000 ms ausblenden.
      if (track_age_ms <= 3000) {
        detected_from_active_tracks = true;
        break;
      }
    }

    if (detected_from_active_tracks != this->target_detected_) {
      this->publish_detection_(detected_from_active_tracks);
    }

    if (!detected_from_active_tracks &&
        this->primary_vehicle_track_id_ != 0) {

      this->primary_vehicle_track_id_ = 0;

      if (this->vehicle_tracking_sensor_ != nullptr) {
        this->vehicle_tracking_sensor_->publish_state(false);
      }

      if (this->vehicle_direction_sensor_ != nullptr) {
        this->vehicle_direction_sensor_->publish_state("Unbekannt");
      }
    }

  } else if (!this->multi_target_stream_mode_ &&
            this->target_detected_ &&
            static_cast<uint32_t>(now - this->last_target_ms_) >=
                this->target_timeout_ms_) {
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

  // Mode 0 may interleave valid target frames with HEADA0 idle frames. Keep the
  // most recent positive snapshot until the configurable hold time expires.
  if (this->multi_target_stream_mode_ && this->target_list_count_ > 0 &&
      this->multi_target_last_seen_ms_ != 0 &&
      static_cast<uint32_t>(now - this->multi_target_last_seen_ms_) >= this->multi_target_hold_time_ms_) {
    for (auto &point : this->target_points_) point.valid = false;
    this->target_list_count_ = 0;
    this->target_list_receiving_ = true;
    this->multi_target_candidate_confirmed_ = false;
    this->multi_target_candidate_frames_ = 0;
    this->multi_target_candidate_first_distance_ = 0.0f;
    this->publish_detection_(false);
    this->publish_target_snapshot_(now);
    ESP_LOGD(TAG, "Multi-target hold expired after %" PRIu32 " ms; publishing no target",
             this->multi_target_hold_time_ms_);
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

  // zwingend auf millis() da sonst afterglow nicht funktioniert
  //this->status_lights_.loop(millis());

  //this->carport_presence_.loop(millis());

  const uint32_t peripheral_now = millis();

  this->status_lights_.loop(
      peripheral_now);

  this->carport_presence_.loop(
      peripheral_now);

  this->update_carport_sensors_(
      peripheral_now);

  this->driveway_controller_.loop(
      peripheral_now);

  this->warning_light_.set_active(
    this->driveway_controller_.active());

  // Some firmware revisions omit PointNum or the final HEADA summary. Close a
  // partially received snapshot after a short quiet period.
  if (this->multitarget_debug_mode_ != "hex" &&
      this->target_list_receiving_ &&
      !this->pending_target_batch_active_ &&
      this->target_list_last_row_ms_ != 0 &&
      static_cast<uint32_t>(now - this->target_list_last_row_ms_) >= 180) {
    this->publish_target_snapshot_(now);
  }
}

void LDL508PROComponent::reset_tracking_state_for_mode_change_() {
  const uint32_t now_ms = millis();

  // Offene Mehrzieltracks als beendet behandeln.
  for (auto &track : this->multi_target_tracks_) {
    track.active = false;
  }

  for (auto &point : this->target_points_) {
    point.valid = false;
    point.track_id = 0;
  }

  this->pending_target_slots_.fill(false);
  this->pending_target_batch_active_ = false;
  this->pending_target_batch_started_ms_ = 0;
  this->pending_target_batch_last_ms_ = 0;

  this->target_list_receiving_ = false;
  this->target_list_count_ = 0;
  this->primary_vehicle_track_id_ = 0;

  if (this->target_detected_) {
    this->publish_detection_(false);
  }

  if (this->vehicle_tracking_sensor_ != nullptr) {
    this->vehicle_tracking_sensor_->publish_state(false);
  }

  if (this->vehicle_direction_sensor_ != nullptr) {
    this->vehicle_direction_sensor_->publish_state("Unbekannt");
  }

  if (this->target_count_sensor_ != nullptr) {
    this->target_count_sensor_->publish_state(0);
    this->last_published_target_count_ = 0;
  }

  if (this->multi_target_active_sensor_ != nullptr) {
    this->multi_target_active_sensor_->publish_state(false);
    this->last_published_multi_active_ = false;
    this->multi_active_has_state_ = true;
  }

  this->rx_buffer_.clear();
  this->ingress_buffer_.clear();
  this->multi_target_stream_buffer_.clear();
  this->multitarget_hex_mqtt_block_length_ = 0;

  ESP_LOGI(TAG, "Operating mode change: tracking state reset");
}

void LDL508PROComponent::request_operating_mode_(
    RadarOperatingMode mode) {

  if (mode == this->operating_mode_ &&
      !this->operating_mode_change_pending_) {
    ESP_LOGI(TAG, "Operating mode already active");
    return;
  }

  if (this->operating_mode_change_pending_ ||
      this->runtime_config_write_pending_ ||
      this->config_manager_.is_busy()) {
    ESP_LOGW(
        TAG,
        "Operating mode change rejected: configuration or mode change busy");
    return;
  }

  this->requested_operating_mode_ = mode;
  this->operating_mode_change_pending_ = true;
  this->operating_mode_change_started_ms_ = millis();

  this->reset_tracking_state_for_mode_change_();

  if (mode == RadarOperatingMode::ASCII_SINGLE_TARGET) {
    ESP_LOGI(TAG, "Operating mode requested: ASCII single target");
    this->begin_runtime_ascii_mode_();
  } else {
    ESP_LOGI(TAG, "Operating mode requested: HEX multi target");
    this->restore_runtime_hex_mode_();
  }
}

void LDL508PROComponent::apply_ascii_single_target_mode_() {
  this->operating_mode_ =
      RadarOperatingMode::ASCII_SINGLE_TARGET;

  this->requested_operating_mode_ =
      RadarOperatingMode::ASCII_SINGLE_TARGET;

  this->operating_mode_change_pending_ = false;
  this->operating_mode_change_started_ms_ = 0;

  this->multi_target_stream_mode_ = false;
  this->requested_target_mode_ = 1;

  if (this->target_mode_status_sensor_ != nullptr) {
    this->target_mode_status_sensor_->publish_state(
        "ASCII Single Target – Modus 1");
  }

  if (this->multi_target_status_sensor_ != nullptr) {
    this->last_multi_target_status_ =
        "ASCII-Einzelzielmodus aktiv";
    this->multi_target_status_sensor_->publish_state(
        this->last_multi_target_status_);
  }

  if (this->operating_mode_select_ != nullptr) {
    this->operating_mode_select_->publish_state(
        "Einzelziel (Kompatibilität)");
  }

  ESP_LOGI(TAG, "Operating mode active: ASCII single target");
}

void LDL508PROComponent::apply_hex_multi_target_mode_() {
  this->operating_mode_ =
      RadarOperatingMode::HEX_MULTI_TARGET;

  this->requested_operating_mode_ =
      RadarOperatingMode::HEX_MULTI_TARGET;

  this->operating_mode_change_pending_ = false;
  this->operating_mode_change_started_ms_ = 0;

  this->multi_target_stream_mode_ = false;
  this->requested_target_mode_ = 2;

  if (this->target_mode_status_sensor_ != nullptr) {
    this->target_mode_status_sensor_->publish_state(
        "HEX Multi Target – Modus 2");
  }

  if (this->multi_target_status_sensor_ != nullptr) {
    this->last_multi_target_status_ =
        "HEX-Mehrzielmodus aktiv";
    this->multi_target_status_sensor_->publish_state(
        this->last_multi_target_status_);
  }

  if (this->operating_mode_select_ != nullptr) {
    this->operating_mode_select_->publish_state(
        "Mehrziel (Empfohlen)");
  }

  ESP_LOGI(TAG, "Operating mode active: HEX multi target");
}

void LDL508PROComponent::process_byte_(uint8_t byte) {
  const uint32_t now_ms = millis();

  bool mode2_complete = false;

  if (this->operating_mode_ ==
          RadarOperatingMode::HEX_MULTI_TARGET &&
      this->multitarget_debug_started_ &&
      this->runtime_config_state_ ==
          RuntimeConfigState::IDLE_HEX) {
    mode2_complete = this->mode2_hex_parser_.feed(byte);
  }

  if (mode2_complete) {
    const uint8_t index = this->mode2_hex_parser_.completed_index();
    const Mode2Target &decoded = this->mode2_hex_parser_.target(index);

    Measurement measurement{};
    measurement.distance_m = decoded.distance_m;
    measurement.speed_kmh = decoded.speed_kmh;
    measurement.timestamp_ms = now_ms;

    if (this->is_artifact_measurement_(measurement)) {
      this->target_points_[index].valid = false;

      ESP_LOGD(TAG,
              "MODE2-HEX ghost filtered in slot %u: %.1f m / %.1f km/h / SNR %.1f",
              static_cast<unsigned>(index),
              decoded.distance_m,
              decoded.speed_kmh,
              decoded.snr);

      return;
    }

    TargetPoint &point = this->target_points_[index];

    if (!point.valid) {
      this->target_list_count_++;
    }

    point.valid = true;
    point.id = decoded.id;
    point.distance_m = decoded.distance_m;
    point.speed_kmh = decoded.speed_kmh;
    point.snr = decoded.snr;
    point.last_seen_ms = now_ms;

    this->target_list_receiving_ = true;
    this->target_list_last_row_ms_ = now_ms;

    ESP_LOGD(TAG,
            "MODE2-HEX target slot %u: id=%u, distance=%.1f m, speed=%.1f km/h, snr=%.1f",
            static_cast<unsigned>(index),
            static_cast<unsigned>(point.id),
            point.distance_m,
            point.speed_kmh,
            point.snr);

    this->pending_target_slots_[index] = true;

    if (!this->pending_target_batch_active_) {
      this->pending_target_batch_active_ = true;
      this->pending_target_batch_started_ms_ = now_ms;
    }

    this->pending_target_batch_last_ms_ = now_ms;

    this->last_target_ms_ = now_ms;
  }

  if (this->operating_mode_ ==
          RadarOperatingMode::HEX_MULTI_TARGET &&
      this->multitarget_debug_started_ &&
      this->runtime_config_state_ ==
          RuntimeConfigState::IDLE_HEX) {
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
      ESP_LOGD(TAG, "MODE2 RAW HEX: %s", payload.c_str());
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
    ESP_LOGI(TAG, "ASCII RAW FRAME: %s", escaped_frame.c_str());
    this->publish_multitarget_raw_mqtt_(frame, false);
  }
  if (frame.size() < 7 || frame.rfind("HEADA", 0) != 0) return;
  const uint8_t count = static_cast<uint8_t>(frame[5] - '0');
  const uint8_t expected = xor_checksum_(frame.substr(0, frame.size() - 1));
  const uint8_t received = static_cast<uint8_t>(frame.back());
  if (expected != received) {
    this->multi_target_checksum_errors_++;
    ESP_LOGW(TAG, "Multi-target checksum error: frame='%s' expected 0x%02X received 0x%02X",
             frame.c_str(), expected, received);
    return;
  }

  this->multi_target_valid_frames_++;

  // A valid HEADA frame is authoritative evidence that the radar currently
  // emits mode-0 data, even if the mode was persisted across a reboot.
  if (!this->multi_target_stream_mode_) {
    this->multi_target_stream_mode_ = true;
    ESP_LOGI(TAG, "Detected active HEADA stream; assigning detection ownership to mode 0");
    if (this->target_mode_status_sensor_ != nullptr)
      this->target_mode_status_sensor_->publish_state("Modus 0 automatisch anhand HEADA erkannt");
  }

  // Empty frames are common between positive detections. Do not immediately
  // clear Home Assistant state; loop() performs that transition after the hold time.
  if (count == 0) {
    this->multi_target_empty_frames_++;
    if (this->target_list_count_ == 0) {
      for (auto &point : this->target_points_) point.valid = false;
      this->target_list_receiving_ = true;
      this->target_list_manual_request_ = false;
      this->publish_detection_(false);
      this->publish_target_snapshot_(now_ms);
    }
    if (this->debug_uart_) {
      ESP_LOGD(TAG, "Multi-target frame #%" PRIu32 ": 0 target(s), checksum OK%s: %s",
               this->multi_target_valid_frames_, this->target_list_count_ > 0 ? " (held)" : "", frame.c_str());
    }
    return;
  }

  // Phase 6.7 deliberately treats frames with more than one payload field as
  // semantically unconfirmed. The manufacturer calls them target distances,
  // but field captures from a single-vehicle test produced isolated type-2
  // frames. Preserve the raw values without presenting them as two vehicles.
  if (count > 1) {
    std::string snapshot = "{\"type\":" + std::to_string(count) +
                           ",\"interpretation\":\"unconfirmed\",\"fields\":[";
    bool first = true;
    for (uint8_t i = 0; i < count; i++) {
      const size_t offset = 6U + static_cast<size_t>(i) * 4U;
      const std::string field = frame.substr(offset, 4);
      if (!first) snapshot += ",";
      snapshot += "\"" + field + "\"";
      first = false;
    }
    snapshot += "],\"raw\":\"" + escaped_frame + "\"}";

    for (auto &point : this->target_points_) point.valid = false;
    this->multi_target_target_frames_++;
    this->target_list_receiving_ = false;
    this->target_list_manual_request_ = false;
    this->multi_target_candidate_frames_ = 0;
    this->multi_target_candidate_confirmed_ = false;

    if (this->target_count_sensor_ != nullptr) {
      this->target_count_sensor_->publish_state(NAN);
      this->last_published_target_count_ = 255;
    }
    if (this->multi_target_active_sensor_ != nullptr &&
        (!this->multi_active_has_state_ || this->last_published_multi_active_)) {
      this->multi_target_active_sensor_->publish_state(false);
      this->last_published_multi_active_ = false;
      this->multi_active_has_state_ = true;
    }
    if (this->multi_target_snapshot_sensor_ != nullptr && snapshot != this->last_published_snapshot_) {
      this->multi_target_snapshot_sensor_->publish_state(snapshot);
      this->last_published_snapshot_ = snapshot;
    }
    const std::string status = std::to_string(count) +
                               " Datenfelder empfangen – Bedeutung unbestätigt";
    if (this->multi_target_status_sensor_ != nullptr && status != this->last_multi_target_status_) {
      this->multi_target_status_sensor_->publish_state(status);
      this->last_multi_target_status_ = status;
    }
    ESP_LOGW(TAG, "Unconfirmed HEADA type-%u frame preserved as raw fields: %s",
             static_cast<unsigned>(count), frame.c_str());
    return;
  }

  for (auto &point : this->target_points_) point.valid = false;
  const size_t offset = 6U;
  unsigned distance = 0;
  if (std::sscanf(frame.c_str() + offset, "%4u", &distance) != 1) return;
  TargetPoint &point = this->target_points_[0];
  point.valid = true;
  point.id = 0;
  point.distance_m = static_cast<float>(distance);
  point.speed_kmh = NAN;
  point.snr = NAN;

  this->multi_target_target_frames_++;
  const float distance_m = static_cast<float>(distance);

  // Once confirmed, every subsequent positive frame updates and refreshes the
  // same target. Do not start a second candidate state machine while active.
  if (this->multi_target_candidate_confirmed_) {
    this->multi_target_candidate_last_ms_ = now_ms;
    this->multi_target_candidate_last_distance_ = distance_m;
    this->multi_target_last_seen_ms_ = now_ms;
    this->target_list_count_ = 1;
    this->target_list_receiving_ = true;
    this->target_list_manual_request_ = false;
    this->last_target_ms_ = now_ms;
    this->publish_detection_(true);
    this->publish_target_snapshot_(now_ms);
  } else {
    const bool candidate_continues = this->multi_target_candidate_frames_ > 0 &&
        static_cast<uint32_t>(now_ms - this->multi_target_candidate_first_ms_) <= this->multi_target_confirmation_window_ms_ &&
        static_cast<uint32_t>(now_ms - this->multi_target_candidate_last_ms_) <= this->multi_target_confirmation_window_ms_ &&
        std::fabs(distance_m - this->multi_target_candidate_last_distance_) <= this->multi_target_confirmation_tolerance_m_;

    if (!candidate_continues) {
      this->multi_target_candidate_frames_ = 1;
      this->multi_target_candidate_first_ms_ = now_ms;
      this->multi_target_candidate_first_distance_ = distance_m;
    } else if (this->multi_target_candidate_frames_ < 255) {
      this->multi_target_candidate_frames_++;
    }
    this->multi_target_candidate_last_ms_ = now_ms;
    this->multi_target_candidate_last_distance_ = distance_m;

    const float movement_m = std::fabs(distance_m - this->multi_target_candidate_first_distance_);
    if (this->multi_target_candidate_frames_ >= this->multi_target_confirmation_frames_ &&
        movement_m >= this->multi_target_min_confirmation_movement_m_) {
      this->multi_target_candidate_confirmed_ = true;
      this->multi_target_last_seen_ms_ = now_ms;
      this->target_list_count_ = 1;
      this->target_list_receiving_ = true;
      this->target_list_manual_request_ = false;
      this->last_target_ms_ = now_ms;
      ESP_LOGI(TAG, "HEADA moving target confirmed after %u frame(s), movement %.1f m, current %.1f m",
               static_cast<unsigned>(this->multi_target_candidate_frames_), movement_m, distance_m);
      this->publish_detection_(true);
      this->publish_target_snapshot_(now_ms);
    } else if (this->debug_uart_) {
      ESP_LOGD(TAG, "HEADA candidate %u/%u at %.1f m, movement %.1f/%.1f m (not published)",
               static_cast<unsigned>(this->multi_target_candidate_frames_),
               static_cast<unsigned>(this->multi_target_confirmation_frames_), distance_m,
               movement_m, this->multi_target_min_confirmation_movement_m_);
    }
  }

  if (this->debug_uart_) {
    ESP_LOGD(TAG, "Multi-target frame #%" PRIu32 ": one distance field, checksum OK: %s",
             this->multi_target_valid_frames_, frame.c_str());
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
    ESP_LOGI(TAG, "ASCII RAW LINE: %s", escaped_line.c_str());
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
    if (this->operating_mode_ == RadarOperatingMode::ASCII_SINGLE_TARGET) {

        const bool was_tracking = this->vehicle_tracker_.tracking();

        this->vehicle_tracker_.add_measurement(measurement);

        if (!was_tracking) {
            if (this->vehicle_tracking_sensor_ != nullptr)
                this->vehicle_tracking_sensor_->publish_state(true);

            if (this->vehicle_id_sensor_ != nullptr)
                this->vehicle_id_sensor_->publish_state(
                    this->vehicle_tracker_.current_id());
        }

        if (this->vehicle_direction_sensor_ != nullptr) {
            this->vehicle_direction_sensor_->publish_state(
                vehicle_direction_to_string(
                    this->vehicle_tracker_.current_direction()));
        }
    }

    this->last_target_ms_ = measurement.timestamp_ms;
    this->publish_detection_(true);
    this->update_driveway_traffic_state_(now_ms);
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

uint8_t LDL508PROComponent::count_visible_multi_target_tracks_(
    uint32_t now_ms) const {
  uint8_t count = 0;

  for (const auto &track : this->multi_target_tracks_) {
    if (!track.active)
      continue;

    const uint32_t age_ms =
        static_cast<uint32_t>(now_ms - track.last_seen_ms);

    if (age_ms <= 1000)
      count++;
  }

  return count;
}

const LDL508PROComponent::MultiTargetTrack *
LDL508PROComponent::find_primary_multi_target_track_(
    uint32_t now_ms) const {
  const MultiTargetTrack *primary_track = nullptr;

  for (const auto &track : this->multi_target_tracks_) {
    if (!track.active)
      continue;

    const uint32_t age_ms =
        static_cast<uint32_t>(now_ms - track.last_seen_ms);

    if (age_ms > 1000)
      continue;

    if (primary_track == nullptr ||
        track.distance_m < primary_track->distance_m) {
      primary_track = &track;
    }
  }

  return primary_track;
}

void LDL508PROComponent::update_driveway_traffic_state_(
    uint32_t now_ms) {

  bool approaching_traffic = false;

  // ----------------------------------------------------------
  // Mehrzielmodus:
  // JEDER aktuell sichtbare Track wird betrachtet.
  // Ein einziges annäherndes Fahrzeug reicht für Warnung.
  // ----------------------------------------------------------
  if (this->operating_mode_ ==
      RadarOperatingMode::HEX_MULTI_TARGET) {

    for (const auto &track :
         this->multi_target_tracks_) {

      if (!track.active) {
        continue;
      }

      // Negative Geschwindigkeit = Annähernd.
      if (track.speed_kmh < -0.5f) {
        approaching_traffic = true;
        break;
      }
    }

  } else {

    // --------------------------------------------------------
    // Einzelzielmodus:
    // Die aktuelle Richtung stammt aus dem VehicleTracker.
    // --------------------------------------------------------
    if (this->target_detected_) {

      approaching_traffic =
          this->vehicle_tracker_.current_direction() ==
          VehicleDirection::APPROACHING;
    }
  }

  this->driveway_controller_.set_traffic_warning(
      approaching_traffic,
      now_ms);
}

void LDL508PROComponent::publish_target_snapshot_(uint32_t now_ms) {

  for (uint8_t slot = 0; slot < this->target_points_.size(); slot++) {
    auto &point = this->target_points_[slot];
    if (!point.valid)
      continue;

    if (now_ms - point.last_seen_ms > 750) {
      ESP_LOGD(TAG,
         "MODE2-HEX slot %u expired (id=%u, distance=%.1f m, speed=%.1f km/h, snr=%.1f)",
         static_cast<unsigned>(slot),
         static_cast<unsigned>(point.id),
         point.distance_m,
         point.speed_kmh,
         point.snr);
      point.valid = false;
    }
  }

  if (!this->target_list_receiving_) return;
  this->target_list_receiving_ = false;

  uint8_t count = 0;

  for (const auto &point : this->target_points_) {
    if (point.valid)
      count++;
  }

  this->target_list_count_ = count;

  const uint8_t visible_tracks =
    this->count_visible_multi_target_tracks_(now_ms);

  //const MultiTargetTrack *primary_track =
  //  this->find_primary_multi_target_track_(now_ms);

  const MultiTargetTrack *primary_track = nullptr;

  // 1. Bisherigen Primärtrack suchen
  if (this->primary_vehicle_track_id_ != 0) {
    for (const auto &track : this->multi_target_tracks_) {
      if (!track.active)
        continue;

      const uint32_t age_ms =
          static_cast<uint32_t>(now_ms - track.last_seen_ms);

      if (age_ms > 1000)
        continue;

      if (track.id == this->primary_vehicle_track_id_) {
        primary_track = &track;
        break;
      }
    }
  }

  // 2. Falls der bisherige Primärtrack verschwunden ist,
  //    neuen auswählen.
  if (primary_track == nullptr) {

    primary_track =
        this->find_primary_multi_target_track_(now_ms);

    if (primary_track != nullptr)
      this->primary_vehicle_track_id_ =
          primary_track->id;
    else
      this->primary_vehicle_track_id_ = 0;
  }

  // 3. Live-Entitäten direkt aus dem stabilen Primärtrack versorgen.
  if (primary_track != nullptr) {

    if (this->distance_sensor_ != nullptr) {
      this->distance_sensor_->publish_state(
          primary_track->distance_m);
    }

    if (this->speed_sensor_ != nullptr) {
      this->speed_sensor_->publish_state(
          primary_track->speed_kmh);
    }

    if (this->vehicle_tracking_sensor_ != nullptr) {
      this->vehicle_tracking_sensor_->publish_state(true);
    }

    if (this->vehicle_id_sensor_ != nullptr) {
      this->vehicle_id_sensor_->publish_state(
          primary_track->id);
    }

    if (this->vehicle_direction_sensor_ != nullptr) {
      const char *direction = "Unbekannt";

      if (primary_track->speed_kmh > 0.5f) {
        direction = "Entfernend";
      } else if (primary_track->speed_kmh < -0.5f) {
        direction = "Annähernd";
      }

      this->vehicle_direction_sensor_->publish_state(direction);
    }
  }


  const bool detected_from_slots = count > 0;
  const bool detected_from_tracks = visible_tracks > 0;

  if (detected_from_slots != detected_from_tracks) {
    ESP_LOGW(
        TAG,
        "Detection mismatch: slots=%u tracks=%u",
        static_cast<unsigned>(count),
        static_cast<unsigned>(visible_tracks));
  }

  const bool detected = detected_from_tracks;

  if (detected != this->target_detected_) {
      this->publish_detection_(detected);
  }

  this->update_driveway_traffic_state_(now_ms);

  if (count > this->max_simultaneous_targets_) this->max_simultaneous_targets_ = count;

  if (this->target_count_sensor_ != nullptr &&
      this->last_published_target_count_ != visible_tracks) {
    this->target_count_sensor_->publish_state(visible_tracks);
    this->last_published_target_count_ = visible_tracks;
  }

  if (this->max_simultaneous_targets_sensor_ != nullptr &&
      this->last_published_max_targets_ != this->max_simultaneous_targets_) {
    this->max_simultaneous_targets_sensor_->publish_state(this->max_simultaneous_targets_);
    this->last_published_max_targets_ = this->max_simultaneous_targets_;
  }

  //const bool multiple_active = count > 1;
  const bool multiple_active = visible_tracks > 1;

  if (this->multi_target_active_sensor_ != nullptr &&
      (!this->multi_active_has_state_ || this->last_published_multi_active_ != multiple_active)) {
    this->multi_target_active_sensor_->publish_state(multiple_active);
    this->last_published_multi_active_ = multiple_active;
    this->multi_active_has_state_ = true;
  }

  std::string snapshot = "{\"count\":" + std::to_string(count) + ",\"targets\":[";
  bool first = true;
  char item[80];

  for (uint8_t slot = 0; slot < this->target_points_.size(); slot++) {
  const auto &point = this->target_points_[slot];
  if (!point.valid) continue;

  if (std::isfinite(point.speed_kmh) && std::isfinite(point.snr)) {
    std::snprintf(
        item, sizeof(item),
        "%s{\"slot\":%u,\"track\":%u,\"id\":%u,\"r\":%.1f,\"v\":%.1f,\"snr\":%.1f}",
        first ? "" : ",",
        static_cast<unsigned>(slot),
        static_cast<unsigned>(point.track_id),
        static_cast<unsigned>(point.id),
        point.distance_m,
        point.speed_kmh,
        point.snr);
  } else {
    std::snprintf(
        item, sizeof(item),
        "%s{\"slot\":%u,\"id\":%u,\"r\":%.1f}",
        first ? "" : ",",
        static_cast<unsigned>(slot),
        static_cast<unsigned>(point.id),
        point.distance_m);
  }

  snapshot += item;
  first = false;
}
  snapshot += "]}";

std::string track_snapshot = "{\"track_count\":";

const uint8_t active_tracks =
    this->count_visible_multi_target_tracks_(now_ms);

track_snapshot += std::to_string(active_tracks);
track_snapshot += ",\"tracks\":[";

bool first_track = true;
char track_item[96];

for (const auto &track : this->multi_target_tracks_) {
  const uint32_t track_age_ms =
      static_cast<uint32_t>(now_ms - track.last_seen_ms);

  if (!track.active || track_age_ms > 1000)
    continue;

  std::snprintf(
      track_item,
      sizeof(track_item),
      "%s{\"track\":%u,\"r\":%.1f,\"v\":%.1f,\"snr\":%.1f}",
      first_track ? "" : ",",
      static_cast<unsigned>(track.id),
      track.distance_m,
      track.speed_kmh,
      track.snr);

  track_snapshot += track_item;
  first_track = false;
}

track_snapshot += "]}";

ESP_LOGI(TAG, "TRACK JSON: %s", track_snapshot.c_str());

  this->publish_multitarget_parsed_mqtt_(snapshot);
  ESP_LOGD(TAG, "MODE2 SLOT JSON: %s", snapshot.c_str());
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

void LDL508PROComponent::publish_completed_track_(
    const MultiTargetTrack &track,
    uint32_t now_ms) {

  const float average_speed =
      track.sample_count > 0
          ? track.speed_sum_kmh / track.sample_count
          : 0.0f;

  const uint32_t duration_ms =
      static_cast<uint32_t>(
          track.last_seen_ms - track.first_seen_ms);

  const float duration_s =
      duration_ms / 1000.0f;

  const float travel_distance_m =
    std::fabs(
        track.end_distance_m -
        track.start_distance_m);

  const float samples_per_second =
      duration_s > 0.0f
          ? static_cast<float>(track.sample_count) / duration_s
          : static_cast<float>(track.sample_count);

  const float meters_per_sample =
      track.sample_count > 1
          ? travel_distance_m /
                static_cast<float>(track.sample_count - 1)
          : 0.0f;   
          
  const char *confidence = "high";

  if (track.sample_count <= 1) {
    confidence = "low";

  } else if (
      track.sample_count <= 3 &&
      travel_distance_m < 5.0f) {
    confidence = "low";

  } else if (
      track.sample_count <= 5 ||
      travel_distance_m < 10.0f) {
    confidence = "medium";
  }

  if (this->vehicle_start_distance_sensor_ != nullptr) {
    this->vehicle_start_distance_sensor_->publish_state(
        track.start_distance_m);
  }

  if (this->vehicle_end_distance_sensor_ != nullptr) {
    this->vehicle_end_distance_sensor_->publish_state(
        track.end_distance_m);
  }

  if (this->vehicle_min_distance_sensor_ != nullptr) {
    this->vehicle_min_distance_sensor_->publish_state(
        track.minimum_distance_m);
  }

  if (this->vehicle_average_speed_sensor_ != nullptr) {
    this->vehicle_average_speed_sensor_->publish_state(
        average_speed);
  }

  if (this->vehicle_max_speed_sensor_ != nullptr) {
    this->vehicle_max_speed_sensor_->publish_state(
        track.maximum_speed_kmh);
  }

  if (this->vehicle_duration_sensor_ != nullptr) {
    this->vehicle_duration_sensor_->publish_state(
        duration_s);
  }

  if (this->vehicle_samples_sensor_ != nullptr) {
    this->vehicle_samples_sensor_->publish_state(
        track.sample_count);
  }

  this->completed_vehicle_count_++;

  if (this->vehicle_count_sensor_ != nullptr) {
    this->vehicle_count_sensor_->publish_state(
        this->completed_vehicle_count_);
  }

  const char *direction = "Unbekannt";

  if (track.speed_kmh > 0.5f) {
    direction = "Entfernend";
  } else if (track.speed_kmh < -0.5f) {
    direction = "Annähernd";
  }

  const bool multi_target_mode =
    this->operating_mode_ ==
        RadarOperatingMode::HEX_MULTI_TARGET;

  const char *mode =
      multi_target_mode
          ? "multi-target"
          : "single-target";

  const char *mode_label =
      multi_target_mode
          ? "Mehrziel"
          : "Einzelziel";

  const char *protocol =
      multi_target_mode
          ? "hex"
          : "ascii";

  //char json[512];
  //char json[640];
  char json[768];

  std::snprintf(
      json,
      sizeof(json),
      "{\"id\":%u,"
      "\"direction\":\"%s\","
      "\"mode\":\"%s\","
      "\"mode_label\":\"%s\","
      "\"protocol\":\"%s\","
      "\"start_distance_m\":%.1f,"
      "\"end_distance_m\":%.1f,"
      "\"minimum_distance_m\":%.1f,"
      "\"max_speed_kmh\":%.1f,"
      "\"average_speed_kmh\":%.1f,"
      "\"duration_s\":%.1f,"
      "\"duration_ms\":%" PRIu32 ","
      "\"first_seen_ms\":%" PRIu32 ","
      "\"last_seen_ms\":%" PRIu32 ","
      "\"max_targets\":%u,"
      "\"samples\":%u,"
      "\"travel_distance_m\":%.1f,"
      "\"samples_per_second\":%.2f,"
      "\"meters_per_sample\":%.2f,"
      "\"confidence\":\"%s\","
      "\"ghosts_filtered\":%" PRIu32 ","
      "\"firmware\":\"%s\"}",
      static_cast<unsigned>(track.id),
      direction,
      mode,
      mode_label,
      protocol,
      track.start_distance_m,
      track.end_distance_m,
      track.minimum_distance_m,
      track.maximum_speed_kmh,
      average_speed,
      duration_s,
      duration_ms,
      track.first_seen_ms,
      track.last_seen_ms,
      static_cast<unsigned>(track.max_targets_seen),
      static_cast<unsigned>(track.sample_count),
      travel_distance_m,
      samples_per_second,
      meters_per_sample,
      confidence,
      this->artifact_filtered_count_,
      FIRMWARE_VERSION);

  if (this->last_vehicle_event_sensor_ != nullptr) {
    char summary[160];

    std::snprintf(
        summary,
        sizeof(summary),
        "#%u | %s | %s | Ø %.1f km/h | %.1f m | %s | max_targets=%u",
        static_cast<unsigned>(track.id),
        mode_label,
        direction,
        average_speed,
        travel_distance_m,
        confidence,
        static_cast<unsigned>(track.max_targets_seen));

    this->last_vehicle_event_sensor_->publish_state(summary);
  }

  this->publish_vehicle_event_mqtt_(json);

  ESP_LOGI(
      TAG,
      "VEHICLE COMPLETE confidence=%s mode=%s protocol=%s "
      "id=%u samples=%u travel=%.1f m "
      "start=%.1f end=%.1f min=%.1f avg=%.1f max=%.1f "
      "duration=%.1f samples_per_s=%.2f",
      confidence,
      mode,
      protocol,
      static_cast<unsigned>(track.id),
      static_cast<unsigned>(track.sample_count),
      travel_distance_m,
      track.start_distance_m,
      track.end_distance_m,
      track.minimum_distance_m,
      average_speed,
      track.maximum_speed_kmh,
      duration_s,
      samples_per_second);

    (void) now_ms;
}

void LDL508PROComponent::publish_vehicle_tracker_event_(
    const VehicleEvent &event) {

  MultiTargetTrack track{};

  track.id = static_cast<uint16_t>(event.id);

  track.start_distance_m = event.first_distance_m;
  track.end_distance_m = event.last_distance_m;
  track.minimum_distance_m = event.minimum_distance_m;

  track.maximum_speed_kmh = event.max_speed_kmh;
  track.sample_count =
      static_cast<uint16_t>(event.sample_count);

  // publish_completed_track_ berechnet den Durchschnitt aus
  // Geschwindigkeitssumme / Messwertanzahl.
  track.speed_sum_kmh =
      event.average_speed_kmh *
      static_cast<float>(event.sample_count);

  // publish_completed_track_ berechnet die Dauer aus diesen Zeitstempeln.
  //track.first_seen_ms = 0;
  //track.last_seen_ms = event.duration_ms;

  const uint32_t completed_at_ms = millis();

  track.last_seen_ms = completed_at_ms;
  track.first_seen_ms =
      static_cast<uint32_t>(
          completed_at_ms - event.duration_ms);

  track.max_targets_seen = 1;

  // Vorzeichen nur zur Richtungsbestimmung im gemeinsamen Publisher.
  track.speed_kmh = event.average_speed_kmh;

  if (std::strcmp(
          vehicle_direction_to_string(event.direction),
          "Annähernd") == 0) {
    track.speed_kmh = -track.speed_kmh;
  }

  // Livezustand des abgeschlossenen ASCII-Fahrzeugs beenden.
  if (this->vehicle_tracking_sensor_ != nullptr) {
    this->vehicle_tracking_sensor_->publish_state(false);
  }

  if (this->vehicle_direction_sensor_ != nullptr) {
    this->vehicle_direction_sensor_->publish_state(
        vehicle_direction_to_string(event.direction));
  }

  if (this->vehicle_id_sensor_ != nullptr) {
    this->vehicle_id_sensor_->publish_state(event.id);
  }

  //this->publish_completed_track_(track, millis());

  this->publish_completed_track_(
    track,
    completed_at_ms);
}

void LDL508PROComponent::update_carport_sensors_(
    uint32_t now_ms) {

  if (!this->carport_presence_.initialized()) {
    return;
  }

  const bool beam_clear =
      this->carport_presence_.beam_clear();

  const bool occupied =
      this->carport_presence_.occupied();

  // ----------------------------------------------------------
  // Aktueller Zustand der Lichtschranke
  // ----------------------------------------------------------
  if (this->carport_beam_clear_sensor_ != nullptr &&
      (!this->carport_beam_clear_has_state_ ||
       beam_clear != this->last_carport_beam_clear_)) {

    this->carport_beam_clear_sensor_->publish_state(
        beam_clear);

    this->last_carport_beam_clear_ = beam_clear;
    this->carport_beam_clear_has_state_ = true;
  }

  // ----------------------------------------------------------
  // Belegungszustand des Carports
  // ----------------------------------------------------------
  if (this->carport_occupied_sensor_ != nullptr &&
      (!this->carport_occupied_has_state_ ||
       occupied != this->last_carport_occupied_)) {

    this->carport_occupied_sensor_->publish_state(
        occupied);

    this->last_carport_occupied_ = occupied;
    this->carport_occupied_has_state_ = true;
  }

  // ----------------------------------------------------------
  // Bestätigte Ausfahrt als kurzer Ereignisimpuls
  // ----------------------------------------------------------
  if (this->carport_presence_.departure_confirmed()) {

    this->carport_presence_.clear_departure_event();

    this->driveway_controller_.trigger_departure(now_ms);

    if (this->carport_departure_sensor_ != nullptr) {
      this->carport_departure_sensor_->publish_state(true);
    }

    this->carport_departure_pulse_active_ = true;
    this->carport_departure_pulse_started_ms_ = now_ms;

    ESP_LOGI(
        TAG,
        "Carport departure event published");
  }

  // Ereignissensor nach 3 Sekunden wieder zurücksetzen.
  if (this->carport_departure_pulse_active_ &&
      static_cast<uint32_t>(
          now_ms -
          this->carport_departure_pulse_started_ms_) >=
          CARPORT_DEPARTURE_PULSE_MS) {

    this->carport_departure_pulse_active_ = false;

    if (this->carport_departure_sensor_ != nullptr) {
      this->carport_departure_sensor_->publish_state(false);
    }
  }
}

void LDL508PROComponent::begin_runtime_ascii_mode_() {
  if (this->runtime_config_state_ !=
      RuntimeConfigState::IDLE_HEX) {
    return;
  }

  ESP_LOGI(TAG, "CONFIG: switching HEX -> ASCII");

  // Einen eventuell begonnenen HEX-Batch verwerfen.
  this->pending_target_batch_active_ = false;
  this->pending_target_batch_started_ms_ = 0;
  this->pending_target_batch_last_ms_ = 0;
  this->pending_target_slots_.fill(false);

  this->target_list_receiving_ = false;
  this->multitarget_hex_mqtt_block_length_ = 0;

  this->rx_buffer_.clear();
  this->ingress_buffer_.clear();
  this->discard_until_newline_ = false;

  this->runtime_config_command_queued_ = false;
  this->runtime_config_state_ =
      RuntimeConfigState::SWITCHING_TO_ASCII;
  this->runtime_config_state_started_ms_ = millis();

  const uint8_t switch_to_ascii[] = {
      0xAA, 0xAA, 0x77, 0x77, 0x55, 0x55
  };

  this->write_array(
      switch_to_ascii,
      sizeof(switch_to_ascii));
}

void LDL508PROComponent::restore_runtime_hex_mode_() {
  ESP_LOGI(TAG, "CONFIG: switching ASCII -> HEX protocol");

  this->runtime_config_state_ =
      RuntimeConfigState::SWITCHING_TO_HEX;

  this->runtime_config_state_started_ms_ = millis();

  this->runtime_config_command_queued_ = false;
  this->runtime_config_write_pending_ = false;

  this->rx_buffer_.clear();
  this->ingress_buffer_.clear();
  this->multi_target_stream_buffer_.clear();
  this->discard_until_newline_ = false;
  this->multitarget_hex_mqtt_block_length_ = 0;

  // Zuerst nur das Kommunikationsprotokoll auf HEX umstellen.
  const uint8_t switch_to_hex_protocol[] = {
      0xAA, 0xAA, 0x66, 0x66, 0x55, 0x55
  };

  this->write_array(
      switch_to_hex_protocol,
      sizeof(switch_to_hex_protocol));
}

void LDL508PROComponent::queue_runtime_config_write_(
    RadarParameter parameter,
    float value) {

  if (this->runtime_config_write_pending_ ||
      this->operating_mode_change_pending_ ||
      this->config_manager_.is_busy()) {
    this->config_error(
        "Eine andere Konfigurationsänderung läuft bereits");
    return;
  }

  this->runtime_config_parameter_ = parameter;
  this->runtime_config_value_ = value;
  this->runtime_config_write_pending_ = true;
  this->runtime_config_command_queued_ = false;

  // Im ASCII-Einzelzielmodus ist kein Protokollwechsel nötig.
  if (this->operating_mode_ ==
          RadarOperatingMode::ASCII_SINGLE_TARGET &&
      this->runtime_config_state_ ==
          RuntimeConfigState::IDLE_ASCII) {

    ESP_LOGI(TAG, "CONFIG: runtime write in active ASCII mode");

    this->runtime_config_state_ =
        RuntimeConfigState::RUNNING_ASCII;
    this->runtime_config_state_started_ms_ = millis();

    this->config_manager_.queue_write(
        this->runtime_config_parameter_,
        this->runtime_config_value_);

    this->runtime_config_command_queued_ = true;
    return;
  }

  // Im HEX-Mehrzielmodus weiterhin kurz zu ASCII wechseln.
  if (this->operating_mode_ ==
          RadarOperatingMode::HEX_MULTI_TARGET &&
      this->runtime_config_state_ ==
          RuntimeConfigState::IDLE_HEX) {

    this->begin_runtime_ascii_mode_();
    return;
  }

  this->runtime_config_write_pending_ = false;

  this->config_error(
      "Konfigurationsänderung in ungültigem Betriebszustand");
}

void LDL508PROComponent::flush_multi_target_batch_(uint32_t now_ms) {
  this->pending_target_batch_active_ = false;
  this->pending_target_batch_started_ms_ = 0;
  this->pending_target_batch_last_ms_ = 0;

  // Zuordnung:
  // Index = Radar-Slot
  // Wert  = Index in multi_target_tracks_, -1 bedeutet noch nicht zugeordnet
  std::array<int8_t, 9> assigned_track{};
  assigned_track.fill(-1);

  // Ein bestehender Track darf innerhalb dieses Batches nur einmal benutzt werden.
  std::array<bool, 9> track_used{};
  track_used.fill(false);

  // Ein Radar-Slot kann eine sehr nahe Doppelreflexion eines anderen Slots sein.
  // Wert -1: eigenständiger Messpunkt
  // Sonst: Index des führenden Radar-Slots
  std::array<int8_t, 9> duplicate_of_slot{};
  duplicate_of_slot.fill(-1);

  // ------------------------------------------------------------
  // 0. Sehr nahe Doppelreflexionen innerhalb des Batches erkennen
  // ------------------------------------------------------------
  for (uint8_t slot_a = 0;
      slot_a < this->pending_target_slots_.size();
      slot_a++) {

    if (!this->pending_target_slots_[slot_a])
      continue;

    const TargetPoint &point_a = this->target_points_[slot_a];

    if (!point_a.valid)
      continue;

    for (uint8_t slot_b = slot_a + 1;
        slot_b < this->pending_target_slots_.size();
        slot_b++) {

      if (!this->pending_target_slots_[slot_b])
        continue;

      const TargetPoint &point_b = this->target_points_[slot_b];

      if (!point_b.valid)
        continue;

      const bool direction_a_positive = point_a.speed_kmh >= 0.0f;
      const bool direction_b_positive = point_b.speed_kmh >= 0.0f;

      if (direction_a_positive != direction_b_positive)
        continue;

      const float distance_difference_m =
          std::fabs(point_a.distance_m - point_b.distance_m);

      const float speed_difference_kmh =
          std::fabs(point_a.speed_kmh - point_b.speed_kmh);

      // Bewusst enge Grenzen:
      // echte hintereinander fahrende Fahrzeuge sollen getrennt bleiben.
      if (distance_difference_m <= 2.0f &&
          speed_difference_kmh <= 1.5f) {

        duplicate_of_slot[slot_b] =
            static_cast<int8_t>(slot_a);

        ESP_LOGI(
            TAG,
            "TRACK-BATCH radar slot %u is duplicate of slot %u "
            "(distance delta %.2f m, speed delta %.2f km/h)",
            static_cast<unsigned>(slot_b),
            static_cast<unsigned>(slot_a),
            distance_difference_m,
            speed_difference_kmh);
      }
    }
  }


  // Anzahl eigenständiger Ziele in diesem Batch.
  // Als Doppelreflexion erkannte Slots werden nicht zusätzlich gezählt.
  uint8_t batch_target_count = 0;

  for (uint8_t radar_slot = 0;
      radar_slot < this->pending_target_slots_.size();
      radar_slot++) {

    if (!this->pending_target_slots_[radar_slot])
      continue;

    if (duplicate_of_slot[radar_slot] >= 0)
      continue;

    if (!this->target_points_[radar_slot].valid)
      continue;

    batch_target_count++;
  }

  // ------------------------------------------------------------
  // 1. Abgelaufene Tracks einmal zentral freigeben
  // ------------------------------------------------------------
  for (uint8_t track_index = 0;
       track_index < this->multi_target_tracks_.size();
       track_index++) {
    auto &track = this->multi_target_tracks_[track_index];

    if (!track.active)
      continue;

    const uint32_t age_ms =
        static_cast<uint32_t>(now_ms - track.last_seen_ms);

    if (age_ms > 3000) {

      this->publish_completed_track_(track, now_ms);

      ESP_LOGI(TAG,
              "TRACK-MGR expired track %u after %" PRIu32 " ms",
              static_cast<unsigned>(track.id),
              age_ms);

      track.active = false;
    }

  }

  // ------------------------------------------------------------
  // 2. Alle Radar-Slots gegen den unveränderten alten Trackzustand matchen
  // ------------------------------------------------------------
  for (uint8_t radar_slot = 0;
       radar_slot < this->pending_target_slots_.size();
       radar_slot++) {
    if (!this->pending_target_slots_[radar_slot])
      continue;

    if (duplicate_of_slot[radar_slot] >= 0)
    continue;

    const TargetPoint &point = this->target_points_[radar_slot];

    if (!point.valid)
      continue;

    int8_t best_track_index = -1;
    float best_score = INFINITY;

    for (uint8_t track_index = 0;
         track_index < this->multi_target_tracks_.size();
         track_index++) {
      const MultiTargetTrack &candidate =
          this->multi_target_tracks_[track_index];

      if (!candidate.active)
        continue;

      // Wichtig: Ein Track darf in diesem Batch nur einem Slot gehören.
      if (track_used[track_index])
        continue;

      const uint32_t age_ms =
          static_cast<uint32_t>(now_ms - candidate.last_seen_ms);

      if (age_ms > 3000)
        continue;

      const bool point_positive = point.speed_kmh >= 0.0f;
      const bool track_positive = candidate.speed_kmh >= 0.0f;

      if (point_positive != track_positive)
        continue;

      const float elapsed_s = age_ms / 1000.0f;

      const float predicted_distance_m =
          candidate.distance_m +
          (candidate.speed_kmh / 3.6f) * elapsed_s;

      const float distance_error_m =
          std::fabs(point.distance_m - predicted_distance_m);

      const float speed_error_kmh =
          std::fabs(point.speed_kmh - candidate.speed_kmh);

      if (distance_error_m > 8.0f)
        continue;

      if (speed_error_kmh > 8.0f)
        continue;

      const float score =
          distance_error_m + speed_error_kmh * 0.25f;

      if (score < best_score) {
        best_score = score;
        best_track_index = static_cast<int8_t>(track_index);
      }
    }

    if (best_track_index >= 0) {
      assigned_track[radar_slot] = best_track_index;
      track_used[best_track_index] = true;

      const auto &track =
          this->multi_target_tracks_[best_track_index];

      ESP_LOGD(TAG,
               "TRACK-BATCH matched radar slot %u to track %u, score=%.2f",
               static_cast<unsigned>(radar_slot),
               static_cast<unsigned>(track.id),
               best_score);
    }
  }

  // ------------------------------------------------------------
  // 3. Für noch nicht zugeordnete Radar-Slots neue Tracks reservieren
  // ------------------------------------------------------------
  for (uint8_t radar_slot = 0;
       radar_slot < this->pending_target_slots_.size();
       radar_slot++) {
    if (!this->pending_target_slots_[radar_slot])
      continue;

    if (duplicate_of_slot[radar_slot] >= 0)
    continue;

    if (assigned_track[radar_slot] >= 0)
      continue;

    const TargetPoint &point = this->target_points_[radar_slot];

    if (!point.valid)
      continue;

    for (uint8_t track_index = 0;
         track_index < this->multi_target_tracks_.size();
         track_index++) {
      auto &track = this->multi_target_tracks_[track_index];

      if (track.active || track_used[track_index])
        continue;

      track.active = true;
      track.id = this->next_multi_target_track_id_++;

      if (this->next_multi_target_track_id_ == 0)
        this->next_multi_target_track_id_ = 1;

      track.first_seen_ms = now_ms;
      track.last_seen_ms = now_ms;
      track.last_radar_slot = radar_slot;
      track.sample_count = 0;
      track.max_targets_seen = 1;

      track.start_distance_m = point.distance_m;
      track.end_distance_m = point.distance_m;
      track.minimum_distance_m = point.distance_m;
      track.maximum_speed_kmh = 0.0f;
      track.speed_sum_kmh = 0.0f;      

      assigned_track[radar_slot] =
          static_cast<int8_t>(track_index);

      track_used[track_index] = true;

      ESP_LOGI(TAG,
               "TRACK-BATCH created track %u from radar slot %u",
               static_cast<unsigned>(track.id),
               static_cast<unsigned>(radar_slot));

      break;
    }

    if (assigned_track[radar_slot] < 0) {
      ESP_LOGW(TAG,
               "TRACK-BATCH has no free track for radar slot %u",
               static_cast<unsigned>(radar_slot));
    }
  }

  // Doppelreflexionen erhalten dieselbe Track-Zuordnung wie ihr Hauptslot.
  for (uint8_t radar_slot = 0;
      radar_slot < duplicate_of_slot.size();
      radar_slot++) {

    const int8_t primary_slot = duplicate_of_slot[radar_slot];

    if (primary_slot < 0)
      continue;

    assigned_track[radar_slot] =
        assigned_track[static_cast<uint8_t>(primary_slot)];
  }

  // ------------------------------------------------------------
  // 4. Erst jetzt alle Tracks aktualisieren
  // ------------------------------------------------------------
  for (uint8_t radar_slot = 0;
       radar_slot < this->pending_target_slots_.size();
       radar_slot++) {
    if (!this->pending_target_slots_[radar_slot])
      continue;

    this->pending_target_slots_[radar_slot] = false;

    const int8_t track_index = assigned_track[radar_slot];

    if (track_index < 0)
      continue;

    TargetPoint &point = this->target_points_[radar_slot];
    MultiTargetTrack &track =
        this->multi_target_tracks_[track_index];

    point.track_id = track.id;

    // Ein erkannter Duplikat-Slot bekommt nur dieselbe Track-ID.
    // Die Trackmessung selbst wird ausschließlich vom Hauptslot aktualisiert.
    if (duplicate_of_slot[radar_slot] >= 0)
      continue;

    if (batch_target_count > track.max_targets_seen) {
      track.max_targets_seen = batch_target_count;
    }

    track.distance_m = point.distance_m;
    track.speed_kmh = point.speed_kmh;
    track.snr = point.snr;
    track.last_seen_ms = now_ms;
    track.last_radar_slot = radar_slot;

    track.end_distance_m = point.distance_m;

    if (point.distance_m < track.minimum_distance_m) {
      track.minimum_distance_m = point.distance_m;
    }

    const float absolute_speed_kmh =
        std::fabs(point.speed_kmh);

    if (absolute_speed_kmh > track.maximum_speed_kmh) {
      track.maximum_speed_kmh = absolute_speed_kmh;
    }

    track.speed_sum_kmh += absolute_speed_kmh;

    if (track.sample_count < UINT16_MAX) {
      track.sample_count++;
    }
  }  

  // Snapshot erst nach vollständiger Batch-Zuordnung erzeugen.
  this->publish_target_snapshot_(now_ms);
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
  if (mode > 2) return;
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

  const bool changed =
      this->target_detected_ != detected;

  const bool sensor_needs_initial_state =
      this->detected_sensor_ != nullptr &&
      !this->detected_sensor_->has_state();

  this->target_detected_ = detected;

  if (this->detected_sensor_ != nullptr &&
      (changed || sensor_needs_initial_state)) {
    this->detected_sensor_->publish_state(detected);
  }

  this->status_lights_.set_detected(
      detected,
      now);
}

void LDL508PROComponent::set_led_setting(
    LEDSetting setting,
    float seconds) {

  if (!std::isfinite(seconds)) {
    return;
  }

  const uint32_t milliseconds =
      static_cast<uint32_t>(
          std::max(0.0f, seconds) * 1000.0f);

  float published_seconds = 0.0f;

  if (setting == LEDSetting::RED_AFTERGLOW) {

    this->status_lights_.set_red_afterglow_ms(
        milliseconds,
        millis());

    published_seconds =
        this->status_lights_.red_afterglow_ms() /
        1000.0f;

  } else if (setting == LEDSetting::STANDBY_TIMEOUT) {

    this->status_lights_.set_standby_timeout_ms(
        milliseconds);

    published_seconds =
        this->status_lights_.standby_timeout_ms() /
        1000.0f;

  } else {

    return;
  }

  auto it = this->led_numbers_.find(setting);

  if (it != this->led_numbers_.end()) {
    it->second->publish_state(published_seconds);
  }
}

void LDL508PROComponent::set_led_fault_(bool active) {
  this->status_lights_.set_fault(
      active,
      millis());
}

void LDL508PROComponent::finish_vehicle_event_(uint32_t now_ms) {
  VehicleEvent event{};
  if (!this->vehicle_tracker_.finish(now_ms, event)) return;
 
  ESP_LOGI(TAG, "Vehicle #%" PRIu32 " completed: %s, %.1f km/h max, %.1f km/h average, %" PRIu32 " samples",
           event.id, vehicle_direction_to_string(event.direction), event.max_speed_kmh, event.average_speed_kmh,
           event.sample_count);
  this->publish_vehicle_tracker_event_(event);
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
  ESP_LOGW(TAG, "MODE2 HEX TX: AA AA 00 48 %02X 55 55", mode);
  this->write_array(command, sizeof(command));
}

void LDL508PROComponent::start_multitarget_debug_() {
  this->rx_buffer_.clear();
  this->ingress_buffer_.clear();
  this->multi_target_stream_buffer_.clear();
  this->discard_until_newline_ = false;
  if (this->multitarget_debug_mode_ == "ascii") {
    ESP_LOGW(TAG, "ASCII multi-target logger starting (MQTT-only escaped raw stream)");
    this->request_target_mode(0);
    this->set_timeout("phase71_headw", 300, [this]() {
      ESP_LOGW(TAG, "MODE2 HEX TX: HEADW4k");
      this->write_str("HEADW4k\r\n");
    });
  } else if (this->multitarget_debug_mode_ == "hex") {
    ESP_LOGW(TAG, "Mode 2 HEX logger starting");
    this->multi_target_stream_mode_ = false;

    const uint8_t switch_to_hex[] = {
      0xAA, 0xAA, 0x66, 0x66, 0x55, 0x55
    };

    ESP_LOGW(TAG, "Switching radar communication protocol to HEX");
    this->write_array(switch_to_hex, sizeof(switch_to_hex));

    this->set_timeout("mode2_hex_start", 300, [this]() {
      this->send_hex_target_mode_command_(2);
    });
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

  this->queue_runtime_config_write_(parameter, value);
}

void LDL508PROComponent::set_switch_parameter(
    RadarParameter parameter,
    bool value) {
  this->queue_runtime_config_write_(
      parameter,
      value ? 1.0f : 0.0f);
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

  this->queue_runtime_config_write_(
    parameter,
    static_cast<float>(raw));
}

void LDL508PROComponent::set_operating_mode(
    const std::string &value) {

  if (value == "Einzelziel (Kompatibilität)") {
    this->request_operating_mode_(
        RadarOperatingMode::ASCII_SINGLE_TARGET);
    return;
  }

  if (value == "Mehrziel (Empfohlen)") {
    this->request_operating_mode_(
        RadarOperatingMode::HEX_MULTI_TARGET);
    return;
  }

  ESP_LOGW(
      TAG,
      "Unknown operating mode selection: %s",
      value.c_str());

  // Bei ungültiger Auswahl den tatsächlich aktiven Modus zurückmelden.
  if (this->operating_mode_select_ != nullptr) {
    if (this->operating_mode_ ==
        RadarOperatingMode::HEX_MULTI_TARGET) {
      this->operating_mode_select_->publish_state(
          "Mehrziel (Empfohlen)");
    } else {
      this->operating_mode_select_->publish_state(
          "Einzelziel (Kompatibilität)");
    }
  }
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
  ESP_LOGCONFIG(TAG, "  Build stage: %s – selectable Einzelziel/Mehrziel", FIRMWARE_VERSION);
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
  ESP_LOGCONFIG(TAG, "  Phase 7.1.1 multi-target debug mode: %s", this->multitarget_debug_mode_.c_str());
  ESP_LOGCONFIG(TAG, "  Phase 7.1 raw MQTT: %s", this->multitarget_raw_mqtt_enabled_ ? "ENABLED" : "DISABLED");
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
  ESP_LOGCONFIG(
    TAG,
    "  Artifact filter: %s",
    YESNO(this->artifact_filter_enabled_));
  ESP_LOGCONFIG(
      TAG,
      "  Artifact signature: %.1f ± %.1f m / %.1f ± %.1f km/h",
      this->artifact_distance_m_,
      this->artifact_distance_tolerance_m_,
      this->artifact_speed_kmh_,
      this->artifact_speed_tolerance_kmh_);
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
