#include "status_light_controller.h"
#include <inttypes.h>

#include "esphome/core/log.h"

namespace esphome {
namespace ldl508pro {

static const char *const TAG = "ldl508pro.status_lights";

void StatusLightController::setup(uint32_t now_ms) {
  if (this->red_output_pin_ != nullptr) {
    this->red_output_pin_->setup();
    this->red_output_pin_->digital_write(false);
  }

  if (this->green_output_pin_ != nullptr) {
    this->green_output_pin_->setup();
    this->green_output_pin_->digital_write(false);
  }

  this->idle_started_ms_ = now_ms;
  this->update_outputs_(now_ms);
}

void StatusLightController::loop(uint32_t now_ms) {
  this->update_outputs_(now_ms);
}

void StatusLightController::set_detected(
    bool detected,
    uint32_t now_ms) {

  const bool changed =
      this->detected_ != detected;

  ESP_LOGI(
      TAG,
      "set_detected(%s): changed=%s initialized=%s "
      "afterglow=%u ms",
      detected ? "true" : "false",
      changed ? "true" : "false",
      this->detection_initialized_ ? "true" : "false",
      static_cast<unsigned>(this->red_afterglow_ms_));

  // Fallende Flanke: Erkennung wurde gerade beendet.
  if (changed && !detected) {

    this->afterglow_active_ =
        this->red_afterglow_ms_ > 0;

    this->afterglow_started_ms_ = now_ms;

    ESP_LOGI(
        TAG,
        "Afterglow %s for %u ms",
        this->afterglow_active_ ? "started" : "disabled",
        static_cast<unsigned>(this->red_afterglow_ms_));

    if (!this->afterglow_active_) {
      this->idle_started_ms_ = now_ms;
    }

  } else if (changed && detected) {

    // Steigende Flanke: aktive Erkennung hat Vorrang.
    this->afterglow_active_ = false;
    this->idle_started_ms_ = now_ms;
  }

  this->detected_ = detected;
  this->detection_initialized_ = true;

  this->update_outputs_(now_ms);
}

void StatusLightController::set_fault(
    bool active,
    uint32_t now_ms) {

  if (this->fault_active_ == active) {
    return;
  }

  this->fault_active_ = active;
  this->fault_blink_state_ = false;
  this->fault_blink_ms_ = now_ms;

  ESP_LOGW(
      TAG,
      "Fault indication: %s",
      active ? "ACTIVE (red blinking)" : "cleared");

  this->update_outputs_(now_ms);
}

void StatusLightController::set_red_afterglow_ms(
    uint32_t value,
    uint32_t now_ms) {

  this->red_afterglow_ms_ = value;

  if (value == 0 && this->afterglow_active_) {
    this->afterglow_active_ = false;
    this->idle_started_ms_ = now_ms;
  }

  this->update_outputs_(now_ms);
}

void StatusLightController::set_standby_timeout_ms(
    uint32_t value) {
  this->standby_timeout_ms_ = value;
}

void StatusLightController::update_outputs_(
    uint32_t now_ms) {
/*
  ESP_LOGI(
    TAG,
    "update: detected=%d afterglow=%d fault=%d",
    this->detected_,
    this->afterglow_active_,
    this->fault_active_);
*/
  bool red = false;
  bool green = false;

  // Fehler besitzt höchste Priorität:
  // Rot blinkt im 500-ms-Takt, Grün bleibt aus.
  if (this->fault_active_) {

    if (static_cast<uint32_t>(
            now_ms - this->fault_blink_ms_) >= 500) {
      this->fault_blink_ms_ = now_ms;
      this->fault_blink_state_ =
          !this->fault_blink_state_;
    }

    red = this->fault_blink_state_;

  } else if (this->detected_) {

    red = true;

  } else {

    if (this->afterglow_active_ &&
        static_cast<uint32_t>(
            now_ms - this->afterglow_started_ms_) >=
            this->red_afterglow_ms_) {

      this->afterglow_active_ = false;
      this->idle_started_ms_ = now_ms;
    }

    if (this->afterglow_active_) {

      //ESP_LOGI(TAG, "RED = afterglow");

      red = true;

    } else {

      // 0 deaktiviert Standby: Grün bleibt dauerhaft an.
      green =
          this->standby_timeout_ms_ == 0 ||
          static_cast<uint32_t>(
              now_ms - this->idle_started_ms_) <
              this->standby_timeout_ms_;
    }
  }

  // Der externe LED-Treiber ist active HIGH.
  if (this->red_output_pin_ != nullptr) {
/*
    ESP_LOGI(
    TAG,
    "OUTPUT red=%d green=%d",
    red,
    green);
*/
    this->red_output_pin_->digital_write(red);
  }

  if (this->green_output_pin_ != nullptr) {
    this->green_output_pin_->digital_write(green);
  }
}

}  // namespace ldl508pro
}  // namespace esphome