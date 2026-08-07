#include "warning_light_controller.h"

#include "esphome/core/log.h"

namespace esphome {
namespace ldl508pro {

static const char *const TAG =
    "ldl508pro.warning_light";

void WarningLightController::setup() {

  if (this->output_pin_ == nullptr) {
    ESP_LOGCONFIG(
        TAG,
        "Warning light not configured");
    return;
  }

  this->output_pin_->setup();

  // Sicherer Startzustand: AUS
  this->output_pin_->digital_write(false);
  this->active_ = false;

  ESP_LOGI(
      TAG,
      "Warning light initialized: OFF");
}

void WarningLightController::set_active(
    bool active) {

  if (this->active_ == active) {
    return;
  }

  this->active_ = active;

  if (this->output_pin_ != nullptr) {
    this->output_pin_->digital_write(active);
  }

  ESP_LOGI(
      TAG,
      "Warning light -> %s",
      active ? "ON" : "OFF");
}

}  // namespace ldl508pro
}  // namespace esphome