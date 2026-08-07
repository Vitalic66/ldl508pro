#include "carport_presence_controller.h"

#include "esphome/core/log.h"

namespace esphome {
namespace ldl508pro {

static const char *const TAG =
    "ldl508pro.carport_presence";

void CarportPresenceController::setup(
    uint32_t now_ms) {

  if (this->input_pin_ == nullptr) {
    ESP_LOGCONFIG(
        TAG,
        "Carport light barrier not configured");
    return;
  }

  this->input_pin_->setup();

  // Verdrahtung:
  // GPIO13 mit internem Pull-up
  // COM -> GND
  // NO  -> GPIO13
  //
  // LOW  = COM-NO geschlossen = Strahl frei
  // HIGH = Kontakt offen      = Strahl unterbrochen
  const bool beam_clear =
      !this->input_pin_->digital_read();

  this->beam_clear_ = beam_clear;
  this->occupied_ = !beam_clear;
  this->initialized_ = true;

  this->clear_candidate_active_ = false;
  this->departure_confirmed_ = false;
  this->clear_started_ms_ = now_ms;

  ESP_LOGI(
      TAG,
      "Initial state: beam=%s, carport=%s",
      this->beam_clear_ ? "clear" : "blocked",
      this->occupied_ ? "occupied" : "free");
}

void CarportPresenceController::loop(
    uint32_t now_ms) {

  if (this->input_pin_ == nullptr) {
    return;
  }

  const bool beam_clear =
      !this->input_pin_->digital_read();

  this->process_input_(
      beam_clear,
      now_ms);
}

void CarportPresenceController::process_input_(
    bool beam_clear,
    uint32_t now_ms) {

  if (!this->initialized_) {
    this->beam_clear_ = beam_clear;
    this->occupied_ = !beam_clear;
    this->initialized_ = true;
    return;
  }

  const bool changed =
      this->beam_clear_ != beam_clear;

  if (changed) {
    this->beam_clear_ = beam_clear;

    if (!beam_clear) {
      // Strahl unterbrochen:
      // Der Stellplatz gilt sofort als belegt.
      this->occupied_ = true;
      this->clear_candidate_active_ = false;
      this->departure_confirmed_ = false;

      ESP_LOGI(
          TAG,
          "Beam blocked: carport occupied");

    } else if (this->occupied_) {
      // Der Strahl wurde nach einer Belegung frei.
      // Erst nach dauerhaft freiem Strahl wird daraus
      // ein bestätigtes Ausfahrtereignis.
      this->clear_candidate_active_ = true;
      this->clear_started_ms_ = now_ms;

      ESP_LOGI(
          TAG,
          "Beam clear: departure candidate started");
    }
  }

  if (this->clear_candidate_active_ &&
      this->beam_clear_ &&
      static_cast<uint32_t>(
          now_ms - this->clear_started_ms_) >=
          this->clear_confirm_ms_) {

    this->clear_candidate_active_ = false;
    this->occupied_ = false;
    this->departure_confirmed_ = true;

    ESP_LOGI(
        TAG,
        "Departure confirmed after %u ms clear beam",
        static_cast<unsigned>(
            this->clear_confirm_ms_));
  }
}

}  // namespace ldl508pro
}  // namespace esphome