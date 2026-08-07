#include "driveway_controller.h"

#include "esphome/core/log.h"

namespace esphome {
namespace ldl508pro {

static const char *const TAG =
    "ldl508pro.driveway";

void DrivewayController::setup(
    uint32_t now_ms) {

  this->state_ = DrivewayState::IDLE;
  this->traffic_warning_active_ = false;
  this->active_timeout_started_ms_ = now_ms;

  ESP_LOGI(
      TAG,
      "Driveway controller initialized: IDLE");
}

void DrivewayController::loop(
    uint32_t now_ms) {

  if (this->state_ == DrivewayState::IDLE) {
    return;
  }

  // Während einer aktiven Radarwarnung soll der
  // Aktivtimer nicht ablaufen.
  if (this->traffic_warning_active_) {
    return;
  }

  if (static_cast<uint32_t>(
          now_ms -
          this->active_timeout_started_ms_) >=
      this->active_timeout_ms_) {

    this->set_state_(
        DrivewayState::IDLE,
        now_ms);

    ESP_LOGI(
        TAG,
        "Driveway assistance timeout expired");
  }
}

void DrivewayController::trigger_departure(
    uint32_t now_ms) {

  this->traffic_warning_active_ = false;

  this->restart_active_timeout_(
      now_ms);

  this->set_state_(
      DrivewayState::ACTIVE_CLEAR,
      now_ms);

  ESP_LOGI(
      TAG,
      "Departure detected: driveway assistance active");
}

void DrivewayController::set_traffic_warning(
    bool active,
    uint32_t now_ms) {

  if (this->traffic_warning_active_ == active) {
    return;
  }

  this->traffic_warning_active_ = active;

  if (!this->active()) {
    // Radarverkehr alleine aktiviert das System nicht.
    // Erst eine bestätigte Ausfahrt darf ACTIVE starten.
    return;
  }

  if (active) {

    // Rot/Warnung setzt den 30-s-Timer zurück.
    this->restart_active_timeout_(now_ms);

    this->set_state_(
        DrivewayState::ACTIVE_TRAFFIC,
        now_ms);

    ESP_LOGI(
        TAG,
        "Relevant traffic detected");

  } else {

    // Verkehr verschwunden:
    // Timer beginnt erneut ab jetzt.
    this->restart_active_timeout_(now_ms);

    this->set_state_(
        DrivewayState::ACTIVE_CLEAR,
        now_ms);

    ESP_LOGI(
        TAG,
        "Relevant traffic cleared");
  }
}

void DrivewayController::restart_active_timeout_(
    uint32_t now_ms) {

  this->active_timeout_started_ms_ = now_ms;
}

void DrivewayController::set_state_(
    DrivewayState state,
    uint32_t now_ms) {

  if (this->state_ == state) {
    return;
  }

  this->state_ = state;

  const char *name = "UNKNOWN";

  switch (state) {
    case DrivewayState::IDLE:
      name = "IDLE";
      break;

    case DrivewayState::ACTIVE_CLEAR:
      name = "ACTIVE_CLEAR";
      break;

    case DrivewayState::ACTIVE_TRAFFIC:
      name = "ACTIVE_TRAFFIC";
      break;
  }

  ESP_LOGI(
      TAG,
      "State -> %s",
      name);

  (void) now_ms;
}

}  // namespace ldl508pro
}  // namespace esphome