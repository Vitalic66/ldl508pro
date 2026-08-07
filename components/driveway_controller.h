#pragma once

#include <cstdint>

namespace esphome {
namespace ldl508pro {

enum class DrivewayState : uint8_t {
  IDLE = 0,
  ACTIVE_CLEAR,
  ACTIVE_TRAFFIC,
};

class DrivewayController {
 public:
  void set_active_timeout_ms(uint32_t value) {
    this->active_timeout_ms_ = value;
  }

  void setup(uint32_t now_ms);
  void loop(uint32_t now_ms);

  // Einmaliges Ereignis von der Lichtschranke:
  // Fahrzeug hat das Carport verlassen.
  void trigger_departure(uint32_t now_ms);

  // Relevanter Radarverkehr.
  // true  = Warnlage
  // false = aktuell kein relevanter Verkehr
  void set_traffic_warning(
      bool active,
      uint32_t now_ms);

  DrivewayState state() const {
    return this->state_;
  }

  bool active() const {
    return this->state_ != DrivewayState::IDLE;
  }

  bool traffic_warning() const {
    return this->state_ ==
           DrivewayState::ACTIVE_TRAFFIC;
  }

 private:
  void set_state_(
      DrivewayState state,
      uint32_t now_ms);

  void restart_active_timeout_(
      uint32_t now_ms);

  DrivewayState state_{DrivewayState::IDLE};

  uint32_t active_timeout_ms_{30000};
  uint32_t active_timeout_started_ms_{0};

  bool traffic_warning_active_{false};
};

}  // namespace ldl508pro
}  // namespace esphome