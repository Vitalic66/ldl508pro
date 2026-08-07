#pragma once

#include <cstdint>

#include "esphome/core/gpio.h"

namespace esphome {
namespace ldl508pro {

class CarportPresenceController {
 public:
  void set_input_pin(GPIOPin *pin) {
    this->input_pin_ = pin;
  }

  void set_clear_confirm_ms(uint32_t value) {
    this->clear_confirm_ms_ = value;
  }

  void setup(uint32_t now_ms);
  void loop(uint32_t now_ms);

  bool beam_clear() const {
    return this->beam_clear_;
  }

  bool occupied() const {
    return this->occupied_;
  }

  bool departure_candidate() const {
    return this->clear_candidate_active_;
  }

  bool departure_confirmed() const {
    return this->departure_confirmed_;
  }

  bool initialized() const {
    return this->initialized_;
  }

  void clear_departure_event() {
    this->departure_confirmed_ = false;
  }

 private:
  void process_input_(
      bool beam_clear,
      uint32_t now_ms);

  GPIOPin *input_pin_{nullptr};

  uint32_t clear_confirm_ms_{2000};
  uint32_t clear_started_ms_{0};

  bool beam_clear_{false};
  bool occupied_{false};
  bool initialized_{false};

  bool clear_candidate_active_{false};
  bool departure_confirmed_{false};
};

}  // namespace ldl508pro
}  // namespace esphome