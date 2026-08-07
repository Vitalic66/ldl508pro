#pragma once

#include <cstdint>

#include "esphome/core/gpio.h"

namespace esphome {
namespace ldl508pro {

class StatusLightController {
 public:
  void set_red_output_pin(GPIOPin *pin) {
    this->red_output_pin_ = pin;
  }

  void set_green_output_pin(GPIOPin *pin) {
    this->green_output_pin_ = pin;
  }

  void setup(uint32_t now_ms);
  void loop(uint32_t now_ms);

  void set_detected(bool detected, uint32_t now_ms);
  void set_fault(bool active, uint32_t now_ms);

  void set_red_afterglow_ms(uint32_t value, uint32_t now_ms);
  void set_standby_timeout_ms(uint32_t value);

  uint32_t red_afterglow_ms() const {
    return this->red_afterglow_ms_;
  }

  uint32_t standby_timeout_ms() const {
    return this->standby_timeout_ms_;
  }

 private:
  void update_outputs_(uint32_t now_ms);

  GPIOPin *red_output_pin_{nullptr};
  GPIOPin *green_output_pin_{nullptr};

  uint32_t red_afterglow_ms_{5000};
  uint32_t standby_timeout_ms_{60000};

  uint32_t afterglow_started_ms_{0};
  uint32_t idle_started_ms_{0};
  uint32_t fault_blink_ms_{0};

  bool detected_{false};
  bool detection_initialized_{false};
  bool afterglow_active_{false};
  bool fault_active_{false};
  bool fault_blink_state_{false};
};

}  // namespace ldl508pro
}  // namespace esphome