#pragma once

#include "esphome/core/gpio.h"

namespace esphome {
namespace ldl508pro {

class WarningLightController {
 public:
  void set_output_pin(GPIOPin *pin) {
    this->output_pin_ = pin;
  }

  void setup();

  void set_active(bool active);

  bool active() const {
    return this->active_;
  }

 private:
  GPIOPin *output_pin_{nullptr};
  bool active_{false};
};

}  // namespace ldl508pro
}  // namespace esphome