#pragma once

#include <cstdint>

#include "uart_parser.h"

namespace esphome {
namespace ldl508pro {

enum class VehicleDirection : uint8_t {
  UNKNOWN = 0,
  APPROACHING,
  RECEDING,
};

struct VehicleEvent {
  uint32_t id{0};
  VehicleDirection direction{VehicleDirection::UNKNOWN};
  float max_speed_kmh{0.0f};
  float average_speed_kmh{0.0f};
  float first_distance_m{0.0f};
  float last_distance_m{0.0f};
  float minimum_distance_m{0.0f};
  uint32_t started_ms{0};
  uint32_t duration_ms{0};
  uint32_t sample_count{0};
};

class VehicleTracker {
 public:
  VehicleTracker() = default;

  void add_measurement(const Measurement &measurement);
  bool finish(uint32_t now_ms, VehicleEvent &event);
  void reset();

  bool tracking() const { return this->tracking_; }
  uint32_t current_id() const { return this->current_.id; }
  VehicleDirection current_direction() const;

 private:
  bool tracking_{false};
  uint32_t next_id_{1};
  VehicleEvent current_{};
  float speed_sum_kmh_{0.0f};
  float previous_distance_m_{0.0f};
  int32_t direction_score_{0};
};

const char *vehicle_direction_to_string(VehicleDirection direction);

}  // namespace ldl508pro
}  // namespace esphome
