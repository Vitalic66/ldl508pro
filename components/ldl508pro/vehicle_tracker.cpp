#include "vehicle_tracker.h"

#include <algorithm>
#include <cmath>

namespace esphome {
namespace ldl508pro {

void VehicleTracker::add_measurement(const Measurement &measurement) {
  const float absolute_speed = std::fabs(measurement.speed_kmh);

  if (!this->tracking_) {
    this->tracking_ = true;
    this->current_ = {};
    this->current_.id = this->next_id_++;
    this->current_.started_ms = measurement.timestamp_ms;
    this->current_.first_distance_m = measurement.distance_m;
    this->current_.last_distance_m = measurement.distance_m;
    this->current_.minimum_distance_m = measurement.distance_m;
    this->current_.max_speed_kmh = absolute_speed;
    this->previous_distance_m_ = measurement.distance_m;
    this->speed_sum_kmh_ = 0.0f;
    this->direction_score_ = 0;
  }

  this->current_.sample_count++;
  this->current_.last_distance_m = measurement.distance_m;
  this->current_.minimum_distance_m = std::min(this->current_.minimum_distance_m, measurement.distance_m);
  this->current_.max_speed_kmh = std::max(this->current_.max_speed_kmh, absolute_speed);
  this->speed_sum_kmh_ += absolute_speed;

  // LDL508PRO convention confirmed in the field: negative speed means approaching.
  if (measurement.speed_kmh < -0.5f) this->direction_score_ += 2;
  if (measurement.speed_kmh > 0.5f) this->direction_score_ -= 2;

  // Distance trend acts as an independent plausibility check and stabilizes noisy speed signs.
  if (this->current_.sample_count > 1) {
    const float delta = measurement.distance_m - this->previous_distance_m_;
    if (delta < -0.15f) this->direction_score_ += 1;
    if (delta > 0.15f) this->direction_score_ -= 1;
  }
  this->previous_distance_m_ = measurement.distance_m;
}

VehicleDirection VehicleTracker::current_direction() const {
  if (this->direction_score_ > 0) return VehicleDirection::APPROACHING;
  if (this->direction_score_ < 0) return VehicleDirection::RECEDING;
  return VehicleDirection::UNKNOWN;
}

bool VehicleTracker::finish(uint32_t now_ms, VehicleEvent &event) {
  if (!this->tracking_ || this->current_.sample_count == 0) return false;

  this->current_.direction = this->current_direction();
  this->current_.average_speed_kmh = this->speed_sum_kmh_ / static_cast<float>(this->current_.sample_count);
  this->current_.duration_ms = static_cast<uint32_t>(now_ms - this->current_.started_ms);
  event = this->current_;
  this->tracking_ = false;
  return true;
}

void VehicleTracker::reset() {
  this->tracking_ = false;
  this->current_ = {};
  this->speed_sum_kmh_ = 0.0f;
  this->previous_distance_m_ = 0.0f;
  this->direction_score_ = 0;
}

const char *vehicle_direction_to_string(VehicleDirection direction) {
  switch (direction) {
    case VehicleDirection::APPROACHING: return "Annähernd";
    case VehicleDirection::RECEDING: return "Entfernend";
    default: return "Unbekannt";
  }
}

}  // namespace ldl508pro
}  // namespace esphome
