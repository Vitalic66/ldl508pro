#pragma once

#include <cmath>

#include "vehicle_tracker.h"

namespace esphome {
namespace ldl508pro {

class GhostFilter {
 public:
  bool matches(const Measurement &measurement, float distance_m, float distance_tolerance_m,
               float speed_kmh, float speed_tolerance_kmh) const {
    const bool distance_match = std::fabs(measurement.distance_m - distance_m) <= distance_tolerance_m;
    const bool speed_match = std::fabs(std::fabs(measurement.speed_kmh) - speed_kmh) <= speed_tolerance_kmh;
    return distance_match && speed_match;
  }
};

}  // namespace ldl508pro
}  // namespace esphome
