#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace esphome {
namespace ldl508pro {

static constexpr uint8_t MODE2_MAX_TARGETS = 9;

struct Mode2Target {
  bool valid = false;

  uint8_t id = 0;

  float distance_m = NAN;
  float speed_kmh = NAN;
  float snr = NAN;

  bool has_distance = false;
  bool has_speed = false;
};

class Mode2HexParser {
 public:
  bool feed(uint8_t byte);

  bool has_complete_target() const;
  uint8_t completed_index() const;

  const Mode2Target &target(uint8_t index) const;

  void clear_cycle();

 private:
  bool parse_frame_();
  bool is_valid_frame_() const;

  static float read_float_(const uint8_t *ptr);

  std::vector<uint8_t> rx_buffer_;

  std::array<Mode2Target, MODE2_MAX_TARGETS> targets_;

  bool target_completed_ = false;
  uint8_t completed_index_ = 0;
};

}  // namespace ldl508pro
}  // namespace esphome