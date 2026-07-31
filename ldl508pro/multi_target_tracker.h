#pragma once

#include <array>
#include <cstdint>

namespace esphome {
namespace ldl508pro {

class MultiTargetTracker {
 public:
  static constexpr uint8_t MAX_TRACKS = 9;

  enum class Direction : uint8_t { UNKNOWN = 0, APPROACHING, RECEDING, STATIONARY };

  struct Track {
    bool active{false};
    bool confirmed{false};
    bool matched{false};
    uint16_t id{0};
    float distance_m{0.0f};
    float speed_kmh{0.0f};
    uint32_t first_seen_ms{0};
    uint32_t last_seen_ms{0};
    uint8_t hits{0};
    uint8_t misses{0};
    Direction direction{Direction::UNKNOWN};
  };

  void update(const std::array<float, MAX_TRACKS> &distances, uint8_t count, uint32_t now_ms);
  void expire(uint32_t now_ms);
  void reset();

  const std::array<Track, MAX_TRACKS> &tracks() const { return this->tracks_; }
  uint8_t active_count() const;
  uint8_t confirmed_count() const;

  void set_confirmation_hits(uint8_t value) { this->confirmation_hits_ = value == 0 ? 1 : value; }
  void set_match_distance(float value) { this->match_distance_m_ = value; }
  void set_tentative_timeout(uint32_t value) { this->tentative_timeout_ms_ = value; }
  void set_confirmed_timeout(uint32_t value) { this->confirmed_timeout_ms_ = value; }

  static const char *direction_to_string(Direction direction);

 protected:
  float predicted_distance_(const Track &track, uint32_t now_ms) const;
  int find_free_track_() const;
  void start_track_(uint8_t index, float distance_m, uint32_t now_ms);
  void update_track_(Track &track, float distance_m, uint32_t now_ms);

  std::array<Track, MAX_TRACKS> tracks_{};
  uint16_t next_id_{1};
  uint8_t confirmation_hits_{2};
  float match_distance_m_{7.0f};
  uint32_t tentative_timeout_ms_{750};
  uint32_t confirmed_timeout_ms_{1500};
};

}  // namespace ldl508pro
}  // namespace esphome
