#include "multi_target_tracker.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace esphome {
namespace ldl508pro {

namespace {
struct Candidate {
  uint8_t track;
  uint8_t measurement;
  float cost;
};
}  // namespace

void MultiTargetTracker::reset() {
  for (auto &track : this->tracks_) track = Track{};
  this->next_id_ = 1;
}

uint8_t MultiTargetTracker::active_count() const {
  uint8_t count = 0;
  for (const auto &track : this->tracks_) if (track.active) count++;
  return count;
}

uint8_t MultiTargetTracker::confirmed_count() const {
  uint8_t count = 0;
  for (const auto &track : this->tracks_) if (track.active && track.confirmed) count++;
  return count;
}

const char *MultiTargetTracker::direction_to_string(Direction direction) {
  switch (direction) {
    case Direction::APPROACHING: return "annaehernd";
    case Direction::RECEDING: return "entfernend";
    case Direction::STATIONARY: return "stationaer";
    default: return "unbekannt";
  }
}

float MultiTargetTracker::predicted_distance_(const Track &track, uint32_t now_ms) const {
  if (!track.active || track.last_seen_ms == 0 || now_ms <= track.last_seen_ms) return track.distance_m;
  const float dt_s = static_cast<float>(now_ms - track.last_seen_ms) / 1000.0f;
  return track.distance_m + (track.speed_kmh / 3.6f) * dt_s;
}

int MultiTargetTracker::find_free_track_() const {
  for (uint8_t i = 0; i < MAX_TRACKS; i++) if (!this->tracks_[i].active) return i;
  return -1;
}

void MultiTargetTracker::start_track_(uint8_t index, float distance_m, uint32_t now_ms) {
  Track &track = this->tracks_[index];
  track = Track{};
  track.active = true;
  track.id = this->next_id_++;
  if (this->next_id_ == 0) this->next_id_ = 1;
  track.distance_m = distance_m;
  track.first_seen_ms = now_ms;
  track.last_seen_ms = now_ms;
  track.hits = 1;
  track.matched = true;
  track.confirmed = this->confirmation_hits_ <= 1;
}

void MultiTargetTracker::update_track_(Track &track, float distance_m, uint32_t now_ms) {
  const uint32_t dt_ms = now_ms - track.last_seen_ms;
  if (dt_ms > 0) {
    const float instantaneous_kmh = (distance_m - track.distance_m) * 3600.0f / static_cast<float>(dt_ms);
    if (std::isfinite(instantaneous_kmh) && std::fabs(instantaneous_kmh) <= 220.0f) {
      if (track.hits <= 1) track.speed_kmh = instantaneous_kmh;
      else track.speed_kmh = track.speed_kmh * 0.65f + instantaneous_kmh * 0.35f;
    }
  }
  track.distance_m = distance_m;
  track.last_seen_ms = now_ms;
  track.matched = true;
  track.misses = 0;
  if (track.hits < 255) track.hits++;
  if (track.hits >= this->confirmation_hits_) track.confirmed = true;

  if (track.speed_kmh > 2.0f) track.direction = Direction::RECEDING;
  else if (track.speed_kmh < -2.0f) track.direction = Direction::APPROACHING;
  else if (track.hits >= 2) track.direction = Direction::STATIONARY;
}

void MultiTargetTracker::expire(uint32_t now_ms) {
  for (auto &track : this->tracks_) {
    if (!track.active) continue;
    const uint32_t timeout = track.confirmed ? this->confirmed_timeout_ms_ : this->tentative_timeout_ms_;
    if (static_cast<uint32_t>(now_ms - track.last_seen_ms) > timeout) track = Track{};
  }
}

void MultiTargetTracker::update(const std::array<float, MAX_TRACKS> &distances, uint8_t count, uint32_t now_ms) {
  this->expire(now_ms);
  for (auto &track : this->tracks_) track.matched = false;

  std::array<bool, MAX_TRACKS> measurement_used{};
  std::vector<Candidate> candidates;
  candidates.reserve(MAX_TRACKS * MAX_TRACKS);

  for (uint8_t ti = 0; ti < MAX_TRACKS; ti++) {
    const Track &track = this->tracks_[ti];
    if (!track.active) continue;
    const float predicted = this->predicted_distance_(track, now_ms);
    const float age_s = static_cast<float>(now_ms - track.last_seen_ms) / 1000.0f;
    const float gate = this->match_distance_m_ + std::min(8.0f, age_s * 6.0f);
    for (uint8_t mi = 0; mi < count; mi++) {
      const float cost = std::fabs(distances[mi] - predicted);
      if (cost <= gate) candidates.push_back({ti, mi, cost});
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
    return a.cost < b.cost;
  });

  for (const auto &candidate : candidates) {
    Track &track = this->tracks_[candidate.track];
    if (!track.active || track.matched || measurement_used[candidate.measurement]) continue;
    this->update_track_(track, distances[candidate.measurement], now_ms);
    measurement_used[candidate.measurement] = true;
  }

  for (auto &track : this->tracks_) {
    if (track.active && !track.matched && track.misses < 255) track.misses++;
  }

  for (uint8_t mi = 0; mi < count; mi++) {
    if (measurement_used[mi]) continue;
    const int index = this->find_free_track_();
    if (index < 0) break;
    this->start_track_(static_cast<uint8_t>(index), distances[mi], now_ms);
  }
}

}  // namespace ldl508pro
}  // namespace esphome
