#include "mode2_hex_parser.h"

#include <cstring>
#include <cmath>

namespace esphome {
namespace ldl508pro {

bool Mode2HexParser::feed(uint8_t byte) {
  target_completed_ = false;

  rx_buffer_.push_back(byte);

  // Synchronisation auf Framebeginn
  while (rx_buffer_.size() >= 2 &&
         !(rx_buffer_[0] == 0xAA && rx_buffer_[1] == 0xAA)) {
    rx_buffer_.erase(rx_buffer_.begin());
  }

  // Noch kein kompletter Frame
  if (rx_buffer_.size() < 14)
    return false;

  // Frameende prüfen
  if (!is_valid_frame_()) {
    // Ein Byte verwerfen und neu synchronisieren
    rx_buffer_.erase(rx_buffer_.begin());
    return false;
  }

  return parse_frame_();
}

bool Mode2HexParser::has_complete_target() const {
  return target_completed_;
}

uint8_t Mode2HexParser::completed_index() const {
  return completed_index_;
}

const Mode2Target &Mode2HexParser::target(uint8_t index) const {
  return targets_[index];
}

void Mode2HexParser::clear_cycle() {
  for (auto &target : targets_) {
    target.valid = false;
  }
}

bool Mode2HexParser::parse_frame_() {
  if (!this->is_valid_frame_())
    return false;

  const uint8_t index = this->rx_buffer_[2] - 0xE0;
  const uint8_t subtype = this->rx_buffer_[3];

  Mode2Target &target = this->targets_[index];

  if (subtype == 0x00) {
    // Bytes 4–7: Ziel-ID, little-endian.
    target.id = this->rx_buffer_[4];

    // Bytes 8–11: Entfernung als float32.
    target.distance_m = read_float_(&this->rx_buffer_[8]);
    target.has_distance = std::isfinite(target.distance_m);

  } else if (subtype == 0x01) {
    // Bytes 4–7: Geschwindigkeit als float32.
    target.speed_kmh = read_float_(&this->rx_buffer_[4]);

    // Bytes 8–11: SNR als float32.
    target.snr = read_float_(&this->rx_buffer_[8]);

    target.has_speed =
        std::isfinite(target.speed_kmh) &&
        std::isfinite(target.snr);
  }

  // Den verarbeiteten 14-Byte-Frame entfernen.
  this->rx_buffer_.erase(
      this->rx_buffer_.begin(),
      this->rx_buffer_.begin() + 14);

  if (target.has_distance && target.has_speed) {
    target.valid = true;
    this->completed_index_ = index;
    this->target_completed_ = true;

    // Für das nächste Datenpaar desselben Ziels vorbereiten.
    target.has_distance = false;
    target.has_speed = false;

    return true;
  }

  return false;
}

bool Mode2HexParser::is_valid_frame_() const {
  if (this->rx_buffer_.size() < 14)
    return false;

  return this->rx_buffer_[0] == 0xAA &&
         this->rx_buffer_[1] == 0xAA &&
         this->rx_buffer_[12] == 0x55 &&
         this->rx_buffer_[13] == 0x55 &&
         this->rx_buffer_[2] >= 0xE0 &&
         this->rx_buffer_[2] < (0xE0 + MODE2_MAX_TARGETS) &&
         (this->rx_buffer_[3] == 0x00 || this->rx_buffer_[3] == 0x01);
}

float Mode2HexParser::read_float_(const uint8_t *ptr) {
  float value;
  std::memcpy(&value, ptr, sizeof(float));
  return value;
}

}  // namespace ldl508pro
}  // namespace esphome