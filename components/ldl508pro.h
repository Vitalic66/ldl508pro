#pragma once

#include <cstdint>
#include <array>
#include <map>
#include <string>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_client.h"
#endif

#include "config_manager.h"
#include "uart_parser.h"
#include "vehicle_tracker.h"
#include "ghost_filter.h"
#include "mode2_hex_parser.h"

namespace esphome {
namespace ldl508pro {

class LDL508PROComponent;

enum class LEDSetting : uint8_t {
  RED_AFTERGLOW,
  STANDBY_TIMEOUT,
};

class LDLNumber : public number::Number {
 public:
  LDLNumber(LDL508PROComponent *parent, RadarParameter parameter) : parent_(parent), parameter_(parameter) {}

 protected:
  void control(float value) override;
  LDL508PROComponent *parent_;
  RadarParameter parameter_;
};

class LDLLEDNumber : public number::Number {
 public:
  LDLLEDNumber(LDL508PROComponent *parent, LEDSetting setting) : parent_(parent), setting_(setting) {}

 protected:
  void control(float value) override;
  LDL508PROComponent *parent_;
  LEDSetting setting_;
};

class LDLSwitch : public switch_::Switch {
 public:
  LDLSwitch(LDL508PROComponent *parent, RadarParameter parameter) : parent_(parent), parameter_(parameter) {}

 protected:
  void write_state(bool state) override;
  LDL508PROComponent *parent_;
  RadarParameter parameter_;
};

class LDLSelect : public select::Select {
 public:
  LDLSelect(LDL508PROComponent *parent, RadarParameter parameter) : parent_(parent), parameter_(parameter) {}

 protected:
  void control(const std::string &value) override;
  LDL508PROComponent *parent_;
  RadarParameter parameter_;
};

class LDLOperatingModeSelect : public select::Select {
 public:
  explicit LDLOperatingModeSelect(LDL508PROComponent *parent)
      : parent_(parent) {}

 protected:
  void control(const std::string &value) override;

  LDL508PROComponent *parent_;
};

enum class LDLButtonAction : uint8_t {
  REFRESH_CONFIGURATION,
  FACTORY_RESET,
  REQUEST_MULTI_TARGET_SNAPSHOT,
  TARGET_MODE_0,
  TARGET_MODE_1,
  START_RAW_CAPTURE,
};

// Compatibility aliases for ESPHome code generation; see config_manager.h.
inline constexpr LDLButtonAction REFRESH_CONFIGURATION = LDLButtonAction::REFRESH_CONFIGURATION;
inline constexpr LDLButtonAction FACTORY_RESET = LDLButtonAction::FACTORY_RESET;
inline constexpr LDLButtonAction REQUEST_MULTI_TARGET_SNAPSHOT = LDLButtonAction::REQUEST_MULTI_TARGET_SNAPSHOT;
inline constexpr LDLButtonAction TARGET_MODE_0 = LDLButtonAction::TARGET_MODE_0;
inline constexpr LDLButtonAction TARGET_MODE_1 = LDLButtonAction::TARGET_MODE_1;
inline constexpr LDLButtonAction START_RAW_CAPTURE = LDLButtonAction::START_RAW_CAPTURE;

class LDLButton : public button::Button {
 public:
  LDLButton(LDL508PROComponent *parent, LDLButtonAction action) : parent_(parent), action_(action) {}

 protected:
  void press_action() override;
  LDL508PROComponent *parent_;
  LDLButtonAction action_;
};

class LDL508PROComponent : public Component,
                           public uart::UARTDevice,
                           public ConfigManagerListener {
 public:
  LDL508PROComponent() : config_manager_(this) {}

  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_debug_uart(bool debug_uart) { this->debug_uart_ = debug_uart; }
  void set_target_timeout(uint32_t value) { this->target_timeout_ms_ = value; }
  void set_boot_read_delay(uint32_t value) { this->boot_read_delay_ms_ = value; }
  void set_command_gap(uint32_t value) { this->config_manager_.set_command_gap(value); this->command_gap_ms_ = value; }
  void set_command_timeout(uint32_t value) { this->config_manager_.set_command_timeout(value); this->command_timeout_ms_ = value; }
  void set_command_retries(uint8_t value) { this->config_manager_.set_retries(value); this->command_retries_ = value; }
  void set_multi_target_polling(bool value) { this->multi_target_polling_ = value; }
  void set_multi_target_poll_interval(uint32_t value) { this->multi_target_poll_interval_ms_ = value; }
  void set_multi_target_hold_time(uint32_t value) { this->multi_target_hold_time_ms_ = value; }
  void set_multi_target_confirmation_frames(uint8_t value) { this->multi_target_confirmation_frames_ = value; }
  void set_multi_target_confirmation_window(uint32_t value) { this->multi_target_confirmation_window_ms_ = value; }
  void set_multi_target_confirmation_tolerance(float value) { this->multi_target_confirmation_tolerance_m_ = value; }
  void set_multi_target_min_confirmation_movement(float value) { this->multi_target_min_confirmation_movement_m_ = value; }
  void set_auto_enable_multi_target_after_sync(bool value) { this->auto_enable_multi_target_after_sync_ = value; }
  void set_raw_capture_duration(uint32_t value) { this->raw_capture_duration_ms_ = value; }
  void set_raw_capture_max_bytes(uint32_t value) { this->raw_capture_max_bytes_ = value; }
  void set_red_output_pin(GPIOPin *pin) { this->red_output_pin_ = pin; }
  void set_green_output_pin(GPIOPin *pin) { this->green_output_pin_ = pin; }
  void set_artifact_filter(bool value) { this->artifact_filter_enabled_ = value; }
  void set_artifact_distance(float value) { this->artifact_distance_m_ = value; }
  void set_artifact_distance_tolerance(float value) { this->artifact_distance_tolerance_m_ = value; }
  void set_artifact_speed(float value) { this->artifact_speed_kmh_ = value; }
  void set_artifact_speed_tolerance(float value) { this->artifact_speed_tolerance_kmh_ = value; }
  void set_mqtt_event_enabled(bool value) { this->mqtt_event_enabled_ = value; }
  void set_mqtt_event_topic(const std::string &value) { this->mqtt_event_topic_ = value; }
  void set_mqtt_event_qos(uint8_t value) { this->mqtt_event_qos_ = value; }
  void set_mqtt_event_retain(bool value) { this->mqtt_event_retain_ = value; }
  void set_multitarget_debug_mode(const std::string &value) { this->multitarget_debug_mode_ = value; }
  void set_multitarget_raw_mqtt_enabled(bool value) { this->multitarget_raw_mqtt_enabled_ = value; }
  void set_multitarget_raw_mqtt_topic(const std::string &value) { this->multitarget_raw_mqtt_topic_ = value; }
  void set_multitarget_parsed_mqtt_topic(const std::string &value) { this->multitarget_parsed_mqtt_topic_ = value; }
  void set_multitarget_mqtt_qos(uint8_t value) { this->multitarget_mqtt_qos_ = value; }
  void set_led_setting(LEDSetting setting, float seconds);
  void register_led_number(LEDSetting setting, LDLLEDNumber *entity) { this->led_numbers_[setting] = entity; }

  void set_distance_sensor(sensor::Sensor *value) { this->distance_sensor_ = value; }
  void set_speed_sensor(sensor::Sensor *value) { this->speed_sensor_ = value; }
  void set_detected_sensor(binary_sensor::BinarySensor *value) { this->detected_sensor_ = value; }
  void set_config_synchronized_sensor(binary_sensor::BinarySensor *value) { this->config_synchronized_sensor_ = value; }
  void set_configuration_sensor(text_sensor::TextSensor *value) { this->configuration_sensor_ = value; }
  void set_last_config_error_sensor(text_sensor::TextSensor *value) { this->last_config_error_sensor_ = value; }
  void set_last_cli_command_sensor(text_sensor::TextSensor *value) { this->last_cli_command_sensor_ = value; }

  void set_vehicle_tracking_sensor(binary_sensor::BinarySensor *value) { this->vehicle_tracking_sensor_ = value; }
  void set_vehicle_direction_sensor(text_sensor::TextSensor *value) { this->vehicle_direction_sensor_ = value; }
  void set_last_vehicle_event_sensor(text_sensor::TextSensor *value) { this->last_vehicle_event_sensor_ = value; }
  void set_vehicle_id_sensor(sensor::Sensor *value) { this->vehicle_id_sensor_ = value; }
  void set_vehicle_count_sensor(sensor::Sensor *value) { this->vehicle_count_sensor_ = value; }
  void set_vehicle_max_speed_sensor(sensor::Sensor *value) { this->vehicle_max_speed_sensor_ = value; }
  void set_vehicle_average_speed_sensor(sensor::Sensor *value) { this->vehicle_average_speed_sensor_ = value; }
  void set_vehicle_start_distance_sensor(sensor::Sensor *value) { this->vehicle_start_distance_sensor_ = value; }
  void set_vehicle_end_distance_sensor(sensor::Sensor *value) { this->vehicle_end_distance_sensor_ = value; }
  void set_vehicle_min_distance_sensor(sensor::Sensor *value) { this->vehicle_min_distance_sensor_ = value; }
  void set_vehicle_duration_sensor(sensor::Sensor *value) { this->vehicle_duration_sensor_ = value; }
  void set_vehicle_samples_sensor(sensor::Sensor *value) { this->vehicle_samples_sensor_ = value; }

  void set_target_count_sensor(sensor::Sensor *value) { this->target_count_sensor_ = value; }
  void set_max_simultaneous_targets_sensor(sensor::Sensor *value) { this->max_simultaneous_targets_sensor_ = value; }
  void set_multi_target_snapshot_sensor(text_sensor::TextSensor *value) { this->multi_target_snapshot_sensor_ = value; }
  void set_multi_target_active_sensor(binary_sensor::BinarySensor *value) { this->multi_target_active_sensor_ = value; }
  void set_multi_target_status_sensor(text_sensor::TextSensor *value) { this->multi_target_status_sensor_ = value; }
  void set_target_mode_status_sensor(text_sensor::TextSensor *value) { this->target_mode_status_sensor_ = value; }
  void set_raw_capture_status_sensor(text_sensor::TextSensor *value) { this->raw_capture_status_sensor_ = value; }

  void register_number(RadarParameter parameter, LDLNumber *entity) { this->numbers_[parameter] = entity; }
  void register_switch(RadarParameter parameter, LDLSwitch *entity) { this->switches_[parameter] = entity; }
  void register_select(RadarParameter parameter, LDLSelect *entity) { this->selects_[parameter] = entity; }
  void set_operating_mode_select(LDLOperatingModeSelect *value) { this->operating_mode_select_ = value; }

  void set_numeric_parameter(RadarParameter parameter, float value);
  void set_switch_parameter(RadarParameter parameter, bool value);
  void set_select_parameter(RadarParameter parameter, const std::string &value);
  void set_operating_mode(const std::string &value);
  void refresh_configuration();
  void factory_reset();
  void request_multi_target_snapshot();
  void request_target_mode(uint8_t mode);
  void start_raw_capture();

  // ConfigManagerListener
  void config_send_command(const std::string &command) override;
  void config_value_received(RadarParameter parameter, float value) override;
  void config_sync_changed(bool synchronized) override;
  void config_error(const std::string &message) override;

 protected:
  struct MultiTargetTrack;

  enum class RuntimeConfigState : uint8_t {
    IDLE_HEX,
    SWITCHING_TO_ASCII,
    RUNNING_ASCII,
    IDLE_ASCII,
    SWITCHING_TO_HEX
  };

  enum class RadarOperatingMode : uint8_t {
  ASCII_SINGLE_TARGET,
  HEX_MULTI_TARGET
  };
  
  void process_byte_(uint8_t byte);
  void process_text_byte_(uint8_t byte);
  void process_ingress_buffer_(uint32_t now_ms);
  void process_line_(std::string line);
  void publish_detection_(bool detected);
  void update_status_outputs_();
  void set_led_fault_(bool active);
  void publish_configuration_();
  void publish_error_(const std::string &message);
  void finish_vehicle_event_(uint32_t now_ms);
  void publish_vehicle_event_(const VehicleEvent &event);
  void publish_vehicle_event_mqtt_(const char *json);
  void start_multitarget_debug_();
  void send_hex_target_mode_command_(uint8_t mode);
  void publish_multitarget_raw_mqtt_(const std::string &payload, bool hex);
  void publish_multitarget_parsed_mqtt_(const std::string &payload);
  std::string resolve_mqtt_topic_(const std::string &configured, const char *suffix) const;
  static std::string escape_raw_bytes_(const std::string &value);
  static bool is_safe_text_line_(const std::string &value);
  bool is_artifact_measurement_(const Measurement &measurement);
  const MultiTargetTrack *find_primary_multi_target_track_(uint32_t now_ms) const;
  void publish_completed_track_(const MultiTargetTrack &track, uint32_t now_ms);
  void publish_vehicle_tracker_event_(const VehicleEvent &event);
  bool process_target_list_line_(const std::string &line, uint32_t now_ms);
  void request_target_list_(uint32_t now_ms);
  void publish_target_snapshot_(uint32_t now_ms);
  void flush_multi_target_batch_(uint32_t now_ms);
  void begin_runtime_ascii_mode_();
  void restore_runtime_hex_mode_();
  void queue_runtime_config_write_(
    RadarParameter parameter,
    float value);
  void request_operating_mode_(
    RadarOperatingMode mode);

  void apply_ascii_single_target_mode_();

  void apply_hex_multi_target_mode_();

  void reset_tracking_state_for_mode_change_();
  uint8_t count_visible_multi_target_tracks_(uint32_t now_ms) const;
  static bool parse_target_row_(const std::string &line, uint8_t &id, float &distance_m, float &speed_kmh, float &snr);
  bool validate_numeric_(RadarParameter parameter, float value, std::string &error) const;
  static const char *select_option_(RadarParameter parameter, int raw);
  void start_raw_capture_(uint32_t now_ms, const char *reason);
  void finish_raw_capture_(uint32_t now_ms);
  void start_hex_capture_(uint32_t now_ms, const char *reason, bool auto_restore_mode_1);
  void capture_hex_byte_(uint8_t byte, uint32_t now_ms);
  void flush_hex_block_();
  void send_target_mode_command_(uint8_t mode);
  void process_multi_target_byte_(uint8_t byte, uint32_t now_ms);
  void process_multi_target_frame_(const std::string &frame, uint32_t now_ms);
  static uint8_t xor_checksum_(const std::string &data);

  static constexpr size_t MAX_LINE_LENGTH = 512;
  static constexpr size_t MAX_INGRESS_LENGTH = 2048;

  UARTParser uart_parser_{};
  ConfigManager config_manager_;
  VehicleTracker vehicle_tracker_{};
  Mode2HexParser mode2_hex_parser_;

  std::string rx_buffer_{};
  std::string ingress_buffer_{};
  bool discard_until_newline_{false};
  bool debug_uart_{false};
  bool target_detected_{false};
  GPIOPin *red_output_pin_{nullptr};
  GPIOPin *green_output_pin_{nullptr};
  uint32_t led_red_afterglow_ms_{5000};
  uint32_t led_standby_timeout_ms_{60000};
  uint32_t led_afterglow_started_ms_{0};
  uint32_t led_idle_started_ms_{0};
  uint32_t led_fault_blink_ms_{0};
  bool led_afterglow_active_{false};
  bool led_fault_active_{false};
  bool led_fault_blink_state_{false};
  bool detection_state_initialized_{false};
  bool boot_read_started_{false};
  bool boot_mode_normalized_{false};
  bool multitarget_debug_started_{false};
  bool auto_mode0_requested_{false};
  uint32_t setup_ms_{0};
  uint32_t target_timeout_ms_{1500};
  bool artifact_filter_enabled_{true};
  float artifact_distance_m_{33.3f};
  float artifact_distance_tolerance_m_{0.6f};
  float artifact_speed_kmh_{89.0f};
  float artifact_speed_tolerance_kmh_{5.0f};
  GhostFilter ghost_filter_{};
  uint32_t artifact_filtered_count_{0};
  uint32_t last_artifact_log_ms_{0};
  uint32_t last_target_ms_{0};
  uint32_t boot_read_delay_ms_{2000};
  uint32_t command_gap_ms_{200};
  uint32_t command_timeout_ms_{1200};
  uint8_t command_retries_{1};
  bool multi_target_polling_{false};
  uint32_t multi_target_poll_interval_ms_{5000};
  uint32_t last_multi_target_poll_ms_{0};
  uint32_t target_list_started_ms_{0};
  uint32_t target_list_last_row_ms_{0};
  uint8_t target_list_expected_{0};
  uint8_t target_list_count_{0};
  uint8_t max_simultaneous_targets_{0};
  bool target_list_receiving_{false};
  bool target_list_manual_request_{false};
  int last_published_target_count_{-1};
  uint8_t last_published_max_targets_{255};
  bool last_published_multi_active_{false};
  bool multi_active_has_state_{false};
  std::string last_published_snapshot_{};
  std::string last_multi_target_status_{};
  uint32_t raw_capture_duration_ms_{8000};
  uint32_t raw_capture_max_bytes_{4096};
  uint32_t raw_capture_started_ms_{0};
  uint32_t raw_capture_line_count_{0};
  uint32_t raw_capture_byte_count_{0};
  uint32_t raw_capture_block_count_{0};
  bool raw_capture_active_{false};
  bool raw_capture_hex_mode_{false};
  bool raw_capture_auto_restore_mode_1_{false};
  std::array<uint8_t, 32> raw_hex_block_{};
  uint8_t raw_hex_block_length_{0};
  int8_t requested_target_mode_{-1};
  bool multi_target_stream_mode_{false};
  std::string multi_target_stream_buffer_{};
  uint32_t multi_target_valid_frames_{0};
  uint32_t multi_target_checksum_errors_{0};
  uint32_t multi_target_target_frames_{0};
  uint32_t multi_target_empty_frames_{0};
  uint32_t multi_target_last_seen_ms_{0};
  uint32_t multi_target_hold_time_ms_{500};
  uint8_t multi_target_confirmation_frames_{3};
  uint32_t multi_target_confirmation_window_ms_{600};
  float multi_target_confirmation_tolerance_m_{3.0f};
  float multi_target_min_confirmation_movement_m_{1.0f};
  bool auto_enable_multi_target_after_sync_{false};
  uint8_t multi_target_candidate_frames_{0};
  uint32_t multi_target_candidate_first_ms_{0};
  uint32_t multi_target_candidate_last_ms_{0};
  float multi_target_candidate_first_distance_{0.0f};
  float multi_target_candidate_last_distance_{0.0f};
  bool multi_target_candidate_confirmed_{false};
  uint32_t ingress_heada_frames_{0};
  uint32_t ingress_text_bytes_{0};
  uint32_t ingress_resyncs_{0};

  struct TargetPoint {
    bool valid{false};
    uint8_t id{0};
    uint16_t track_id{0};
    float distance_m{0.0f};
    float speed_kmh{0.0f};
    float snr{0.0f};
    uint32_t last_seen_ms{0};
  };
  std::array<TargetPoint, 9> target_points_{};

  struct MultiTargetTrack {
    bool active{false};
    uint16_t id{0};

    float distance_m{0.0f};
    float speed_kmh{0.0f};
    float snr{0.0f};

    uint32_t first_seen_ms{0};
    uint32_t last_seen_ms{0};

    uint8_t last_radar_slot{255};
    uint16_t sample_count{0};

    // Fahrzeugstatistik
    float start_distance_m{0.0f};
    float end_distance_m{0.0f};
    float minimum_distance_m{0.0f};

    float maximum_speed_kmh{0.0f};
    float speed_sum_kmh{0.0f};
  };

std::array<MultiTargetTrack, 9> multi_target_tracks_{};
uint16_t next_multi_target_track_id_{1};
uint16_t primary_vehicle_track_id_{0};
std::array<bool, 9> pending_target_slots_{};
bool pending_target_batch_active_{false};
uint32_t pending_target_batch_last_ms_{0};
uint32_t pending_target_batch_started_ms_{0};

RuntimeConfigState runtime_config_state_{
    RuntimeConfigState::IDLE_HEX};

RadarOperatingMode operating_mode_{
    RadarOperatingMode::HEX_MULTI_TARGET};

RadarOperatingMode requested_operating_mode_{
    RadarOperatingMode::HEX_MULTI_TARGET};

bool operating_mode_change_pending_{false};
uint32_t operating_mode_change_started_ms_{0};

uint32_t runtime_config_state_started_ms_{0};
bool runtime_config_command_queued_{false};

RadarParameter runtime_config_parameter_{};
float runtime_config_value_{0.0f};
bool runtime_config_write_pending_{false};

  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  binary_sensor::BinarySensor *detected_sensor_{nullptr};
  binary_sensor::BinarySensor *config_synchronized_sensor_{nullptr};
  text_sensor::TextSensor *configuration_sensor_{nullptr};
  text_sensor::TextSensor *last_config_error_sensor_{nullptr};
  text_sensor::TextSensor *last_cli_command_sensor_{nullptr};

  binary_sensor::BinarySensor *vehicle_tracking_sensor_{nullptr};
  text_sensor::TextSensor *vehicle_direction_sensor_{nullptr};
  text_sensor::TextSensor *last_vehicle_event_sensor_{nullptr};
  sensor::Sensor *vehicle_id_sensor_{nullptr};
  sensor::Sensor *vehicle_count_sensor_{nullptr};
  sensor::Sensor *vehicle_max_speed_sensor_{nullptr};
  sensor::Sensor *vehicle_average_speed_sensor_{nullptr};
  sensor::Sensor *vehicle_start_distance_sensor_{nullptr};
  sensor::Sensor *vehicle_end_distance_sensor_{nullptr};
  sensor::Sensor *vehicle_min_distance_sensor_{nullptr};
  sensor::Sensor *vehicle_duration_sensor_{nullptr};
  sensor::Sensor *vehicle_samples_sensor_{nullptr};
  sensor::Sensor *target_count_sensor_{nullptr};
  sensor::Sensor *max_simultaneous_targets_sensor_{nullptr};
  text_sensor::TextSensor *multi_target_snapshot_sensor_{nullptr};
  binary_sensor::BinarySensor *multi_target_active_sensor_{nullptr};
  text_sensor::TextSensor *multi_target_status_sensor_{nullptr};
  text_sensor::TextSensor *target_mode_status_sensor_{nullptr};
  text_sensor::TextSensor *raw_capture_status_sensor_{nullptr};
  uint32_t completed_vehicle_count_{0};
  bool mqtt_event_enabled_{true};
  std::string mqtt_event_topic_{};
  uint8_t mqtt_event_qos_{0};
  bool mqtt_event_retain_{false};
  std::string multitarget_debug_mode_{"off"};
  bool multitarget_raw_mqtt_enabled_{true};
  std::string multitarget_raw_mqtt_topic_{};
  std::string multitarget_parsed_mqtt_topic_{};
  uint8_t multitarget_mqtt_qos_{0};
  std::array<uint8_t, 32> multitarget_hex_mqtt_block_{};
  uint8_t multitarget_hex_mqtt_block_length_{0};
  uint32_t multitarget_hex_last_byte_ms_{0};

  LDLOperatingModeSelect *operating_mode_select_{nullptr};
  std::map<RadarParameter, LDLNumber *> numbers_{};
  std::map<LEDSetting, LDLLEDNumber *> led_numbers_{};
  std::map<RadarParameter, LDLSwitch *> switches_{};
  std::map<RadarParameter, LDLSelect *> selects_{};
};

}  // namespace ldl508pro
}  // namespace esphome
