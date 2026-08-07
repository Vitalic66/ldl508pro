import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import (
    binary_sensor,
    button,
    number,
    select,
    sensor,
    switch,
    text_sensor,
    uart,
)
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_OCCUPANCY,
    DEVICE_CLASS_SPEED,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_KILOMETER_PER_HOUR,
    UNIT_METER,
    UNIT_SECOND,
)

CODEOWNERS = []
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "button", "number", "select", "sensor", "switch", "text_sensor"]

CONF_DEBUG_UART = "debug_uart"
CONF_TARGET_TIMEOUT = "target_timeout"
CONF_BOOT_READ_DELAY = "boot_read_delay"
CONF_COMMAND_GAP = "command_gap"
CONF_COMMAND_TIMEOUT = "command_timeout"
CONF_COMMAND_RETRIES = "command_retries"
CONF_MULTI_TARGET_POLLING = "multi_target_polling"
CONF_MULTI_TARGET_POLL_INTERVAL = "multi_target_poll_interval"
CONF_MULTI_TARGET_HOLD_TIME = "multi_target_hold_time"
CONF_MULTI_TARGET_CONFIRMATION_FRAMES = "multi_target_confirmation_frames"
CONF_MULTI_TARGET_CONFIRMATION_WINDOW = "multi_target_confirmation_window"
CONF_MULTI_TARGET_CONFIRMATION_TOLERANCE = "multi_target_confirmation_tolerance"
CONF_MULTI_TARGET_MIN_CONFIRMATION_MOVEMENT = "multi_target_min_confirmation_movement"
CONF_AUTO_ENABLE_MULTI_TARGET_AFTER_SYNC = "auto_enable_multi_target_after_sync"
CONF_RAW_CAPTURE_DURATION = "raw_capture_duration"
CONF_RAW_CAPTURE_MAX_BYTES = "raw_capture_max_bytes"
CONF_RED_OUTPUT_PIN = "red_output_pin"
CONF_GREEN_OUTPUT_PIN = "green_output_pin"
CONF_CARPORT_BARRIER_PIN = "carport_barrier_pin"
CONF_WARNING_OUTPUT_PIN = "warning_output_pin"
CONF_CARPORT_CLEAR_CONFIRM = "carport_clear_confirm"
CONF_ARTIFACT_FILTER = "artifact_filter"
CONF_ARTIFACT_DISTANCE = "artifact_distance"
CONF_ARTIFACT_DISTANCE_TOLERANCE = "artifact_distance_tolerance"
CONF_ARTIFACT_SPEED = "artifact_speed"
CONF_ARTIFACT_SPEED_TOLERANCE = "artifact_speed_tolerance"
CONF_LED_RED_AFTERGLOW = "led_red_afterglow"
CONF_LED_STANDBY_TIMEOUT = "led_standby_timeout"
CONF_MQTT_EVENT_ENABLED = "mqtt_event_enabled"
CONF_MQTT_EVENT_TOPIC = "mqtt_event_topic"
CONF_MQTT_EVENT_QOS = "mqtt_event_qos"
CONF_MQTT_EVENT_RETAIN = "mqtt_event_retain"
CONF_MULTITARGET_DEBUG_MODE = "multitarget_debug_mode"
CONF_MULTITARGET_RAW_MQTT_ENABLED = "multitarget_raw_mqtt_enabled"
CONF_MULTITARGET_RAW_MQTT_TOPIC = "multitarget_raw_mqtt_topic"
CONF_MULTITARGET_PARSED_MQTT_TOPIC = "multitarget_parsed_mqtt_topic"
CONF_MULTITARGET_MQTT_QOS = "multitarget_mqtt_qos"

CONF_DISTANCE = "distance"
CONF_SPEED = "speed"
CONF_DETECTED = "detected"
CONF_CARPORT_BEAM_CLEAR = "carport_beam_clear"
CONF_CARPORT_OCCUPIED = "carport_occupied"
CONF_CARPORT_DEPARTURE = "carport_departure"
CONF_CONFIG_SYNCHRONIZED = "config_synchronized"
CONF_CONFIGURATION = "configuration"
CONF_LAST_CONFIG_ERROR = "last_config_error"
CONF_LAST_CLI_COMMAND = "last_cli_command"

CONF_VEHICLE_TRACKING = "vehicle_tracking"
CONF_VEHICLE_DIRECTION = "vehicle_direction"
CONF_LAST_VEHICLE_EVENT = "last_vehicle_event"
CONF_VEHICLE_ID = "vehicle_id"
CONF_VEHICLE_COUNT = "vehicle_count"
CONF_VEHICLE_MAX_SPEED = "vehicle_max_speed"
CONF_VEHICLE_AVERAGE_SPEED = "vehicle_average_speed"
CONF_VEHICLE_START_DISTANCE = "vehicle_start_distance"
CONF_VEHICLE_END_DISTANCE = "vehicle_end_distance"
CONF_VEHICLE_MIN_DISTANCE = "vehicle_min_distance"
CONF_VEHICLE_DURATION = "vehicle_duration"
CONF_VEHICLE_SAMPLES = "vehicle_samples"
CONF_TARGET_COUNT = "target_count"
CONF_MAX_SIMULTANEOUS_TARGETS = "max_simultaneous_targets"
CONF_MULTI_TARGET_SNAPSHOT = "multi_target_snapshot"
CONF_MULTI_TARGET_ACTIVE = "multi_target_active"
CONF_MULTI_TARGET_STATUS = "multi_target_status"
CONF_REQUEST_MULTI_TARGET_SNAPSHOT = "request_multi_target_snapshot"
CONF_TARGET_MODE_STATUS = "target_mode_status"
CONF_RAW_CAPTURE_STATUS = "raw_capture_status"
CONF_TARGET_MODE_0 = "target_mode_0"
CONF_TARGET_MODE_1 = "target_mode_1"
CONF_START_RAW_CAPTURE = "start_raw_capture"

CONF_CFAR = "cfar"
CONF_MAX_FRAMERATE = "max_framerate"
CONF_STATIC_DETECTION = "static_detection"
CONF_SPEED_LIMIT_ENABLED = "speed_limit_enabled"
CONF_SPEED_LIMIT_HIGH = "speed_limit_high"
CONF_SPEED_LIMIT_LOW = "speed_limit_low"
CONF_POWER_MODE = "power_mode"
CONF_DOPPLER_FILTER = "doppler_filter"
CONF_OPERATING_MODE = "operating_mode"
CONF_DISTANCE_LIMIT_HIGH = "distance_limit_high"
CONF_DISTANCE_LIMIT_LOW = "distance_limit_low"
CONF_SPEED_THRESHOLD = "speed_threshold"
CONF_DURATION = "duration"
CONF_SNR_FILTER = "snr_filter"
CONF_REFRESH_SETTINGS = "refresh_settings"
CONF_FACTORY_RESET = "factory_reset"

ldl508pro_ns = cg.esphome_ns.namespace("ldl508pro")
LDL508PROComponent = ldl508pro_ns.class_(
    "LDL508PROComponent", cg.Component, uart.UARTDevice
)
LDLNumber = ldl508pro_ns.class_("LDLNumber", number.Number)
LDLLEDNumber = ldl508pro_ns.class_("LDLLEDNumber", number.Number)
LEDSetting = ldl508pro_ns.enum("LEDSetting", is_class=True)
LDLSwitch = ldl508pro_ns.class_("LDLSwitch", switch.Switch)
LDLSelect = ldl508pro_ns.class_("LDLSelect", select.Select)
LDLOperatingModeSelect = ldl508pro_ns.class_("LDLOperatingModeSelect", select.Select,)
LDLButton = ldl508pro_ns.class_("LDLButton", button.Button)
RadarParameter = ldl508pro_ns.enum("RadarParameter")
LDLButtonAction = ldl508pro_ns.enum("LDLButtonAction")

NUMBER_PARAMETERS = {
    CONF_CFAR: (RadarParameter.CFAR, 0.0, 100.0, 0.1),
    CONF_MAX_FRAMERATE: (RadarParameter.MAX_FRAMERATE, 1.0, 100.0, 1.0),
    CONF_SPEED_LIMIT_HIGH: (RadarParameter.SPEED_LIMIT_HIGH, 0.0, 180.0, 0.1),
    CONF_SPEED_LIMIT_LOW: (RadarParameter.SPEED_LIMIT_LOW, 0.0, 180.0, 0.1),
    CONF_DISTANCE_LIMIT_HIGH: (RadarParameter.DISTANCE_LIMIT_HIGH, 0.0, 140.0, 0.1),
    CONF_DISTANCE_LIMIT_LOW: (RadarParameter.DISTANCE_LIMIT_LOW, 0.0, 140.0, 0.1),
    CONF_SPEED_THRESHOLD: (RadarParameter.SPEED_THRESHOLD, 0.0, 180.0, 1.0),
    CONF_DURATION: (RadarParameter.DURATION, 0.0, 60.0, 1.0),
    CONF_SNR_FILTER: (RadarParameter.SNR_FILTER, 0.0, 100.0, 0.1),
}

def _validate_status_output_pair(config):
    red = CONF_RED_OUTPUT_PIN in config
    green = CONF_GREEN_OUTPUT_PIN in config
    if red != green:
        raise cv.Invalid("red_output_pin and green_output_pin must be configured together")
    return config


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LDL508PROComponent),
            cv.Optional(CONF_DEBUG_UART, default=False): cv.boolean,
            cv.Optional(CONF_TARGET_TIMEOUT, default="1500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_BOOT_READ_DELAY, default="2s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_COMMAND_GAP, default="200ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_COMMAND_TIMEOUT, default="1200ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_COMMAND_RETRIES, default=1): cv.int_range(min=0, max=5),
            cv.Optional(CONF_MULTI_TARGET_POLLING, default=False): cv.boolean,
            cv.Optional(CONF_MULTI_TARGET_POLL_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MULTI_TARGET_HOLD_TIME, default="500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MULTI_TARGET_CONFIRMATION_FRAMES, default=3): cv.int_range(min=1, max=10),
            cv.Optional(CONF_MULTI_TARGET_CONFIRMATION_WINDOW, default="600ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MULTI_TARGET_CONFIRMATION_TOLERANCE, default=3.0): cv.float_range(min=0.0, max=25.0),
            cv.Optional(CONF_MULTI_TARGET_MIN_CONFIRMATION_MOVEMENT, default=1.0): cv.float_range(min=0.0, max=25.0),
            cv.Optional(CONF_AUTO_ENABLE_MULTI_TARGET_AFTER_SYNC, default=False): cv.boolean,
            cv.Optional(CONF_RAW_CAPTURE_DURATION, default="8s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_RAW_CAPTURE_MAX_BYTES, default=4096): cv.int_range(min=256, max=65536),
            cv.Optional(CONF_RED_OUTPUT_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_GREEN_OUTPUT_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_CARPORT_BARRIER_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_WARNING_OUTPUT_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_CARPORT_CLEAR_CONFIRM, default="2s",): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ARTIFACT_FILTER, default=True): cv.boolean,
            cv.Optional(CONF_ARTIFACT_DISTANCE, default=33.3): cv.float_range(min=0.0, max=140.0),
            cv.Optional(CONF_ARTIFACT_DISTANCE_TOLERANCE, default=0.6): cv.float_range(min=0.0, max=20.0),
            cv.Optional(CONF_ARTIFACT_SPEED, default=89.0): cv.float_range(min=0.0, max=180.0),
            cv.Optional(CONF_ARTIFACT_SPEED_TOLERANCE, default=5.0): cv.float_range(min=0.0, max=30.0),
            cv.Optional(CONF_MQTT_EVENT_ENABLED, default=True): cv.boolean,
            cv.Optional(CONF_MQTT_EVENT_TOPIC, default=""): cv.string,
            cv.Optional(CONF_MQTT_EVENT_QOS, default=0): cv.int_range(min=0, max=2),
            cv.Optional(CONF_MQTT_EVENT_RETAIN, default=False): cv.boolean,
            cv.Optional(CONF_MULTITARGET_DEBUG_MODE, default="off"): cv.one_of("off", "ascii", "hex", lower=True),
            cv.Optional(CONF_MULTITARGET_RAW_MQTT_ENABLED, default=True): cv.boolean,
            cv.Optional(CONF_MULTITARGET_RAW_MQTT_TOPIC, default=""): cv.string,
            cv.Optional(CONF_MULTITARGET_PARSED_MQTT_TOPIC, default=""): cv.string,
            cv.Optional(CONF_MULTITARGET_MQTT_QOS, default=0): cv.int_range(min=0, max=2),
            cv.Optional(CONF_LED_RED_AFTERGLOW): number.number_schema(
                LDLLEDNumber, entity_category=ENTITY_CATEGORY_CONFIG, unit_of_measurement=UNIT_SECOND
            ),
            cv.Optional(CONF_LED_STANDBY_TIMEOUT): number.number_schema(
                LDLLEDNumber, entity_category=ENTITY_CATEGORY_CONFIG, unit_of_measurement=UNIT_SECOND
            ),
            cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_METER,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_DISTANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SPEED): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_SPEED,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_DETECTED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OCCUPANCY,
            ),
            cv.Optional(CONF_CARPORT_BEAM_CLEAR): binary_sensor.binary_sensor_schema(
            ),
            cv.Optional(CONF_CARPORT_OCCUPIED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OCCUPANCY,
            ),
            cv.Optional(CONF_CARPORT_DEPARTURE): binary_sensor.binary_sensor_schema(
            ),
            cv.Optional(CONF_CONFIG_SYNCHRONIZED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_CONFIGURATION): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_LAST_CONFIG_ERROR): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_LAST_CLI_COMMAND): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_VEHICLE_TRACKING): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OCCUPANCY,
            ),
            cv.Optional(CONF_VEHICLE_DIRECTION): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_LAST_VEHICLE_EVENT): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_VEHICLE_ID): sensor.sensor_schema(
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_VEHICLE_COUNT): sensor.sensor_schema(
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_VEHICLE_MAX_SPEED): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_SPEED,
            ),
            cv.Optional(CONF_VEHICLE_AVERAGE_SPEED): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_SPEED,
            ),
            cv.Optional(CONF_VEHICLE_START_DISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_METER,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_DISTANCE,
            ),
            cv.Optional(CONF_VEHICLE_END_DISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_METER,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_DISTANCE,
            ),
            cv.Optional(CONF_VEHICLE_MIN_DISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_METER,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_DISTANCE,
            ),
            cv.Optional(CONF_VEHICLE_DURATION): sensor.sensor_schema(
                unit_of_measurement=UNIT_SECOND,
                accuracy_decimals=1,
            ),
            cv.Optional(CONF_VEHICLE_SAMPLES): sensor.sensor_schema(
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_TARGET_COUNT): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_MAX_SIMULTANEOUS_TARGETS): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_MULTI_TARGET_SNAPSHOT): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_MULTI_TARGET_ACTIVE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OCCUPANCY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_MULTI_TARGET_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_TARGET_MODE_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RAW_CAPTURE_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_CFAR): number.number_schema(
                LDLNumber, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_MAX_FRAMERATE): number.number_schema(
                LDLNumber, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_SPEED_LIMIT_HIGH): number.number_schema(
                LDLNumber, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_SPEED_LIMIT_LOW): number.number_schema(
                LDLNumber, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_DISTANCE_LIMIT_HIGH): number.number_schema(
                LDLNumber, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_DISTANCE_LIMIT_LOW): number.number_schema(
                LDLNumber, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_SPEED_THRESHOLD): number.number_schema(
                LDLNumber, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_DURATION): number.number_schema(
                LDLNumber, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_SNR_FILTER): number.number_schema(
                LDLNumber, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_STATIC_DETECTION): switch.switch_schema(
                LDLSwitch, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_SPEED_LIMIT_ENABLED): switch.switch_schema(
                LDLSwitch, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_POWER_MODE): select.select_schema(
                LDLSelect, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_DOPPLER_FILTER): select.select_schema(
                LDLSelect, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_OPERATING_MODE): select.select_schema(
                LDLOperatingModeSelect, entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_REFRESH_SETTINGS): button.button_schema(
                LDLButton, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_FACTORY_RESET): button.button_schema(
                LDLButton, entity_category=ENTITY_CATEGORY_CONFIG
            ),
            cv.Optional(CONF_REQUEST_MULTI_TARGET_SNAPSHOT): button.button_schema(
                LDLButton, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
            cv.Optional(CONF_TARGET_MODE_0): button.button_schema(
                LDLButton, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
            cv.Optional(CONF_TARGET_MODE_1): button.button_schema(
                LDLButton, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
            cv.Optional(CONF_START_RAW_CAPTURE): button.button_schema(
                LDLButton, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)
CONFIG_SCHEMA = cv.All(CONFIG_SCHEMA, _validate_status_output_pair)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_debug_uart(config[CONF_DEBUG_UART]))
    cg.add(var.set_target_timeout(config[CONF_TARGET_TIMEOUT].total_milliseconds))
    cg.add(var.set_boot_read_delay(config[CONF_BOOT_READ_DELAY].total_milliseconds))
    cg.add(var.set_command_gap(config[CONF_COMMAND_GAP].total_milliseconds))
    cg.add(var.set_command_timeout(config[CONF_COMMAND_TIMEOUT].total_milliseconds))
    cg.add(var.set_command_retries(config[CONF_COMMAND_RETRIES]))
    cg.add(var.set_multi_target_polling(config[CONF_MULTI_TARGET_POLLING]))
    cg.add(var.set_multi_target_poll_interval(config[CONF_MULTI_TARGET_POLL_INTERVAL].total_milliseconds))
    cg.add(var.set_multi_target_hold_time(config[CONF_MULTI_TARGET_HOLD_TIME].total_milliseconds))
    cg.add(var.set_multi_target_confirmation_frames(config[CONF_MULTI_TARGET_CONFIRMATION_FRAMES]))
    cg.add(var.set_multi_target_confirmation_window(config[CONF_MULTI_TARGET_CONFIRMATION_WINDOW].total_milliseconds))
    cg.add(var.set_multi_target_confirmation_tolerance(config[CONF_MULTI_TARGET_CONFIRMATION_TOLERANCE]))
    cg.add(var.set_multi_target_min_confirmation_movement(config[CONF_MULTI_TARGET_MIN_CONFIRMATION_MOVEMENT]))
    cg.add(var.set_auto_enable_multi_target_after_sync(config[CONF_AUTO_ENABLE_MULTI_TARGET_AFTER_SYNC]))
    cg.add(var.set_raw_capture_duration(config[CONF_RAW_CAPTURE_DURATION].total_milliseconds))
    cg.add(var.set_raw_capture_max_bytes(config[CONF_RAW_CAPTURE_MAX_BYTES]))
    cg.add(var.set_artifact_filter(config[CONF_ARTIFACT_FILTER]))
    cg.add(var.set_artifact_distance(config[CONF_ARTIFACT_DISTANCE]))
    cg.add(var.set_artifact_distance_tolerance(config[CONF_ARTIFACT_DISTANCE_TOLERANCE]))
    cg.add(var.set_artifact_speed(config[CONF_ARTIFACT_SPEED]))
    cg.add(var.set_artifact_speed_tolerance(config[CONF_ARTIFACT_SPEED_TOLERANCE]))
    cg.add(var.set_mqtt_event_enabled(config[CONF_MQTT_EVENT_ENABLED]))
    cg.add(var.set_mqtt_event_topic(config[CONF_MQTT_EVENT_TOPIC]))
    cg.add(var.set_mqtt_event_qos(config[CONF_MQTT_EVENT_QOS]))
    cg.add(var.set_mqtt_event_retain(config[CONF_MQTT_EVENT_RETAIN]))
    cg.add(var.set_multitarget_debug_mode(config[CONF_MULTITARGET_DEBUG_MODE]))
    cg.add(var.set_multitarget_raw_mqtt_enabled(config[CONF_MULTITARGET_RAW_MQTT_ENABLED]))
    cg.add(var.set_multitarget_raw_mqtt_topic(config[CONF_MULTITARGET_RAW_MQTT_TOPIC]))
    cg.add(var.set_multitarget_parsed_mqtt_topic(config[CONF_MULTITARGET_PARSED_MQTT_TOPIC]))
    cg.add(var.set_multitarget_mqtt_qos(config[CONF_MULTITARGET_MQTT_QOS]))

    if CONF_RED_OUTPUT_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_RED_OUTPUT_PIN])
        cg.add(var.set_red_output_pin(pin))
    if CONF_GREEN_OUTPUT_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_GREEN_OUTPUT_PIN])
        cg.add(var.set_green_output_pin(pin))

    led_number_settings = {
        CONF_LED_RED_AFTERGLOW: (LEDSetting.RED_AFTERGLOW, 0.0, 60.0, 1.0),
        CONF_LED_STANDBY_TIMEOUT: (LEDSetting.STANDBY_TIMEOUT, 0.0, 3600.0, 5.0),
    }
    for key, (setting, min_value, max_value, step) in led_number_settings.items():
        if key not in config:
            continue
        number_config = config[key]
        entity = cg.new_Pvariable(number_config[CONF_ID], var, setting)
        await number.register_number(
            entity, number_config, min_value=min_value, max_value=max_value, step=step
        )
        cg.add(var.register_led_number(setting, entity))

    if CONF_CARPORT_BARRIER_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_CARPORT_BARRIER_PIN])
        cg.add(var.set_carport_barrier_pin(pin))

    cg.add(var.set_carport_clear_confirm_ms(config[CONF_CARPORT_CLEAR_CONFIRM].total_milliseconds))

    if CONF_WARNING_OUTPUT_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_WARNING_OUTPUT_PIN])
        cg.add(var.set_warning_output_pin(pin))

    if distance_config := config.get(CONF_DISTANCE):
        entity = await sensor.new_sensor(distance_config)
        cg.add(var.set_distance_sensor(entity))
    if speed_config := config.get(CONF_SPEED):
        entity = await sensor.new_sensor(speed_config)
        cg.add(var.set_speed_sensor(entity))
    if detected_config := config.get(CONF_DETECTED):
        entity = await binary_sensor.new_binary_sensor(detected_config)
        cg.add(var.set_detected_sensor(entity))
    if beam_config := config.get(CONF_CARPORT_BEAM_CLEAR):
        entity = await binary_sensor.new_binary_sensor(beam_config)
        cg.add(var.set_carport_beam_clear_sensor(entity))
    if occupied_config := config.get(CONF_CARPORT_OCCUPIED):
        entity = await binary_sensor.new_binary_sensor(occupied_config)
        cg.add(var.set_carport_occupied_sensor(entity))
    if departure_config := config.get(CONF_CARPORT_DEPARTURE):
        entity = await binary_sensor.new_binary_sensor(departure_config)
        cg.add(var.set_carport_departure_sensor(entity))
    if sync_config := config.get(CONF_CONFIG_SYNCHRONIZED):
        entity = await binary_sensor.new_binary_sensor(sync_config)
        cg.add(var.set_config_synchronized_sensor(entity))
    if configuration_config := config.get(CONF_CONFIGURATION):
        entity = await text_sensor.new_text_sensor(configuration_config)
        cg.add(var.set_configuration_sensor(entity))
    if error_config := config.get(CONF_LAST_CONFIG_ERROR):
        entity = await text_sensor.new_text_sensor(error_config)
        cg.add(var.set_last_config_error_sensor(entity))
    if cli_config := config.get(CONF_LAST_CLI_COMMAND):
        entity = await text_sensor.new_text_sensor(cli_config)
        cg.add(var.set_last_cli_command_sensor(entity))

    if tracker_config := config.get(CONF_VEHICLE_TRACKING):
        entity = await binary_sensor.new_binary_sensor(tracker_config)
        cg.add(var.set_vehicle_tracking_sensor(entity))
    if direction_config := config.get(CONF_VEHICLE_DIRECTION):
        entity = await text_sensor.new_text_sensor(direction_config)
        cg.add(var.set_vehicle_direction_sensor(entity))
    if event_config := config.get(CONF_LAST_VEHICLE_EVENT):
        entity = await text_sensor.new_text_sensor(event_config)
        cg.add(var.set_last_vehicle_event_sensor(entity))

    tracker_sensors = {
        CONF_VEHICLE_ID: var.set_vehicle_id_sensor,
        CONF_VEHICLE_COUNT: var.set_vehicle_count_sensor,
        CONF_VEHICLE_MAX_SPEED: var.set_vehicle_max_speed_sensor,
        CONF_VEHICLE_AVERAGE_SPEED: var.set_vehicle_average_speed_sensor,
        CONF_VEHICLE_START_DISTANCE: var.set_vehicle_start_distance_sensor,
        CONF_VEHICLE_END_DISTANCE: var.set_vehicle_end_distance_sensor,
        CONF_VEHICLE_MIN_DISTANCE: var.set_vehicle_min_distance_sensor,
        CONF_VEHICLE_DURATION: var.set_vehicle_duration_sensor,
        CONF_VEHICLE_SAMPLES: var.set_vehicle_samples_sensor,
        CONF_TARGET_COUNT: var.set_target_count_sensor,
        CONF_MAX_SIMULTANEOUS_TARGETS: var.set_max_simultaneous_targets_sensor,
    }
    for key, setter in tracker_sensors.items():
        if sensor_config := config.get(key):
            entity = await sensor.new_sensor(sensor_config)
            cg.add(setter(entity))

    if snapshot_config := config.get(CONF_MULTI_TARGET_SNAPSHOT):
        entity = await text_sensor.new_text_sensor(snapshot_config)
        cg.add(var.set_multi_target_snapshot_sensor(entity))
    if active_config := config.get(CONF_MULTI_TARGET_ACTIVE):
        entity = await binary_sensor.new_binary_sensor(active_config)
        cg.add(var.set_multi_target_active_sensor(entity))
    if status_config := config.get(CONF_MULTI_TARGET_STATUS):
        entity = await text_sensor.new_text_sensor(status_config)
        cg.add(var.set_multi_target_status_sensor(entity))
    if status_config := config.get(CONF_TARGET_MODE_STATUS):
        entity = await text_sensor.new_text_sensor(status_config)
        cg.add(var.set_target_mode_status_sensor(entity))
    if status_config := config.get(CONF_RAW_CAPTURE_STATUS):
        entity = await text_sensor.new_text_sensor(status_config)
        cg.add(var.set_raw_capture_status_sensor(entity))

    for key, (parameter, min_value, max_value, step) in NUMBER_PARAMETERS.items():
        if key not in config:
            continue
        number_config = config[key]
        entity = cg.new_Pvariable(number_config[CONF_ID], var, parameter)
        await number.register_number(
            entity,
            number_config,
            min_value=min_value,
            max_value=max_value,
            step=step,
        )
        cg.add(var.register_number(parameter, entity))

    if switch_config := config.get(CONF_STATIC_DETECTION):
        entity = cg.new_Pvariable(
            switch_config[CONF_ID], var, RadarParameter.STATIC_DETECTION
        )
        await switch.register_switch(entity, switch_config)
        cg.add(var.register_switch(RadarParameter.STATIC_DETECTION, entity))

    if switch_config := config.get(CONF_SPEED_LIMIT_ENABLED):
        entity = cg.new_Pvariable(
            switch_config[CONF_ID], var, RadarParameter.SPEED_LIMIT_ENABLED
        )
        await switch.register_switch(entity, switch_config)
        cg.add(var.register_switch(RadarParameter.SPEED_LIMIT_ENABLED, entity))

    if select_config := config.get(CONF_POWER_MODE):
        entity = cg.new_Pvariable(
            select_config[CONF_ID], var, RadarParameter.POWER_MODE
        )
        await select.register_select(
            entity,
            select_config,
            options=["Volle Geschwindigkeit", "Normal", "Ultra-Low-Power"],
        )
        cg.add(var.register_select(RadarParameter.POWER_MODE, entity))

    if select_config := config.get(CONF_DOPPLER_FILTER):
        entity = cg.new_Pvariable(
            select_config[CONF_ID], var, RadarParameter.DOPPLER_FILTER
        )
        await select.register_select(
            entity,
            select_config,
            options=[
                "Beide Richtungen",
                "Nur kommende Fahrzeuge",
                "Nur sich entfernende Fahrzeuge",
            ],
        )
        cg.add(var.register_select(RadarParameter.DOPPLER_FILTER, entity))

    if select_config := config.get(CONF_OPERATING_MODE):
        entity = cg.new_Pvariable(
            select_config[CONF_ID],
            var,
        )

        await select.register_select(
            entity,
            select_config,
            options=[
                "Mehrziel (Empfohlen)",
                "Einzelziel (Kompatibilität)",
            ],
        )

        cg.add(
            var.set_operating_mode_select(entity)
        )

    if button_config := config.get(CONF_REFRESH_SETTINGS):
        entity = cg.new_Pvariable(
            button_config[CONF_ID], var, LDLButtonAction.REFRESH_CONFIGURATION
        )
        await button.register_button(entity, button_config)

    if button_config := config.get(CONF_FACTORY_RESET):
        entity = cg.new_Pvariable(
            button_config[CONF_ID], var, LDLButtonAction.FACTORY_RESET
        )
        await button.register_button(entity, button_config)

    if button_config := config.get(CONF_REQUEST_MULTI_TARGET_SNAPSHOT):
        entity = cg.new_Pvariable(
            button_config[CONF_ID], var, LDLButtonAction.REQUEST_MULTI_TARGET_SNAPSHOT
        )
        await button.register_button(entity, button_config)

    if button_config := config.get(CONF_TARGET_MODE_0):
        entity = cg.new_Pvariable(button_config[CONF_ID], var, LDLButtonAction.TARGET_MODE_0)
        await button.register_button(entity, button_config)

    if button_config := config.get(CONF_TARGET_MODE_1):
        entity = cg.new_Pvariable(button_config[CONF_ID], var, LDLButtonAction.TARGET_MODE_1)
        await button.register_button(entity, button_config)

    if button_config := config.get(CONF_START_RAW_CAPTURE):
        entity = cg.new_Pvariable(button_config[CONF_ID], var, LDLButtonAction.START_RAW_CAPTURE)
        await button.register_button(entity, button_config)


