# LDL508PRO Radar for ESPHome

An enhanced ESPHome component for the **LDL508PRO Doppler radar sensor** with support for both **Single Target** and **Multi Target** operation.

Originally based on the standard LDL508PRO ESPHome component, this project has evolved into an advanced radar platform featuring improved tracking, vehicle statistics, MQTT integration and runtime configuration.

---

# Features

## Dual Operating Modes

### Multi Target (Recommended)

- Native Mode 2 (HEX protocol)
- Simultaneous tracking of multiple targets
- Stable Track IDs
- Primary target selection
- Automatic vehicle completion detection
- Ghost/artifact filtering
- Vehicle statistics
- MQTT event publishing
- Grafana / InfluxDB ready

### Single Target (Compatibility)

- Native Mode 1 (ASCII protocol)
- Compatible with the original radar firmware
- Same Home Assistant entities
- Same MQTT event format
- Runtime configurable
- Ideal for testing and comparison

The operating mode can be changed directly from Home Assistant without rebooting.

---

# Home Assistant

The component exposes:

## Live Sensors

- Detection
- Distance
- Speed
- Direction
- Vehicle Tracking
- Vehicle ID
- Target Count
- Multi Target Active

## Vehicle Statistics

- Vehicle Counter
- Maximum Speed
- Average Speed
- Start Distance
- End Distance
- Minimum Distance
- Duration
- Sample Count
- Last Vehicle Event

---

# Runtime Configuration

Most radar parameters can be changed while the radar is running.

The component automatically

```
HEX
    ↓
ASCII
    ↓
write/read parameter
    ↓
HEX
```

without requiring a reboot.

When operating permanently in Single Target mode no protocol switching is necessary.

---

# MQTT

Every completed vehicle produces a JSON event.

Example:

```json
{
  "id": 42,
  "mode": "multi-target",
  "protocol": "hex",
  "direction": "Approaching",
  "start_distance_m": 72.3,
  "end_distance_m": 7.8,
  "minimum_distance_m": 7.8,
  "max_speed_kmh": 41.6,
  "average_speed_kmh": 37.4,
  "duration_s": 5.9,
  "samples": 22,
  "ghosts_filtered": 3,
  "firmware": "stable-1.1-dual-mode"
}
```

---

# Ghost Filtering

Known radar artifacts can be filtered before they reach

- Home Assistant
- MQTT
- Vehicle Tracker
- LED Outputs

Example:

```yaml
artifact_filter: true
artifact_distance: 33.3
artifact_distance_tolerance: 0.6
artifact_speed: 89.0
artifact_speed_tolerance: 5.0
```

---

# Supported Hardware

- ESP32-S3
- LDL508PRO Radar Sensor

---

# Planned Features

- Smart car detection
- Carport automation
- Warning light control
- Traffic statistics
- Adaptive radar parameters
- Multiple artifact signatures
- Automatic ghost learning

---

# Version

Current stable release

**stable-1.1-dual-mode**

Highlights

- Dual operating mode
- Runtime parameter configuration
- Unified vehicle publisher
- Multi Target tracking
- Home Assistant integration
- MQTT event system
- Grafana ready

---

# Credits

Based on the original ESPHome LDL508PRO component.

Extended and redesigned with

- Dual-mode architecture
- Multi Target tracking
- Runtime protocol switching
- Vehicle event system
- MQTT event publishing
- Advanced artifact filtering
