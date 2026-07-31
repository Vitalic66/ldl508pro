# ESPHome LDL508PRO – stabile Einzelzielversion mit Rot/Grün-Ausgängen

Diese Variante konzentriert sich auf den stabilen, dokumentierten Einzelzielmodus des LDL508PRO.
Die experimentelle automatische Mehrzielumschaltung ist in der Beispielkonfiguration deaktiviert.

## GPIO-Ausgänge

Die Komponente besitzt zwei optionale GPIO-Ausgänge:

- `red_output_pin`
- `green_output_pin`

Beide sind für einen **active-HIGH LED-Treiber** ausgelegt. Ein HIGH-Pegel am ESP32 lässt den externen Treiber den jeweiligen GND-Kanal durchschalten.

| Radarzustand | Rot | Grün |
|---|---:|---:|
| Ziel erkannt | HIGH | LOW |
| Kein Ziel | LOW | HIGH |

Die Pins werden direkt durch die Komponente geschaltet und folgen damit demselben internen Erkennungszustand wie der Home-Assistant-Binärsensor `detected`.

## Beispiel

```yaml
ldl508pro:
  id: radar
  uart_id: radar_uart
  auto_enable_multi_target_after_sync: false
  multi_target_polling: false

  red_output_pin:
    number: GPIO4
    inverted: false
  green_output_pin:
    number: GPIO5
    inverted: false
```

`GPIO4` und `GPIO5` sind nur Beispiele. Vor dem Flashen müssen sie an die tatsächlich freien Pins deines Boards angepasst werden.

Beide Ausgänge müssen gemeinsam konfiguriert werden. Wird nur einer angegeben, schlägt die YAML-Validierung absichtlich fehl.

## Elektrischer Hinweis

Die ESP32-GPIOs dürfen nur die Logikeingänge des LED-Treibers ansteuern. LED-Strom oder Versorgungsspannungen dürfen nicht direkt über die GPIOs geführt werden. ESP32 und Treiber benötigen eine gemeinsame Masse auf der Logikseite.

## Stabiler Betrieb

Empfohlene Einstellungen:

```yaml
logger:
  baud_rate: 0
  level: INFO

ldl508pro:
  debug_uart: false
  target_timeout: 1500ms
  auto_enable_multi_target_after_sync: false
  multi_target_polling: false
```

Der Radar wird beim Start zunächst in Zielmodus 1 normalisiert, danach wird seine Konfiguration eingelesen. Entfernung, Geschwindigkeit, Richtung und Fahrzeugereignisse stammen aus den normalen `R distance speed`-Frames.

## Inhalt

- `components/ldl508pro/` – External Component
- `example.yaml` – vollständige Beispielkonfiguration


## Geistermessungsfilter

Das beobachtete Fehlermuster `R 033.x -089.x` wird standardmäßig verworfen, bevor es Entfernung, Geschwindigkeit, Zielstatus, LEDs oder Fahrzeugzähler beeinflusst. Der Filter greift nur, wenn **Entfernung und Betrag der Geschwindigkeit gleichzeitig** im konfigurierten Fenster liegen.

```yaml
artifact_filter: true
artifact_distance: 33.3
artifact_distance_tolerance: 0.6
artifact_speed: 89.0
artifact_speed_tolerance: 2.0
```

Im Log erscheint höchstens alle fünf Sekunden eine zusammengefasste Warnung mit der Anzahl verworfener Messungen.


## LED-Zeitsteuerung und Störungsanzeige

Die Beispielkonfiguration erzeugt zwei in Home Assistant veränderbare Number-Entitäten:

- `Radar LED Rot Nachlaufzeit`: 0–60 Sekunden, Startwert nach Neustart 5 Sekunden.
- `Radar LED Standby-Zeit`: 0–3600 Sekunden, Startwert nach Neustart 60 Sekunden. `0` hält Grün dauerhaft aktiv.

Nach einer Erkennung bleibt Rot für die Nachlaufzeit aktiv. Anschließend leuchtet Grün, bis die Standby-Zeit ohne neue Erkennung abgelaufen ist; danach sind beide Ausgänge LOW. Bei einem Konfigurationsfehler blinkt Rot mit 1 Hz und Grün bleibt aus. Eine erfolgreiche Konfigurationssynchronisierung beendet die Störungsanzeige.


## Direktes MQTT-Fahrzeugereignis

Bei jedem abgeschlossenen Fahrzeug publiziert die Komponente genau eine JSON-Nachricht.
Standardtopic bei leerem `mqtt_event_topic`:

```text
<mqtt.topic_prefix>/vehicle/event
```

Mit dem Beispiel ist das:

```text
ldl508pro/radar/vehicle/event
```

Konfiguration:

```yaml
ldl508pro:
  mqtt_event_enabled: true
  mqtt_event_topic: ""
  mqtt_event_qos: 0
  mqtt_event_retain: false
```

Beispielpayload:

```json
{"id":11,"direction":"Annähernd","start_distance_m":39.2,"end_distance_m":12.3,"minimum_distance_m":12.3,"max_speed_kmh":21.2,"average_speed_kmh":17.0,"duration_s":8.1,"samples":32,"ghosts_filtered":7,"firmware":"stable-1.0-mqtt"}
```

`retain` ist standardmäßig aus, damit Node-RED nach einem Neustart kein altes Fahrzeug erneut in InfluxDB schreibt.
Die vorhandene Home-Assistant-Textsensor-Nachricht bleibt parallel erhalten.
