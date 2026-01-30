# Plan Maandag/Dinsdag — Speaker + Home Assistant

## Status nu (vrijdag 30 jan)
- UI v9.0 met DSEG7 displays + Chart.js: WERKT
- Battery tester CORS: WERKT
- Cross-origin fetch V/I/P in sensorboard UI: WERKT
- Audio/speaker: WERKT NOG NIET — code is klaar maar geen geluid

---

## STAP 1: Speaker werkend krijgen

### Het probleem
De ESP32 LyraT heeft een ES8388 audio codec. De code initialiseert de chip en roept Google TTS aan, maar er komt geen geluid uit de speaker. Mogelijke oorzaken:

1. **I2C adres verkeerd** — ES8388 kan op 0x10 of 0x11 zitten afhankelijk van LyraT versie
2. **I2C pinnen verkeerd** — Code gebruikt SDA=18, SCL=23 (standaard LyraT v4.3). Memoircy gebruikte SDA=33, SCL=32 (ander board)
3. **ESP32-audioI2S library conflict** — Library v2.0.0 doet eigen I2S init die mogelijk botst met handmatige ES8388 init
4. **Google TTS verbinding faalt** — connecttospeech() retourneert false (SSL/DNS/redirect)
5. **PA enable pin verkeerd** — GPIO 21 is standaard, maar check LyraT versie

### Debug stappen (in deze volgorde)

#### A. Serial monitor checken
```
pio device monitor --port COM12 --baud 115200
```
Kijk naar de `[AUDIO]` regels:
- `ES8388 I2C probe: FOUND` of `NOT FOUND`?
- `connecttospeech returned: OK` of `FAIL`?

#### B. Als ES8388 NOT FOUND — I2C scan
Voeg toe aan setup() (tijdelijk):
```cpp
// I2C bus scan
Serial.println("[I2C SCAN] Starting...");
for (uint8_t addr = 1; addr < 127; addr++) {
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() == 0) {
    Serial.printf("[I2C SCAN] Device at 0x%02X\n", addr);
  }
}
```
Dit laat zien welke devices op de bus zitten en op welk adres de ES8388 werkelijk zit.

#### C. Als ES8388 FOUND maar geen geluid — test zonder TTS
Vervang `say("Sensor hub online")` door een simpele I2S sinustoon:
```cpp
// Direct I2S test - bypasses ES8388 codec issues
#include <driver/i2s.h>
void testTone() {
  int16_t buf[256];
  for (int i = 0; i < 128; i++) {
    int16_t v = (int16_t)(sin(2.0 * PI * 1000.0 * i / 16000.0) * 10000);
    buf[i*2] = v;
    buf[i*2+1] = v;
  }
  size_t w;
  for (int r = 0; r < 100; r++) {
    i2s_write(I2S_NUM_0, buf, sizeof(buf), &w, portMAX_DELAY);
  }
}
```
Als dit ook geen geluid geeft: hardware probleem (PA, speaker, bedrading).

#### D. Als TTS FAIL — test met simpele URL
```cpp
audio.connecttohost("http://streams.calmradio.com/api/39/128/stream");
```
Als dit WEL geluid geeft: Google TTS URL is het probleem (geblokkeerd/redirect).

#### E. Memoircy pinout vergelijken
Check welke LyraT versie het board fysiek is (v1.2, v4.2, v4.3) — staat op de PCB. Elke versie kan andere pinnen hebben:

| Pin      | LyraT v4.3 | AudioKit (Memoircy) |
|----------|-------------|---------------------|
| I2C SDA  | 18          | 33                  |
| I2C SCL  | 23          | 32                  |
| I2S BCLK | 5           | 27                  |
| I2S LRC  | 25          | 25                  |
| I2S DOUT | 26          | 26                  |
| I2S MCLK | 0           | 0                   |
| PA Enable| 21          | 21                  |
| ES8388   | 0x10        | 0x10                |

### Gewenste audio functies als het werkt
- Startup: "Sensor hub online"
- `/say?t=text` endpoint voor remote spraak
- Battery tester roept `/say` aan bij: test start, phase change, test complete, error
- Optioneel: beep tonen als TTS niet beschikbaar

---

## STAP 2: Home Assistant integratie

### Architectuur
```
Delta SM15K (192.168.1.27)
    |
    v  HTTP/CGI
ESP32 Battery Tester (192.168.1.40) --MQTT--> Home Assistant
    |                                              |
    v  HTTP /status                                v
ESP32 Sensor Hub (192.168.1.24) ----MQTT--> HA Dashboard
    |                                              |
    Speaker (TTS)                          Hikvision Camera's
```

### 2A. MQTT toevoegen aan beide ESP32's
Beide ESP32's publiceren hun data via MQTT naar Home Assistant:

**Battery Tester publiceert:**
- `anorgion/battery/voltage` — spanning (V)
- `anorgion/battery/current` — stroom (A)
- `anorgion/battery/power` — vermogen (W)
- `anorgion/battery/wh` — totaal Wh
- `anorgion/battery/ah_charge` — Ah geladen
- `anorgion/battery/ah_discharge` — Ah ontladen
- `anorgion/battery/mode` — huidige modus
- `anorgion/battery/running` — test actief
- `anorgion/battery/cycle` — huidige cyclus
- `anorgion/battery/error` — laatste fout

**Sensor Hub publiceert:**
- `anorgion/sensor/t1` — DS18B20 temp 1
- `anorgion/sensor/t2` — DS18B20 temp 2
- `anorgion/sensor/mlx_max` — MLX90640 max temp
- `anorgion/sensor/mlx_avg` — MLX90640 gem temp

**Home Assistant subscribet + commands:**
- `anorgion/command/say` — tekst om uit te spreken
- `anorgion/command/stop` — noodstop
- `anorgion/command/start_charge` — start laden
- `anorgion/command/start_discharge` — start ontladen

### Library nodig
```ini
; In platformio.ini van beide projecten
lib_deps =
    knolleary/PubSubClient@^2.8
```

### 2B. Home Assistant configuratie

**MQTT Broker:** Mosquitto add-on (al geinstalleerd? anders installeren)

**sensor configuratie (configuration.yaml of via MQTT auto-discovery):**
De ESP32's sturen MQTT discovery messages zodat HA automatisch sensors aanmaakt.

### 2C. Dashboard
- Gauges voor V, I, P
- Lijn-grafiek voor V/I verloop (HA history)
- Temperatuur indicators met kleur-alarm
- Hikvision camera stream (RTSP via generic camera)
- Start/Stop knoppen die MQTT commands sturen

### 2D. Automatiseringen
1. **Thermal alarm:** mlx_max > 45°C → noodstop + push notificatie
2. **Voice feedback:** Bij state changes → TTS via sensorboard speaker
3. **Camera snapshot:** Bij error → Hikvision snapshot + Discord webhook

---

## Technische details

### Netwerk
| Device           | IP             | Poort |
|------------------|----------------|-------|
| Delta SM15K      | 192.168.1.27   | 80    |
| Battery Tester   | 192.168.1.40   | 80    |
| Sensor Hub       | 192.168.1.24   | 80    |
| Home Assistant   | TBD            | 8123  |
| MQTT Broker      | = HA IP        | 1883  |

### Repositories
- Sensor Hub: https://github.com/KevinJonkman/lyrat-temp-thermal
- Battery Tester: https://github.com/KevinJonkman/Dinstinct-batt
- **BTAC-GLC org (bestaande HA/MQTT/ESPHome infra):**
  - `BTAC-GLC/ha-config` — Volledige HA configuratie (YAML dashboards, MQTT sensors, InfluxDB, light groups)
  - `BTAC-GLC/ESPHome-software` — ESPHome configs (BatSensor, HVAC, Water Level, shared configs)
  - `BTAC-GLC/proto-mqtt` — Protobuf↔JSON MQTT bridge (Rust)
  - `BTAC-GLC/ESPHome-devices` — Device definities
  - `BTAC-GLC/has` — HA core fork

### Huidige versies
- Sensor Hub: v9.0 (commit de74eab)
- Battery Tester: v8.0 (commit 4e3da8e)

---

## Wat er al bestaat in BTAC-GLC (om op voort te bouwen)

### ha-config — bestaande patronen
De HA instance heeft al:
- **MQTT sensors** via `mqtt:` blok in configuration.yaml — topic formaat: `homeassistant/light/{id}/sensor/temp`
- **YAML dashboards** (lovelace mode: yaml) met meerdere views (operator, technician, grower, lights, phases, energy)
- **Custom cards:** bignumber-card, plotly-graph-card, apexcharts-card, timeslot-scheduler, plant-card
- **InfluxDB** integratie (192.168.1.94, database "has") voor long-term storage
- **Light groups** met cell/row/level/segment naamgeving
- **Input helpers** (input_boolean, input_number, input_text, input_datetime)

### ESPHome-software — bestaande patronen
- **Shared configs** in `shared_configs/` (esphome.yaml, esp32.yaml, project_base.yaml)
- **Sensor includes:** tmp117, scd4x, bme280, shtcx, bh1750
- **BatSensor** device met I2C bus, GPIO buttons, RGB LED, switches
- **Namensconventie:** `${cell}${row}${level}${segment}_sensorname`

### proto-mqtt — MQTT bridge
- Rust binary die MQTT berichten vertaalt van protobuf naar JSON
- Kan als template dienen voor onze MQTT message structuur

---

## HA als backbone — voordelen voor battery tester

Met HA als centraal punt kan de battery tester ESP32 eenvoudiger worden:

### Nu (zonder HA)
```
Battery Tester ESP32:
  - Delta PSU aansturing (Core 0)
  - Webserver + UI (Core 1)
  - Remote sensor polling (HTTP naar .24)
  - Test logica (cycle, SGS, Wh test)
  - Data logging (SPIFFS)
  - Safety limits
  → Alles in 1 ESP32, complex
```

### Straks (met HA backbone)
```
Battery Tester ESP32:
  - Delta PSU aansturing (Core 0)
  - MQTT publish (V, I, P, status)
  - MQTT subscribe (start/stop commands)
  → Veel simpeler

Home Assistant:
  - Dashboard (gauges, charts, camera)
  - Automatiseringen (thermal safety, alerts)
  - Data logging (InfluxDB, al aanwezig!)
  - TTS via sensorboard speaker
  - Push notificaties (mobiel, Discord)
  - Hikvision camera snapshots bij events

Sensor Hub ESP32:
  - Temperatuur meting
  - MQTT publish (T1, T2, MLX)
  - Speaker (TTS via HA of direct)
```

### Wat dit oplevert
1. **Minder code op ESP32** — geen webserver UI nodig, HA doet dashboard
2. **InfluxDB logging** — gratis, onbeperkt, al draaiend op 192.168.1.94
3. **Automatiseringen in YAML** — makkelijker te wijzigen dan C++ code
4. **Camera integratie** — Hikvision snapshot bij alarm
5. **Push notificaties** — mobiel/Discord bij events
6. **Meerdere gebruikers** — HA dashboard via browser, geen ESP32 belasting
7. **Herbruikbaar** — zelfde patronen als bestaande BTAC-GLC setup
