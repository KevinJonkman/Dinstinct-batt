# Dinstinct Battery Tester - Continuation Prompt

Use this file to resume working on the battery tester project with Claude Code. Copy the content below as your prompt.

---

## Project Context

I'm working on two ESP32 LyraT boards for battery testing:

### 1. Battery Tester (this repo: Dinstinct-batt)
- **Hardware**: ESP32 LyraT v1.2 (ESP32-D0WD-V3, 240MHz, PSRAM)
- **COM port**: COM4
- **IP**: 192.168.1.40
- **Version**: v7.8 "REST HOLD + REMOTE SENSOR"
- **Code**: `src/main.cpp` (~3500 lines)
- **Function**: Controls a Delta SM70-CP-450 bidirectional PSU (192.168.1.16:8462) via SCPI over TCP for battery charge/discharge cycling (SGS: charge -> rest -> discharge -> rest)
- **Architecture**: Dual-core FreeRTOS
  - Core 0: WiFi + Delta SCPI communication task
  - Core 1: Arduino loop() with WebServer, logging, safety checks
- **Web UI**: Shows voltage, current, elapsed time, 4 remote temps, cycle progress, Wh/CE%/EE% per cycle
- **Safety**: Hard limits V:2.71-4.18V, over-temp, over-current alarms (all using remote sensor data)
- **Logging**: CSV to SPIFFS with per-cycle Wh tracking

### 2. Sensor Board (repo: lyrat-temp-thermal)
- **Hardware**: ESP32 LyraT v1.2 (identical board)
- **COM port**: COM12
- **IP**: 192.168.1.24
- **Code**: `src/main.cpp` (~450 lines)
- **Function**: Slave temperature sensor board providing data to battery tester via WiFi
- **Sensors**:
  - 2x DS18B20 on OneWire (currently GPIO 13, **NEEDS MOVE** - see pending tasks)
  - MLX90640 32x24 thermal camera on Wire1 (SDA=15, SCL=14) at 4Hz
- **API endpoints**: `/status` (JSON), `/thermaldata` (pixel array), `/rescan` (bus scan)
- **WiFi**: BTAC Medewerkers / Next3600$!

### Communication Flow
Battery tester fetches `http://192.168.1.24/status` every 5s using lightweight raw WiFiClient. Remote fetch is disabled until Delta is connected to prevent TCP stack contention. If sensor board is offline, backoff increases to 30s after 3 failures.

---

## Recent Changes (latest session)

### SCPI Connection Fix
The Delta's SCPI parser was getting stuck after abrupt TCP disconnects (e.g. ESP32 firmware upload). Confirmed by testing directly from PC - TCP connects but `*IDN?` gets no response. Fix:
- **SCPI reset sequence**: On every new TCP connection, send `\r\n\r\n` + `*CLS` to flush stuck parser state before attempting `*IDN?`
- **3x IDN retry**: If `*IDN?` gets no response, disconnect TCP, reconnect, reset, and retry (up to 3 attempts)
- **Faster backoff**: Reduced max backoff from 30s to 10s (2s, 4s, 8s, 10s)
- **Clear lastError**: `status.lastError` is now cleared on successful connect (was showing stale "SCPI NOT RESPONDING" on web UI even when connected)
- **Remote fetch guard**: `fetchRemoteSensorData()` skips when `!status.deltaConnected` to prevent TCP stack contention during Delta connect phase

### Known Issue
Delta power cycle is still required if SCPI parser gets truly stuck (the reset sequence helps but can't fix all cases). After power cycle, the ESP32 connects within 2-5 attempts (Delta TCP port comes up before SCPI processor).

---

## Pending Tasks (in priority order)

### 1. DS18B20 GPIO Fix (Sensor Board)
**Problem**: DS18B20 sensors not detected. GPIO 13 (TCK) has JTAG circuitry on LyraT pulling pin LOW, overriding 4.7K pullup.
**Fix**: Move DS18B20 data wire from GPIO 13 to **GPIO 4** or **GPIO 27**. Update `ONE_WIRE_BUS` in `lyrat-temp-thermal/src/main.cpp`.
**Status**: Waiting for physical hardware access.

### 2. Test deltaRestHold() (Battery Tester)
**What**: During SGS REST phases, keeps Delta output ON at battery voltage with 0A current limits (instead of OUTPut OFF which caused -1.64A sink current in v7.7).
**Status**: Implemented but UNTESTED. Needs real SGS cycle.

### 3. Test New quickDeltaOff() (Battery Tester)
**What**: Rewritten with proper SCPI response handling using `deltaSet()`/`deltaQuery()` instead of flooding 8 commands without reading responses.
**Status**: Implemented but UNTESTED.

### 4. Full SGS Cycle Test (4 cycles)
Verify with Delta powered on:
- No sink current during REST phases
- deltaRestHold() holds battery voltage correctly
- quickDeltaOff() properly shuts down at end
- Remote sensor data flows during cycle
- Wh tracking and CE%/EE% calculations work
- User wanted to run 4-cycle test but couldn't because of SCPI connection issues

---

## Key Technical Details

### Delta SM70-CP-450 SCPI
- TCP connection to 192.168.1.16:8462
- Commands: `deltaSet()` (send + delay) and `deltaQuery()` (send + return response)
- SCPI boots slower than TCP port - needs reset sequence + retry on new connections
- Bidirectional: positive current = charge, negative current = discharge
- OUTPut OFF doesn't physically disconnect - current flows through output stage

### SCPI Connection Sequence (deltaConnect)
1. TCP connect with 1s timeout
2. 300ms delay
3. Send `\r\n\r\n` (flush partial commands) + `*CLS` (clear status)
4. Send `*IDN?`, wait 2s for response
5. If no response: disconnect, reconnect, repeat (up to 3 times)
6. On success: reset backoff, clear lastError

### ESP32 LyraT Pin Constraints
- GPIO 13 (TCK): JTAG - don't use for OneWire
- GPIO 14 (TMS): Used for MLX SCL (sensor board)
- GPIO 15 (TDO): Used for MLX SDA (sensor board)
- GPIO 12 (TDI): JTAG - avoid
- GPIO 22: Blue LED
- GPIO 4, 27: Available for OneWire

### Flash/RAM Usage
- Battery tester: Flash 87.1%, RAM 15.4%
- Sensor board: ~75%

### PlatformIO
- Build/upload: `C:\Users\KevinBtacseu\.platformio\penv\Scripts\pio.exe run -t upload`
- Monitor: `pio.exe device monitor`
- Battery tester: COM4 (auto-detected)
- Sensor board: COM12 (hardcoded in platformio.ini)

### GitHub Repos
- Battery tester: https://github.com/KevinJonkman/Dinstinct-batt
- Sensor board: https://github.com/KevinJonkman/lyrat-temp-thermal
