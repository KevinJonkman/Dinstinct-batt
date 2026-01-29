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
- **Code**: `src/main.cpp` (~3400 lines)
- **Function**: Controls a Delta SM70-CP-450 bidirectional PSU (192.168.1.16:8462) via SCPI over TCP for battery charge/discharge cycling (SGS: charge → rest → discharge → rest)
- **Architecture**: Dual-core FreeRTOS
  - Core 0: WiFi + Delta SCPI communication task
  - Core 1: Arduino loop() with WebServer, logging, safety checks
- **Web UI**: Shows voltage, current, elapsed time, 4 remote temps, cycle progress, Wh/CE%/EE% per cycle
- **Safety**: Over-temp, over-voltage, over-current, thermal max alarms (all using remote sensor data)
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
Battery tester fetches `http://192.168.1.24/status` every 5 seconds using lightweight raw WiFiClient (not HTTPClient, which caused crashes at 98.3% flash usage).

---

## Pending Tasks (in priority order)

### 1. DS18B20 GPIO Fix (Sensor Board)
**Problem**: DS18B20 sensors are not detected. They are wired to GPIO 13 (TCK) which has JTAG circuitry on the LyraT that pulls the pin LOW, overriding the 4.7K pullup resistor. A `/rescan` confirms 0 devices found on all scanned GPIOs, and GPIO 13 idle state = LOW.

**Fix**: Physically move the DS18B20 data wire from GPIO 13 to **GPIO 4** or **GPIO 27** (both are free and not JTAG-conflicted). Then update `ONE_WIRE_BUS` constant in `lyrat-temp-thermal/src/main.cpp`.

**Status**: User said "kan er niet bij nu" (can't physically access the hardware right now).

### 2. Test deltaRestHold() (Battery Tester)
**What**: During SGS REST phases, the code now calls `deltaRestHold()` instead of `quickDeltaOff()`. This keeps the Delta output ON but sets voltage = battery voltage and current limits = 0A, preventing the -1.64A sink current observed in v7.7.

**Status**: Implemented but UNTESTED with Delta powered on. Needs a real SGS cycle test.

### 3. Test New quickDeltaOff() (Battery Tester)
**What**: `quickDeltaOff()` was rewritten to use `deltaSet()`/`deltaQuery()` instead of flooding 8 SCPI commands without reading responses. The old version caused buffer overflow and commands (including OUTPut OFF) being dropped.

**Status**: Implemented but UNTESTED with Delta powered on.

### 4. Full SGS Cycle Test
Run a complete SGS cycle (charge → rest → discharge → rest) with all v7.8 fixes and verify:
- No -1.64A sink current during REST phases
- deltaRestHold() correctly holds battery voltage with 0A
- quickDeltaOff() properly shuts down at end of test
- Remote sensor data flows correctly during cycle
- Wh tracking and CE%/EE% calculations work

---

## Key Technical Details

### Delta SM70-CP-450 SCPI
- TCP connection to 192.168.1.16:8462
- Commands use `deltaSet()` (send + read response) and `deltaQuery()` (send + return response)
- SCPI interface boots slower than TCP port after power cycle - backoff mechanism handles this
- Bidirectional: positive current = charge, negative current = discharge
- OUTPut OFF doesn't physically disconnect - current can still flow through output stage

### ESP32 LyraT Pin Constraints
- GPIO 13 (TCK): JTAG - don't use for OneWire
- GPIO 14 (TMS): Used for MLX SCL
- GPIO 15 (TDO): Used for MLX SDA
- GPIO 12 (TDI): JTAG - avoid
- GPIO 22: Blue LED
- GPIO 4, 27: Available for OneWire

### Flash Usage
- Battery tester: 87.0% (was 98.3% before removing local sensors and HTTPClient)
- Sensor board: ~75%

### PlatformIO
- Build/upload: `C:\Users\KevinBtacseu\.platformio\penv\Scripts\pio.exe run -t upload`
- Monitor: `pio.exe device monitor`
- Battery tester upload_port: auto (COM4)
- Sensor board upload_port: COM12
