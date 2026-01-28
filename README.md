# Dinstinct Battery Tester v7.3

Professional battery testing system with thermal imaging, web interface, and comprehensive safety features.

## Hardware

- **Controller**: ESP32 LyraT v1.2
- **Power Supply**: Delta SM70-CP-450 (70V/45A bidirectional)
- **Temperature Sensor**: DS18B20 (OneWire)
- **Thermal Camera**: MLX90640 32x24 IR array (optional)
- **Audio**: ES8311 codec for audible feedback
- **Status LED**: Blue LED on GPIO 22

## Features

### Battery Testing
- **Charge Mode**: Constant voltage charging with configurable current limit
- **Discharge Mode**: Controlled discharge to specified cutoff voltage
- **Cycle Mode**: Automated charge/discharge cycles for capacity testing
- **Anorgion Test**: Multi-cycle test with configurable parameters and cutoff current detection
- **Wh Capacity Test**: Single cycle test at specified current to measure battery capacity
- **Real-time Monitoring**: Voltage, current, power, temperature, Ah, Wh

### Web Interface
- Modern responsive dashboard accessible from any browser
- Large 7-segment style displays for voltage/current/temperature
- Real-time Chart.js graphs with voltage, current, and temperature
- Thermal camera heatmap display (when MLX90640 connected)
- Chunked page transfer for reliable loading during Delta communication

### Delta PSU Integration
- **CV/CC Indicator**: Shows Constant Voltage or Constant Current mode
- **Set vs Measured**: Compare programmed values with actual readings
- **Cable Loss Detection**: Calculates voltage drop in cables using 4-wire sensing
- Full SCPI control over TCP/IP (port 8462)

### Safety System
- **Hard Limits**: Absolute voltage and current boundaries that cannot be exceeded
- **Confirmation Dialogs**: Required for all charge/discharge operations
- **Parameter Validation**: Blocks dangerous values (e.g., 250A or 20V)
- **Safety Code Protection**: Required to modify safety limits
- **Audit Logging**: All parameter changes and blocked attempts are logged

### Status LED (GPIO 22)
- **Fast blink (200ms)**: Test running
- **Slow blink (1000ms)**: Idle/standby

### Thermal Monitoring
- MLX90640 32x24 pixel thermal imaging
- Real-time min/max/average temperature display
- Thermal alarm with automatic shutdown
- Color-coded heatmap visualization

### Data Logging
- Automatic logging to internal SPIFFS storage
- CSV export for analysis in Excel/Google Sheets
- Logs: timestamp, voltage, current, power, temperatures, mode, Ah, Wh

## Safety Limits (Default)

| Parameter | Min | Max | Warning |
|-----------|-----|-----|---------|
| Voltage | 2.50V | 4.25V | >4.22V |
| Charge Current | 0.1A | 30A | >10A |
| Discharge Current | 0.1A | 30A | >10A |
| Temperature (DS18B20) | - | 45°C | - |
| Temperature (Thermal) | - | 60°C | - |

## Pin Configuration

| Function | GPIO |
|----------|------|
| DS18B20 Temperature | 13 |
| Blue Status LED | 22 |
| I2C SDA (MLX90640) | 18 |
| I2C SCL (MLX90640) | 23 |
| PA Enable | 21 |
| I2S MCLK | 0 |
| I2S BCLK | 5 |
| I2S LRCK | 25 |
| I2S DOUT | 26 |
| I2S DIN | 35 |

## Network Configuration

Edit `src/main.cpp` to set your network:
```cpp
const char* WIFI_SSID = "Your_WiFi_SSID";
const char* WIFI_PASS = "Your_WiFi_Password";
const char* DELTA_IP = "192.168.1.16";  // Delta PSU IP
const int DELTA_PORT = 8462;
```

Current BTAC config:
- WiFi: BTAC Medewerkers
- ESP32 IP: 192.168.1.40
- Delta IP: 192.168.1.16

## Building & Uploading

Requires PlatformIO:

```bash
# Build
pio run

# Upload
pio run --target upload

# Monitor serial output
pio device monitor
```

## Web Endpoints

| Endpoint | Description |
|----------|-------------|
| `/` | Main dashboard |
| `/settings` | Configuration page |
| `/thermal` | Full-screen thermal view |
| `/sgs` | Anorgion Test page |
| `/whtest` | Wh Capacity Test page |
| `/status` | JSON status data |
| `/safety` | Safety limits (JSON) |
| `/safety/log` | Safety audit log |
| `/charge?v=X&i=Y` | Start charging |
| `/discharge?v=X&i=Y` | Start discharging |
| `/cycle` | Start auto cycle |
| `/whtest/start` | Start Wh capacity test |
| `/whtest/status` | Wh test status (JSON) |
| `/sgs/start` | Start Anorgion test |
| `/sgs/status` | Anorgion test status (JSON) |
| `/stop` | Stop current test |
| `/download` | Download CSV log |

## Delta PSU Helper Scripts

PowerShell scripts voor directe Delta controle (in user home folder):

```powershell
# delta_stop.ps1 - Zet Delta output uit
# delta_on.ps1 - Zet Delta output aan
# delta_query.ps1 - Query Delta metingen
# delta_setup_charge.ps1 - Setup voor laden (4.15V/12A)
```

## Current Status (28 Jan 2025)

**Versie: 7.4** - CRITICAL SAFETY UPDATE

### v7.4 Fixes (BELANGRIJK!):
- **HARD VOLTAGE LIMITS**: Delta stopt ONMIDDELLIJK bij V < 2.70V of V > 4.15V
- **Fixed SCPI truncation**: Stroommeting was soms afgekapt (-11A ipv -12A)
- **Fixed discharge cutoff**: Stopt nu bij voltage OF stroom limiet (was: beide nodig)
- **Double-tap safety**: Delta OFF commando wordt 2x gestuurd voor zekerheid

### Wat werkt:
- Laden/ontladen met Delta SM70-CP-450
- **HARDE VEILIGHEIDSLIMIETEN** - kan NOOIT onder 2.70V of boven 4.15V komen
- STOP knop reageert tijdens tests
- Safety limits en bevestigingsdialogen
- CV/CC mode indicator
- Cable loss detectie
- Data logging naar SPIFFS
- **Blue LED status indicator** (fast=running, slow=idle)
- **Anorgion Test pagina** met multi-cycle testing
- **Wh Capacity Test pagina** voor capaciteitsmeting
- **Chunked page transfer** - pagina's laden betrouwbaar

### Bekende issues:
- Delta queries blokkeren ~800ms per query (hardware limitatie)
- Bij Delta communicatie problemen: power cycle Delta PSU
- Voltage display kan 0.01V afwijken van Delta LCD (afrondingsverschil)

### Tips:
- Als ESP32 niet reageert: eerst Delta uitzetten, dan ESP32 resetten
- Delta moet in REMOTE mode staan voor SCPI communicatie
- Gebruik lange timeouts bij curl requests (30-60s) tijdens tests

## Wh Capacity Test

Meet de werkelijke capaciteit van een batterij:

1. **Laad fase**: Laadt tot ingestelde spanning (default 4.15V) bij ingestelde stroom (default 12A)
2. **Cutoff detectie**: Stopt laden wanneer stroom onder cutoff waarde zakt (default 1.5A)
3. **Ontlaad fase**: Ontlaadt tot ingestelde spanning (default 2.70V) bij ingestelde stroom (default 12A)
4. **Resultaat**: Toont Wh ontladen, Ah geladen/ontladen, energie-efficiëntie

## Anorgion Test

Multi-cycle test voor uitgebreide batterij karakterisering:

- Configureerbare aantal cycles
- Per-cycle tracking van Ah en Wh
- Cutoff current detectie voor CV→CC transitie
- Cycle history met efficiëntie per cycle

## License

MIT License

## Author

Dinstinct / BTAC
