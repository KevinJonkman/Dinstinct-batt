# Dinstinct Battery Tester v7.1

Professional battery testing system with thermal imaging, web interface, and comprehensive safety features.

## Hardware

- **Controller**: ESP32 LyraT v1.2
- **Power Supply**: Delta SM70-CP-450 (70V/45A bidirectional)
- **Temperature Sensor**: DS18B20 (OneWire)
- **Thermal Camera**: MLX90640 32x24 IR array (optional)
- **Audio**: ES8311 codec for audible feedback

## Features

### Battery Testing
- **Charge Mode**: Constant voltage charging with configurable current limit
- **Discharge Mode**: Controlled discharge to specified cutoff voltage
- **Cycle Mode**: Automated charge/discharge cycles for capacity testing
- **Real-time Monitoring**: Voltage, current, power, temperature, Ah, Wh

### Web Interface
- Modern responsive dashboard accessible from any browser
- Large 7-segment style displays for voltage/current/temperature
- Real-time Chart.js graphs with voltage, current, and temperature
- Thermal camera heatmap display (when MLX90640 connected)

### Delta PSU Integration
- **CV/CC Indicator**: Shows Constant Voltage or Constant Current mode
- **Set vs Measured**: Compare programmed values with actual readings
- **Cable Loss Detection**: Calculates voltage drop in cables using 4-wire sensing
- Full SCPI control over TCP/IP

### Safety System
- **Hard Limits**: Absolute voltage and current boundaries that cannot be exceeded
- **Confirmation Dialogs**: Required for all charge/discharge operations
- **Parameter Validation**: Blocks dangerous values (e.g., 250A or 20V)
- **Safety Code Protection**: Required to modify safety limits
- **Audit Logging**: All parameter changes and blocked attempts are logged

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
const char* DELTA_IP = "192.168.1.27";  // Delta PSU IP
```

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
| `/status` | JSON status data |
| `/safety` | Safety limits (JSON) |
| `/safety/log` | Safety audit log |
| `/charge?v=X&i=Y` | Start charging |
| `/discharge?v=X&i=Y` | Start discharging |
| `/cycle` | Start auto cycle |
| `/stop` | Stop current test |
| `/download` | Download CSV log |

## Screenshots

Access the web interface at `http://<ESP32_IP>/`

## License

MIT License

## Author

Dinstinct / BTAC
