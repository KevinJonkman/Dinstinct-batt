/*
 * Battery Tester v7.5 - HARD SAFETY LIMITS
 * Based on v7.3 + CRITICAL safety fixes:
 * - Fixed SCPI response truncation (was cutting off at 4 chars)
 * - Added HARD voltage limits: IMMEDIATE stop if V < 2.70V or V > 4.15V
 * - Fixed discharge cutoff: stops on voltage OR current, not both
 *
 * Hardware:
 * - ESP32 LyraT v1.2
 * - DS18B20 temp sensor on GPIO13
 * - MLX90640 thermal camera on I2C (GPIO18/23)
 * - Delta SM70-CP-450 power supply via TCP
 *
 * Safety Features:
 * - Hard limits that require safety code to change
 * - Confirmation dialogs for all operations
 * - All setting changes are logged
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <FS.h>

// Adafruit MLX90640 library
#include <Adafruit_MLX90640.h>

// Data logging settings
#define LOG_FILE "/datalog.csv"
#define SAFETY_LOG_FILE "/safety.log"
#define MAX_LOG_SIZE 500000  // 500KB max log size
#define LOG_INTERVAL_MS 5000 // Log every 5 seconds when running

// Default safety code (change this!)
#define DEFAULT_SAFETY_CODE "1234"

const char* WIFI_SSID = "BTAC Medewerkers";
const char* WIFI_PASS = "Next3600$!";
const char* DELTA_IP = "192.168.1.16";
const int DELTA_PORT = 8462;

#define ONE_WIRE_BUS 13
#define PA_ENABLE_PIN   21
#define BLUE_LED_PIN    22  // Blue LED on ESP32-LyraT
#define I2C_SDA_PIN     18  // ES8311 audio codec (hardwired on LyraT)
#define I2C_SCL_PIN     23  // ES8311 audio codec (hardwired on LyraT)
#define MLX_SDA_PIN     15  // MLX90640 thermal camera (Wire1)
#define MLX_SCL_PIN     14  // MLX90640 thermal camera (Wire1)
#define ES8311_ADDR     0x18
#define SAMPLE_RATE     16000
#define I2S_NUM         I2S_NUM_0
#define I2S_MCLK_PIN    0
#define I2S_BCLK_PIN    5
#define I2S_LRCK_PIN    25
#define I2S_DOUT_PIN    26
#define I2S_DIN_PIN     35

WebServer server(80);
WiFiClient deltaClient;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
Preferences prefs;

volatile bool stopRequested = false;

// Blue LED status indicator
unsigned long lastLedToggle = 0;
bool ledState = false;

// Forward declaration for quick stop
void quickDeltaOff();

// MLX90640 variables
Adafruit_MLX90640 mlxSensor;
#define MLX_COLS 32
#define MLX_ROWS 24
#define MLX_PIXELS (MLX_COLS * MLX_ROWS)
float mlxFrame[MLX_PIXELS];
float mlxMin = 0, mlxMax = 0, mlxAvg = 0;
bool mlxConnected = false;
unsigned long lastMlxRead = 0;
#define MLX_REFRESH_MS 500  // Read thermal every 500ms

enum TestMode { MODE_IDLE, MODE_CHARGE, MODE_DISCHARGE, MODE_CYCLE };

// SAFETY LIMITS - These are the absolute boundaries that protect the battery
// These can ONLY be changed with the safety code
struct {
  // Voltage limits (for Li-ion: typically 2.5V-4.25V per cell)
  float absMinVoltage = 2.50;   // Absolute minimum voltage allowed
  float absMaxVoltage = 4.25;   // Absolute maximum voltage allowed

  // Current limits (depends on your battery and PSU)
  float absMaxChargeCurrent = 30.0;     // Max charge current allowed
  float absMaxDischargeCurrent = 30.0;  // Max discharge current allowed

  // Warning thresholds - values above these trigger confirmation
  float warnHighCurrent = 10.0;   // Warn if current > this
  float warnHighVoltage = 4.22;   // Warn if charge voltage > this
  float warnLowVoltage = 2.60;    // Warn if discharge voltage < this

  // Safety code for changing limits
  String safetyCode = DEFAULT_SAFETY_CODE;
} safetyLimits;

// Normal operating config (within safety limits)
struct {
  float minVoltage = 2.71;
  float maxVoltage = 4.20;
  float chargeCurrent = 24.0;
  float dischargeCurrent = 24.0;
  float tempAlarm = 45.0;
  float thermalAlarm = 60.0;  // Thermal camera alarm
  int numCycles = 4;
  int logInterval = 10;
} config;

struct {
  TestMode mode = MODE_IDLE;
  volatile bool running = false;
  int currentCycle = 0;
  float voltage = 0.0;
  float current = 0.0;
  float power = 0.0;
  float temperature = 0.0;
  float totalWh = 0.0;
  float totalAhCharge = 0.0;
  float totalAhDischarge = 0.0;
  unsigned long startTime = 0;
  unsigned long lastLogTime = 0;
  unsigned long lastMeasureTime = 0;
  String lastError = "";
  bool deltaConnected = false;
  unsigned long lastDeltaSuccess = 0;  // Last successful Delta communication
  int measureFailCount = 0;
  unsigned long lowCurrentStartTime = 0;
  bool lowCurrentActive = false;
  // Delta extended info
  float setVoltage = 0.0;      // Programmed voltage
  float setCurrent = 0.0;      // Programmed current (positive)
  float setCurrentNeg = 0.0;   // Programmed current (negative/discharge)
  bool cvMode = false;         // true = Constant Voltage, false = Constant Current
  float cableLoss = 0.0;       // Voltage drop in cables
  // Cycle tracking
  unsigned long cycleStartTime = 0;  // Start time of current cycle phase
  float cycleAhCharge = 0.0;         // Ah charged in current cycle
  float cycleAhDischarge = 0.0;      // Ah discharged in current cycle
  float peakTempThisCycle = 0.0;     // Peak temperature in current cycle
} status;

// Cycle history for tracking multiple cycles (SGS uses 100 cycles)
#define MAX_CYCLE_HISTORY 100
struct CycleData {
  float ahCharge;
  float ahDischarge;
  float whCharge;
  float whDischarge;
  unsigned long chargeDurationMs;
  unsigned long dischargeDurationMs;
  float peakTemp;
  float coulombEfficiency;  // Ah_discharge / Ah_charge * 100
  float energyEfficiency;   // Wh_discharge / Wh_charge * 100
  bool completed;
};
CycleData cycleHistory[MAX_CYCLE_HISTORY];
int cycleHistoryCount = 0;

// Anorgion Test Configuration - Professional battery testing parameters
struct {
  // Cell specifications (for energy density calculation)
  float cellWeightKg = 0.312;      // Cell weight in kg (default from SGS: 312g)
  float cellVolumeLiter = 0.120;   // Cell volume in liters (default from SGS: 120ml)
  float nominalCapacity = 24.0;    // Nominal capacity in Ah
  float nominalVoltage = 3.55;     // Nominal voltage

  // SGS test parameters
  float chargeVoltage = 4.15;      // Charge cutoff voltage (SGS: 4.15V)
  float chargeCurrent = 24.0;      // Charge current (SGS: 24A = 1C)
  float cutoffCurrent = 1.5;       // CV phase cutoff current (SGS: 1.5A = C/16)
  float dischargeVoltage = 2.70;   // Discharge cutoff voltage (SGS: 2.7V)
  float dischargeCurrent = 24.0;   // Discharge current (SGS: 24A = 1C)

  int restTimeSeconds = 300;       // Rest between charge/discharge (SGS: 5 min)
  int cycleRestSeconds = 300;      // Rest between full cycles (SGS: 5 min)
  int totalCycles = 100;           // Total cycles to run (SGS: 100)
  float maxTempStop = 45.0;        // Temperature stop condition (SGS: 45°C)

  // Test state
  bool sgsTestActive = false;
  int sgsCurrentPhase = 0;         // 0=idle, 1=charging, 2=rest1, 3=discharging, 4=rest2
  unsigned long phaseStartTime = 0;
  float currentCycleWhCharge = 0;
  float currentCycleWhDischarge = 0;
} sgsConfig;

// ============== MLX90640 THERMAL CAMERA ==============

bool mlxInit() {
  Serial.println("[MLX] Initializing...");

  if (!mlxSensor.begin(MLX90640_I2CADDR_DEFAULT, &Wire1)) {
    Serial.println("[MLX] Not found on I2C");
    return false;
  }

  // Set to 4Hz refresh rate (good balance)
  mlxSensor.setRefreshRate(MLX90640_4_HZ);

  // Set to chess pattern for better accuracy
  mlxSensor.setMode(MLX90640_CHESS);

  // Set resolution
  mlxSensor.setResolution(MLX90640_ADC_18BIT);

  Serial.println("[MLX] Initialized OK - 32x24 @ 4Hz");
  return true;
}

void mlxRead() {
  if (!mlxConnected) return;
  if (millis() - lastMlxRead < MLX_REFRESH_MS) return;
  lastMlxRead = millis();

  if (mlxSensor.getFrame(mlxFrame) != 0) {
    return;  // Error reading frame
  }

  // Calculate min/max/avg
  mlxMin = 1000;
  mlxMax = -40;
  float sum = 0;
  int validCount = 0;

  for (int i = 0; i < MLX_PIXELS; i++) {
    float t = mlxFrame[i];
    if (t > -40 && t < 300) {  // Valid range
      if (t < mlxMin) mlxMin = t;
      if (t > mlxMax) mlxMax = t;
      sum += t;
      validCount++;
    }
  }

  if (validCount > 0) {
    mlxAvg = sum / validCount;
  }
}

// ============== DATA LOGGING ==============
unsigned long lastLogWrite = 0;
bool loggingEnabled = false;

bool spiffsReady = false;

void initSPIFFS() {
  Serial.println("[SPIFFS] Initializing...");

  // Try to mount - if it fails, skip (don't format, takes too long)
  if (SPIFFS.begin(false)) {
    spiffsReady = true;
    Serial.printf("[SPIFFS] Mounted OK - Total: %u, Used: %u bytes\n",
      SPIFFS.totalBytes(), SPIFFS.usedBytes());
  } else {
    Serial.println("[SPIFFS] Not available - logging disabled");
    Serial.println("[SPIFFS] To enable: run 'pio run -t uploadfs' once");
    spiffsReady = false;
  }
}

void startNewLog() {
  if (!spiffsReady) return;

  // Create new log file with header
  File f = SPIFFS.open(LOG_FILE, FILE_WRITE);
  if (f) {
    f.println("timestamp,voltage,current,power,temp_ds18b20,temp_thermal_max,mode,ah_charge,ah_discharge,wh");
    f.close();
    loggingEnabled = true;
    Serial.println("[LOG] New log started");
  }
}

void appendLogEntry() {
  if (!spiffsReady || !loggingEnabled || !status.running) return;
  if (millis() - lastLogWrite < LOG_INTERVAL_MS) return;
  lastLogWrite = millis();

  // Check file size
  File f = SPIFFS.open(LOG_FILE, FILE_APPEND);
  if (!f) return;

  if (f.size() > MAX_LOG_SIZE) {
    f.close();
    Serial.println("[LOG] Max size reached, stopping log");
    loggingEnabled = false;
    return;
  }

  // Get elapsed time in seconds
  unsigned long elapsed = (millis() - status.startTime) / 1000;

  // Mode string
  const char* modeStr = "IDLE";
  if (status.mode == MODE_CHARGE) modeStr = "CHARGE";
  else if (status.mode == MODE_DISCHARGE) modeStr = "DISCHARGE";
  else if (status.mode == MODE_CYCLE) modeStr = status.current >= 0 ? "CYCLE_CHG" : "CYCLE_DIS";

  // Write CSV line
  f.printf("%lu,%.4f,%.3f,%.2f,%.2f,%.2f,%s,%.4f,%.4f,%.3f\n",
    elapsed, status.voltage, status.current, status.power,
    status.temperature, mlxMax, modeStr,
    status.totalAhCharge, status.totalAhDischarge, status.totalWh);
  f.close();
}

void deleteLog() {
  if (!spiffsReady) return;
  SPIFFS.remove(LOG_FILE);
  loggingEnabled = false;
  Serial.println("[LOG] Log deleted");
}

size_t getLogSize() {
  if (!spiffsReady) return 0;
  File f = SPIFFS.open(LOG_FILE, FILE_READ);
  if (!f) return 0;
  size_t s = f.size();
  f.close();
  return s;
}

// ============== CONFIG ==============
void saveConfig() {
  prefs.begin("bt", false);
  prefs.putFloat("minV", config.minVoltage);
  prefs.putFloat("maxV", config.maxVoltage);
  prefs.putFloat("chgI", config.chargeCurrent);
  prefs.putFloat("disI", config.dischargeCurrent);
  prefs.putFloat("tmpA", config.tempAlarm);
  prefs.putFloat("thrmA", config.thermalAlarm);
  prefs.putInt("cyc", config.numCycles);
  prefs.putInt("logI", config.logInterval);
  prefs.end();
}

void loadConfig() {
  prefs.begin("bt", true);
  config.minVoltage = prefs.getFloat("minV", 2.71);
  config.maxVoltage = prefs.getFloat("maxV", 4.20);
  config.chargeCurrent = prefs.getFloat("chgI", 24.0);
  config.dischargeCurrent = prefs.getFloat("disI", 24.0);
  config.tempAlarm = prefs.getFloat("tmpA", 45.0);
  config.thermalAlarm = prefs.getFloat("thrmA", 60.0);
  config.numCycles = prefs.getInt("cyc", 4);
  config.logInterval = prefs.getInt("logI", 10);
  prefs.end();
  Serial.printf("[CFG] V:%.2f-%.2f I:%.1f/%.1f T:%.0f TH:%.0f C:%d\n",
    config.minVoltage, config.maxVoltage, config.chargeCurrent,
    config.dischargeCurrent, config.tempAlarm, config.thermalAlarm, config.numCycles);
}

// ============== SAFETY LIMITS ==============
void saveSafetyLimits() {
  prefs.begin("safety", false);
  prefs.putFloat("absMinV", safetyLimits.absMinVoltage);
  prefs.putFloat("absMaxV", safetyLimits.absMaxVoltage);
  prefs.putFloat("absMaxCI", safetyLimits.absMaxChargeCurrent);
  prefs.putFloat("absMaxDI", safetyLimits.absMaxDischargeCurrent);
  prefs.putFloat("warnHiI", safetyLimits.warnHighCurrent);
  prefs.putFloat("warnHiV", safetyLimits.warnHighVoltage);
  prefs.putFloat("warnLoV", safetyLimits.warnLowVoltage);
  prefs.putString("code", safetyLimits.safetyCode);
  prefs.end();
}

void loadSafetyLimits() {
  prefs.begin("safety", true);
  safetyLimits.absMinVoltage = prefs.getFloat("absMinV", 2.50);
  safetyLimits.absMaxVoltage = prefs.getFloat("absMaxV", 4.25);
  safetyLimits.absMaxChargeCurrent = prefs.getFloat("absMaxCI", 30.0);
  safetyLimits.absMaxDischargeCurrent = prefs.getFloat("absMaxDI", 30.0);
  safetyLimits.warnHighCurrent = prefs.getFloat("warnHiI", 10.0);
  safetyLimits.warnHighVoltage = prefs.getFloat("warnHiV", 4.22);
  safetyLimits.warnLowVoltage = prefs.getFloat("warnLoV", 2.60);
  safetyLimits.safetyCode = prefs.getString("code", DEFAULT_SAFETY_CODE);
  prefs.end();
  Serial.printf("[SAFETY] Limits: V:%.2f-%.2f A:%.1f/%.1f Warn:%.1fA\n",
    safetyLimits.absMinVoltage, safetyLimits.absMaxVoltage,
    safetyLimits.absMaxChargeCurrent, safetyLimits.absMaxDischargeCurrent,
    safetyLimits.warnHighCurrent);
}

// Log safety-related events
void logSafetyEvent(const String& event) {
  Serial.printf("[SAFETY LOG] %s\n", event.c_str());

  if (!spiffsReady) return;

  File f = SPIFFS.open(SAFETY_LOG_FILE, FILE_APPEND);
  if (f) {
    // Get uptime as timestamp
    unsigned long secs = millis() / 1000;
    f.printf("[%lu] %s\n", secs, event.c_str());
    f.close();
  }
}

// Validate charge parameters - returns error message or empty string if OK
String validateChargeParams(float voltage, float current) {
  if (voltage > safetyLimits.absMaxVoltage) {
    return "BLOCKED: Voltage " + String(voltage, 2) + "V exceeds maximum " +
           String(safetyLimits.absMaxVoltage, 2) + "V";
  }
  if (voltage < safetyLimits.absMinVoltage) {
    return "BLOCKED: Voltage " + String(voltage, 2) + "V below minimum " +
           String(safetyLimits.absMinVoltage, 2) + "V";
  }
  if (current > safetyLimits.absMaxChargeCurrent) {
    return "BLOCKED: Current " + String(current, 1) + "A exceeds maximum " +
           String(safetyLimits.absMaxChargeCurrent, 1) + "A";
  }
  if (current < 0.1) {
    return "BLOCKED: Current too low";
  }
  return "";  // OK
}

// Validate discharge parameters
String validateDischargeParams(float voltage, float current) {
  if (voltage > safetyLimits.absMaxVoltage) {
    return "BLOCKED: Voltage " + String(voltage, 2) + "V exceeds maximum " +
           String(safetyLimits.absMaxVoltage, 2) + "V";
  }
  if (voltage < safetyLimits.absMinVoltage) {
    return "BLOCKED: Voltage " + String(voltage, 2) + "V below minimum " +
           String(safetyLimits.absMinVoltage, 2) + "V";
  }
  if (current > safetyLimits.absMaxDischargeCurrent) {
    return "BLOCKED: Current " + String(current, 1) + "A exceeds maximum " +
           String(safetyLimits.absMaxDischargeCurrent, 1) + "A";
  }
  if (current < 0.1) {
    return "BLOCKED: Current too low";
  }
  return "";  // OK
}

// Get warnings for parameters (not blocking, just warnings)
String getParamWarnings(float voltage, float current, bool isCharge) {
  String warnings = "";

  if (current > safetyLimits.warnHighCurrent) {
    warnings += "HIGH CURRENT (" + String(current, 1) + "A)! ";
  }

  if (isCharge && voltage > safetyLimits.warnHighVoltage) {
    warnings += "HIGH VOLTAGE (" + String(voltage, 2) + "V)! ";
  }

  if (!isCharge && voltage < safetyLimits.warnLowVoltage) {
    warnings += "LOW CUTOFF (" + String(voltage, 2) + "V)! ";
  }

  return warnings;
}

// ============== AUDIO ==============
void es8311_init() {
  Wire.beginTransmission(ES8311_ADDR); Wire.write(0x00); Wire.write(0x1F); Wire.endTransmission(); delay(20);
  Wire.beginTransmission(ES8311_ADDR); Wire.write(0x00); Wire.write(0x00); Wire.endTransmission(); delay(20);
  uint8_t r[][2]={{0x01,0x3F},{0x02,0x00},{0x03,0x10},{0x04,0x10},{0x05,0x00},{0x09,0x0C},{0x0A,0x0C},
    {0x0D,0x01},{0x0E,0x02},{0x0F,0x44},{0x10,0x0C},{0x32,0xC0},{0x00,0x80},{0x01,0x3F},{0x14,0x1A},{0x12,0x00},{0x13,0x10}};
  for(auto &x:r){Wire.beginTransmission(ES8311_ADDR);Wire.write(x[0]);Wire.write(x[1]);Wire.endTransmission();}
  delay(50);
}

void i2s_init() {
  i2s_config_t cfg={.mode=(i2s_mode_t)(I2S_MODE_MASTER|I2S_MODE_TX),.sample_rate=SAMPLE_RATE,
    .bits_per_sample=I2S_BITS_PER_SAMPLE_16BIT,.channel_format=I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format=I2S_COMM_FORMAT_STAND_I2S,.intr_alloc_flags=ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count=8,.dma_buf_len=64,.use_apll=true,.tx_desc_auto_clear=true};
  i2s_pin_config_t pins={.mck_io_num=I2S_MCLK_PIN,.bck_io_num=I2S_BCLK_PIN,
    .ws_io_num=I2S_LRCK_PIN,.data_out_num=I2S_DOUT_PIN,.data_in_num=I2S_DIN_PIN};
  i2s_driver_install(I2S_NUM,&cfg,0,NULL);
  i2s_set_pin(I2S_NUM,&pins);
}

void playTone(int freq,int ms){
  int samples=(SAMPLE_RATE*ms)/1000;int16_t buf[128];size_t w;
  float phase=0,inc=2.0*PI*freq/SAMPLE_RATE;int rem=samples;
  while(rem>0){int c=min(64,rem);for(int i=0;i<c;i++){
    int16_t v=(int16_t)(sin(phase)*8000);buf[i*2]=v;buf[i*2+1]=v;
    phase+=inc;if(phase>=2*PI)phase-=2*PI;}
  i2s_write(I2S_NUM,buf,c*4,&w,portMAX_DELAY);rem-=c;}
}
void playSilence(int ms){int16_t buf[128]={0};size_t w;int rem=(SAMPLE_RATE*ms)/1000;
  while(rem>0){int c=min(64,rem);i2s_write(I2S_NUM,buf,c*4,&w,portMAX_DELAY);rem-=c;}}

void beepOK(){playTone(1000,100);playSilence(50);playTone(1500,150);}
void beepFail(){playTone(400,300);}
void beepStart(){playTone(1000,100);playSilence(100);playTone(1000,100);playSilence(100);playTone(1200,400);}
void beepStop(){playTone(800,100);playSilence(100);playTone(600,100);}
void beepDone(){playTone(800,150);playSilence(50);playTone(1000,150);playSilence(50);playTone(1200,150);playSilence(50);playTone(1500,400);}
void beepThermalWarn(){playTone(2000,100);playSilence(100);playTone(2000,100);}

// ============== DELTA COMMUNICATIE ==============

bool deltaConnect() {
  // Don't kill existing healthy connection
  if (deltaClient.connected()) {
    return true;
  }

  // Only stop if we need to reconnect
  deltaClient.stop();
  delay(200);
  yield();

  deltaClient.setTimeout(10);
  if (!deltaClient.connect(DELTA_IP, DELTA_PORT)) {
    Serial.println("[DELTA] Connect FAILED");
    status.deltaConnected = false;
    return false;
  }

  status.deltaConnected = true;
  delay(300);  // Give Delta time to be ready
  yield();
  while(deltaClient.available()) deltaClient.read();
  Serial.println("[DELTA] Connected OK");
  return true;
}

// Force reconnect - only use when you explicitly need a fresh connection
bool deltaForceReconnect() {
  deltaClient.stop();
  delay(200);
  status.deltaConnected = false;
  return deltaConnect();
}

String deltaQuery(String cmd) {
  if (!deltaClient.connected()) {
    if (!deltaConnect()) return "";
  }

  // Drain any leftover bytes from previous responses
  while(deltaClient.available()) deltaClient.read();

  deltaClient.print(cmd + "\r\n");
  deltaClient.flush();
  delay(30);

  String response = "";
  unsigned long start = millis();
  unsigned long timeout = 1500;  // Generous timeout for slow Delta

  while ((millis() - start) < timeout) {
    yield();
    // NO server.handleClient() here - prevents re-entrant SCPI corruption

    if (deltaClient.available()) {
      char c = deltaClient.read();
      if (c == '\n' || c == '\r') {
        if (response.length() > 0) break;
      } else if (c >= 32 && c <= 126) {
        response += c;
      }
      if (response.length() >= 30) break;
    } else {
      delay(5);
    }
  }

  response.trim();
  if (response.length() > 0) {
    status.lastDeltaSuccess = millis();
  }
  return response;
}

void deltaSet(String cmd) {
  if (!deltaClient.connected()) {
    if (!deltaConnect()) return;
  }
  // NO server.handleClient() here - prevents re-entrant SCPI corruption
  deltaClient.print(cmd + "\r\n");
  deltaClient.flush();
  delay(80);  // Give Delta time to process command
  yield();
}

bool deltaSetupCharge(float voltage, float current) {
  Serial.printf("[DELTA] Setup CHARGE: %.2fV %.1fA\n", voltage, current);

  // Use existing connection if healthy, only reconnect if needed
  if (!deltaClient.connected()) {
    if (!deltaConnect()) return false;
  }

  yield();
  deltaSet("OUTPut OFF");
  delay(500);  // Extra time for Delta to turn off
  yield();

  deltaSet("SYSTem:REMote:CV Remote");
  yield();
  deltaSet("SYSTem:REMote:CC Remote");
  yield();
  deltaSet("SYSTem:REMote:CP Remote");
  yield();

  deltaSet("SOURce:VOLtage " + String(voltage, 2));
  yield();
  deltaSet("SOURce:CURrent " + String(current, 2));
  yield();
  float power = voltage * current * 1.5;
  deltaSet("SOURce:POWer " + String(power, 0));
  yield();

  deltaSet("SOURce:CURrent:NEGative 0");
  yield();
  deltaSet("SOURce:POWer:NEGative 0");
  yield();

  delay(300);
  deltaSet("OUTPut ON");
  delay(500);  // Extra time for output to stabilize
  yield();

  String vSet = deltaQuery("SOURce:VOLtage?");
  yield();
  String iSet = deltaQuery("SOURce:CURrent?");
  yield();
  String pSet = deltaQuery("SOURce:POWer?");
  Serial.printf("[DELTA] Set: V=%s I=%s P=%s\n", vSet.c_str(), iSet.c_str(), pSet.c_str());

  return true;
}

bool deltaSetupDischarge(float voltage, float current) {
  Serial.printf("[DELTA] Setup DISCHARGE: %.2fV %.1fA\n", voltage, current);

  // Use existing connection if healthy, only reconnect if needed
  if (!deltaClient.connected()) {
    if (!deltaConnect()) return false;
  }

  yield();
  deltaSet("OUTPut OFF");
  delay(500);  // Extra time for Delta to turn off
  yield();

  deltaSet("SYSTem:REMote:CV Remote");
  yield();
  deltaSet("SYSTem:REMote:CC Remote");
  yield();
  deltaSet("SYSTem:REMote:CP Remote");
  yield();

  deltaSet("SOURce:VOLtage " + String(voltage, 2));
  yield();
  deltaSet("SOURce:CURrent 0");
  yield();
  deltaSet("SOURce:POWer 0");
  yield();

  deltaSet("SOURce:CURrent:NEGative -" + String(current, 2));
  yield();
  float power = 70.0 * current * 1.5;
  deltaSet("SOURce:POWer:NEGative -" + String(power, 0));
  yield();

  delay(300);
  deltaSet("OUTPut ON");
  delay(500);  // Extra time for output to stabilize
  yield();

  String vSet = deltaQuery("SOURce:VOLtage?");
  yield();
  String iNeg = deltaQuery("SOURce:CURrent:NEGative?");
  yield();
  String pNeg = deltaQuery("SOURce:POWer:NEGative?");
  Serial.printf("[DELTA] Set: V=%s I-=%s P-=%s\n", vSet.c_str(), iNeg.c_str(), pNeg.c_str());

  return true;
}

// Hot-update charge parameters without cycling output (live adjustment)
bool deltaUpdateChargeParams(float voltage, float current) {
  Serial.printf("[DELTA] LIVE UPDATE CHARGE: %.2fV %.1fA\n", voltage, current);

  if (!deltaClient.connected()) {
    if (!deltaConnect()) return false;
  }

  deltaSet("SOURce:VOLtage " + String(voltage, 2));
  yield();
  deltaSet("SOURce:CURrent " + String(current, 2));
  yield();
  float power = voltage * current * 1.5;
  deltaSet("SOURce:POWer " + String(power, 0));
  yield();

  Serial.printf("[DELTA] Live update done: %.2fV %.1fA\n", voltage, current);
  return true;
}

// Hot-update discharge parameters without cycling output (live adjustment)
bool deltaUpdateDischargeParams(float voltage, float current) {
  Serial.printf("[DELTA] LIVE UPDATE DISCHARGE: %.2fV %.1fA\n", voltage, current);

  if (!deltaClient.connected()) {
    if (!deltaConnect()) return false;
  }

  deltaSet("SOURce:VOLtage " + String(voltage, 2));
  yield();
  deltaSet("SOURce:CURrent:NEGative -" + String(current, 2));
  yield();
  float power = 70.0 * current * 1.5;
  deltaSet("SOURce:POWer:NEGative -" + String(power, 0));
  yield();

  Serial.printf("[DELTA] Live update done: %.2fV %.1fA\n", voltage, current);
  return true;
}

bool deltaReadMeasurements() {
  // Check stop FIRST
  if (stopRequested) return false;

  // NO server.handleClient() here - prevents re-entrant SCPI corruption
  yield();

  if (!deltaClient.connected()) {
    if (!deltaConnect()) {
      status.measureFailCount++;
      return false;
    }
  }

  // Check stop between each query
  if (stopRequested) return false;
  String vStr = deltaQuery("MEASure:VOLtage?");

  if (stopRequested) return false;
  String iStr = deltaQuery("MEASure:CURrent?");

  if (stopRequested) return false;
  String pStr = deltaQuery("MEASure:POWer?");

  if (vStr.length() == 0) {
    status.measureFailCount++;
    if (status.measureFailCount >= 3) {
      Serial.println("[DELTA] Reconnecting...");
      deltaConnect();
      status.measureFailCount = 0;
    }
    return false;
  }

  status.measureFailCount = 0;

  float v = vStr.toFloat();
  float i = iStr.toFloat();
  float p = pStr.toFloat();

  // ============== HARD SAFETY LIMITS - IMMEDIATE STOP ==============
  // These limits are NON-NEGOTIABLE. If voltage is outside safe range,
  // we IMMEDIATELY turn off Delta output, no matter what.
  #define HARD_MIN_VOLTAGE 2.70
  #define HARD_MAX_VOLTAGE 4.15

  // LOW VOLTAGE HARD SAFETY - RE-ENABLED v7.5
  if (v > 0.5 && v < HARD_MIN_VOLTAGE && i < -0.5) {
    Serial.printf("\n!!! HARD SAFETY STOP: V=%.3fV < %.2fV (discharging) !!!\n", v, HARD_MIN_VOLTAGE);
    deltaSet("OUTPut OFF");
    delay(100);
    deltaSet("OUTPut OFF");
    stopRequested = true;
    status.running = false;
    status.mode = MODE_IDLE;
    status.lastError = "HARD STOP: Voltage below " + String(HARD_MIN_VOLTAGE, 2) + "V";
    logSafetyEvent("HARD STOP: V=" + String(v, 3) + "V < min (discharge)");
    beepFail();
    return false;
  }

  if (v > HARD_MAX_VOLTAGE) {
    // CRITICAL: Battery voltage too high! STOP IMMEDIATELY!
    Serial.printf("\n!!! HARD SAFETY STOP: V=%.3fV > %.2fV !!!\n", v, HARD_MAX_VOLTAGE);
    deltaSet("OUTPut OFF");
    delay(100);
    deltaSet("OUTPut OFF");  // Send twice to be sure
    stopRequested = true;
    status.running = false;
    status.mode = MODE_IDLE;
    status.lastError = "HARD STOP: Voltage above " + String(HARD_MAX_VOLTAGE, 2) + "V";
    logSafetyEvent("HARD STOP: V=" + String(v, 3) + "V > max");
    beepFail();
    return false;
  }
  // ============== END HARD SAFETY LIMITS ==============

  if (v >= 0 && v <= 100) {
    status.voltage = v;
    status.current = i;
    status.power = p;

    // Read programmed/set values - but check stop between each!
    if (stopRequested) return false;
    String setVStr = deltaQuery("SOURce:VOLtage?");

    if (stopRequested) return false;
    String setIStr = deltaQuery("SOURce:CURrent?");

    if (stopRequested) return false;
    String setINegStr = deltaQuery("SOURce:CURrent:NEGative?");

    if (setVStr.length() > 0) {
      status.setVoltage = setVStr.toFloat();
    }
    if (setIStr.length() > 0) {
      status.setCurrent = setIStr.toFloat();
    }
    if (setINegStr.length() > 0) {
      status.setCurrentNeg = abs(setINegStr.toFloat());
    }

    // Determine CV/CC mode based on whether we're at the voltage or current limit
    // In CV mode: actual voltage ≈ set voltage, current varies
    // In CC mode: actual current ≈ set current, voltage varies
    float targetCurrent = (i >= 0) ? status.setCurrent : status.setCurrentNeg;
    float voltageDiff = abs(status.setVoltage - v);
    float currentDiff = abs(targetCurrent - abs(i));

    // If voltage is close to set point and current is not at limit -> CV mode
    // If current is close to limit -> CC mode
    if (targetCurrent > 0.1) {
      float currentRatio = abs(i) / targetCurrent;
      status.cvMode = (currentRatio < 0.95);  // CV if not at current limit
    } else {
      status.cvMode = true;  // No current set, assume CV
    }

    // Calculate cable loss (difference between set voltage and measured at sense)
    // Only meaningful when charging (current > 0) and in CV mode
    if (i > 0.5 && status.cvMode) {
      status.cableLoss = status.setVoltage - v;
    } else if (i < -0.5) {
      // During discharge, sense voltage is higher than "set" due to battery
      status.cableLoss = v - status.setVoltage;
    } else {
      status.cableLoss = 0;
    }

    if (status.running && status.lastMeasureTime > 0) {
      float hours = (millis() - status.lastMeasureTime) / 3600000.0;
      status.totalWh += abs(p) * hours;
      if (i > 0.01) {
        status.totalAhCharge += i * hours;
        status.cycleAhCharge += i * hours;  // Per-cycle tracking
        sgsConfig.currentCycleWhCharge += abs(p) * hours;  // Wh tracking for efficiency
      } else if (i < -0.01) {
        status.totalAhDischarge += abs(i) * hours;
        status.cycleAhDischarge += abs(i) * hours;  // Per-cycle tracking
        sgsConfig.currentCycleWhDischarge += abs(p) * hours;  // Wh tracking for efficiency
      }
      // Track peak temperature this cycle
      if (status.temperature > status.peakTempThisCycle) {
        status.peakTempThisCycle = status.temperature;
      }
      if (mlxConnected && mlxMax > status.peakTempThisCycle) {
        status.peakTempThisCycle = mlxMax;
      }
    }
    status.lastMeasureTime = millis();

    Serial.printf("[M] V=%.3f(set:%.2f) I=%.2f(set:%.1f) %s Loss=%.3fV\n",
      v, status.setVoltage, i, targetCurrent,
      status.cvMode ? "CV" : "CC", status.cableLoss);
    return true;
  }

  return false;
}

void deltaStop() {
  Serial.println("[DELTA] STOP - Emergency shutdown");

  // Try multiple times to ensure output is off
  for (int attempt = 0; attempt < 3; attempt++) {
    yield();
    if (deltaClient.connected() || deltaConnect()) {
      // Zero setpoints first to prevent residual current
      deltaClient.print("SOURce:CURrent 0\r\n");
      delay(80);
      deltaClient.print("SOURce:CURrent:NEGative 0\r\n");
      delay(80);
      deltaClient.print("SOURce:POWer 0\r\n");
      delay(80);
      deltaClient.print("SOURce:POWer:NEGative 0\r\n");
      delay(80);
      deltaClient.print("SOURce:VOLtage 0\r\n");
      delay(80);
      deltaClient.print("OUTPut OFF\r\n");
      deltaClient.flush();
      delay(200);
      yield();
      Serial.printf("[DELTA] Stop attempt %d sent (with zero setpoints)\n", attempt + 1);
    }
  }

  deltaClient.stop();
  status.deltaConnected = false;
  Serial.println("[DELTA] Output disabled, setpoints zeroed, connection closed");
}

String deltaPing() {
  Serial.println("[DELTA] PING");

  if (!deltaConnect()) {
    beepFail();
    return "FAIL: Cannot connect";
  }

  String idn = deltaQuery("*IDN?");
  // Don't close connection - leave it for future use

  if (idn.length() > 5) {
    beepOK();
    Serial.printf("[DELTA] IDN: %s\n", idn.c_str());
    return "OK: " + idn;
  }

  beepFail();
  return "FAIL: No response";
}

// ============== TEMPERATURE ==============
void readTemp() {
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);
  if (t > -50 && t < 100 && t != 85.0) status.temperature = t;
}

// ============== TEST CONTROL ==============
void resetStats() {
  status.totalWh = 0;
  status.totalAhCharge = 0;
  status.totalAhDischarge = 0;
  status.startTime = millis();
  status.lastLogTime = millis();
  status.lastMeasureTime = millis();
  status.lastError = "";
  status.measureFailCount = 0;
  status.lowCurrentStartTime = 0;
  status.lowCurrentActive = false;
  stopRequested = false;
  // Reset cycle tracking
  status.cycleStartTime = millis();
  status.cycleAhCharge = 0;
  status.cycleAhDischarge = 0;
  status.peakTempThisCycle = 0;
}

void resetCycleHistory() {
  cycleHistoryCount = 0;
  for (int i = 0; i < MAX_CYCLE_HISTORY; i++) {
    cycleHistory[i] = {0, 0, 0, 0, 0, false};
  }
}

void saveCycleToHistory(bool isChargePhase) {
  if (cycleHistoryCount >= MAX_CYCLE_HISTORY) return;

  // For cycle mode, we save after discharge phase completes
  // Each "cycle" = charge + discharge
  if (!isChargePhase && status.currentCycle > 0) {
    int idx = status.currentCycle - 1;
    if (idx < MAX_CYCLE_HISTORY) {
      cycleHistory[idx].ahCharge = status.cycleAhCharge;
      cycleHistory[idx].ahDischarge = status.cycleAhDischarge;
      cycleHistory[idx].whCharge = sgsConfig.currentCycleWhCharge;
      cycleHistory[idx].whDischarge = sgsConfig.currentCycleWhDischarge;
      cycleHistory[idx].dischargeDurationMs = millis() - status.cycleStartTime;
      cycleHistory[idx].peakTemp = status.peakTempThisCycle;

      // Calculate efficiencies
      if (status.cycleAhCharge > 0.01) {
        cycleHistory[idx].coulombEfficiency = (status.cycleAhDischarge / status.cycleAhCharge) * 100.0;
      } else {
        cycleHistory[idx].coulombEfficiency = 0;
      }
      if (sgsConfig.currentCycleWhCharge > 0.01) {
        cycleHistory[idx].energyEfficiency = (sgsConfig.currentCycleWhDischarge / sgsConfig.currentCycleWhCharge) * 100.0;
      } else {
        cycleHistory[idx].energyEfficiency = 0;
      }

      cycleHistory[idx].completed = true;
      cycleHistoryCount = max(cycleHistoryCount, status.currentCycle);

      Serial.printf("[CYCLE] Saved cycle %d: %.3fAh/%.2fWh charge, %.3fAh/%.2fWh discharge, CE=%.1f%%, EE=%.1f%%\n",
        status.currentCycle, status.cycleAhCharge, sgsConfig.currentCycleWhCharge,
        status.cycleAhDischarge, sgsConfig.currentCycleWhDischarge,
        cycleHistory[idx].coulombEfficiency, cycleHistory[idx].energyEfficiency);
    }
  }
}

void startNewCyclePhase() {
  status.cycleStartTime = millis();
  status.cycleAhCharge = 0;
  status.cycleAhDischarge = 0;
  status.peakTempThisCycle = 0;
  sgsConfig.currentCycleWhCharge = 0;
  sgsConfig.currentCycleWhDischarge = 0;
}

void startCharge(float voltage, float current) {
  if (status.running) return;

  Serial.printf("\n===== CHARGE %.2fV %.1fA =====\n", voltage, current);
  resetStats();
  startNewLog();

  if (!deltaSetupCharge(voltage, current)) {
    status.lastError = "Delta connect failed";
    beepFail();
    return;
  }

  beepStart();
  status.mode = MODE_CHARGE;
  status.running = true;
}

void startDischarge(float voltage, float current) {
  if (status.running) return;

  Serial.printf("\n===== DISCHARGE %.2fV %.1fA =====\n", voltage, current);
  resetStats();
  startNewLog();

  if (!deltaSetupDischarge(voltage, current)) {
    status.lastError = "Delta connect failed";
    beepFail();
    return;
  }

  beepStart();
  status.mode = MODE_DISCHARGE;
  status.running = true;
}

void startCycle() {
  if (status.running) return;

  Serial.printf("\n===== CYCLE %.2f-%.2fV =====\n", config.minVoltage, config.maxVoltage);
  resetStats();
  resetCycleHistory();  // Clear previous cycle data
  startNewLog();

  if (!deltaSetupCharge(config.maxVoltage, config.chargeCurrent)) {
    status.lastError = "Delta connect failed";
    beepFail();
    return;
  }

  beepStart();
  status.mode = MODE_CYCLE;
  status.running = true;
  status.currentCycle = 1;
}

void stopTest() {
  Serial.println("\n===== STOP =====");
  status.running = false;
  status.mode = MODE_IDLE;
  stopRequested = false;

  // Use quick off instead of full deltaStop
  quickDeltaOff();
  beepStop();
}

void updateTest() {
  if (!status.running || stopRequested) {
    if (stopRequested) stopTest();
    return;
  }

  deltaReadMeasurements();
  readTemp();
  appendLogEntry();

  if (!status.running) return;

  if (status.measureFailCount >= 10) {
    status.lastError = "Communication lost";
    stopTest();
    beepFail();
    return;
  }

  // DS18B20 temp alarm
  if (status.temperature > config.tempAlarm) {
    status.lastError = "TEMP ALARM (DS18B20)";
    stopTest();
    beepFail();
    return;
  }

  // MLX90640 thermal alarm - check max temp
  if (mlxConnected && mlxMax > config.thermalAlarm) {
    status.lastError = "THERMAL ALARM (MLX90640): " + String(mlxMax, 1) + "C";
    stopTest();
    beepFail();
    return;
  }

  // Log
  if (millis() - status.lastLogTime >= (unsigned long)config.logInterval * 1000) {
    status.lastLogTime = millis();
    char modeChar = 'I';
    if (status.mode == MODE_CHARGE) modeChar = 'C';
    else if (status.mode == MODE_DISCHARGE) modeChar = 'D';
    else if (status.mode == MODE_CYCLE) modeChar = status.current >= 0 ? 'C' : 'D';
    Serial.printf("[%c] V=%.3f I=%.2f T=%.1f TH=%.1f AhC=%.3f AhD=%.3f\n",
      modeChar, status.voltage, status.current, status.temperature, mlxMax,
      status.totalAhCharge, status.totalAhDischarge);
  }

  if (status.voltage < 0.1) return;

  // Mode logic
  switch (status.mode) {
    case MODE_CHARGE:
      if (status.voltage >= config.maxVoltage - 0.02) {
        if (abs(status.current) < 4.0) {
          if (!status.lowCurrentActive) {
            status.lowCurrentActive = true;
            status.lowCurrentStartTime = millis();
            Serial.println("[CHARGE] Low current detected, starting 2 min timer");
          } else if (millis() - status.lowCurrentStartTime >= 120000) {
            Serial.println("[CHARGE] Target reached (2 min low current)");
            beepDone();
            stopTest();
          }
        } else {
          if (status.lowCurrentActive) {
            Serial.println("[CHARGE] Current increased, timer reset");
          }
          status.lowCurrentActive = false;
          status.lowCurrentStartTime = 0;
        }
      }
      break;

    case MODE_DISCHARGE:
      if (status.voltage <= config.minVoltage + 0.02) {
        if (abs(status.current) < 4.0) {
          if (!status.lowCurrentActive) {
            status.lowCurrentActive = true;
            status.lowCurrentStartTime = millis();
            Serial.println("[DISCHARGE] Low current detected, starting 2 min timer");
          } else if (millis() - status.lowCurrentStartTime >= 120000) {
            Serial.println("[DISCHARGE] Target reached (2 min low current)");
            beepDone();
            stopTest();
          }
        } else {
          if (status.lowCurrentActive) {
            Serial.println("[DISCHARGE] Current increased, timer reset");
          }
          status.lowCurrentActive = false;
          status.lowCurrentStartTime = 0;
        }
      }
      break;

    case MODE_CYCLE:
      if (status.current >= -0.1) {
        if (status.voltage >= config.maxVoltage - 0.02) {
          if (abs(status.current) < 4.0) {
            if (!status.lowCurrentActive) {
              status.lowCurrentActive = true;
              status.lowCurrentStartTime = millis();
              Serial.println("[CYCLE] Charge low current, starting 2 min timer");
            } else if (millis() - status.lowCurrentStartTime >= 120000) {
              Serial.printf("[CYCLE %d] Full (2 min low current) -> discharge\n", status.currentCycle);
              status.lowCurrentActive = false;
              status.lowCurrentStartTime = 0;
              deltaStop();
              delay(2000);
              deltaSetupDischarge(config.minVoltage, config.dischargeCurrent);
            }
          } else {
            if (status.lowCurrentActive) {
              Serial.println("[CYCLE] Charge current increased, timer reset");
            }
            status.lowCurrentActive = false;
            status.lowCurrentStartTime = 0;
          }
        }
      } else {
        if (status.voltage <= config.minVoltage + 0.02) {
          if (abs(status.current) < 4.0) {
            if (!status.lowCurrentActive) {
              status.lowCurrentActive = true;
              status.lowCurrentStartTime = millis();
              Serial.println("[CYCLE] Discharge low current, starting 2 min timer");
            } else if (millis() - status.lowCurrentStartTime >= 120000) {
              Serial.printf("[CYCLE %d] Empty (2 min low current): %.3f Ah\n", status.currentCycle, status.cycleAhDischarge);
              playTone(1000, 200);
              status.lowCurrentActive = false;
              status.lowCurrentStartTime = 0;

              // Save this cycle to history
              saveCycleToHistory(false);  // false = discharge phase complete

              if (status.currentCycle >= config.numCycles) {
                beepDone();
                stopTest();
              } else {
                status.currentCycle++;
                status.totalAhDischarge = 0;
                // Start fresh tracking for new cycle
                startNewCyclePhase();
                deltaStop();
                delay(2000);
                deltaSetupCharge(config.maxVoltage, config.chargeCurrent);
              }
            }
          } else {
            if (status.lowCurrentActive) {
              Serial.println("[CYCLE] Discharge current increased, timer reset");
            }
            status.lowCurrentActive = false;
            status.lowCurrentStartTime = 0;
          }
        }
      }
      break;

    default:
      break;
  }
}

// ============== WEB SERVER ==============

// Color gradient for thermal image (blue -> green -> yellow -> red)
String getThermalColor(float temp, float minT, float maxT) {
  if (maxT <= minT) maxT = minT + 1;
  float ratio = (temp - minT) / (maxT - minT);
  ratio = constrain(ratio, 0.0, 1.0);

  int r, g, b;
  if (ratio < 0.25) {
    float t = ratio / 0.25;
    r = 0; g = (int)(255 * t); b = 255;
  } else if (ratio < 0.5) {
    float t = (ratio - 0.25) / 0.25;
    r = 0; g = 255; b = (int)(255 * (1 - t));
  } else if (ratio < 0.75) {
    float t = (ratio - 0.5) / 0.25;
    r = (int)(255 * t); g = 255; b = 0;
  } else {
    float t = (ratio - 0.75) / 0.25;
    r = 255; g = (int)(255 * (1 - t)); b = 0;
  }

  char hex[8];
  sprintf(hex, "#%02X%02X%02X", r, g, b);
  return String(hex);
}

// Helper function to send HTML page with chunked transfer
// This prevents browser timeout by sending headers immediately
void sendChunkedPage(const String& content) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  // Send in chunks of 2KB to avoid memory issues
  const int chunkSize = 2048;
  int pos = 0;
  while (pos < content.length()) {
    int len = min(chunkSize, (int)(content.length() - pos));
    server.sendContent(content.substring(pos, pos + len));
    pos += len;
    yield();  // Allow other tasks between chunks
  }
  server.sendContent("");  // End chunked transfer
}

void sendMainPage() {
  yield();  // Allow other tasks before building page
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Battery Tester v7.5</title>";
  h += "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>";
  h += "<link href='https://fonts.googleapis.com/css2?family=DSEG7+Classic:wght@400;700&display=swap' rel='stylesheet'>";
  h += "<style>";
  h += "@font-face{font-family:'DSEG7';src:url('https://cdn.jsdelivr.net/npm/dseg@0.46.0/fonts/DSEG7-Classic/DSEG7Classic-Bold.woff2')format('woff2')}";
  h += "*{box-sizing:border-box;margin:0;padding:0}";
  h += "body{font-family:sans-serif;background:#1a1a2e;color:#eee;padding:10px}";
  h += "h1{text-align:center;color:#0cf;margin-bottom:10px}";
  h += ".main-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:10px}";
  h += ".full-width{grid-column:1/-1}";
  h += ".panel{background:#16213e;border-radius:6px;padding:12px;margin-bottom:10px}";
  h += ".panel h2{color:#0cf;font-size:0.9em;margin-bottom:8px}";
  h += ".graph-container{height:300px;position:relative}";
  h += ".thermal-container{text-align:center}";
  h += ".thermal-img{image-rendering:pixelated;border:2px solid #333;border-radius:4px;max-width:100%}";
  h += ".thermal-stats{display:flex;justify-content:space-around;margin-top:8px}";
  h += ".thermal-stat{text-align:center}.thermal-stat .label{font-size:0.7em;color:#888}";
  h += ".thermal-stat .val{font-size:1.1em;font-weight:bold}";
  h += ".tmin{color:#00f}.tmax{color:#f00}.tavg{color:#0f0}";
  // 7-segment display styles
  h += ".seg-display{display:flex;gap:20px;justify-content:center;margin-bottom:15px;flex-wrap:wrap}";
  h += ".seg-box{background:#0a0a15;border:2px solid #333;border-radius:8px;padding:15px 25px;text-align:center;min-width:180px}";
  h += ".seg-label{font-size:0.75em;color:#666;margin-bottom:5px;text-transform:uppercase}";
  h += ".seg-value{font-family:'DSEG7',monospace;font-size:3em;color:#0f0;text-shadow:0 0 10px #0f0}";
  h += ".seg-value.voltage{color:#0cf;text-shadow:0 0 10px #0cf}";
  h += ".seg-value.current{color:#f80;text-shadow:0 0 10px #f80}";
  h += ".seg-value.current.negative{color:#f44;text-shadow:0 0 10px #f44}";
  h += ".seg-unit{font-size:0.4em;color:#888;margin-left:5px}";
  // Small cards
  h += ".cards{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-bottom:10px}";
  h += ".card{background:#16213e;border-radius:6px;padding:8px;text-align:center}";
  h += ".card .l{font-size:0.65em;color:#888}.card .v{font-size:1.1em;font-weight:bold;color:#0cf}";
  h += ".controls{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}";
  h += ".row{display:flex;justify-content:space-between;font-size:0.85em;padding:3px 0}.row .k{color:#888}";
  h += "button{padding:8px 12px;border:none;border-radius:6px;font-weight:bold;cursor:pointer;margin:2px}";
  h += ".bg{background:#0a4;color:#fff}.br{background:#d33;color:#fff}.bb{background:#08c;color:#fff}.bo{background:#c80;color:#fff}.bp{background:#808;color:#fff}";
  h += "input[type=number]{width:60px;padding:6px;background:#0f0f23;border:1px solid #333;border-radius:4px;color:#fff;text-align:center}";
  h += ".st{text-align:center;padding:8px;border-radius:6px;margin-bottom:10px;font-weight:bold}";
  h += ".st.on{background:#0a4}.st.off{background:#555}.st.run{background:#c80}";
  h += ".err{background:#422;border:1px solid #d33;padding:8px;border-radius:5px;margin-bottom:10px}";
  h += ".mode-row{display:flex;align-items:center;gap:5px;margin-bottom:8px;flex-wrap:wrap}";
  h += ".mode-row label{color:#888;font-size:0.8em}";
  h += ".stop-btn{width:100%;padding:12px;font-size:1.1em}";
  h += ".log-info{font-size:0.8em;color:#888;margin-left:10px}";
  h += "@media(max-width:900px){.main-grid{grid-template-columns:1fr}.cards{grid-template-columns:repeat(2,1fr)}.controls{grid-template-columns:1fr}.seg-display{flex-direction:column;align-items:center}}";
  h += "</style></head><body>";

  h += "<h1>Battery Tester v7.5</h1>";
  h += "<div class='st' id='st'>Loading...</div>";
  h += "<div class='err' id='err' style='display:none'></div>";

  // Large 7-segment displays for Voltage, Current, Temp and Elapsed Time
  h += "<div class='seg-display'>";
  h += "<div class='seg-box'><div class='seg-label'>Voltage</div><div class='seg-value voltage' id='vBig'>--.---<span class='seg-unit'>V</span></div></div>";
  h += "<div class='seg-box'><div class='seg-label'>Current</div><div class='seg-value current' id='iBig'>--.--<span class='seg-unit'>A</span></div></div>";
  h += "<div class='seg-box'><div class='seg-label'>Temperature</div><div class='seg-value' id='tBig'>--.-<span class='seg-unit'>°C</span></div></div>";
  h += "<div class='seg-box' style='border-color:#c80'><div class='seg-label'>Elapsed Time</div><div class='seg-value' id='elapsed' style='color:#c80;text-shadow:0 0 10px #c80'>00:00:00</div></div>";
  h += "</div>";

  // Smaller status cards
  h += "<div class='cards'>";
  h += "<div class='card'><div class='l'>Ah Charge</div><div class='v' id='ahc'>--</div></div>";
  h += "<div class='card'><div class='l'>Ah Discharge</div><div class='v' id='ahd'>--</div></div>";
  h += "<div class='card'><div class='l'>Total Wh</div><div class='v' id='wh'>--</div></div>";
  h += "<div class='card'><div class='l'>Power</div><div class='v' id='pwr'>--</div></div>";
  h += "</div>";

  // Cycle Tracking Panel (visible during cycle mode)
  h += "<div class='panel' id='cyclePanel' style='background:#1a1a2a;border:2px solid #808;display:none'>";
  h += "<h2 style='color:#c8f'>Cycle Progress</h2>";
  h += "<div style='display:grid;grid-template-columns:repeat(5,1fr);gap:10px;text-align:center;margin-bottom:15px'>";
  h += "<div class='card' style='background:#0a0a1a'><div class='l'>Cycle</div><div class='v' id='cycleNum' style='font-size:1.8em;color:#c8f'>-/-</div></div>";
  h += "<div class='card' style='background:#0a0a1a'><div class='l'>Phase</div><div class='v' id='cyclePhase' style='color:#0f0'>--</div></div>";
  h += "<div class='card' style='background:#0a0a1a'><div class='l'>Cycle Time</div><div class='v' id='cycleTime'>--:--</div></div>";
  h += "<div class='card' style='background:#0a0a1a'><div class='l'>Cycle Ah+</div><div class='v' id='cycAhc' style='color:#0f0'>--</div></div>";
  h += "<div class='card' style='background:#0a0a1a'><div class='l'>Cycle Ah-</div><div class='v' id='cycAhd' style='color:#f44'>--</div></div>";
  h += "</div>";
  // Cycle History Table
  h += "<div id='cycleHistoryDiv' style='display:none'>";
  h += "<h3 style='color:#888;font-size:0.9em;margin-bottom:8px'>Completed Cycles</h3>";
  h += "<table style='width:100%;border-collapse:collapse;font-size:0.85em'>";
  h += "<thead><tr style='color:#888;border-bottom:1px solid #333'>";
  h += "<th style='padding:5px'>Cycle</th><th>Ah Charge</th><th>Ah Discharge</th><th>Duration</th><th>Peak Temp</th>";
  h += "</tr></thead>";
  h += "<tbody id='cycleHistoryBody'></tbody>";
  h += "</table></div>";
  h += "</div>";

  // Delta PSU Status Panel - CV/CC, Set vs Measured, Cable Loss
  h += "<div class='panel' style='background:#1a2a1a'>";
  h += "<h2 style='color:#4f8'>Delta PSU Status</h2>";
  h += "<div style='display:grid;grid-template-columns:repeat(4,1fr);gap:10px;text-align:center'>";
  // CV/CC Mode indicator
  h += "<div class='card' style='background:#0a1a0a'><div class='l'>Mode</div>";
  h += "<div class='v' id='cvcc' style='font-size:1.5em;font-weight:bold'>--</div></div>";
  // Set Voltage
  h += "<div class='card' style='background:#0a1a0a'><div class='l'>Set Voltage</div>";
  h += "<div class='v' id='setV'>--</div></div>";
  // Set Current
  h += "<div class='card' style='background:#0a1a0a'><div class='l'>Set Current</div>";
  h += "<div class='v' id='setI'>--</div></div>";
  // Cable Loss
  h += "<div class='card' style='background:#0a1a0a'><div class='l'>Cable Loss</div>";
  h += "<div class='v' id='cableLoss' style='color:#f80'>--</div></div>";
  h += "</div>";
  // Comparison bar
  h += "<div style='margin-top:10px;padding:10px;background:#0a1a0a;border-radius:6px'>";
  h += "<div style='display:flex;justify-content:space-between;font-size:0.85em;margin-bottom:5px'>";
  h += "<span style='color:#888'>Measured vs Set</span>";
  h += "<span id='compText' style='color:#4f8'>--</span>";
  h += "</div>";
  h += "<div style='display:flex;gap:10px'>";
  h += "<div style='flex:1'><div style='color:#666;font-size:0.7em'>VOLTAGE</div>";
  h += "<div style='display:flex;align-items:center;gap:5px'>";
  h += "<span style='color:#0cf' id='measV'>--.---</span>";
  h += "<span style='color:#666'>vs</span>";
  h += "<span style='color:#4a8' id='progV'>--.--</span>";
  h += "<span style='color:#f80;font-size:0.8em' id='vDiff'></span>";
  h += "</div></div>";
  h += "<div style='flex:1'><div style='color:#666;font-size:0.7em'>CURRENT</div>";
  h += "<div style='display:flex;align-items:center;gap:5px'>";
  h += "<span style='color:#f80' id='measI'>--.--</span>";
  h += "<span style='color:#666'>vs</span>";
  h += "<span style='color:#4a8' id='progI'>--.--</span>";
  h += "<span style='color:#f80;font-size:0.8em' id='iDiff'></span>";
  h += "</div></div>";
  h += "</div></div>";
  h += "</div>";

  // Main grid: Graph left, Thermal right
  h += "<div class='main-grid'>";

  // Left: Graph
  h += "<div class='panel'><h2>Real-time Data</h2>";
  h += "<div class='graph-container'><canvas id='chart'></canvas></div></div>";

  // Right: Thermal
  h += "<div class='panel'><h2>Thermal Camera</h2>";
  h += "<div id='mlxStatus'>Checking...</div>";
  h += "<div class='thermal-container' id='thermalContainer' style='display:none'>";
  h += "<canvas id='thermal' class='thermal-img' width='640' height='480' style='width:100%;max-width:640px;height:auto'></canvas>";
  h += "<div class='thermal-stats'>";
  h += "<div class='thermal-stat'><div class='label'>MIN</div><div class='val tmin' id='tmin'>--</div></div>";
  h += "<div class='thermal-stat'><div class='label'>MAX</div><div class='val tmax' id='tmax'>--</div></div>";
  h += "<div class='thermal-stat'><div class='label'>AVG</div><div class='val tavg' id='tavg'>--</div></div>";
  h += "</div></div></div>";

  h += "</div>"; // end main-grid

  // Controls row
  h += "<div class='controls'>";

  // Charge
  h += "<div class='panel'><h2>Charge</h2>";
  h += "<div class='mode-row'><label>V:</label><input type='number' id='chgV' step='0.01' value='4.20'>";
  h += "<label>A:</label><input type='number' id='chgI' step='0.1' value='5'></div>";
  h += "<button class='bg' onclick='startChg()'>START</button></div>";

  // Discharge
  h += "<div class='panel'><h2>Discharge</h2>";
  h += "<div class='mode-row'><label>V:</label><input type='number' id='disV' step='0.01' value='2.71'>";
  h += "<label>A:</label><input type='number' id='disI' step='0.1' value='5'></div>";
  h += "<button class='bo' onclick='startDis()'>START</button></div>";

  // Cycle
  h += "<div class='panel'><h2>Auto Cycle</h2>";
  h += "<div class='row'><span class='k'>Range</span><span>" + String(config.minVoltage,2) + "-" + String(config.maxVoltage,2) + "V</span></div>";
  h += "<div class='row'><span class='k'>Cycle</span><span id='cyc'>0/" + String(config.numCycles) + "</span></div>";
  h += "<button class='bg' onclick='startCyc()'>START</button></div>";

  h += "</div>"; // end controls

  // Bottom row
  h += "<div class='panel' style='display:flex;gap:10px;align-items:center;flex-wrap:wrap'>";
  h += "<button class='br stop-btn' style='flex:1;min-width:150px' onclick='stop()'>STOP</button>";
  h += "<button class='bb' onclick='ping()'>PING DELTA</button><span id='ds'>-</span>";
  h += "<button onclick='location.href=\"/settings\"'>Settings</button>";
  h += "<button style='background:#f80;color:#fff' onclick='location.href=\"/sgs\"'>Anorgion Test</button>";
  h += "<button style='background:#0a0;color:#fff' onclick='location.href=\"/whtest\"'>Wh Test (12A)</button>";
  h += "<button onclick='clearChart()'>Clear Graph</button>";
  h += "</div>";

  // Data logging row
  h += "<div class='panel' style='display:flex;gap:10px;align-items:center;flex-wrap:wrap'>";
  h += "<b style='color:#0cf'>Data Log:</b>";
  h += "<button class='bp' onclick='downloadCSV()'>Download CSV</button>";
  h += "<button class='br' onclick='deleteLog()'>Delete Log</button>";
  h += "<span class='log-info' id='logInfo'>-</span>";
  h += "</div>";

  // JavaScript
  h += "<script>";
  h += "function $(i){return document.getElementById(i)}";

  // Chart setup
  h += "var maxPoints=100,labels=[],vData=[],iData=[],tData=[];";
  h += "var ctx=$('chart').getContext('2d');";
  h += "var chart=new Chart(ctx,{type:'line',data:{labels:labels,datasets:[";
  h += "{label:'Voltage (V)',data:vData,borderColor:'#0cf',backgroundColor:'rgba(0,204,255,0.1)',yAxisID:'y',tension:0.3},";
  h += "{label:'Current (A)',data:iData,borderColor:'#f80',backgroundColor:'rgba(255,136,0,0.1)',yAxisID:'y1',tension:0.3},";
  h += "{label:'Temp (C)',data:tData,borderColor:'#f44',backgroundColor:'rgba(255,68,68,0.1)',yAxisID:'y2',tension:0.3}";
  h += "]},options:{responsive:true,maintainAspectRatio:false,animation:{duration:0},interaction:{intersect:false,mode:'index'},";
  h += "scales:{x:{display:true,grid:{color:'#333'}},";
  h += "y:{type:'linear',position:'left',title:{display:true,text:'Voltage',color:'#0cf'},grid:{color:'#333'},ticks:{color:'#0cf'}},";
  h += "y1:{type:'linear',position:'right',title:{display:true,text:'Current',color:'#f80'},grid:{drawOnChartArea:false},ticks:{color:'#f80'}},";
  h += "y2:{type:'linear',position:'right',title:{display:true,text:'Temp',color:'#f44'},grid:{drawOnChartArea:false},ticks:{color:'#f44'}}";
  h += "},plugins:{legend:{labels:{color:'#fff'}}}}});";

  h += "function addData(v,i,t){var now=new Date().toLocaleTimeString();";
  h += "labels.push(now);vData.push(v);iData.push(i);tData.push(t);";
  h += "if(labels.length>maxPoints){labels.shift();vData.shift();iData.shift();tData.shift();}";
  h += "chart.update();}";
  h += "function clearChart(){labels.length=0;vData.length=0;iData.length=0;tData.length=0;chart.update();}";

  // Thermal canvas
  h += "var tCanvas=$('thermal'),tCtx=tCanvas.getContext('2d');";

  // Helper function to format seconds as HH:MM:SS
  h += "function fmtTime(s){var h=Math.floor(s/3600);var m=Math.floor((s%3600)/60);var sec=s%60;";
  h += "return (h<10?'0':'')+h+':'+(m<10?'0':'')+m+':'+(sec<10?'0':'')+sec;}";

  // Update function
  h += "function upd(){fetch('/status').then(r=>r.json()).then(d=>{";
  // 7-segment displays
  h += "$('vBig').innerHTML=d.v.toFixed(3)+'<span class=\"seg-unit\">V</span>';";
  h += "var iEl=$('iBig');iEl.innerHTML=d.i.toFixed(2)+'<span class=\"seg-unit\">A</span>';";
  h += "iEl.className='seg-value current'+(d.i<0?' negative':'');";
  h += "$('tBig').innerHTML=d.t.toFixed(1)+'<span class=\"seg-unit\">°C</span>';";
  // Elapsed time display
  h += "$('elapsed').innerText=fmtTime(d.elapsed||0);";
  // Cards
  h += "$('ahc').innerText=d.ahc.toFixed(3);";
  h += "$('ahd').innerText=d.ahd.toFixed(3);";
  h += "$('wh').innerText=d.wh.toFixed(2);";
  h += "$('pwr').innerText=d.p.toFixed(1)+'W';";
  h += "$('cyc').innerText=d.cyc+'/'+d.ncyc;";
  // Cycle panel - show only during cycle mode (mode 3)
  h += "var cp=$('cyclePanel');";
  h += "if(d.mode==3){cp.style.display='block';";
  h += "$('cycleNum').innerText=d.cyc+'/'+d.ncyc;";
  h += "var phase=d.i>=0?'CHARGING':'DISCHARGING';";
  h += "$('cyclePhase').innerText=phase;";
  h += "$('cyclePhase').style.color=d.i>=0?'#0f0':'#f44';";
  h += "$('cycleTime').innerText=fmtTime(d.cycleElapsed||0);";
  h += "$('cycAhc').innerText=(d.cycAhc||0).toFixed(3);";
  h += "$('cycAhd').innerText=(d.cycAhd||0).toFixed(3);";
  // Update cycle history table
  h += "var hist=d.cycleHistory||[];";
  h += "var histDiv=$('cycleHistoryDiv');";
  h += "if(hist.length>0){histDiv.style.display='block';";
  h += "var tb=$('cycleHistoryBody');tb.innerHTML='';";
  h += "for(var i=0;i<hist.length;i++){var c=hist[i];";
  h += "var row='<tr style=\"color:#aaa;border-bottom:1px solid #222\">';";
  h += "row+='<td style=\"padding:5px;color:#c8f\">#'+c.n+'</td>';";
  h += "row+='<td style=\"color:#0f0\">'+c.ahc.toFixed(3)+'</td>';";
  h += "row+='<td style=\"color:#f44\">'+c.ahd.toFixed(3)+'</td>';";
  h += "row+='<td>'+fmtTime(c.dur)+'</td>';";
  h += "row+='<td>'+c.pt.toFixed(1)+'°C</td></tr>';";
  h += "tb.innerHTML+=row;}}else{histDiv.style.display='none';}";
  h += "}else{cp.style.display='none';}";
  h += "$('ds').innerText=d.delta?'Connected':'Offline';";
  h += "$('ds').style.color=d.delta?'#0f0':'#f44';";
  h += "var modes=['IDLE','CHARGING','DISCHARGING','CYCLING'];";
  h += "var s=$('st');";
  h += "if(d.run){s.innerText=modes[d.mode];s.className='st run';}";
  h += "else{s.innerText=d.delta?'Ready':'Offline';s.className='st '+(d.delta?'on':'off');}";
  h += "var e=$('err');if(d.err&&d.err.length>0){e.innerText=d.err;e.style.display='block';}else{e.style.display='none';}";
  // Delta PSU extended info
  h += "var cvcc=$('cvcc');";
  h += "if(d.delta&&d.run){";
  h += "cvcc.innerText=d.cvMode?'CV':'CC';";
  h += "cvcc.style.color=d.cvMode?'#0cf':'#f80';";
  h += "}else{cvcc.innerText='--';cvcc.style.color='#666';}";
  h += "$('setV').innerText=d.setV.toFixed(2)+'V';";
  h += "var activeI=d.i>=0?d.setI:d.setINeg;";
  h += "$('setI').innerText=activeI.toFixed(1)+'A';";
  h += "var loss=$('cableLoss');";
  h += "if(Math.abs(d.cableLoss)>0.001){";
  h += "loss.innerText=(d.cableLoss>0?'+':'')+d.cableLoss.toFixed(3)+'V';";
  h += "loss.style.color=Math.abs(d.cableLoss)>0.1?'#f44':'#f80';";
  h += "}else{loss.innerText='0.000V';loss.style.color='#4f8';}";
  // Measured vs Set comparison
  h += "$('measV').innerText=d.v.toFixed(3);";
  h += "$('progV').innerText=d.setV.toFixed(2);";
  h += "$('measI').innerText=d.i.toFixed(2);";
  h += "$('progI').innerText=activeI.toFixed(2);";
  h += "var vD=d.v-d.setV;";
  h += "$('vDiff').innerText=(vD>=0?'+':'')+vD.toFixed(3)+'V';";
  h += "var iD=Math.abs(d.i)-activeI;";
  h += "$('iDiff').innerText=(iD>=0?'+':'')+iD.toFixed(2)+'A';";
  // Comparison text
  h += "var ct=$('compText');";
  h += "if(!d.delta){ct.innerText='PSU Offline';ct.style.color='#f44';}";
  h += "else if(!d.run){ct.innerText='Standby';ct.style.color='#888';}";
  h += "else if(d.cvMode){ct.innerText='Constant Voltage Mode';ct.style.color='#0cf';}";
  h += "else{ct.innerText='Constant Current Mode';ct.style.color='#f80';}";
  // Chart data
  h += "addData(d.v,d.i,d.t);";
  // Thermal camera
  h += "if(d.mlx){";
  h += "$('mlxStatus').style.display='none';$('thermalContainer').style.display='block';";
  h += "$('tmin').innerText=d.mlxMin.toFixed(1)+'C';";
  h += "$('tmax').innerText=d.mlxMax.toFixed(1)+'C';";
  h += "$('tavg').innerText=d.mlxAvg.toFixed(1)+'C';";
  h += "}else{$('mlxStatus').innerText='MLX90640 not connected';$('thermalContainer').style.display='none';}";
  h += "}).catch(()=>{$('st').innerText='ESP Offline';$('st').className='st off';})}";

  // Log info update
  h += "function updLog(){fetch('/loginfo').then(r=>r.json()).then(d=>{";
  h += "var kb=(d.logSize/1024).toFixed(1);var free=(d.freeSpace/1024).toFixed(0);";
  h += "$('logInfo').innerText='Size: '+kb+'KB | Free: '+free+'KB'+(d.logging?' | Recording...':'');";
  h += "}).catch(()=>{})}";

  // Thermal update
  h += "function updThermal(){var tc=$('thermalContainer');if(!tc||tc.style.display=='none')return;";
  h += "fetch('/thermaldata').then(r=>r.json()).then(d=>{";
  h += "if(!d.data)return;var w=32,h=24,scale=10;";
  h += "for(var y=0;y<h;y++){for(var x=0;x<w;x++){";
  h += "tCtx.fillStyle=d.colors[y*w+x];tCtx.fillRect(x*scale,y*scale,scale,scale);}}";
  h += "}).catch(()=>{})}";

  // Safety limits (loaded from server)
  h += "var safety={absMinV:2.5,absMaxV:4.25,absMaxCI:30,absMaxDI:30,warnHiI:10,warnHiV:4.22,warnLoV:2.6};";
  h += "fetch('/safety').then(r=>r.json()).then(d=>{safety=d;}).catch(()=>{});";

  // Validation function
  h += "function validateParams(v,i,isCharge){";
  h += "var errs=[];";
  h += "if(v>safety.absMaxV)errs.push('Voltage '+v+'V exceeds max '+safety.absMaxV+'V!');";
  h += "if(v<safety.absMinV)errs.push('Voltage '+v+'V below min '+safety.absMinV+'V!');";
  h += "if(isCharge&&i>safety.absMaxCI)errs.push('Current '+i+'A exceeds max '+safety.absMaxCI+'A!');";
  h += "if(!isCharge&&i>safety.absMaxDI)errs.push('Current '+i+'A exceeds max '+safety.absMaxDI+'A!');";
  h += "return errs;}";

  // Warning function
  h += "function getWarnings(v,i,isCharge){";
  h += "var w=[];";
  h += "if(i>safety.warnHiI)w.push('HIGH CURRENT: '+i+'A');";
  h += "if(isCharge&&v>safety.warnHiV)w.push('HIGH VOLTAGE: '+v+'V');";
  h += "if(!isCharge&&v<safety.warnLoV)w.push('LOW CUTOFF: '+v+'V');";
  h += "return w;}";

  // Confirmation dialog
  h += "function confirmStart(mode,v,i,isCharge){";
  h += "var errs=validateParams(v,i,isCharge);";
  h += "if(errs.length>0){alert('BLOCKED!\\n\\n'+errs.join('\\n'));return false;}";
  h += "var warns=getWarnings(v,i,isCharge);";
  h += "var msg=mode+'\\n\\nVoltage: '+v+'V\\nCurrent: '+i+'A';";
  h += "if(warns.length>0)msg+='\\n\\n⚠️ WARNINGS:\\n'+warns.join('\\n');";
  h += "msg+='\\n\\nAre you sure you want to start?';";
  h += "return confirm(msg);}";

  h += "function ping(){fetch('/ping').then(r=>r.json()).then(d=>{alert(d.result);upd();})}";

  // Charge with confirmation
  h += "function startChg(){";
  h += "var v=parseFloat($('chgV').value),i=parseFloat($('chgI').value);";
  h += "if(!confirmStart('START CHARGING',v,i,true))return;";
  h += "fetch('/charge?v='+v+'&i='+i).then(r=>r.json()).then(d=>{";
  h += "if(!d.ok)alert('ERROR: '+d.error);";
  h += "setTimeout(upd,500);}).catch(e=>alert('Error: '+e));}";

  // Discharge with confirmation
  h += "function startDis(){";
  h += "var v=parseFloat($('disV').value),i=parseFloat($('disI').value);";
  h += "if(!confirmStart('START DISCHARGING',v,i,false))return;";
  h += "fetch('/discharge?v='+v+'&i='+i).then(r=>r.json()).then(d=>{";
  h += "if(!d.ok)alert('ERROR: '+d.error);";
  h += "setTimeout(upd,500);}).catch(e=>alert('Error: '+e));}";

  // Cycle with confirmation
  h += "function startCyc(){";
  h += "var msg='START AUTO CYCLE\\n\\nRange: " + String(config.minVoltage,2) + "V - " + String(config.maxVoltage,2) + "V\\n";
  h += "Charge: " + String(config.chargeCurrent,1) + "A\\nDischarge: " + String(config.dischargeCurrent,1) + "A\\n";
  h += "Cycles: " + String(config.numCycles) + "\\n\\nAre you sure?';";
  h += "if(!confirm(msg))return;";
  h += "fetch('/cycle').then(r=>r.json()).then(d=>{";
  h += "if(!d.ok)alert('ERROR: '+d.error);";
  h += "setTimeout(upd,500);}).catch(e=>alert('Error: '+e));}";

  h += "function stop(){if(confirm('STOP the current test?')){fetch('/stop');setTimeout(upd,200)}}";
  h += "function downloadCSV(){window.location.href='/download'}";
  h += "function deleteLog(){if(confirm('Delete all logged data?')){fetch('/deletelog').then(()=>updLog())}}";
  h += "setInterval(upd,2000);setInterval(updThermal,1000);setInterval(updLog,5000);upd();updLog();setTimeout(updThermal,500);";
  h += "</script></body></html>";

  sendChunkedPage(h);
}

void sendThermalPage() {
  yield();  // Allow other tasks before building page
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Thermal Camera</title><style>";
  h += "*{margin:0;padding:0}body{background:#000;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:100vh;font-family:sans-serif;color:#fff}";
  h += "canvas{image-rendering:pixelated;max-width:100%;border:2px solid #333}";
  h += ".stats{display:flex;gap:30px;margin:20px;font-size:1.5em}";
  h += ".stat{text-align:center}.label{font-size:0.6em;color:#888}";
  h += ".tmin{color:#00f}.tmax{color:#f00}.tavg{color:#0f0}";
  h += ".back{position:fixed;top:10px;left:10px;padding:10px 20px;background:#333;color:#fff;text-decoration:none;border-radius:5px}";
  h += "</style></head><body>";
  h += "<a href='/' class='back'>Back</a>";
  h += "<h1 style='margin:20px;color:#0cf'>MLX90640 Thermal Camera</h1>";
  h += "<canvas id='thermal' width='640' height='480'></canvas>";
  h += "<div class='stats'>";
  h += "<div class='stat'><div class='label'>MIN</div><div class='tmin' id='tmin'>--</div></div>";
  h += "<div class='stat'><div class='label'>MAX</div><div class='tmax' id='tmax'>--</div></div>";
  h += "<div class='stat'><div class='label'>AVG</div><div class='tavg' id='tavg'>--</div></div>";
  h += "</div>";
  h += "<script>";
  h += "var canvas=document.getElementById('thermal'),ctx=canvas.getContext('2d');";
  h += "function upd(){fetch('/thermaldata').then(r=>r.json()).then(d=>{";
  h += "if(!d.data)return;var w=32,h=24,scale=20;";
  h += "for(var y=0;y<h;y++){for(var x=0;x<w;x++){";
  h += "ctx.fillStyle=d.colors[y*w+x];ctx.fillRect(x*scale,y*scale,scale,scale);}}";
  h += "document.getElementById('tmin').innerText=d.min.toFixed(1)+'C';";
  h += "document.getElementById('tmax').innerText=d.max.toFixed(1)+'C';";
  h += "document.getElementById('tavg').innerText=d.avg.toFixed(1)+'C';";
  h += "}).catch(()=>{})}";
  h += "setInterval(upd,500);upd();";
  h += "</script></body></html>";

  sendChunkedPage(h);
}

void sendSettingsPage() {
  yield();  // Allow other tasks before building page
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Settings</title><style>";
  h += "*{box-sizing:border-box;margin:0;padding:0}";
  h += "body{font-family:sans-serif;background:#1a1a2e;color:#eee;padding:10px;max-width:600px;margin:0 auto}";
  h += "h1{text-align:center;color:#0cf;margin-bottom:10px}";
  h += ".panel{background:#16213e;border-radius:6px;padding:15px;margin-bottom:10px}";
  h += ".panel h2{color:#0cf;font-size:0.95em;margin-bottom:10px}";
  h += ".panel.safety{background:#2a1a1a;border:2px solid #d33}";
  h += ".panel.safety h2{color:#f44}";
  h += ".form-row{margin-bottom:15px}";
  h += ".form-row label{display:block;color:#888;margin-bottom:5px}";
  h += ".form-row input{width:100%;padding:12px;background:#0f0f23;border:1px solid #333;border-radius:4px;color:#fff}";
  h += ".form-row input:disabled{background:#1a1a1a;color:#666}";
  h += ".form-row .hint{font-size:0.75em;color:#666;margin-top:3px}";
  h += ".two-col{display:grid;grid-template-columns:1fr 1fr;gap:10px}";
  h += "button{width:100%;padding:12px;border:none;border-radius:6px;font-weight:bold;cursor:pointer;margin-top:10px}";
  h += ".btn-save{background:#0a4;color:#fff}.btn-back{background:#666;color:#fff}";
  h += ".btn-unlock{background:#c80;color:#fff}.btn-danger{background:#d33;color:#fff}";
  h += ".limits-info{background:#0a0a15;padding:10px;border-radius:4px;margin-bottom:15px;font-size:0.85em}";
  h += ".limits-info span{color:#0cf}";
  h += ".locked{opacity:0.5;pointer-events:none}";
  h += "</style></head><body>";
  h += "<h1>Settings</h1>";

  // Show current safety limits
  h += "<div class='panel'><h2>Current Safety Limits</h2>";
  h += "<div class='limits-info'>";
  h += "Voltage: <span>" + String(safetyLimits.absMinVoltage,2) + "V - " + String(safetyLimits.absMaxVoltage,2) + "V</span><br>";
  h += "Max Charge Current: <span>" + String(safetyLimits.absMaxChargeCurrent,1) + "A</span><br>";
  h += "Max Discharge Current: <span>" + String(safetyLimits.absMaxDischargeCurrent,1) + "A</span><br>";
  h += "Warning at: <span>>" + String(safetyLimits.warnHighCurrent,1) + "A</span>";
  h += "</div></div>";

  h += "<div class='panel'><h2>Voltage (within limits)</h2>";
  h += "<div class='two-col'>";
  h += "<div class='form-row'><label>Min V (discharge cutoff)</label><input type='number' id='minV' step='0.01' min='" + String(safetyLimits.absMinVoltage,2) + "' max='" + String(safetyLimits.absMaxVoltage,2) + "' value='" + String(config.minVoltage,2) + "'></div>";
  h += "<div class='form-row'><label>Max V (charge target)</label><input type='number' id='maxV' step='0.01' min='" + String(safetyLimits.absMinVoltage,2) + "' max='" + String(safetyLimits.absMaxVoltage,2) + "' value='" + String(config.maxVoltage,2) + "'></div>";
  h += "</div></div>";

  h += "<div class='panel'><h2>Current (within limits)</h2>";
  h += "<div class='two-col'>";
  h += "<div class='form-row'><label>Charge A (max " + String(safetyLimits.absMaxChargeCurrent,1) + ")</label><input type='number' id='chgI' step='0.1' min='0.1' max='" + String(safetyLimits.absMaxChargeCurrent,1) + "' value='" + String(config.chargeCurrent,1) + "'></div>";
  h += "<div class='form-row'><label>Discharge A (max " + String(safetyLimits.absMaxDischargeCurrent,1) + ")</label><input type='number' id='disI' step='0.1' min='0.1' max='" + String(safetyLimits.absMaxDischargeCurrent,1) + "' value='" + String(config.dischargeCurrent,1) + "'></div>";
  h += "</div></div>";

  h += "<div class='panel'><h2>Temperature Alarms</h2>";
  h += "<div class='two-col'>";
  h += "<div class='form-row'><label>DS18B20 Alarm (C)</label><input type='number' id='temp' value='" + String(config.tempAlarm,0) + "'></div>";
  h += "<div class='form-row'><label>Thermal Camera Alarm (C)</label><input type='number' id='therm' value='" + String(config.thermalAlarm,0) + "'></div>";
  h += "</div></div>";

  h += "<div class='panel'><h2>Test Settings</h2>";
  h += "<div class='two-col'>";
  h += "<div class='form-row'><label>Cycles</label><input type='number' id='cyc' min='1' max='100' value='" + String(config.numCycles) + "'></div>";
  h += "<div class='form-row'><label>Log Interval (s)</label><input type='number' id='log' min='1' max='300' value='" + String(config.logInterval) + "'></div>";
  h += "</div></div>";

  h += "<div class='panel'>";
  h += "<button class='btn-save' onclick='save()'>SAVE SETTINGS</button>";
  h += "<button class='btn-back' onclick='location.href=\"/\"'>BACK TO MAIN</button>";
  h += "</div>";

  // SAFETY LIMITS SECTION - Requires code to unlock
  h += "<div class='panel safety'><h2>⚠️ SAFETY LIMITS (Protected)</h2>";
  h += "<p style='color:#f88;margin-bottom:15px;font-size:0.85em'>Changing these limits can damage batteries or cause fire. Requires safety code.</p>";

  h += "<div class='form-row'><label>Safety Code</label><input type='password' id='safetyCode' placeholder='Enter code to unlock'></div>";
  h += "<button class='btn-unlock' onclick='unlockSafety()'>UNLOCK SAFETY SETTINGS</button>";

  h += "<div id='safetyFields' class='locked'>";
  h += "<div class='two-col' style='margin-top:15px'>";
  h += "<div class='form-row'><label>Absolute Min Voltage</label><input type='number' id='absMinV' step='0.01' value='" + String(safetyLimits.absMinVoltage,2) + "'><div class='hint'>Hard floor - cannot go below</div></div>";
  h += "<div class='form-row'><label>Absolute Max Voltage</label><input type='number' id='absMaxV' step='0.01' value='" + String(safetyLimits.absMaxVoltage,2) + "'><div class='hint'>Hard ceiling - cannot exceed</div></div>";
  h += "</div>";
  h += "<div class='two-col'>";
  h += "<div class='form-row'><label>Max Charge Current (A)</label><input type='number' id='absMaxCI' step='0.1' value='" + String(safetyLimits.absMaxChargeCurrent,1) + "'></div>";
  h += "<div class='form-row'><label>Max Discharge Current (A)</label><input type='number' id='absMaxDI' step='0.1' value='" + String(safetyLimits.absMaxDischargeCurrent,1) + "'></div>";
  h += "</div>";
  h += "<div class='two-col'>";
  h += "<div class='form-row'><label>Warn High Current (A)</label><input type='number' id='warnHiI' step='0.1' value='" + String(safetyLimits.warnHighCurrent,1) + "'><div class='hint'>Show warning above this</div></div>";
  h += "<div class='form-row'><label>Warn High Voltage (V)</label><input type='number' id='warnHiV' step='0.01' value='" + String(safetyLimits.warnHighVoltage,2) + "'></div>";
  h += "</div>";
  h += "<div class='form-row'><label>New Safety Code (min 4 chars)</label><input type='password' id='newCode' placeholder='Leave empty to keep current'></div>";
  h += "<button class='btn-danger' onclick='saveSafety()'>SAVE SAFETY LIMITS</button>";
  h += "</div></div>";

  // Safety Log
  h += "<div class='panel'><h2>Safety Log</h2>";
  h += "<button onclick='viewLog()'>View Safety Log</button>";
  h += "<pre id='logPre' style='margin-top:10px;font-size:0.75em;max-height:200px;overflow:auto;background:#0a0a15;padding:10px;display:none'></pre>";
  h += "</div>";

  h += "<script>";
  h += "function $(i){return document.getElementById(i)}";

  // Validate before save
  h += "function save(){";
  h += "var minV=parseFloat($('minV').value),maxV=parseFloat($('maxV').value);";
  h += "var chgI=parseFloat($('chgI').value),disI=parseFloat($('disI').value);";
  // Check within safety limits
  h += "if(minV<" + String(safetyLimits.absMinVoltage,2) + "||maxV>" + String(safetyLimits.absMaxVoltage,2) + "){";
  h += "alert('Voltage outside safety limits!');return;}";
  h += "if(chgI>" + String(safetyLimits.absMaxChargeCurrent,1) + "||disI>" + String(safetyLimits.absMaxDischargeCurrent,1) + "){";
  h += "alert('Current exceeds safety limits!');return;}";
  h += "if(minV>=maxV){alert('Min voltage must be less than max!');return;}";
  // Confirm if high values
  h += "var warns=[];";
  h += "if(chgI>" + String(safetyLimits.warnHighCurrent,1) + ")warns.push('High charge current: '+chgI+'A');";
  h += "if(disI>" + String(safetyLimits.warnHighCurrent,1) + ")warns.push('High discharge current: '+disI+'A');";
  h += "if(warns.length>0&&!confirm('WARNING:\\n'+warns.join('\\n')+'\\n\\nSave anyway?'))return;";
  h += "var p='minV='+minV+'&maxV='+maxV+'&chgI='+chgI+'&disI='+disI+'&cyc='+$('cyc').value+'&log='+$('log').value+'&temp='+$('temp').value+'&therm='+$('therm').value;";
  h += "fetch('/save?'+p).then(r=>r.json()).then(d=>{if(d.ok)location.href='/';else alert('Error: '+d.error);}).catch(e=>alert('Error: '+e));}";

  // Unlock safety section
  h += "function unlockSafety(){";
  h += "var code=$('safetyCode').value;";
  h += "if(code.length<4){alert('Enter safety code');return;}";
  h += "$('safetyFields').classList.remove('locked');}";

  // Save safety limits
  h += "function saveSafety(){";
  h += "var code=$('safetyCode').value;";
  h += "if(!confirm('⚠️ WARNING!\\n\\nYou are about to change SAFETY LIMITS.\\n\\nIncorrect values can:\\n- Destroy batteries\\n- Cause fires\\n- Damage equipment\\n\\nAre you absolutely sure?'))return;";
  h += "var p='code='+encodeURIComponent(code);";
  h += "p+='&absMinV='+$('absMinV').value;";
  h += "p+='&absMaxV='+$('absMaxV').value;";
  h += "p+='&absMaxCI='+$('absMaxCI').value;";
  h += "p+='&absMaxDI='+$('absMaxDI').value;";
  h += "p+='&warnHiI='+$('warnHiI').value;";
  h += "p+='&warnHiV='+$('warnHiV').value;";
  h += "if($('newCode').value.length>=4)p+='&newCode='+encodeURIComponent($('newCode').value);";
  h += "fetch('/safety/update?'+p).then(r=>r.json()).then(d=>{";
  h += "if(d.ok){alert('Safety limits updated!');location.reload();}";
  h += "else alert('Error: '+d.error);}).catch(e=>alert('Error: '+e));}";

  // View log
  h += "function viewLog(){";
  h += "fetch('/safety/log').then(r=>r.text()).then(t=>{";
  h += "$('logPre').style.display='block';$('logPre').innerText=t;});}";

  h += "</script></body></html>";

  sendChunkedPage(h);
}

// ============== SGS TEST PAGE ==============
void sendSGSPage() {
  yield();  // Allow other tasks before building page
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Anorgion Test Mode</title>";
  h += "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>";
  h += "<style>";
  h += "*{box-sizing:border-box;margin:0;padding:0}";
  h += "body{font-family:sans-serif;background:#1a1a2e;color:#eee;padding:10px}";
  h += "h1{text-align:center;color:#f80;margin-bottom:10px}";
  h += "h2{color:#0cf;font-size:1em;margin-bottom:10px}";
  h += ".panel{background:#16213e;border-radius:6px;padding:15px;margin-bottom:10px}";
  h += ".panel.sgs{border:2px solid #f80}";
  h += ".grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}";
  h += ".grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}";
  h += ".grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}";
  h += ".form-row{margin-bottom:10px}";
  h += ".form-row label{display:block;color:#888;font-size:0.8em;margin-bottom:3px}";
  h += ".form-row input{width:100%;padding:8px;background:#0f0f23;border:1px solid #333;border-radius:4px;color:#fff}";
  h += ".card{background:#0a0a15;border-radius:6px;padding:10px;text-align:center}";
  h += ".card .l{font-size:0.7em;color:#888}.card .v{font-size:1.3em;font-weight:bold;color:#0cf}";
  h += "button{padding:10px 15px;border:none;border-radius:6px;font-weight:bold;cursor:pointer;margin:3px}";
  h += ".btn-start{background:#0a4;color:#fff;font-size:1.1em;width:100%}";
  h += ".btn-stop{background:#d33;color:#fff}.btn-back{background:#555;color:#fff}";
  h += "table{width:100%;border-collapse:collapse;font-size:0.8em;margin-top:10px}";
  h += "th,td{padding:6px;text-align:center;border-bottom:1px solid #333}";
  h += "th{background:#0a0a15;color:#0cf}";
  h += ".status-box{padding:15px;border-radius:6px;text-align:center;font-size:1.2em;font-weight:bold;margin-bottom:10px}";
  h += ".status-idle{background:#333;color:#888}";
  h += ".status-run{background:#c80;color:#fff}";
  h += ".status-done{background:#0a4;color:#fff}";
  h += ".ref-val{color:#f80;font-size:0.9em}";
  h += ".graph-container{height:250px;position:relative;margin-top:15px}";
  h += ".eff-good{color:#0f0}.eff-warn{color:#f80}.eff-bad{color:#f44}";
  h += "</style></head><body>";

  h += "<h1>Anorgion Test Mode</h1>";
  h += "<div style='text-align:center;margin-bottom:10px'>";
  h += "<button class='btn-back' onclick='location.href=\"/\"'>← Back to Main</button></div>";

  // Status box
  h += "<div class='status-box status-idle' id='sgsStatus'>IDLE - Configure and Start Test</div>";

  // Main grid
  h += "<div class='grid2'>";

  // Left column - Configuration
  h += "<div>";

  // Cell Specifications
  h += "<div class='panel'><h2>📦 Cell Specifications</h2>";
  h += "<div class='grid2'>";
  h += "<div class='form-row'><label>Cell Weight (g)</label><input type='number' id='cellWeight' step='0.1' value='" + String(sgsConfig.cellWeightKg * 1000, 1) + "'></div>";
  h += "<div class='form-row'><label>Cell Volume (ml)</label><input type='number' id='cellVolume' step='0.1' value='" + String(sgsConfig.cellVolumeLiter * 1000, 1) + "'></div>";
  h += "<div class='form-row'><label>Nominal Capacity (Ah)</label><input type='number' id='nomCap' step='0.1' value='" + String(sgsConfig.nominalCapacity, 1) + "'></div>";
  h += "<div class='form-row'><label>Nominal Voltage (V)</label><input type='number' id='nomVolt' step='0.01' value='" + String(sgsConfig.nominalVoltage, 2) + "'></div>";
  h += "</div></div>";

  // Anorgion Test Parameters
  h += "<div class='panel sgs'><h2>⚡ Anorgion Test Parameters</h2>";
  h += "<div class='grid2'>";
  h += "<div class='form-row'><label>Charge Voltage (V)</label><input type='number' id='chgV' step='0.01' value='" + String(sgsConfig.chargeVoltage, 2) + "'><div class='ref-val'>SGS: 4.15V</div></div>";
  h += "<div class='form-row'><label>Charge Current (A)</label><input type='number' id='chgI' step='0.1' value='" + String(sgsConfig.chargeCurrent, 1) + "'><div class='ref-val'>SGS: 24A (1C)</div></div>";
  h += "<div class='form-row'><label>Cutoff Current (A)</label><input type='number' id='cutI' step='0.1' value='" + String(sgsConfig.cutoffCurrent, 1) + "'><div class='ref-val'>SGS: 1.5A (C/16)</div></div>";
  h += "<div class='form-row'><label>Discharge Voltage (V)</label><input type='number' id='disV' step='0.01' value='" + String(sgsConfig.dischargeVoltage, 2) + "'><div class='ref-val'>SGS: 2.70V</div></div>";
  h += "<div class='form-row'><label>Discharge Current (A)</label><input type='number' id='disI' step='0.1' value='" + String(sgsConfig.dischargeCurrent, 1) + "'><div class='ref-val'>SGS: 24A (1C)</div></div>";
  h += "<div class='form-row'><label>Max Temp Stop (°C)</label><input type='number' id='maxT' value='" + String(sgsConfig.maxTempStop, 0) + "'><div class='ref-val'>SGS: 45°C</div></div>";
  h += "<div class='form-row'><label>Rest Time (sec)</label><input type='number' id='restT' value='" + String(sgsConfig.restTimeSeconds) + "'><div class='ref-val'>SGS: 300s (5min)</div></div>";
  h += "<div class='form-row'><label>Total Cycles</label><input type='number' id='numCyc' min='1' max='100' value='" + String(sgsConfig.totalCycles) + "'><div class='ref-val'>SGS: 100</div></div>";
  h += "</div>";
  h += "<button class='btn-start' id='startBtn' onclick='startSGS()'>▶ START SGS TEST</button>";
  h += "<button class='btn-stop' onclick='stopSGS()' style='width:100%;margin-top:5px'>■ STOP TEST</button>";
  h += "</div>";

  h += "</div>"; // end left column

  // Right column - Live data & results
  h += "<div>";

  // Live Measurements
  h += "<div class='panel'><h2>📊 Live Measurements</h2>";
  h += "<div class='grid4'>";
  h += "<div class='card'><div class='l'>Voltage</div><div class='v' id='liveV'>--</div></div>";
  h += "<div class='card'><div class='l'>Current</div><div class='v' id='liveI'>--</div></div>";
  h += "<div class='card'><div class='l'>Temperature</div><div class='v' id='liveT'>--</div></div>";
  h += "<div class='card'><div class='l'>Power</div><div class='v' id='liveP'>--</div></div>";
  h += "</div>";
  h += "<div class='grid4' style='margin-top:8px'>";
  h += "<div class='card'><div class='l'>Cycle</div><div class='v' id='liveCyc' style='color:#f80'>-/-</div></div>";
  h += "<div class='card'><div class='l'>Phase</div><div class='v' id='livePhase' style='font-size:1em'>--</div></div>";
  h += "<div class='card'><div class='l'>Elapsed</div><div class='v' id='liveElapsed' style='font-size:1em'>--:--:--</div></div>";
  h += "<div class='card'><div class='l'>CV/CC</div><div class='v' id='liveCVCC'>--</div></div>";
  h += "</div></div>";

  // Current Cycle Stats
  h += "<div class='panel'><h2>🔄 Current Cycle</h2>";
  h += "<div class='grid4'>";
  h += "<div class='card'><div class='l'>Ah Charge</div><div class='v' id='cycAhC' style='color:#0f0'>--</div></div>";
  h += "<div class='card'><div class='l'>Ah Discharge</div><div class='v' id='cycAhD' style='color:#f44'>--</div></div>";
  h += "<div class='card'><div class='l'>Wh Charge</div><div class='v' id='cycWhC' style='color:#0f0'>--</div></div>";
  h += "<div class='card'><div class='l'>Wh Discharge</div><div class='v' id='cycWhD' style='color:#f44'>--</div></div>";
  h += "</div></div>";

  // Calculated Values (Energy Density)
  h += "<div class='panel'><h2>📐 Energy Density (Live)</h2>";
  h += "<div class='grid4'>";
  h += "<div class='card'><div class='l'>Wh/kg (Grav)</div><div class='v' id='whKg'>--</div></div>";
  h += "<div class='card'><div class='l'>Wh/L (Vol)</div><div class='v' id='whL'>--</div></div>";
  h += "<div class='card'><div class='l'>Coulomb Eff</div><div class='v' id='effC'>--</div></div>";
  h += "<div class='card'><div class='l'>Energy Eff</div><div class='v' id='effE'>--</div></div>";
  h += "</div>";
  h += "<div style='font-size:0.8em;color:#888;margin-top:8px'>SGS Reference (Cycle 100): <span style='color:#f80'>267.96 Wh/kg | 697.46 Wh/L | CE: 100.02% | EE: 88.65%</span></div>";
  h += "</div>";

  h += "</div>"; // end right column
  h += "</div>"; // end main grid

  // Results Section
  h += "<div class='panel'><h2>📋 Cycle Results</h2>";

  // Graph
  h += "<div class='graph-container'><canvas id='sgsChart'></canvas></div>";

  // Results table
  h += "<div style='max-height:300px;overflow-y:auto;margin-top:15px'>";
  h += "<table id='resultsTable'>";
  h += "<thead><tr><th>Cycle</th><th>Ah Chg</th><th>Ah Dis</th><th>Wh Chg</th><th>Wh Dis</th><th>CE %</th><th>EE %</th><th>Wh/kg</th><th>Wh/L</th><th>Peak T</th></tr></thead>";
  h += "<tbody id='resultsBody'></tbody>";
  h += "</table></div>";

  // Summary stats
  h += "<div class='grid3' style='margin-top:15px'>";
  h += "<div class='card' style='background:#1a2a1a'><div class='l'>Avg Coulomb Efficiency</div><div class='v' id='avgCE'>--</div></div>";
  h += "<div class='card' style='background:#1a2a1a'><div class='l'>Avg Energy Efficiency</div><div class='v' id='avgEE'>--</div></div>";
  h += "<div class='card' style='background:#1a2a1a'><div class='l'>Capacity Retention</div><div class='v' id='capRet'>--</div></div>";
  h += "</div>";
  h += "</div>";

  // JavaScript
  h += "<script>";
  h += "function $(i){return document.getElementById(i)}";
  h += "function fmtTime(s){var h=Math.floor(s/3600);var m=Math.floor((s%3600)/60);var sec=s%60;return (h<10?'0':'')+h+':'+(m<10?'0':'')+m+':'+(sec<10?'0':'')+sec;}";

  // Chart setup
  h += "var ctx=$('sgsChart').getContext('2d');";
  h += "var sgsChart=new Chart(ctx,{type:'line',data:{labels:[],datasets:[";
  h += "{label:'Ah Discharge',data:[],borderColor:'#f44',backgroundColor:'rgba(255,68,68,0.1)',yAxisID:'y',tension:0.3},";
  h += "{label:'Coulomb Eff %',data:[],borderColor:'#0f0',backgroundColor:'rgba(0,255,0,0.1)',yAxisID:'y1',tension:0.3},";
  h += "{label:'Energy Eff %',data:[],borderColor:'#0cf',backgroundColor:'rgba(0,204,255,0.1)',yAxisID:'y1',tension:0.3}";
  h += "]},options:{responsive:true,maintainAspectRatio:false,animation:{duration:0},";
  h += "scales:{x:{grid:{color:'#333'}},y:{type:'linear',position:'left',title:{display:true,text:'Capacity (Ah)',color:'#f44'},grid:{color:'#333'},ticks:{color:'#f44'}},";
  h += "y1:{type:'linear',position:'right',min:80,max:105,title:{display:true,text:'Efficiency %',color:'#0f0'},grid:{drawOnChartArea:false},ticks:{color:'#0f0'}}},";
  h += "plugins:{legend:{labels:{color:'#fff'}}}}});";

  // Start SGS test
  h += "function startSGS(){";
  h += "var cfg={weight:parseFloat($('cellWeight').value)/1000,volume:parseFloat($('cellVolume').value)/1000,";
  h += "nomCap:parseFloat($('nomCap').value),nomVolt:parseFloat($('nomVolt').value),";
  h += "chgV:parseFloat($('chgV').value),chgI:parseFloat($('chgI').value),cutI:parseFloat($('cutI').value),";
  h += "disV:parseFloat($('disV').value),disI:parseFloat($('disI').value),maxT:parseFloat($('maxT').value),";
  h += "restT:parseInt($('restT').value),numCyc:parseInt($('numCyc').value)};";
  h += "if(!confirm('Start Anorgion Test?\\n\\nCycles: '+cfg.numCyc+'\\nCharge: '+cfg.chgV+'V / '+cfg.chgI+'A\\nDischarge: '+cfg.disV+'V / '+cfg.disI+'A\\nCutoff: '+cfg.cutI+'A'))return;";
  h += "var p=Object.keys(cfg).map(k=>k+'='+cfg[k]).join('&');";
  h += "fetch('/sgs/start?'+p).then(r=>r.json()).then(d=>{";
  h += "if(d.ok){$('sgsStatus').innerText='STARTING...';$('sgsStatus').className='status-box status-run';}";
  h += "else alert('Error: '+d.error);}).catch(e=>alert('Error: '+e));}";

  // Stop SGS test
  h += "function stopSGS(){if(confirm('Stop Anorgion Test?')){fetch('/sgs/stop').then(()=>{";
  h += "$('sgsStatus').innerText='STOPPED';$('sgsStatus').className='status-box status-idle';});}}";

  // Update function
  h += "function upd(){fetch('/sgs/status').then(r=>r.json()).then(d=>{";
  // Live measurements
  h += "$('liveV').innerText=d.v.toFixed(3)+'V';";
  h += "$('liveI').innerText=d.i.toFixed(2)+'A';";
  h += "$('liveI').style.color=d.i>=0?'#0f0':'#f44';";
  h += "$('liveT').innerText=d.t.toFixed(1)+'°C';";
  h += "$('liveP').innerText=d.p.toFixed(1)+'W';";
  h += "$('liveCyc').innerText=d.cyc+'/'+d.ncyc;";
  h += "$('liveElapsed').innerText=fmtTime(d.elapsed);";
  h += "$('liveCVCC').innerText=d.cvMode?'CV':'CC';";
  h += "$('liveCVCC').style.color=d.cvMode?'#0cf':'#f80';";

  // Phase
  h += "var phases=['IDLE','CHARGING','REST','DISCHARGING','REST'];";
  h += "$('livePhase').innerText=phases[d.phase]||'--';";
  h += "var phaseColors=['#888','#0f0','#888','#f44','#888'];";
  h += "$('livePhase').style.color=phaseColors[d.phase]||'#888';";

  // Status box
  h += "if(d.active){$('sgsStatus').innerText='RUNNING - Cycle '+d.cyc+'/'+d.ncyc+' - '+phases[d.phase];$('sgsStatus').className='status-box status-run';}";
  h += "else if(d.cyc>=d.ncyc&&d.cyc>0){$('sgsStatus').innerText='COMPLETE - '+d.cyc+' cycles finished';$('sgsStatus').className='status-box status-done';}";
  h += "else{$('sgsStatus').innerText='IDLE - Configure and Start Test';$('sgsStatus').className='status-box status-idle';}";

  // Current cycle data
  h += "$('cycAhC').innerText=d.cycAhC.toFixed(3);";
  h += "$('cycAhD').innerText=d.cycAhD.toFixed(3);";
  h += "$('cycWhC').innerText=d.cycWhC.toFixed(2);";
  h += "$('cycWhD').innerText=d.cycWhD.toFixed(2);";

  // Energy density calculations
  h += "if(d.cycWhD>0.1&&d.weight>0){var whkg=d.cycWhD/d.weight;$('whKg').innerText=whkg.toFixed(1);}else{$('whKg').innerText='--';}";
  h += "if(d.cycWhD>0.1&&d.volume>0){var whl=d.cycWhD/d.volume;$('whL').innerText=whl.toFixed(1);}else{$('whL').innerText='--';}";

  // Efficiency (current cycle)
  h += "if(d.cycAhC>0.1){var ce=(d.cycAhD/d.cycAhC)*100;$('effC').innerText=ce.toFixed(1)+'%';$('effC').className='v '+(ce>99?'eff-good':ce>95?'eff-warn':'eff-bad');}else{$('effC').innerText='--';}";
  h += "if(d.cycWhC>0.1){var ee=(d.cycWhD/d.cycWhC)*100;$('effE').innerText=ee.toFixed(1)+'%';$('effE').className='v '+(ee>85?'eff-good':ee>80?'eff-warn':'eff-bad');}else{$('effE').innerText='--';}";

  // Update results table and chart
  h += "var hist=d.history||[];";
  h += "if(hist.length>0){";
  h += "var tb=$('resultsBody');tb.innerHTML='';";
  h += "var labels=[],ahData=[],ceData=[],eeData=[];";
  h += "var sumCE=0,sumEE=0,firstAh=0,lastAh=0;";
  h += "for(var i=0;i<hist.length;i++){var c=hist[i];";
  h += "if(i==0)firstAh=c.ahd;lastAh=c.ahd;";
  h += "sumCE+=c.ce;sumEE+=c.ee;";
  h += "var whkg=(c.whd/d.weight).toFixed(1);var whl=(c.whd/d.volume).toFixed(1);";
  h += "var row='<tr><td style=\"color:#f80\">#'+(i+1)+'</td>';";
  h += "row+='<td style=\"color:#0f0\">'+c.ahc.toFixed(3)+'</td>';";
  h += "row+='<td style=\"color:#f44\">'+c.ahd.toFixed(3)+'</td>';";
  h += "row+='<td>'+c.whc.toFixed(2)+'</td>';";
  h += "row+='<td>'+c.whd.toFixed(2)+'</td>';";
  h += "row+='<td class=\"'+(c.ce>99?'eff-good':c.ce>95?'eff-warn':'eff-bad')+'\">'+c.ce.toFixed(1)+'</td>';";
  h += "row+='<td class=\"'+(c.ee>85?'eff-good':c.ee>80?'eff-warn':'eff-bad')+'\">'+c.ee.toFixed(1)+'</td>';";
  h += "row+='<td>'+whkg+'</td>';";
  h += "row+='<td>'+whl+'</td>';";
  h += "row+='<td>'+c.pt.toFixed(1)+'°C</td></tr>';";
  h += "tb.innerHTML+=row;";
  h += "labels.push(''+(i+1));ahData.push(c.ahd);ceData.push(c.ce);eeData.push(c.ee);}";

  // Update chart
  h += "sgsChart.data.labels=labels;";
  h += "sgsChart.data.datasets[0].data=ahData;";
  h += "sgsChart.data.datasets[1].data=ceData;";
  h += "sgsChart.data.datasets[2].data=eeData;";
  h += "sgsChart.update();";

  // Summary stats
  h += "$('avgCE').innerText=(sumCE/hist.length).toFixed(2)+'%';";
  h += "$('avgEE').innerText=(sumEE/hist.length).toFixed(2)+'%';";
  h += "if(firstAh>0){$('capRet').innerText=((lastAh/firstAh)*100).toFixed(1)+'%';}";
  h += "}";

  h += "}).catch(()=>{})}";

  h += "setInterval(upd,2000);upd();";
  h += "</script></body></html>";

  sendChunkedPage(h);
}

// SGS Status endpoint
void handleSGSStatus() {
  String j = "{\"v\":" + String(status.voltage, 3);
  j += ",\"i\":" + String(status.current, 2);
  j += ",\"p\":" + String(status.power, 1);
  j += ",\"t\":" + String(status.temperature, 1);
  j += ",\"cyc\":" + String(status.currentCycle);
  j += ",\"ncyc\":" + String(sgsConfig.totalCycles);
  j += ",\"active\":" + String(sgsConfig.sgsTestActive ? "true" : "false");
  j += ",\"phase\":" + String(sgsConfig.sgsCurrentPhase);
  j += ",\"cvMode\":" + String(status.cvMode ? "true" : "false");

  unsigned long elapsed = status.running ? (millis() - status.startTime) / 1000 : 0;
  j += ",\"elapsed\":" + String(elapsed);

  // Current cycle data
  j += ",\"cycAhC\":" + String(status.cycleAhCharge, 4);
  j += ",\"cycAhD\":" + String(status.cycleAhDischarge, 4);
  j += ",\"cycWhC\":" + String(sgsConfig.currentCycleWhCharge, 2);
  j += ",\"cycWhD\":" + String(sgsConfig.currentCycleWhDischarge, 2);

  // Cell specs for calculations
  j += ",\"weight\":" + String(sgsConfig.cellWeightKg, 4);
  j += ",\"volume\":" + String(sgsConfig.cellVolumeLiter, 4);

  // Cycle history
  j += ",\"history\":[";
  for (int i = 0; i < cycleHistoryCount && i < MAX_CYCLE_HISTORY; i++) {
    if (i > 0) j += ",";
    j += "{\"ahc\":" + String(cycleHistory[i].ahCharge, 4);
    j += ",\"ahd\":" + String(cycleHistory[i].ahDischarge, 4);
    j += ",\"whc\":" + String(cycleHistory[i].whCharge, 2);
    j += ",\"whd\":" + String(cycleHistory[i].whDischarge, 2);
    j += ",\"ce\":" + String(cycleHistory[i].coulombEfficiency, 2);
    j += ",\"ee\":" + String(cycleHistory[i].energyEfficiency, 2);
    j += ",\"pt\":" + String(cycleHistory[i].peakTemp, 1) + "}";
  }
  j += "]}";

  server.send(200, "application/json", j);
}

// SGS Start endpoint
void handleSGSStart() {
  if (status.running) {
    server.send(200, "application/json", "{\"ok\":false,\"error\":\"Test already running\"}");
    return;
  }

  // Parse parameters
  if (server.hasArg("weight")) sgsConfig.cellWeightKg = server.arg("weight").toFloat();
  if (server.hasArg("volume")) sgsConfig.cellVolumeLiter = server.arg("volume").toFloat();
  if (server.hasArg("nomCap")) sgsConfig.nominalCapacity = server.arg("nomCap").toFloat();
  if (server.hasArg("nomVolt")) sgsConfig.nominalVoltage = server.arg("nomVolt").toFloat();
  if (server.hasArg("chgV")) sgsConfig.chargeVoltage = server.arg("chgV").toFloat();
  if (server.hasArg("chgI")) sgsConfig.chargeCurrent = server.arg("chgI").toFloat();
  if (server.hasArg("cutI")) sgsConfig.cutoffCurrent = server.arg("cutI").toFloat();
  if (server.hasArg("disV")) sgsConfig.dischargeVoltage = server.arg("disV").toFloat();
  if (server.hasArg("disI")) sgsConfig.dischargeCurrent = server.arg("disI").toFloat();
  if (server.hasArg("maxT")) sgsConfig.maxTempStop = server.arg("maxT").toFloat();
  if (server.hasArg("restT")) sgsConfig.restTimeSeconds = server.arg("restT").toInt();
  if (server.hasArg("numCyc")) sgsConfig.totalCycles = server.arg("numCyc").toInt();

  // Validate
  String error = validateChargeParams(sgsConfig.chargeVoltage, sgsConfig.chargeCurrent);
  if (error.length() > 0) {
    server.send(200, "application/json", "{\"ok\":false,\"error\":\"" + error + "\"}");
    return;
  }

  // Log
  Serial.printf("\n===== SGS TEST START =====\n");
  Serial.printf("Cell: %.1fg, %.1fml, %.1fAh nom\n",
    sgsConfig.cellWeightKg * 1000, sgsConfig.cellVolumeLiter * 1000, sgsConfig.nominalCapacity);
  Serial.printf("Charge: %.2fV, %.1fA, cutoff %.1fA\n",
    sgsConfig.chargeVoltage, sgsConfig.chargeCurrent, sgsConfig.cutoffCurrent);
  Serial.printf("Discharge: %.2fV, %.1fA\n",
    sgsConfig.dischargeVoltage, sgsConfig.dischargeCurrent);
  Serial.printf("Cycles: %d, Rest: %ds, MaxT: %.0f°C\n",
    sgsConfig.totalCycles, sgsConfig.restTimeSeconds, sgsConfig.maxTempStop);

  logSafetyEvent("Anorgion Test started: " + String(sgsConfig.totalCycles) + " cycles");

  // Reset everything
  resetStats();
  resetCycleHistory();
  startNewLog();

  // Start first charge cycle
  sgsConfig.sgsTestActive = true;
  sgsConfig.sgsCurrentPhase = 1; // Charging
  sgsConfig.phaseStartTime = millis();
  status.currentCycle = 1;

  // Configure Delta for charging
  if (!deltaSetupCharge(sgsConfig.chargeVoltage, sgsConfig.chargeCurrent)) {
    sgsConfig.sgsTestActive = false;
    server.send(200, "application/json", "{\"ok\":false,\"error\":\"Delta connection failed\"}");
    return;
  }

  beepStart();
  status.mode = MODE_CYCLE;
  status.running = true;

  server.send(200, "application/json", "{\"ok\":true}");
}

// SGS Stop endpoint
void handleSGSStop() {
  sgsConfig.sgsTestActive = false;
  sgsConfig.sgsCurrentPhase = 0;
  stopTest();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStatus() {
  String j = "{\"v\":" + String(status.voltage, 3);
  j += ",\"i\":" + String(status.current, 2);
  j += ",\"p\":" + String(status.power, 1);
  j += ",\"t\":" + String(status.temperature, 1);
  j += ",\"wh\":" + String(status.totalWh, 2);
  j += ",\"ahc\":" + String(status.totalAhCharge, 4);
  j += ",\"ahd\":" + String(status.totalAhDischarge, 4);
  j += ",\"run\":" + String(status.running ? "true" : "false");
  j += ",\"mode\":" + String(status.mode);
  j += ",\"cyc\":" + String(status.currentCycle);
  j += ",\"ncyc\":" + String(config.numCycles);
  // Delta status - consider "online" if we had success within last 10 seconds
  bool deltaOk = (millis() - status.lastDeltaSuccess < 10000) || status.deltaConnected;
  j += ",\"delta\":" + String(deltaOk ? "true" : "false");
  // Elapsed time tracking
  unsigned long elapsedSec = status.running ? (millis() - status.startTime) / 1000 : 0;
  unsigned long cycleElapsedSec = status.running ? (millis() - status.cycleStartTime) / 1000 : 0;
  j += ",\"elapsed\":" + String(elapsedSec);
  j += ",\"cycleElapsed\":" + String(cycleElapsedSec);
  // Current cycle Ah tracking
  j += ",\"cycAhc\":" + String(status.cycleAhCharge, 4);
  j += ",\"cycAhd\":" + String(status.cycleAhDischarge, 4);
  j += ",\"peakT\":" + String(status.peakTempThisCycle, 1);
  // Delta extended info
  j += ",\"setV\":" + String(status.setVoltage, 2);
  j += ",\"setI\":" + String(status.setCurrent, 2);
  j += ",\"setINeg\":" + String(status.setCurrentNeg, 2);
  j += ",\"cvMode\":" + String(status.cvMode ? "true" : "false");
  j += ",\"cableLoss\":" + String(status.cableLoss, 3);
  // Thermal
  j += ",\"mlx\":" + String(mlxConnected ? "true" : "false");
  j += ",\"mlxMin\":" + String(mlxMin, 1);
  j += ",\"mlxMax\":" + String(mlxMax, 1);
  j += ",\"mlxAvg\":" + String(mlxAvg, 1);
  // Cycle history array
  j += ",\"cycleHistory\":[";
  for (int i = 0; i < cycleHistoryCount && i < MAX_CYCLE_HISTORY; i++) {
    if (i > 0) j += ",";
    j += "{\"n\":" + String(i + 1);
    j += ",\"ahc\":" + String(cycleHistory[i].ahCharge, 3);
    j += ",\"ahd\":" + String(cycleHistory[i].ahDischarge, 3);
    j += ",\"dur\":" + String((cycleHistory[i].chargeDurationMs + cycleHistory[i].dischargeDurationMs) / 1000);
    j += ",\"pt\":" + String(cycleHistory[i].peakTemp, 1);
    j += ",\"done\":" + String(cycleHistory[i].completed ? "true" : "false") + "}";
  }
  j += "]";
  j += ",\"err\":\"" + status.lastError + "\"}";
  server.send(200, "application/json", j);
}

void handleThermalData() {
  if (!mlxConnected) {
    server.send(200, "application/json", "{\"error\":\"MLX90640 not connected\"}");
    return;
  }

  String j = "{\"data\":true,\"min\":" + String(mlxMin, 1);
  j += ",\"max\":" + String(mlxMax, 1);
  j += ",\"avg\":" + String(mlxAvg, 1);
  j += ",\"colors\":[";

  for (int i = 0; i < MLX_PIXELS; i++) {
    if (i > 0) j += ",";
    j += "\"" + getThermalColor(mlxFrame[i], mlxMin, mlxMax) + "\"";
  }
  j += "]}";

  server.send(200, "application/json", j);
}

void handlePing() {
  // Block ping during active test - it would destroy the TCP connection
  if (status.running) {
    server.send(200, "application/json", "{\"result\":\"BLOCKED: Test running - ping would kill connection\"}");
    return;
  }
  String r = deltaPing();
  server.send(200, "application/json", "{\"result\":\"" + r + "\"}");
}

void handleCharge() {
  float v = server.hasArg("v") ? server.arg("v").toFloat() : config.maxVoltage;
  float i = server.hasArg("i") ? server.arg("i").toFloat() : config.chargeCurrent;

  // Validate parameters
  String error = validateChargeParams(v, i);
  if (error.length() > 0) {
    logSafetyEvent("CHARGE BLOCKED: " + error + " (V=" + String(v,2) + " I=" + String(i,1) + ")");
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"" + error + "\"}");
    beepFail();
    return;
  }

  // Live update if already charging - adjust Delta without stopping
  if (status.running && status.mode == MODE_CHARGE) {
    logSafetyEvent("CHARGE LIVE UPDATE: V=" + String(v,2) + " I=" + String(i,1));
    if (deltaUpdateChargeParams(v, i)) {
      server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Parameters updated live\"}");
    } else {
      server.send(500, "application/json", "{\"ok\":false,\"error\":\"Delta live update failed\"}");
    }
    return;
  }

  logSafetyEvent("CHARGE STARTED: V=" + String(v,2) + " I=" + String(i,1));
  startCharge(v, i);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleDischarge() {
  float v = server.hasArg("v") ? server.arg("v").toFloat() : config.minVoltage;
  float i = server.hasArg("i") ? server.arg("i").toFloat() : config.dischargeCurrent;

  // Validate parameters
  String error = validateDischargeParams(v, i);
  if (error.length() > 0) {
    logSafetyEvent("DISCHARGE BLOCKED: " + error + " (V=" + String(v,2) + " I=" + String(i,1) + ")");
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"" + error + "\"}");
    beepFail();
    return;
  }

  // Live update if already discharging - adjust Delta without stopping
  if (status.running && status.mode == MODE_DISCHARGE) {
    logSafetyEvent("DISCHARGE LIVE UPDATE: V=" + String(v,2) + " I=" + String(i,1));
    if (deltaUpdateDischargeParams(v, i)) {
      server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Parameters updated live\"}");
    } else {
      server.send(500, "application/json", "{\"ok\":false,\"error\":\"Delta live update failed\"}");
    }
    return;
  }

  logSafetyEvent("DISCHARGE STARTED: V=" + String(v,2) + " I=" + String(i,1));
  startDischarge(v, i);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleCycle() {
  // Validate cycle parameters (uses config values)
  String error = validateChargeParams(config.maxVoltage, config.chargeCurrent);
  if (error.length() > 0) {
    logSafetyEvent("CYCLE BLOCKED (charge params): " + error);
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"" + error + "\"}");
    beepFail();
    return;
  }
  error = validateDischargeParams(config.minVoltage, config.dischargeCurrent);
  if (error.length() > 0) {
    logSafetyEvent("CYCLE BLOCKED (discharge params): " + error);
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"" + error + "\"}");
    beepFail();
    return;
  }

  logSafetyEvent("CYCLE STARTED: " + String(config.minVoltage,2) + "-" + String(config.maxVoltage,2) + "V");
  startCycle();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStop() {
  Serial.println("[WEB] STOP requested - setting flag only!");

  // ONLY set flags - do NOT call deltaStop here!
  // The main loop will handle the actual stop
  stopRequested = true;
  status.running = false;
  status.mode = MODE_IDLE;

  // Send response immediately
  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Stop flag set\"}");

  // Log it
  Serial.println("[WEB] Stop flag set, returning to browser");
}

void handleSave() {
  String changes = "";

  // Validate and apply settings with safety checks
  if (server.hasArg("minV")) {
    float newV = server.arg("minV").toFloat();
    if (newV < safetyLimits.absMinVoltage || newV > safetyLimits.absMaxVoltage) {
      server.send(400, "application/json",
        "{\"ok\":false,\"error\":\"Min voltage outside safety limits\"}");
      return;
    }
    if (newV != config.minVoltage) {
      changes += "minV:" + String(config.minVoltage,2) + "->" + String(newV,2) + " ";
      config.minVoltage = newV;
    }
  }

  if (server.hasArg("maxV")) {
    float newV = server.arg("maxV").toFloat();
    if (newV < safetyLimits.absMinVoltage || newV > safetyLimits.absMaxVoltage) {
      server.send(400, "application/json",
        "{\"ok\":false,\"error\":\"Max voltage outside safety limits\"}");
      return;
    }
    if (newV != config.maxVoltage) {
      changes += "maxV:" + String(config.maxVoltage,2) + "->" + String(newV,2) + " ";
      config.maxVoltage = newV;
    }
  }

  if (server.hasArg("chgI")) {
    float newI = server.arg("chgI").toFloat();
    if (newI > safetyLimits.absMaxChargeCurrent) {
      server.send(400, "application/json",
        "{\"ok\":false,\"error\":\"Charge current exceeds safety limit of " +
        String(safetyLimits.absMaxChargeCurrent,1) + "A\"}");
      return;
    }
    if (newI != config.chargeCurrent) {
      changes += "chgI:" + String(config.chargeCurrent,1) + "->" + String(newI,1) + " ";
      config.chargeCurrent = newI;
    }
  }

  if (server.hasArg("disI")) {
    float newI = server.arg("disI").toFloat();
    if (newI > safetyLimits.absMaxDischargeCurrent) {
      server.send(400, "application/json",
        "{\"ok\":false,\"error\":\"Discharge current exceeds safety limit of " +
        String(safetyLimits.absMaxDischargeCurrent,1) + "A\"}");
      return;
    }
    if (newI != config.dischargeCurrent) {
      changes += "disI:" + String(config.dischargeCurrent,1) + "->" + String(newI,1) + " ";
      config.dischargeCurrent = newI;
    }
  }

  if (server.hasArg("cyc")) {
    int newCyc = server.arg("cyc").toInt();
    if (newCyc != config.numCycles) {
      changes += "cyc:" + String(config.numCycles) + "->" + String(newCyc) + " ";
      config.numCycles = newCyc;
    }
  }

  if (server.hasArg("log")) {
    int newLog = server.arg("log").toInt();
    if (newLog != config.logInterval) {
      changes += "log:" + String(config.logInterval) + "->" + String(newLog) + " ";
      config.logInterval = newLog;
    }
  }

  if (server.hasArg("temp")) {
    float newT = server.arg("temp").toFloat();
    if (newT != config.tempAlarm) {
      changes += "temp:" + String(config.tempAlarm,0) + "->" + String(newT,0) + " ";
      config.tempAlarm = newT;
    }
  }

  if (server.hasArg("therm")) {
    float newT = server.arg("therm").toFloat();
    if (newT != config.thermalAlarm) {
      changes += "therm:" + String(config.thermalAlarm,0) + "->" + String(newT,0) + " ";
      config.thermalAlarm = newT;
    }
  }

  if (changes.length() > 0) {
    logSafetyEvent("CONFIG CHANGED: " + changes);
  }

  saveConfig();
  server.send(200, "application/json", "{\"ok\":true}");
}

// Get safety limits for frontend
void handleGetSafetyLimits() {
  String j = "{";
  j += "\"absMinV\":" + String(safetyLimits.absMinVoltage, 2);
  j += ",\"absMaxV\":" + String(safetyLimits.absMaxVoltage, 2);
  j += ",\"absMaxCI\":" + String(safetyLimits.absMaxChargeCurrent, 1);
  j += ",\"absMaxDI\":" + String(safetyLimits.absMaxDischargeCurrent, 1);
  j += ",\"warnHiI\":" + String(safetyLimits.warnHighCurrent, 1);
  j += ",\"warnHiV\":" + String(safetyLimits.warnHighVoltage, 2);
  j += ",\"warnLoV\":" + String(safetyLimits.warnLowVoltage, 2);
  j += "}";
  server.send(200, "application/json", j);
}

// Verify safety code and update limits
void handleUpdateSafetyLimits() {
  // Verify safety code
  if (!server.hasArg("code") || server.arg("code") != safetyLimits.safetyCode) {
    logSafetyEvent("SAFETY LIMITS CHANGE DENIED - wrong code");
    beepFail();
    server.send(403, "application/json", "{\"ok\":false,\"error\":\"Invalid safety code\"}");
    return;
  }

  String changes = "SAFETY LIMITS UPDATED: ";

  if (server.hasArg("absMinV")) {
    float v = server.arg("absMinV").toFloat();
    changes += "absMinV:" + String(safetyLimits.absMinVoltage,2) + "->" + String(v,2) + " ";
    safetyLimits.absMinVoltage = v;
  }
  if (server.hasArg("absMaxV")) {
    float v = server.arg("absMaxV").toFloat();
    changes += "absMaxV:" + String(safetyLimits.absMaxVoltage,2) + "->" + String(v,2) + " ";
    safetyLimits.absMaxVoltage = v;
  }
  if (server.hasArg("absMaxCI")) {
    float i = server.arg("absMaxCI").toFloat();
    changes += "absMaxCI:" + String(safetyLimits.absMaxChargeCurrent,1) + "->" + String(i,1) + " ";
    safetyLimits.absMaxChargeCurrent = i;
  }
  if (server.hasArg("absMaxDI")) {
    float i = server.arg("absMaxDI").toFloat();
    changes += "absMaxDI:" + String(safetyLimits.absMaxDischargeCurrent,1) + "->" + String(i,1) + " ";
    safetyLimits.absMaxDischargeCurrent = i;
  }
  if (server.hasArg("warnHiI")) {
    float i = server.arg("warnHiI").toFloat();
    safetyLimits.warnHighCurrent = i;
  }
  if (server.hasArg("warnHiV")) {
    float v = server.arg("warnHiV").toFloat();
    safetyLimits.warnHighVoltage = v;
  }
  if (server.hasArg("warnLoV")) {
    float v = server.arg("warnLoV").toFloat();
    safetyLimits.warnLowVoltage = v;
  }
  if (server.hasArg("newCode") && server.arg("newCode").length() >= 4) {
    changes += "CODE CHANGED ";
    safetyLimits.safetyCode = server.arg("newCode");
  }

  logSafetyEvent(changes);
  saveSafetyLimits();
  beepOK();
  server.send(200, "application/json", "{\"ok\":true}");
}

// Get safety log
void handleGetSafetyLog() {
  if (!spiffsReady) {
    server.send(503, "text/plain", "Storage not available");
    return;
  }
  File f = SPIFFS.open(SAFETY_LOG_FILE, FILE_READ);
  if (!f) {
    server.send(200, "text/plain", "No safety log entries");
    return;
  }
  server.streamFile(f, "text/plain");
  f.close();
}

void handleDownloadCSV() {
  if (!spiffsReady) {
    server.send(503, "text/plain", "Storage not available");
    return;
  }
  File f = SPIFFS.open(LOG_FILE, FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "No log file found");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=battery_log.csv");
  server.streamFile(f, "text/csv");
  f.close();
}

void handleDeleteLog() {
  deleteLog();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleLogInfo() {
  if (!spiffsReady) {
    server.send(200, "application/json", "{\"logSize\":0,\"freeSpace\":0,\"logging\":false,\"error\":\"Storage unavailable\"}");
    return;
  }
  size_t logSize = getLogSize();
  size_t freeSpace = SPIFFS.totalBytes() - SPIFFS.usedBytes();
  String j = "{\"logSize\":" + String(logSize);
  j += ",\"freeSpace\":" + String(freeSpace);
  j += ",\"logging\":" + String(loggingEnabled ? "true" : "false") + "}";
  server.send(200, "application/json", j);
}

// ============== Wh CAPACITY TEST ==============
// Quick single-cycle test at 12A to determine Wh capacity

void sendWhTestPage() {
  yield();  // Allow other tasks
  String h = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Wh Capacity Test</title>";
  h += "<style>";
  h += "body{font:16px sans-serif;background:#1a1a2e;color:#eee;margin:0;padding:10px}";
  h += ".panel{background:#16213e;border-radius:8px;padding:15px;margin:10px 0}";
  h += "h1{color:#0cf;margin:0 0 15px 0;font-size:1.5em}";
  h += "h2{color:#0f6;margin:0 0 10px 0;font-size:1.1em}";
  h += ".row{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #333}";
  h += ".k{color:#888}.v{color:#fff;font-weight:bold}";
  h += "input{background:#0d1b2a;border:1px solid #444;color:#fff;padding:8px;border-radius:4px;width:80px;text-align:center}";
  h += "button{padding:12px 20px;border:none;border-radius:4px;cursor:pointer;font-weight:bold;margin:5px}";
  h += ".bg{background:#0a0;color:#fff}.br{background:#f00;color:#fff}.bb{background:#06f;color:#fff}";
  h += ".big{font-size:2.5em;color:#0f6;text-align:center;padding:20px}";
  h += ".result{background:#0a3;padding:20px;border-radius:8px;margin:10px 0}";
  h += "</style></head><body>";

  h += "<h1>Wh Capacity Test</h1>";
  h += "<a href='/' style='color:#0cf'>← Back to Dashboard</a>";

  // Status panel
  h += "<div class='panel'><h2>Current Status</h2>";
  h += "<div class='row'><span class='k'>Voltage</span><span class='v' id='volt'>-</span></div>";
  h += "<div class='row'><span class='k'>Current</span><span class='v' id='curr'>-</span></div>";
  h += "<div class='row'><span class='k'>Power</span><span class='v' id='pwr'>-</span></div>";
  h += "<div class='row'><span class='k'>Temperature</span><span class='v' id='temp'>-</span></div>";
  h += "<div class='row'><span class='k'>Phase</span><span class='v' id='phase'>Idle</span></div>";
  h += "</div>";

  // Result panel
  h += "<div class='panel result' id='resultPanel' style='display:none'>";
  h += "<h2>Test Result</h2>";
  h += "<div class='big' id='whResult'>0.00 Wh</div>";
  h += "<div class='row'><span class='k'>Ah Charged</span><span class='v' id='ahChg'>-</span></div>";
  h += "<div class='row'><span class='k'>Ah Discharged</span><span class='v' id='ahDis'>-</span></div>";
  h += "<div class='row'><span class='k'>Wh Charged</span><span class='v' id='whChg'>-</span></div>";
  h += "<div class='row'><span class='k'>Wh Discharged</span><span class='v' id='whDis'>-</span></div>";
  h += "<div class='row'><span class='k'>Energy Efficiency</span><span class='v' id='eff'>-</span></div>";
  h += "</div>";

  // Test parameters
  h += "<div class='panel'><h2>Test Parameters</h2>";
  h += "<div class='row'><span class='k'>Charge Voltage</span><input type='number' id='chgV' value='4.15' step='0.01'> V</div>";
  h += "<div class='row'><span class='k'>Charge Current</span><input type='number' id='chgI' value='12' step='0.1'> A</div>";
  h += "<div class='row'><span class='k'>Cutoff Current</span><input type='number' id='cutI' value='1.5' step='0.1'> A</div>";
  h += "<div class='row'><span class='k'>Discharge Voltage</span><input type='number' id='disV' value='2.70' step='0.01'> V</div>";
  h += "<div class='row'><span class='k'>Discharge Current</span><input type='number' id='disI' value='12' step='0.1'> A</div>";
  h += "</div>";

  // Control buttons
  h += "<div class='panel' style='text-align:center'>";
  h += "<button class='bg' style='font-size:1.2em;padding:15px 40px' onclick='startTest()'>START Wh TEST</button>";
  h += "<button class='br' style='font-size:1.2em;padding:15px 40px' onclick='stopTest()'>STOP</button>";
  h += "</div>";

  // JavaScript
  h += "<script>";
  h += "function startTest(){";
  h += "var p='chgV='+$('chgV').value+'&chgI='+$('chgI').value+'&cutI='+$('cutI').value+'&disV='+$('disV').value+'&disI='+$('disI').value;";
  h += "if(!confirm('Start Wh Capacity Test?\\n\\n1. Charge to '+$('chgV').value+'V at '+$('chgI').value+'A\\n2. Discharge to '+$('disV').value+'V at '+$('disI').value+'A'))return;";
  h += "fetch('/whtest/start?'+p).then(r=>r.json()).then(d=>{if(d.ok)alert('Test started!');else alert('Error: '+d.msg);});}";
  h += "function stopTest(){if(confirm('Stop test?')){fetch('/stop').then(()=>alert('Stopped'));}}";
  h += "function $(i){return document.getElementById(i)}";
  h += "function upd(){fetch('/whtest/status').then(r=>r.json()).then(d=>{";
  h += "$('volt').innerText=d.v.toFixed(3)+'V';";
  h += "$('curr').innerText=d.i.toFixed(2)+'A';";
  h += "$('pwr').innerText=d.p.toFixed(1)+'W';";
  h += "$('temp').innerText=d.t.toFixed(1)+'°C';";
  h += "$('phase').innerText=d.phase;";
  h += "if(d.done){";
  h += "$('resultPanel').style.display='block';";
  h += "$('whResult').innerText=d.whDis.toFixed(2)+' Wh';";
  h += "$('ahChg').innerText=d.ahChg.toFixed(3)+' Ah';";
  h += "$('ahDis').innerText=d.ahDis.toFixed(3)+' Ah';";
  h += "$('whChg').innerText=d.whChg.toFixed(2)+' Wh';";
  h += "$('whDis').innerText=d.whDis.toFixed(2)+' Wh';";
  h += "var eff=d.whChg>0?(d.whDis/d.whChg*100):0;";
  h += "$('eff').innerText=eff.toFixed(1)+'%';";
  h += "}}).catch(e=>console.log(e));}";
  h += "setInterval(upd,1000);upd();";
  h += "</script></body></html>";

  sendChunkedPage(h);
}

// Wh test state
struct {
  bool active = false;
  int phase = 0;  // 0=idle, 1=charging, 2=discharging, 3=done
  float chargeVoltage = 4.15;
  float chargeCurrent = 12.0;
  float cutoffCurrent = 1.5;
  float dischargeVoltage = 2.70;
  float dischargeCurrent = 12.0;
  float ahCharge = 0;
  float ahDischarge = 0;
  float whCharge = 0;
  float whDischarge = 0;
  unsigned long phaseStartTime = 0;
} whTest;

void handleWhTestStatus() {
  String phase = "Idle";
  if (whTest.phase == 1) phase = "Charging";
  else if (whTest.phase == 2) phase = "Discharging";
  else if (whTest.phase == 3) phase = "Complete";

  String j = "{\"v\":" + String(status.voltage, 3);
  j += ",\"i\":" + String(status.current, 2);
  j += ",\"p\":" + String(status.power, 1);
  j += ",\"t\":" + String(status.temperature, 1);
  j += ",\"phase\":\"" + phase + "\"";
  j += ",\"done\":" + String(whTest.phase == 3 ? "true" : "false");
  j += ",\"ahChg\":" + String(whTest.ahCharge, 3);
  j += ",\"ahDis\":" + String(whTest.ahDischarge, 3);
  j += ",\"whChg\":" + String(whTest.whCharge, 2);
  j += ",\"whDis\":" + String(whTest.whDischarge, 2);
  j += "}";
  server.send(200, "application/json", j);
}

void handleWhTestStart() {
  if (status.running) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"Test already running\"}");
    return;
  }

  whTest.chargeVoltage = server.hasArg("chgV") ? server.arg("chgV").toFloat() : 4.15;
  whTest.chargeCurrent = server.hasArg("chgI") ? server.arg("chgI").toFloat() : 12.0;
  whTest.cutoffCurrent = server.hasArg("cutI") ? server.arg("cutI").toFloat() : 1.5;
  whTest.dischargeVoltage = server.hasArg("disV") ? server.arg("disV").toFloat() : 2.70;
  whTest.dischargeCurrent = server.hasArg("disI") ? server.arg("disI").toFloat() : 12.0;

  // Validate against safety limits
  if (whTest.chargeVoltage > safetyLimits.absMaxVoltage || whTest.chargeVoltage < safetyLimits.absMinVoltage) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"Charge voltage outside safety limits\"}");
    return;
  }
  if (whTest.chargeCurrent > safetyLimits.absMaxChargeCurrent) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"Charge current exceeds safety limit\"}");
    return;
  }

  // Reset counters
  whTest.ahCharge = 0;
  whTest.ahDischarge = 0;
  whTest.whCharge = 0;
  whTest.whDischarge = 0;
  whTest.phase = 1;
  whTest.active = true;
  whTest.phaseStartTime = millis();

  // Reset status counters
  status.totalAhCharge = 0;
  status.totalAhDischarge = 0;
  status.totalWh = 0;
  status.cycleAhCharge = 0;
  status.cycleAhDischarge = 0;
  sgsConfig.currentCycleWhCharge = 0;
  sgsConfig.currentCycleWhDischarge = 0;

  // Start charging
  if (!deltaSetupCharge(whTest.chargeVoltage, whTest.chargeCurrent)) {
    whTest.active = false;
    whTest.phase = 0;
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"Delta connection failed\"}");
    return;
  }

  status.running = true;
  status.mode = MODE_CHARGE;
  status.startTime = millis();
  status.lastMeasureTime = millis();

  logSafetyEvent("Wh Test started");
  beepStart();

  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Wh Test started\"}");
}

void updateWhTest() {
  if (!whTest.active) return;

  // Update energy counters from status
  whTest.ahCharge = status.totalAhCharge;
  whTest.ahDischarge = status.totalAhDischarge;
  whTest.whCharge = sgsConfig.currentCycleWhCharge;
  whTest.whDischarge = sgsConfig.currentCycleWhDischarge;

  switch (whTest.phase) {
    case 1: // Charging
      if (status.voltage >= whTest.chargeVoltage - 0.02) {
        // At voltage limit - check if current dropped below cutoff
        if (abs(status.current) < whTest.cutoffCurrent) {
          Serial.printf("[WhTest] Charge complete: %.3f Ah, %.2f Wh\n", whTest.ahCharge, whTest.whCharge);
          playTone(1000, 150);

          // Stop charging, start discharge
          quickDeltaOff();
          delay(1000);  // Brief pause

          if (!deltaSetupDischarge(whTest.dischargeVoltage, whTest.dischargeCurrent)) {
            status.lastError = "Delta connection failed during discharge setup";
            whTest.active = false;
            whTest.phase = 0;
            return;
          }

          whTest.phase = 2;
          whTest.phaseStartTime = millis();
          status.mode = MODE_DISCHARGE;
        }
      }
      break;

    case 2: // Discharging
      // FIXED: Stop when voltage reaches limit OR current drops below threshold
      // Previously required BOTH conditions which was dangerous!
      if (status.voltage > 0.5) {
        bool voltageReached = (status.voltage <= whTest.dischargeVoltage + 0.02);
        bool currentLow = (abs(status.current) < 1.0);

        // Stop if EITHER condition is met (voltage limit is primary!)
        if (voltageReached || currentLow) {
          Serial.printf("[WhTest] Discharge complete: %.3f Ah, %.2f Wh (V=%.3f, I=%.2f)\n",
            whTest.ahDischarge, whTest.whDischarge, status.voltage, status.current);
          playTone(600, 200);

          // Test complete - STOP IMMEDIATELY
          quickDeltaOff();
          delay(100);
          quickDeltaOff();  // Double-tap for safety

          whTest.phase = 3;
          whTest.active = false;
          status.running = false;
          status.mode = MODE_IDLE;

          beepDone();
          String reason = voltageReached ? "voltage limit" : "current low";
          logSafetyEvent("Wh Test complete (" + reason + "): " + String(whTest.whDischarge, 2) + " Wh");
        }
      }
      break;
  }
}

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========== Battery Tester v7.5 - HARD SAFETY LIMITS ==========\n");

  initSPIFFS();
  loadConfig();
  loadSafetyLimits();

  pinMode(PA_ENABLE_PIN, OUTPUT);
  digitalWrite(PA_ENABLE_PIN, HIGH);

  // Blue LED for status indication
  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);  // 100kHz for ES8311

  es8311_init();
  i2s_init();

  // Initialize MLX90640 on separate I2C bus (Wire1, GPIO 15/14)
  Wire1.begin(MLX_SDA_PIN, MLX_SCL_PIN);
  Wire1.setClock(400000);  // 400kHz for MLX90640
  mlxConnected = mlxInit();
  if (mlxConnected) {
    Serial.println("[MLX] Thermal camera ready!");
  } else {
    Serial.println("[MLX] Not found - continuing without thermal");
  }

  tempSensor.begin();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WIFI] %s\n", WiFi.localIP().toString().c_str());

  server.on("/", sendMainPage);
  server.on("/settings", sendSettingsPage);
  server.on("/thermal", sendThermalPage);
  server.on("/sgs", sendSGSPage);
  server.on("/sgs/status", handleSGSStatus);
  server.on("/sgs/start", handleSGSStart);
  server.on("/sgs/stop", handleSGSStop);
  server.on("/whtest", sendWhTestPage);
  server.on("/whtest/status", handleWhTestStatus);
  server.on("/whtest/start", handleWhTestStart);
  server.on("/status", handleStatus);
  server.on("/thermaldata", handleThermalData);
  server.on("/ping", handlePing);
  server.on("/charge", handleCharge);
  server.on("/discharge", handleDischarge);
  server.on("/cycle", handleCycle);
  server.on("/stop", handleStop);
  server.on("/save", handleSave);
  server.on("/download", handleDownloadCSV);
  server.on("/deletelog", handleDeleteLog);
  server.on("/loginfo", handleLogInfo);
  server.on("/safety", handleGetSafetyLimits);
  server.on("/safety/update", handleUpdateSafetyLimits);
  server.on("/safety/log", handleGetSafetyLog);
  server.begin();

  readTemp();
  playTone(800, 100); playSilence(50); playTone(1000, 100); playSilence(50); playTone(1200, 150);

  Serial.printf("\nReady: http://%s\n\n", WiFi.localIP().toString().c_str());
}

// ============== SGS TEST UPDATE ==============

void updateSGSTest() {
  if (!sgsConfig.sgsTestActive) return;

  // Temperature safety check
  if (status.temperature > sgsConfig.maxTempStop) {
    Serial.printf("[SGS] Temperature %.1f°C exceeds limit %.1f°C - STOPPING\n",
      status.temperature, sgsConfig.maxTempStop);
    status.lastError = "TEMP ALARM: " + String(status.temperature, 1) + "°C";
    sgsConfig.sgsTestActive = false;
    stopTest();
    beepFail();
    return;
  }

  unsigned long phaseElapsed = (millis() - sgsConfig.phaseStartTime) / 1000;

  switch (sgsConfig.sgsCurrentPhase) {
    case 1: // CHARGING (CC/CV)
      // Check if we reached CV phase and current dropped below cutoff
      if (status.voltage >= sgsConfig.chargeVoltage - 0.02) {
        // We're at voltage limit (CV mode)
        if (abs(status.current) < sgsConfig.cutoffCurrent) {
          Serial.printf("[SGS] Cycle %d: Charge complete (I=%.2fA < cutoff %.1fA)\n",
            status.currentCycle, status.current, sgsConfig.cutoffCurrent);
          playTone(1000, 150);

          // Stop charging, start rest
          quickDeltaOff();
          sgsConfig.sgsCurrentPhase = 2; // Rest after charge
          sgsConfig.phaseStartTime = millis();
          Serial.printf("[SGS] Starting rest period (%d sec)\n", sgsConfig.restTimeSeconds);
        }
      }
      break;

    case 2: // REST after charge
      if (phaseElapsed >= (unsigned long)sgsConfig.restTimeSeconds) {
        Serial.printf("[SGS] Cycle %d: Rest complete, starting discharge\n", status.currentCycle);

        // Start discharge
        if (!deltaSetupDischarge(sgsConfig.dischargeVoltage, sgsConfig.dischargeCurrent)) {
          status.lastError = "Delta connection failed during discharge setup";
          sgsConfig.sgsTestActive = false;
          stopTest();
          beepFail();
          return;
        }

        sgsConfig.sgsCurrentPhase = 3; // Discharging
        sgsConfig.phaseStartTime = millis();
        playTone(800, 150);
      }
      break;

    case 3: // DISCHARGING (CC)
      // FIXED: Stop when voltage reaches limit OR current is low
      // Voltage limit is the PRIMARY safety condition!
      if (status.voltage > 0.5) {
        bool voltageReached = (status.voltage <= sgsConfig.dischargeVoltage + 0.02);
        bool currentLow = (abs(status.current) < 2.0);

        // Stop if EITHER condition is met
        if (voltageReached || currentLow) {
          Serial.printf("[SGS] Cycle %d: Discharge complete (V=%.3fV, I=%.2fA, reason=%s)\n",
            status.currentCycle, status.voltage, status.current,
            voltageReached ? "voltage" : "current");
          playTone(600, 150);

          // Stop discharging - IMMEDIATELY
          quickDeltaOff();
          delay(100);
          quickDeltaOff();  // Double-tap for safety

          sgsConfig.sgsCurrentPhase = 4; // Rest after discharge
          sgsConfig.phaseStartTime = millis();

          // Save cycle data
          saveCycleToHistory(false);

          Serial.printf("[SGS] Starting rest period (%d sec)\n", sgsConfig.restTimeSeconds);
        }
      }
      break;

    case 4: // REST after discharge
      if (phaseElapsed >= (unsigned long)sgsConfig.restTimeSeconds) {
        // Check if we're done
        if (status.currentCycle >= sgsConfig.totalCycles) {
          Serial.printf("\n===== SGS TEST COMPLETE: %d cycles =====\n", status.currentCycle);
          logSafetyEvent("Anorgion Test completed: " + String(status.currentCycle) + " cycles");
          sgsConfig.sgsTestActive = false;
          sgsConfig.sgsCurrentPhase = 0;
          status.running = false;
          status.mode = MODE_IDLE;
          beepDone();
          return;
        }

        // Start next cycle
        status.currentCycle++;
        startNewCyclePhase();
        Serial.printf("[SGS] Starting cycle %d/%d\n", status.currentCycle, sgsConfig.totalCycles);

        // Start charging
        if (!deltaSetupCharge(sgsConfig.chargeVoltage, sgsConfig.chargeCurrent)) {
          status.lastError = "Delta connection failed during charge setup";
          sgsConfig.sgsTestActive = false;
          stopTest();
          beepFail();
          return;
        }

        sgsConfig.sgsCurrentPhase = 1; // Charging
        sgsConfig.phaseStartTime = millis();
        playTone(1000, 100);
      }
      break;
  }
}

// ============== LOOP ==============

// Quick Delta off - minimal blocking
void quickDeltaOff() {
  Serial.println("[DELTA] Quick OFF - zeroing setpoints + output off");
  // Reconnect if needed - safety is more important than speed
  if (!deltaClient.connected()) {
    Serial.println("[DELTA] Quick OFF - reconnecting to Delta...");
    deltaConnect();
  }
  if (deltaClient.connected()) {
    // Zero all setpoints FIRST to prevent residual current flow
    deltaClient.print("SOURce:CURrent 0\r\n");
    delay(80);
    deltaClient.print("SOURce:CURrent:NEGative 0\r\n");
    delay(80);
    deltaClient.print("SOURce:POWer 0\r\n");
    delay(80);
    deltaClient.print("SOURce:POWer:NEGative 0\r\n");
    delay(80);
    deltaClient.print("SOURce:VOLtage 0\r\n");
    delay(80);
    // NOW turn output off
    deltaClient.print("OUTPut OFF\r\n");
    deltaClient.flush();
    delay(300);  // Give Delta time to process before closing TCP
    Serial.println("[DELTA] Quick OFF - commands sent OK");
  } else {
    Serial.println("[DELTA] Quick OFF - FAILED to reach Delta!");
  }
  deltaClient.stop();
  status.deltaConnected = false;
}

void loop() {
  // Handle web requests first - highest priority
  server.handleClient();
  yield();

  // Blue LED status indicator
  // Fast blink (200ms) = running, slow blink (1000ms) = idle
  unsigned long blinkInterval = status.running ? 200 : 1000;
  if (millis() - lastLedToggle >= blinkInterval) {
    lastLedToggle = millis();
    ledState = !ledState;
    digitalWrite(BLUE_LED_PIN, ledState ? HIGH : LOW);
  }

  // Check for stop request - handle with minimal blocking
  static bool stopInProgress = false;
  if (stopRequested && !stopInProgress) {
    stopInProgress = true;
    Serial.println("[LOOP] Processing stop request...");

    // Quick Delta off - don't wait for confirmation
    quickDeltaOff();

    // Reset ALL test states
    status.running = false;
    status.mode = MODE_IDLE;
    sgsConfig.sgsTestActive = false;
    sgsConfig.sgsCurrentPhase = 0;
    whTest.active = false;
    whTest.phase = 0;
    stopRequested = false;
    stopInProgress = false;

    logSafetyEvent("STOP executed via loop");
    beepStop();
    Serial.println("[LOOP] Stop complete");
    return;
  }

  // Read thermal camera
  if (mlxConnected) {
    mlxRead();
    yield();
  }

  // Update test every 2 seconds (increased for slow Delta)
  static unsigned long lastUpd = 0;
  if (millis() - lastUpd >= 2000) {
    lastUpd = millis();
    if (status.running && !stopRequested) {
      // SGS/Anorgion test has its own state machine with cutoff current detection
      if (sgsConfig.sgsTestActive) {
        // Read measurements for SGS mode
        deltaReadMeasurements();
        readTemp();
        appendLogEntry();
        yield();
        updateSGSTest();
      } else if (whTest.active) {
        // Wh capacity test mode
        deltaReadMeasurements();
        readTemp();
        appendLogEntry();
        yield();
        updateWhTest();
      } else {
        // Normal mode - updateTest handles everything
        updateTest();
      }
      yield();
    }
  }

  // Read temperature every 5 seconds when idle
  static unsigned long lastTemp = 0;
  if (millis() - lastTemp >= 5000) {
    lastTemp = millis();
    if (!status.running) {
      readTemp();
      yield();
    }
  }

  // Read Delta measurements every 5 seconds when idle (to keep display updated)
  // Uses shorter timeout to avoid blocking web requests too long
  static unsigned long lastIdleDelta = 0;
  if (millis() - lastIdleDelta >= 5000) {
    lastIdleDelta = millis();
    if (!status.running && !stopRequested) {
      // Quick single voltage read to check if Delta is responsive
      if (deltaClient.connected() || deltaConnect()) {
        // Handle any pending web requests first
        server.handleClient();
        yield();

        // Quick voltage query with shorter effective timeout (exit early on response)
        String vStr = deltaQuery("MEASure:VOLtage?");
        if (vStr.length() > 0) {
          server.handleClient();
          yield();
          String iStr = deltaQuery("MEASure:CURrent?");
          server.handleClient();
          yield();
          String pStr = deltaQuery("MEASure:POWer?");

          float v = vStr.toFloat();
          float i = iStr.toFloat();
          float p = pStr.toFloat();
          if (v >= 0 && v <= 100) {
            status.voltage = v;
            status.current = i;
            status.power = p;
          }

          // IDLE SAFETY MONITOR: protect battery even when no test is running
          // If voltage is dangerously low and current is negative (battery draining),
          // emergency shutdown the Delta to prevent battery damage
          if (v > 0.5 && v < HARD_MIN_VOLTAGE && i < -0.5) {
            Serial.printf("\n!!! IDLE SAFETY: V=%.3fV < %.2fV while draining %.2fA !!!\n", v, HARD_MIN_VOLTAGE, i);
            logSafetyEvent("IDLE SAFETY STOP: V=" + String(v, 3) + "V I=" + String(i, 2) + "A");
            quickDeltaOff();
            delay(100);
            quickDeltaOff();  // Double-tap for safety
            status.lastError = "IDLE SAFETY: Battery below " + String(HARD_MIN_VOLTAGE, 2) + "V!";
            beepFail();
          }
          // Also protect against overvoltage while idle
          if (v > HARD_MAX_VOLTAGE) {
            Serial.printf("\n!!! IDLE SAFETY: V=%.3fV > %.2fV !!!\n", v, HARD_MAX_VOLTAGE);
            logSafetyEvent("IDLE SAFETY STOP: V=" + String(v, 3) + "V > max");
            quickDeltaOff();
            delay(100);
            quickDeltaOff();
            status.lastError = "IDLE SAFETY: Battery above " + String(HARD_MAX_VOLTAGE, 2) + "V!";
            beepFail();
          }
        }
      }
      yield();
    }
  }

  delay(10);
  yield();
}
