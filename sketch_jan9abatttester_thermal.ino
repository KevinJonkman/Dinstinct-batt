/*
 * Battery Tester v7.0 - WITH MLX90640 THERMAL CAMERA
 * Based on v6.3 + MLX90640 32x24 thermal imaging
 *
 * Hardware:
 * - ESP32 LyraT v1.2
 * - DS18B20 temp sensor on GPIO13
 * - MLX90640 thermal camera on I2C (GPIO18/23)
 * - Delta SM70-CP-450 power supply via TCP
 */

#include <WiFi.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <Preferences.h>

// MLX90640 library
#include <MLX90640_API.h>
#include <MLX90640_I2C_Driver.h>

const char* WIFI_SSID = "BTAC Medewerkers";
const char* WIFI_PASS = "Next3600$!";
const char* DELTA_IP = "192.168.1.27";
const int DELTA_PORT = 8462;

#define ONE_WIRE_BUS 13
#define PA_ENABLE_PIN   21
#define I2C_SDA_PIN     18
#define I2C_SCL_PIN     23
#define ES8311_ADDR     0x18
#define MLX90640_ADDR   0x33
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

// MLX90640 variables
#define MLX_COLS 32
#define MLX_ROWS 24
#define MLX_PIXELS (MLX_COLS * MLX_ROWS)
paramsMLX90640 mlxParams;
float mlxFrame[MLX_PIXELS];
float mlxMin = 0, mlxMax = 0, mlxAvg = 0;
bool mlxConnected = false;
unsigned long lastMlxRead = 0;
#define MLX_REFRESH_MS 500  // Read thermal every 500ms

enum TestMode { MODE_IDLE, MODE_CHARGE, MODE_DISCHARGE, MODE_CYCLE };

struct {
  float minVoltage = 2.71;
  float maxVoltage = 4.20;
  float chargeCurrent = 24.0;
  float dischargeCurrent = 24.0;
  float tempAlarm = 45.0;
  float thermalAlarm = 60.0;  // NEW: Thermal camera alarm
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
  int measureFailCount = 0;
  unsigned long lowCurrentStartTime = 0;
  bool lowCurrentActive = false;
} status;

// ============== MLX90640 THERMAL CAMERA ==============

bool mlxInit() {
  Serial.println("[MLX] Initializing...");

  // Check if MLX90640 is present
  Wire.beginTransmission(MLX90640_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[MLX] Not found on I2C");
    return false;
  }

  // Get device parameters
  uint16_t eeData[832];
  int status = MLX90640_DumpEE(MLX90640_ADDR, eeData);
  if (status != 0) {
    Serial.printf("[MLX] Failed to dump EE: %d\n", status);
    return false;
  }

  status = MLX90640_ExtractParameters(eeData, &mlxParams);
  if (status != 0) {
    Serial.printf("[MLX] Failed to extract params: %d\n", status);
    return false;
  }

  // Set refresh rate to 4Hz (good balance between speed and noise)
  MLX90640_SetRefreshRate(MLX90640_ADDR, 0x04);

  // Set chess pattern mode for better image quality
  MLX90640_SetChessMode(MLX90640_ADDR);

  Serial.println("[MLX] Initialized OK - 32x24 @ 4Hz");
  return true;
}

void mlxRead() {
  if (!mlxConnected) return;
  if (millis() - lastMlxRead < MLX_REFRESH_MS) return;
  lastMlxRead = millis();

  uint16_t frameData[834];
  int status = MLX90640_GetFrameData(MLX90640_ADDR, frameData);
  if (status < 0) {
    Serial.printf("[MLX] Frame error: %d\n", status);
    return;
  }

  float Ta = MLX90640_GetTa(frameData, &mlxParams);
  float tr = Ta - 8.0;  // Reflected temperature estimate
  float emissivity = 0.95;  // Good for most objects

  MLX90640_CalculateTo(frameData, &mlxParams, emissivity, tr, mlxFrame);

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
  deltaClient.stop();
  delay(100);

  deltaClient.setTimeout(3);
  if (!deltaClient.connect(DELTA_IP, DELTA_PORT)) {
    Serial.println("[DELTA] Connect FAILED");
    status.deltaConnected = false;
    return false;
  }

  status.deltaConnected = true;
  delay(100);
  while(deltaClient.available()) deltaClient.read();
  return true;
}

String deltaQuery(String cmd) {
  if (!deltaClient.connected()) {
    if (!deltaConnect()) return "";
  }

  while(deltaClient.available()) deltaClient.read();

  deltaClient.print(cmd + "\n");
  deltaClient.flush();

  String response = "";
  unsigned long start = millis();

  while ((millis() - start) < 1000) {
    if (deltaClient.available()) {
      char c = deltaClient.read();
      if (c >= 32 && c <= 126) response += c;
      start = millis();
    } else {
      delay(5);
    }
  }

  response.trim();
  return response;
}

void deltaSet(String cmd) {
  if (!deltaClient.connected()) {
    if (!deltaConnect()) return;
  }
  deltaClient.print(cmd + "\n");
  deltaClient.flush();
  delay(50);
}

bool deltaSetupCharge(float voltage, float current) {
  Serial.printf("[DELTA] Setup CHARGE: %.2fV %.1fA\n", voltage, current);

  if (!deltaConnect()) return false;

  deltaSet("OUTPut OFF");
  delay(200);

  deltaSet("SYSTem:REMote:CV Remote");
  deltaSet("SYSTem:REMote:CC Remote");
  deltaSet("SYSTem:REMote:CP Remote");

  deltaSet("SOURce:VOLtage " + String(voltage, 2));
  deltaSet("SOURce:CURrent " + String(current, 2));
  float power = voltage * current * 1.5;
  deltaSet("SOURce:POWer " + String(power, 0));

  deltaSet("SOURce:CURrent:NEGative 0");
  deltaSet("SOURce:POWer:NEGative 0");

  deltaSet("OUTPut ON");
  delay(200);

  String vSet = deltaQuery("SOURce:VOLtage?");
  String iSet = deltaQuery("SOURce:CURrent?");
  String pSet = deltaQuery("SOURce:POWer?");
  Serial.printf("[DELTA] Set: V=%s I=%s P=%s\n", vSet.c_str(), iSet.c_str(), pSet.c_str());

  return true;
}

bool deltaSetupDischarge(float voltage, float current) {
  Serial.printf("[DELTA] Setup DISCHARGE: %.2fV %.1fA\n", voltage, current);

  if (!deltaConnect()) return false;

  deltaSet("OUTPut OFF");
  delay(200);

  deltaSet("SYSTem:REMote:CV Remote");
  deltaSet("SYSTem:REMote:CC Remote");
  deltaSet("SYSTem:REMote:CP Remote");

  deltaSet("SOURce:VOLtage " + String(voltage, 2));
  deltaSet("SOURce:CURrent 0");
  deltaSet("SOURce:POWer 0");

  deltaSet("SOURce:CURrent:NEGative -" + String(current, 2));
  float power = 70.0 * current * 1.5;
  deltaSet("SOURce:POWer:NEGative -" + String(power, 0));

  deltaSet("OUTPut ON");
  delay(200);

  String vSet = deltaQuery("SOURce:VOLtage?");
  String iNeg = deltaQuery("SOURce:CURrent:NEGative?");
  String pNeg = deltaQuery("SOURce:POWer:NEGative?");
  Serial.printf("[DELTA] Set: V=%s I-=%s P-=%s\n", vSet.c_str(), iNeg.c_str(), pNeg.c_str());

  return true;
}

bool deltaReadMeasurements() {
  if (!deltaClient.connected()) {
    if (!deltaConnect()) {
      status.measureFailCount++;
      return false;
    }
  }

  String vStr = deltaQuery("MEASure:VOLtage?");
  String iStr = deltaQuery("MEASure:CURrent?");
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

  if (v >= 0 && v <= 100) {
    status.voltage = v;
    status.current = i;
    status.power = p;

    if (status.running && status.lastMeasureTime > 0) {
      float hours = (millis() - status.lastMeasureTime) / 3600000.0;
      status.totalWh += abs(p) * hours;
      if (i > 0.01) status.totalAhCharge += i * hours;
      else if (i < -0.01) status.totalAhDischarge += abs(i) * hours;
    }
    status.lastMeasureTime = millis();

    Serial.printf("[M] V=%.3f I=%.2f P=%.1f\n", v, i, p);
    return true;
  }

  return false;
}

void deltaStop() {
  Serial.println("[DELTA] STOP");

  if (deltaConnect()) {
    deltaSet("OUTPut OFF");
    delay(100);
  }

  deltaClient.stop();
  status.deltaConnected = false;
}

String deltaPing() {
  Serial.println("[DELTA] PING");

  if (!deltaConnect()) {
    beepFail();
    return "FAIL: Cannot connect";
  }

  String idn = deltaQuery("*IDN?");
  deltaClient.stop();

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
}

void startCharge(float voltage, float current) {
  if (status.running) return;

  Serial.printf("\n===== CHARGE %.2fV %.1fA =====\n", voltage, current);
  resetStats();

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
  deltaStop();
  beepStop();
}

void updateTest() {
  if (!status.running || stopRequested) {
    if (stopRequested) stopTest();
    return;
  }

  deltaReadMeasurements();
  readTemp();

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

  // Mode logic (same as before)
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
              Serial.printf("[CYCLE %d] Empty (2 min low current): %.3f Ah\n", status.currentCycle, status.totalAhDischarge);
              playTone(1000, 200);
              status.lowCurrentActive = false;
              status.lowCurrentStartTime = 0;

              if (status.currentCycle >= config.numCycles) {
                beepDone();
                stopTest();
              } else {
                status.currentCycle++;
                status.totalAhDischarge = 0;
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
    // Blue to Cyan
    float t = ratio / 0.25;
    r = 0; g = (int)(255 * t); b = 255;
  } else if (ratio < 0.5) {
    // Cyan to Green
    float t = (ratio - 0.25) / 0.25;
    r = 0; g = 255; b = (int)(255 * (1 - t));
  } else if (ratio < 0.75) {
    // Green to Yellow
    float t = (ratio - 0.5) / 0.25;
    r = (int)(255 * t); g = 255; b = 0;
  } else {
    // Yellow to Red
    float t = (ratio - 0.75) / 0.25;
    r = 255; g = (int)(255 * (1 - t)); b = 0;
  }

  char hex[8];
  sprintf(hex, "#%02X%02X%02X", r, g, b);
  return String(hex);
}

void sendMainPage() {
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Battery Tester v7.0</title><style>";
  h += "*{box-sizing:border-box;margin:0;padding:0}";
  h += "body{font-family:sans-serif;background:#1a1a2e;color:#eee;padding:10px;max-width:600px;margin:0 auto}";
  h += "h1{text-align:center;color:#0cf;margin-bottom:10px}";
  h += ".grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:10px}";
  h += ".card{background:#16213e;border-radius:6px;padding:10px;text-align:center}";
  h += ".card .l{font-size:0.7em;color:#888}.card .v{font-size:1.3em;font-weight:bold;color:#0cf}";
  h += ".panel{background:#16213e;border-radius:6px;padding:12px;margin-bottom:10px}";
  h += ".panel h2{color:#0cf;font-size:0.9em;margin-bottom:8px}";
  h += ".row{display:flex;justify-content:space-between;font-size:0.85em;padding:3px 0}.row .k{color:#888}";
  h += "button{padding:10px 15px;border:none;border-radius:6px;font-weight:bold;cursor:pointer;margin:3px}";
  h += ".bg{background:#0a4;color:#fff}.br{background:#d33;color:#fff}.bb{background:#08c;color:#fff}.bo{background:#c80;color:#fff}";
  h += "input[type=number]{width:70px;padding:8px;background:#0f0f23;border:1px solid #333;border-radius:4px;color:#fff;text-align:center}";
  h += ".st{text-align:center;padding:10px;border-radius:6px;margin-bottom:10px;font-weight:bold}";
  h += ".st.on{background:#0a4}.st.off{background:#555}.st.run{background:#c80}";
  h += ".err{background:#422;border:1px solid #d33;padding:8px;border-radius:5px;margin-bottom:10px}";
  h += ".mode-row{display:flex;align-items:center;gap:8px;margin-bottom:10px}";
  h += ".mode-row label{color:#888;font-size:0.85em;min-width:60px}";
  h += ".stop-btn{width:100%;padding:15px;font-size:1.2em}";
  h += ".thermal-container{text-align:center;margin:10px 0}";
  h += ".thermal-img{image-rendering:pixelated;border:2px solid #333;border-radius:4px}";
  h += ".thermal-stats{display:flex;justify-content:space-around;margin-top:8px}";
  h += ".thermal-stat{text-align:center}.thermal-stat .label{font-size:0.7em;color:#888}";
  h += ".thermal-stat .val{font-size:1.1em;font-weight:bold}";
  h += ".tmin{color:#00f}.tmax{color:#f00}.tavg{color:#0f0}";
  h += "</style></head><body>";
  h += "<h1>Battery Tester v7.0</h1>";
  h += "<div class='st' id='st'>Loading...</div>";
  h += "<div class='err' id='err' style='display:none'></div>";
  h += "<div class='panel'><button class='bb' onclick='ping()'>PING DELTA</button><span id='ds' style='margin-left:10px'>-</span></div>";
  h += "<div class='grid'>";
  h += "<div class='card'><div class='l'>Voltage</div><div class='v' id='v'>--</div></div>";
  h += "<div class='card'><div class='l'>Current</div><div class='v' id='i'>--</div></div>";
  h += "<div class='card'><div class='l'>Temp DS18B20</div><div class='v' id='t'>--</div></div>";
  h += "<div class='card'><div class='l'>Ah Charge</div><div class='v' id='ahc'>--</div></div>";
  h += "<div class='card'><div class='l'>Ah Discharge</div><div class='v' id='ahd'>--</div></div>";
  h += "<div class='card'><div class='l'>Wh</div><div class='v' id='wh'>--</div></div>";
  h += "</div>";

  // Thermal Camera Section
  h += "<div class='panel'><h2>Thermal Camera (MLX90640)</h2>";
  h += "<div id='mlxStatus'>Checking...</div>";
  h += "<div class='thermal-container' id='thermalContainer' style='display:none'>";
  h += "<canvas id='thermal' class='thermal-img' width='320' height='240'></canvas>";
  h += "<div class='thermal-stats'>";
  h += "<div class='thermal-stat'><div class='label'>MIN</div><div class='val tmin' id='tmin'>--</div></div>";
  h += "<div class='thermal-stat'><div class='label'>MAX</div><div class='val tmax' id='tmax'>--</div></div>";
  h += "<div class='thermal-stat'><div class='label'>AVG</div><div class='val tavg' id='tavg'>--</div></div>";
  h += "</div></div></div>";

  // Manual Charge
  h += "<div class='panel'><h2>Manual Charge</h2>";
  h += "<div class='mode-row'><label>Max V:</label><input type='number' id='chgV' step='0.01' value='4.20'>";
  h += "<label>Current:</label><input type='number' id='chgI' step='0.1' value='5'><span>A</span></div>";
  h += "<button class='bg' onclick='startChg()'>START CHARGE</button></div>";

  // Manual Discharge
  h += "<div class='panel'><h2>Manual Discharge</h2>";
  h += "<div class='mode-row'><label>Min V:</label><input type='number' id='disV' step='0.01' value='2.71'>";
  h += "<label>Current:</label><input type='number' id='disI' step='0.1' value='5'><span>A</span></div>";
  h += "<button class='bo' onclick='startDis()'>START DISCHARGE</button></div>";

  // Auto Cycle
  h += "<div class='panel'><h2>Auto Cycle</h2>";
  h += "<div class='row'><span class='k'>Range</span><span>" + String(config.minVoltage,2) + " - " + String(config.maxVoltage,2) + " V</span></div>";
  h += "<div class='row'><span class='k'>Current</span><span>" + String(config.chargeCurrent,1) + " / " + String(config.dischargeCurrent,1) + " A</span></div>";
  h += "<div class='row'><span class='k'>Cycles</span><span id='cyc'>0 / " + String(config.numCycles) + "</span></div>";
  h += "<button class='bg' onclick='startCyc()' style='margin-top:10px'>START CYCLE</button></div>";

  // Stop
  h += "<div class='panel'><button class='br stop-btn' onclick='stop()'>STOP</button></div>";

  // Links
  h += "<div class='panel'><button onclick='location.href=\"/settings\"'>Settings</button>";
  h += "<button onclick='location.href=\"/thermal\"' style='margin-left:10px'>Thermal Fullscreen</button></div>";

  // JavaScript
  h += "<script>";
  h += "function $(i){return document.getElementById(i)}";
  h += "var canvas=$('thermal'),ctx=canvas.getContext('2d');";
  h += "function upd(){fetch('/status').then(r=>r.json()).then(d=>{";
  h += "$('v').innerText=d.v.toFixed(3);";
  h += "$('i').innerText=d.i.toFixed(2);";
  h += "$('t').innerText=d.t.toFixed(1);";
  h += "$('ahc').innerText=d.ahc.toFixed(3);";
  h += "$('ahd').innerText=d.ahd.toFixed(3);";
  h += "$('wh').innerText=d.wh.toFixed(2);";
  h += "$('cyc').innerText=d.cyc+'/'+d.ncyc;";
  h += "$('ds').innerText=d.delta?'Connected':'Offline';";
  h += "$('ds').style.color=d.delta?'#0f0':'#f44';";
  h += "var modes=['IDLE','CHARGING','DISCHARGING','CYCLING'];";
  h += "var s=$('st');";
  h += "if(d.run){s.innerText=modes[d.mode];s.className='st run';}";
  h += "else{s.innerText=d.delta?'Ready':'Offline';s.className='st '+(d.delta?'on':'off');}";
  h += "var e=$('err');if(d.err){e.innerText=d.err;e.style.display='block';}else{e.style.display='none';}";
  h += "if(d.mlx){";
  h += "$('mlxStatus').style.display='none';$('thermalContainer').style.display='block';";
  h += "$('tmin').innerText=d.mlxMin.toFixed(1)+'C';";
  h += "$('tmax').innerText=d.mlxMax.toFixed(1)+'C';";
  h += "$('tavg').innerText=d.mlxAvg.toFixed(1)+'C';";
  h += "}else{$('mlxStatus').innerText='MLX90640 not connected';$('thermalContainer').style.display='none';}";
  h += "}).catch(()=>{$('st').innerText='ESP Offline';$('st').className='st off';})}";

  // Thermal image update
  h += "function updThermal(){if(!$('thermalContainer').style.display||$('thermalContainer').style.display=='none')return;";
  h += "fetch('/thermal').then(r=>r.json()).then(d=>{";
  h += "if(!d.data)return;var w=32,h=24,scale=10;";
  h += "for(var y=0;y<h;y++){for(var x=0;x<w;x++){";
  h += "ctx.fillStyle=d.colors[y*w+x];ctx.fillRect(x*scale,y*scale,scale,scale);}}";
  h += "}).catch(()=>{})}";

  h += "function ping(){fetch('/ping').then(r=>r.json()).then(d=>{alert(d.result);upd();})}";
  h += "function startChg(){fetch('/charge?v='+$('chgV').value+'&i='+$('chgI').value);setTimeout(upd,500)}";
  h += "function startDis(){fetch('/discharge?v='+$('disV').value+'&i='+$('disI').value);setTimeout(upd,500)}";
  h += "function startCyc(){fetch('/cycle');setTimeout(upd,500)}";
  h += "function stop(){fetch('/stop');setTimeout(upd,200)}";
  h += "setInterval(upd,2000);setInterval(updThermal,1000);upd();updThermal();";
  h += "</script></body></html>";

  server.send(200, "text/html", h);
}

void sendThermalPage() {
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
  h += "function upd(){fetch('/thermal').then(r=>r.json()).then(d=>{";
  h += "if(!d.data)return;var w=32,h=24,scale=20;";
  h += "for(var y=0;y<h;y++){for(var x=0;x<w;x++){";
  h += "ctx.fillStyle=d.colors[y*w+x];ctx.fillRect(x*scale,y*scale,scale,scale);}}";
  h += "document.getElementById('tmin').innerText=d.min.toFixed(1)+'C';";
  h += "document.getElementById('tmax').innerText=d.max.toFixed(1)+'C';";
  h += "document.getElementById('tavg').innerText=d.avg.toFixed(1)+'C';";
  h += "}).catch(()=>{})}";
  h += "setInterval(upd,500);upd();";
  h += "</script></body></html>";

  server.send(200, "text/html", h);
}

void sendSettingsPage() {
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Settings</title><style>";
  h += "*{box-sizing:border-box;margin:0;padding:0}";
  h += "body{font-family:sans-serif;background:#1a1a2e;color:#eee;padding:10px;max-width:500px;margin:0 auto}";
  h += "h1{text-align:center;color:#0cf;margin-bottom:10px}";
  h += ".panel{background:#16213e;border-radius:6px;padding:15px;margin-bottom:10px}";
  h += ".panel h2{color:#0cf;font-size:0.95em;margin-bottom:10px}";
  h += ".form-row{margin-bottom:15px}";
  h += ".form-row label{display:block;color:#888;margin-bottom:5px}";
  h += ".form-row input{width:100%;padding:12px;background:#0f0f23;border:1px solid #333;border-radius:4px;color:#fff}";
  h += "button{width:100%;padding:12px;border:none;border-radius:6px;font-weight:bold;cursor:pointer;margin-top:10px}";
  h += ".btn-save{background:#0a4;color:#fff}.btn-back{background:#666;color:#fff}";
  h += "</style></head><body>";
  h += "<h1>Settings</h1>";

  h += "<div class='panel'><h2>Voltage</h2>";
  h += "<div class='form-row'><label>Min V</label><input type='number' id='minV' step='0.01' value='" + String(config.minVoltage,2) + "'></div>";
  h += "<div class='form-row'><label>Max V</label><input type='number' id='maxV' step='0.01' value='" + String(config.maxVoltage,2) + "'></div></div>";

  h += "<div class='panel'><h2>Current</h2>";
  h += "<div class='form-row'><label>Charge A</label><input type='number' id='chgI' step='0.1' value='" + String(config.chargeCurrent,1) + "'></div>";
  h += "<div class='form-row'><label>Discharge A</label><input type='number' id='disI' step='0.1' value='" + String(config.dischargeCurrent,1) + "'></div></div>";

  h += "<div class='panel'><h2>Temperature Alarms</h2>";
  h += "<div class='form-row'><label>DS18B20 Alarm (C)</label><input type='number' id='temp' value='" + String(config.tempAlarm,0) + "'></div>";
  h += "<div class='form-row'><label>Thermal Camera Alarm (C)</label><input type='number' id='therm' value='" + String(config.thermalAlarm,0) + "'></div></div>";

  h += "<div class='panel'><h2>Test</h2>";
  h += "<div class='form-row'><label>Cycles</label><input type='number' id='cyc' value='" + String(config.numCycles) + "'></div>";
  h += "<div class='form-row'><label>Log Interval (s)</label><input type='number' id='log' value='" + String(config.logInterval) + "'></div></div>";

  h += "<div class='panel'>";
  h += "<button class='btn-save' onclick='save()'>SAVE</button>";
  h += "<button class='btn-back' onclick='location.href=\"/\"'>BACK</button>";
  h += "</div>";

  h += "<script>";
  h += "function $(i){return document.getElementById(i)}";
  h += "function save(){";
  h += "var p='minV='+$('minV').value+'&maxV='+$('maxV').value+'&chgI='+$('chgI').value+'&disI='+$('disI').value+'&cyc='+$('cyc').value+'&log='+$('log').value+'&temp='+$('temp').value+'&therm='+$('therm').value;";
  h += "fetch('/save?'+p).then(()=>location.href='/');}";
  h += "</script></body></html>";

  server.send(200, "text/html", h);
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
  j += ",\"delta\":" + String(status.deltaConnected ? "true" : "false");
  j += ",\"mlx\":" + String(mlxConnected ? "true" : "false");
  j += ",\"mlxMin\":" + String(mlxMin, 1);
  j += ",\"mlxMax\":" + String(mlxMax, 1);
  j += ",\"mlxAvg\":" + String(mlxAvg, 1);
  j += ",\"err\":\"" + status.lastError + "\"}";
  server.send(200, "application/json", j);
}

void handleThermal() {
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
  String r = deltaPing();
  server.send(200, "application/json", "{\"result\":\"" + r + "\"}");
}

void handleCharge() {
  float v = server.hasArg("v") ? server.arg("v").toFloat() : config.maxVoltage;
  float i = server.hasArg("i") ? server.arg("i").toFloat() : config.chargeCurrent;
  startCharge(v, i);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleDischarge() {
  float v = server.hasArg("v") ? server.arg("v").toFloat() : config.minVoltage;
  float i = server.hasArg("i") ? server.arg("i").toFloat() : config.dischargeCurrent;
  startDischarge(v, i);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleCycle() {
  startCycle();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStop() {
  stopRequested = true;
  status.running = false;
  deltaStop();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSave() {
  if (server.hasArg("minV")) config.minVoltage = server.arg("minV").toFloat();
  if (server.hasArg("maxV")) config.maxVoltage = server.arg("maxV").toFloat();
  if (server.hasArg("chgI")) config.chargeCurrent = server.arg("chgI").toFloat();
  if (server.hasArg("disI")) config.dischargeCurrent = server.arg("disI").toFloat();
  if (server.hasArg("cyc")) config.numCycles = server.arg("cyc").toInt();
  if (server.hasArg("log")) config.logInterval = server.arg("log").toInt();
  if (server.hasArg("temp")) config.tempAlarm = server.arg("temp").toFloat();
  if (server.hasArg("therm")) config.thermalAlarm = server.arg("therm").toFloat();
  saveConfig();
  server.send(200, "application/json", "{\"ok\":true}");
}

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========== Battery Tester v7.0 + Thermal ==========\n");

  loadConfig();

  pinMode(PA_ENABLE_PIN, OUTPUT);
  digitalWrite(PA_ENABLE_PIN, HIGH);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);  // 400kHz for MLX90640

  es8311_init();
  i2s_init();

  // Initialize MLX90640
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
  server.on("/status", handleStatus);
  server.on("/thermal", HTTP_GET, handleThermal);
  server.on("/ping", handlePing);
  server.on("/charge", handleCharge);
  server.on("/discharge", handleDischarge);
  server.on("/cycle", handleCycle);
  server.on("/stop", handleStop);
  server.on("/save", handleSave);
  server.begin();

  readTemp();
  playTone(800, 100); playSilence(50); playTone(1000, 100); playSilence(50); playTone(1200, 150);

  Serial.printf("\nReady: http://%s\n\n", WiFi.localIP().toString().c_str());
}

// ============== LOOP ==============
void loop() {
  server.handleClient();

  // Read thermal camera
  if (mlxConnected) {
    mlxRead();
  }

  static unsigned long lastUpd = 0;
  if (millis() - lastUpd >= 1000) {
    lastUpd = millis();
    if (status.running) updateTest();
  }

  static unsigned long lastTemp = 0;
  if (millis() - lastTemp >= 5000) {
    lastTemp = millis();
    if (!status.running) readTemp();
  }

  delay(10);
  yield();
}
