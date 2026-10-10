#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
#include <math.h>

// ====== SETTINGS ======
const char *TB_HOST = "";
const int TB_PORT = ;
const char *TB_TOKEN = "";
const char *TB_CLIENT_ID = "ESP8266_RIVER";
const char *TB_TOPIC = "v1/devices/me/telemetry";
const char *APN = "";
const unsigned long TELEMETRY_MS = 10000;

// Turbidity calibration
const int TURB_ADC_DIRTY = 378;
const int TURB_ADC_CLEAR = 545;
const float TURB_NTU_MAX = 3000.0f;

// ====== THRESHOLDS (prototype) ======
const float ACCEL_MAG_WARN_LO = 9.5f;
const float ACCEL_MAG_WARN_HI = 11.5f;
const float ACCEL_MAG_CRIT_LO = 8.0f;
const float ACCEL_MAG_CRIT_HI = 13.0f;
const float ACCEL_AXIS_WARN = 10.0f;
const float ACCEL_AXIS_CRIT = 20.0f;
const uint8_t ACCEL_ABNORMAL_PERSIST = 3;
const float TEMP_WARN_C = 35.0f;
const float TEMP_CRIT_C = 40.0f;
const float CLARITY_WARN_PCT = 50.0f;
const float CLARITY_CRIT_PCT = 25.0f;
const float ALT_WARN_M = 2.0f;
const float ALT_CRIT_M = 5.0f;
const uint8_t BASELINE_SAMPLES = 5;

// Pins
static const uint8_t PIN_SDA = D2;
static const uint8_t PIN_SCL = D1;
static const uint8_t PIN_TURBIDITY = A0;
static const uint8_t PIN_MODEM_RX = D5;
static const uint8_t PIN_MODEM_TX = D6;
static const uint8_t PIN_GPS_RX = D7;
static const uint8_t PIN_GPS_TX = D8;

SoftwareSerial modem(PIN_MODEM_RX, PIN_MODEM_TX);
SoftwareSerial gpsSerial(PIN_GPS_RX, PIN_GPS_TX);
TinyGPSPlus gps;
Adafruit_ADXL345_Unified accel(12345);
Adafruit_BMP280 bmp;

bool accelOk = false;
bool bmpOk = false;
bool modemReady = false;
bool mqttReady = false;
bool gpsFix = false;
uint32_t lastTelemetryMs = 0;

double gpsLat = 0, gpsLng = 0, gpsAlt = 0, gpsSpeed = 0;
uint32_t gpsSatellites = 0, gpsHdop = 0;

bool baroBaselineSet = false, gpsBaselineSet = false;
float baroAltBaseline = 0, gpsAltBaseline = 0;
uint8_t baroBaseCount = 0, gpsBaseCount = 0;
float baroBaseSum = 0, gpsBaseSum = 0;

uint8_t accelAbnormalCount = 0;
bool accelAbnormal = false, accelAbnormalPersistent = false;
uint8_t alertLevel = 0;
char alertReason[96];

void clearModem() {
  while (modem.available()) modem.read();
}

String modemRead(uint32_t ms) {
  String out;
  uint32_t start = millis(), lastByte = start;
  while (millis() - start < ms) {
    while (modem.available()) {
      char c = (char)modem.read();
      out += c;
      Serial.write(c);
      lastByte = millis();
    }
    if (out.length() > 0 && (millis() - lastByte) > 250) {
      if (out.indexOf("OK") >= 0 || out.indexOf("ERROR") >= 0 ||
          out.indexOf("+QMT") >= 0 || out.indexOf("+QI") >= 0) break;
    }
    delay(5);
  }
  return out;
}

String atCmd(const String &cmd, uint32_t waitMs = 1500) {
  modem.listen();
  Serial.println();
  Serial.print(F("AT>> "));
  Serial.println(cmd);
  clearModem();
  delay(30);
  modem.print(cmd);
  modem.print("\r\n");
  return modemRead(waitMs);
}

bool waitForPrompt(uint32_t timeoutMs) {
  modem.listen();
  uint32_t start = millis();
  String buf;
  while (millis() - start < timeoutMs) {
    while (modem.available()) {
      char c = (char)modem.read();
      Serial.write(c);
      buf += c;
      if (c == '>') return true;
      if (buf.indexOf("ERROR") >= 0) return false;
    }
    delay(5);
  }
  return false;
}

void serviceGps(uint32_t ms) {
  gpsSerial.listen();
  uint32_t start = millis();
  while (millis() - start < ms) {
    while (gpsSerial.available()) gps.encode((char)gpsSerial.read());
    delay(1);
  }
  if (gps.location.isValid()) {
    gpsFix = true;
    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
  }
  if (gps.altitude.isValid()) gpsAlt = gps.altitude.meters();
  if (gps.speed.isValid()) gpsSpeed = gps.speed.kmph();
  if (gps.satellites.isValid()) gpsSatellites = gps.satellites.value();
  if (gps.hdop.isValid()) gpsHdop = gps.hdop.value();
}

bool pdpActive() {
  String r = atCmd("AT+QIACT?", 3000);
  return r.indexOf("+QIACT: 1,1") >= 0 || r.indexOf("+QIACT:1,1") >= 0;
}

bool activatePdp() {
  if (pdpActive()) {
    Serial.println(F("PDP already active"));
    return true;
  }
  atCmd("AT+CGATT=1", 4000);
  atCmd(String("AT+QICSGP=1,1,\"") + APN + "\",\"\",\"\",1", 2000);
  atCmd("AT+QIDEACT=1", 5000);
  delay(500);
  atCmd("AT+QIACT=1", 15000);
  delay(300);
  bool ok = pdpActive();
  Serial.println(ok ? F("PDP OK") : F("PDP FAILED"));
  return ok;
}

bool modemInit() {
  Serial.println(F("Modem init..."));
  delay(2000);
  modem.listen();
  modem.begin(115200);
  delay(500);
  bool linked = false;
  for (int i = 0; i < 5; i++) {
    if (atCmd("AT", 1000).indexOf("OK") >= 0) { linked = true; break; }
    delay(300);
  }
  if (!linked) {
    modem.begin(9600);
    delay(500);
    for (int i = 0; i < 5; i++) {
      if (atCmd("AT", 1000).indexOf("OK") >= 0) { linked = true; break; }
      delay(300);
    }
  }
  if (!linked) return false;
  atCmd("ATE0", 500);
  atCmd("AT+CMEE=2", 500);
  atCmd("AT+CPIN?", 1500);
  atCmd("AT+CSQ", 1500);
  atCmd("AT+CREG?", 1500);
  return activatePdp();
}

bool mqttConnect() {
  Serial.println(F("MQTT connect..."));
  atCmd("AT+QMTCFG=\"version\",0,4", 1500);
  atCmd("AT+QMTCFG=\"pdpcid\",0,1", 1500);
  atCmd("AT+QMTCFG=\"keepalive\",0,60", 1500);
  atCmd("AT+QMTCFG=\"session\",0,1", 1500);
  atCmd("AT+QMTDISC=0", 3000);
  delay(300);
  atCmd("AT+QMTCLOSE=0", 3000);
  delay(500);

  String openCmd = String("AT+QMTOPEN=0,\"") + TB_HOST + "\"," + String(TB_PORT);
  String openResp = atCmd(openCmd, 20000);
  bool openOk = openResp.indexOf("+QMTOPEN: 0,0") >= 0 ||
                openResp.indexOf("+QMTOPEN:0,0") >= 0 ||
                openResp.indexOf("+QMTOPEN: 0,2") >= 0;
  if (!openOk) {
    delay(1500);
    openResp += modemRead(8000);
    openOk = openResp.indexOf("+QMTOPEN: 0,0") >= 0 ||
             openResp.indexOf("+QMTOPEN:0,0") >= 0 ||
             openResp.indexOf("+QMTOPEN: 0,2") >= 0;
  }
  if (!openOk) {
    mqttReady = false;
    Serial.println(F("QMTOPEN failed"));
    return false;
  }

  delay(400);
  String connCmd =
      String("AT+QMTCONN=0,\"") + TB_CLIENT_ID + "\",\"" + TB_TOKEN + "\",\"\"";
  String connResp = atCmd(connCmd, 20000);
  bool connOk = connResp.indexOf("+QMTCONN: 0,0,0") >= 0 ||
                connResp.indexOf("+QMTCONN:0,0,0") >= 0;
  if (!connOk) {
    delay(1500);
    connResp += modemRead(8000);
    connOk = connResp.indexOf("+QMTCONN: 0,0,0") >= 0 ||
             connResp.indexOf("+QMTCONN:0,0,0") >= 0;
  }
  mqttReady = connOk;
  Serial.println(mqttReady ? F("MQTT CONNECTED") : F("MQTT CONNECT FAILED"));
  return mqttReady;
}

bool ensureMqtt() {
  if (!pdpActive() && !activatePdp()) return false;
  String st = atCmd("AT+QMTCONN?", 3000);
  if (st.indexOf("+QMTCONN: 0,3") >= 0 || st.indexOf("+QMTCONN:0,3") >= 0) {
    mqttReady = true;
    return true;
  }
  mqttReady = false;
  return mqttConnect();
}

bool mqttPublish(const String &payload) {
  if (!ensureMqtt()) {
    Serial.println(F("Publish skipped: MQTT not ready"));
    return false;
  }
  clearModem();
  delay(50);
  String pubCmd = String("AT+QMTPUBEX=0,0,0,0,\"") + TB_TOPIC + "\"," +
                  String(payload.length());
  Serial.println();
  Serial.print(F("AT>> "));
  Serial.println(pubCmd);
  modem.listen();
  modem.print(pubCmd);
  modem.print("\r\n");

  if (!waitForPrompt(8000)) {
    Serial.println(F("No '>' - reconnect"));
    modemRead(500);
    mqttReady = false;
    if (!ensureMqtt()) return false;
    clearModem();
    delay(50);
    modem.print(pubCmd);
    modem.print("\r\n");
    if (!waitForPrompt(8000)) {
      mqttReady = false;
      return false;
    }
  }
  modem.print(payload);
  String pubResp = modemRead(12000);
  bool ok = pubResp.indexOf("+QMTPUBEX: 0,0,0") >= 0 ||
            pubResp.indexOf("+QMTPUBEX:0,0,0") >= 0;
  Serial.println(ok ? F("Publish OK") : F("Publish ERROR"));
  mqttReady = ok;
  return ok;
}

float ntuFromAdc(int raw) {
  float span = (float)(TURB_ADC_CLEAR - TURB_ADC_DIRTY);
  if (span < 1.0f) span = 1.0f;
  float ntu = TURB_NTU_MAX * (float)(TURB_ADC_CLEAR - raw) / span;
  if (ntu < 0.0f) ntu = 0.0f;
  if (ntu > TURB_NTU_MAX) ntu = TURB_NTU_MAX;
  return ntu;
}

float clarityFromNtu(float ntu) {
  float c = 100.0f * (1.0f - (ntu / TURB_NTU_MAX));
  if (c < 0.0f) c = 0.0f;
  if (c > 100.0f) c = 100.0f;
  return c;
}

void updateBaselines(float baroAlt) {
  if (!baroBaselineSet && bmpOk && baroAlt > 0.0f) {
    baroBaseSum += baroAlt;
    baroBaseCount++;
    if (baroBaseCount >= BASELINE_SAMPLES) {
      baroAltBaseline = baroBaseSum / (float)baroBaseCount;
      baroBaselineSet = true;
      Serial.print(F("Baro baseline: "));
      Serial.println(baroAltBaseline, 2);
    }
  }
  if (!gpsBaselineSet && gpsFix && gps.altitude.isValid()) {
    gpsBaseSum += (float)gpsAlt;
    gpsBaseCount++;
    if (gpsBaseCount >= BASELINE_SAMPLES) {
      gpsAltBaseline = gpsBaseSum / (float)gpsBaseCount;
      gpsBaselineSet = true;
      Serial.print(F("GPS alt baseline: "));
      Serial.println(gpsAltBaseline, 2);
    }
  }
}

void evaluateThresholds(float ax, float ay, float az, float mag, float tempC,
                        float baroAlt, float clarity) {
  alertLevel = 0;
  alertReason[0] = '\0';

  auto raise = [](uint8_t level, const char *reason) {
    if (level > alertLevel) {
      alertLevel = level;
      strncpy(alertReason, reason, sizeof(alertReason) - 1);
      alertReason[sizeof(alertReason) - 1] = '\0';
    }
  };

  if (accelOk) {
    float absx = fabsf(ax), absy = fabsf(ay), absz = fabsf(az);
    if (absx > ACCEL_AXIS_CRIT || absy > ACCEL_AXIS_CRIT || absz > ACCEL_AXIS_CRIT)
      raise(2, "accel_axis_critical");
    else if (absx > ACCEL_AXIS_WARN || absy > ACCEL_AXIS_WARN || absz > ACCEL_AXIS_WARN)
      raise(1, "accel_axis_warning");

    if (mag < ACCEL_MAG_CRIT_LO || mag > ACCEL_MAG_CRIT_HI) {
      raise(2, "accel_magnitude_critical");
      accelAbnormal = true;
    } else if (mag < ACCEL_MAG_WARN_LO || mag > ACCEL_MAG_WARN_HI) {
      raise(1, "accel_magnitude_warning");
      accelAbnormal = true;
    } else {
      accelAbnormal = false;
    }

    if (accelAbnormal) {
      if (accelAbnormalCount < 255) accelAbnormalCount++;
    } else {
      accelAbnormalCount = 0;
    }
    accelAbnormalPersistent = (accelAbnormalCount >= ACCEL_ABNORMAL_PERSIST);
    if (accelAbnormalPersistent) raise(2, "accel_abnormal_persistent");
    else if (accelAbnormal) raise(1, "accel_abnormal");
  }

  if (bmpOk) {
    if (tempC > TEMP_CRIT_C) raise(2, "temp_critical");
    else if (tempC > TEMP_WARN_C) raise(1, "temp_warning");
  }

  if (clarity < CLARITY_CRIT_PCT) raise(2, "clarity_critical");
  else if (clarity < CLARITY_WARN_PCT) raise(1, "clarity_warning");

  if (bmpOk && baroBaselineSet) {
    float d = fabsf(baroAlt - baroAltBaseline);
    if (d > ALT_CRIT_M) raise(2, "baro_altitude_critical");
    else if (d > ALT_WARN_M) raise(1, "baro_altitude_warning");
  }

  if (gpsFix && gpsBaselineSet) {
    float d = fabsf((float)gpsAlt - gpsAltBaseline);
    if (d > ALT_CRIT_M) raise(2, "gps_altitude_critical");
    else if (d > ALT_WARN_M) raise(1, "gps_altitude_warning");
  }
}

void readSensors(float &ax, float &ay, float &az, float &mag, float &tempC,
                 float &hPa, float &altM, int &turbRaw, float &turbV,
                 float &ntu, float &clarity) {
  ax = ay = az = mag = 0;
  tempC = hPa = altM = 0;
  ntu = clarity = 0;

  if (accelOk) {
    sensors_event_t e;
    accel.getEvent(&e);
    ax = e.acceleration.x;
    ay = e.acceleration.y;
    az = e.acceleration.z;
    mag = sqrtf(ax * ax + ay * ay + az * az);
  }
  if (bmpOk) {
    tempC = bmp.readTemperature();
    hPa = bmp.readPressure() / 100.0F;
    altM = bmp.readAltitude(1013.25);
  }

  long sum = 0;
  for (int i = 0; i < 25; i++) {
    sum += analogRead(PIN_TURBIDITY);
    delay(4);
  }
  turbRaw = (int)(sum / 25);
  turbV = (turbRaw / 1023.0f) * 3.3f;
  ntu = ntuFromAdc(turbRaw);
  clarity = clarityFromNtu(ntu);
}

String buildJson(float ax, float ay, float az, float mag, float tempC,
                 float hPa, float altM, int turbRaw, float turbV, float ntu,
                 float clarity) {
  JsonDocument doc;

  if (accelOk) {
    doc["accel_x"] = serialized(String(ax, 2));
    doc["accel_y"] = serialized(String(ay, 2));
    doc["accel_z"] = serialized(String(az, 2));
    doc["accel_magnitude"] = serialized(String(mag, 2));
    doc["accel_abnormal"] = accelAbnormal;
    doc["accel_abnormal_persistent"] = accelAbnormalPersistent;
  }
  if (bmpOk) {
    doc["temp"] = serialized(String(tempC, 2));
    doc["pressure_hpa"] = serialized(String(hPa, 2));
    doc["baro_altitude"] = serialized(String(altM, 2));
    if (baroBaselineSet) {
      doc["baro_altitude_baseline"] = serialized(String(baroAltBaseline, 2));
      doc["baro_altitude_delta"] = serialized(String(altM - baroAltBaseline, 2));
    }
  }

  doc["turbidity_raw"] = turbRaw;
  doc["turbidity_v"] = serialized(String(turbV, 3));
  doc["turbidity_ntu"] = serialized(String(ntu, 1));
  doc["clarity_percent"] = serialized(String(clarity, 1));
  doc["water_clarity"] = serialized(String(clarity, 1));

  doc["gps_fix"] = gpsFix;
  if (gpsFix) {
    doc["latitude"] = serialized(String(gpsLat, 6));
    doc["longitude"] = serialized(String(gpsLng, 6));
    doc["gps_altitude"] = serialized(String(gpsAlt, 1));
    doc["gps_altitude_m"] = serialized(String(gpsAlt, 1));
    doc["gps_speed_kmh"] = serialized(String(gpsSpeed, 1));
    if (gpsBaselineSet) {
      doc["gps_altitude_baseline"] = serialized(String(gpsAltBaseline, 2));
      doc["gps_altitude_delta"] =
          serialized(String((float)gpsAlt - gpsAltBaseline, 2));
    }
  }
  doc["gps_satellites"] = gpsSatellites;
  doc["gps_hdop"] = gpsHdop;

  doc["alert_level"] = alertLevel;
  doc["alert_status"] = (alertLevel == 0) ? "OK" : (alertLevel == 1) ? "WARNING" : "CRITICAL";
  if (alertReason[0]) doc["alert_reason"] = alertReason;

  doc["th_temp_warn"] = TEMP_WARN_C;
  doc["th_temp_crit"] = TEMP_CRIT_C;
  doc["th_clarity_warn"] = CLARITY_WARN_PCT;
  doc["th_clarity_crit"] = CLARITY_CRIT_PCT;
  doc["th_alt_warn_m"] = ALT_WARN_M;
  doc["th_alt_crit_m"] = ALT_CRIT_M;

  String payload;
  serializeJson(doc, payload);
  return payload;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("River monitor + thresholds + GPS + MQTT"));

  Wire.begin(PIN_SDA, PIN_SCL);
  delay(50);

  gpsSerial.begin(9600);
  Serial.println(F("GPS on D7 @ 9600"));

  accelOk = accel.begin();
  if (accelOk) {
    accel.setRange(ADXL345_RANGE_4_G);
    Serial.println(F("ADXL345 OK"));
  } else Serial.println(F("ADXL345 FAIL"));

  bmpOk = bmp.begin(0x76) || bmp.begin(0x77);
  if (bmpOk) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println(F("BMP280 OK"));
  } else Serial.println(F("BMP280 FAIL"));

  pinMode(PIN_TURBIDITY, INPUT);

  modemReady = modemInit();
  if (modemReady) mqttReady = mqttConnect();
  else Serial.println(F("Modem FAIL"));

  lastTelemetryMs = millis() - TELEMETRY_MS;
}

void loop() {
  serviceGps(800);

  float ax, ay, az, mag, tempC, hPa, altM, turbV, ntu, clarity;
  int turbRaw;
  readSensors(ax, ay, az, mag, tempC, hPa, altM, turbRaw, turbV, ntu, clarity);
  updateBaselines(altM);
  evaluateThresholds(ax, ay, az, mag, tempC, altM, clarity);
  String json = buildJson(ax, ay, az, mag, tempC, hPa, altM, turbRaw, turbV, ntu, clarity);

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 2000) {
    lastPrint = millis();
    Serial.print(F("ALERT="));
    Serial.print(alertLevel == 0 ? F("OK") : alertLevel == 1 ? F("WARNING") : F("CRITICAL"));
    if (alertReason[0]) {
      Serial.print(F(" reason="));
      Serial.print(alertReason);
    }
    Serial.println();
    Serial.print(F("JSON: "));
    Serial.println(json);
  }

  if (modemReady && (millis() - lastTelemetryMs >= TELEMETRY_MS)) {
    lastTelemetryMs = millis();
    mqttPublish(json);
  }
  delay(20);
}
