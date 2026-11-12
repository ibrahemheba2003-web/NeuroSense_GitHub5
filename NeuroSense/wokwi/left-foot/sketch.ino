// =====================================================================
// NEUROSENSE - SMART SOCK (research prototype, NOT a medical device)
// Wokwi simulation. One file for each foot: change FOOT_SIDE only.
// =====================================================================

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TM1637Display.h>
#include <math.h>

// ---------------------------------------------------------------------
// WHICH FOOT IS THIS?   "RIGHT" in the right-foot project,
//                       "LEFT"  in the left-foot project.
// ---------------------------------------------------------------------
const char* FOOT_SIDE  = "LEFT";
const char* OTHER_SIDE = "RIGHT";

// DEMO_MODE true  = pressure window 60 s (for the video)
// DEMO_MODE false = pressure window 15 min (real rule)
#define DEMO_MODE true

// ---------------------------------------------------------------------
// CLOUD (Firebase Realtime Database)
// ---------------------------------------------------------------------
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";
const char* FIREBASE_URL = "https://neurosense-ca40a-default-rtdb.firebaseio.com/feet/";
const unsigned long cloudInterval = 5000;   // upload + read other foot every 5 s

// ---------------------------------------------------------------------
// PINS (same as the Wokwi diagram)
// ---------------------------------------------------------------------
// Main pressure layer: 3 sensors (pot1, pot2, pot3)
const int pressureHeelPin      = 34;   // heel
const int pressureForefoot1Pin = 35;   // inner forefoot (big toe joint)
const int pressureForefoot2Pin = 32;   // outer forefoot (little toe joint)

// Flexible pressure layer: 6 points (pot4..pot9) through CD74HC4067
const int muxSIGPin = 33;
const int muxS0Pin = 13;
const int muxS1Pin = 14;
const int muxS2Pin = 16;
const int muxS3Pin = 17;

const char* layerPoints[6] = {
  "In-Heel", "Out-Heel", "In-Mid", "Out-Mid", "In-Fore", "Out-Fore"
};

// Temperature (DS18B20) on GPIO4
const int tempPin = 4;
OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);

// Humidity (DHT22) on GPIO5
const int dhtPin = 5;
#define DHTTYPE DHT22
DHT dht(dhtPin, DHTTYPE);

// Alerts
const int buzzerPin = 18;
const int hapticPin = 19;   // yellow LED = stand-in for calf vibration motor

// RGB status LED
const int redPin = 25;
const int greenPin = 26;
const int bluePin = 27;

// OLED (I2C 21/22)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Motion (MPU6050, I2C 21/22)
Adafruit_MPU6050 mpu;

// Battery: pot10 on GPIO36 (VP) shown on TM1637
const int BATTERY_CLK = 23;
const int BATTERY_DIO = 15;
const int BATTERY_PIN = 36;
TM1637Display batteryDisplay(BATTERY_CLK, BATTERY_DIO);

// ---------------------------------------------------------------------
// THRESHOLDS
// ---------------------------------------------------------------------
// Simulation calibration: full pot (ADC 4095) = 158 kPa,
// about the top of the FSR 402 range (20 N over a 12.7 mm circle).
// NOT clinically calibrated.
const float simulatedMaxPressureKPa = 158.0;

// Sustained pressure rule (Najafi et al. 2017, about 35 mmHg)
const float pressureThresholdKPa = 4.7;
const unsigned long pressureSampleMs = 1000;            // 1 sample per second
const int WINDOW_SAMPLES = DEMO_MODE ? 60 : 900;        // 60 s demo / 15 min real
const float MEDIUM_FRACTION = 0.35;                     // team design choice
const float HIGH_FRACTION   = 0.95;                     // Najafi et al. 2017

// Flexible pressure layer (team simulation values, not medical thresholds)
const int layerStartADC = 1500;          // point must be at least this high
const int layerDiffADC = 900;            // and this much above the other 5
const int layerHighADC = 2800;           // very high point
const unsigned long layerAlertMs = 4000; // must last 4 s -> alert
const unsigned long layerDangerMs = 8000;// must last 8 s (or be very high) -> danger

// Temperature: difference between the two feet (IWGDF 2023: > 2.2 C)
// IWGDF confirms on two consecutive days; the simulation uses 3 readings.
const float temperatureDifferenceLimit = 2.2;
const int requiredTemperatureReadings = 3;

// Valid skin temperature range. Outside this range the reading is
// treated as a sensor error and is not used (a foot cannot be 70 C).
const float skinTempMin = 10.0;
const float skinTempMax = 45.0;

// Humidity: experimental only (no medical standard) -> warning, never danger
const float humidityWarning = 60.0;

// Motion
const float motionThreshold = 2.0;

// ---------------------------------------------------------------------
// VALUES
// ---------------------------------------------------------------------
int pressureHeelRaw = 0, pressureFore1Raw = 0, pressureFore2Raw = 0;
float pressureHeelKPa = 0, pressureFore1KPa = 0, pressureFore2KPa = 0;
float highestBasicPressureKPa = 0;

uint8_t pressureWindow[900];
int windowIndex = 0;
int windowCount = 0;
int windowAbove = 0;
float aboveFraction = 0;
int pressureLevel = 0;          // 0 OK, 1 MEDIUM, 2 HIGH

int flexPressureReading[6] = {0, 0, 0, 0, 0, 0};
int highestFlexPressure = 0;
int highestFlexPoint = -1;
int flexPressureDifference = 0;
bool flexPressureCandidate = false;
unsigned long flexPressureSince = 0;
int flexPressureLevel = 0;             // 0 OK, 1 ALERT, 2 DANGER

float footTemp = 0;
bool tempValid = false;
bool tempOutOfRange = false;
float otherFootTemp = 0;
bool otherTempValid = false;
long otherSeq = -1;
unsigned long otherSeqChangedAt = 0;
float temperatureDifference = 0;
int temperatureDifferenceCount = 0;
bool tempAlert = false;

float humidity = 0;
bool humidityValid = false;
bool humidityAlert = false;

float accelerationMagnitude = 0;
bool isMoving = false;
bool motionValid = false;

int batteryRaw = 0;
int batteryPercent = 0;

bool oledReady = false;
bool mpuReady = false;

int alertCount = 0;
bool dangerMode = false;
bool warningMode = false;

bool wifiReady = false;
int lastPutCode = 0;
int lastGetCode = 0;
long uploadSeq = 0;

// ---------------------------------------------------------------------
// TIMERS
// ---------------------------------------------------------------------
unsigned long lastSensorRead = 0, lastLayerRead = 0, lastPressureSample = 0;
unsigned long lastDisplayChange = 0, lastSerialPrint = 0, lastCloud = 0;
unsigned long lastBatteryRead = 0, lastTempRequest = 0;
const unsigned long sensorInterval = 2000;
const unsigned long layerInterval = 200;
const unsigned long displayInterval = 3000;
const unsigned long serialInterval = 2000;
const unsigned long batteryInterval = 200;
const unsigned long tempConversionTime = 750;
int currentPage = 0;

// ---------------------------------------------------------------------
// HELPERS
// ---------------------------------------------------------------------
float convertToKPa(int rawValue) {
  rawValue = constrain(rawValue, 0, 4095);
  return (rawValue / 4095.0) * simulatedMaxPressureKPa;
}

const char* levelText(int level) {
  if (level == 2) return "HIGH";
  if (level == 1) return "MEDIUM";
  return "OK";
}

const char* statusText() {
  if (dangerMode) return "DANGER";
  if (warningMode) return "WARNING";
  return "NORMAL";
}

// ---------------------------------------------------------------------
// BATTERY (pot10 -> TM1637)
// ---------------------------------------------------------------------
void updateBattery() {
  if (millis() - lastBatteryRead < batteryInterval) return;
  lastBatteryRead = millis();
  batteryRaw = analogRead(BATTERY_PIN);
  batteryPercent = constrain(map(batteryRaw, 0, 4095, 0, 100), 0, 100);
  batteryDisplay.showNumberDec(batteryPercent, false, 4, 0);
}

// ---------------------------------------------------------------------
// MULTIPLEXER
// ---------------------------------------------------------------------
int readMuxChannel(int channel) {
  digitalWrite(muxS0Pin, bitRead(channel, 0));
  digitalWrite(muxS1Pin, bitRead(channel, 1));
  digitalWrite(muxS2Pin, bitRead(channel, 2));
  digitalWrite(muxS3Pin, bitRead(channel, 3));
  delayMicroseconds(1000);
  analogRead(muxSIGPin);           // discard first reading after switching
  return analogRead(muxSIGPin);
}

// ---------------------------------------------------------------------
// FLEXIBLE PRESSURE LAYER: detects unusual localized pressure patterns
// ---------------------------------------------------------------------
void updateFlexPressureLayer() {
  if (millis() - lastLayerRead < layerInterval) return;
  lastLayerRead = millis();

  for (int i = 0; i < 6; i++) flexPressureReading[i] = readMuxChannel(i);

  highestFlexPressure = 0;
  highestFlexPoint = -1;
  for (int i = 0; i < 6; i++) {
    if (flexPressureReading[i] > highestFlexPressure) {
      highestFlexPressure = flexPressureReading[i];
      highestFlexPoint = i;
    }
  }

  int otherTotal = 0;
  for (int i = 0; i < 6; i++) {
    if (i != highestFlexPoint) otherTotal += flexPressureReading[i];
  }
  flexPressureDifference = highestFlexPressure - (otherTotal / 5);

  bool candidate = (highestFlexPoint >= 0 &&
                    highestFlexPressure >= layerStartADC &&
                    flexPressureDifference >= layerDiffADC);

  if (candidate) {
    if (!flexPressureCandidate) {
      flexPressureCandidate = true;
      flexPressureSince = millis();
    }
    unsigned long lasting = millis() - flexPressureSince;
    if (lasting >= layerDangerMs ||
        (lasting >= layerAlertMs && highestFlexPressure >= layerHighADC)) {
      flexPressureLevel = 2;
    } else if (lasting >= layerAlertMs) {
      flexPressureLevel = 1;
    } else {
      flexPressureLevel = 0;
    }
  } else {
    flexPressureCandidate = false;
    flexPressureLevel = 0;
  }
}

// ---------------------------------------------------------------------
// SUSTAINED PRESSURE: share of the last window above 4.7 kPa
// ---------------------------------------------------------------------
void updatePressureWindow() {
  if (millis() - lastPressureSample < pressureSampleMs) return;
  lastPressureSample = millis();

  pressureHeelRaw  = analogRead(pressureHeelPin);
  pressureFore1Raw = analogRead(pressureForefoot1Pin);
  pressureFore2Raw = analogRead(pressureForefoot2Pin);

  pressureHeelKPa  = convertToKPa(pressureHeelRaw);
  pressureFore1KPa = convertToKPa(pressureFore1Raw);
  pressureFore2KPa = convertToKPa(pressureFore2Raw);
  highestBasicPressureKPa = max(pressureHeelKPa, max(pressureFore1KPa, pressureFore2KPa));

  uint8_t above = (highestBasicPressureKPa > pressureThresholdKPa) ? 1 : 0;

  if (windowCount == WINDOW_SAMPLES) {
    windowAbove -= pressureWindow[windowIndex];   // drop the oldest sample
  } else {
    windowCount++;
  }
  pressureWindow[windowIndex] = above;
  windowAbove += above;
  windowIndex = (windowIndex + 1) % WINDOW_SAMPLES;

  aboveFraction = (float)windowAbove / WINDOW_SAMPLES;

  // Decide only after a full window has been collected
  if (windowCount < WINDOW_SAMPLES) {
    pressureLevel = 0;
  } else if (aboveFraction >= HIGH_FRACTION) {
    pressureLevel = 2;
  } else if (aboveFraction >= MEDIUM_FRACTION) {
    pressureLevel = 1;
  } else {
    pressureLevel = 0;
  }
}

// ---------------------------------------------------------------------
// TEMPERATURE: this foot (DS18B20)
// ---------------------------------------------------------------------
void readOwnTemperature() {
  if (millis() - lastTempRequest < tempConversionTime) return;
  float t = tempSensor.getTempCByIndex(0);
  bool connected = (t != DEVICE_DISCONNECTED_C);
  tempOutOfRange = connected && (t < skinTempMin || t > skinTempMax);
  tempValid = connected && !tempOutOfRange;
  footTemp = connected ? t : 0;
  tempSensor.requestTemperatures();
  lastTempRequest = millis();
}

// Called once per cloud cycle, after the other foot has been read
void compareTemperatures() {
  if (tempValid && otherTempValid) {
    temperatureDifference = fabs(footTemp - otherFootTemp);
    if (temperatureDifference > temperatureDifferenceLimit) {
      if (temperatureDifferenceCount < requiredTemperatureReadings) temperatureDifferenceCount++;
    } else {
      temperatureDifferenceCount = 0;
    }
  } else {
    temperatureDifference = 0;
    temperatureDifferenceCount = 0;
  }
  tempAlert = (temperatureDifferenceCount >= requiredTemperatureReadings);
}

// ---------------------------------------------------------------------
// HUMIDITY AND MOTION
// ---------------------------------------------------------------------
void readSlowSensors() {
  if (millis() - lastSensorRead < sensorInterval) return;
  lastSensorRead = millis();

  humidity = dht.readHumidity();
  humidityValid = (!isnan(humidity) && humidity >= 0 && humidity <= 100);
  humidityAlert = (humidityValid && humidity > humidityWarning);

  if (mpuReady) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    accelerationMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                                 a.acceleration.y * a.acceleration.y +
                                 a.acceleration.z * a.acceleration.z);
    isMoving = (fabs(accelerationMagnitude - 9.81) > motionThreshold);
    motionValid = true;
  } else {
    motionValid = false;
  }
}

// ---------------------------------------------------------------------
// OVERALL STATUS
// ---------------------------------------------------------------------
void calculateStatus() {
  alertCount = 0;
  if (pressureLevel > 0) alertCount++;
  if (flexPressureLevel > 0) alertCount++;
  if (tempAlert) alertCount++;
  if (humidityAlert) alertCount++;

  dangerMode = (pressureLevel == 2 || flexPressureLevel == 2 || alertCount >= 2);
  warningMode = (alertCount >= 1 && !dangerMode);
}

// ---------------------------------------------------------------------
// RGB LED: green normal, yellow warning, red danger
// ---------------------------------------------------------------------
void setupRGB() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(redPin, 5000, 8);
  ledcAttach(greenPin, 5000, 8);
  ledcAttach(bluePin, 5000, 8);
#else
  ledcSetup(0, 5000, 8); ledcAttachPin(redPin, 0);
  ledcSetup(1, 5000, 8); ledcAttachPin(greenPin, 1);
  ledcSetup(2, 5000, 8); ledcAttachPin(bluePin, 2);
#endif
}

void setLED(int r, int g, int b) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(redPin, r);
  ledcWrite(greenPin, g);
  ledcWrite(bluePin, b);
#else
  ledcWrite(0, r);
  ledcWrite(1, g);
  ledcWrite(2, b);
#endif
}

void updateLED() {
  if (dangerMode) {
    setLED((millis() / 150) % 2 == 0 ? 255 : 0, 0, 0);       // red, fast blink
  } else if (warningMode) {
    setLED(255, 120, 0);                                      // yellow
  } else {
    setLED(0, 255, 0);                                        // green
  }
}

// ---------------------------------------------------------------------
// VIBRATION PATTERNS (yellow LED on GPIO19 stands in for the motor)
// pressure  = long pulses,   flexible pressure alert = fast double pulse,
// temperature = three short pulses
// ---------------------------------------------------------------------
void updateHaptic() {
  unsigned long c;
  bool on = false;

  if (pressureLevel == 2) {               // two long pulses, repeated
    c = millis() % 3000;
    on = (c < 800) || (c >= 1100 && c < 1900);
  } else if (pressureLevel == 1) {        // one long pulse
    c = millis() % 2000;
    on = (c < 800);
  } else if (flexPressureLevel > 0) {            // fast double pulse
    c = millis() % 1000;
    on = (c < 100) || (c >= 200 && c < 300);
  } else if (tempAlert) {                 // three short pulses
    c = millis() % 1500;
    on = (c < 150) || (c >= 300 && c < 450) || (c >= 600 && c < 750);
  } else if (humidityAlert) {             // one short pulse
    c = millis() % 3000;
    on = (c < 150);
  }
  digitalWrite(hapticPin, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------
// BUZZER: a different tone for each active alert
// ---------------------------------------------------------------------
void updateBuzzer() {
  int frequencies[4] = {1000, 1200, 1400, 1600};
  bool active[4] = {pressureLevel > 0, flexPressureLevel > 0, tempAlert, humidityAlert};

  int freqs[4];
  int count = 0;
  for (int i = 0; i < 4; i++) {
    if (active[i]) freqs[count++] = frequencies[i] + (dangerMode ? 400 : 0);
  }
  if (count == 0) { noTone(buzzerPin); return; }

  unsigned long cycle = millis() % 900;
  int selected = (millis() / 900) % count;
  if (cycle < 250) tone(buzzerPin, freqs[selected]);
  else noTone(buzzerPin);
}

// ---------------------------------------------------------------------
// CLOUD: upload this foot, read the other foot
// ---------------------------------------------------------------------
String buildJson() {
  String j = "{";
  j += "\"foot\":\"" + String(FOOT_SIDE) + "\",";
  j += "\"seq\":" + String(uploadSeq) + ",";
  j += "\"status\":\"" + String(statusText()) + "\",";
  j += "\"battery\":" + String(batteryPercent) + ",";
  j += "\"p_heel\":" + String(pressureHeelKPa, 1) + ",";
  j += "\"p_fore1\":" + String(pressureFore1KPa, 1) + ",";
  j += "\"p_fore2\":" + String(pressureFore2KPa, 1) + ",";
  j += "\"p_above_pct\":" + String(aboveFraction * 100.0, 0) + ",";
  j += "\"p_window_ready\":" + String(windowCount >= WINDOW_SAMPLES ? "true" : "false") + ",";
  j += "\"pressure_level\":\"" + String(levelText(pressureLevel)) + "\",";
  j += "\"flex_pressure\":[";
  for (int i = 0; i < 6; i++) {
    j += String(flexPressureReading[i]);
    if (i < 5) j += ",";
  }
  j += "],";
  j += "\"flex_pressure_point\":\"" + String(highestFlexPoint >= 0 ? layerPoints[highestFlexPoint] : "-") + "\",";
  j += "\"flex_pressure_level\":\"" + String(flexPressureLevel == 2 ? "DANGER" : flexPressureLevel == 1 ? "ALERT" : "OK") + "\",";
  j += "\"temp_c\":" + (tempValid ? String(footTemp, 1) : String("null")) + ",";
  j += "\"temp_other\":" + (otherTempValid ? String(otherFootTemp, 1) : String("null")) + ",";
  j += "\"temp_diff\":" + String(temperatureDifference, 1) + ",";
  j += "\"temp_alert\":" + String(tempAlert ? "true" : "false") + ",";
  j += "\"humidity\":" + (humidityValid ? String(humidity, 1) : String("null")) + ",";
  j += "\"humidity_alert\":" + String(humidityAlert ? "true" : "false") + ",";
  j += "\"moving\":" + String(isMoving ? "true" : "false") + ",";
  j += "\"demo_mode\":" + String(DEMO_MODE ? "true" : "false");
  j += "}";
  return j;
}

// Read a number after "key": in a JSON text. Returns false if missing or null.
bool jsonNumber(const String& body, const char* key, float& out) {
  String k = String("\"") + key + "\":";
  int p = body.indexOf(k);
  if (p < 0) return false;
  p += k.length();
  if (body.startsWith("null", p)) return false;
  out = body.substring(p).toFloat();
  return true;
}

void readOtherFoot(WiFiClientSecure& client) {
  HTTPClient http;
  String url = String(FIREBASE_URL) + OTHER_SIDE + ".json";
  http.begin(client, url);
  lastGetCode = http.GET();
  otherTempValid = false;

  if (lastGetCode == 200) {
    String body = http.getString();
    float t, s;
    bool hasTemp = jsonNumber(body, "temp_c", t);
    bool hasSeq = jsonNumber(body, "seq", s);
    if (hasSeq && (long)s != otherSeq) {
      otherSeq = (long)s;
      otherSeqChangedAt = millis();
    }
    // the other foot must have uploaded in the last 15 s (it is running)
    bool fresh = hasSeq && (millis() - otherSeqChangedAt < 15000);
    if (hasTemp && fresh) {
      otherFootTemp = t;
      otherTempValid = true;
    }
  }
  http.end();
}

void updateCloud() {
  if (millis() - lastCloud < cloudInterval) return;
  lastCloud = millis();

  wifiReady = (WiFi.status() == WL_CONNECTED);
  if (!wifiReady) {
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry > 15000) {      // try again every 15 s
      lastRetry = millis();
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    otherTempValid = false;
    compareTemperatures();
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();   // simulation only: no certificate check

  // 1) read the other foot, 2) compare temperatures, 3) upload this foot
  readOtherFoot(client);
  compareTemperatures();
  calculateStatus();

  uploadSeq++;
  HTTPClient http;
  String url = String(FIREBASE_URL) + FOOT_SIDE + ".json";
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  lastPutCode = http.PUT(buildJson());
  http.end();
}

// ---------------------------------------------------------------------
// OLED PAGES
// ---------------------------------------------------------------------
void pageHeader(const char* title) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(title);
  display.print(" ");
  display.println(FOOT_SIDE);
  display.println("---------------------");
}

void showReadingsPage() {
  pageHeader("READINGS");
  display.print("Heel:  "); display.print(pressureHeelKPa, 1);  display.println(" kPa");
  display.print("Fore1: "); display.print(pressureFore1KPa, 1); display.println(" kPa");
  display.print("Fore2: "); display.print(pressureFore2KPa, 1); display.println(" kPa");
  display.print("Temp:  ");
  if (tempValid) { display.print(footTemp, 1); display.println(" C"); }
  else if (tempOutOfRange) display.println("RANGE ERR");
  else display.println("ERR");
  display.print("Other: ");
  if (otherTempValid) { display.print(otherFootTemp, 1); display.println(" C"); } else display.println("--");
  display.display();
}

void showStatusPage() {
  pageHeader("STATUS");
  display.print("Risk: ");        display.println(statusText());
  display.print("Pressure: ");    display.println(levelText(pressureLevel));
  display.print("Flex pressure: "); display.println(flexPressureLevel == 2 ? "DANGER" : flexPressureLevel == 1 ? "ALERT" : "OK");
  display.print("Temp diff: ");   display.println(tempAlert ? "ALERT" : "OK");
  display.print("Humidity: ");    display.println(humidityAlert ? "HIGH" : "OK");
  display.display();
}

void showPressurePage() {
  pageHeader("PRESSURE");
  display.print("Above 4.7kPa: "); display.print(aboveFraction * 100.0, 0); display.println("%");
  display.print("Window: "); display.print(windowCount); display.print("/"); display.println(WINDOW_SAMPLES);
  display.print("Layer max: "); display.println(highestFlexPressure);
  display.print("Layer diff: "); display.println(flexPressureDifference);
  display.print("Point: "); display.println(highestFlexPoint >= 0 ? layerPoints[highestFlexPoint] : "-");
  display.display();
}

void showSystemPage() {
  pageHeader("SYSTEM");
  display.print("Battery: "); display.print(batteryPercent); display.println("%");
  display.print("Humidity: ");
  if (humidityValid) { display.print(humidity, 0); display.println("%"); } else display.println("ERR");
  display.print("Move: "); display.println(motionValid ? (isMoving ? "YES" : "NO") : "ERR");
  display.print("WiFi: "); display.println(wifiReady ? "OK" : "...");
  display.print("Cloud: "); display.println(lastPutCode);
  display.display();
}

void updateOLED() {
  if (!oledReady) return;
  if (millis() - lastDisplayChange < displayInterval) return;
  lastDisplayChange = millis();
  currentPage = (currentPage + 1) % 4;
  switch (currentPage) {
    case 0: showReadingsPage(); break;
    case 1: showStatusPage(); break;
    case 2: showPressurePage(); break;
    case 3: showSystemPage(); break;
  }
}

// ---------------------------------------------------------------------
// SERIAL MONITOR
// ---------------------------------------------------------------------
void printSerialData() {
  if (millis() - lastSerialPrint < serialInterval) return;
  lastSerialPrint = millis();

  Serial.println("\n===== NEUROSENSE =====");
  Serial.print("Foot: "); Serial.println(FOOT_SIDE);
  Serial.print("Battery: "); Serial.print(batteryPercent); Serial.println("%");

  Serial.println("--- MAIN PRESSURE (3 sensors) ---");
  Serial.print("Heel: ");  Serial.print(pressureHeelKPa, 1);  Serial.println(" kPa");
  Serial.print("Fore1: "); Serial.print(pressureFore1KPa, 1); Serial.println(" kPa");
  Serial.print("Fore2: "); Serial.print(pressureFore2KPa, 1); Serial.println(" kPa");
  Serial.print("Time above 4.7 kPa: "); Serial.print(aboveFraction * 100.0, 0);
  Serial.print("% of window ("); Serial.print(windowCount); Serial.print("/");
  Serial.print(WINDOW_SAMPLES); Serial.println(" s)");
  Serial.print("Pressure level: "); Serial.println(levelText(pressureLevel));

  Serial.println("--- FLEXIBLE PRESSURE LAYER (6 points) ---");
  for (int i = 0; i < 6; i++) {
    Serial.print(layerPoints[i]); Serial.print(": "); Serial.println(flexPressureReading[i]);
  }
  Serial.print("Highest point: ");
  Serial.println(highestFlexPoint >= 0 ? layerPoints[highestFlexPoint] : "NONE");
  Serial.print("Difference from others: "); Serial.println(flexPressureDifference);
  Serial.print("Layer level: ");
  Serial.println(flexPressureLevel == 2 ? "DANGER" : flexPressureLevel == 1 ? "ALERT" : "OK");

  Serial.println("--- TEMPERATURE ---");
  Serial.print("This foot: ");
  if (tempValid) Serial.println(String(footTemp, 1));
  else if (tempOutOfRange) { Serial.print(String(footTemp, 1)); Serial.println("  OUT OF SKIN RANGE (10-45 C) - not used"); }
  else Serial.println("ERROR");
  Serial.print("Other foot (cloud): ");
  Serial.println(otherTempValid ? String(otherFootTemp, 1) : "not available");
  Serial.print("Difference: "); Serial.println(temperatureDifference, 1);
  Serial.print("Consecutive readings > 2.2 C: "); Serial.println(temperatureDifferenceCount);
  Serial.print("Temperature alert: "); Serial.println(tempAlert ? "YES" : "NO");

  Serial.println("--- HUMIDITY & MOTION ---");
  Serial.print("Humidity: "); Serial.println(humidityValid ? String(humidity, 1) : "ERROR");
  Serial.print("Motion: "); Serial.println(motionValid ? (isMoving ? "MOVING" : "STILL") : "ERROR");

  Serial.println("--- SYSTEM ---");
  Serial.print("Status: "); Serial.println(statusText());
  Serial.print("WiFi: "); Serial.println(wifiReady ? "CONNECTED" : "CONNECTING");
  Serial.print("Upload code: "); Serial.print(lastPutCode);
  Serial.print(" | Read other foot code: "); Serial.println(lastGetCode);
  Serial.println("======================");
}

// ---------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(buzzerPin, OUTPUT);
  pinMode(hapticPin, OUTPUT);
  digitalWrite(hapticPin, LOW);

  pinMode(muxS0Pin, OUTPUT);
  pinMode(muxS1Pin, OUTPUT);
  pinMode(muxS2Pin, OUTPUT);
  pinMode(muxS3Pin, OUTPUT);
  pinMode(muxSIGPin, INPUT);
  pinMode(BATTERY_PIN, INPUT);

  setupRGB();
  setLED(0, 0, 255);   // blue while starting

  batteryDisplay.setBrightness(0x0f);
  batteryDisplay.clear();

  tempSensor.begin();
  tempSensor.requestTemperatures();          // first reading: wait for it
  {
    float t0 = tempSensor.getTempCByIndex(0);
    bool connected = (t0 != DEVICE_DISCONNECTED_C);
    tempOutOfRange = connected && (t0 < skinTempMin || t0 > skinTempMax);
    tempValid = connected && !tempOutOfRange;
    footTemp = connected ? t0 : 0;
  }
  tempSensor.setWaitForConversion(false);    // after that: do not block
  tempSensor.requestTemperatures();
  lastTempRequest = millis();

  dht.begin();

  Wire.begin(21, 22);
  oledReady = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledReady) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("NEUROSENSE");
    display.println(FOOT_SIDE);
    display.println("Starting...");
    display.display();
  }

  mpuReady = mpu.begin();
  Serial.print("MPU6050: ");
  Serial.println(mpuReady ? "CONNECTED" : "NOT FOUND");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(250);
    Serial.print(".");
  }
  wifiReady = (WiFi.status() == WL_CONNECTED);
  Serial.println(wifiReady ? " CONNECTED" : " not yet (will keep trying)");

  Serial.print("NeuroSense started - ");
  Serial.println(FOOT_SIDE);
}

// ---------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------
void loop() {
  updateBattery();
  updateFlexPressureLayer();
  updatePressureWindow();
  readOwnTemperature();
  readSlowSensors();
  calculateStatus();

  updateLED();
  updateHaptic();
  updateBuzzer();

  updateCloud();
  updateOLED();
  printSerialData();

  delay(5);   // give the WiFi task time to run
}
