#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ====== User config ======
constexpr char WIFI_SSID[] = "UGM-Hotspot";
constexpr char WIFI_PASSWORD[] = "";
constexpr char TOKEN[] = "ByExtlDrqCtw59F5gM6C";
constexpr char HOST[] = "thingsboard.cloud";
constexpr int HTTPS_PORT = 443;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr int HTTP_TIMEOUT = 6000;

// ====== LED ======
#define LED_BUILTIN 15

inline void ledOn() {
  digitalWrite(LED_BUILTIN, HIGH);
}
inline void ledOff() {
  digitalWrite(LED_BUILTIN, LOW);
}

void blinkBoot(uint8_t times = 3, uint16_t on_ms = 150, uint16_t off_ms = 150) {
  for (uint8_t i = 0; i < times; i++) {
    ledOn();
    delay(on_ms);
    ledOff();
    delay(off_ms);
  }
}

// ====== Pins ======
// ESP32-S3 example where GPIO7/8 are ADC-capable. Adjust for your board if needed.
constexpr int VOLTAGE_PIN = 7;  // divider midpoint
constexpr int ACS_PIN = 6;      // ACS712 signal directly (no divider)
constexpr int DS_PIN = 5;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(DS_PIN);
DallasTemperature DSSensor(&oneWire);


// ====== Voltage sensor config ======
constexpr float ADC_VREF = 3.3f;  // ESP32 ADC reference (approx.)
constexpr int ADC_MAX = 4095;     // 12-bit
const float R1_V = 7500.0f;       // ohms (top resistor, 7501)
const float R2_V = 30000.0f;      // ohms (bottom resistor, 3012)
const float VOLT_CAL = 1.00f;     // fine calibration multiplier

// ACS172 configuration
const int mVperAmp = 185;           // this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module

// ====== Wi-Fi ======
WiFiClientSecure client;

void initWiFi() {
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  client.setInsecure();  // ⚠️ For production, replace with client.setCACert(root_ca)
}


// Voltage via divider
float readVoltage_mV() {
  const int SAMPLES = 32;
  uint32_t sum = 0;
  for (int i = 0; i < SAMPLES; ++i) {
    sum += analogRead(ACS_PIN);
    delayMicroseconds(120);
  }
  float raw = static_cast<float>(sum) / SAMPLES;
  float vpin = (raw / ADC_MAX) * ADC_VREF;
  float vin = vpin * ((R1_V + R2_V) / R2_V) * VOLT_CAL;
  return vin * 1000.0f;  // convert V to mV
}

// Current via ACS712
float readCurrent_mA() {
  int maxValue = 0;     // store max value here
  int minValue = 4096;  // store min value here ESP32 ADC resolution
  uint32_t start_time = millis();
  while ((millis() - start_time) < 1000)  //sample for 1 Sec
  {
    int readValue = analogRead(ACS_PIN);
    // see if you have a new maxValue
    if (readValue > maxValue) {
      /*record the maximum sensor value*/
      maxValue = readValue;
    }
    if (readValue < minValue) {
      /*record the minimum sensor value*/
      minValue = readValue;
    }
  }
  // Subtract min from max
  float Voltage = ((maxValue - minValue) * 3.3) / 4096.0;  //ESP32 ADC resolution 4096
  double VRMS = (Voltage / 2.0) * 0.707;                   //root 2 is 0.707
  double AmpsRMS = ((VRMS * 1000) / mVperAmp) - 0.3;       //0.3 is the error I got for my sensor
  return AmpsRMS * 1000;                                   // convert A to mA
}

float readTemperatureC() {
  DSSensor.requestTemperatures();
  return DSSensor.getTempCByIndex(0);
}

// ====== Send telemetry via HTTPS ======
void sendTelemetry(float voltage, float current_mA, float temperature, int light) {
  if (!client.connect(HOST, HTTPS_PORT, HTTP_TIMEOUT)) {
    Serial.println("Connection to server failed!");
    return;
  }

  String payload = "{";
  payload += "\"voltage\":" + String(voltage, 2) + ",";
  payload += "\"current\":" + String(current_mA, 1) + ",";  // send milliamps
  payload += "\"temperature\":" + String(temperature, 1) + ",";
  payload += "\"light\":" + String(light);
  payload += "}";

  String url = "/api/v1/" + String(TOKEN) + "/telemetry";
  String request =
    "POST " + url + " HTTP/1.1\r\n" + "Host: " + String(HOST) + "\r\n" + "Content-Type: application/json\r\n" + "Content-Length: " + String(payload.length()) + "\r\n" + "Connection: close\r\n\r\n" + payload;

  client.print(request);

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }
  client.stop();
}

// ====== Setup & Loop ======
unsigned long lastSend = 0;
const unsigned long SEND_PERIOD_MS = 1000;  // 1s

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  ledOff();
  Serial.begin(SERIAL_BAUD);
  delay(100);

  blinkBoot();
  randomSeed(esp_random());
  initWiFi();

  DSSensor.begin();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(15, LOW);
    initWiFi();
  }

  digitalWrite(15, HIGH);

  unsigned long now = millis();
  if (now - lastSend >= SEND_PERIOD_MS) {
    lastSend = now;

    float voltage = readVoltage_mV();        // ~0–4.1 V (7.5k/30k divider)
    float current_mA = readCurrent_mA();     // **milliamps**
    float temperature = readTemperatureC();  // 20.0–39.9 °C (random)
    int light = random(0, 1024);             // 0–1023 (random)

    Serial.printf("VIN=%.2f mV, I=%.1f mA, T=%.1f C, L=%d\n",
                  voltage, current_mA, temperature, light);
    sendTelemetry(voltage, current_mA, temperature, light);
  }
}

/*
Notes:
- Telemetry key is now **current_mA** (milliamps). Update dashboards accordingly.
- For DC, offset is subtracted; for AC, set ACS_MEASURE_AC=true to compute RMS mA.
- Tune ACS_SENS and ACS_CAL against a known load for accuracy.
- ESP32 ADC non-linearity can be improved with calibration tables if needed.
- For production TLS, use client.setCACert(<root CA>) instead of setInsecure().
*/
