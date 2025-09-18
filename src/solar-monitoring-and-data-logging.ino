#include <DallasTemperature.h>
#include <OneWire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

// ====== User config ======
constexpr char WIFI_SSID[] = "UGM-Hotspot";
constexpr char WIFI_PASSWORD[] = "";
constexpr char TOKEN[] = "ByExtlDrqCtw59F5gM6C";
constexpr char HOST[] = "thingsboard.cloud";
constexpr int HTTPS_PORT = 443;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr int HTTP_TIMEOUT = 6000;

// ====== Wi-Fi ======
WiFiClientSecure client;

// ====== LED ======
#define LED_PIN 15
inline void ledOn() { digitalWrite(LED_PIN, HIGH); }
inline void ledOff() { digitalWrite(LED_PIN, LOW); }

// ====== Pins ======
// ESP32-S3 example where GPIO7/8 are ADC-capable. Adjust for your board if needed.
constexpr int VS_PIN = 7;   // divider midpoint
constexpr int ACS_PIN = 6;  // ACS712 signal directly (no divider)
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
const int nSamples = 1000;
const float vcc = 5.0;
const int adcMax = 1023;

const float sens = 0.185;  // 5A
// const float sens = 0.100;  // 20A
// const float sens = 0.66;  // 30A

void initWiFi();
float readVoltage_mV();
float readCurrent_mA();
float readTemperatureC();
void sendTelemetry(float voltage, float current_mA, float temperature, int light);

// ====== Setup & Loop ======
unsigned long lastSend = 0;
const unsigned long SEND_PERIOD_MS = 1000;  // 1s

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  ledOn();
  delay(500);
  ledOff();

  Serial.begin(SERIAL_BAUD);
  delay(100);
  Serial.println("\nSolar Monitoring and Data Logging");

  randomSeed(esp_random());
  initWiFi();

  DSSensor.begin();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    ledOff();
    initWiFi();
  }

  unsigned long now = millis();
  if (now - lastSend >= SEND_PERIOD_MS) {
    lastSend = now;

    float voltage = readVoltage_mV();        // ~0–4.1 V (7.5k/30k divider)
    float current_mA = readCurrent_mA();     // **milliamps**
    float temperature = readTemperatureC();  // 20.0–39.9 °C (random)
    int light = random(0, 1024);             // 0–1023 (random)

    Serial.printf("VIN=%.2f mV, I=%.1f mA, T=%.1f C, L=%d\n", voltage, current_mA, temperature, light);
    sendTelemetry(voltage, current_mA, temperature, light);
  }
}

// ====== Functions ======

void initWiFi() {
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  client.setInsecure();  // ⚠️ For production, replace with client.setCACert(root_ca)

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  ledOn();
}

// Voltage via divider
float readVoltage_mV() {
  const int SAMPLES = 32;
  uint32_t sum = 0;
  for (int i = 0; i < SAMPLES; ++i) {
    sum += analogRead(VS_PIN);
    delayMicroseconds(120);
  }
  float raw = static_cast<float>(sum) / SAMPLES;
  float vpin = (raw / ADC_MAX) * ADC_VREF;
  float vin = vpin * ((R1_V + R2_V) / R2_V) * VOLT_CAL;
  return vin * 1000.0f;  // convert V to mV
}

// Current via ACS712
float readCurrent_mA() {
  float sensorValue = 0;
  for (int i = 0; i < nSamples; i++) {
    sensorValue += analogRead(ACS_PIN);
    delay(1);
  }
  float avg = sensorValue / adcMax / nSamples;
  float current = (vcc / 2 - vcc * avg) / sens;

  return current * 1000.0f;  // convert A to mA
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
  String request = "POST " + url + " HTTP/1.1\r\n" + "Host: " + String(HOST) + "\r\n" + "Content-Type: application/json\r\n" +
                   "Content-Length: " + String(payload.length()) + "\r\n" + "Connection: close\r\n\r\n" + payload;

  client.print(request);

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }
  client.stop();
}