#include <BH1750.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

#include "ACS712.h"

// ====== User config ======
constexpr char WIFI_SSID[] = "UGM-Hotspot";
constexpr char WIFI_PASSWORD[] = "";
constexpr int WIFI_TIMEOUT_MS = 5000;
constexpr char TOKEN[] = "ByExtlDrqCtw59F5gM6C";
constexpr char HOST[] = "thingsboard.cloud";
constexpr int HTTPS_PORT = 443;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr int HTTP_TIMEOUT = 6000;

// ====== Wi-Fi ======
WiFiClientSecure client;

// ====== LED ======
#define LED_PIN 2
inline void ledOn() { digitalWrite(LED_PIN, HIGH); }
inline void ledOff() { digitalWrite(LED_PIN, LOW); }

// ====== Pins ======
constexpr int VS_PIN = 32;   // Voltage sensor
constexpr int ACS_PIN = 33;  // ACS712 signal directly (no divider)
constexpr int DS_PIN = 23;   // DS18B20 data (with 4.7k pull-up to 3.3V)

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(DS_PIN);
DallasTemperature DSSensor(&oneWire);

// ====== Voltage sensor config ======
constexpr float ADC_VREF = 3.3f;  // ESP32 ADC reference (approx.)
constexpr int ADC_MAX = 4095;     // 12-bit
const float R1_V = 7500.0f;       // ohms (top resistor, 7501)
const float R2_V = 30000.0f;      // ohms (bottom resistor, 3012)
const float VOLT_CAL = 1.00f;     // fine calibration multiplier

// ====== ACS712 configuration ======
//  Arduino UNO has 5.0 volt with a max ADC value of 1023 steps
//  ACS712 5A  uses 185 mV per A
//  ACS712 20A uses 100 mV per A
//  ACS712 30A uses  66 mV per A

// ACS712 ACS(A0, 5.0, 1023, 100);
//  ESP 32 example (might requires resistors to step down the logic voltage)
ACS712 ACS(25, 3.3, 4095, 185);

// ====== BH1750 light sensor ======
BH1750 lightMeter(0x23);

// ====== LCD I2C 16x2 ======
LiquidCrystal_I2C lcd(0x27, 16, 2);

void initWiFi();
float readVoltage_mV();
float readTemperatureC();
float readLight();
void sendTelemetry(float voltage, float current_mA, float temperature, int light);

// ====== Setup & Loop ======
unsigned long lastSend = 0;
const unsigned long SEND_PERIOD_MS = 1000;  // 1s

void setup() {
  Wire.begin();
  pinMode(LED_PIN, OUTPUT);
  ledOn();
  delay(500);
  ledOff();

  //  assuming no current flowing at startup
  ACS.autoMidPoint();

  lcd.begin(16, 2);
  lcd.backlight();

  lcd.setCursor(1, 0);
  lcd.print("Solar Monitor");
  lcd.setCursor(2, 1);
  lcd.print("Starting...");

  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Serial.println(F("\nSolar Monitoring and Data Logging"));
  Serial.println(F("Starting..."));

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("BH1750 Advanced begin"));
  } else {
    Serial.println(F("Error initialising BH1750"));
  }

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

    float voltage_mV = readVoltage_mV();     // ~0–4.1 V (7.5k/30k divider)
    float current_mA = ACS.mA_DC();          // -5000 to +5000 mA (5A version)
    float temperature = readTemperatureC();  // 20.0–39.9 °C (random)
    float light = readLight();               // BH1750

    Serial.printf("VIN=%.2f mV, I=%.1f mA, T=%.1f C, L=%.1f lux\n", voltage_mV, current_mA, temperature, light);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printf("V:%4.1fmV", voltage_mV);
    lcd.setCursor(0, 1);
    lcd.printf("I:%4.1fmA", current_mA);

    sendTelemetry(voltage_mV, current_mA, temperature, light);
  }
}

// ====== Functions ======

void initWiFi() {
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    // Timeout after 10 seconds
    if (millis() - startAttemptTime > 10000) {
      Serial.println(F("\nFailed to connect to WiFi"));
      return;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("\nWiFi connected"));
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
  }

  client.setInsecure();  // ⚠️ For production, replace with client.setCACert(root_ca)
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

float readTemperatureC() {
  DSSensor.requestTemperatures();
  return DSSensor.getTempCByIndex(0);
}

// ====== Send telemetry via HTTPS ======
void sendTelemetry(float voltage, float current_mA, float temperature, float light) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("WiFi not connected, cannot send telemetry"));
    return;
  }

  if (!client.connect(HOST, HTTPS_PORT, HTTP_TIMEOUT)) {
    Serial.println(F("Connection to server failed!"));
    return;
  }

  String payload = "{";
  payload += "\"voltage\":" + String(voltage, 2) + ",";
  payload += "\"current\":" + String(current_mA, 1) + ",";  // send milliamps
  payload += "\"temperature\":" + String(temperature, 1) + ",";
  payload += "\"light\":" + String(light, 1);
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

float readLight() {
  if (lightMeter.measurementReady()) {
    return lightMeter.readLightLevel();
  }
  return -1.0f;
}