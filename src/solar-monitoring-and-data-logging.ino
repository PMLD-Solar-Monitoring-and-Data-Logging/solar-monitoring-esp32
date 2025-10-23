#include <BH1750.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>

#include "ACS712.h"

// ====== User config ======
#include "secrets.h"
constexpr int WIFI_TIMEOUT_MS = 5000;
constexpr uint32_t SERIAL_BAUD = 115200;

// ====== LED ======
#define LED_PIN 2
inline void ledOn() { digitalWrite(LED_PIN, HIGH); }
inline void ledOff() { digitalWrite(LED_PIN, LOW); }

// ====== Sensor Pins ======
constexpr int VS_PIN = 32;     // Voltage sensor
constexpr int ACS_PIN = 33;    // ACS712 signal directly (no divider)
constexpr int DS_PIN = 23;     // DS18B20 data (with 4.7k pull-up to 3.3V)
constexpr int RELAY_PIN = 13;  // Relay control (active HIGH)

// ===== DS18B20 temperature sensor configuration =====
OneWire oneWire(DS_PIN);
DallasTemperature DSSensor(&oneWire);

// ====== Voltage sensor configuration ======
constexpr float ADC_VREF = 3.3f;  // ESP32 ADC reference (approx.)
constexpr int ADC_MAX = 4095;     // 12-bit
const float R1_V = 7500.0f;       // ohms (top resistor, 7501)
const float R2_V = 30000.0f;      // ohms (bottom resistor, 3012)
const float VOLT_CAL = 4.15f;     // fine calibration multiplier

// ====== ACS712 configuration ======
//  Arduino UNO has 5.0 volt with a max ADC value of 1023 steps
//  ACS712 5A  uses 185 mV per A
//  ACS712 20A uses 100 mV per A
//  ACS712 30A uses  66 mV per A

// ACS712 ACS(A0, 5.0, 1023, 100);
ACS712 ACS(ACS_PIN, 3.3, 3725, 185);
const float CURRENT_CAL = 0.85f;  // fine calibration multiplier

// ====== BH1750 light sensor configuration ======
BH1750 lightMeter(0x23);

// ====== LCD I2C 16x2 configuration ======
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ====== Function declarations ======
void initWiFi();
float readVoltage();
float readTemperature();
float readLight();
void sendTelemetry(float voltage_mV, float current_mA, float temperature, float light);
void mqttConnect();
void callback(char* topic, byte* payload, unsigned int length);

// ====== WiFi and MQTT ======
WiFiClient espClient;
PubSubClient client(HOST, PORT, callback, espClient);

// ====== Setup & Loop ======
unsigned long lastSend = 0;
const unsigned long SEND_PERIOD_MS = 1000;  // 1s

void setup() {
  Wire.begin();
  pinMode(LED_PIN, OUTPUT);

  // Relay & ACS712 initialization
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  ACS.autoMidPoint();

  ledOn();
  delay(500);
  ledOff();

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

  if (!client.connected()) {
    mqttConnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastSend >= SEND_PERIOD_MS) {
    lastSend = now;

    float voltage = readVoltage();          // ~0–4.1 V (7.5k/30k divider)
    float current = readCurrent();          // -5000 to +5000 mA (5A version)
    float temperature = readTemperature();  // 20.0–39.9 °C (random)
    float light = readLight();              // BH1750

    Serial.printf("VIN=%.2f V, I=%.3f A, T=%.1f C, L=%.1f lux\n", voltage, current, temperature, light);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printf("V:%1.2fV", voltage);
    lcd.setCursor(0, 1);
    lcd.printf("I:%1.2fA", current);

    lcd.setCursor(9, 0);
    lcd.printf("T:%d", (int)temperature);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(9, 1);
    lcd.printf("L:%dlx", (int)light);

    sendTelemetry(voltage, current, temperature, light);
  }
}

// ====== Functions ======

// ===== Initialize WiFi ======
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
}

// ===== Voltage sensor (0-25v) ======
float readVoltage() {
  const int SAMPLES = 32;
  uint32_t sum = 0;
  for (int i = 0; i < SAMPLES; ++i) {
    sum += analogRead(VS_PIN);
    delayMicroseconds(120);
  }
  float raw = sum / (float)SAMPLES;
  float vpin = (raw / ADC_MAX) * ADC_VREF;
  float vin = vpin * ((R1_V + R2_V) / R2_V) * VOLT_CAL;
  return vin;
}

// ===== Current via ACS712 ======
float readCurrent() {
  const int SAMPLES = 32;
  float sum = 0.0f;
  for (int i = 0; i < SAMPLES; ++i) {
    sum += ACS.mA_DC();
    delayMicroseconds(120);
  }

  return sum / (float)SAMPLES / 1000.0f * CURRENT_CAL;  // Convert mA to A
}

// ===== Temperature via DS18B20 ======
float readTemperature() {
  DSSensor.requestTemperatures();
  return DSSensor.getTempCByIndex(0);
}

// ===== Light via BH1750 ======
float readLight() {
  if (lightMeter.measurementReady()) {
    return lightMeter.readLightLevel();
  }
  return -1.0f;
}

// ===== MQTT connect ======
void mqttConnect() {
  if (!client.connected()) {
    ledOff();
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), TOKEN, NULL)) {
      Serial.println("connected");
      client.subscribe("v1/devices/me/attributes");
      client.subscribe("v1/devices/me/attributes/response/+");
      client.publish("v1/devices/me/attributes/request/1", "{\"sharedKeys\":\"relay\"}");
      ledOn();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(1000);
    }
  }
}

// ===== MQTT message callback ======
void callback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  //   Serial.println("Message arrived: " + message);

  int relayStart = message.indexOf("\"relay\":");
  if (relayStart >= 0) {
    if (message.indexOf("true", relayStart) >= 0) {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Relay set to ON from MQTT");
    } else if (message.indexOf("false", relayStart) >= 0) {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Relay set to OFF from MQTT");
    }
  }
}

// ====== Send sensor telemetry ======
void sendTelemetry(float voltage, float current, float temperature, float light) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("WiFi not connected, cannot send telemetry"));
    return;
  }

  String payload = "{";
  payload += "\"voltage\":" + String(voltage, 2) + ",";
  payload += "\"current\":" + String(current, 3) + ",";
  payload += "\"temperature\":" + String(temperature, 1) + ",";
  payload += "\"light\":" + String(light, 1);
  payload += "}";

  if (!client.publish("v1/devices/me/telemetry", payload.c_str())) {
    Serial.println("Failed to send telemetry");
  }
}
