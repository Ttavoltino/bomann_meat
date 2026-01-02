// === Libraries ===
#include "Adafruit_SHT31.h"
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "arduino_secrets.h"

const char* temp_topic = "sushilna/temperature";
const char* hum_topic  = "sushilna/humidity";
const char* set_hum_topic  = "sushilna/set/humidity";
const char* set_temp_topic = "sushilna/set/temperature";
const char* temp_status_topic = "sushilna/status/temperature";
const char* hum_status_topic  = "sushilna/status/humidity";
const char* set_topfanspeed_topic = "sushilna/set/topfanspeed";
const char* compressor_status_topic = "sushilna/compressor";
const char* humifider_status_topic = "sushilna/humifider";
const char* dehumifider_status_topic = "sushilna/dehumifider";
const char* tempctr_status_topic = "sushilna/temp_control";
const char* heat_status_topic = "sushilna/status/heater";
const char* status_online_topic = "sushilna/status/";

WiFiClient espClient;
PubSubClient client(espClient);

// === SHT31 Sensor ===
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// === Variables ===
String clientId = "Bomann_Sushilna_" + String((uint32_t)ESP.getEfuseMac(), HEX);
unsigned long compressorRun;
unsigned long sensorRead;
unsigned long wifiCheck;
unsigned long lastReconnectAttempt;
int temperature;
byte humidity;
byte topfanSpeed = 35;

byte tempSet = 13;
byte humSet = 75;
int lastTemp = -100;
int lastTempSet = -100;
byte lastHum = 150;
byte lastHumSet = 150;
bool lastHeatcontrol = false;

// States
bool compressorActive     = false;
bool tempControlEnabled   = true;
bool allowHumidityControl = true;
bool fanBoostActive       = true;
bool dehumifider          = false;
bool humifider            = false;
bool tempControl          = false;
bool lastDehumifider      = false;
bool lastHumifider        = false;
bool lastTempcontrol      = false;
bool lastCompressorActive = false;
bool heatCtrl             = false;

// === Pins ===
const byte heaterPin = 4;
const byte compPin   = 26;
const byte cFanPin   = 27;
const byte topFanPin = 14;

// === Setup ===
void setup() {
  Serial.begin(115200);
  sht31.begin(0x44);

  pinMode(compPin, OUTPUT);
  digitalWrite(compPin, LOW);
  pinMode(heaterPin, OUTPUT);
  digitalWrite(heaterPin, LOW);

  // PWM setup (ESP32 core 3.x)
  ledcAttach(cFanPin, 25000, 8);
  ledcWrite(cFanPin, 0);

  ledcAttach(topFanPin, 25000, 8);
  ledcWrite(topFanPin, 0);


  temperature = sht31.readTemperature();
  humidity    = sht31.readHumidity();

  WiFi.mode(WIFI_STA);
  connectToWiFi();

  client.setServer(MQTT_SERVER , MQTT_PORT );
  client.setCallback(mqttCallback);
}

// === Loop ===
void loop() {

  ledcWrite(topFanPin, map(topfanSpeed, 5, 100, 14, 255));

  if (!client.connected()) {
    reconnectMQTT();
  } else {
    client.loop();
  }

  if (timeWait(30000, 4)) {
    wifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) connectToWiFi();
  }

  sht3xRead();
  tempCtrl();
  humCtrl();
  publishData();
}

// === Functions ===

void sht3xRead() {
  if (timeWait(5000, 1)) {
    int newTemp = sht35.readTemperature();
    byte newHum  = sht35.readHumidity();
    // ---------- TEMP ----------
if (newTemp >= 0 && newTemp <= 45 &&
    abs(newTemp - temperature) <= 3) {

  if (newTemp > temperature + 1)
    temperature += 0.3;
  else if (newTemp < temperature - 1)
    temperature -= 0.3;
  else
    temperature = newTemp;
}

// ---------- HUM ----------
if (newHum >= 0 && newHum <= 100 &&
    abs(newHum - humidity) <= 5) {

  if (newHum > humidity + 1)
    humidity += 1;
  else if (newHum < humidity - 1)
    humidity -= 1;
  else
    humidity = newHum;
}
    // temperature = sht31.readTemperature();
    // humidity    = sht31.readHumidity();
    sensorRead = millis();
  }
}

void tempCtrl() {
  if (temperature >= tempSet + 2) {
    compressor(true);
    tempControlEnabled = true;
    allowHumidityControl = false;
    tempControl = true;
  }
  else if (temperature <= tempSet) {
    if (tempControlEnabled) {
      compressor(false);
      tempControlEnabled = false;
      allowHumidityControl = true;
      tempControl = false;
    }
  }
  if (temperature <= tempSet - 2) {
    digitalWrite(heaterPin, HIGH);
    heatCtrl = true;
  } else if (temperature >= tempSet) {
    digitalWrite(heaterPin, LOW);
    heatCtrl = false;
  }
}

void humCtrl() {
  if (tempSet >= 8 && allowHumidityControl) {
    if (humidity >= humSet + 3) {
      compressor(true);
      dehumifider = true;
    }
    else if (humidity <= humSet - 3) {
      compressor(false);
      dehumifider = false;
    }

    if (humidity <= humSet - 5) {
      ledcWrite(cFanPin, 254);
      fanBoostActive = true;
      humifider = true;
    }
    else if (humidity >= humSet) {
      if (fanBoostActive) {
        ledcWrite(cFanPin, 0);
        fanBoostActive = false;
        humifider = false;
      }
    }
  }
}

void compressor(bool state) {
  if (state && timeWait(180000, 2)) {
    digitalWrite(compPin, HIGH);
    ledcWrite(cFanPin, 150);
    compressorActive = true;
  }
  else if (!state) {
    digitalWrite(compPin, LOW);
    if (compressorActive) {
      ledcWrite(cFanPin, 0);
      compressorActive = false;
      compressorRun = millis();
    }
  }
}

void publishData() {
  if (temperature != lastTemp) {
    client.publish(temp_topic, String(temperature).c_str(), true);
    lastTemp = temperature;
  }
  if (humidity != lastHum) {
    client.publish(hum_topic, String(humidity).c_str(), true);
    lastHum = humidity;
  }
  if (tempSet != lastTempSet) {
    client.publish(temp_status_topic, String(tempSet).c_str(), true);
    lastTempSet = tempSet;
  }
  if (humSet != lastHumSet) {
    client.publish(hum_status_topic, String(humSet).c_str(), true);
    lastHumSet = humSet;
  }
  if (compressorActive != lastCompressorActive) {
    client.publish(compressor_status_topic, compressorActive ? "1" : "0", true);
    lastCompressorActive = compressorActive;
  }
  if (dehumifider != lastDehumifider) {
    client.publish(dehumifider_status_topic, dehumifider ? "1" : "0", true);
    lastDehumifider = dehumifider;
  }
  if (humifider != lastHumifider) {
    client.publish(humifider_status_topic, humifider ? "1" : "0", true);
    lastHumifider = humifider;
  }
  if (tempControl != lastTempcontrol) {
    client.publish(tempctr_status_topic, tempControl ? "1" : "0", true);
    lastTempcontrol = tempControl;
  }
  if (heatCtrl != lastHeatcontrol) {
    client.publish(heat_status_topic, heatCtrl ? "1" : "0", true);
    lastHeatcontrol = heatCtrl;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];

  if (String(topic) == set_temp_topic) {
    tempSet = constrain(message.toInt(), 0, 22);
     Serial.print("New Temp Set: "); Serial.println(tempSet);
  }
  if (String(topic) == set_hum_topic) {
    humSet = constrain(message.toInt(), 50, 95);
    Serial.print("New Hum Set: "); Serial.println(humSet);
  }
  if (String(topic) == set_topfanspeed_topic) {
    topfanSpeed = constrain(message.toInt(), 5, 100);
    Serial.print("New Top Fan Speed: "); Serial.println(topfanSpeed);
  }
}

void reconnectMQTT() {
  if (client.connected()) return;
  if (timeWait(3000, 5)) {
    lastReconnectAttempt = millis();
    if (client.connect(MQTT_DEVICE_NAME, MQTT_USER , MQTT_PASSWORD)) {
      client.subscribe(set_temp_topic);
      client.subscribe(set_hum_topic);
      client.subscribe(set_topfanspeed_topic);
    }
  }
}


bool timeWait(int i, byte z) {
  unsigned long previousMillis = 0;
  switch (z) {
    case 1: previousMillis = sensorRead; break;
    case 2: previousMillis = compressorRun; break;
    case 4: previousMillis = wifiCheck; break;
    case 5: previousMillis = lastReconnectAttempt; break;
  }
  return (millis() - previousMillis >= i);
}

void connectToWiFi() {
  WiFi.begin(WIFI_SSID , WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) delay(100);
}
