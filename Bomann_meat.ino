// === Libraries ===
#include "Adafruit_SHT31.h"
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "arduino_secrets.h"

// === MQTT Topics ===
const char* temp_topic                = "sushilna/temperature";
const char* hum_topic                 = "sushilna/humidity";
const char* set_hum_topic             = "sushilna/set/humidity";
const char* set_temp_topic            = "sushilna/set/temperature";
const char* temp_status_topic         = "sushilna/status/temperature";
const char* hum_status_topic          = "sushilna/status/humidity";
const char* set_topfanspeed_topic     = "sushilna/set/topfanspeed";
const char* compressor_status_topic   = "sushilna/compressor";
const char* humifider_status_topic    = "sushilna/humifider";
const char* dehumifider_status_topic  = "sushilna/dehumifider";
const char* tempctr_status_topic      = "sushilna/temp_control";
const char* heat_status_topic         = "sushilna/status/heater";

// === WiFi / MQTT ===
WiFiClient espClient;
PubSubClient client(espClient);

// === Sensor ===
Adafruit_SHT31 sht31          = Adafruit_SHT31();

// === Variables ===
int temperature               = 0;
byte humidity                 = 0;

byte tempSet                  = 13;
byte humSet                   = 75;
byte topfanSpeed              = 40;
//byte topFanSpd                = 0;

// === States ===
bool compressorActive         = false;
bool allowHumidityControl     = true;
bool fanBoostActive           = true;
bool dehumifider              = false;
bool humifider                = false;
bool tempControl              = false;
bool heatCtrl                 = false;
bool fanIsOn                  = true;
bool onlyFridge               = false;

// === Pins ===
const byte heaterPin          = 4;
const byte compPin            = 26;
const byte cFanPin            = 27;
const byte circFanPin          = 14;
const byte heatFanPin          = 25;

// === Fan timing ===
unsigned long previousMillisCircFan;
const unsigned long intervalOn  = 300000;
const unsigned long intervalOff = 900000;

// =======================================================
// SETUP
// =======================================================
void setup() {
  pinMode(compPin, OUTPUT);
  digitalWrite(compPin, LOW);
  pinMode(heaterPin, OUTPUT);
  digitalWrite(heaterPin, LOW);

  Serial.begin(115200);
  delay(500);

  //topFanSpd = map(topfanSpeed, 5, 100, 14, 254);

  // PWM (ESP32 core 3.x)
  ledcAttach(circFanPin, 25000, 8);
  ledcAttach(cFanPin, 25000, 8);
  ledcAttach(heatFanPin, 25000, 8);
  ledcWrite(circFanPin, /*topFanSpd*/ 40);
  ledcWrite(cFanPin, 0);
  ledcWrite(heatFanPin, 0);

  previousMillisCircFan = 0;


  sht31.begin(0x44);

  WiFi.mode(WIFI_STA);
  connectToWiFi();

  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(mqttCallback);
}

// =======================================================
// LOOP
// =======================================================
void loop() {

  //topFanSpd = map(topfanSpeed, 5, 100, 14, 254);
  if (tempSet < 9){
    onlyFridge = true;
    ledcWrite(heatFanPin, 75);
  } else {
    onlyFridge = false;
  }

  if (!client.connected()) reconnectMQTT();
  else client.loop();

  checkWiFi();
  sht3xRead();
  circFan();
  tempCtrl();
  humCtrl();
  publishData();
}

// =======================================================
// FUNCTIONS
// =======================================================

void sht3xRead() {
  static unsigned long lastRead = 0;
  if ((millis() - lastRead) > 5000){
    lastRead = millis();

    float t = sht31.readTemperature();
    float h = sht31.readHumidity();

    if (!isnan(t) && t >= -10 && t <= 60) temperature = (int)t;
    if (!isnan(h) && h >= 0 && h <= 100) humidity = (byte)h;
  }
}

void checkWiFi() {
  static unsigned long lastCheck = 0;
  if ((millis() - lastCheck ) > 30000){
    lastCheck = millis();

    if (WiFi.status() != WL_CONNECTED) connectToWiFi();
  }
}

void tempCtrl() {
  static bool chk = false;

  if (temperature >= tempSet + 2) {
    compressor(true);
    chk = true ;
    allowHumidityControl = false;
    tempControl = true;
  }
  else if (temperature <= tempSet) {
    if (chk){
      compressor(false);
      chk = false ;
    }
    allowHumidityControl = true;
    tempControl = false;
  }

  if (temperature <= tempSet - 2) {
    digitalWrite(heaterPin, HIGH);
    if (!onlyFridge)ledcWrite(heatFanPin, 30);
    heatCtrl = true;
  }
  else if (temperature >= tempSet) {
    digitalWrite(heaterPin, LOW);
    if (!onlyFridge)ledcWrite(heatFanPin, 0);
    heatCtrl = false;
  }
}

void humCtrl() {
  if (allowHumidityControl && !onlyFridge){

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
    else if (humidity >= humSet && fanBoostActive) {
      ledcWrite(cFanPin, 0);
      fanBoostActive = false;
      humifider = false;
     }
  } else {
      humifider = false;
      dehumifider = false;
  }
}

void compressor(bool state) {
  static unsigned long lastStop = 0;
  if ((millis() - lastStop ) > 180000) {
      if (state) { 
        digitalWrite(compPin, HIGH);
        ledcWrite(cFanPin, 150);
        compressorActive = true;
    } else {
        digitalWrite(compPin, LOW);
        if (compressorActive) {
          compressorActive = false;
          ledcWrite(cFanPin, 0);
          lastStop = millis();
      }
    }
  } else {
    if (compressorActive) {
    compressorActive = false;
    ledcWrite(cFanPin, 0);
    }
  }
}

void circFan() {
  if (onlyFridge) return;

  unsigned long now = millis();
  if (fanIsOn) {
    if ((now - previousMillisCircFan) > intervalOn) {
      fanIsOn = false;
      previousMillisCircFan = now;
      ledcWrite(circFanPin, 0);
    }
  } else {
    if ((now - previousMillisCircFan) > intervalOff) {
      fanIsOn = true;
      previousMillisCircFan = now;
      ledcWrite(circFanPin, /*topFanSpd*/42);
    }
  }
}

void publishData() {
  // === Last published ===
  static int lastTemp = -100;
  static byte lastHum = 150;
  static int lastTempSet  = -100;
  static byte lastHumSet  = 150;
  static bool lastHeatcontrol = false;
  static bool lastDehumifider = false;
  static bool lastHumifider = false;
  static bool lastTempcontrol = false;
  static bool lastCompressorActive  = false;

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
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == set_temp_topic) tempSet = constrain(msg.toInt(), 0, 22); Serial.print("New Temp Set: "); Serial.println(tempSet);
    
  if (String(topic) == set_hum_topic) humSet = constrain(msg.toInt(), 50, 95); Serial.print("New Hum Set: "); Serial.println(humSet);

  //if (String(topic) == set_topfanspeed_topic) topfanSpeed = constrain(msg.toInt(), 5, 100); Serial.print("New Top Fan Speed: "); Serial.println(topfanSpeed);
}

void reconnectMQTT() {
  if (client.connected()) return;
    static unsigned long lastTry = 0;
    if ((millis() - lastTry ) > 3000){
      lastTry = millis();

      if (client.connect(MQTT_DEVICE_NAME, MQTT_USER, MQTT_PASSWORD)) {
        client.subscribe(set_temp_topic);
        client.subscribe(set_hum_topic);
        client.subscribe(set_topfanspeed_topic);
      }
  }
}

void connectToWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) delay(100);
}
