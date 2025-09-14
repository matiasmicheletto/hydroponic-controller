#include <Arduino.h>
#include <WiFi.h>
#include "WebSocketsServer.h"
#include <Wire.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include "GravityTDS.h"
#include "esp_timer.h"


/* Define WiFi credentials in a separate file named credentials.h
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
*/
#include "credentials.h"
#include "config.h"



GravityTDS gtds;
DHT dht(DHT_PIN, DHT_TYPE);

WebSocketsServer web_socker_server(WSS_PORT);  
uint8_t client_num; // Number of clients 
bool wss_connected = false;

volatile uint32_t timertick = 0; 
hw_timer_t * timer = NULL;

// Sensor readings
struct Sensor { 
  float temperature; // DHT22
  int humidity; // DHT22
  float ppm; // TDS
  float ec25; // TDS (converted to conductivity)
} sensor = {0.0, 0, 0.0, 0.0};

static unsigned long lastSensorRead = 0; // Timer for sensor acquiring and readings update

void readSensors() {
// DHT22 reading (temperature and humidity)
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) sensor.temperature = t;
  if (!isnan(h)) sensor.humidity = (int) h;

  // TDS sensor reading (conductivity)
  gtds.setTemperature(sensor.temperature);
  gtds.update();
  sensor.ppm = gtds.getTdsValue();
  sensor.ec25 = gtds.getEcValue();
}


void webSocketEvent(uint8_t clientNum, WStype_t type, uint8_t *payload, size_t length) {
  switch(type) {
    case WStype_CONNECTED: {
      IPAddress ip = webSocketServer.remoteIP(clientNum);
      #ifdef DEBUG
      Serial.printf("WebSocket client connected: %u, IP: %s\n", clientNum, ip.toString().c_str());
      #endif
      break;
    }
    case WStype_DISCONNECTED: {
      #ifdef DEBUG
      Serial.printf("WebSocket client disconnected: %u\n", clientNum);
      #endif
      break;
    }
    case WStype_TEXT: {
      // handle incoming commands, e.g., update sampling period
      break;
    }
  }
}

void IRAM_ATTR on_timer_tick() {
  timertick++;
}

void setup() {
  #ifdef DEBUG
  Serial.begin(BAUDRATE);
  #endif

  // Wi-Fi connect with retry
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 50) {
    delay(500);
    attempts++;
    #ifdef DEBUG
    Serial.print(".");
    #endif
  }
  if (WiFi.status() != WL_CONNECTED) {
    #ifdef DEBUG
    Serial.println("Failed to connect to Wi-Fi");
    #endif
    return;
  }
  #ifdef DEBUG
  Serial.print("Connected to Wi-Fi. IP: ");
  Serial.println(WiFi.localIP());
  #endif

  // Initialize sensors
  dht.begin();
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogSetAttenuation(ADC_11db);
  gtds.setPin(TDS_INPUT_PIN);
  gtds.setAref(TDS_A_REF);
  gtds.setAdcRange(ADC_RANGE);
  gtds.setTemperature(25);

  // Start WebSocket server
  webSocketServer.begin();
  webSocketServer.onEvent(webSocketEvent);

  // Initial sensor read
  readSensors();
  lastSensorRead = millis();
}


void loop() {
  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_READ_PERIOD) {
    readSensors();
    lastSensorRead = now;

    // Prepare JSON using ArduinoJson
    StaticJsonDocument<128> jsonDoc;
    jsonDoc["temperature"] = sensor.temperature;
    jsonDoc["humidity"] = sensor.humidity;
    jsonDoc["ppm"] = sensor.ppm;
    jsonDoc["ec25"] = sensor.ec25;
    String jsonStr;
    serializeJson(jsonDoc, jsonStr);

    // Broadcast to all connected clients
    webSocketServer.broadcastTXT(jsonStr);
  }

  webSocketServer.loop();
}
