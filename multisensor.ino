#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "Timer.h"
#include "Config.h"
#include <DHT.h>
#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

const String SENSORNAME = "multisensor-berlin-bad";
const PROGMEM char* IP_TOPIC =     "home-assistant/sensors/10/ip";
const PROGMEM char* LOGS_TOPIC =   "home-assistant/sensors/10/logs";
const PROGMEM char* STATE_TOPIC =  "home-assistant/sensors/10/state";
const PROGMEM char* HEALTH_TOPIC = "home-assistant/sensors/10/health";

#define LDRPIN    A0
#define PIRPIN    D8
#define DHTPIN    D7
#define DHTTYPE   DHT22

//test to remove those when it worked initially
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);
Timer t;
Adafruit_BMP280 bmp; // I2C
DHT dht(DHTPIN, DHTTYPE);

boolean hasPir = false;
boolean hasLdr = true;
boolean hasBmp = true;
boolean hasDht = false;

bool initial = true;
int ldr;
float diffLdr = 25;

float diffTemperature = 0.1;
float temperatureDHT = 0.0;
float temperatureBMP = 0.0;
float temperatureAvg = 0.0;

float diffHumidity = 1;
float humidity = 0.0;

float diffPressure = 20;
float pressure = 0.0;

float diffAltitude = 1;
float altitude = 0.0;

int pirValue;
int pirStatus;
String motionStatus = "standby";

const int BUFFER_SIZE = 222;

const PROGMEM char* NIGHT_MODE_TOPIC = "home-assistant/nightmode";
const PROGMEM char* DEBUG_MODE_TOPIC = "home-assistant/debug";
const PROGMEM char* PANIC_TOPIC = "home-assistant/panic";
const PROGMEM char* BLINDS_RESET_TOPIC = "home-assistant/blinds/reset";

void setup() {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting sensor node named " + String(SENSORNAME));
  
  setupPins();
  setupWifi();
  setupMqtt();
  setupOTA();
  setupTimer();
}

void setupPins() {
  if (hasPir) {
    pinMode(PIRPIN, INPUT);  
  }

  if (hasDht) {
    pinMode(DHTPIN, INPUT);  
  }

  if (hasLdr) {
    pinMode(LDRPIN, INPUT);
  }

  if (hasBmp) {
    Wire.begin(D3,D4); 
    if (!bmp.begin()) {  
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      while (1);
    }
  }
}

void setupWifi() {
  delay(10);
  WiFiManager wifiManager;
  wifiManager.setTimeout(300);

  //resetToFactoryDefaults();

  if (!wifiManager.autoConnect(SENSORNAME.c_str(), DEFAULT_PW)) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
}

void setupOTA() {
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(SENSORNAME.c_str());
  ArduinoOTA.setPassword(DEFAULT_PW);
  
  ArduinoOTA.onStart([]() {
    Serial.println("Starting");
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
}

void setupMqtt() {
  pubSubClient.setCallback(callback);
}

void setupTimer() {
  t.every(100, checkMotion);
  t.every(1000, checkSensors);
  t.every(600000, sendSensorState); //every 10 minutes
  
  t.every(10000, sendAlive);
  t.every(60000, publishIp);
}

void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }

  Serial.println("");
  Serial.print("Handle topic: '");
  Serial.print(p_topic);
  Serial.print("' with payload: '");
  Serial.print(payload);
  Serial.println("'");
}

void sendAlive() {
  bool sent = pubSubClient.publish(HEALTH_TOPIC, "alive", false);
  if (sent == true) {
    Serial.println("Successfully sent alive.");
  } else {
    Serial.println("Failed to send alive.");
  }
}

void sendSensorState() {
  if (temperatureDHT != 0 && temperatureBMP != 0) {
    temperatureAvg = (temperatureDHT + temperatureBMP) / 2;  
  } else if (temperatureDHT == 0 && temperatureBMP != 0) {
    temperatureAvg = temperatureBMP;
  } else if (temperatureDHT != 0 && temperatureBMP == 0) {
    temperatureAvg = temperatureDHT;
  } else {
    temperatureAvg = 0.00;
  }

  int brightness = 0;
  if (ldr != 0) {
    brightness = 1023 - ldr;  
  }

  float correctedPressure = 0.00;
  if (pressure != 0) {
    correctedPressure = (pressure + 500)/100; 
  }
  
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();
  root["temperature"] = (String)temperatureAvg;
  if (hasPir) {
    root["motion"] = (String)motionStatus;
  }

  if (hasBmp) {
    root["temperature_bmp"] = (String)temperatureBMP;
    root["pressure"] = (String)correctedPressure;
    root["altitude"] = (String)altitude;
  }

  if (hasDht) {
    root["temperature_dht"] = (String)temperatureDHT;  
    root["humidity"] = (String)humidity;  
  }

  if (hasLdr) {
    root["brightness"] = (String)brightness;  
  }
  
  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  bool publishedState = pubSubClient.publish(STATE_TOPIC, buffer, false);
  if (publishedState == true) {
    Serial.println("Successfully sent sensor states.");
  } else {
    Serial.println("Failed to send sensor states.");
  }
}

void reconnect() {
  while (!pubSubClient.connected()) {
    printState();
    if (connectToPrimary(false)) {
      subscribeToTopics();
    } else if (connectToPrimary(true)) {
      subscribeToTopics();
    } else if (connectToSecondary()) {
      subscribeToTopics();
    } else {
      Serial.println("DEBUG: try again in 5 seconds");
      delay(5000);
    }
  }
}

bool connectToPrimary(boolean fallbackPort) {
  uint16_t port = (fallbackPort == true) ? MQTT_SERVER_FALLBACK_PORT : MQTT_SERVER_PORT;

  Serial.print("Attempting primary MQTT connection to ");
  return doConnect(MQTT_SERVER_IP, port);
}

bool connectToSecondary() {
  Serial.print("Attempting secondary MQTT connection to ");
  return doConnect(MQTT_FALLBACK_SERVER_IP, MQTT_FALLBACK_SERVER_PORT);
}

bool doConnect(const char* ip, uint16_t port) {
  Serial.print(ip);
  Serial.print(":");
  Serial.print((String) port);
  Serial.println(" ... ");

  pubSubClient.setServer(ip, port);
  
  bool isConnected = pubSubClient.connect(SENSORNAME.c_str(), MQTT_USER, MQTT_PASSWORD);
  if (isConnected == true) {
    Serial.println("connected");
  } else {
    Serial.println("ERROR: failed, rc=" + pubSubClient.state());
    printState();
  }

  return isConnected;
}

bool printState() {
  switch (pubSubClient.state()) {
    case -4:
      Serial.println("Server didn't respond within the keepalive time");
      break;
    case -3:
      Serial.println("Network connection was broken");
      break;
    case -2:
      Serial.println("Network connection failed");
      break;
    case -1:
      Serial.println("Client is disconnected cleanly");
      break;
    case 0:
      Serial.println("Cient is connected");
      break;
    case 1:
      Serial.println("Server doesn't support the requested version of MQTT");
      break;
    case 2:
      Serial.println("Server rejected the client identifier");
      break;
    case 3:
      Serial.println("Server was unable to accept the connection");
      break;
    case 4:
      Serial.println("username/password were rejected");
      break;
    case 5:
      Serial.println("Client was not authorized to connect");
      break;
  }
}

void subscribeToTopics() {
  Serial.println("Subscribe to " + String(PANIC_TOPIC));
  pubSubClient.subscribe(PANIC_TOPIC);
  pubSubClient.loop();

  Serial.println("Subscribe to " + String(NIGHT_MODE_TOPIC));
  pubSubClient.subscribe(NIGHT_MODE_TOPIC);
  pubSubClient.loop();

  Serial.println("Subscribe to " + String(DEBUG_MODE_TOPIC));
  pubSubClient.subscribe(DEBUG_MODE_TOPIC);
  pubSubClient.loop();
}

bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}

void loop() {
  reconnect();
  pubSubClient.loop();
  
  ArduinoOTA.handle();
  
  t.update();
  
  if (initial == true) {
    publishIp();
    initial = false;  
  }
}

void checkMotion() {
  if (!hasPir) {
    return;
  }
  
  pirValue = digitalRead(PIRPIN);

  if (pirValue == LOW && pirStatus != 1) {
    motionStatus = "standby";
    sendSensorState();
    pirStatus = 1;
  } else if (pirValue == HIGH && pirStatus != 2) {
    motionStatus = "action";
    sendSensorState();
    pirStatus = 2;
  }
}

void checkSensors() {
  bool hasChanges = false;

  if (hasBmp) {
    float pressureNew = bmp.readPressure();
    if (checkBoundSensor(pressureNew, pressure, diffPressure)) {
      pressure = pressureNew;
      hasChanges = true;
    }
  
    float altitudeNew = bmp.readAltitude(1021);
    if (checkBoundSensor(altitudeNew, altitude, diffAltitude)) {
      altitude = altitudeNew;
      hasChanges = true;
    }  
    
    float temperatureBMPNew = bmp.readTemperature();
    if (checkBoundSensor(temperatureBMPNew, temperatureBMP, diffTemperature)) {
      temperatureBMP = temperatureBMPNew;
      hasChanges = true;
    }
  }
  
  if (hasDht) {
    float temperatureDHTNew = dht.readTemperature(); //to use celsius remove the true text inside the parentheses  
    if (checkBoundSensor(temperatureDHTNew, temperatureDHT, diffTemperature)) {
      temperatureDHT = temperatureDHTNew;
      hasChanges = true;
    }
  
    float humidityNew = dht.readHumidity();
    if (checkBoundSensor(humidityNew, humidity, diffHumidity)) {
      humidity = humidityNew;
      hasChanges = true;
    }  
  }

  if (hasLdr) {
    int ldrNew = analogRead(LDRPIN);
    if (checkBoundSensor(ldrNew, ldr, diffLdr)) {
      ldr = ldrNew;
      hasChanges = true;
    }  
  }

  if (hasChanges == true) {
    sendSensorState();
  }
}

String ipAddress2String(const IPAddress& ipAddress){
  return String(ipAddress[0]) + String(".") +\
    String(ipAddress[1]) + String(".") +\
    String(ipAddress[2]) + String(".") +\
    String(ipAddress[3]);
}

void publishIp() {
  String ipString = String(ipAddress2String(WiFi.localIP()));

  bool publishedIp = pubSubClient.publish(IP_TOPIC, ipString.c_str(), false);
  if (publishedIp == true) {
    Serial.println("Published IP");
  } else {
    Serial.println("Could not publish IP.");
  }
}

void resetToFactoryDefaults() {
  Serial.println("Reset to factory defaults");
  WiFi.disconnect();
  delay(3000);
}

