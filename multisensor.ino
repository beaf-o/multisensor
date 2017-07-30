#include <ESP8266WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "Timer.h"
#include "Config.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Timer t;

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

Adafruit_BMP280 bmp; // I2C

const PROGMEM char* STATE_TOPIC = "home-assistant/sensors/1";
const PROGMEM char* HEALTH_TOPIC = "home-assistant/sensors/1/health";
const PROGMEM char* IP_TOPIC = "home-assistant/sensors/1/ip";

const String SENSORNAME = "sensornode1";

#define PIRPIN    D2
#define DHTPIN    D7
#define DHTTYPE   DHT22
#define LDRPIN    A0

int ldr;
float diffLdr = 25;

float diffTemperature = 0.1;
float temperatureDHT;
float temperatureBMP;
float temperatureAvg;

float diffHumidity = 1;
float humidity;

float diffPressure = 20;
float pressure;

float diffAltitude = 1;
float altitude;

int pirValue;
int pirStatus;
String motionStatus;



char message_buff[100];
const int BUFFER_SIZE = 300;
#define MQTT_MAX_PACKET_SIZE 512

WiFiClient espClient;
PubSubClient pubSubClient(espClient);
DHT dht(DHTPIN, DHTTYPE);

/********************************** START SETUP*****************************************/
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
  Wire.begin(D3,D4); 
  if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  pinMode(PIRPIN, INPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(LDRPIN, INPUT);

  delay(10);
}

void setupWifi() {
  delay(10);
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);

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
  t.every(10000, checkSensors);
  t.every(600000, sendState);
  t.every(10000, sendAlive); // every 10 seconds
  t.every(660000, publishIp); // every 11 minutes
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);

  if (!processJson(message)) {
    return;
  }

  sendState();
}

bool processJson(char* message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }

  return true;
}

void sendAlive() {
  bool sent = pubSubClient.publish(HEALTH_TOPIC, "alive", true);
  if (sent == true) {
    Serial.println("Successfully sent alive.");
  } else {
    Serial.println("Failed to send alive.");
  }
}

void sendState() {
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
  root["temperature_dht"] = (String)temperatureDHT;
  root["temperature_bmp"] = (String)temperatureBMP;
  root["humidity"] = (String)humidity;
  root["brightness"] = (String)brightness;
  root["pressure"] = (String)correctedPressure;
  root["altitude"] = (String)altitude;
  
  root["motion"] = (String)motionStatus;

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  pubSubClient.publish(STATE_TOPIC, buffer, true);
}

void reconnect() {
  while (!pubSubClient.connected()) {
    printState();
    if (connectToPrimary()) {
      return;
    } else if (connectToSecondary()) {
      return;
    } else {
      Serial.println("DEBUG: try again in 5 seconds");
      delay(5000);
    }
  }
}

bool connectToPrimary() {
  Serial.print("Attempting primary MQTT connection to ");
  Serial.print(String(MQTT_SERVER_IP));
  Serial.print(":");
  Serial.print(String(MQTT_SERVER_PORT));
  Serial.println(" ... ");

  pubSubClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  return doConnect();
}

bool connectToSecondary() {
  pubSubClient.setServer(MQTT_FALLBACK_SERVER_IP, MQTT_FALLBACK_SERVER_PORT);
  
  Serial.print("Attempting secondary MQTT connection to ");
  Serial.print(String(MQTT_FALLBACK_SERVER_IP));
  Serial.print(":");
  Serial.print(String(MQTT_FALLBACK_SERVER_PORT));
  Serial.println(" ... ");

  return doConnect();
}

bool doConnect() {
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

bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}

void loop() {
  reconnect();
  ArduinoOTA.handle();
  t.update();
}

void checkMotion() {
  pirValue = digitalRead(PIRPIN); //read state of the

  if (pirValue == LOW && pirStatus != 1) {
    motionStatus = "standby";
    sendState();
    pirStatus = 1;
  } else if (pirValue == HIGH && pirStatus != 2) {
    motionStatus = "action";
    sendState();
    pirStatus = 2;
  }
}

void checkSensors() {
  bool hasChanges = false;
  
  float temperatureBMPNew = bmp.readTemperature();
  if (checkBoundSensor(temperatureBMPNew, temperatureBMP, diffTemperature)) {
    temperatureBMP = temperatureBMPNew;
    hasChanges = true;
  }

  float temperatureDHTNew = dht.readTemperature(); //to use celsius remove the true text inside the parentheses  
  if (checkBoundSensor(temperatureDHTNew, temperatureDHT, diffTemperature)) {
    temperatureDHT = temperatureDHTNew;
    hasChanges = true;
  }

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

  float humidityNew = dht.readHumidity();
  if (checkBoundSensor(humidityNew, humidity, diffHumidity)) {
    humidity = humidityNew;
    hasChanges = true;
  }

  int ldrNew = analogRead(LDRPIN);
  if (checkBoundSensor(ldrNew, ldr, diffLdr)) {
    ldr = ldrNew;
    hasChanges = true;
  }

  if (hasChanges == true) {
    sendState();
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

