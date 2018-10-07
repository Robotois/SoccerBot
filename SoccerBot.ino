#include "PIDs.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "AXTEL XTREMO-7969";
const char* password = "03367969";
const char* brokerAdd = "192.168.15.5";
const char* clientId = "SoccerBot-01";
String driveTopic = "SoccerBots/tablet-01/soccerBot-01/drive";

WiFiClient espClient;
PubSubClient client(espClient);

uint8_t motorIdx = 0;
uint8_t pwmPin = 2;
uint8_t encAPin = 15;
uint8_t encBPin = 4;
uint8_t cwPin = 16;
uint8_t ccwPin = 17;

void setupWifi() {
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe(driveTopic.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      // Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
}

void mqttLoop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void drive(byte* payload) {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(payload);

  if(!root.success()) {
    Serial.println("[JSON Error] -> parseObject error");
    return;
  }

  float x = root["x"];
  float y = root["y"];
  setMotorPWM(x * 1024, 0);
  setMotorPWM( - x * 1024, 2);
  setMotorPWM(y * 1024, 1);
  setMotorPWM( - y * 1024, 3);
  float rotation = root["rotation"];
  // Serial.println("JSON Parsed: { x: " + String(x) + ", y: " + String(y) + ", rotation: " + String(rotation) + " }");
}

void leds(byte* payload) {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(payload);

  if(!root.success()) {
    Serial.println("[JSON Error] -> parseObject error");
    return;
  }
  uint8_t celebrate = root["celebrate"];
  Serial.println("JSON Parsed: { celebrate: " + String(celebrate) + " }");
}

void messageProcessor(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);

  // Serial.print("[" + topicStr + "]: ");
  // String msg = String((char*)payload);
  // Serial.println(msg);

  if(topicStr.equals(driveTopic)) {
    drive(payload);
    return;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 SoccerBot");
  setupWifi();
  client.setServer(brokerAdd, 1883);
  client.setCallback(messageProcessor);

  motorsInit(0, 15, 2, 0);
  motorsInit(4, 16, 17, 1);
  motorsInit(5, 18, 19, 2);
  motorsInit(21, 22, 23, 3);
  // motorInit(encAPin, encBPin, pwmPin, cwPin, ccwPin, motorIdx);
  // pirControlInit();
  // setMotorSpeed(-50, 0);
}

void loop() {
  // pidControl();
  mqttLoop();
}


/*
  MQTT comunication
*/
