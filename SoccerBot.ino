#include "PIDs.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "Wireless-N";
const char* password = "robotois8899";
const char* brokerAdd = "192.168.10.103";
const char* clientId = "SoccerBot-01";
String driveTopic = "SoccerBots/tablet-01/soccerBot-01/drive";
unsigned long timerr;

WiFiClient espClient;
PubSubClient client(espClient);



uint8_t motorIdx = 0;
uint8_t pwmPin = 2;
uint8_t encAPin = 15;
uint8_t encBPin = 4;
uint8_t cwPin = 16;
uint8_t ccwPin = 17;

void setupWifi() {

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  randomSeed(micros());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    //Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId)) {
      //Serial.println("connected");
      // Once connected, publish an announcement...
      // client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe(driveTopic.c_str());
    } else {
      //Serial.print("failed, rc=");
      //Serial.println(client.state());
      // //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
}

void mqttLoop() {
  if (!client.connected()) {
    setMotorPWM(0, 0);
    setMotorPWM(0, 2);
    setMotorPWM(0, 1);
    setMotorPWM(0, 3);
    reconnect();
  }
  client.loop();
}

void drive(byte* payload) {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(payload);

  if(!root.success()) {
    //Serial.println("[JSON Error] -> parseObject error");
    return;
  }

  float x = root["x"];
  float y = root["y"];
  setMotorPWM(x * 1024, 0);
  setMotorPWM( - x * 1024, 2);
  setMotorPWM(y * 1024, 1);
  setMotorPWM( - y * 1024, 3);
  float rotation = root["rotation"];
  // //Serial.println("JSON Parsed: { x: " + String(x) + ", y: " + String(y) + ", rotation: " + String(rotation) + " }");
}

void leds(byte* payload) {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(payload);

  if(!root.success()) {
    //Serial.println("[JSON Error] -> parseObject error");
    return;
  }
  uint8_t celebrate = root["celebrate"];
  //Serial.println("JSON Parsed: { celebrate: " + String(celebrate) + " }");
}

void messageProcessor(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);

  // //Serial.print("[" + topicStr + "]: ");
  // String msg = String((char*)payload);
  // //Serial.println(msg);

  // set message timestamp
  timerr = millis();
  if(topicStr.equals(driveTopic)) {
    drive(payload);
    return;
  }
}

void setup() {
  //Serial.begin(115200);
  //Serial.println("ESP32 SoccerBot");
  setupWifi();
  client.setServer(brokerAdd, 1883);
  client.setCallback(messageProcessor);

  motorsInit(21, 3, 1, 0);
  motorsInit(0, 15, 2, 1);
  motorsInit(4, 16, 17, 2);
  motorsInit(5, 18, 19, 3);
  // motorInit(encAPin, encBPin, pwmPin, cwPin, ccwPin, motorIdx);
  // pirControlInit();
  // setMotorSpeed(-50, 0);
}

void loop() {
  // pidControl();
  if((millis()) - timerr > 1200) {    
    setMotorPWM(0, 0);
    setMotorPWM(0, 2);
    setMotorPWM(0, 1);
    setMotorPWM(0, 3);
   }
  mqttLoop();
}


/*
  MQTT comunication
*/
