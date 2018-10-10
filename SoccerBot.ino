#include "PIDs.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "Wireless-N";
const char* password = "robotois8899";
const char* brokerAdd = "192.168.10.101";
const char* clientId = "SoccerBot-01";
String driveTopic = "SoccerBots/tablet-01/soccerBot-01/drive";
unsigned long timerr;

WiFiClient espClient;
PubSubClient client(espClient);

// x, y, r
float tupla[3] = {0, 0, 0};

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
      client.subscribe(driveTopic.c_str(), 1);
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

void centerOffset(float theta = 45) {
  float x, y;
  // x = x * cos \theta - y * sin \theta
  // y = x * sin \theta + y * cos \theta
  x = tupla[0] * cos(theta) - tupla[1] * sin(theta);
  y = tupla[0] * sin(theta) + tupla[1] * cos(theta);
  tupla[0] = x;
  tupla[1] = y;
}

void drive(String payload, unsigned int length) {
  uint8_t prevIdx = 0;
  uint8_t tuplaIdx = 0;
  for (uint8_t i = 0; i <= length; i++) {
    if(payload.charAt(i) == ',' || i == length) {
      tupla[tuplaIdx] = payload.substring(prevIdx, i).toFloat();
      tuplaIdx++;
      prevIdx = i + 1;
    }
  }
  centerOffset();
  setMotorPWM((tupla[0] - tupla[2] * 0.35) * 1024, 0);
  setMotorPWM((- tupla[0] - tupla[2] * 0.35) * 1024, 2);
  setMotorPWM((tupla[1] - tupla[2] * 0.35) * 1024, 1);
  setMotorPWM((- tupla[1] - tupla[2] * 0.35) * 1024, 3);
}

// void leds(byte* payload) {
//   StaticJsonBuffer<200> jsonBuffer;
//   JsonObject& root = jsonBuffer.parseObject(payload);
//
//   if(!root.success()) {
//     //Serial.println("[JSON Error] -> parseObject error");
//     return;
//   }
//   uint8_t celebrate = root["celebrate"];
//   //Serial.println("JSON Parsed: { celebrate: " + String(celebrate) + " }");
// }

void messageProcessor(char* topic, byte* payload, unsigned int length) {
  timerr = millis();
  String msg = String((char*)payload);
  // Serial.println(msg);

  // set message timestamp
  drive(msg, length);
}

void setup() {
  // Serial.begin(115200);
  // Serial.println("ESP32 SoccerBot");
  setupWifi();
  client.setServer(brokerAdd, 1883);
  client.setCallback(messageProcessor);

  /*OLD PINES*/
  motorsInit(0, 15, 2, 0);
  motorsInit(4, 16, 17, 1);
  motorsInit(5, 18, 19, 2);
  motorsInit(21, 22, 23, 3);
  /*NEW PINES*/
  /*motorsInit(21, 3, 1, 0);
  motorsInit(0, 15, 2, 1);
  motorsInit(4, 16, 17, 2);
  motorsInit(5, 18, 19, 3);*/

  // motorInit(encAPin, encBPin, pwmPin, cwPin, ccwPin, motorIdx);
  // pirControlInit();
  // setMotorSpeed(-50, 0);
}

void loop() {
  // pidControl();
  if((millis()) - timerr > 1000) {
    setMotorPWM(0, 0);
    setMotorPWM(0, 2);
    setMotorPWM(0, 1);
    setMotorPWM(0, 3);
   }
  mqttLoop();
}
