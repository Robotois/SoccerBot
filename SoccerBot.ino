#include "PIDs.h"
#include <WiFi.h>
// #include <WiFiMulti.h>
// #include <WiFiClientSecure.h>
// #include <WebSocketsServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// #define USE_SERIAL

const char* ssid = "robotoisAP";
const char* password = "robotois8899";
const char* brokerAdd = "192.168.50.27";
const char* clientId = "SoccerBot-01";
String driveTopic = "SoccerBots/tablet-01/soccerBot-01/drive";
unsigned long timerr = 0;

// WiFiMulti WiFiMulti;
// WebSocketsServer webSocket = WebSocketsServer(81);

WiFiClient espClient;
PubSubClient client(espClient);
// WiFiServer server(80);
// WebSocketServer webSocketServer;

// x, y, r
float x = 0, y = 0, r = 0;
float thetaSin = sin(-45.0f), thetaCos = cos(-45.0f);

void motorsStop() {
  setMotorPWM(0, 0);
  setMotorPWM(0, 2);
  setMotorPWM(0, 1);
  setMotorPWM(0, 3);
}

void motorsStart() {
  /*NEW PINES*/
  /*motorsInit(0, 15, 2, 0);
  motorsInit(4, 16, 17, 1);
  motorsInit(5, 18, 19, 2);
  motorsInit(21, 22, 23, 3);*/
  /*OLD PINES*/
  motorsInit(21, 3, 1, 0);
  motorsInit(0, 15, 2, 1);
  motorsInit(4, 16, 17, 2);
  motorsInit(5, 18, 19, 3);
}

void setupWifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    #ifdef USE_SERIAL
    Serial.println("Connecting to WiFi..");
    #endif
    delay(1000);
  }
  #ifdef USE_SERIAL
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
  #endif
  randomSeed(micros());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    #ifdef USE_SERIAL
    //Serial.print("Attempting MQTT connection...");
    #endif
    // Attempt to connect
    if (client.connect(clientId)) {
      #ifdef USE_SERIAL
      Serial.println("connected");
      #endif
      // Once connected, publish an announcement...
      // client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe(driveTopic.c_str());
    } else {
      #ifdef USE_SERIAL
      Serial.print("failed, rc=");
      Serial.println(client.state());
      #endif
      // //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
}

void mqttLoop() {
  if (!client.connected()) {
    motorsStop();
    reconnect();
  }
  client.loop();
}

void drive(byte* payload) {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(payload);

  if(!root.success()) {
    return;
  }

  float xTemp = root["x"];
  float yTemp = root["y"];
  r = root["r"];

  // Center Offset -45 degree
  // x = x * cos \theta - y * sin \theta
  // y = x * sin \theta + y * cos \theta
  x = xTemp * thetaCos - yTemp * thetaSin;
  y = xTemp * thetaSin + yTemp * thetaCos;

  setMotorPWM((-x - r * 0.30) * 1024, 0);
  setMotorPWM((x - r * 0.30) * 1024, 2);
  setMotorPWM((y - r * 0.30) * 1024, 1);
  setMotorPWM((-y - r * 0.30) * 1024, 3);
  // #ifdef USE_SERIAL
  // Serial.println("JSON Parsed: { x: " + String(x) + ", y: " + String(y) + ", r: " + String(r) + " }");
  // #endif
}

void messageProcessor(char* topic, byte* payload, unsigned int length) {
  timerr = millis();
  #ifdef USE_SERIAL
  String msg = String((char*)payload);
  Serial.println(msg);
  #endif
  drive(payload);
}

void setup() {
  #ifdef USE_SERIAL
  Serial.begin(115200);
  Serial.println("ESP32 SoccerBot");
  Serial.setDebugOutput(true);
  #endif
  // for(uint8_t t = 4; t > 0; t--) {
  //     #ifdef USE_SERIAL
  //     Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
  //     Serial.flush();
  //     #endif
  //     delay(1000);
  // }
  //
  // WiFiMulti.addAP("AXTEL XTREMO-7969", "03367969");
  //
  // while(WiFiMulti.run() != WL_CONNECTED) {
  //   #ifdef USE_SERIAL
  //   Serial.println("Connecting to WiFi..");
  //   #endif
  //   delay(500);
  // }
  // #ifdef USE_SERIAL
  // Serial.println("Connected to the WiFi network:");
  // Serial.println(WiFi.localIP());
  // #endif
  // webSocket.begin();
  // webSocket.onEvent(webSocketEvent);

  setupWifi();
  client.setServer(brokerAdd, 1883);
  client.setCallback(messageProcessor);

  motorsStart();
}

void loop() {
  if((millis()) - timerr > 1000) {
    motorsStop();
   }
  // webSocket.loop();
  mqttLoop();
}


// void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
// 	const uint8_t* src = (const uint8_t*) mem;
// 	Serial.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
// 	for(uint32_t i = 0; i < len; i++) {
// 		if(i % cols == 0) {
// 			Serial.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
// 		}
// 		Serial.printf("%02X ", *src);
// 		src++;
// 	}
// 	Serial.printf("\n");
// }

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
// void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
//     switch(type) {
//       case WStype_DISCONNECTED:
//         #ifdef USE_SERIAL
//         Serial.printf("[%u] Disconnected!\n", num);
//         #endif
//         break;
//       case WStype_CONNECTED:
//         {
//           IPAddress ip = webSocket.remoteIP(num);
//           #ifdef USE_SERIAL
//           Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
//           #endif
//   				// send message to client
//   				webSocket.sendTXT(num, "Welcome Dear!");
//         }
//         break;
//       case WStype_TEXT:
//         timerr = millis();
//         #ifdef USE_SERIAL
//         Serial.printf("[%u] get Text: %s\n", num, payload);
//         #endif
//         drive(payload);
//         break;
//       case WStype_BIN:
//   		case WStype_ERROR:
//   		case WStype_FRAGMENT_TEXT_START:
//   		case WStype_FRAGMENT_BIN_START:
//   		case WStype_FRAGMENT:
//   		case WStype_FRAGMENT_FIN:
//   		  break;
//     }
// }
