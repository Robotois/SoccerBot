#ifndef MESSAGEPROCESSOR
#define MESSAGEPROCESSOR



#endif

// #ifndef MQTT
// #define MQTT
// #include <WiFi.h>
// #include <PubSubClient.h>
// #include <ArduinoJson.h>
//
// const char* ssid = "AXTEL XTREMO-7969";
// const char* password = "03367969";
// const char* brokerAdd = "192.168.15.2";
// const char* clientId = "SoccerBot-01";
// // String myTeam = "team1";
// // String myId = "soccerBot1";
// String baseTopic = "SoccerBots";
// char *driveTopic;
//
// WiFiClient espClient;
// PubSubClient client(espClient);
//
// String getTopic(String team = "team1") {
//   return baseTopic + "/" + team + "/" + String(clientId);
// }
//
// void setupWifi() {
//   // We start by connecting to a WiFi network
//   Serial.println();
//   Serial.print("Connecting to: ");
//   Serial.println(ssid);
//
//   WiFi.begin(ssid, password);
//
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//
//   randomSeed(micros());
//
//   Serial.println("");
//   Serial.println("WiFi connected");
//   Serial.println("IP address: ");
//   Serial.println(WiFi.localIP());
// }
//
// /*
//   MQTT comunication
// */
//
// void robotMovemnt() {
//
// }
//
// void messageProcessor(char* topic, byte* payload, unsigned int length) {
//   Serial.print("Message arrived [");
//   Serial.print(topic);
//   Serial.print("]: ");
//   String msg = String((char*)payload);
//   Serial.println(msg);
//
//   StaticJsonBuffer<200> jsonBuffer;
//   JsonObject& root = jsonBuffer.parseObject(msg);
//   const char* message1 = root["m1"];
//   Serial.println("JSON[m1]: " + String(message1));
//
//   if(!root.success()) {
//     Serial.println("[JSON Error] -> parseObject error");
//     return;
//   }
// }
//
// void mqttConnect() {
//   client.setServer(brokerAdd, 1883);
//   client.setCallback(messageProcessor);
// }
//
// void reconnect() {
//   // Loop until we're reconnected
//   while (!client.connected()) {
//     Serial.print("Attempting MQTT connection...");
//     // Attempt to connect
//     if (client.connect(clientId)) {
//       Serial.println("connected");
//       // Once connected, publish an announcement...
//       // client.publish("outTopic", "hello world");
//       // ... and resubscribe
//       client.subscribe(getTopic().c_str());
//     } else {
//       Serial.print("failed, rc=");
//       Serial.println(client.state());
//       // Serial.println(" try again in 5 seconds");
//       // Wait 5 seconds before retrying
//       delay(1000);
//     }
//   }
// }
//
// void mqttLoop() {
//   if (!client.connected()) {
//     reconnect();
//   }
//   client.loop();
// }
//
// #endif
