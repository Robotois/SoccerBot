#include "PIDs.h"

uint8_t ledPin = 5;
uint8_t encAPin = 15;
uint8_t encBPin = 4;

void setup() {
  motorInit(encAPin, encBPin, ledPin, 0);
  pidTimerInit();
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("ESP32 SoccerBot");
  setMotorSpeed(50, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  // if (prevCount != encCount[0]) {
  //   // Serial.println("Encoder Counter: " + String(encCount[0]));
  //   prevCount = encCount[0];
  // }
  pidControl();
}


// void blink() {
//   state = !state;
//
//   digitalWrite(ledPin, state);
// }
