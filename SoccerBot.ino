#include "PIDs.h"

uint8_t motorIdx = 0;
uint8_t pwmPin = 2;
uint8_t encAPin = 15;
uint8_t encBPin = 4;
uint8_t cwPin = 16;
uint8_t ccwPin = 17;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 SoccerBot");

  motorInit(encAPin, encBPin, pwmPin, cwPin, ccwPin, motorIdx);
  pirControlInit();
  setMotorSpeed(-50, 0);
}

void loop() {
  pidControl();
}
