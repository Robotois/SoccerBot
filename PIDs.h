// #include <stdint.h>
#ifndef PIDS
#define PIDS

const uint8_t CW_DIR = 1;
const uint8_t CCW_DIR = 2;

uint8_t pwmPins[4] = {};
uint8_t encoderPins[4][2] = {};
uint8_t cwPins[4] = {};
uint8_t ccwPins[4] = {};
uint8_t motorCount = 0;

int pwmFreq = 5000, resolution = 10;

int boundValue(int value, int min, int max) {
  if(value < min) {
    return min;
  }
  if(value > max) {
    return max;
  }
  return value;
}

/*
  Motors
*/
const int lookUpTable[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
volatile int encCount[4] = {0, 0, 0, 0}, prevEncCount[4] = {0, 0, 0, 0};
volatile uint8_t prevEnc[4] = {0, 0, 0, 0};

void encThick(uint8_t currentEnc, uint8_t motorNumber) {
    uint8_t lookAdd = (prevEnc[motorNumber] <<2) | currentEnc;
    prevEnc[motorNumber] = currentEnc;
    encCount[motorNumber] += lookUpTable[lookAdd];
}

void encoderTick0() {
  uint8_t currentEnc = digitalRead(encoderPins[0][0]) << 1 | digitalRead(encoderPins[0][1]);
  encThick(currentEnc, 0);
}
void encoderTick1() {
  uint8_t currentEnc = digitalRead(encoderPins[1][0]) << 1 | digitalRead(encoderPins[1][1]);
  encThick(currentEnc, 1);
}
void encoderTick2() {
  uint8_t currentEnc = digitalRead(encoderPins[2][0]) << 1 | digitalRead(encoderPins[2][1]);
  encThick(currentEnc, 2);
}
void encoderTick3() {
  uint8_t currentEnc = digitalRead(encoderPins[3][0]) << 1 | digitalRead(encoderPins[3][1]);
  encThick(currentEnc, 3);
}

void encInterrupt(uint8_t encAPin, uint8_t encBPin, uint8_t index) {
  switch (index) {
    case 0:
      attachInterrupt(digitalPinToInterrupt(encAPin), encoderTick0, CHANGE);
      attachInterrupt(digitalPinToInterrupt(encBPin), encoderTick0, CHANGE);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(encAPin), encoderTick1, CHANGE);
      attachInterrupt(digitalPinToInterrupt(encBPin), encoderTick1, CHANGE);
      break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(encAPin), encoderTick2, CHANGE);
      attachInterrupt(digitalPinToInterrupt(encBPin), encoderTick2, CHANGE);
      break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(encAPin), encoderTick3, CHANGE);
      attachInterrupt(digitalPinToInterrupt(encBPin), encoderTick3, CHANGE);
      break;
  }
}

void motorInit(
  uint8_t encAPin,
  uint8_t encBPin,
  uint8_t pwmPin,
  uint8_t cwPin,
  uint8_t ccwPin,
  uint8_t motorIdx
) {
  pwmPins[motorIdx] = pwmPin;
  encoderPins[motorIdx][0] = encAPin;
  encoderPins[motorIdx][1] = encBPin;
  cwPins[motorIdx] = cwPin;
  ccwPins[motorIdx] = ccwPin;

  pinMode(encAPin, INPUT_PULLUP);
  pinMode(encBPin, INPUT_PULLUP);
  encInterrupt(encAPin, encBPin, motorIdx);

  pinMode(cwPin, OUTPUT);
  pinMode(ccwPin, OUTPUT);
  digitalWrite(cwPin, LOW);
  digitalWrite(ccwPin, LOW);

  pinMode(pwmPin, OUTPUT);
  ledcSetup(motorIdx, pwmFreq, resolution);
  ledcAttachPin(pwmPin, motorIdx);
  ledcWrite(motorIdx, 0);

  motorCount++;
}

void motorsInit(
  uint8_t cwPin,
  uint8_t ccwPin,
  uint8_t pwmPin,
  uint8_t motorIdx
) {
  pwmPins[motorIdx] = pwmPin;
  cwPins[motorIdx] = cwPin;
  ccwPins[motorIdx] = ccwPin;

  pinMode(cwPin, OUTPUT);
  pinMode(ccwPin, OUTPUT);
  digitalWrite(cwPin, LOW);
  digitalWrite(ccwPin, LOW);

  pinMode(pwmPin, OUTPUT);
  ledcSetup(motorIdx, pwmFreq, resolution);
  ledcAttachPin(pwmPin, motorIdx);
  ledcWrite(motorIdx, 0);

  motorCount++;
}

void setDirection(uint8_t motorIdx, uint8_t dir) {
  switch (dir) {
    case 1: // Clockwise
      digitalWrite(cwPins[motorIdx], HIGH);
      digitalWrite(ccwPins[motorIdx], LOW);
      break;
    case 2: // CounterClockwise
      digitalWrite(cwPins[motorIdx], LOW);
      digitalWrite(ccwPins[motorIdx], HIGH);
      break;
    default:
    // case 0: // Stop on any other value
      digitalWrite(cwPins[motorIdx], HIGH);
      digitalWrite(ccwPins[motorIdx], HIGH);
  }
}

void setMotorPWM(int pwm, uint8_t motorIdx) {
  int newPwm = boundValue(pwm, -1024, 1024);
  if(newPwm == 0) {
    setDirection(motorIdx, 0);
    ledcWrite(motorIdx, 0);
    return;
  }
  if(newPwm > 0) {
    setDirection(motorIdx, CW_DIR);
    ledcWrite(motorIdx, newPwm);
  } else {
    setDirection(motorIdx, CCW_DIR);
    ledcWrite(motorIdx, -newPwm);
  }
  // Serial.println("Motor PWM: " + String(pwm) + ", Encoder Counter: " + String(prevEncCount[motorIdx]));
}

/*
  PID Functions
*/
hw_timer_t * timer = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t pidFlag = 0;

const int MAX_RPM = 320;
const int ENC_COUNT_REV = 330; // (11 PPR) x (1:30 Gearbox)
// const uint32_t MAX_ENC_COUNT = MAX_RPM * ENC_COUNT_REV; // 100% speed
// ((MaxCount) / (60 secs)) * (20ms)
// const double MAX_TARGET_SPEED = (MAX_ENC_COUNT / 60.0f) * 0.02f;
// const uint8_t MAX_TARGET_SPEED = 35;
const uint8_t MAX_TARGET_SPEED = 100;
const float speedRatio = MAX_TARGET_SPEED / 100.0f;

uint8_t idx = 0;
volatile int encTarget[4] = {50, 0, 0, 0};
float kp = 1, ki = 0.001, kd = 5;
float controlPWM;
float currentError, prevError[4] = {0, 0, 0, 0};
float integral[4] = {0, 0, 0, 0};
int currentPWM[4], prevPWM[4] = {0, 0, 0, 0};

void IRAM_ATTR pidCycle() {
  portENTER_CRITICAL_ISR(&mux);
  pidFlag = 1;
  portEXIT_CRITICAL_ISR(&mux);
}

void pirControlInit() {
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &pidCycle, true);
  timerAlarmWrite(timer, 20000, true);
  timerAlarmEnable(timer);
}

void pidControl() {
  if(pidFlag == 0) {
    return;
  }
  for(idx = 0; idx < motorCount; idx++) {
    currentError = encTarget[idx] - encCount[idx];
    integral[idx] += currentError;
    controlPWM = currentError * kp +
      integral[idx] * ki +
      (currentError - prevError[idx]) * kd;

    currentPWM[idx] += 0.5*controlPWM;
    currentPWM[idx] = boundValue(currentPWM[idx], -512, 512);

    prevError[idx] = currentError;
    prevEncCount[idx] = encCount[idx];
    encCount[idx] = 0;
    prevPWM[idx] = currentPWM[idx];
    setMotorPWM(currentPWM[idx], idx);
  }
  portENTER_CRITICAL(&mux);
  pidFlag = 0;
  portEXIT_CRITICAL(&mux);
}

void setMotorSpeed(float speed, uint8_t motorIdx) {
  encTarget[motorIdx] = (int) (speed * speedRatio);
}
#endif // PIDS
