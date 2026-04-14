/*
 * 4-Motor Differential Drive Robot — Serial Command Interface
 *
 * Single command:
 *   STRAIGHT<ms>   e.g. STRAIGHT2000   drive forward N milliseconds
 *   LEFT<deg>      e.g. LEFT45         turn left N degrees
 *   RIGHT<deg>     e.g. RIGHT90        turn right N degrees
 *   TURN<deg>      e.g. TURN180        turn N degrees (+left / -right)
 *   STOP                               stop immediately
 *
 * Sequence (comma-separated, no spaces):
 *   STRAIGHT1500,LEFT90,STRAIGHT1500,RIGHT90,STRAIGHT2000,TURN180
 *
 * Responses:
 *   CMD x/n:<token>      about to execute step x of n
 *   DONE x/n             step completed
 *   SEQUENCE_COMPLETE    all steps finished
 *   WARN:TURN_TIMEOUT    turn gave up after 5 s
 *   ERR:<reason>         bad command / param — sequence aborts
 *
 * Board:     Arduino Nano
 * Libraries: Wire (built-in), MPU6050_light by rfetick, PinChangeInterrupt
 *
 * Pin Map:
 *   D2  IN2 FR    D3  PWM FL   D4  Enc FL   D5  PWM FR
 *   D6  PWM BL    D7  IN1 BL   D8  Enc BL   D9  PWM BR
 *   D10 Enc FR    D11 IN2 BL   D12 IN1 BR   D13 IN2 BR
 *   A0  Enc BR    A1  IN1 FL   A2  IN2 FL   A3  IN1 FR
 *   A4  IMU SDA   A5  IMU SCL
 */

#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ════════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ════════════════════════════════════════════════════════════════════════════

const uint8_t ENC_FL = 4,  ENC_FR = 10, ENC_BL = 8,  ENC_BR = A0;
const uint8_t PWM_FL = 3,  PWM_FR = 5,  PWM_BL = 6,  PWM_BR = 9;
const uint8_t IN1_FL = A1, IN2_FL = A2;
const uint8_t IN1_FR = A3, IN2_FR = 2;
const uint8_t IN1_BL = 7,  IN2_BL = 11;
const uint8_t IN1_BR = 12, IN2_BR = 13;

// ════════════════════════════════════════════════════════════════════════════
//  CONFIGURATION — tune these for your robot
// ════════════════════════════════════════════════════════════════════════════

const int DRIVE_PWM = 150;
const int TURN_PWM  = 150;

float CAL_FL = 0.80;
float CAL_FR = 1.00;
float CAL_BL = 1.20;
float CAL_BR = 0.85;

float headingKp = 20.0;
float headingKi = 0.4;
float headingKd = 0.01;

const float         TURN_TOLERANCE_DEG = 1.5;
const float         TURN_SLOWDOWN_DEG  = 25.0;
const int           TURN_MIN_PWM       = 60;
const unsigned long TURN_TIMEOUT_MS    = 5000;
const unsigned long TURN_SETTLE_MS     = 200;

const unsigned long IMU_INTERVAL_MS     = 10;
const unsigned long HEADING_INTERVAL_MS = 50;
const unsigned long DIR_CHANGE_DELAY_MS = 80;

// ════════════════════════════════════════════════════════════════════════════
//  STATE
// ════════════════════════════════════════════════════════════════════════════

MPU6050 mpu(Wire);

float         totalAngleZ           = 0.0;
float         gyroZ                 = 0.0;
unsigned long lastIMUTime           = 0;

float         targetHeading         = 0.0;
bool          drivingStraight       = false;
float         headingIntegral       = 0.0;
unsigned long lastHeadingUpdateTime = 0;

bool applyCalFactors = true;
int  _lastLeftPWM    = 0;
int  _lastRightPWM   = 0;

int           motorDirection[4] = {1, 1, 1, 1};
bool          motorBraking[4]   = {false, false, false, false};
int           motorPending[4]   = {0, 0, 0, 0};
unsigned long brakeStart[4]     = {0, 0, 0, 0};

volatile long encFL = 0, encFR = 0, encBL = 0, encBR = 0;
int           dirFL = 1,  dirFR = 1,  dirBL = 1,  dirBR = 1;

String cmdBuffer = "";

// ════════════════════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ════════════════════════════════════════════════════════════════════════════

void stopRobot();

// ════════════════════════════════════════════════════════════════════════════
//  ENCODER ISRs
// ════════════════════════════════════════════════════════════════════════════

void ISR_FL() { encFL += dirFL; }
void ISR_FR() { encFR += dirFR; }
void ISR_BL() { encBL += dirBL; }

// ════════════════════════════════════════════════════════════════════════════
//  LOW-LEVEL MOTOR DRIVER
// ════════════════════════════════════════════════════════════════════════════

void applyMotor(uint8_t pwmPin, uint8_t in1, uint8_t in2, int speed) {
  if      (speed > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                { digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); }
  analogWrite(pwmPin, abs(speed));
}

void setMotorRaw(uint8_t idx, uint8_t pwmPin, uint8_t in1, uint8_t in2, int speed) {
  speed = constrain(speed, -255, 255);
  int newDir = (speed > 0) ? 1 : (speed < 0) ? -1 : 0;

  if (newDir != 0 && motorDirection[idx] != 0 && newDir != motorDirection[idx]) {
    motorBraking[idx]   = true;
    motorPending[idx]   = speed;
    brakeStart[idx]     = millis();
    motorDirection[idx] = 0;
    applyMotor(pwmPin, in1, in2, 0);
    return;
  }
  if (!motorBraking[idx]) {
    motorDirection[idx] = newDir;
    applyMotor(pwmPin, in1, in2, speed);
  }
}

void updateBrakes() {
  unsigned long now = millis();
  struct { uint8_t pwm, in1, in2; } motors[4] = {
    {PWM_FL, IN1_FL, IN2_FL},
    {PWM_FR, IN1_FR, IN2_FR},
    {PWM_BL, IN1_BL, IN2_BL},
    {PWM_BR, IN1_BR, IN2_BR}
  };
  for (int i = 0; i < 4; i++) {
    if (motorBraking[i] && now - brakeStart[i] >= DIR_CHANGE_DELAY_MS) {
      motorBraking[i]   = false;
      motorDirection[i] = (motorPending[i] > 0) ? 1 : -1;
      applyMotor(motors[i].pwm, motors[i].in1, motors[i].in2, motorPending[i]);
    }
  }
}

void setFL(int s) { dirFL=(s>0)?1:(s<0)?-1:0; setMotorRaw(0,PWM_FL,IN1_FL,IN2_FL, applyCalFactors?(int)(s*CAL_FL):s); }
void setFR(int s) { dirFR=(s>0)?1:(s<0)?-1:0; setMotorRaw(1,PWM_FR,IN1_FR,IN2_FR, applyCalFactors?(int)(s*CAL_FR):s); }
void setBL(int s) { dirBL=(s>0)?1:(s<0)?-1:0; setMotorRaw(2,PWM_BL,IN1_BL,IN2_BL, applyCalFactors?(int)(s*CAL_BL):s); }
void setBR(int s) { dirBR=(s>0)?1:(s<0)?-1:0; setMotorRaw(3,PWM_BR,IN1_BR,IN2_BR, applyCalFactors?(int)(s*CAL_BR):s); }

void setSides(int leftPWM, int rightPWM) {
  _lastLeftPWM  = leftPWM;
  _lastRightPWM = rightPWM;
  setFL(leftPWM);  setBL(leftPWM);
  setFR(rightPWM); setBR(rightPWM);
}

// ════════════════════════════════════════════════════════════════════════════
//  IMU POLL
// ════════════════════════════════════════════════════════════════════════════

void pollIMU() {
  unsigned long now = millis();
  if (now - lastIMUTime >= IMU_INTERVAL_MS) {
    lastIMUTime = now;
    mpu.update();
    gyroZ       = mpu.getGyroZ();
    totalAngleZ = mpu.getAngleZ();
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  MOVEMENT API
// ════════════════════════════════════════════════════════════════════════════



void goStraight(unsigned long durationMs) {
  applyCalFactors       = true;
  targetHeading         = totalAngleZ;
  drivingStraight       = true;
  headingIntegral       = 0.0;
  lastHeadingUpdateTime = millis();
  setSides(DRIVE_PWM, DRIVE_PWM);

  unsigned long startMs = millis();

  while (millis() - startMs < durationMs) {
    if(readSerial()){
      if(cmdBuffer == "STOP"){
        cmdBuffer = "";
        stopRobot();
        return;
      }
      cmdBuffer = "";
    }
    unsigned long now = millis();
    pollIMU();
    updateBrakes();

    if (now - lastHeadingUpdateTime >= HEADING_INTERVAL_MS) {
      lastHeadingUpdateTime = now;

      float dt_h         = HEADING_INTERVAL_MS / 1000.0;
      float headingError = targetHeading - totalAngleZ;

      if (abs(headingError) > 0.2) {
        headingIntegral += headingError * dt_h;
        headingIntegral  = constrain(headingIntegral,
                                     -30.0 / (headingKi + 0.001),
                                      30.0 / (headingKi + 0.001));

        float correction = (headingKp * headingError)
                         + (headingKi * headingIntegral)
                         + (headingKd * (-gyroZ));
        correction = constrain(correction, -50.0, 50.0);

        int leftPWM  = constrain(DRIVE_PWM - (int)correction, 0, 255);
        int rightPWM = constrain(DRIVE_PWM + (int)correction, 0, 255);
        setSides(leftPWM, rightPWM);
      } else {
        headingIntegral *= 0.95;
        setSides(DRIVE_PWM, DRIVE_PWM);
      }
    }
  }

  stopRobot();
}

void stopRobot() {
  drivingStraight = false;
  headingIntegral = 0.0;
  applyCalFactors = true;
  setSides(0, 0);
}

void _turnToHeading(float targetDeg, int pwm) {
  drivingStraight = false;
  headingIntegral = 0.0;
  applyCalFactors = false;

  setSides(0, 0);

  unsigned long settleStart = millis();
  while (millis() - settleStart < TURN_SETTLE_MS) {
    pollIMU();
    updateBrakes();
  }

  unsigned long startMs = millis();

  while (true) {
    pollIMU();
    updateBrakes();

    if (millis() - startMs >= TURN_TIMEOUT_MS) {
      Serial.println(F("WARN:TURN_TIMEOUT"));
      break;
    }

    float remaining = targetDeg - totalAngleZ;
    while (remaining >  180.0) remaining -= 360.0;
    while (remaining < -180.0) remaining += 360.0;

    if (abs(remaining) <= TURN_TOLERANCE_DEG) break;

    int activePWM = pwm;
    if (abs(remaining) < TURN_SLOWDOWN_DEG) {
      activePWM = (int)(TURN_MIN_PWM + (pwm - TURN_MIN_PWM) * (abs(remaining) / TURN_SLOWDOWN_DEG));
      activePWM = max(activePWM, TURN_MIN_PWM);
    }

    if (remaining > 0) setSides(-activePWM,  activePWM);
    else               setSides( activePWM, -activePWM);
  }

  stopRobot();
}

void turnLeft(float deg)  { _turnToHeading(totalAngleZ + deg, TURN_PWM); }
void turnRight(float deg) { _turnToHeading(totalAngleZ - deg, TURN_PWM); }
void turnAbs(float deg)   { _turnToHeading(totalAngleZ + deg, TURN_PWM); }

// ════════════════════════════════════════════════════════════════════════════
//  COMMAND PARSER & QUEUE
// ════════════════════════════════════════════════════════════════════════════

bool parseParam(const String& cmd, const String& prefix, float& value) {
  if (!cmd.startsWith(prefix)) return false;
  String numStr = cmd.substring(prefix.length());
  numStr.trim();
  if (numStr.length() == 0) return false;
  value = numStr.toFloat();
  return true;
}

bool readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuffer.trim();
      if (cmdBuffer.length() > 0) return true;  // line ready
    } else {
      cmdBuffer += c;
    }
  }
  return false;
}


 

// Executes one token. Returns false on error (sequence should abort).
bool executeSingle(const String& cmd) {
  float param = 0.0;

  if (parseParam(cmd, "STRAIGHT", param)) {
    if (param <= 0) { Serial.println(F("ERR:STRAIGHT_PARAM")); return false; }
    goStraight((unsigned long)param);

  } else if (cmd == "STOP") {
    stopRobot();

  } else if (parseParam(cmd, "LEFT", param)) {
    if (param <= 0) { Serial.println(F("ERR:LEFT_PARAM")); return false; }
    turnLeft(param);

  } else if (parseParam(cmd, "RIGHT", param)) {
    if (param <= 0) { Serial.println(F("ERR:RIGHT_PARAM")); return false; }
    turnRight(param);

  } else if (parseParam(cmd, "TURN", param)) {
    if (param == 0) { Serial.println(F("ERR:TURN_PARAM")); return false; }
    turnAbs(param);

  } else {
    Serial.print(F("ERR:"));
    Serial.println(cmd);
    return false;
  }
  return true;
}

// Splits input on commas and executes each token in order.
void handleCommand(const String& input) {
  String remaining = input;

  // Count tokens
  int total = 1;
  for (int i = 0; i < (int)remaining.length(); i++)
    if (remaining[i] == ',') total++;

  int done = 0;

  while (remaining.length() > 0) {
    String token;
    int commaIdx = remaining.indexOf(',');

    if (commaIdx == -1) {
      token     = remaining;
      remaining = "";
    } else {
      token     = remaining.substring(0, commaIdx);
      remaining = remaining.substring(commaIdx + 1);
    }

    token.trim();
    if (token.length() == 0) continue;

    done++;
    Serial.print(F("CMD "));
    Serial.print(done);
    Serial.print('/');
    Serial.print(total);
    Serial.print(':');
    Serial.println(token);

    if (!executeSingle(token)) {
      Serial.println(F("ERR:SEQUENCE_ABORTED"));
      stopRobot();
      return;
    }

    Serial.print(F("DONE "));
    Serial.print(done);
    Serial.print('/');
    Serial.println(total);
  }

  Serial.println(F("SEQUENCE_COMPLETE"));
}

// ════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Robot Motion Controller ==="));

  Wire.begin();
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print(F("ERR:IMU_INIT:"));
    Serial.println(status);
    while (1);
  }
  Serial.println(F("Calibrating gyro — keep robot still..."));
  delay(2000);
  mpu.calcGyroOffsets();
  lastIMUTime = millis();
  Serial.println(F("READY"));

  pinMode(ENC_FL, INPUT_PULLUP); pinMode(ENC_FR, INPUT_PULLUP);
  pinMode(ENC_BL, INPUT_PULLUP); pinMode(ENC_BR, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(ENC_FL), ISR_FL, RISING);
  attachPCINT(digitalPinToPCINT(ENC_FR), ISR_FR, RISING);
  attachPCINT(digitalPinToPCINT(ENC_BL), ISR_BL, RISING);

  uint8_t outPins[] = {PWM_FL, PWM_FR, PWM_BL, PWM_BR,
                       IN1_FL, IN2_FL, IN1_FR, IN2_FR,
                       IN1_BL, IN2_BL, IN1_BR, IN2_BR};
  for (uint8_t p : outPins) pinMode(p, OUTPUT);

  setFL(0); setFR(0); setBL(0); setBR(0);
  cmdBuffer.reserve(64);
}

// ════════════════════════════════════════════════════════════════════════════
//  LOOP — serial reader + idle IMU/brake polling
// ════════════════════════════════════════════════════════════════════════════

void loop() {
  pollIMU();
  updateBrakes();

  if (readSerial()){
    handleCommand(cmdBuffer);
    cmdBuffer = "";
  }
}