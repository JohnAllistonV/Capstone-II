/*
 * 4-Motor Differential Drive Robot
 * Open-Loop Motor Control + MPU-6050 IMU Heading Correction
 * + Serial Command Queue
 *
 * Serial commands (newline-terminated):
 *   forward <pwm> <cm>     — drive forward  at pwm for distance in cm
 *   backward <pwm> <cm>    — drive backward at pwm for distance in cm
 *   turn <degrees>         — turn to absolute heading (uses turnToHeading)
 *   stop                   — stop immediately and clear queue
 *
 * Examples:
 *   forward 150 50         — drive forward at PWM 150 for 50 cm
 *   backward 120 30        — drive backward at PWM 120 for 30 cm
 *   turn 90                — turn to 90° absolute heading
 *   stop                   — halt and clear queue
 */

#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ════════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ════════════════════════════════════════════════════════════════════════════

const uint8_t ENC_FL = 4;
const uint8_t ENC_FR = 10;
const uint8_t ENC_BL = 8;
const uint8_t ENC_BR = A0;

const uint8_t PWM_FL = 3;
const uint8_t PWM_FR = 5;
const uint8_t PWM_BL = 6;
const uint8_t PWM_BR = 9;

const uint8_t IN1_FL = A1;  const uint8_t IN2_FL = A2;
const uint8_t IN1_FR = A3;  const uint8_t IN2_FR = 2;
const uint8_t IN1_BL = 7;   const uint8_t IN2_BL = 11;
const uint8_t IN1_BR = 12;  const uint8_t IN2_BR = 13;


// ════════════════════════════════════════════════════════════════════════════
//  CONFIGURATION
// ════════════════════════════════════════════════════════════════════════════

const unsigned long DIRECTION_CHANGE_DELAY_MS = 80;
const unsigned long REPORT_INTERVAL_MS        = 200;

const int   PULSES_PER_REV      = 225;
const float WHEEL_DIAMETER_M    = 0.0762;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_M;

const float ENC_CAL_FL = 1.000;
const float ENC_CAL_FR = 1.000;
const float ENC_CAL_BL = 1.000;
const float ENC_CAL_BR = 1.000;

const int DRIVE_PWM = 150;

float CAL_FL = 0.80;
float CAL_FR = 1.00;
float CAL_BL = 1.20;
float CAL_BR = 0.85;

bool applyCalFactors = true;

const unsigned long IMU_INTERVAL_MS     = 10;
const unsigned long HEADING_INTERVAL_MS = 50;

float headingKp = 20.0;
float headingKi = 0.4;
float headingKd = 0.01;

// ── Turn parameters ───────────────────────────────────────────────────────
const int          TURN_PWM         = 80;
const float        TURN_TOLERANCE   = 1.0;
const unsigned long TURN_TIMEOUT_MS = 3000;


// ════════════════════════════════════════════════════════════════════════════
//  COMMAND QUEUE
// ════════════════════════════════════════════════════════════════════════════

// Command types
enum CmdType : uint8_t {
  CMD_NONE = 0,
  CMD_FORWARD,
  CMD_BACKWARD,
  CMD_TURN
};

struct Command {
  CmdType type;
  int     pwm;      // PWM for forward/backward; unused for turn
  float   param;    // cm for forward/backward; target degrees for turn
};

const uint8_t QUEUE_SIZE = 8;
Command  cmdQueue[QUEUE_SIZE];
uint8_t  qHead = 0;   // next command to execute
uint8_t  qTail = 0;   // next empty slot
uint8_t  qCount = 0;

bool queuePush(Command c) {
  if (qCount >= QUEUE_SIZE) return false;
  cmdQueue[qTail] = c;
  qTail = (qTail + 1) % QUEUE_SIZE;
  qCount++;
  return true;
}

Command queuePop() {
  Command c = cmdQueue[qHead];
  qHead = (qHead + 1) % QUEUE_SIZE;
  qCount--;
  return c;
}

void queueClear() {
  qHead = qTail = qCount = 0;
}


// ════════════════════════════════════════════════════════════════════════════
//  ACTIVE COMMAND STATE
// ════════════════════════════════════════════════════════════════════════════

bool     cmdActive       = false;
Command  activeCmd;

// Distance tracking for forward/backward
long     distStartFL = 0;
long     distStartFR = 0;
long     distStartBL = 0;
long     distTargetPulses = 0;   // pulses to travel


// ════════════════════════════════════════════════════════════════════════════
//  MOTOR STATE
// ════════════════════════════════════════════════════════════════════════════

int           motorDirection[4] = {1, 1, 1, 1};
bool          motorBraking[4]   = {false, false, false, false};
int           motorPending[4]   = {0, 0, 0, 0};
unsigned long brakeStart[4]     = {0, 0, 0, 0};


// ════════════════════════════════════════════════════════════════════════════
//  ENCODER STATE
// ════════════════════════════════════════════════════════════════════════════

volatile long encFL = 0, encFR = 0, encBL = 0, encBR = 0;
int           dirFL = 1,  dirFR = 1,  dirBL = 1,  dirBR = 1;
long          prevFL = 0, prevFR = 0, prevBL = 0, prevBR = 0;
float         rpmFL  = 0.0, rpmFR = 0.0, rpmBL = 0.0, rpmBR = 0.0;


// ════════════════════════════════════════════════════════════════════════════
//  IMU STATE
// ════════════════════════════════════════════════════════════════════════════

MPU6050       mpu(Wire);
float         totalAngleZ        = 0.0;
float         gyroZ              = 0.0;
unsigned long lastIMUTime        = 0;

float         targetHeading         = 0.0;
bool          drivingStraight       = false;
float         headingIntegral       = 0.0;
unsigned long lastHeadingUpdateTime = 0;


// ════════════════════════════════════════════════════════════════════════════
//  TIMING
// ════════════════════════════════════════════════════════════════════════════

unsigned long lastReportTime = 0;
int _lastLeftPWM  = 0;
int _lastRightPWM = 0;


// ════════════════════════════════════════════════════════════════════════════
//  ENCODER ISRs
// ════════════════════════════════════════════════════════════════════════════

void ISR_FL() { encFL += dirFL; }
void ISR_FR() { encFR += dirFR; }
void ISR_BL() { encBL += dirBL; }
void ISR_BR() { encBR += dirBR; }


// ════════════════════════════════════════════════════════════════════════════
//  MOTOR DRIVER
// ════════════════════════════════════════════════════════════════════════════

void applyMotor(uint8_t pwmPin, uint8_t in1, uint8_t in2, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);
  }
  analogWrite(pwmPin, abs(speed));
}

void setMotor(uint8_t idx, uint8_t pwmPin, uint8_t in1, uint8_t in2, int speed) {
  speed = constrain(speed, -255, 255);
  int newDirection = (speed > 0) ? 1 : (speed < 0) ? -1 : 0;

  if (newDirection != 0 && motorDirection[idx] != 0 && newDirection != motorDirection[idx]) {
    motorBraking[idx]   = true;
    motorPending[idx]   = speed;
    brakeStart[idx]     = millis();
    motorDirection[idx] = 0;
    applyMotor(pwmPin, in1, in2, 0);
    return;
  }

  if (!motorBraking[idx]) {
    motorDirection[idx] = newDirection;
    applyMotor(pwmPin, in1, in2, speed);
  }
}

void updateBrakes() {
  unsigned long now = millis();
  if (motorBraking[0] && now - brakeStart[0] >= DIRECTION_CHANGE_DELAY_MS) {
    motorBraking[0] = false; motorDirection[0] = (motorPending[0] > 0) ? 1 : -1;
    applyMotor(PWM_FL, IN1_FL, IN2_FL, motorPending[0]);
  }
  if (motorBraking[1] && now - brakeStart[1] >= DIRECTION_CHANGE_DELAY_MS) {
    motorBraking[1] = false; motorDirection[1] = (motorPending[1] > 0) ? 1 : -1;
    applyMotor(PWM_FR, IN1_FR, IN2_FR, motorPending[1]);
  }
  if (motorBraking[2] && now - brakeStart[2] >= DIRECTION_CHANGE_DELAY_MS) {
    motorBraking[2] = false; motorDirection[2] = (motorPending[2] > 0) ? 1 : -1;
    applyMotor(PWM_BL, IN1_BL, IN2_BL, motorPending[2]);
  }
  if (motorBraking[3] && now - brakeStart[3] >= DIRECTION_CHANGE_DELAY_MS) {
    motorBraking[3] = false; motorDirection[3] = (motorPending[3] > 0) ? 1 : -1;
    applyMotor(PWM_BR, IN1_BR, IN2_BR, motorPending[3]);
  }
}

void setFL(int s) {
  dirFL = (s > 0) ? 1 : (s < 0) ? -1 : 0;
  setMotor(0, PWM_FL, IN1_FL, IN2_FL, applyCalFactors ? (int)(s * CAL_FL) : s);
}
void setFR(int s) {
  dirFR = (s > 0) ? 1 : (s < 0) ? -1 : 0;
  setMotor(1, PWM_FR, IN1_FR, IN2_FR, applyCalFactors ? (int)(s * CAL_FR) : s);
}
void setBL(int s) {
  dirBL = (s > 0) ? 1 : (s < 0) ? -1 : 0;
  setMotor(2, PWM_BL, IN1_BL, IN2_BL, applyCalFactors ? (int)(s * CAL_BL) : s);
}
void setBR(int s) {
  dirBR = (s > 0) ? 1 : (s < 0) ? -1 : 0;
  setMotor(3, PWM_BR, IN1_BR, IN2_BR, applyCalFactors ? (int)(s * CAL_BR) : s);
}


// ════════════════════════════════════════════════════════════════════════════
//  MOVEMENT API
// ════════════════════════════════════════════════════════════════════════════

void setSides(int leftPWM, int rightPWM) {
  _lastLeftPWM  = leftPWM;
  _lastRightPWM = rightPWM;
  setFR(rightPWM);
  setFL(leftPWM);
  setBL(leftPWM);
  setBR(rightPWM);
}

void driveAll(int pwm) {
  if (!drivingStraight) {
    applyCalFactors       = true;
    targetHeading         = totalAngleZ;
    drivingStraight       = true;
    headingIntegral       = 0.0;
    lastHeadingUpdateTime = millis();
    setSides(pwm, pwm);
  }
}

void stopAll() {
  drivingStraight       = false;
  headingIntegral       = 0.0;
  lastHeadingUpdateTime = 0;
  applyCalFactors       = true;
  setSides(0, 0);
}

const float        TURN_SLOWDOWN_DEG = 25.0;
const int          TURN_MIN_PWM      = 60;
const unsigned long TURN_SETTLE_MS   = 200;

void turnToHeading(float targetDeg, int pwm, float toleranceDeg, unsigned long timeoutMs) {
  drivingStraight = false;
  headingIntegral = 0.0;
  applyCalFactors = false;

  setSides(0, 0);

  unsigned long settleStart = millis();
  while (millis() - settleStart < TURN_SETTLE_MS) {
    unsigned long now = millis();
    if (now - lastIMUTime >= IMU_INTERVAL_MS) {
      lastIMUTime = now;
      mpu.update();
      gyroZ       = mpu.getGyroZ();
      totalAngleZ = mpu.getAngleZ();
    }
    updateBrakes();
  }

  unsigned long startMs = millis();

  while (true) {
    unsigned long now = millis();

    if (now - lastIMUTime >= IMU_INTERVAL_MS) {
      lastIMUTime = now;
      mpu.update();
      gyroZ       = mpu.getGyroZ();
      totalAngleZ = mpu.getAngleZ();
    }

    if (timeoutMs > 0 && (now - startMs) >= timeoutMs) {
      Serial.println(F("turnToHeading: TIMEOUT"));
      break;
    }

    float remaining = targetDeg - totalAngleZ;
    while (remaining >  180.0) remaining -= 360.0;
    while (remaining < -180.0) remaining += 360.0;

    if (abs(remaining) <= toleranceDeg) break;

    int activePWM = pwm;
    if (abs(remaining) < TURN_SLOWDOWN_DEG) {
      activePWM = (int)(TURN_MIN_PWM + (pwm - TURN_MIN_PWM) * (abs(remaining) / TURN_SLOWDOWN_DEG));
      activePWM = max(activePWM, TURN_MIN_PWM);
    }

    if (remaining > 0) {
      setSides(-activePWM,  activePWM);
    } else {
      setSides( activePWM, -activePWM);
    }

    updateBrakes();
  }

  stopAll();
}


// ════════════════════════════════════════════════════════════════════════════
//  SERIAL COMMAND PARSER
// ════════════════════════════════════════════════════════════════════════════

// Reads a newline-terminated string from Serial into buf. Returns true when
// a complete line is ready. Safe to call every loop — non-blocking.
bool readSerialLine(char* buf, uint8_t maxLen) {
  static char   lineBuf[64];
  static uint8_t linePos = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (linePos > 0) {
        lineBuf[linePos] = '\0';
        strncpy(buf, lineBuf, maxLen - 1);
        buf[maxLen - 1] = '\0';
        linePos = 0;
        return true;
      }
    } else if (linePos < (uint8_t)(sizeof(lineBuf) - 1)) {
      lineBuf[linePos++] = c;
    }
  }
  return false;
}

void parseAndEnqueue(const char* line) {
  char   verb[16];
  int    pwm   = 0;
  float  param = 0.0;

  // "stop" — immediate, clears queue
  if (strncmp(line, "stop", 4) == 0) {
    queueClear();
    cmdActive = false;
    stopAll();
    Serial.println(F("CMD: stop — queue cleared"));
    return;
  }

  // "turn <degrees>"
  if (sscanf(line, "turn %f", &param) == 1) {
    Command c = { CMD_TURN, 0, param };
    if (queuePush(c)) {
      Serial.print(F("CMD queued: turn ")); Serial.println(param);
    } else {
      Serial.println(F("ERR: queue full"));
    }
    return;
  }

  // "forward <pwm> <cm>"  or  "backward <pwm> <cm>"
  if (sscanf(line, "%15s %d %f", verb, &pwm, &param) == 3) {
    CmdType t = CMD_NONE;
    if      (strcmp(verb, "forward")  == 0) t = CMD_FORWARD;
    else if (strcmp(verb, "backward") == 0) t = CMD_BACKWARD;

    if (t != CMD_NONE) {
      pwm   = constrain(pwm,   0, 255);
      param = constrain(param, 0, 5000);  // max 50 m — sanity check
      Command c = { t, pwm, param };
      if (queuePush(c)) {
        Serial.print(F("CMD queued: ")); Serial.print(verb);
        Serial.print(' '); Serial.print(pwm);
        Serial.print(' '); Serial.println(param);
      } else {
        Serial.println(F("ERR: queue full"));
      }
      return;
    }
  }

  Serial.print(F("ERR: unknown command: ")); Serial.println(line);
}


// ════════════════════════════════════════════════════════════════════════════
//  COMMAND EXECUTOR  (non-blocking, called every loop)
// ════════════════════════════════════════════════════════════════════════════

// Returns the average absolute encoder travel across the three good encoders
// since the snapshot taken at command start.
long distancePulsesNow() {
  noInterrupts();
  long fl = encFL, fr = encFR, bl = encBL;
  interrupts();

  long dFL = abs(fl - distStartFL);
  long dFR = abs(fr - distStartFR);
  long dBL = abs(bl - distStartBL);
  return (dFL + dFR + dBL) / 3;
}

void startNextCommand() {
  if (qCount == 0) return;
  activeCmd = queuePop();
  cmdActive = true;

  if (activeCmd.type == CMD_FORWARD || activeCmd.type == CMD_BACKWARD) {
    // Snapshot encoder positions
    noInterrupts();
    distStartFL = encFL;
    distStartFR = encFR;
    distStartBL = encBL;
    interrupts();

    // Convert cm → pulses
    float targetCm     = activeCmd.param;
    float targetMetres = targetCm / 100.0;
    distTargetPulses   = (long)((targetMetres / WHEEL_CIRCUMFERENCE) * PULSES_PER_REV);

    int pwm = (activeCmd.type == CMD_BACKWARD) ? -activeCmd.pwm : activeCmd.pwm;
    driveAll(pwm);   // enables heading PID

    Serial.print(F("EXEC: "));
    Serial.print(activeCmd.type == CMD_FORWARD ? F("forward") : F("backward"));
    Serial.print(F(" pwm=")); Serial.print(activeCmd.pwm);
    Serial.print(F(" target_pulses=")); Serial.println(distTargetPulses);

  } else if (activeCmd.type == CMD_TURN) {
    Serial.print(F("EXEC: turn to ")); Serial.println(activeCmd.param);
    // turnToHeading is blocking — runs synchronously here.
    // Serial parsing resumes on next loop() after it returns.
    turnToHeading(activeCmd.param, TURN_PWM, TURN_TOLERANCE, TURN_TIMEOUT_MS);
    cmdActive = false;   // command finished synchronously
    Serial.println(F("DONE: turn"));
  }
}

void updateCommandExecutor() {
  if (!cmdActive) {
    if (qCount > 0) startNextCommand();
    return;
  }

  // ── Forward / backward — check distance ──────────────────────────────────
  if (activeCmd.type == CMD_FORWARD || activeCmd.type == CMD_BACKWARD) {
    if (distancePulsesNow() >= distTargetPulses) {
      stopAll();
      cmdActive = false;
      Serial.println(F("DONE: distance reached"));
    }
    // Heading PID correction runs in the main loop — nothing extra needed here
  }
}


// ════════════════════════════════════════════════════════════════════════════
//  SERIAL OUTPUT
// ════════════════════════════════════════════════════════════════════════════

void printData(long snapFL, long snapFR, long snapBL, long snapBR) {
  float headingError = drivingStraight ? (targetHeading - totalAngleZ) : 0.0;
  float distFL = ((float)snapFL / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE;
  float distFR = ((float)snapFR / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE;
  float distBL = ((float)snapBL / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE;
  float distBR = ((float)snapBR / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE;
  Serial.println(F("----------------------------------------"));
  Serial.print(F("Queue  | Count: ")); Serial.print(qCount);
  Serial.print(F("  | Active: ")); Serial.println(cmdActive ? F("YES") : F("NO"));
  Serial.print(F("Motors | Left PWM: "));    Serial.print(_lastLeftPWM);
  Serial.print(F("  | Right PWM: "));        Serial.println(_lastRightPWM);
  Serial.print(F("FL     | RPM: "));         Serial.print(rpmFL, 1);
  Serial.print(F("  | Dist: "));             Serial.print(distFL, 3); Serial.println(F(" m"));
  Serial.print(F("FR     | RPM: "));         Serial.print(rpmFR, 1);
  Serial.print(F("  | Dist: "));             Serial.print(distFR, 3); Serial.println(F(" m"));
  Serial.print(F("BL     | RPM: "));         Serial.print(rpmBL, 1);
  Serial.print(F("  | Dist: "));             Serial.print(distBL, 3); Serial.println(F(" m"));
  Serial.print(F("BR     | RPM: "));         Serial.print(rpmBR, 1);
  Serial.print(F("  | Dist: "));             Serial.print(distBR, 3); Serial.println(F(" m"));
  Serial.print(F("IMU   | GyroZ: "));        Serial.print(gyroZ, 2);
  Serial.print(F(" deg/s  | Heading: "));    Serial.print(totalAngleZ, 2);
  Serial.print(F(" deg  | Error: "));        Serial.print(headingError, 2);
  Serial.print(F(" deg  | Integral: "));     Serial.println(headingIntegral, 2);
  Serial.print(F("Cal   | Active: "));       Serial.println(applyCalFactors ? F("YES") : F("NO (turn mode)"));
}


// ════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Differential Drive Robot + IMU Heading Correction ==="));
  Serial.println(F("Commands: forward <pwm> <cm> | backward <pwm> <cm> | turn <deg> | stop"));

  Wire.begin();
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print(F("MPU6050 init failed: "));
    Serial.println(status);
    while (1);
  }
  Serial.println(F("Calibrating gyro... keep robot still!"));
  delay(2000);
  mpu.calcGyroOffsets();
  Serial.println(F("Gyro calibrated."));
  lastIMUTime = millis();

  pinMode(ENC_FL, INPUT_PULLUP); pinMode(ENC_FR, INPUT_PULLUP);
  pinMode(ENC_BL, INPUT_PULLUP); pinMode(ENC_BR, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(ENC_FL), ISR_FL, RISING);
  attachPCINT(digitalPinToPCINT(ENC_FR), ISR_FR, RISING);
  attachPCINT(digitalPinToPCINT(ENC_BL), ISR_BL, RISING);

  pinMode(PWM_FL, OUTPUT); pinMode(PWM_FR, OUTPUT);
  pinMode(PWM_BL, OUTPUT); pinMode(PWM_BR, OUTPUT);
  pinMode(IN1_FL, OUTPUT); pinMode(IN2_FL, OUTPUT);
  pinMode(IN1_FR, OUTPUT); pinMode(IN2_FR, OUTPUT);
  pinMode(IN1_BL, OUTPUT); pinMode(IN2_BL, OUTPUT);
  pinMode(IN1_BR, OUTPUT); pinMode(IN2_BR, OUTPUT);

  setFL(0); setFR(0); setBL(0); setBR(0);

  lastReportTime        = millis();
  lastHeadingUpdateTime = millis();
  drivingStraight       = false;
  applyCalFactors       = true;

  queueClear();
  cmdActive = false;

  Serial.println(F("Ready."));
}


// ════════════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // ── Brakes ───────────────────────────────────────────────────────────────
  updateBrakes();

  // ── IMU update ───────────────────────────────────────────────────────────
  if (now - lastIMUTime >= IMU_INTERVAL_MS) {
    lastIMUTime  = now;
    mpu.update();
    gyroZ        = mpu.getGyroZ();
    totalAngleZ  = mpu.getAngleZ();
  }

  // ── Serial input ─────────────────────────────────────────────────────────
  char lineBuf[64];
  if (readSerialLine(lineBuf, sizeof(lineBuf))) {
    parseAndEnqueue(lineBuf);
  }

  // ── Command executor ─────────────────────────────────────────────────────
  updateCommandExecutor();

  // ── IMU heading PID correction (straight driving only, 20 Hz) ───────────
  if (drivingStraight && now - lastHeadingUpdateTime >= HEADING_INTERVAL_MS) {
    lastHeadingUpdateTime = now;

    float dt_h         = HEADING_INTERVAL_MS / 1000.0;
    float headingError = targetHeading - totalAngleZ;

    if (abs(headingError) > 0.2) {
      headingIntegral += headingError * dt_h;
      headingIntegral  = constrain(headingIntegral, -30.0 / (headingKi + 0.001), 30.0 / (headingKi + 0.001));

      float headingDerivative = -gyroZ;
      float headingCorrection = (headingKp * headingError)
                              + (headingKi * headingIntegral)
                              + (headingKd * headingDerivative);
      headingCorrection = constrain(headingCorrection, -50.0, 50.0);

      int leftPWM  = constrain(_lastLeftPWM  - (int)headingCorrection, 0, 255);
      int rightPWM = constrain(_lastRightPWM + (int)headingCorrection, 0, 255);
      setSides(leftPWM, rightPWM);
    } else {
      headingIntegral *= 0.95;
    }
  }

  // ── Serial report ────────────────────────────────────────────────────────
  if (now - lastReportTime >= REPORT_INTERVAL_MS) {
    lastReportTime = now;
    float dt = REPORT_INTERVAL_MS / 1000.0;

    noInterrupts();
    long snapFL = encFL, snapFR = encFR, snapBL = encBL, snapBR = encBR;
    interrupts();

    rpmFL  = ((float)(snapFL - prevFL) / PULSES_PER_REV / dt) * 60.0 * ENC_CAL_FL;
    rpmFR  = ((float)(snapFR - prevFR) / PULSES_PER_REV / dt) * 60.0 * ENC_CAL_FR;
    rpmBL  = ((float)(snapBL - prevBL) / PULSES_PER_REV / dt) * 60.0 * ENC_CAL_BL;
    rpmBR  = ((float)(snapBR - prevBR) / PULSES_PER_REV / dt) * 60.0 * ENC_CAL_BR;
    prevFL = snapFL; prevFR = snapFR; prevBL = snapBL; prevBR = snapBR;

    printData(snapFL, snapFR, snapBL, snapBR);
  }
}