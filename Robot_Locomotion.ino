/*
 * 4-Motor Differential Drive Robot
 * Open-Loop Motor Control + MPU-6050 IMU Heading Correction
 *
 * Board:   Arduino Uno / Nano
 * Libraries: Wire (built-in)
 *            MPU6050_light — Sketch -> Include Library -> Manage Libraries
 *                            Search: "MPU6050_light" by rfetick
 *
 * ── Drive Architecture ───────────────────────────────────────────────────────
 *
 *   L298N #1 handles FRONT motors (FL via CH-A, FR via CH-B)
 *   L298N #2 handles BACK  motors (BL via CH-A, BR via CH-B)
 *
 *   Motors run open-loop at a fixed PWM. The MPU-6050 gyro measures
 *   heading drift and trims left/right PWM to keep the robot driving straight.
 *
 * ── Pin Map ───────────────────────────────────────────────────────────────────
 *
 *   D0, D1  — reserved for Serial (do not use)
 *   D2      — IN4 FR    (FR reverse)
 *   D3      — PWM FL    (hardware PWM)
 *   D4      — Encoder FL – A (monitoring only)
 *   D5      — PWM FR    (hardware PWM)
 *   D6      — PWM BL    (hardware PWM)
 *   D7      — IN4 BL    (BL forward)
 *   D8      — Encoder BL – A (monitoring only)
 *   D9      — PWM BR    (hardware PWM)
 *   D10     — Encoder FR – A (monitoring only)
 *   D11     — IN3 BL    (BL reverse)
 *   D12     — IN2 BR    (BR forward)
 *   D13     — IN1 BR    (BR reverse)
 *   A0      — Encoder BR – A (faulty, monitoring only)
 *   A1      — IN1 FL    (FL forward)
 *   A2      — IN2 FL    (FL reverse)
 *   A3      — IN3 FR    (FR forward)
 *   A4      — IMU SDA   (MPU-6050)
 *   A5      — IMU SCL   (MPU-6050)
 *
 * ── Wiring Summary ───────────────────────────────────────────────────────────
 *
 *   L298N #1 — FRONT (FL=CH-A, FR=CH-B)   L298N #2 — BACK (BL=CH-A, BR=CH-B)
 *   ──────────────────────────────          ──────────────────────────────────
 *   ENA  → D3  (PWM, FL)                   ENA  → D6  (PWM, BL)
 *   IN1  → A1  (FL forward)                IN1  → D7  (BL forward)
 *   IN2  → A2  (FL reverse)                IN2  → D11 (BL reverse)
 *   ENB  → D5  (PWM, FR)                   ENB  → D9  (PWM, BR)
 *   IN3  → A3  (FR forward)                IN3  → D12 (BR forward)
 *   IN4  → D2  (FR reverse)                IN4  → D13 (BR reverse)
 *
 *   MPU-6050
 *   VCC → 3.3V or 5V
 *   GND → GND
 *   SDA → A4
 *   SCL → A5
 *   AD0 → GND  (sets I2C address to 0x68)
 *
 * ── Power ────────────────────────────────────────────────────────────────────
 *   - Motor battery (6-12V) → L298N Vs on both boards
 *   - L298N 5V jumper ON if battery <= 12V
 *   - Connect ALL GNDs: Arduino, both L298Ns, battery negative
 *   - Do NOT draw motor power from Arduino 5V pin
 *
 * ── Cal Factor / Turn Fix ────────────────────────────────────────────────────
 *   CAL factors (CAL_FL, CAL_FR, CAL_BL, CAL_BR) are applied during straight
 *   driving only. During turns, applyCalFactors is set false so all four wheels
 *   receive symmetric raw PWM — preventing per-motor scaling from distorting
 *   turn geometry. A forced symmetric stop is also issued at the start of
 *   turnToHeading() to clear any asymmetric PWM state left by the heading PID.
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

// ── Per-motor PWM calibration factors ────────────────────────────────────────
// Applied during straight driving only — NOT during turns.
// To calibrate: set headingKp/Ki/Kd to 0, run straight, adjust until straight.
float CAL_FL = 0.80;
float CAL_FR = 1.00;
float CAL_BL = 1.20;
float CAL_BR = 0.85;

// ── Cal factor gate ───────────────────────────────────────────────────────────
// true  = CAL factors active (straight driving)
// false = raw symmetric PWM (turns)
// Managed automatically — do not set manually.
bool applyCalFactors = true;

const unsigned long IMU_INTERVAL_MS     = 10;
const unsigned long HEADING_INTERVAL_MS = 50;

float headingKp = 20.0;
float headingKi = 0.4;
float headingKd = 0.01;


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
unsigned long sequenceStart  = 0;
int _lastLeftPWM  = 0;
int _lastRightPWM = 0;


// ════════════════════════════════════════════════════════════════════════════
//  ENCODER ISRs
// ════════════════════════════════════════════════════════════════════════════

void ISR_FL() { encFL += dirFL; }
void ISR_FR() { encFR += dirFR; }
void ISR_BL() { encBL += dirBL; }
void ISR_BR() { encBR += dirBR; }  // Not attached — BR encoder faulty


// ════════════════════════════════════════════════════════════════════════════
//  MOTOR DRIVER
// ════════════════════════════════════════════════════════════════════════════

void applyMotor(uint8_t pwmPin, uint8_t in1, uint8_t in2, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);  // Active brake
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

// ── Per-motor setters — CAL factors applied only when applyCalFactors is true ──
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
    applyCalFactors       = true;   // Enable CAL factors for straight driving
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
  applyCalFactors       = true;   // Restore CAL factors for next driveAll()
  setSides(0, 0);
}

void turnLeft(int pwm)               { drivingStraight = false; setSides(-pwm,  pwm);   }
void turnRight(int pwm)              { drivingStraight = false; setSides( pwm, -pwm);   }
void curveLeft(int outer, int inner) { drivingStraight = false; setSides(inner, outer); }
void curveRight(int outer, int inner){ drivingStraight = false; setSides(outer, inner); }

// ── turnToHeading ─────────────────────────────────────────────────────────────
// Turns to an absolute heading (degrees). Blocking.
//
// CAL factors are disabled for the entire turn so all four wheels receive
// symmetric raw PWM. A forced symmetric stop clears any asymmetric PID state
// before the settle pause begins.

const float        TURN_SLOWDOWN_DEG = 25.0;
const int          TURN_MIN_PWM      = 60;
const unsigned long TURN_SETTLE_MS   = 200;

void turnToHeading(float targetDeg, int pwm, float toleranceDeg, unsigned long timeoutMs) {
  drivingStraight = false;
  headingIntegral = 0.0;

  // ── Disable CAL factors — turns use raw symmetric PWM ────────────────────
  applyCalFactors = false;

  // ── Forced symmetric stop — clears any asymmetric PID state ──────────────
  setSides(0, 0);

  // ── Settle ────────────────────────────────────────────────────────────────
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

  // stopAll() restores applyCalFactors = true for the next driveAll()
  stopAll();
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
  Serial.print(F("Motors | Left PWM: "));    Serial.print(_lastLeftPWM);
  Serial.print(F("  | Right PWM: "));        Serial.println(_lastRightPWM);
  Serial.print(F("FL     | RPM: "));         Serial.print(rpmFL, 1);
  Serial.print(F("  | Dist: "));    Serial.print(distFL, 3); Serial.println(F(" m"));
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
  // ENC_BR not attached — BR encoder faulty

  pinMode(PWM_FL, OUTPUT); pinMode(PWM_FR, OUTPUT);
  pinMode(PWM_BL, OUTPUT); pinMode(PWM_BR, OUTPUT);
  pinMode(IN1_FL, OUTPUT); pinMode(IN2_FL, OUTPUT);
  pinMode(IN1_FR, OUTPUT); pinMode(IN2_FR, OUTPUT);
  pinMode(IN1_BL, OUTPUT); pinMode(IN2_BL, OUTPUT);
  pinMode(IN1_BR, OUTPUT); pinMode(IN2_BR, OUTPUT);

  setFL(0); setFR(0); setBL(0); setBR(0);

  lastReportTime        = millis();
  sequenceStart         = millis();
  lastHeadingUpdateTime = millis();
  drivingStraight       = false;
  applyCalFactors       = true;
  Serial.println(F("Ready."));
}


// ════════════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  updateBrakes();

  if (now - lastIMUTime >= IMU_INTERVAL_MS) {
    lastIMUTime  = now;
    mpu.update();
    gyroZ        = mpu.getGyroZ();
    totalAngleZ  = mpu.getAngleZ();
  }

  unsigned long elapsed = now - sequenceStart;
  if (elapsed < 5000) {
    driveAll(DRIVE_PWM);
  } else if (elapsed == 5000 || (elapsed > 5000 && elapsed < 5100)) {
    static bool turned = false;
    if (!turned) {
      turned = true;
      turnToHeading(90.0, 80, 1.0, 3000);
    }
  } else {
    stopAll();
  }

  // ── IMU heading PID correction (20 Hz) ──────────────────────────────────
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

      int leftPWM  = constrain(DRIVE_PWM - (int)headingCorrection, 0, 255);
      int rightPWM = constrain(DRIVE_PWM + (int)headingCorrection, 0, 255);
      setSides(leftPWM, rightPWM);
    } else {
      headingIntegral *= 0.95;
      setSides(DRIVE_PWM, DRIVE_PWM);
    }
  }

  // ── Serial report ────────────────────────────────────────────────────────
  if (elapsed < 6000 && now - lastReportTime >= REPORT_INTERVAL_MS) {
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
    lastReportTime = now;
  }
}