/*
 * 4-Motor Differential Drive Robot
 * Single-Channel Encoder + L298N Motor Driver + PID Speed Control
 *
 * Board:   Arduino Uno / Nano
 * Library: PinChangeInterrupt (install via Library Manager)
 *          Sketch -> Include Library -> Manage Libraries
 *          Search: "PinChangeInterrupt" by NicoHood
 *
 * ── Drive Architecture ───────────────────────────────────────────────────────
 *
 *   L298N #1 handles FRONT motors (FL via CH-A, FR via CH-B)
 *   L298N #2 handles BACK  motors (BL via CH-A, BR via CH-B)
 *
 *   PID channels match physical L298N boards:
 *     FRONT PID: FR only (FL encoder faulty) → controls L298N #1
 *     BACK  PID: BL + BR average             → controls L298N #2
 *
 *   Differential steering is achieved by setting different speeds on
 *   FRONT and BACK boards — but since each board drives one left and
 *   one right motor, true differential steering requires per-motor PWM.
 *   See setSides() which sets all four motors independently.
 *
 *   Single-channel encoders: only the A channel is wired and used.
 *   Direction is inferred from the motor command rather than the encoder.
 *   This frees up 4 pins, resolving all conflicts on the Uno.
 *
 * ── Pin Map ───────────────────────────────────────────────────────────────────
 *
 *   D0, D1  — reserved for Serial (do not use)
 *   D2      — free (spare)
 *   D3      — PWM FL    (hardware PWM)
 *   D4      — Encoder FL – A
 *   D5      — PWM FR    (hardware PWM)
 *   D6      — PWM BL    (hardware PWM)
 *   D7      — IN1 BL    (BL forward)
 *   D8      — Encoder BL – A
 *   D9      — PWM BR    (hardware PWM)
 *   D10     — Encoder FR – A
 *   D11     — IN2 BL    (BL reverse)
 *   D12     — IN1 BR    (BR forward)
 *   D13     — IN2 BR    (BR reverse)
 *   A0      — Encoder BR – A
 *   A1      — IN1 FL    (FL forward)
 *   A2      — IN2 FL    (FL reverse)
 *   A3      — IN1 FR    (FR forward)
 *   A4      — IN2 FR    (FR reverse)
 *   A5      — free (spare)
 *
 * ── Wiring Summary ───────────────────────────────────────────────────────────
 *
 *   ENCODERS (A channel only — connect only the A wire from each encoder)
 *   Motor FL: A=D4
 *   Motor FR: A=D10
 *   Motor BL: A=D8
 *   Motor BR: A=A0
 *
 *   L298N #1 — FRONT (FL=CH-A, FR=CH-B)   L298N #2 — BACK (BL=CH-A, BR=CH-B)
 *   ──────────────────────────────          ──────────────────────────────────
 *   ENA  → D3  (PWM, FL)                   ENA  → D6  (PWM, BL)
 *   IN1  → A1  (FL forward)                IN1  → D7  (BL forward)
 *   IN2  → A2  (FL reverse)                IN2  → D11 (BL reverse)
 *   ENB  → D5  (PWM, FR)                   ENB  → D9  (PWM, BR)
 *   IN3  → A3  (FR forward)                IN3  → D12 (BR forward)
 *   IN4  → A4  (FR reverse)                IN4  → D13 (BR reverse)
 *
 * ── Power ────────────────────────────────────────────────────────────────────
 *   - Motor battery (6-12V) → L298N Vs on both boards
 *   - L298N 5V jumper ON if battery <= 12V
 *   - Connect ALL GNDs: Arduino, both L298Ns, battery negative
 *   - Do NOT draw motor power from Arduino's 5V pin
 */

#include <PinChangeInterrupt.h>

// ════════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ════════════════════════════════════════════════════════════════════════════

// Encoder A channel pins only — 0=FL, 1=FR, 2=BL, 3=BR
const uint8_t ENC_A[4] = {4, 10, 8, A0};

// PWM pins — all hardware PWM pins on Uno
const uint8_t PWM_FL = 3;
const uint8_t PWM_FR = 5;
const uint8_t PWM_BL = 6;
const uint8_t PWM_BR = 9;

// Direction pins
const uint8_t IN1_FL = A1;  const uint8_t IN2_FL = A2;
const uint8_t IN1_FR = A3;  const uint8_t IN2_FR = A4;
const uint8_t IN1_BL = 7;   const uint8_t IN2_BL = 11;
const uint8_t IN1_BR = 12;  const uint8_t IN2_BR = 13;


// ════════════════════════════════════════════════════════════════════════════
//  CONFIGURATION
// ════════════════════════════════════════════════════════════════════════════

const int   PULSES_PER_REV      = 225;   // 225 pulses per revolution
const float WHEEL_DIAMETER_M    = 0.0762; // Wheel diameter in metres (3 inches)
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_M;

const unsigned long PID_INTERVAL_MS           = 50;  // PID update rate (20 Hz)
const unsigned long DIRECTION_CHANGE_DELAY_MS = 80;  // Brake time between direction changes (ms)
const unsigned long REPORT_INTERVAL_MS        = 200; // Serial print rate

// ── PID Gains ────────────────────────────────────────────────────────────────
// Tuning guide:
//   Kp — increase if response to speed changes is too slow
//   Ki — increase if speed drifts from target at steady state
//   Kd — increase slightly if speed oscillates around the target
float Kp = 2.0;
float Ki = 1.5;
float Kd = 0.05;

// Maximum RPM your motors can physically reach at full PWM
const float MAX_RPM = 150.0;


// ════════════════════════════════════════════════════════════════════════════
//  ENCODER STATE
// ════════════════════════════════════════════════════════════════════════════

// Counts increment on every rising edge of the A channel.
// Direction is tracked separately via motorDirection[].
volatile long encoderCount[4] = {0, 0, 0, 0};  // FL, FR, BL, BR

// +1 = forward, -1 = reverse — set by setMotor() when direction changes
int motorDirection[4] = {1, 1, 1, 1};

// Non-blocking direction change: track which motors are in a brake pause
bool  motorBraking[4]   = {false, false, false, false};
int   motorPending[4]   = {0, 0, 0, 0};      // Speed to apply after brake completes
unsigned long brakeStart[4] = {0, 0, 0, 0};  // millis() when brake began

long  prevCount[4] = {0, 0, 0, 0};
float rpm[4]       = {0.0, 0.0, 0.0, 0.0};

const char* MOTOR_LABELS[4] = {"FL", "FR", "BL", "BR"};


// ════════════════════════════════════════════════════════════════════════════
//  PID STATE  (one controller per side: LEFT and RIGHT)
// ════════════════════════════════════════════════════════════════════════════

struct PIDState {
  float setpoint;   // Target RPM (positive = forward, negative = reverse)
  float integral;   // Accumulated integral term
  float prevError;  // Error from last update (for derivative)
  int   basePWM;    // Requested PWM before PID correction (-255 to +255)
  int   outputPWM;  // Final PWM after PID correction
};

PIDState pidFront = {0, 0, 0, 0, 0};
PIDState pidBack  = {0, 0, 0, 0, 0};

unsigned long lastPIDTime    = 0;
unsigned long lastReportTime = 0;

// Per-side RPM targets for differential steering
float _leftRPM  = 0;
float _rightRPM = 0;

// Sequence state
unsigned long sequenceStart = 0;  // millis() when sequence began


// ════════════════════════════════════════════════════════════════════════════
//  INTERRUPT SERVICE ROUTINES
//  Single channel — count every rising edge, apply direction sign
// ════════════════════════════════════════════════════════════════════════════

void ISR_FL() { encoderCount[0] += motorDirection[0]; }
void ISR_FR() { encoderCount[1] += motorDirection[1]; }
void ISR_BL() { encoderCount[2] += motorDirection[2]; }
void ISR_BR() { encoderCount[3] += motorDirection[3]; }



// ════════════════════════════════════════════════════════════════════════════
//  PID CONTROLLER
// ════════════════════════════════════════════════════════════════════════════

int computePID(PIDState& pid, float measuredRPM, float dt) {
  float error   = pid.setpoint - measuredRPM;
  pid.integral += error * dt;
  pid.integral  = constrain(pid.integral, -255.0 / Ki, 255.0 / Ki); // Anti-windup
  float derivative = (error - pid.prevError) / dt;
  pid.prevError = error;
  float correction = Kp * error + Ki * pid.integral + Kd * derivative;
  int output = constrain(pid.basePWM + (int)correction, -255, 255);
  pid.outputPWM = output;
  return output;
}


// ════════════════════════════════════════════════════════════════════════════
//  MOTOR DRIVER
//  speed: -255 (full reverse) to +255 (full forward), 0 = brake
// ════════════════════════════════════════════════════════════════════════════

void applyMotor(uint8_t pwmPin, uint8_t in1, uint8_t in2, int speed) {
  // Low-level: immediately apply speed and direction to the hardware pins.
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

  // If direction is reversing, start a non-blocking brake pause.
  // Store the intended speed and apply it once the pause expires in updateBrakes().
  if (newDirection != 0 && motorDirection[idx] != 0 && newDirection != motorDirection[idx]) {
    motorBraking[idx] = true;
    motorPending[idx] = speed;
    brakeStart[idx]   = millis();
    motorDirection[idx] = 0;
    applyMotor(pwmPin, in1, in2, 0);  // Begin brake
    return;
  }

  // Not a direction reversal — apply immediately
  if (!motorBraking[idx]) {
    motorDirection[idx] = newDirection;
    applyMotor(pwmPin, in1, in2, speed);
  }
}

// Call once per loop() — releases motors from brake pause when delay has elapsed
void updateBrakes() {
  unsigned long now = millis();
  if (motorBraking[0] && now - brakeStart[0] >= DIRECTION_CHANGE_DELAY_MS) {
    motorBraking[0]   = false;
    motorDirection[0] = (motorPending[0] > 0) ? 1 : -1;
    applyMotor(PWM_FL, IN1_FL, IN2_FL, motorPending[0]);
  }
  if (motorBraking[1] && now - brakeStart[1] >= DIRECTION_CHANGE_DELAY_MS) {
    motorBraking[1]   = false;
    motorDirection[1] = (motorPending[1] > 0) ? 1 : -1;
    applyMotor(PWM_FR, IN1_FR, IN2_FR, motorPending[1]);
  }
  if (motorBraking[2] && now - brakeStart[2] >= DIRECTION_CHANGE_DELAY_MS) {
    motorBraking[2]   = false;
    motorDirection[2] = (motorPending[2] > 0) ? 1 : -1;
    applyMotor(PWM_BL, IN1_BL, IN2_BL, motorPending[2]);
  }
  if (motorBraking[3] && now - brakeStart[3] >= DIRECTION_CHANGE_DELAY_MS) {
    motorBraking[3]   = false;
    motorDirection[3] = (motorPending[3] > 0) ? 1 : -1;
    applyMotor(PWM_BR, IN1_BR, IN2_BR, motorPending[3]);
  }
}

void setFL(int s) { setMotor(0, PWM_FL, IN1_FL, IN2_FL, s); }
void setFR(int s) { setMotor(1, PWM_FR, IN1_FR, IN2_FR, s); }
void setBL(int s) { setMotor(2, PWM_BL, IN1_BL, IN2_BL, s); }
void setBR(int s) { setMotor(3, PWM_BR, IN1_BR, IN2_BR, s); }


// ════════════════════════════════════════════════════════════════════════════
//  MOVEMENT API — all speeds in RPM
// ════════════════════════════════════════════════════════════════════════════

// Drive each motor at its own RPM target.
// FL and FR are on L298N #1 (front board).
// BL and BR are on L298N #2 (back board).
// PID corrects each board independently using its encoder feedback.
void setSides(float leftRPM, float rightRPM) {
  leftRPM  = constrain(leftRPM,  -MAX_RPM, MAX_RPM);
  rightRPM = constrain(rightRPM, -MAX_RPM, MAX_RPM);

  // Front board PID targets the average of FL+FR.
  // Since FL encoder is faulty, front PID is driven by FR only (see PID loop).
  // We use the average of left and right RPM as the front board target.
  float frontRPM = (leftRPM + rightRPM) / 2.0;
  pidFront.setpoint = frontRPM;
  pidFront.basePWM  = (int)(frontRPM / MAX_RPM * 200);

  // Back board PID targets the average of BL+BR.
  // BL and BR may differ in speed — averaging them gives the back board target.
  float backRPM = (leftRPM + rightRPM) / 2.0;
  pidBack.setpoint = backRPM;
  pidBack.basePWM  = (int)(backRPM / MAX_RPM * 200);

  // Store individual left/right targets so motors can be driven asymmetrically
  // for differential steering within each board.
  _leftRPM  = leftRPM;
  _rightRPM = rightRPM;
}

void driveAll(float rpm)                  { setSides(rpm, rpm);         }
void stopAll()                            { setSides(0, 0);             }
void turnLeft(float rpm)                  { setSides(-rpm, rpm);        }
void turnRight(float rpm)                 { setSides(rpm, -rpm);        }
void curveLeft(float outer, float inner)  { setSides(inner, outer);     }
void curveRight(float outer, float inner) { setSides(outer, inner);     }


// ════════════════════════════════════════════════════════════════════════════
//  ENCODER UTILITIES
// ════════════════════════════════════════════════════════════════════════════

void resetEncoders() {
  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) {
    encoderCount[i] = 0;
    prevCount[i]    = 0;
  }
  interrupts();
  for (uint8_t i = 0; i < 4; i++) rpm[i] = 0;
  pidFront.integral = 0; pidFront.prevError = 0;
  pidBack.integral  = 0; pidBack.prevError  = 0;
}


// ════════════════════════════════════════════════════════════════════════════
//  SERIAL OUTPUT
// ════════════════════════════════════════════════════════════════════════════

void printData(long* counts) {
  Serial.println(F("----------------------------------------"));
  for (uint8_t i = 0; i < 4; i++) {
    float totalRevs = (float)counts[i] / PULSES_PER_REV;
    float distM     = totalRevs * WHEEL_CIRCUMFERENCE;
    Serial.print(F("Motor ")); Serial.print(MOTOR_LABELS[i]);
    Serial.print(F("  | Count: ")); Serial.print(counts[i]);
    Serial.print(F("  | Dist: "));  Serial.print(distM, 3); Serial.print(F(" m"));
    Serial.print(F("  | RPM: "));   Serial.println(rpm[i], 1);
  }
  float frontAvgRPM = rpm[1];                    // FR only (FL encoder faulty)
  float backAvgRPM  = (rpm[2] + rpm[3]) / 2.0;  // BL + BR
  Serial.print(F("FRONT | Target: ")); Serial.print(pidFront.setpoint, 1);
  Serial.print(F(" RPM  | Actual: ")); Serial.print(frontAvgRPM, 1);
  Serial.print(F(" RPM  | PWM: "));   Serial.println(pidFront.outputPWM);
  Serial.print(F("BACK  | Target: ")); Serial.print(pidBack.setpoint, 1);
  Serial.print(F(" RPM  | Actual: ")); Serial.print(backAvgRPM, 1);
  Serial.print(F(" RPM  | PWM: "));   Serial.println(pidBack.outputPWM);
}


// ════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Differential Drive Robot w/ PID ==="));
  Serial.println(F("L298N #1: FL+FR  |  L298N #2: BL+BR"));
  Serial.println(F("PID FRONT: FR only (FL faulty) |  PID BACK: BL+BR"));
  Serial.println(F("Encoders: single channel (A only)"));

  // Encoder A pins
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(ENC_A[i], INPUT_PULLUP);
  }

  // Motor driver pins
  pinMode(PWM_FL, OUTPUT); pinMode(PWM_FR, OUTPUT);
  pinMode(PWM_BL, OUTPUT); pinMode(PWM_BR, OUTPUT);
  pinMode(IN1_FL, OUTPUT); pinMode(IN2_FL, OUTPUT);
  pinMode(IN1_FR, OUTPUT); pinMode(IN2_FR, OUTPUT);
  pinMode(IN1_BL, OUTPUT); pinMode(IN2_BL, OUTPUT);
  pinMode(IN1_BR, OUTPUT); pinMode(IN2_BR, OUTPUT);

  // Ensure motors off at startup
  setFL(0); setFR(0); setBL(0); setBR(0);

  // Attach rising-edge interrupts on encoder A pins
  // NOTE: FL encoder is faulty — ISR_FL not attached, left side PID uses BL only
  attachPCINT(digitalPinToPCINT(ENC_A[1]), ISR_FR, RISING);
  attachPCINT(digitalPinToPCINT(ENC_A[2]), ISR_BL, RISING);
  attachPCINT(digitalPinToPCINT(ENC_A[3]), ISR_BR, RISING);

  lastPIDTime    = millis();
  lastReportTime = millis();
  sequenceStart  = millis();
  Serial.println(F("Ready."));
}


// ════════════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // ── Non-blocking brake release ────────────────────────────────────────────
  updateBrakes();

  // ── Motion sequence (runs first so setpoints are set before PID acts) ────
  unsigned long elapsed = now - sequenceStart;
  if (elapsed < 5000) {
    driveAll(80);   // Drive straight at 80 RPM for 5 seconds
  } else {
    stopAll();      // Stop after 5 seconds
  }

  // ── PID update ────────────────────────────────────────────────────────────
  if (now - lastPIDTime >= PID_INTERVAL_MS) {
    float dt = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;

    long snapshot[4];
    noInterrupts();
    for (uint8_t i = 0; i < 4; i++) snapshot[i] = encoderCount[i];
    interrupts();

    for (uint8_t i = 0; i < 4; i++) {
      long  delta  = snapshot[i] - prevCount[i];
      float revs   = (float)delta / PULSES_PER_REV;
      rpm[i]       = (revs / dt) * 60.0;
      prevCount[i] = snapshot[i];
    }

    float frontAvgRPM = rpm[1];                    // FR only (FL encoder faulty)
    float backAvgRPM  = (rpm[2] + rpm[3]) / 2.0;  // BL + BR

    if (pidFront.setpoint == 0 && pidBack.setpoint == 0) {
      setFL(0); setFR(0); setBL(0); setBR(0);
    } else {
      int frontPWM = computePID(pidFront, frontAvgRPM, dt);
      int backPWM  = computePID(pidBack,  backAvgRPM,  dt);

      // Apply front board PWM — scale left/right asymmetrically for steering
      float frontScale = (pidFront.setpoint != 0) ? (float)frontPWM / (pidFront.setpoint / MAX_RPM * 200 + 0.001) : 1.0;
      setFL((int)(_leftRPM  / MAX_RPM * 200 * frontScale));
      setFR((int)(_rightRPM / MAX_RPM * 200 * frontScale));

      // Apply back board PWM — same scaling for back motors
      float backScale  = (pidBack.setpoint  != 0) ? (float)backPWM  / (pidBack.setpoint  / MAX_RPM * 200 + 0.001) : 1.0;
      setBL((int)(_leftRPM  / MAX_RPM * 200 * backScale));
      setBR((int)(_rightRPM / MAX_RPM * 200 * backScale));
    }
  }

  // ── Serial report ─────────────────────────────────────────────────────────
  if (now - lastReportTime >= REPORT_INTERVAL_MS) {
    long snapshot[4];
    noInterrupts();
    for (uint8_t i = 0; i < 4; i++) snapshot[i] = encoderCount[i];
    interrupts();
    printData(snapshot);
    lastReportTime = now;
  }
}
