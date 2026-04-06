// ─────────────────────────────────────────────
//  Robotic Arm Controller and Ramp Control
//  Interface: Serial commands from Raspberry Pi
// ─────────────────────────────────────────────

#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

//----------------------------------------------
// John's Definitions and Initialization
//----------------------------------------------

// ── Pin Definitions ───────────────────────────
#define BASE_PIN        3
#define SHOULDER_PIN    5
#define ELBOW_PIN       6

#define CLAW_IN1        7    // DC motor direction pin 1
#define CLAW_IN2        8    // DC motor direction pin 2
#define CLAW_PWM        9    // DC motor speed (PWM)

// ── Servo Angle Limits ────────────────────────
#define BASE_MIN        0
#define BASE_MAX        180
#define SHOULDER_MIN    55
#define SHOULDER_MAX    180
#define ELBOW_MIN       0
#define ELBOW_MAX       180

// ── Movement Tuning ───────────────────────────
#define SERVO_STEP      2       // Degrees per loop tick (lower = smoother/slower)
#define SERVO_DELAY_MS  15      // Delay between steps in ms
#define CLAW_SPEED      200     // DC motor PWM speed (0–255)
#define CLAW_TIMEOUT_MS 2000    // Max ms to run claw motor before assuming done

// ── State Machine ─────────────────────────────
//  MOVING_POSITION : Phase 1 — move base and elbow into position first
//  MOVING_SHOULDER : Phase 2 — lower shoulder once base/elbow are in place
enum State {
  IDLE,
  MOVING_POSITION,
  MOVING_SHOULDER,
  CLAW_CLOSING,
  CLAW_OPENING
};

State currentState = IDLE;

// ── Servo Objects ─────────────────────────────
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;

// ── Position Tracking ─────────────────────────
int currentBase     = 90;
int currentShoulder = 90;
int currentElbow    = 90;

int targetBase      = 90;
int targetShoulder  = 90;
int targetElbow     = 90;

// ── Claw Timing ───────────────────────────────
unsigned long clawStartTime = 0;

// ── Serial Input Buffer ───────────────────────
String inputBuffer = "";

//-----------------------------------------------
// Tuan's Definitions and Setup
//-----------------------------------------------

// ---------- Pins ----------
#define SERVO_PIN  12
#define LEFT_DOOR  5
#define RIGHT_DOOR 6
 
// ---------- L298N ----------
#define ENA 11
#define IN1 7
#define IN2 8
 
// ---------- Servo ----------
Servo latchServo;
Servo leftDoor;
Servo rightDoor;
 
int currentServoPos = 90;
bool motorOn = false;
// ---------- Color Sensor ----------
Adafruit_TCS34725 tcs =
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);
 
// ---------- Constants ----------
const float MARGIN = 0.08;
const int SERVO_CENTER = 105;
const int SERVO_RIGHT = 35;
const int SERVO_LEFT = 162;
const int SERVO_LGOAL   = 150;
const int SERVO_RGOAL  = 10;
const int SERVO_LSTART = 115;
const int SERVO_RSTART = 95;
 
// ---------- Servo Control ----------
void setServo(int target) {
  if (currentServoPos != target) {
    latchServo.write(target);
    currentServoPos = target;
  }
}
 
// ---------- SERIAL DOOR CONTROL ----------
void handleSerial() {
  while (Serial.available()) {
    char cmd = Serial.read();
 
    // ---- DOORS ----
    if (cmd == 'L') {
      Serial.println("Opening LEFT door");
      leftDoor.write(SERVO_LGOAL);
      delay(500);
      leftDoor.write(SERVO_LSTART);
    }
    else if (cmd == 'R') {
      Serial.println("Opening RIGHT door");
      rightDoor.write(SERVO_RGOAL);
      delay(500);
      rightDoor.write(SERVO_RSTART);
    }
 
    // ---- MOTOR CONTROL ----
    else if (cmd == 'B') {
      Serial.println("Motors ON");
      motorOn = true;
 
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 106);  // adjust speed here
    }
 
    else if (cmd == 'S') {
      Serial.println("Motors OFF");
      motorOn = false;
 
      analogWrite(ENA, 0);
    }
  }
}

// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
//----------------------------------------------
// John Setup
//----------------------------------------------
  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);

  pinMode(CLAW_IN1, OUTPUT);
  pinMode(CLAW_IN2, OUTPUT);
  pinMode(CLAW_PWM, OUTPUT);

  // Move to home position
  baseServo.write(currentBase);
  shoulderServo.write(currentShoulder);
  elbowServo.write(currentElbow);
  stopClaw();

  sendStatus("READY");

  //---------------------------------------
  // Tuan Setup
  //---------------------------------------
    // Servos
  latchServo.attach(SERVO_PIN);
  leftDoor.attach(LEFT_DOOR);
  rightDoor.attach(RIGHT_DOOR);
 
  setServo(SERVO_CENTER);
  leftDoor.write(SERVO_LSTART);
  rightDoor.write(SERVO_RSTART);
 
  // ---------- L298N SETUP ----------
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
 
  // Color sensor
  if (!tcs.begin()) {
    while (1);
  }
}

// ─────────────────────────────────────────────
void loop() {
  //--------------------------------------------
  // John's loop functionality
  //--------------------------------------------
  readSerial();

  switch (currentState) {

    // ── IDLE: Do nothing, wait for commands ───
    case IDLE:
      break;

    // ── MOVING_POSITION: Phase 1 — move base and elbow only ──
    case MOVING_POSITION:
      currentBase  = moveToward(currentBase,  targetBase,  SERVO_STEP);
      currentElbow = moveToward(currentElbow, targetElbow, SERVO_STEP);

      baseServo.write(currentBase);
      elbowServo.write(currentElbow);

      delay(SERVO_DELAY_MS);

      // Once base and elbow are in place, move to phase 2
      if (currentBase == targetBase && currentElbow == targetElbow) {
        currentState = MOVING_SHOULDER;
        sendStatus("POSITIONING_SHOULDER");
      }
      break;

    // ── MOVING_SHOULDER: Phase 2 — lower shoulder into position ──
    case MOVING_SHOULDER:
      currentShoulder = moveToward(currentShoulder, targetShoulder, SERVO_STEP);

      shoulderServo.write(currentShoulder);

      delay(SERVO_DELAY_MS);

      if (currentShoulder == targetShoulder) {
        currentState = IDLE;
        sendStatus("AT_TARGET");
      }
      break;

    // ── CLAW_CLOSING: Run motor until timeout ─
    case CLAW_CLOSING:
      if (millis() - clawStartTime >= CLAW_TIMEOUT_MS) {
        stopClaw();
        currentState = IDLE;
        sendStatus("CLAW_CLOSED");
      }
      break;

    // ── CLAW_OPENING: Run motor until timeout ─
    case CLAW_OPENING:
      if (millis() - clawStartTime >= CLAW_TIMEOUT_MS) {
        stopClaw();
        currentState = IDLE;
        sendStatus("CLAW_OPEN");
      }
      break;
  }

  //--------------------------------------------
  // Tuan's loop functionality
  //--------------------------------------------
  // 🔹 NEW: Serial control
  handleSerial();
 
  // -------- COLOR SCANNING --------
  uint32_t rSum = 0, gSum = 0, bSum = 0, cSum = 0;
 
  for (int i = 0; i < 2; i++) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
 
    rSum += r;
    gSum += g;
    bSum += b;
    cSum += c;
  }
 
  uint16_t rAvg = rSum / 2;
  uint16_t gAvg = gSum / 2;
  uint16_t bAvg = bSum / 2;
  uint16_t cAvg = cSum / 2;
 
  if (cAvg < 40) return;
 
  float red   = (float)rAvg / cAvg;
  float green = (float)gAvg / cAvg;
  float blue  = (float)bAvg / cAvg;
 
  Serial.print("R: "); Serial.print(rAvg);
  Serial.print(" G: "); Serial.print(gAvg);
  Serial.print(" B: "); Serial.print(bAvg);
  Serial.print(" C: "); Serial.println(cAvg);
 
  // -------- COLOR DECISION --------
 
  if (red > 0.40 && green > 0.25 && blue < 0.20) {
    delay(1000);
    setServo(SERVO_LEFT);
    delay(200);
    setServo(SERVO_CENTER);
  }
  else if (green > red + MARGIN && green > blue + MARGIN) {
    delay(1000);
    setServo(SERVO_RIGHT);
    delay(200);
    setServo(SERVO_CENTER);
  }
  else if (red > green + MARGIN && red > blue + MARGIN) {
    delay(1000);
    setServo(SERVO_LEFT);
    delay(200);
    setServo(SERVO_CENTER);
  }
  else if (blue > red + MARGIN && blue > green + MARGIN) {
    delay(1000);
    setServo(SERVO_RIGHT);
    delay(200);
    setServo(SERVO_CENTER);
  }
}
//----------------------------------------------
// Everything beneath here pertains's to John's system
// ─────────────────────────────────────────────
//  Serial Parsing
//  Expected formats:
//    MOVE <base> <shoulder> <elbow>
//    CLAW OPEN
//    CLAW CLOSE
//    HOME
// ─────────────────────────────────────────────
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      parseCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

void parseCommand(String cmd) {
  cmd.toUpperCase();

  if (cmd.startsWith("MOVE")) {
    int b, s, e;
    int parsed = sscanf(cmd.c_str(), "MOVE %d %d %d", &b, &s, &e);
    if (parsed == 3) {
      targetBase     = constrain(b, BASE_MIN,     BASE_MAX);
      targetShoulder = constrain(s, SHOULDER_MIN, SHOULDER_MAX);
      targetElbow    = constrain(e, ELBOW_MIN,    ELBOW_MAX);
      currentState   = MOVING_POSITION;
      sendStatus("MOVING");
    } else {
      sendStatus("ERROR_BAD_ARGS");
    }

  } else if (cmd == "CLAW CLOSE") {
    closeClaw();
    clawStartTime = millis();
    currentState  = CLAW_CLOSING;
    sendStatus("CLAW_CLOSING");

  } else if (cmd == "CLAW OPEN") {
    openClaw();
    clawStartTime = millis();
    currentState  = CLAW_OPENING;
    sendStatus("CLAW_OPENING");

  } else if (cmd == "HOME") {
    targetBase     = 90;
    targetShoulder = 90;
    targetElbow    = 90;
    currentState   = MOVING_POSITION;
    sendStatus("MOVING");

  } else {
    sendStatus("ERROR_UNKNOWN_CMD");
  }
}

// ─────────────────────────────────────────────
//  Claw Motor Helpers
// ─────────────────────────────────────────────
void openClaw() {
  digitalWrite(CLAW_IN1, HIGH);
  digitalWrite(CLAW_IN2, LOW);
  analogWrite(CLAW_PWM, CLAW_SPEED);
}

void closeClaw() {
  digitalWrite(CLAW_IN1, LOW);
  digitalWrite(CLAW_IN2, HIGH);
  analogWrite(CLAW_PWM, CLAW_SPEED);
}

void stopClaw() {
  digitalWrite(CLAW_IN1, LOW);
  digitalWrite(CLAW_IN2, LOW);
  analogWrite(CLAW_PWM, 0);
}

// ─────────────────────────────────────────────
//  Servo Helpers
// ─────────────────────────────────────────────
int moveToward(int current, int target, int step) {
  if (abs(current - target) <= step) return target;
  return current + (current < target ? step : -step);
}

// (atTarget removed — phase 1 and phase 2 check their own joints independently)

// ─────────────────────────────────────────────
//  Status Output
// ─────────────────────────────────────────────
void sendStatus(const char* status) {
  Serial.println(status);
}
