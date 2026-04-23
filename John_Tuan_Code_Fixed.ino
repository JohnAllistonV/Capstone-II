// ─────────────────────────────────────────────
//  Robotic Arm Controller + Ramp Control
//  Interface: Serial commands from Raspberry Pi
//
//  FIXES APPLIED:
//  1. Resolved pin conflict on IN1/IN2 (pins 7,8)
//     John's claw motor moved to A1, A2 (Tuan's wiring left intact)
//  2. Merged dual serial readers into one unified handler
//  3. Replaced blocking delay() in color loop with millis() timing
// ─────────────────────────────────────────────

#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// =============================================
//  JOHN — PIN DEFINITIONS
// =============================================
#define BUTTON_PIN      A3
#define BASE_PIN        2    // was 2
#define SHOULDER_PIN    3    // was 2
#define ELBOW_PIN       4    // unchanged

#define CLAW_IN1        10   // L298N IN3 — channel 2
#define CLAW_IN2        A0   // L298N IN4 — channel 2
#define CLAW_PWM        A1   // L298N ENB — channel 2 PWM

// ── Servo Angle Limits ────────────────────────
#define BASE_MIN        0
#define BASE_MAX        180
#define SHOULDER_MIN    0
#define SHOULDER_MAX    180
#define ELBOW_MIN       0
#define ELBOW_MAX       180

// ── Movement Tuning ───────────────────────────
#define SERVO_STEP      2
#define SERVO_DELAY_MS  15
#define CLAW_SPEED      150
#define CLAW_TIMEOUT_MS 2000

// ── State Machine ─────────────────────────────
enum State {
  IDLE,
  MOVING_POSITION,
  MOVING_SHOULDER,
  GASP,
  OPEN
};

State currentState = IDLE;

// ── Servo Objects ─────────────────────────────
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;

// ── Button Initialization ─────────────────────────
bool lastButtonState = HIGH;  // Unpressed state with pullup
bool running         = false; // Tracks START/STOP toggle state
int count = 0;                // Counter for Pushbutton

// ── Position Tracking ─────────────────────────
int currentBase     = 90;
int currentShoulder = 90;
int currentElbow    = 60;

int targetBase      = 90;
int targetShoulder  = 90;
int targetElbow     = 90;

// ── Claw Timing ───────────────────────────────
unsigned long clawStartTime = 0;

// ── Serial Input Buffer ───────────────────────
String inputBuffer = "";

// =============================================
//  TUAN — PIN DEFINITIONS
//  All original pin assignments preserved (already physically wired)
// =============================================
#define SERVO_PIN   12
#define LEFT_DOOR   5
#define RIGHT_DOOR  6

#define ENA         11
#define RAMP_IN1    7
#define RAMP_IN2    8

// ── Ramp Servos ───────────────────────────────
Servo latchServo;
Servo leftDoor;
Servo rightDoor;

int currentServoPos = 90;
bool motorOn = false;

// ── Color Sensor ──────────────────────────────
Adafruit_TCS34725 tcs =
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

// ── Ramp Constants ────────────────────────────
const float MARGIN       = 0.08;
const int SERVO_CENTER   = 105;
const int SERVO_RIGHT    = 35;
const int SERVO_LEFT     = 162;
const int SERVO_LGOAL    = 180;
const int SERVO_RGOAL    = 10;
const int SERVO_LSTART   = 115;
const int SERVO_RSTART   = 95;

// FIX 3: Non-blocking color sort timing
unsigned long colorActionStart = 0;
bool colorActionPending        = false;
int  pendingServoTarget        = SERVO_CENTER;

// =============================================
//  SETUP
// =============================================
void setup() {
  Serial.begin(115200);

  // ── Emergency Stop ──────────────────────────
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // ── John's Setup ──────────────────────────
  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);

  pinMode(CLAW_IN1, OUTPUT);
  pinMode(CLAW_IN2, OUTPUT);
  pinMode(CLAW_PWM, OUTPUT);

  baseServo.write(currentBase);
  shoulderServo.write(currentShoulder);
  elbowServo.write(currentElbow);
  stopClaw();

  sendStatus("READY");

  // ── Tuan's Setup ──────────────────────────
  latchServo.attach(SERVO_PIN);
  leftDoor.attach(LEFT_DOOR);
  rightDoor.attach(RIGHT_DOOR);

  setRampServo(SERVO_CENTER);
  leftDoor.write(SERVO_LSTART);
  rightDoor.write(SERVO_RSTART);

  pinMode(ENA,      OUTPUT);
  pinMode(RAMP_IN1, OUTPUT);
  pinMode(RAMP_IN2, OUTPUT);

  digitalWrite(RAMP_IN1, LOW);
  digitalWrite(RAMP_IN2, LOW);
  analogWrite(ENA, 0);

  if (!tcs.begin()) {
    while (1);  // Halt if color sensor not found
  }
}

// =============================================
//  LOOP
// =============================================
void loop() {

  // ─Single unified serial reader ───
  readSerial();
  bool currentButtonState = digitalRead(BUTTON_PIN);
  //--------------------------------------------------------------------- PUSHBUTTON TEST CODE----------------------------------------------------------------------------------

if (lastButtonState == HIGH && currentButtonState == LOW) {
  count++;

  if (count % 2 == 1) {
    Serial.println("Doors Open");
    Serial.println("Motors OFF");
    motorOn = false;

    digitalWrite(RAMP_IN1, LOW);
    digitalWrite(RAMP_IN2, LOW);

    rightDoor.write(SERVO_RGOAL);
    leftDoor.write(SERVO_LGOAL);
    delay(5000);
    rightDoor.write(SERVO_RSTART);
    leftDoor.write(SERVO_LSTART);
  } 
  else {
    Serial.println("Motors ON");
    motorOn = true;

    digitalWrite(RAMP_IN1, HIGH);
    digitalWrite(RAMP_IN2, LOW);
    analogWrite(ENA, 106);
  }

  delay(200); // simple debounce
}

lastButtonState = currentButtonState;

  // ── Emergency Stop ──────────────────
  //bool currentButtonState = digitalRead(BUTTON_PIN);

  // Detect falling edge (HIGH -> LOW = button pressed) -------------------------------------Actual E Stop Code---------------------------------------------
  // if (lastButtonState == HIGH && currentButtonState == LOW) {
  //   delay(50);  // Debounce

  //   running = !running;  // Toggle state

  //   if (running) {
  //     Serial.println("START");
  //   } else {
  //     Serial.println("STOP");
  //   }
  // }

  // lastButtonState = currentButtonState;

  // ── John's State Machine ──────────────────
  switch (currentState) {

    case IDLE:
      break;

    case MOVING_POSITION:
      currentBase  = moveToward(currentBase,  targetBase,  SERVO_STEP);
      currentElbow = moveToward(currentElbow, targetElbow, SERVO_STEP);

      baseServo.write(currentBase);
      elbowServo.write(currentElbow);

      delay(SERVO_DELAY_MS);

      if (currentBase == targetBase && currentElbow == targetElbow) {
        currentState = MOVING_SHOULDER;
        sendStatus("POSITIONING_SHOULDER");
      }
      break;

    case MOVING_SHOULDER:
      currentShoulder = moveToward(currentShoulder, targetShoulder, SERVO_STEP);
      shoulderServo.write(currentShoulder);

      delay(SERVO_DELAY_MS);

      if (currentShoulder == targetShoulder) {
        currentState = IDLE;
        sendStatus("AT_TARGET");
      }
      break;

    case GASP:
      if (millis() - clawStartTime >= CLAW_TIMEOUT_MS) {
        stopClaw();
        currentState = IDLE;
        sendStatus("CLAW_CLOSED");
      }
      break;

    case OPEN:
      if (millis() - clawStartTime >= CLAW_TIMEOUT_MS) {
        stopClaw();
        currentState = IDLE;
        sendStatus("CLAW_OPEN");
      }
      break;
  }

  // ── Tuan's Color Scanning ─────────────────
  // Only scan when arm is idle — prevents color sensor
  // reads from slowing down servo movement
  if (currentState != IDLE) return;

  uint32_t rSum = 0, gSum = 0, bSum = 0, cSum = 0;

  for (int i = 0; i < 2; i++) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    rSum += r; gSum += g; bSum += b; cSum += c;
  }

  uint16_t rAvg = rSum / 2;
  uint16_t gAvg = gSum / 2;
  uint16_t bAvg = bSum / 2;
  uint16_t cAvg = cSum / 2;

  if (cAvg < 40) return;

  float red   = (float)rAvg / cAvg;
  float green = (float)gAvg / cAvg;
  float blue  = (float)bAvg / cAvg;

  // Serial.print("R: "); Serial.print(rAvg);
  // Serial.print(" G: "); Serial.print(gAvg);
  // Serial.print(" B: "); Serial.print(bAvg);
  // Serial.print(" C: "); Serial.println(cAvg);

  // ── FIX 3: Non-blocking color sort ────────
  // Instead of delay(1000) blocking everything,
  // we set a pending action and handle it next loop
  if (!colorActionPending) {
    if ((red > 0.40 && green > 0.25 && blue < 0.20) ||
        (red > green + MARGIN && red > blue + MARGIN)) {
      pendingServoTarget  = SERVO_LEFT;
      colorActionPending  = true;
      colorActionStart    = millis();
      setRampServo(SERVO_LEFT);
    }
    else if ((green > red + MARGIN && green > blue + MARGIN) ||
             (blue > red + MARGIN  && blue > green + MARGIN)) {
      pendingServoTarget  = SERVO_RIGHT;
      colorActionPending  = true;
      colorActionStart    = millis();
      setRampServo(SERVO_RIGHT);
    }
  }

  // Return servo to center after 1000ms non-blocking
  if (colorActionPending && millis() - colorActionStart >= 1000) {
    setRampServo(SERVO_CENTER);
    colorActionPending = false;
  }
}

// =============================================
//  FIX 2: UNIFIED SERIAL READER
//  Routes to arm commands (MOVE/CLAW/HOME)
//  or ramp commands (L/R/B/S) automatically
// =============================================
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    // Ramp single-char commands handled immediately
    if (c == 'L') {
      Serial.println("Opening LEFT door");
      leftDoor.write(SERVO_LGOAL);
      delay(500);
      leftDoor.write(SERVO_LSTART);
      return;
    }
    else if (c == 'R') {
      Serial.println("Opening RIGHT door");
      rightDoor.write(SERVO_RGOAL);
      delay(500);
      rightDoor.write(SERVO_RSTART);
      return;
    }
    else if (c == 'B') {
      Serial.println("Motors ON");
      motorOn = true;
      digitalWrite(RAMP_IN1, HIGH);
      digitalWrite(RAMP_IN2, LOW);
      analogWrite(ENA, 106); //                                      RAMP 106
      return;
    }
    else if (c == 'S') {
      Serial.println("Motors OFF");
      motorOn = false;
      analogWrite(ENA, 0);
      return;
    }

    // Arm commands are newline-terminated strings
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        parseArmCommand(inputBuffer);
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

// =============================================
//  ARM COMMAND PARSER
// =============================================
void parseArmCommand(String cmd) {
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

  } else if (cmd == "GASP") {
    closeClaw();
    clawStartTime = millis();
    currentState  = GASP;
    sendStatus("CLAW_CLOSING");

  } else if (cmd == "OPEN") {
    openClaw();
    clawStartTime = millis();
    currentState  = OPEN;
    sendStatus("CLAW_OPENING");

  } else if (cmd == "HOME") {
    targetBase     = 90;
    targetShoulder = 90;
    targetElbow    = 60;
    currentState   = MOVING_POSITION;
    sendStatus("MOVING");

  } else {
    sendStatus("ERROR_UNKNOWN_CMD");
  }
}

// =============================================
//  CLAW MOTOR HELPERS
// =============================================
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

// =============================================
//  RAMP SERVO HELPER
// =============================================
void setRampServo(int target) {
  if (currentServoPos != target) {
    latchServo.write(target);
    currentServoPos = target;
  }
}

// =============================================
//  ARM SERVO HELPER
// =============================================
int moveToward(int current, int target, int step) {
  if (abs(current - target) <= step) return target;
  return current + (current < target ? step : -step);
}

// =============================================
//  STATUS OUTPUT
// =============================================
void sendStatus(const char* status) {
  Serial.println(status);
}
