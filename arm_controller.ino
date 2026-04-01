// ─────────────────────────────────────────────
//  Robotic Arm Controller
//  Hardware: Base, Shoulder, Elbow servos + DC claw motor
//  Interface: Serial commands from Raspberry Pi
// ─────────────────────────────────────────────

#include <Servo.h>

// ── Pin Definitions ───────────────────────────
#define BASE_PIN        11
#define SHOULDER_PIN    10
#define ELBOW_PIN       9

#define CLAW_IN1        3    // DC motor direction pin 1
#define CLAW_IN2        4    // DC motor direction pin 2
#define CLAW_PWM        5    // DC motor speed (PWM)

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
#define CLAW_SPEED      100     // DC motor PWM speed (0–255)
#define CLAW_TIMEOUT_MS 500    // Max ms to run claw motor before assuming done // 120 60 90

// ── State Machine ─────────────────────────────
enum State {
  IDLE,
  MOVING,
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

// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

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
}

// ─────────────────────────────────────────────
void loop() {
  readSerial();

  switch (currentState) {

    // ── IDLE: Do nothing, wait for commands ───
    case IDLE:
      break;

    // ── MOVING: Interpolate servos to target ──
    case MOVING:
      currentBase     = moveToward(currentBase,     targetBase,     SERVO_STEP);
      currentShoulder = moveToward(currentShoulder, targetShoulder, SERVO_STEP);
      currentElbow    = moveToward(currentElbow,    targetElbow,    SERVO_STEP);

      baseServo.write(currentBase);
      shoulderServo.write(currentShoulder);
      elbowServo.write(currentElbow);

      delay(SERVO_DELAY_MS);

      if (atTarget()) {
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
}

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
      currentState   = MOVING;
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
    currentState   = MOVING;
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

bool atTarget() {
  return currentBase     == targetBase &&
         currentShoulder == targetShoulder &&
         currentElbow    == targetElbow;
}

// ─────────────────────────────────────────────
//  Status Output
// ─────────────────────────────────────────────
void sendStatus(const char* status) {
  Serial.println(status);
}
