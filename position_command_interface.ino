#include <FastAccelStepper.h>

// ======================================================
// PIN SETUP
// ======================================================
// Arduino Uno R3 + FastAccelStepper:
// STEP/PUL must be on pin 9 or pin 10.
const int EN_PIN      = 2;
const int DIR_PIN     = 3;
const int STEP_PIN    = 10;   // Use 9 or 10 only
const int RIGHT_LIMIT = 8;
const int LEFT_LIMIT  = 7;
const int LED_PIN     = 6;   // LED moved away from pin 9

// TB6600 enable is usually active LOW
const bool DRIVER_ENABLE_ACTIVE_LOW = true;

// Direction setting.
// If the actuator moves the wrong way, change this value.
const bool DIR_HIGH_COUNTS_UP = false;

// ======================================================
// MOTOR / ACTUATOR CALIBRATION
// ======================================================
const int FULL_STEPS_PER_REV = 200;

// IMPORTANT:
// This must match the TB6600 DIP switch microstepping setting.
const int MICROSTEPPING = 8;

const int STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPPING;

// Measured actuator travel per motor revolution
const float TRAVEL_MM_PER_REV = 70.0;
const float STEPS_PER_MM = STEPS_PER_REV / TRAVEL_MM_PER_REV;

// ======================================================
// POSITION LIMITS
// ======================================================
const float MIN_TARGET_MM = 5.0;
const float MAX_TARGET_MM = 165.0;

// Homing settings
const float HOME_BACKOFF_MM = 5.0;
const float HOME_SEARCH_MAX_MM = 230.0;

// ======================================================
// SPEED SETTINGS IN REAL-WORLD UNITS
// ======================================================
// Set these values in mm/s and mm/s^2.
// The code automatically converts them to steps/s and steps/s^2.

const float MOVE_SPEED_MM_S = 1000.0;    // normal move speed
const float ACCEL_MM_S2     = 9000.0;   //50 normal acceleration
const float HOME_SPEED_MM_S = 200.0;     // homing speed

// Converted values used by FastAccelStepper
const uint32_t MOVE_SPEED_STEPS_S =
  (uint32_t)(MOVE_SPEED_MM_S * STEPS_PER_MM + 0.5);

const uint32_t ACCEL_STEPS_S2 =
  (uint32_t)(ACCEL_MM_S2 * STEPS_PER_MM + 0.5);

const uint32_t HOME_SPEED_STEPS_S =
  (uint32_t)(HOME_SPEED_MM_S * STEPS_PER_MM + 0.5);

// Startup wait before automatic homing
const unsigned long STARTUP_WAIT_MS = 3000;

// Status heartbeat interval
const unsigned long HEARTBEAT_INTERVAL_MS = 1000;

// ======================================================
// LED STARTUP FLICKER SETTINGS
// ======================================================
const float LED_FLICKER_FREQUENCY_HZ = 25.0;
const unsigned long LED_FLICKER_DURATION_MS = 5000;

const unsigned long LED_HALF_PERIOD_MS =
  (unsigned long)(1000.0 / (2.0 * LED_FLICKER_FREQUENCY_HZ));

// ======================================================
// FASTACCELSTEPPER OBJECTS
// ======================================================
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Store commanded target so limit checks know direction of travel
long commandedTargetSteps = 0;

// ======================================================
// SYSTEM STATE
// ======================================================
bool homed = false;
bool fault = false;
bool driverEnabled = false;

float currentTargetMM = MIN_TARGET_MM;

unsigned long lastHeartbeatTime = 0;

enum FaultCode {
  NO_FAULT,
  LEFT_LIMIT_FAULT,
  RIGHT_LIMIT_FAULT,
  HOME_NOT_FOUND_FAULT,
  HOME_BACKOFF_FAULT,
  STOP_FAULT
};

FaultCode faultCode = NO_FAULT;

// ======================================================
// BASIC FUNCTIONS
// ======================================================

long getCurrentSteps() {
  if (stepper == NULL) return 0;
  return stepper->getCurrentPosition();
}

void hardStopStepperAtCurrentPosition() {
  if (stepper == NULL) return;

  long pos = stepper->getCurrentPosition();
  stepper->forceStopAndNewPosition(pos);
  commandedTargetSteps = pos;
}

void enableDriver(bool enable) {
  driverEnabled = enable;

  if (DRIVER_ENABLE_ACTIVE_LOW) {
    digitalWrite(EN_PIN, enable ? LOW : HIGH);
  } else {
    digitalWrite(EN_PIN, enable ? HIGH : LOW);
  }
}

void startupLedFlicker() {
  unsigned long startTime = millis();

  while (millis() - startTime < LED_FLICKER_DURATION_MS) {
    digitalWrite(LED_PIN, HIGH);
    delay(LED_HALF_PERIOD_MS);

    digitalWrite(LED_PIN, LOW);
    delay(LED_HALF_PERIOD_MS);
  }

  digitalWrite(LED_PIN, LOW);
}

bool leftLimitTriggered() {
  // NC limit switch with INPUT_PULLUP:
  // LOW  = normal / closed
  // HIGH = triggered / open / wire fault
  return digitalRead(LEFT_LIMIT) == HIGH;
}

bool rightLimitTriggered() {
  return digitalRead(RIGHT_LIMIT) == HIGH;
}

long mmToSteps(float mm) {
  if (mm >= 0) {
    return (long)(mm * STEPS_PER_MM + 0.5);
  } else {
    return (long)(mm * STEPS_PER_MM - 0.5);
  }
}

float stepsToMM(long steps) {
  return steps / STEPS_PER_MM;
}

float clampTarget(float targetMM) {
  if (targetMM < MIN_TARGET_MM) return MIN_TARGET_MM;
  if (targetMM > MAX_TARGET_MM) return MAX_TARGET_MM;
  return targetMM;
}

void applyMoveSettings() {
  if (stepper == NULL) return;

  stepper->setSpeedInHz(MOVE_SPEED_STEPS_S);
  stepper->setAcceleration(ACCEL_STEPS_S2);
}

void applyHomeSettings() {
  if (stepper == NULL) return;

  stepper->setSpeedInHz(HOME_SPEED_STEPS_S);
  stepper->setAcceleration(ACCEL_STEPS_S2);
}

// ======================================================
// FAULT / STATUS FUNCTIONS
// ======================================================

void printFault() {
  Serial.print(F("Fault: "));

  switch (faultCode) {
    case NO_FAULT:
      Serial.println(F("none"));
      break;

    case LEFT_LIMIT_FAULT:
      Serial.println(F("left limit triggered during motion"));
      break;

    case RIGHT_LIMIT_FAULT:
      Serial.println(F("right limit triggered during motion"));
      break;

    case HOME_NOT_FOUND_FAULT:
      Serial.println(F("home switch not found"));
      break;

    case HOME_BACKOFF_FAULT:
      Serial.println(F("home backoff failed"));
      break;

    case STOP_FAULT:
      Serial.println(F("STOP command received"));
      break;

    default:
      Serial.println(F("unknown"));
      break;
  }
}

void enterFault(FaultCode code) {
  fault = true;
  faultCode = code;

  hardStopStepperAtCurrentPosition();
  enableDriver(false);

  Serial.println(F("FAULT ACTIVE. Driver disabled."));
  printFault();
  Serial.println(F("Send CLEAR, then HOME before resuming."));
}

void clearFault() {
  fault = false;
  faultCode = NO_FAULT;
  Serial.println(F("Fault cleared."));
}

void emergencyStop() {
  enterFault(STOP_FAULT);
}

void printLimits() {
  Serial.print(F("Left limit: "));
  Serial.print(leftLimitTriggered() ? F("TRIGGERED/OPEN") : F("NORMAL/CLOSED"));

  Serial.print(F(" | Right limit: "));
  Serial.println(rightLimitTriggered() ? F("TRIGGERED/OPEN") : F("NORMAL/CLOSED"));
}

void printPosition() {
  Serial.print(F("POS "));
  Serial.print(stepsToMM(getCurrentSteps()), 2);
  Serial.print(F(" TARGET "));
  Serial.println(currentTargetMM, 2);
}

void printStatus() {
  Serial.println(F("========== STATUS =========="));

  Serial.print(F("Homed: "));
  Serial.println(homed ? F("YES") : F("NO"));

  Serial.print(F("Fault: "));
  Serial.println(fault ? F("YES") : F("NO"));

  Serial.print(F("Driver enabled: "));
  Serial.println(driverEnabled ? F("YES") : F("NO"));

  printFault();
  printLimits();

  Serial.print(F("Position mm: "));
  Serial.println(stepsToMM(getCurrentSteps()), 2);

  Serial.print(F("Target mm: "));
  Serial.println(currentTargetMM, 2);

  Serial.print(F("Command range mm: "));
  Serial.print(MIN_TARGET_MM, 2);
  Serial.print(F(" to "));
  Serial.println(MAX_TARGET_MM, 2);

  Serial.print(F("Microstepping: "));
  Serial.println(MICROSTEPPING);

  Serial.print(F("Steps/rev: "));
  Serial.println(STEPS_PER_REV);

  Serial.print(F("Steps/mm: "));
  Serial.println(STEPS_PER_MM, 3);

  Serial.print(F("Move speed mm/s: "));
  Serial.println(MOVE_SPEED_MM_S, 1);

  Serial.print(F("Move speed steps/s: "));
  Serial.println(MOVE_SPEED_STEPS_S);

  Serial.print(F("Acceleration mm/s^2: "));
  Serial.println(ACCEL_MM_S2, 1);

  Serial.print(F("Acceleration steps/s^2: "));
  Serial.println(ACCEL_STEPS_S2);

  Serial.print(F("Home speed mm/s: "));
  Serial.println(HOME_SPEED_MM_S, 1);

  Serial.print(F("Home speed steps/s: "));
  Serial.println(HOME_SPEED_STEPS_S);

  Serial.print(F("Home backoff mm: "));
  Serial.println(HOME_BACKOFF_MM, 2);

  Serial.print(F("LED flicker frequency Hz: "));
  Serial.println(LED_FLICKER_FREQUENCY_HZ, 1);

  Serial.print(F("LED flicker duration ms: "));
  Serial.println(LED_FLICKER_DURATION_MS);

  Serial.println(F("============================"));
}

// ======================================================
// SERIAL LINE READER
// ======================================================

bool readLine(char *buffer, int bufferSize) {
  static int index = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      buffer[index] = '\0';
      index = 0;
      return true;
    }

    if (index < bufferSize - 1) {
      buffer[index++] = c;
    } else {
      index = 0;
      buffer[0] = '\0';
      return false;
    }
  }

  return false;
}

void uppercase(char *s) {
  for (int i = 0; s[i] != '\0'; i++) {
    if (s[i] >= 'a' && s[i] <= 'z') {
      s[i] = s[i] - 32;
    }
  }
}

bool isNumericCommand(const char *s) {
  bool hasDigit = false;
  bool hasDot = false;
  int start = 0;

  if (s[0] == '-' || s[0] == '+') {
    start = 1;
  }

  for (int i = start; s[i] != '\0'; i++) {
    if (s[i] >= '0' && s[i] <= '9') {
      hasDigit = true;
    } else if (s[i] == '.' && !hasDot) {
      hasDot = true;
    } else {
      return false;
    }
  }

  return hasDigit;
}

// ======================================================
// LIMIT SAFETY DURING NORMAL MOTION
// ======================================================

void checkLimitsDuringMotion() {
  if (fault || stepper == NULL) return;

  if (!stepper->isRunning()) return;

  long currentSteps = stepper->getCurrentPosition();
  long distanceToGo = commandedTargetSteps - currentSteps;

  // Only fault the limit in the current direction of travel.
  if (distanceToGo < 0 && leftLimitTriggered()) {
    enterFault(LEFT_LIMIT_FAULT);
    return;
  }

  if (distanceToGo > 0 && rightLimitTriggered()) {
    enterFault(RIGHT_LIMIT_FAULT);
    return;
  }
}

// ======================================================
// TARGET CONTROL
// ======================================================

void setTargetMM(float requestedMM) {
  if (stepper == NULL) {
    Serial.println(F("Target rejected: stepper not initialized."));
    return;
  }

  if (fault) {
    Serial.println(F("Target rejected: fault active."));
    return;
  }

  if (!homed) {
    Serial.println(F("Target rejected: actuator not homed."));
    return;
  }

  float targetMM = clampTarget(requestedMM);
  currentTargetMM = targetMM;

  commandedTargetSteps = mmToSteps(targetMM);

  applyMoveSettings();
  enableDriver(true);

  stepper->moveTo(commandedTargetSteps);

  Serial.print(F("TARGET "));
  Serial.println(targetMM, 2);
}

// ======================================================
// HELPER FOR HOMING MOVES
// ======================================================

bool moveRelativeAndMonitor(long moveSteps) {
  if (stepper == NULL) return false;

  long target = stepper->getCurrentPosition() + moveSteps;
  commandedTargetSteps = target;

  stepper->moveTo(target);

  while (stepper->isRunning()) {
    if (moveSteps > 0 && rightLimitTriggered()) {
      enterFault(RIGHT_LIMIT_FAULT);
      return false;
    }

    if (moveSteps < 0 && leftLimitTriggered()) {
      enterFault(LEFT_LIMIT_FAULT);
      return false;
    }

    delay(1);
  }

  return true;
}

// ======================================================
// HOMING FUNCTION
// ======================================================

void homeActuator() {
  if (stepper == NULL) {
    Serial.println(F("Cannot home: stepper not initialized."));
    return;
  }

  if (fault) {
    Serial.println(F("Cannot home: fault active. Send CLEAR first."));
    return;
  }

  Serial.println(F("Starting homing..."));
  enableDriver(true);

  applyHomeSettings();

  // If already pressing the left switch, back off first.
  if (leftLimitTriggered()) {
    Serial.println(F("Left limit already active. Backing off."));

    if (!moveRelativeAndMonitor(mmToSteps(HOME_BACKOFF_MM))) {
      return;
    }

    delay(200);

    if (leftLimitTriggered()) {
      enterFault(HOME_BACKOFF_FAULT);
      return;
    }
  }

  // Move left until the left limit triggers.
  long startSteps = stepper->getCurrentPosition();
  long maxSearchSteps = mmToSteps(HOME_SEARCH_MAX_MM);

  applyHomeSettings();

  Serial.println(F("Searching for home switch..."));

  stepper->runBackward();

  while (!leftLimitTriggered()) {
    if (rightLimitTriggered()) {
      enterFault(RIGHT_LIMIT_FAULT);
      return;
    }

    if (labs(stepper->getCurrentPosition() - startSteps) > maxSearchSteps) {
      enterFault(HOME_NOT_FOUND_FAULT);
      return;
    }

    delay(1);
  }

  Serial.println(F("Home switch found."));

  hardStopStepperAtCurrentPosition();
  delay(200);

  // Switch contact point is temporary zero for the backoff move.
  stepper->setCurrentPosition(0);
  commandedTargetSteps = 0;

  // Back off from the physical switch by 5 mm.
  Serial.println(F("Backing off home switch."));

  applyHomeSettings();

  if (!moveRelativeAndMonitor(mmToSteps(HOME_BACKOFF_MM))) {
    return;
  }

  delay(200);

  if (leftLimitTriggered()) {
    enterFault(HOME_BACKOFF_FAULT);
    return;
  }

  // The actuator has physically backed off 5 mm from the switch.
  // This backed-off safe position is now defined as software 0 mm.
  stepper->setCurrentPosition(0);
  commandedTargetSteps = 0;
  homed = true;

  applyMoveSettings();

  // After homing, command the actuator to the minimum target position.
  currentTargetMM = MIN_TARGET_MM;
  commandedTargetSteps = mmToSteps(currentTargetMM);
  stepper->moveTo(commandedTargetSteps);

  Serial.println(F("Homing complete."));
  printPosition();
}

// ======================================================
// COMMAND PROCESSING
// ======================================================

void processCommand(char *cmd) {
  if (cmd[0] == '\0') return;

  char upperCmd[32];
  strncpy(upperCmd, cmd, sizeof(upperCmd));
  upperCmd[sizeof(upperCmd) - 1] = '\0';
  uppercase(upperCmd);

  // Manual command: emergency stop
  if (strcmp(upperCmd, "STOP") == 0 || strcmp(upperCmd, "X") == 0) {
    emergencyStop();
    return;
  }

  // Manual command: clear fault
  if (strcmp(upperCmd, "CLEAR") == 0) {
    clearFault();
    return;
  }

  // Manual command: home
  if (strcmp(upperCmd, "HOME") == 0) {
    homeActuator();
    return;
  }

  // Manual command: status
  if (strcmp(upperCmd, "STATUS") == 0) {
    printStatus();
    return;
  }

  // Manual command: position
  if (strcmp(upperCmd, "POS") == 0) {
    printPosition();
    return;
  }

  // Normal Python command: single float number
  if (isNumericCommand(cmd)) {
    float requestedTarget = atof(cmd);
    setTargetMM(requestedTarget);
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(cmd);
}

// ======================================================
// SETUP
// ======================================================

void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(LEFT_LIMIT, INPUT_PULLUP);
  pinMode(RIGHT_LIMIT, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);

  // Flicker LED at selected frequency for selected duration as soon as Arduino starts.
  startupLedFlicker();

  enableDriver(false);

  // Initialize FastAccelStepper
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);

  if (stepper == NULL) {
    Serial.println(F("ERROR: FastAccelStepper could not connect to STEP pin."));
    Serial.println(F("On Arduino Uno R3, STEP must be on pin 9 or pin 10."));

    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
  }

  stepper->setDirectionPin(DIR_PIN, DIR_HIGH_COUNTS_UP);

  applyMoveSettings();

  Serial.println(F("Arduino actuator controller started with FastAccelStepper."));
  Serial.println(F("STEP/PUL must be wired to Arduino pin 9 or pin 10."));
  Serial.println(F("LED moved to Arduino pin 6."));
  Serial.println(F("Speed is set in mm/s and converted internally to steps/s."));
  Serial.println(F("Waiting before automatic homing..."));

  delay(STARTUP_WAIT_MS);

  printLimits();

  homeActuator();

  Serial.println(F("Ready."));
  Serial.println(F("Home: switch contact, back off 5 mm, then set backed-off position as software 0 mm."));
  Serial.print(F("Send float target from "));
  Serial.print(MIN_TARGET_MM, 0);
  Serial.print(F(" to "));
  Serial.print(MAX_TARGET_MM, 0);
  Serial.println(F(" mm."));
  Serial.println(F("Commands: STOP, X, CLEAR, HOME, STATUS, POS"));
}

// ======================================================
// MAIN LOOP
// ======================================================

void loop() {
  char cmd[32];

  // Read serial command from Python or Serial Monitor
  if (readLine(cmd, sizeof(cmd))) {
    processCommand(cmd);
  }

  // No stepper.run() needed with FastAccelStepper.
  // Pulses are handled by the library.
  if (!fault && homed) {
    checkLimitsDuringMotion();
  }

  // Slow heartbeat so Python can monitor the Arduino
  if (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatTime = millis();

    if (!fault && homed) {
      Serial.print(F("POS "));
      Serial.print(stepsToMM(getCurrentSteps()), 2);
      Serial.print(F(" TARGET "));
      Serial.println(currentTargetMM, 2);
    }
  }
}