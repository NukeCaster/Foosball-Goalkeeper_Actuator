#include <FastAccelStepper.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ======================================================
// PIN SETUP
// ======================================================
// Arduino Uno R3 + FastAccelStepper:
// STEP/PUL must be on pin 9 or pin 10.
const int EN_PIN      = 2;
const int DIR_PIN     = 3;
const int STEP_PIN    = 10;   // Use pin 9 or 10 only
const int RIGHT_LIMIT = 8;
const int LEFT_LIMIT  = 7;
const int LED_PIN     = 6;

// TB6600 enable is usually active LOW
const bool DRIVER_ENABLE_ACTIVE_LOW = true;

// If the actuator moves the wrong way, change this.
const bool DIR_HIGH_COUNTS_UP = false;

// ======================================================
// MOTOR / ACTUATOR CALIBRATION
// ======================================================
const int FULL_STEPS_PER_REV = 200;

// This must match the TB6600 DIP switch setting.
// This value can be changed at runtime using CONFIG.
int microstepping = 8;

// Measured actuator travel per motor revolution
const float TRAVEL_MM_PER_REV = 70.0;

int stepsPerRev = 0;
float stepsPerMM = 0.0;

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
float moveSpeedMM_S = 1000.0;
float accelMM_S2    = 9000.0;
float homeSpeedMM_S = 200.0;

uint32_t moveSpeedStepsS = 0;
uint32_t accelStepsS2    = 0;
uint32_t homeSpeedStepsS = 0;

// Startup wait before automatic homing
const unsigned long STARTUP_WAIT_MS = 3000;

// Status heartbeat interval
const unsigned long HEARTBEAT_INTERVAL_MS = 1000;

// ======================================================
// LED STARTUP FLICKER SETTINGS
// ======================================================
const float LED_FLICKER_FREQUENCY_HZ = 10.0;
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
// CONVERSION / SETTINGS
// ======================================================

void updateMotionConversions() {
  stepsPerRev = FULL_STEPS_PER_REV * microstepping;
  stepsPerMM = stepsPerRev / TRAVEL_MM_PER_REV;

  moveSpeedStepsS = (uint32_t)(moveSpeedMM_S * stepsPerMM + 0.5);
  accelStepsS2    = (uint32_t)(accelMM_S2 * stepsPerMM + 0.5);
  homeSpeedStepsS = (uint32_t)(homeSpeedMM_S * stepsPerMM + 0.5);
}

long mmToSteps(float mm) {
  if (mm >= 0) {
    return (long)(mm * stepsPerMM + 0.5);
  } else {
    return (long)(mm * stepsPerMM - 0.5);
  }
}

float stepsToMM(long steps) {
  return steps / stepsPerMM;
}

float clampTarget(float targetMM) {
  if (targetMM < MIN_TARGET_MM) return MIN_TARGET_MM;
  if (targetMM > MAX_TARGET_MM) return MAX_TARGET_MM;
  return targetMM;
}

void applyMoveSettings() {
  if (stepper == NULL) return;

  stepper->setSpeedInHz(moveSpeedStepsS);
  stepper->setAcceleration(accelStepsS2);
}

void applyHomeSettings() {
  if (stepper == NULL) return;

  stepper->setSpeedInHz(homeSpeedStepsS);
  stepper->setAcceleration(accelStepsS2);
}

// ======================================================
// MOTION TIME CALCULATION
// ======================================================

bool isTriangularMove(float distanceMM, float speedMM_S, float accelMM_S2) {
  float s = fabs(distanceMM);
  float v = speedMM_S;
  float a = accelMM_S2;

  if (s <= 0.0 || v <= 0.0 || a <= 0.0) {
    return true;
  }

  float accelDistance = (v * v) / (2.0 * a);

  return s < (2.0 * accelDistance);
}

float calculatedMoveTime(float distanceMM, float speedMM_S, float accelMM_S2) {
  float s = fabs(distanceMM);
  float v = speedMM_S;
  float a = accelMM_S2;

  if (s <= 0.0 || v <= 0.0 || a <= 0.0) {
    return 0.0;
  }

  float accelDistance = (v * v) / (2.0 * a);

  // Triangular profile: the actuator does not reach full commanded speed
  if (s < (2.0 * accelDistance)) {
    return 2.0 * sqrt(s / a);
  }

  // Trapezoidal profile: the actuator reaches full commanded speed
  return (s / v) + (v / a);
}

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
  // HIGH = triggered / open / broken wire
  return digitalRead(LEFT_LIMIT) == HIGH;
}

bool rightLimitTriggered() {
  return digitalRead(RIGHT_LIMIT) == HIGH;
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
  Serial.println(microstepping);

  Serial.print(F("Steps/rev: "));
  Serial.println(stepsPerRev);

  Serial.print(F("Steps/mm: "));
  Serial.println(stepsPerMM, 4);

  Serial.print(F("Move speed mm/s: "));
  Serial.println(moveSpeedMM_S, 1);

  Serial.print(F("Move speed steps/s: "));
  Serial.println(moveSpeedStepsS);

  Serial.print(F("Acceleration mm/s^2: "));
  Serial.println(accelMM_S2, 1);

  Serial.print(F("Acceleration steps/s^2: "));
  Serial.println(accelStepsS2);

  Serial.print(F("Home speed mm/s: "));
  Serial.println(homeSpeedMM_S, 1);

  Serial.print(F("Home speed steps/s: "));
  Serial.println(homeSpeedStepsS);

  Serial.println(F("============================"));
}

void printCSVHeader() {
  Serial.println(F("RESULT_FORMAT,microstepping,current_position_mm,new_commanded_position_mm,commanded_move_mm,commanded_speed_mm_s,commanded_accel_mm_s2,motion_profile,calculated_move_time_s,recorded_move_time_s,average_internal_speed_mm_s,internal_start_mm,internal_end_mm"));
  Serial.println(F("REPEAT_FORMAT,requested_cycles,completed_cycles,left_mm,right_mm,commanded_move_mm,commanded_speed_mm_s,commanded_accel_mm_s2,motion_profile,calculated_move_time_s,total_recorded_move_time_s,average_outward_time_s,average_return_time_s,average_outward_speed_mm_s,average_return_speed_mm_s,final_internal_mm"));
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

  // Only fault the limit in the direction of travel.
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
// MOVE HELPERS
// ======================================================

bool waitUntilMoveCompleteNoDelay() {
  while (stepper != NULL && stepper->isRunning()) {
    checkLimitsDuringMotion();

    if (fault) {
      return false;
    }
  }

  return true;
}

bool moveToMMAndWait(float requestedMM) {
  if (stepper == NULL) {
    Serial.println(F("MOVE_REJECTED,stepper_not_initialized"));
    return false;
  }

  if (fault) {
    Serial.println(F("MOVE_REJECTED,fault_active"));
    return false;
  }

  if (!homed) {
    Serial.println(F("MOVE_REJECTED,not_homed"));
    return false;
  }

  float targetMM = clampTarget(requestedMM);
  currentTargetMM = targetMM;

  commandedTargetSteps = mmToSteps(targetMM);

  applyMoveSettings();
  enableDriver(true);

  stepper->moveTo(commandedTargetSteps);

  while (stepper->isRunning()) {
    checkLimitsDuringMotion();

    if (fault) {
      return false;
    }

    delay(1);
  }

  return true;
}

void setTargetMM(float requestedMM) {
  if (moveToMMAndWait(requestedMM)) {
    Serial.print(F("TARGET "));
    Serial.println(clampTarget(requestedMM), 2);
  }
}

// ======================================================
// TEST FUNCTION: CURRENT POSITION TO NEW POSITION
// ======================================================

void performPositionTest(float requestedStartMM, float requestedTargetMM) {
  if (stepper == NULL) {
    Serial.println(F("TEST_REJECTED,stepper_not_initialized"));
    return;
  }

  if (fault) {
    Serial.println(F("TEST_REJECTED,fault_active"));
    return;
  }

  if (!homed) {
    Serial.println(F("TEST_REJECTED,not_homed"));
    return;
  }

  float startMM = clampTarget(requestedStartMM);
  float targetMM = clampTarget(requestedTargetMM);
  float commandedMoveMM = fabs(targetMM - startMM);

  if (commandedMoveMM <= 0.0) {
    Serial.println(F("TEST_REJECTED,start_and_target_are_same"));
    return;
  }

  Serial.print(F("TEST_SETUP_MOVING_TO_START,"));
  Serial.println(startMM, 3);

  // First move to the requested current/start position.
  // This is not the timed test movement.
  if (!moveToMMAndWait(startMM)) {
    Serial.println(F("TEST_REJECTED,could_not_reach_start_position"));
    return;
  }

  float internalStartMM = stepsToMM(getCurrentSteps());

  bool triangular = isTriangularMove(commandedMoveMM, moveSpeedMM_S, accelMM_S2);
  float calcTime = calculatedMoveTime(commandedMoveMM, moveSpeedMM_S, accelMM_S2);

  currentTargetMM = targetMM;
  commandedTargetSteps = mmToSteps(targetMM);

  applyMoveSettings();
  enableDriver(true);

  Serial.print(F("TEST_TIMED_MOVE_START,"));
  Serial.print(startMM, 3);
  Serial.print(F(","));
  Serial.println(targetMM, 3);

  unsigned long startMicros = micros();

  stepper->moveTo(commandedTargetSteps);

  while (stepper->isRunning()) {
    checkLimitsDuringMotion();

    if (fault) {
      return;
    }

    delay(1);
  }

  unsigned long endMicros = micros();

  float recordedTime = (endMicros - startMicros) / 1000000.0;
  float internalEndMM = stepsToMM(getCurrentSteps());

  float averageInternalSpeed = 0.0;

  if (recordedTime > 0.0) {
    averageInternalSpeed = commandedMoveMM / recordedTime;
  }

  Serial.print(F("RESULT,"));
  Serial.print(microstepping);
  Serial.print(F(","));
  Serial.print(startMM, 3);
  Serial.print(F(","));
  Serial.print(targetMM, 3);
  Serial.print(F(","));
  Serial.print(commandedMoveMM, 3);
  Serial.print(F(","));
  Serial.print(moveSpeedMM_S, 3);
  Serial.print(F(","));
  Serial.print(accelMM_S2, 3);
  Serial.print(F(","));
  Serial.print(triangular ? F("TRIANGULAR") : F("TRAPEZOIDAL"));
  Serial.print(F(","));
  Serial.print(calcTime, 4);
  Serial.print(F(","));
  Serial.print(recordedTime, 4);
  Serial.print(F(","));
  Serial.print(averageInternalSpeed, 3);
  Serial.print(F(","));
  Serial.print(internalStartMM, 3);
  Serial.print(F(","));
  Serial.println(internalEndMM, 3);
}

// ======================================================
// NEW REPEAT FUNCTION: NO DELIBERATE DELAY BETWEEN MOVES
// ======================================================

void repeatBackAndForth(int requestedCycles, float requestedLeftMM, float requestedRightMM) {
  if (stepper == NULL) {
    Serial.println(F("REPEAT_REJECTED,stepper_not_initialized"));
    return;
  }

  if (fault) {
    Serial.println(F("REPEAT_REJECTED,fault_active"));
    return;
  }

  if (!homed) {
    Serial.println(F("REPEAT_REJECTED,not_homed"));
    return;
  }

  if (requestedCycles <= 0) {
    Serial.println(F("REPEAT_REJECTED,cycles_must_be_positive"));
    return;
  }

  float leftMM = clampTarget(requestedLeftMM);
  float rightMM = clampTarget(requestedRightMM);

  if (leftMM == rightMM) {
    Serial.println(F("REPEAT_REJECTED,left_and_right_positions_are_same"));
    return;
  }

  float commandedMoveMM = fabs(rightMM - leftMM);
  bool triangular = isTriangularMove(commandedMoveMM, moveSpeedMM_S, accelMM_S2);
  float calcTime = calculatedMoveTime(commandedMoveMM, moveSpeedMM_S, accelMM_S2);

  applyMoveSettings();
  enableDriver(true);

  Serial.println(F("REPEAT_SETUP_MOVING_TO_LEFT_START"));

  // Move to the left/start position once before the repeatability test.
  currentTargetMM = leftMM;
  commandedTargetSteps = mmToSteps(leftMM);
  stepper->moveTo(commandedTargetSteps);

  if (!waitUntilMoveCompleteNoDelay()) {
    Serial.println(F("REPEAT_ABORTED,fault_during_start_move"));
    return;
  }

  Serial.println(F("REPEAT_START"));

  int completedCycles = 0;
  float totalOutwardTime = 0.0;
  float totalReturnTime = 0.0;

  for (int cycle = 1; cycle <= requestedCycles; cycle++) {
    // Move A: left -> right
    currentTargetMM = rightMM;
    commandedTargetSteps = mmToSteps(rightMM);

    unsigned long startA = micros();
    stepper->moveTo(commandedTargetSteps);

    if (!waitUntilMoveCompleteNoDelay()) {
      Serial.println(F("REPEAT_ABORTED,fault_during_outward_move"));
      return;
    }

    unsigned long endA = micros();

    // Move B: right -> left
    // This is commanded immediately after Move A completes.
    currentTargetMM = leftMM;
    commandedTargetSteps = mmToSteps(leftMM);

    unsigned long startB = micros();
    stepper->moveTo(commandedTargetSteps);

    if (!waitUntilMoveCompleteNoDelay()) {
      Serial.println(F("REPEAT_ABORTED,fault_during_return_move"));
      return;
    }

    unsigned long endB = micros();

    totalOutwardTime += (endA - startA) / 1000000.0;
    totalReturnTime += (endB - startB) / 1000000.0;
    completedCycles++;
  }

  float averageOutwardTime = 0.0;
  float averageReturnTime = 0.0;
  float averageOutwardSpeed = 0.0;
  float averageReturnSpeed = 0.0;

  if (completedCycles > 0) {
    averageOutwardTime = totalOutwardTime / completedCycles;
    averageReturnTime = totalReturnTime / completedCycles;

    if (averageOutwardTime > 0.0) {
      averageOutwardSpeed = commandedMoveMM / averageOutwardTime;
    }

    if (averageReturnTime > 0.0) {
      averageReturnSpeed = commandedMoveMM / averageReturnTime;
    }
  }

  float totalRecordedMoveTime = totalOutwardTime + totalReturnTime;
  float finalInternalMM = stepsToMM(getCurrentSteps());

  Serial.print(F("REPEAT_RESULT,"));
  Serial.print(requestedCycles);
  Serial.print(F(","));
  Serial.print(completedCycles);
  Serial.print(F(","));
  Serial.print(leftMM, 3);
  Serial.print(F(","));
  Serial.print(rightMM, 3);
  Serial.print(F(","));
  Serial.print(commandedMoveMM, 3);
  Serial.print(F(","));
  Serial.print(moveSpeedMM_S, 3);
  Serial.print(F(","));
  Serial.print(accelMM_S2, 3);
  Serial.print(F(","));
  Serial.print(triangular ? F("TRIANGULAR") : F("TRAPEZOIDAL"));
  Serial.print(F(","));
  Serial.print(calcTime, 4);
  Serial.print(F(","));
  Serial.print(totalRecordedMoveTime, 4);
  Serial.print(F(","));
  Serial.print(averageOutwardTime, 4);
  Serial.print(F(","));
  Serial.print(averageReturnTime, 4);
  Serial.print(F(","));
  Serial.print(averageOutwardSpeed, 3);
  Serial.print(F(","));
  Serial.print(averageReturnSpeed, 3);
  Serial.print(F(","));
  Serial.println(finalInternalMM, 3);
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

  // Move to the minimum usable target position.
  currentTargetMM = MIN_TARGET_MM;

  if (!moveRelativeAndMonitor(mmToSteps(MIN_TARGET_MM))) {
    return;
  }

  commandedTargetSteps = mmToSteps(MIN_TARGET_MM);
  currentTargetMM = MIN_TARGET_MM;

  Serial.println(F("Homing complete."));
  printPosition();
}

// ======================================================
// RUNTIME CONFIGURATION
// ======================================================

void setRuntimeConfig(int newMicrostepping, float newSpeedMM_S, float newAccelMM_S2) {
  if (newMicrostepping <= 0 || newSpeedMM_S <= 0.0 || newAccelMM_S2 <= 0.0) {
    Serial.println(F("CONFIG_REJECTED,values_must_be_positive"));
    return;
  }

  microstepping = newMicrostepping;
  moveSpeedMM_S = newSpeedMM_S;
  accelMM_S2 = newAccelMM_S2;

  updateMotionConversions();
  applyMoveSettings();

  Serial.print(F("CONFIG_OK,"));
  Serial.print(microstepping);
  Serial.print(F(","));
  Serial.print(moveSpeedMM_S, 3);
  Serial.print(F(","));
  Serial.print(accelMM_S2, 3);
  Serial.print(F(",steps_per_mm,"));
  Serial.println(stepsPerMM, 4);
}

// ======================================================
// COMMAND PROCESSING
// ======================================================

void processCommand(char *cmd) {
  if (cmd[0] == '\0') return;

  char upperCmd[96];
  strncpy(upperCmd, cmd, sizeof(upperCmd));
  upperCmd[sizeof(upperCmd) - 1] = '\0';
  uppercase(upperCmd);

  if (strcmp(upperCmd, "STOP") == 0 || strcmp(upperCmd, "X") == 0) {
    emergencyStop();
    return;
  }

  if (strcmp(upperCmd, "CLEAR") == 0) {
    clearFault();
    return;
  }

  if (strcmp(upperCmd, "HOME") == 0) {
    homeActuator();
    return;
  }

  if (strcmp(upperCmd, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcmp(upperCmd, "POS") == 0) {
    printPosition();
    return;
  }

  if (strcmp(upperCmd, "HEADER") == 0) {
    printCSVHeader();
    return;
  }

  // CONFIG microstepping speed acceleration
  // Example: CONFIG 8 1000 9000
  if (strncmp(upperCmd, "CONFIG", 6) == 0) {
    char temp[96];
    strncpy(temp, upperCmd, sizeof(temp));
    temp[sizeof(temp) - 1] = '\0';

    strtok(temp, " "); // CONFIG
    char *mTok = strtok(NULL, " ");
    char *sTok = strtok(NULL, " ");
    char *aTok = strtok(NULL, " ");

    if (mTok == NULL || sTok == NULL || aTok == NULL) {
      Serial.println(F("CONFIG_REJECTED,use_CONFIG_microstepping_speed_accel"));
      return;
    }

    int newMicrostepping = atoi(mTok);
    float newSpeed = atof(sTok);
    float newAccel = atof(aTok);

    setRuntimeConfig(newMicrostepping, newSpeed, newAccel);
    return;
  }

  // TEST current_position new_commanded_position
  // Example: TEST 5 165
  if (strncmp(upperCmd, "TEST", 4) == 0) {
    char temp[96];
    strncpy(temp, upperCmd, sizeof(temp));
    temp[sizeof(temp) - 1] = '\0';

    strtok(temp, " "); // TEST
    char *startTok = strtok(NULL, " ");
    char *targetTok = strtok(NULL, " ");

    if (startTok == NULL || targetTok == NULL) {
      Serial.println(F("TEST_REJECTED,use_TEST_start_mm_target_mm"));
      return;
    }

    float startMM = atof(startTok);
    float targetMM = atof(targetTok);

    performPositionTest(startMM, targetMM);
    return;
  }

  // REPEAT cycles left_position right_position
  // Example: REPEAT 10 5 165
  if (strncmp(upperCmd, "REPEAT", 6) == 0) {
    char temp[96];
    strncpy(temp, upperCmd, sizeof(temp));
    temp[sizeof(temp) - 1] = '\0';

    strtok(temp, " "); // REPEAT
    char *cyclesTok = strtok(NULL, " ");
    char *leftTok = strtok(NULL, " ");
    char *rightTok = strtok(NULL, " ");

    if (cyclesTok == NULL || leftTok == NULL || rightTok == NULL) {
      Serial.println(F("REPEAT_REJECTED,use_REPEAT_cycles_leftmm_rightmm"));
      return;
    }

    int cycles = atoi(cyclesTok);
    float leftMM = atof(leftTok);
    float rightMM = atof(rightTok);

    repeatBackAndForth(cycles, leftMM, rightMM);
    return;
  }

  // Normal command: single float number = absolute target position
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

  updateMotionConversions();

  startupLedFlicker();

  enableDriver(false);

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
  Serial.println(F("Waiting before automatic homing..."));

  delay(STARTUP_WAIT_MS);

  printLimits();

  homeActuator();

  Serial.println(F("Ready."));
  Serial.println(F("Commands: CONFIG m speed accel, TEST start target, REPEAT cycles left right, STOP, X, CLEAR, HOME, STATUS, POS, HEADER"));
  printCSVHeader();
}

// ======================================================
// MAIN LOOP
// ======================================================

void loop() {
  char cmd[96];

  if (readLine(cmd, sizeof(cmd))) {
    processCommand(cmd);
  }

  if (!fault && homed) {
    checkLimitsDuringMotion();
  }

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
