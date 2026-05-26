#include <AccelStepper.h>

// ---------------- PINS ----------------
// Use these based on your corrected limit-switch setup.
// If left/right limits are reversed again, swap 7 and 8.
const int EN_PIN      = 2;
const int DIR_PIN     = 3;
const int STEP_PIN    = 4;
const int RIGHT_LIMIT = 8;
const int LEFT_LIMIT  = 7;

// ---------------- DRIVER ----------------
const bool DRIVER_ENABLE_ACTIVE_LOW = true;

// ---------------- MOTOR ----------------
const int FULL_STEPS_PER_REV = 200;
const int MICROSTEPPING = 2;
const int STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPPING; // 1600 pulses/rev

// ---------------- BELT DRIVE CALIBRATION ----------------
// Measured result: 1 motor revolution moved the carriage 70 mm.
const float TRAVEL_MM_PER_REV = 70.0;
const float STEPS_PER_MM = STEPS_PER_REV / TRAVEL_MM_PER_REV; // 22.857 steps/mm

// ---------------- TRAVEL LIMITS ----------------
// Foosball table usable range.
const float MIN_TRAVEL_MM = 5.0;
const float MAX_TRAVEL_MM = 165.0;
const float HOME_SEARCH_MAX_MM = 230.0;

// ---------------- SPEEDS ----------------
// 8x microstepping:
// 22857 steps/s / 22.857 steps/mm = approximately 1000 mm/s.
const float TEST_SPEED = 2000.0;
const float MOVE_SPEED = 4000;                     //22857.0;
const float HOME_SPEED = 500.0;
const float ACCEL = 50000.0;  // 22.857

// JOGLEFT / JOGRIGHT now use calibrated mm movement
const float JOG_MM = 1.0;

// REVLEFT / REVRIGHT test amount
const float TEST_REVOLUTIONS = 1.0;

const float HOME_BACKOFF_MM = 5.0;

// ---------------- SPEED TEST SETTINGS ----------------
// Updated to match the foosball usable range.
const float SPEEDTEST_POS_A_MM = MIN_TRAVEL_MM;
const float SPEEDTEST_POS_B_MM = MAX_TRAVEL_MM;
const int SPEEDTEST_REPEATS = 6;

// ---------------- STEPPER ----------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ---------------- STATE ----------------
bool homed = false;
bool fault = false;
bool driverEnabled = false;

enum FaultCode {
  NO_FAULT,
  LEFT_LIMIT_FAULT,
  RIGHT_LIMIT_FAULT,
  HOME_NOT_FOUND,
  STOP_FAULT,
  HOME_BACKOFF_FAULT,
  UNKNOWN_FAULT
};

FaultCode faultCode = NO_FAULT;

// ---------------- BASIC FUNCTIONS ----------------

void enableDriver(bool enable) {
  driverEnabled = enable;

  if (DRIVER_ENABLE_ACTIVE_LOW) {
    digitalWrite(EN_PIN, enable ? LOW : HIGH);
  } else {
    digitalWrite(EN_PIN, enable ? HIGH : LOW);
  }
}

bool leftLimitTriggered() {
  // NC switch with INPUT_PULLUP:
  // LOW = normal/closed
  // HIGH = triggered/open/wire fault
  return digitalRead(LEFT_LIMIT) == HIGH;
}

bool rightLimitTriggered() {
  return digitalRead(RIGHT_LIMIT) == HIGH;
}

long mmToSteps(float mm) {
  if (mm >= 0) return (long)(mm * STEPS_PER_MM + 0.5);
  else return (long)(mm * STEPS_PER_MM - 0.5);
}

float stepsToMM(long steps) {
  return steps / STEPS_PER_MM;
}

void printFault() {
  Serial.print(F("Fault: "));

  switch (faultCode) {
    case NO_FAULT:
      Serial.println(F("none"));
      break;
    case LEFT_LIMIT_FAULT:
      Serial.println(F("left limit triggered"));
      break;
    case RIGHT_LIMIT_FAULT:
      Serial.println(F("right limit triggered"));
      break;
    case HOME_NOT_FOUND:
      Serial.println(F("home switch not found"));
      break;
    case STOP_FAULT:
      Serial.println(F("STOP command received"));
      break;
    case HOME_BACKOFF_FAULT:
      Serial.println(F("home backoff failed"));
      break;
    default:
      Serial.println(F("unknown"));
      break;
  }
}

void enterFault(FaultCode code) {
  fault = true;
  faultCode = code;
  stepper.stop();
  enableDriver(false);

  Serial.println(F("FAULT. Driver disabled."));
  printFault();
  Serial.println(F("Fix issue, then send CLEAR."));
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
  Serial.print(F("Left: "));
  Serial.print(leftLimitTriggered() ? F("TRIGGERED/OPEN") : F("NORMAL/CLOSED"));

  Serial.print(F(" | Right: "));
  Serial.println(rightLimitTriggered() ? F("TRIGGERED/OPEN") : F("NORMAL/CLOSED"));
}

void printPosition() {
  Serial.print(F("Position: "));
  Serial.print(stepsToMM(stepper.currentPosition()), 2);
  Serial.print(F(" mm | Steps: "));
  Serial.println(stepper.currentPosition());
}

void printStatus() {
  Serial.println(F("----- STATUS -----"));

  Serial.print(F("Driver enabled: "));
  Serial.println(driverEnabled ? F("YES") : F("NO"));

  Serial.print(F("Homed: "));
  Serial.println(homed ? F("YES") : F("NO"));

  Serial.print(F("Fault active: "));
  Serial.println(fault ? F("YES") : F("NO"));

  printFault();
  printPosition();
  printLimits();

  Serial.print(F("Min travel: "));
  Serial.print(MIN_TRAVEL_MM, 1);
  Serial.println(F(" mm"));

  Serial.print(F("Max travel: "));
  Serial.print(MAX_TRAVEL_MM, 1);
  Serial.println(F(" mm"));

  Serial.print(F("Microstepping: "));
  Serial.println(MICROSTEPPING);

  Serial.print(F("Steps/rev: "));
  Serial.println(STEPS_PER_REV);

  Serial.print(F("Measured travel mm/rev: "));
  Serial.println(TRAVEL_MM_PER_REV, 3);

  Serial.print(F("Steps/mm: "));
  Serial.println(STEPS_PER_MM, 3);

  Serial.print(F("Test speed: "));
  Serial.print(TEST_SPEED, 1);
  Serial.println(F(" steps/s"));

  Serial.print(F("Move speed: "));
  Serial.print(MOVE_SPEED, 1);
  Serial.println(F(" steps/s"));

  Serial.print(F("Theoretical max move speed: "));
  Serial.print(MOVE_SPEED / STEPS_PER_MM, 1);
  Serial.println(F(" mm/s"));

  Serial.print(F("Acceleration: "));
  Serial.print(ACCEL, 1);
  Serial.println(F(" steps/s^2"));

  Serial.print(F("JOG distance mm: "));
  Serial.println(JOG_MM, 3);

  Serial.println(F("------------------"));
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("HELP"));
  Serial.println(F("LIMITS"));
  Serial.println(F("ENABLE"));
  Serial.println(F("DISABLE"));
  Serial.println(F("JOGLEFT"));
  Serial.println(F("JOGRIGHT"));
  Serial.println(F("REVLEFT"));
  Serial.println(F("REVRIGHT"));
  Serial.println(F("HOME"));
  Serial.println(F("MOVE <mm>"));
  Serial.println(F("SPEEDTEST"));
  Serial.println(F("POS"));
  Serial.println(F("STATUS"));
  Serial.println(F("STOP"));
  Serial.println(F("CLEAR"));
}

// ---------------- SERIAL COMMAND READING ----------------

bool readCommand(char *buffer, int bufferSize) {
  if (Serial.available() == 0) return false;

  int len = Serial.readBytesUntil('\n', buffer, bufferSize - 1);
  buffer[len] = '\0';

  for (int i = 0; i < len; i++) {
    if (buffer[i] == '\r') buffer[i] = '\0';
  }

  for (int i = 0; buffer[i] != '\0'; i++) {
    if (buffer[i] >= 'a' && buffer[i] <= 'z') {
      buffer[i] = buffer[i] - 32;
    }
  }

  return true;
}

bool checkStopDuringMotion() {
  char cmd[20];

  if (readCommand(cmd, sizeof(cmd))) {
    if (strcmp(cmd, "STOP") == 0) {
      emergencyStop();
      return true;
    } else {
      Serial.println(F("Busy. Only STOP works during motion."));
    }
  }

  return false;
}

// ---------------- SAFETY DURING MOTION ----------------

bool checkLimitsDuringMotion() {
  long distanceToGo = stepper.distanceToGo();

  if (distanceToGo < 0 && leftLimitTriggered()) {
    enterFault(LEFT_LIMIT_FAULT);
    return false;
  }

  if (distanceToGo > 0 && rightLimitTriggered()) {
    enterFault(RIGHT_LIMIT_FAULT);
    return false;
  }

  return true;
}

// ---------------- MOVEMENT ----------------

bool moveToMM(float targetMM) {
  if (fault) {
    Serial.println(F("Cannot move: fault active."));
    return false;
  }

  if (!homed) {
    Serial.println(F("Cannot move: send HOME first."));
    return false;
  }

  if (targetMM < MIN_TRAVEL_MM || targetMM > MAX_TRAVEL_MM) {
    Serial.println(F("Target outside soft limits."));
    Serial.print(F("Allowed range: "));
    Serial.print(MIN_TRAVEL_MM, 1);
    Serial.print(F(" to "));
    Serial.print(MAX_TRAVEL_MM, 1);
    Serial.println(F(" mm"));
    return false;
  }

  stepper.setMaxSpeed(MOVE_SPEED);
  stepper.setAcceleration(ACCEL);

  enableDriver(true);

  long startSteps = stepper.currentPosition();
  long targetSteps = mmToSteps(targetMM);
  float startMM = stepsToMM(startSteps);
  float moveDistanceMM = abs(targetMM - startMM);

  stepper.moveTo(targetSteps);

  Serial.print(F("Moving to "));
  Serial.print(targetMM);
  Serial.println(F(" mm"));

  unsigned long startTime_us = micros();

  while (stepper.distanceToGo() != 0) {
    if (checkStopDuringMotion()) return false;
    if (!checkLimitsDuringMotion()) return false;
    stepper.run();
  }

  unsigned long endTime_us = micros();
  unsigned long elapsed_us = endTime_us - startTime_us;

  float elapsed_s = elapsed_us / 1000000.0;
  float averageSpeed_mm_s = moveDistanceMM / elapsed_s;
  float averageStepRate_steps_s = averageSpeed_mm_s * STEPS_PER_MM;

  Serial.println(F("Move complete."));
  printPosition();

  Serial.print(F("Move distance: "));
  Serial.print(moveDistanceMM, 2);
  Serial.println(F(" mm"));

  Serial.print(F("Move time: "));
  Serial.print(elapsed_s, 4);
  Serial.println(F(" s"));

  Serial.print(F("Average speed: "));
  Serial.print(averageSpeed_mm_s, 2);
  Serial.println(F(" mm/s"));

  Serial.print(F("Average step rate: "));
  Serial.print(averageStepRate_steps_s, 0);
  Serial.println(F(" steps/s"));

  return true;
}

void jogMM(float distanceMM) {
  if (fault) {
    Serial.println(F("Cannot jog: fault active."));
    return;
  }

  if (!homed) {
    Serial.println(F("Direction/mm test only. Keep away from end stops."));
  }

  if (distanceMM < 0 && leftLimitTriggered()) {
    enterFault(LEFT_LIMIT_FAULT);
    return;
  }

  if (distanceMM > 0 && rightLimitTriggered()) {
    enterFault(RIGHT_LIMIT_FAULT);
    return;
  }

  stepper.setMaxSpeed(TEST_SPEED);
  stepper.setAcceleration(ACCEL);

  enableDriver(true);
  stepper.move(mmToSteps(distanceMM));

  Serial.print(F("Jogging "));
  Serial.print(distanceMM);
  Serial.println(F(" mm"));

  while (stepper.distanceToGo() != 0) {
    if (checkStopDuringMotion()) return;
    if (!checkLimitsDuringMotion()) return;
    stepper.run();
  }

  Serial.println(F("Jog complete."));
  printPosition();
}

void jogRevolutions(float revolutions) {
  if (fault) {
    Serial.println(F("Cannot run revolution test: fault active."));
    return;
  }

  if (revolutions < 0 && leftLimitTriggered()) {
    enterFault(LEFT_LIMIT_FAULT);
    return;
  }

  if (revolutions > 0 && rightLimitTriggered()) {
    enterFault(RIGHT_LIMIT_FAULT);
    return;
  }

  long moveSteps = (long)(revolutions * STEPS_PER_REV);

  stepper.setMaxSpeed(TEST_SPEED);
  stepper.setAcceleration(ACCEL);

  enableDriver(true);
  stepper.move(moveSteps);

  Serial.print(F("Moving motor revolutions: "));
  Serial.println(revolutions, 2);

  Serial.print(F("Step pulses commanded: "));
  Serial.println(moveSteps);

  while (stepper.distanceToGo() != 0) {
    if (checkStopDuringMotion()) return;
    if (!checkLimitsDuringMotion()) return;
    stepper.run();
  }

  Serial.println(F("Revolution test complete."));
  printPosition();
}

// ---------------- QUANTITATIVE SPEED TEST ----------------

void speedTest(float posA, float posB, int repeats) {
  if (fault) {
    Serial.println(F("Cannot run speed test: fault active."));
    return;
  }

  if (!homed) {
    Serial.println(F("Cannot run speed test: send HOME first."));
    return;
  }

  if (posA < MIN_TRAVEL_MM || posA > MAX_TRAVEL_MM || posB < MIN_TRAVEL_MM || posB > MAX_TRAVEL_MM) {
    Serial.println(F("Speed test positions outside soft limits."));
    return;
  }

  stepper.setMaxSpeed(MOVE_SPEED);
  stepper.setAcceleration(ACCEL);

  enableDriver(true);

  Serial.println(F("----- SPEED TEST START -----"));

  Serial.print(F("Travel range: "));
  Serial.print(posA, 1);
  Serial.print(F(" to "));
  Serial.print(posB, 1);
  Serial.println(F(" mm"));

  Serial.print(F("Microstepping: "));
  Serial.println(MICROSTEPPING);

  Serial.print(F("Steps/mm: "));
  Serial.println(STEPS_PER_MM, 3);

  Serial.print(F("Commanded MOVE_SPEED: "));
  Serial.print(MOVE_SPEED, 0);
  Serial.println(F(" steps/s"));

  Serial.print(F("Theoretical max speed: "));
  Serial.print(MOVE_SPEED / STEPS_PER_MM, 1);
  Serial.println(F(" mm/s"));

  Serial.print(F("Acceleration: "));
  Serial.print(ACCEL, 0);
  Serial.println(F(" steps/s^2"));

  Serial.println(F("----------------------------"));

  for (int i = 0; i < repeats; i++) {
    float startMM = stepsToMM(stepper.currentPosition());
    float targetMM = (i % 2 == 0) ? posB : posA;

    float distanceMM = abs(targetMM - startMM);
    long targetSteps = mmToSteps(targetMM);

    stepper.moveTo(targetSteps);

    Serial.print(F("Test "));
    Serial.print(i + 1);
    Serial.print(F(": moving from "));
    Serial.print(startMM, 2);
    Serial.print(F(" mm to "));
    Serial.print(targetMM, 2);
    Serial.println(F(" mm"));

    unsigned long startTime_us = micros();

    while (stepper.distanceToGo() != 0) {
      if (checkStopDuringMotion()) return;
      if (!checkLimitsDuringMotion()) return;
      stepper.run();
    }

    unsigned long endTime_us = micros();
    unsigned long elapsed_us = endTime_us - startTime_us;

    float elapsed_s = elapsed_us / 1000000.0;
    float actualSpeed_mm_s = distanceMM / elapsed_s;
    float actualStepRate_steps_s = actualSpeed_mm_s * STEPS_PER_MM;

    Serial.print(F("Distance: "));
    Serial.print(distanceMM, 2);
    Serial.println(F(" mm"));

    Serial.print(F("Time: "));
    Serial.print(elapsed_s, 4);
    Serial.println(F(" s"));

    Serial.print(F("Average speed: "));
    Serial.print(actualSpeed_mm_s, 1);
    Serial.println(F(" mm/s"));

    Serial.print(F("Average step rate: "));
    Serial.print(actualStepRate_steps_s, 0);
    Serial.println(F(" steps/s"));

    Serial.println(F("----------------------------"));
  }

  Serial.println(F("----- SPEED TEST COMPLETE -----"));
  printPosition();
}

// ---------------- HOMING ----------------

void homeActuator() {
  if (fault) {
    Serial.println(F("Cannot home: fault active. Send CLEAR."));
    return;
  }

  Serial.println(F("Homing left..."));
  enableDriver(true);

  stepper.setMaxSpeed(HOME_SPEED);
  stepper.setAcceleration(ACCEL);

  if (leftLimitTriggered()) {
    Serial.println(F("Left limit active. Backing off."));

    stepper.move(mmToSteps(HOME_BACKOFF_MM));

    while (stepper.distanceToGo() != 0) {
      if (checkStopDuringMotion()) return;

      if (rightLimitTriggered()) {
        enterFault(RIGHT_LIMIT_FAULT);
        return;
      }

      stepper.run();
    }

    delay(200);

    if (leftLimitTriggered()) {
      enterFault(HOME_BACKOFF_FAULT);
      return;
    }
  }

  long startSteps = stepper.currentPosition();
  long maxSearchSteps = mmToSteps(HOME_SEARCH_MAX_MM);

  stepper.setSpeed(-HOME_SPEED);

  while (!leftLimitTriggered()) {
    if (checkStopDuringMotion()) return;

    if (rightLimitTriggered()) {
      enterFault(RIGHT_LIMIT_FAULT);
      return;
    }

    if (labs(stepper.currentPosition() - startSteps) > maxSearchSteps) {
      enterFault(HOME_NOT_FOUND);
      return;
    }

    stepper.runSpeed();
  }

  Serial.println(F("Home switch found."));
  stepper.stop();
  delay(200);

  stepper.setCurrentPosition(0);

  Serial.println(F("Backing off home."));

  stepper.setMaxSpeed(HOME_SPEED);
  stepper.move(mmToSteps(HOME_BACKOFF_MM));

  while (stepper.distanceToGo() != 0) {
    if (checkStopDuringMotion()) return;
    stepper.run();
  }

  if (leftLimitTriggered()) {
    enterFault(HOME_BACKOFF_FAULT);
    return;
  }

  // Important:
  // After backing off the physical switch, this sets that backed-off position as 0 mm.
  // Normal commanded movement is then limited from 0 mm to 170 mm.
  stepper.setCurrentPosition(0);
  homed = true;

  Serial.println(F("Homing complete."));
  printPosition();
}

// ---------------- COMMAND PROCESSING ----------------

void processCommand(char *cmd) {
  if (strcmp(cmd, "HELP") == 0) {
    printHelp();
  }

  else if (strcmp(cmd, "LIMITS") == 0) {
    printLimits();
  }

  else if (strcmp(cmd, "ENABLE") == 0) {
    if (!fault) {
      enableDriver(true);
      Serial.println(F("Driver enabled."));
    } else {
      Serial.println(F("Fault active. Send CLEAR first."));
    }
  }

  else if (strcmp(cmd, "DISABLE") == 0) {
    enableDriver(false);
    Serial.println(F("Driver disabled."));
  }

  else if (strcmp(cmd, "JOGLEFT") == 0) {
    jogMM(-JOG_MM);
  }

  else if (strcmp(cmd, "JOGRIGHT") == 0) {
    jogMM(JOG_MM);
  }

  else if (strcmp(cmd, "REVLEFT") == 0) {
    jogRevolutions(-TEST_REVOLUTIONS);
  }

  else if (strcmp(cmd, "REVRIGHT") == 0) {
    jogRevolutions(TEST_REVOLUTIONS);
  }

  else if (strcmp(cmd, "HOME") == 0) {
    homeActuator();
  }

  else if (strncmp(cmd, "MOVE ", 5) == 0) {
    float target = atof(cmd + 5);
    moveToMM(target);
  }

  else if (strcmp(cmd, "SPEEDTEST") == 0) {
    speedTest(SPEEDTEST_POS_A_MM, SPEEDTEST_POS_B_MM, SPEEDTEST_REPEATS);
  }

  else if (strcmp(cmd, "POS") == 0) {
    printPosition();
  }

  else if (strcmp(cmd, "STATUS") == 0) {
    printStatus();
  }

  else if (strcmp(cmd, "STOP") == 0) {
    emergencyStop();
  }

  else if (strcmp(cmd, "CLEAR") == 0) {
    clearFault();
  }

  else {
    Serial.println(F("Unknown command. Send HELP."));
  }
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(2);

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  pinMode(LEFT_LIMIT, INPUT_PULLUP);
  pinMode(RIGHT_LIMIT, INPUT_PULLUP);

  enableDriver(false);

  stepper.setMinPulseWidth(5);
  stepper.setMaxSpeed(TEST_SPEED);
  stepper.setAcceleration(ACCEL);

  // Direction correction:
  // If REVLEFT moves the wrong direction, change true to false.
  stepper.setPinsInverted(true, false, false);

  Serial.println(F("Actuator test code started."));
  Serial.println(F("Driver disabled at startup."));
  Serial.println(F("Calibration: 1 motor rev = 70 mm."));
  Serial.println(F("Foosball travel range: 0 to 170 mm."));
  Serial.println(F("Updated speed-test version."));
  Serial.println(F("Send HELP."));
  printLimits();
}

// ---------------- LOOP ----------------

void loop() {
  char cmd[32];

  if (readCommand(cmd, sizeof(cmd))) {
    processCommand(cmd);
  }
}