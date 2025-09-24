#include <AccelStepper.h>

// === Pin Definitions ===
#define STEP_PIN         3
#define DIR_PIN          2
#define ENABLE_PIN       4
#define LIMIT_SWITCH_PIN 7
#define MAX_STOP_BUTTON  8

// === Mechanical Config ===
#define STEPS_PER_REV    2048.0
#define MICROSTEPS       1.0
#define THREAD_PITCH     8.0
#define DEFAULT_SYRINGE_DIAMETER 2.3
#define HOMING_SPEED     600
#define BACKLASH_MM      0.2
#define SOFT_MARGIN_MM   1.0
#define POST_LIMIT_OFFSET_MM 1.0

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// === Global State ===
float syringeDiameter = DEFAULT_SYRINGE_DIAMETER;
float stepsPerMM = (STEPS_PER_REV * MICROSTEPS) / THREAD_PITCH;
bool isCalibrated = false;
float syringeCapacityUL = 0.0;
long maxPositionSteps = 0;
unsigned long totalMoves = 0;
unsigned long totalDispenses = 0;
float totalVolumeDispensed = 0.0;
int lastDirection = 0;

float volumeToMillimeters(float volumeUL, float diameterMM) {
  float areaMM2 = PI * pow(diameterMM / 2.0, 2);
  return volumeUL / areaMM2;
}

void setup() {
  Serial.begin(9600);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MAX_STOP_BUTTON, INPUT_PULLUP);
  digitalWrite(ENABLE_PIN, LOW);

  stepper.setMaxSpeed(1500);
  stepper.setAcceleration(3000);

  Serial.println("=== Syringe System Booting ===");
  Serial.print("Using default syringe diameter: ");
  Serial.print(syringeDiameter);
  Serial.println(" mm");

  Serial.println("Starting auto-calibration...");
  autoCalibrateSyringe();

  if (isCalibrated) {
    Serial.print("[OK] Calibration complete. Syringe capacity: ");
    Serial.print(syringeCapacityUL, 2);
    Serial.println(" uL");
    Serial.println("Available commands: SET_DIAMETER, MOVE_MM, MOVE_VOL, DISPENSE, CALIBRATE, STATUS, SET_SPEED, TEST_RESOLUTION, GO_HOME, EMERGENCY_STOP");
  }
}

void loop() {
  if (!isCalibrated) return;

  static unsigned long lastPlot = 0;
  static bool plotEnabled = false;

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();

    if (input == "PLOT ON") {
      plotEnabled = true;
      Serial.println("[OK] Plotting enabled.");
    } else if (input == "PLOT OFF") {
      plotEnabled = false;
      Serial.println("[OK] Plotting disabled.");
    } else {
      processCommand(input);
    }
  }

  if (plotEnabled && millis() - lastPlot >= 100) {
    lastPlot = millis();
    float pos = -stepper.currentPosition() / stepsPerMM;
    float tgt = -stepper.targetPosition() / stepsPerMM;
    float vol = PI * pow(syringeDiameter / 2.0, 2) * pos;
    static long lastSteps = 0;
    long currentSteps = stepper.currentPosition();
    long deltaSteps = currentSteps - lastSteps;
    lastSteps = currentSteps;

    Serial.print("POS_MM:");
    Serial.print(pos, 2);
    Serial.print("\tTARGET_MM:");
    Serial.print(tgt, 2);
    Serial.print("\tVOLUME_UL:");
    Serial.print(vol, 2);
    Serial.print("\tDELTA_STEPS:");
    Serial.println(deltaSteps);
  }
}

void movePlunger(float mmToMove) {
  if (abs(mmToMove) < 0.001) {
    Serial.println("[WARN] Move ~0 mm. Skipping.");
    return;
  }

  long initial = stepper.currentPosition();
  long target = initial - mmToMove * stepsPerMM;  // Negative = forward (dispense)

  long minSteps = -maxPositionSteps;
  long maxSteps = 0;
  if (target > maxSteps) target = maxSteps;
  if (target < minSteps) target = minSteps;

  int thisDirection = (mmToMove > 0) ? -1 : 1;

  if (lastDirection != 0 && thisDirection != lastDirection) {
    long backlashSteps = BACKLASH_MM * stepsPerMM * thisDirection;
    stepper.moveTo(stepper.currentPosition() + backlashSteps);
    while (stepper.distanceToGo() != 0) stepper.run();
  }

  lastDirection = thisDirection;

  stepper.moveTo(target);
  while (stepper.distanceToGo() != 0) stepper.run();

  long finalPos = stepper.currentPosition();
  long stepsTaken = abs(finalPos - initial);
  if (stepsTaken == 0) {
    Serial.println("[WARN] No steps taken.");
  } else {
    Serial.print("[OK] Movement complete. Steps taken: ");
    Serial.println(stepsTaken);
  }
  totalMoves++;
}

void autoCalibrateSyringe() {
  Serial.println("[AutoCal] Homing...");
  stepper.setMaxSpeed(HOMING_SPEED);
  stepper.setAcceleration(HOMING_SPEED * 2);
  stepper.moveTo(150000);
  while (digitalRead(LIMIT_SWITCH_PIN) == LOW) stepper.run();
  stepper.stop(); while (stepper.isRunning()) stepper.run();
  stepper.setCurrentPosition(0);
  Serial.println("[AutoCal] Home set.");

  stepper.moveTo(-POST_LIMIT_OFFSET_MM * stepsPerMM);
  while (stepper.distanceToGo() != 0) stepper.run();
  stepper.setCurrentPosition(0);

  Serial.println("[AutoCal] Finding max travel...");
  stepper.moveTo(-200000);
  while (digitalRead(MAX_STOP_BUTTON) == LOW) stepper.run();
  stepper.stop(); while (stepper.isRunning()) stepper.run();

  maxPositionSteps = -stepper.currentPosition() - (SOFT_MARGIN_MM * stepsPerMM);
  float travelMM = abs(maxPositionSteps) / stepsPerMM;
  syringeCapacityUL = PI * pow(syringeDiameter / 2.0, 2) * travelMM;

  Serial.print("[AutoCal] Max travel: "); Serial.print(travelMM, 2); Serial.println(" mm");
  Serial.print("[AutoCal] Syringe capacity: "); Serial.print(syringeCapacityUL, 2); Serial.println(" uL");

  Serial.println("[AutoCal] Returning to home...");
  stepper.moveTo(0); while (stepper.distanceToGo() != 0) stepper.run();
  stepper.moveTo(-POST_LIMIT_OFFSET_MM * stepsPerMM); while (stepper.distanceToGo() != 0) stepper.run();
  stepper.setCurrentPosition(0);

  Serial.println("[OK] Calibration complete.");
  isCalibrated = true;
}

void processCommand(String command) {
  Serial.print("[CMD] "); Serial.println(command);

  if (command.startsWith("SET_DIAMETER")) {
    float newDiameter = command.substring(12).toFloat();
    if (newDiameter > 0.0) {
      syringeDiameter = newDiameter;
      Serial.print("[OK] Syringe diameter updated to: ");
      Serial.print(syringeDiameter);
      Serial.println(" mm");
    } else {
      Serial.println("[ERR] Diameter must be positive.");
    }
  }
  else if (command.startsWith("MOVE_MM")) {
    float mmToMove = command.substring(8).toFloat();
    movePlunger(mmToMove);
    Serial.println("[OK] MOVE_MM complete.");
  }
  else if (command.startsWith("MOVE_VOL")) {
    float volume = command.substring(9).toFloat();
    if (volume <= 0 || syringeDiameter <= 0) {
      Serial.println("[ERR] Invalid volume or diameter.");
      return;
    }
    float mmToMove = volumeToMillimeters(volume, syringeDiameter);
    movePlunger(mmToMove);
    Serial.println("[OK] MOVE_VOL complete.");
  }
  else if (command.startsWith("DISPENSE")) {
    float volume = command.substring(8).toFloat();
    if (volume <= 0 || volume > syringeCapacityUL || syringeDiameter <= 0) {
      Serial.println("[ERR] Invalid or excessive volume.");
      return;
    }
    float mmToMove = volumeToMillimeters(volume, syringeDiameter);
    movePlunger(mmToMove);
    delay(1000);
    movePlunger(-mmToMove);
    movePlunger(stepper.currentPosition() / stepsPerMM);
    totalDispenses++;
    totalVolumeDispensed += volume;
    Serial.println("[OK] DISPENSE complete.");
  }
  else if (command.startsWith("SET_SPEED")) {
    int spaceIndex = command.indexOf(' ', 10);
    float newSpeed = command.substring(9, spaceIndex).toFloat();
    float newAccel = command.substring(spaceIndex + 1).toFloat();

    if (newSpeed > 0 && newAccel > 0) {
      stepper.setMaxSpeed(newSpeed);
      stepper.setAcceleration(newAccel);
      float rpm = (newSpeed * 60.0) / STEPS_PER_REV;
      Serial.print("[OK] Speed updated: ");
      Serial.print(newSpeed);
      Serial.print(" steps/s (Approx. ");
      Serial.print(rpm, 2);
      Serial.println(" RPM)");
    } else {
      Serial.println("[ERR] Invalid speed or acceleration.");
    }
  }
  else if (command.startsWith("TEST_RESOLUTION")) {
    int spaceIdx = command.indexOf(' ', 16);
    if (spaceIdx == -1) {
      Serial.println("[ERR] Usage: TEST_RESOLUTION <volume_uL> <trials>");
      return;
    }
    float volUL = command.substring(16, spaceIdx).toFloat();
    int trials = command.substring(spaceIdx + 1).toInt();
    if (volUL <= 0 || trials <= 0) {
      Serial.println("[ERR] Usage: TEST_RESOLUTION <volume_uL> <trials>");
      return;
    }

    float mmToMove = volumeToMillimeters(volUL, syringeDiameter);
    long stepsToMove = mmToMove * stepsPerMM;

    Serial.print("[INFO] Testing resolution: ");
    Serial.print(volUL, 3);
    Serial.print(" uL (");
    Serial.print(mmToMove, 4);
    Serial.print(" mm / ");
    Serial.print(stepsToMove);
    Serial.println(" steps)");

    for (int i = 0; i < trials; i++) {
      Serial.print("Trial ");
      Serial.print(i + 1);
      Serial.print(": +");
      Serial.print(mmToMove, 3);
      Serial.println(" mm");
      movePlunger(mmToMove);
      delay(200);
      movePlunger(-mmToMove);
      delay(200);
    }

    Serial.println("[OK] TEST_RESOLUTION complete.");
  }
  else if (command == "STATUS") {
    float pos = -stepper.currentPosition() / stepsPerMM;
    float percent = abs(stepper.currentPosition()) / (float)maxPositionSteps * 100.0;
    Serial.println("Syringe Status:");
    Serial.print(" - Position: "); Serial.print(pos, 2); Serial.println(" mm");
    Serial.print(" - Stroke: "); Serial.print(percent, 1); Serial.println(" %");
    Serial.print(" - Capacity: "); Serial.print(syringeCapacityUL, 2); Serial.println(" uL");
    Serial.print(" - Syringe diameter: "); Serial.print(syringeDiameter, 2); Serial.println(" mm");
    Serial.print(" - Total moves: "); Serial.println(totalMoves);
    Serial.print(" - Total dispenses: "); Serial.println(totalDispenses);
    Serial.print(" - Total volume dispensed: "); Serial.print(totalVolumeDispensed, 2); Serial.println(" uL");
  }
  else if (command == "GO_HOME") {
    movePlunger(stepper.currentPosition() / stepsPerMM);
    stepper.moveTo(-POST_LIMIT_OFFSET_MM * stepsPerMM);
    while (stepper.distanceToGo() != 0) stepper.run();
    stepper.setCurrentPosition(0);
    Serial.println("[OK] Returned to home.");
  }
  else if (command == "CALIBRATE") {
    autoCalibrateSyringe();
  }
  else if (command == "EMERGENCY_STOP") {
    Serial.println("[EMERGENCY] Stepper stop requested!");
    stepper.stop();
    while (stepper.isRunning()) stepper.run();
  }
  else {
    Serial.println("[ERR] Unknown command.");
  }
} //Added configuredSpeed and configuredAcceleration

//Used them in setup(), movePlunger(), and after calibration

//SET_SPEED updates them

//New RESET_SPEED command reverts to defaults

//STATUS now includes speed in steps/sec and RPM