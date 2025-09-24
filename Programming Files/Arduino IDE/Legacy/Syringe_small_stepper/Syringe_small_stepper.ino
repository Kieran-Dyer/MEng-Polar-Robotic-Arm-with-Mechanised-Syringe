#include <AccelStepper.h>

#define STEP_PIN         3
#define DIR_PIN          2
#define ENABLE_PIN       4 //tied LOW

#define LIMIT_SWITCH_PIN 7
#define MAX_STOP_BUTTON  8

#define STEPS_PER_REV    2048.0
#define MICROSTEPS       1.0
#define THREAD_PITCH     8.0
#define DEFAULT_SYRINGE_DIAMETER 2.3

#define HOMING_SPEED     450

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

float syringeDiameter = DEFAULT_SYRINGE_DIAMETER;
float stepsPerMM = (STEPS_PER_REV * MICROSTEPS) / THREAD_PITCH;

bool isCalibrated = false;
float syringeCapacityUL = 0.0;
long maxPositionSteps = 0;

//  CORRECT: µL to mm based on syringe inner diameter
float volumeToMillimeters(float volumeUL, float diameterMM) {
  float areaMM2 = PI * pow(diameterMM / 2.0, 2); // mm²
  return volumeUL / areaMM2; // mm³ / mm² = mm
}

void setup() {
  Serial.begin(9600);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MAX_STOP_BUTTON, INPUT_PULLUP);
  digitalWrite(ENABLE_PIN, LOW);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  Serial.println("=== Syringe System Booting ===");
  Serial.print("Using default syringe diameter: ");
  Serial.print(syringeDiameter);
  Serial.println(" mm");

  Serial.println("Starting auto-calibration...");
  autoCalibrateSyringe();
  if (isCalibrated) {
    Serial.print("Calibration complete. Syringe capacity: ");
    Serial.print(syringeCapacityUL, 2);
    Serial.println(" µL");
    Serial.println("Available commands: SET_DIAMETER, MOVE_MM, MOVE_VOL, DISPENSE, CALIBRATE, STATUS");
  }
}

void loop() {
  if (!isCalibrated) return;
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();
    processCommand(input);
  }
}

void processCommand(String command) {
  if (command.startsWith("SET_DIAMETER")) {
    float newDiameter = command.substring(12).toFloat();
    if (newDiameter > 0.0) {
      syringeDiameter = newDiameter;
      Serial.print("Syringe diameter updated to: ");
      Serial.print(syringeDiameter);
      Serial.println(" mm");
    } else {
      Serial.println("Error: Diameter must be positive.");
    }
  } 
  else if (command.startsWith("MOVE_MM")) {
    float mmToMove = command.substring(8).toFloat();
    Serial.print("MOVE_MM: ");
    Serial.print(mmToMove, 3);
    Serial.println(" mm");
    movePlunger(mmToMove);
  } 
  else if (command.startsWith("MOVE_VOL")) {
    float volume = command.substring(9).toFloat();

    if (volume <= 0) {
      Serial.println("Error: Volume must be greater than zero.");
      return;
    }
    if (syringeDiameter <= 0) {
      Serial.println("Error: Invalid syringe diameter.");
      return;
    }

    float mmToMove = volumeToMillimeters(volume, syringeDiameter);

    Serial.print("MOVE_VOL: ");
    Serial.print(volume);
    Serial.print(" µL → ");
    Serial.print(mmToMove, 3);
    Serial.println(" mm");

    movePlunger(mmToMove);
  } 
  else if (command.startsWith("DISPENSE")) {
    float volume = command.substring(8).toFloat();

    if (volume <= 0) {
      Serial.println("Error: Volume must be greater than zero.");
      return;
    }
    if (syringeDiameter <= 0) {
      Serial.println("Error: Invalid syringe diameter.");
      return;
    }
    if (volume > syringeCapacityUL) {
      Serial.println("Requested volume exceeds syringe capacity.");
      return;
    }

    float mmToMove = volumeToMillimeters(volume, syringeDiameter);
    float currentMM = stepper.currentPosition() / stepsPerMM;
    float maxMM = maxPositionSteps / stepsPerMM;

    if ((currentMM + mmToMove) > maxMM + 0.01) {
      Serial.println("Plunger would overextend. Dispense canceled.");
      return;
    }

    Serial.print("Drawing ");
    Serial.print(volume, 2);
    Serial.print(" µL (");
    Serial.print(mmToMove, 3);
    Serial.println(" mm)");
    movePlunger(mmToMove);

    delay(1000);

    Serial.println("Dispensing...");
    movePlunger(-mmToMove);

    Serial.println("Returning to home...");
    movePlunger(-stepper.currentPosition() / stepsPerMM);
  } 
  else if (command.startsWith("STATUS")) {
    float pos = stepper.currentPosition() / stepsPerMM;
    float percent = (float)stepper.currentPosition() / maxPositionSteps * 100.0;
    Serial.println("Syringe Status:");
    Serial.print(" - Position: "); Serial.print(pos, 2); Serial.println(" mm");
    Serial.print(" - Stroke: "); Serial.print(percent, 1); Serial.println(" %");
    Serial.print(" - Capacity: "); Serial.print(syringeCapacityUL, 2); Serial.println(" µL");
    Serial.print(" - Syringe diameter: "); Serial.print(syringeDiameter, 2); Serial.println(" mm");
  } 
  else if (command.startsWith("CALIBRATE")) {
    autoCalibrateSyringe();
  } 
  else {
    Serial.println("Unknown command. Try: SET_DIAMETER, MOVE_MM, MOVE_VOL, DISPENSE, CALIBRATE, STATUS");
  }
}

void movePlunger(float mmToMove) {
  if (abs(mmToMove) < 0.001) {
    Serial.println("Requested move is ~0 mm. Skipping.");
    return;
  }

  long target = stepper.currentPosition() + mmToMove * stepsPerMM;

  Serial.print("Requested move: ");
  Serial.print(mmToMove, 3);
  Serial.print(" mm → target: ");
  Serial.print(target);
  Serial.println(" steps");

  if (target > maxPositionSteps) target = maxPositionSteps;
  if (target < 0) target = 0;

  stepper.moveTo(target);
  while (stepper.distanceToGo() != 0) {
    if (digitalRead(MAX_STOP_BUTTON) == LOW) {
      Serial.println("Max stop hit.");
      stepper.stop();
      break;
    }
    stepper.run();
  }

  Serial.println("Movement complete.");
}

void autoCalibrateSyringe() {
  Serial.println("[AutoCal] Homing...");
  stepper.setMaxSpeed(HOMING_SPEED);
  stepper.setAcceleration(HOMING_SPEED * 2);
  stepper.moveTo(-15000);
  while (digitalRead(LIMIT_SWITCH_PIN) == LOW) stepper.run();
  stepper.stop(); while (stepper.isRunning()) stepper.run();
  stepper.setCurrentPosition(0);
  Serial.println("[AutoCal] Home set.");

  Serial.println("[AutoCal] Finding max travel...");
  stepper.setMaxSpeed(600);
  stepper.setAcceleration(400);
  stepper.moveTo(20000);
  while (digitalRead(MAX_STOP_BUTTON) == HIGH) stepper.run();
  stepper.stop(); while (stepper.isRunning()) stepper.run();

  maxPositionSteps = stepper.currentPosition();
  float travelMM = maxPositionSteps / stepsPerMM;
  syringeCapacityUL = PI * pow(syringeDiameter / 2.0, 2) * travelMM;

  Serial.print("[AutoCal] Max travel: "); Serial.print(travelMM, 2); Serial.println(" mm");
  Serial.print("[AutoCal] Syringe capacity: "); Serial.print(syringeCapacityUL, 2); Serial.println(" µL");

  Serial.println("[AutoCal] Returning to home...");
  stepper.moveTo(0);
  while (stepper.distanceToGo() != 0) stepper.run();
  Serial.println("[AutoCal] Calibration complete.");
  isCalibrated = true;
}
