// Syringe Module Control Code

#include <AccelStepper.h>

#define STEP_PIN 2       // A4988 Step pin
#define DIR_PIN 3        // A4988 Direction pin
#define ENABLE_PIN 4     // A4988 Enable pin

#define STEPS_PER_REV 200.0  // Steps per revolution for Nema 8 motor
#define MICROSTEPS 16.0      // Microstepping (1/16)
#define THREAD_PITCH 1.0     // Lead screw pitch (mm per revolution)
#define DEFAULT_SYRINGE_DIAMETER 10.0  // Default syringe diameter in mm

AccelStepper stepper(1, STEP_PIN, DIR_PIN); // Using driver mode (1), STEP and DIR pins

float syringeDiameter = DEFAULT_SYRINGE_DIAMETER;
float stepsPerMM = (STEPS_PER_REV * MICROSTEPS) / THREAD_PITCH;

void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable the motor driver
  Serial.println("Syringe Control System Ready. Enter commands.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    processCommand(input);
  }
}

void processCommand(String command) {
  if (command.startsWith("SET_DIAMETER")) {
    syringeDiameter = command.substring(12).toFloat();
    Serial.print("Syringe Diameter set to: ");
    Serial.println(syringeDiameter);
  } 
  else if (command.startsWith("MOVE_MM")) {
    float mmToMove = command.substring(8).toFloat();
    movePlunger(mmToMove);
  }
  else if (command.startsWith("MOVE_VOL")) {
    float volumeToDispense = command.substring(9).toFloat();
    float mmToMove = volumeToDispense / (PI * pow(syringeDiameter / 2.0, 2)) * 1000;
    movePlunger(mmToMove);
  }
  else if (command.startsWith("CALIBRATE")) {
    Serial.println("Calibration mode. Move motor manually if needed.");
    delay(2000);
  }
  else {
    Serial.println("Unknown command. Available commands: SET_DIAMETER, MOVE_MM, MOVE_VOL, CALIBRATE");
  }
}

void movePlunger(float mmToMove) {
  long stepsToMove = mmToMove * stepsPerMM;
  stepper.move(stepsToMove);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  Serial.print("Movement completed. Moved: ");
  Serial.print(mmToMove);
  Serial.println(" mm");
}
