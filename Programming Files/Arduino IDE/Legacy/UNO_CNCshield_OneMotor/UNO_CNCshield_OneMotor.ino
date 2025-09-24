#include <AccelStepper.h>

#define DIR_PIN 5      // X-axis direction pin
#define STEP_PIN 2     // X-axis step pin
#define ENABLE_PIN 8   // Enable pin for stepper driver

#define MICROSTEP_FACTOR 16  // Manually assigned microstepping factor

// State Definitions
enum State {
    READY,
    HOMING,
    IK_PROCESSING,
    MOVING,
    HELP
};

State currentState = READY;

// Stepper Motor
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); //to do, add 2nd motor functionality

// Global Variables
bool emergencyStop = false;
float targetAngle = 0.0;
float currentAngle = 0.0;
String command;
long steps;
float newTarget;
float diff;

void setup() {
    Serial.begin(9600);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); // Enable stepper driver

    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(500);
    
    Serial.println("System Ready. Type 'help' for available commands.");
}

void loop() {
    if (Serial.available()) {
        command = Serial.readStringUntil('\n');
        command.trim();
        Serial.println(command);

        if (command == "!") {
            emergencyStop = true;
            stepper.stop();
            Serial.println("EMERGENCY STOP ACTIVATED");
            command = "";
        } 
        else if (command == "reset") {
            emergencyStop = false;
            stepper.stop();
            stepper.setSpeed(0); // Ensure motor does not move after reset
            stepper.setCurrentPosition(stepper.currentPosition());  // Reassert current position without altering it
            currentAngle = stepper.currentPosition() * (360.0 / (200L * MICROSTEP_FACTOR)); // Align currentAngle
            targetAngle = currentAngle; // Prevent unexpected jumps
            Serial.println("System reset");
            currentState = READY;
            command = "";
        } 
    }

    if (!emergencyStop) {
        switch (currentState) {
            case READY:
                if (command.startsWith("move")) {  //things to do, fix high speed/bad acceleration after "reset" when specifically "move" is run
                    command = command.substring(5);
                    newTarget = command.toFloat();
                    if (newTarget > 3600) newTarget = 3600;
                    if (newTarget < -3600) newTarget = -3600;

                    stepper.setSpeed(0); //necessary to prevent odd movement after "reset"
                    currentAngle = stepper.currentPosition() * (360.0 / (200L * MICROSTEP_FACTOR)); // Sync current angle
                    targetAngle = currentAngle + newTarget;

                    Serial.print("New current angle: ");
                    Serial.println(targetAngle, 2);
                    currentState = MOVING;
                } 
                else if (command.startsWith("point")) {
                    command = command.substring(6);
                    newTarget = command.toFloat();

                    currentAngle = stepper.currentPosition() * (360.0 / (200L * MICROSTEP_FACTOR)); // Sync current angle
                    currentAngle = fmod(currentAngle, 360);
                    if (currentAngle < 0) currentAngle += 360;

                    diff = newTarget - currentAngle;
                    if (diff > 180) diff -= 360;
                    else if (diff < -180) diff += 360;

                    targetAngle += diff;  //targetAngle = currentAngle + diff;, change to this to make "point" absolute instead of relative.  Might come in handy later.

                    Serial.print("Target angle set to: ");
                    Serial.println(targetAngle, 2);
                    currentState = MOVING;
                } 
                else if (command == "help") {
                    currentState = HELP;
                } 
                break;

            case HELP:
                Serial.println("Available commands:");
                Serial.println("  move <angle>  - Moves by relative angle");
                Serial.println("  point <angle> - Moves to absolute angle (0-360)");
                Serial.println("  !             - Emergency stop");
                Serial.println("  reset         - Resets after emergency stop");
                Serial.println("  help          - Displays this message");
                currentState = READY;
                command = "";
                break;

            case HOMING:
                Serial.println("Homing function not implemented yet.");
                currentState = READY;
                command = "";
                break;

            case IK_PROCESSING:
                Serial.println("Inverse Kinematics processing not implemented yet.");
                currentState = READY;
                command = "";
                
                break;

            case MOVING:
                steps = (targetAngle * (200L * MICROSTEP_FACTOR)) / 360L;
                stepper.moveTo(steps);
                stepper.run();
                if (stepper.distanceToGo() == 0) {
                    Serial.println("Movement complete.");
                    currentState = READY;
                }
                break;
        }
    }
}
