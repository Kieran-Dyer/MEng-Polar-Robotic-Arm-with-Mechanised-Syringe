#include <AccelStepper.h>

// Pin Definitions
#define DIR_PIN_WAIST 5               // Waist (X-axis) direction pin
#define STEP_PIN_WAIST 2              // Waist (X-axis) step pin
#define WAIST_LIMIT_SWITCH_PIN  9     // X+ Limit Switch
#define DIR_PIN_SHOULDER 6            // Shoulder (Y-axis) direction pin
#define STEP_PIN_SHOULDER 3           // Shoulder (Y-axis) step pin
#define SHOULDER_LIMIT_SWITCH_PIN 10  // Y+ Limit Switch
#define DIR_PIN_ELBOW 7               // Elbow (Z-axis) direction pin
#define STEP_PIN_ELBOW 4              // Elbow (Z-axis) step pin
#define ELBOW_LIMIT_SWITCH_PIN 12     // Z+ Limit Switch

#define WAIST_GEAR_RATIO 3.95       // Waist gear ratio multiplier 79:20
#define SHOULDER_GEAR_RATIO 5.0     // Shoulder gear ratio multiplier 60:12
#define ELBOW_GEAR_RATIO 5.0        // elbow gear ratio multiplier 60:12

#define MICROSTEP_FACTOR 16  // Microstepping factor for stepper motors

// State Definitions
enum State {
    READY,          // System is idle and ready for commands
    HOMING,         // System is homing (not yet implemented)
    IK_PROCESSING,  // Inverse Kinematics processing (not yet implemented)
    MOVING,         // Motors are moving to target angles
    HELP            // Display help menu
};

State currentState;  // Tracks the current state of the system

enum HomingStep {
    HOMING_SHOULDER,  // Move the Shoulder first
    HOMING_ELBOW,     // Move the Elbow second
    HOMING_WAIST,     // Move the Waist last
    HOMING_COMPLETE   // Print that homing sequence is complete
};

HomingStep homingStep = HOMING_SHOULDER; // Tracks which joint is homing

enum HomingPhase { 
    MOVE_TOWARDS_SWITCH,  // Move towards the limit switch
    WAIT_FOR_TRIGGER,     // Wait until the switch is triggered
    MOVE_AWAY,            // Move away slightly
    FINAL_APPROACH        // Move back slowly towards the switch
};

HomingPhase phase = MOVE_TOWARDS_SWITCH; // Tracks phase of homing routine

// Stepper Motor Objects
AccelStepper waist(AccelStepper::DRIVER, STEP_PIN_WAIST, DIR_PIN_WAIST);
AccelStepper shoulder(AccelStepper::DRIVER, STEP_PIN_SHOULDER, DIR_PIN_SHOULDER);
AccelStepper elbow(AccelStepper::DRIVER, STEP_PIN_ELBOW, DIR_PIN_ELBOW);

// Global Variables
bool emergencyStop;  // Flag to indicate emergency stop state

// Command parsing variables
int startIndex;
int commaIndex;

// Angle tracking variables
float targetAngleWaist = 0.0;
float currentAngleWaist = 0.0;
float targetAngleShoulder = 0.0;
float currentAngleShoulder = 0.0;
float targetAngleElbow = 0.0;
float currentAngleElbow = 0.0;
float newTarget;
float diff;

// Command string
String command;

//ISR volatile Bools
volatile bool waistTriggered = false;
volatile bool shoulderTriggered = false;
volatile bool elbowTriggered = false;

void setup() {
    Serial.begin(9600);

    // Configure motor settings
    waist.setMaxSpeed(1000);
    waist.setAcceleration(500);
    shoulder.setMaxSpeed(1000);
    shoulder.setAcceleration(500);
    elbow.setMaxSpeed(1000);
    elbow.setAcceleration(500);

    // Configure the limit switch pins
    pinMode(WAIST_LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(SHOULDER_LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(ELBOW_LIMIT_SWITCH_PIN, INPUT_PULLUP);

    // If using interrupts for limit switches (optional)
    attachInterrupt(digitalPinToInterrupt(WAIST_LIMIT_SWITCH_PIN), waistHoming, FALLING);
    attachInterrupt(digitalPinToInterrupt(SHOULDER_LIMIT_SWITCH_PIN), shoulderHoming, FALLING);
    attachInterrupt(digitalPinToInterrupt(ELBOW_LIMIT_SWITCH_PIN), elbowHoming, FALLING);

    emergencyStop = false;
    command = "";
    currentState = READY;

    Serial.println("System Ready. Type 'help' for available commands.");
}

void loop() {

    //********* Command Parser *********//

    if (Serial.available()) {
        command = Serial.readStringUntil('\n'); // Read incoming command
        command.trim(); // Remove leading/trailing spaces
        Serial.println(command);

        // Split commands by comma to allow multiple commands in one message
        startIndex = 0;
        while ((commaIndex = command.indexOf(',', startIndex)) != -1) {
            processCommand(command.substring(startIndex, commaIndex));
            startIndex = commaIndex + 1;
        }
        processCommand(command.substring(startIndex)); // Process last command
    }

    //********* Finite State Machine *********//
    
    if (!emergencyStop) {
        switch (currentState) {
            case READY:
                break; // System is idle
            case HOMING:
                switch (homingStep) {
                    case HOMING_SHOULDER:
                        if (homeAxis(shoulder, SHOULDER_LIMIT_SWITCH_PIN, shoulderTriggered)) {
                            homingStep = HOMING_ELBOW;
                        }
                        break;
                    case HOMING_ELBOW:
                        if (homeAxis(elbow, ELBOW_LIMIT_SWITCH_PIN, elbowTriggered)) {
                            homingStep = HOMING_WAIST;
                        }
                        break;
                    case HOMING_WAIST:
                        if (homeAxis(waist, WAIST_LIMIT_SWITCH_PIN, waistTriggered)) {
                            homingStep = HOMING_COMPLETE;
                        }
                        break;
                    case HOMING_COMPLETE:
                        Serial.println("Homing Sequence Complete");
                        currentState = READY;
                        break;
                }
                break;
            case IK_PROCESSING:
                break; // IK processing logic to be implemented
            case MOVING:
                // Convert angles to stepper positions and move motors
                waist.moveTo(((targetAngleWaist * WAIST_GEAR_RATIO) * (200L * MICROSTEP_FACTOR)) / 360L);
                shoulder.moveTo(((targetAngleShoulder * SHOULDER_GEAR_RATIO) * (200L * MICROSTEP_FACTOR)) / 360L);
                elbow.moveTo(((targetAngleElbow * ELBOW_GEAR_RATIO) * (200L * MICROSTEP_FACTOR)) / 360L);
                waist.run();
                shoulder.run();
                elbow.run();
                // Check if movement is complete
                if (waist.distanceToGo() == 0 && shoulder.distanceToGo() == 0 && elbow.distanceToGo() == 0) {
                    Serial.println("Movement complete");
                    currentState = READY;
                }
                break;
            case HELP:
                // Display available commands
                Serial.println("Available commands:");
                Serial.println("! - Emergency Stop");
                Serial.println("reset - Reset system");
                Serial.println("move [motor] <angle.f> - Move motor by angle");
                Serial.println("point [motor] <angle.f> - Point motor to relative angle");
                Serial.println("Available motors: waist, shoulder, elbow");
                currentState = READY;
                break;
        }
    }
}

//********* Input Serial Command Parser w/ target angle calculations *********//

void processCommand(String cmd) {
    cmd.trim();
    
    if (cmd == "!") {
        // Emergency stop all motors
        emergencyStop = true;
        waist.stop();
        shoulder.stop();
        elbow.stop();
        Serial.println("EMERGENCY STOP ACTIVATED");
    } 
    else if (cmd == "reset") {
        // Reset system state and motor positions
        emergencyStop = false;
        waist.stop();
        waist.setSpeed(0);
        shoulder.stop();
        shoulder.setSpeed(0);
        elbow.stop();
        elbow.setSpeed(0);
        Serial.println("System reset");
    }
    else if (cmd == "home") {
        currentState = HOMING;
    }
    else if (cmd.startsWith("move waist")) {
    cmd = cmd.substring(10);
    newTarget = cmd.toFloat();
    currentAngleWaist = (waist.currentPosition() * 360.0) / (200L * MICROSTEP_FACTOR * WAIST_GEAR_RATIO);
    targetAngleWaist = currentAngleWaist + newTarget;
    currentState = MOVING;
    Serial.print("New absolute waist angle: ");
    Serial.println(targetAngleWaist, 2);
    }
    else if (cmd.startsWith("move shoulder")) {
        cmd = cmd.substring(13);
        newTarget = cmd.toFloat();
        currentAngleShoulder = (shoulder.currentPosition() * 360.0) / (200L * MICROSTEP_FACTOR * SHOULDER_GEAR_RATIO);
        targetAngleShoulder = currentAngleShoulder + newTarget;
        currentState = MOVING;
        Serial.print("New absolute shoulder angle: ");
        Serial.println(targetAngleShoulder, 2);
    }
    else if (cmd.startsWith("move elbow")) {
        cmd = cmd.substring(10);
        newTarget = cmd.toFloat();
        currentAngleElbow = (elbow.currentPosition() * 360.0) / (200L * MICROSTEP_FACTOR * ELBOW_GEAR_RATIO);
        targetAngleElbow = currentAngleElbow + newTarget;
        currentState = MOVING;
        Serial.print("New absolute elbow angle: ");
        Serial.println(targetAngleElbow, 2);
    }
    else if (cmd.startsWith("point waist")) {
        cmd = cmd.substring(11);
        newTarget = cmd.toFloat();
        currentAngleWaist = (waist.currentPosition() * 360.0) / (200L * MICROSTEP_FACTOR * WAIST_GEAR_RATIO);
        targetAngleWaist = newTarget; // Absolute positioning
        currentState = MOVING;
        Serial.print("Target waist angle set to: ");
        Serial.println(targetAngleWaist, 2);
    }
    else if (cmd.startsWith("point shoulder")) {
        cmd = cmd.substring(14);
        newTarget = cmd.toFloat();
        currentAngleShoulder = (shoulder.currentPosition() * 360.0) / (200L * MICROSTEP_FACTOR * SHOULDER_GEAR_RATIO);
        targetAngleShoulder = newTarget; // Absolute positioning
        currentState = MOVING;
        Serial.print("Target shoulder angle set to: ");
        Serial.println(targetAngleShoulder, 2);
    }
    else if (cmd.startsWith("point elbow")) {
        cmd = cmd.substring(11);
        newTarget = cmd.toFloat();
        currentAngleElbow = (elbow.currentPosition() * 360.0) / (200L * MICROSTEP_FACTOR * ELBOW_GEAR_RATIO);
        targetAngleElbow = newTarget; // Absolute positioning
        currentState = MOVING;
        Serial.print("Target elbow angle set to: ");
        Serial.println(targetAngleElbow, 2);
    }
    else if (cmd == "help") {
        currentState = HELP;
    }
    else if (cmd != "") {
        Serial.println("Command not recognised");
    }
}

//********* Stepper Motor Homing Sequence Function *********//

bool homeAxis(AccelStepper &motor, int limitSwitchPin, volatile bool &triggeredFlag) {

    switch (phase) {
        case MOVE_TOWARDS_SWITCH:
            motor.setMaxSpeed(100);
            motor.setAcceleration(50);
            motor.move(-5000); // Arbitrary large negative move
            phase = WAIT_FOR_TRIGGER;
            break;

        case WAIT_FOR_TRIGGER:
            // Check if the interrupt has set the flag
            if (triggeredFlag) {
                motor.stop();
                motor.move(50); // Move away slightly
                phase = MOVE_AWAY;
                triggeredFlag = false; // Reset flag after handling
            }
            break;

        case MOVE_AWAY:
            if (motor.distanceToGo() == 0) {
                motor.setMaxSpeed(50);
                motor.move(-5000); // Move slowly back
                phase = FINAL_APPROACH;
            }
            break;

        case FINAL_APPROACH:
            // Check again if the switch has been triggered
            if (triggeredFlag) {
                motor.stop();
                motor.setCurrentPosition(0);
                phase = MOVE_TOWARDS_SWITCH; // Reset for next motor
                triggeredFlag = false; // Reset flag after homing
                return true; // Homing done
            }
            break;
    }
    motor.run();
    return false; // Still homing
}

//********* ISR service Routines *********//

void waistHoming() {
    waistTriggered = true;
}

void shoulderHoming() {
    shoulderTriggered = true;
}

void elbowHoming() {
    elbowTriggered = true;
}