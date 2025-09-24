#include <AccelStepper.h>

// Pin Definitions
#define DIR_PIN_WAIST 5      // Waist (X-axis) direction pin
#define STEP_PIN_WAIST 2     // Waist (X-axis) step pin
#define DIR_PIN_SHOULDER 6   // Shoulder (Y-axis) direction pin
#define STEP_PIN_SHOULDER 3  // Shoulder (Y-axis) step pin

#define MICROSTEP_FACTOR 16  // Microstepping factor for stepper motors

#define SHOULDER_GEAR_FACTOR 5 //

// State Definitions
enum State {
    READY,          // System is idle and ready for commands
    HOMING,         // System is homing (not yet implemented)
    IK_PROCESSING,  // Inverse Kinematics processing (not yet implemented)
    MOVING,         // Motors are moving to target angles
    HELP            // Display help menu
};

State currentState;

// Stepper Motor Objects
AccelStepper waist(AccelStepper::DRIVER, STEP_PIN_WAIST, DIR_PIN_WAIST);
AccelStepper shoulder(AccelStepper::DRIVER, STEP_PIN_SHOULDER, DIR_PIN_SHOULDER);

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
float newTarget;
float diff;

// Command string
String command;

void setup() {
    Serial.begin(9600);
  
    // Configure motor settings
    waist.setMaxSpeed(1000);
    waist.setAcceleration(500);
    shoulder.setMaxSpeed(1000);
    shoulder.setAcceleration(500);

    emergencyStop = false;  
    command = "";
    currentState = READY;
    
    Serial.println("System Ready. Type 'help' for available commands.");
}

void loop() {
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

    if (!emergencyStop) {
        switch (currentState) {
            case READY:
                break; // System is idle
            case HOMING:
                break; // Homing logic to be implemented
            case IK_PROCESSING:
                break; // IK processing logic to be implemented
            case MOVING:
                // Convert angles to stepper positions and move motors
                waist.moveTo((targetAngleWaist * (200L * MICROSTEP_FACTOR)) / 360L);
                shoulder.moveTo((targetAngleShoulder * (200L * MICROSTEP_FACTOR)) / 360L);
                waist.run();
                shoulder.run();
                // Check if movement is complete
                if (waist.distanceToGo() == 0 && shoulder.distanceToGo() == 0) {
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
                Serial.println("Available motors: waist, shoulder");
                currentState = READY;
                break;
        }
    }
}

void processCommand(String cmd) {
    cmd.trim();
    
    if (cmd == "!") {
        // Emergency stop all motors
        emergencyStop = true;
        waist.stop();
        shoulder.stop();
        Serial.println("EMERGENCY STOP ACTIVATED");
    } 
    else if (cmd == "reset") {
        // Reset system state and motor positions
        emergencyStop = false;
        waist.stop();
        waist.setSpeed(0);
        waist.setCurrentPosition(waist.currentPosition());
        currentAngleWaist = waist.currentPosition() * (360.0 / (200L * MICROSTEP_FACTOR));
        targetAngleWaist = currentAngleWaist;
        
        shoulder.stop();
        shoulder.setSpeed(0);
        shoulder.setCurrentPosition(shoulder.currentPosition());
        currentAngleShoulder = shoulder.currentPosition() * (360.0 / (200L * MICROSTEP_FACTOR));
        targetAngleShoulder = currentAngleShoulder;
        
        Serial.println("System reset");
    } 
    else if (cmd.startsWith("move waist")) {
        // Increment waist angle
        cmd = cmd.substring(10);
        newTarget = cmd.toFloat();
        targetAngleWaist += newTarget;
        currentState = MOVING;
        Serial.print("New waist angle: ");
        Serial.println(targetAngleWaist, 2);
    }
    else if (cmd.startsWith("move shoulder")) {
        // Increment shoulder angle
        cmd = cmd.substring(13);
        newTarget = cmd.toFloat();
        targetAngleShoulder += newTarget;
        currentState = MOVING;
        Serial.print("New shoulder angle: ");
        Serial.println(targetAngleShoulder, 2);
    }
    else if (cmd.startsWith("point waist")) {
        // Set waist to absolute angle
        cmd = cmd.substring(12);
        newTarget = cmd.toFloat();
        
        currentAngleWaist = waist.currentPosition() * (360.0 / (200L * MICROSTEP_FACTOR));
        currentAngleWaist = fmod(currentAngleWaist, 360);
        if (currentAngleWaist < 0) currentAngleWaist += 360;
        
        diff = newTarget - currentAngleWaist;
        if (diff > 180) diff -= 360;
        else if (diff < -180) diff += 360;
        
        targetAngleWaist += diff;
        currentState = MOVING;
        Serial.print("Target waist angle set to: ");
        Serial.println(targetAngleWaist, 2);
    }
    else if (cmd.startsWith("point shoulder")) {
        // Set shoulder to absolute angle
        cmd = cmd.substring(15);
        newTarget = cmd.toFloat();
        
        currentAngleShoulder = shoulder.currentPosition() * (360.0 / (200L * MICROSTEP_FACTOR));
        currentAngleShoulder = fmod(currentAngleShoulder, 360);
        if (currentAngleShoulder < 0) currentAngleShoulder += 360;
        
        diff = newTarget - currentAngleShoulder;
        if (diff > 180) diff -= 360;
        else if (diff < -180) diff += 360;
        
        targetAngleShoulder += diff;
        currentState = MOVING;
        Serial.print("Target shoulder angle set to: ");
        Serial.println(targetAngleShoulder, 2);
    }
    else if (cmd == "help") {
        currentState = HELP;
    }
    else if (cmd != "") {
        Serial.println("Command not recognised");
    }
}