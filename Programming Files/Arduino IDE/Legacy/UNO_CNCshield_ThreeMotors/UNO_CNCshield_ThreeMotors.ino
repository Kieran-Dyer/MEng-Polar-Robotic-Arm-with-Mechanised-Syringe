#include "AccelStepper.h"
#include "math.h"

// Pin Definitions
#define DIR_PIN_WAIST 3               // Waist (X-axis) direction pin
#define STEP_PIN_WAIST 2              // Waist (X-axis) step pin
#define WAIST_LIMIT_SWITCH_PIN A0     // X+ Limit Switch

#define DIR_PIN_SHOULDER 5            // Shoulder (Y-axis) direction pin
#define STEP_PIN_SHOULDER 4           // Shoulder (Y-axis) step pin
#define SHOULDER_LIMIT_SWITCH_PIN A1  // Y+ Limit Switch

#define DIR_PIN_ELBOW 7               // Elbow (Z-axis) direction pin
#define STEP_PIN_ELBOW 6              // Elbow (Z-axis) step pin
#define ELBOW_LIMIT_SWITCH_PIN A2     // Z+ Limit Switch

#define DIR_PIN_SYRINGE 9             // Syringe direction pin
#define STEP_PIN_SYRINGE 8            // Syringe step pin
#define SYRINGE_LIMIT_SWITCH_PIN A3   // Syring limit pin

//Additional Definitions
#define WAIST_GEAR_RATIO 3.95       // Waist gear ratio multiplier 79:20
#define SHOULDER_GEAR_RATIO 5.0     // Shoulder gear ratio multiplier 60:12
#define ELBOW_GEAR_RATIO 5.0        // Elbow gear ratio multiplier 60:12
#define SYRINGE_GEAR_RATIO 8        // Syringe threaded rod gear ratio XX:XX

#define ARM_MICROSTEP_FACTOR 16     // Microstepping factor for shoulder and elbow
#define WAIST_MICROSTEP_FACTOR 16   // Microstepping factor for waist
#define SYRINGE_MICROSTEP_FACTOR 1  // Microstepping factor for Syringe

#define SYRINGE_DIAMETER 1     // Cross sectional area of the syringe in mm

// State Definitions
enum State {
    READY,          // System is idle and ready for commands
    HOMING,         // System is homing (not yet implemented)
    IVK_PROCESSING,  // Inverse Kinematics processing (not yet implemented)
    MOVING,         // Motors are moving to target angles
    SYRINGE,        // Move the syringe motor to inject or draw the desired volume
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
    MOVE_AWAY_1,          // Move away slightly in case there is a collision.
    MOVE_TOWARDS_SWITCH,  // Move towards the limit switch
    WAIT_FOR_TRIGGER,     // Wait until the switch is triggered
    MOVE_AWAY_2,          // Move away slightly after hitting the switch
    FINAL_APPROACH        // Move back slowly towards the switch
};

HomingPhase phase; // Tracks phase of homing routine

// Stepper Motor Objects
AccelStepper waist(AccelStepper::DRIVER, STEP_PIN_WAIST, DIR_PIN_WAIST);
AccelStepper shoulder(AccelStepper::DRIVER, STEP_PIN_SHOULDER, DIR_PIN_SHOULDER);
AccelStepper elbow(AccelStepper::DRIVER, STEP_PIN_ELBOW, DIR_PIN_ELBOW);
AccelStepper syringe(AccelStepper::DRIVER, STEP_PIN_SYRINGE, DIR_PIN_SYRINGE);

// Global Variables
// Booleans
bool emergencyStop;  // Flag to indicate emergency stop state
bool ivkFlag;   // Flag to tell the parser functions that the IVK calculation has finished

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

float targetPositionSyringe = 0.0;
float currentPositionSyringe = 0.0;
const float crossSectionArea = square(SYRINGE_DIAMETER/2.0) * M_PI;  // Area of a circle A = pi*(r^2)

float newTarget;
float diff;

// ------- IVK floats -------

// Angle conversions (probably not necessary)
float radToDeg = 180.0 / M_PI;
float degToRad = M_PI / 180;

// Orange Vector Offsets in mm
const float O_X_offset = -117.316;
const float O_Y_offset = -88.741;
const float O_Z_offset = 45.064;

// Purple Vector Offsets in mm
const float P_L_4 = 24.884;
const float P_Y_offset = 88.741;
const float P_Z_3 = -106.808;

// Blue Vector Offsets in mm
const float B_L_1 = 35;

// Arm segment length in mm
const float A = 175;

// Offset Angle values in degrees, converted to radians
const float theta_offset_1 = degToRad * 123.13;
const float theta_offset_2 = degToRad * 27.88;
const float theta_offset_3 = degToRad * -10;
const float theta_offset_4 = degToRad * 28.99; // Predefined offset angles in deg

// IVK variable variables
float X_pos, Y_pos, Z_pos; // Target position in Cartesian coordinates

float theta_3;
float BigRadius_RT;
float theta_offset_5;
float Radius;
float q_2;
float q_1;
float theta_2;
float theta_1;
float theta_2_min;
float theta_2_max;

// Command string
String command;

void setup() {
    Serial.begin(9600);

    // Configure motor settings
    waist.setMaxSpeed(1000 * WAIST_MICROSTEP_FACTOR/16);
    waist.setAcceleration(500 * WAIST_MICROSTEP_FACTOR/16);
    waist.setCurrentPosition(0);

    shoulder.setMaxSpeed(1000 * ARM_MICROSTEP_FACTOR/16);
    shoulder.setAcceleration(500 * ARM_MICROSTEP_FACTOR/16);
    shoulder.setCurrentPosition(0);

    elbow.setMaxSpeed(1000 * ARM_MICROSTEP_FACTOR/16);
    elbow.setAcceleration(500 * ARM_MICROSTEP_FACTOR/16);
    elbow.setCurrentPosition(0);

    // Configure the limit switch pins
    pinMode(WAIST_LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(SHOULDER_LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(ELBOW_LIMIT_SWITCH_PIN, INPUT_PULLUP);
    //pinMode(SYRINGE_LIMIT_SWITCH_PIN, INPUT_PULLUP);

//clear variables
    emergencyStop = false;
    ivkFlag = false;
    command = "";
    currentState = READY;
    homingStep = HOMING_SHOULDER;
    phase = MOVE_AWAY_1;

    Serial.println("");
    Serial.println("System Ready. Type 'help' for available commands.");
}

void loop() {

    //********* Command Parser *********//
    if (Serial.available() || ivkFlag) {
        if(ivkFlag){  // don't oeverwrite the pre-written 'command' string from IVK_PROCESSING.
            ivkFlag = false; // Reset the flag
        } else {
            command = Serial.readStringUntil('\n'); // Read incoming command
            command.trim(); // Remove leading/trailing spaces
        }
        Serial.println(command);

        // Check if the command starts with "goto ("
        if (command.startsWith("goto (") && command.endsWith(")")) {
            // Extract and parse X, Y, Z
            command = command.substring(6, command.length() - 1); // Remove "goto (" and ")"

            int space1 = command.indexOf(' ');
            int space2 = command.lastIndexOf(' ');

            if (space1 != -1 && space2 != -1 && space1 != space2) {
                X_pos = command.substring(0, space1).toFloat();
                Y_pos = command.substring(space1 + 1, space2).toFloat();
                Z_pos = command.substring(space2 + 1).toFloat();

                Serial.println("Moving to (" + String(X_pos, 2) + ", " + String(Y_pos, 2) + ", " + String(Z_pos, 2) + ")");

                currentState = IVK_PROCESSING;
            } else {
                Serial.println("Invalid format. Use: goto (X, Y, Z)");
            }
        } else {
            // Split commands by comma to allow multiple commands in one message
            startIndex = 0;
            while ((commaIndex = command.indexOf(',', startIndex)) != -1) {
                processCommand(command.substring(startIndex, commaIndex));
                startIndex = commaIndex + 1;
            }
            processCommand(command.substring(startIndex)); // Process last command
        }
    }

    //********* Finite State Machine *********//
    
    if (!emergencyStop) {
        switch (currentState) {
            case READY:
                break; // System is idle
            case HOMING:
                switch (homingStep) {
                    case HOMING_SHOULDER:
                        if (homeAxis(shoulder, ARM_MICROSTEP_FACTOR, SHOULDER_LIMIT_SWITCH_PIN)) {
                            homingStep = HOMING_ELBOW;
                        }
                        break;
                    case HOMING_ELBOW:
                        if (homeAxis(elbow, ARM_MICROSTEP_FACTOR, ELBOW_LIMIT_SWITCH_PIN)) {
                            homingStep = HOMING_WAIST;
                        }
                        break;
                    case HOMING_WAIST:
                        if (homeAxis(waist, WAIST_MICROSTEP_FACTOR, WAIST_LIMIT_SWITCH_PIN)) {
                            homingStep = HOMING_COMPLETE;
                        }
                        break;
                    case HOMING_COMPLETE:
                        Serial.println("Homing Sequence Complete");
                        homingStep = HOMING_SHOULDER;
                        currentState = READY;
                        break;
                }
                break;
            case IVK_PROCESSING:
                // Calculate waistMotorAngle on the XY plane
                theta_3 = atan2((Y_pos - O_Y_offset), (X_pos - O_X_offset));
                BigRadius_RT = sqrt(pow(X_pos - O_X_offset, 2) + pow(Y_pos - O_Y_offset, 2));
                theta_offset_5 = asin(P_Y_offset / BigRadius_RT);
                targetAngleWaist = theta_3 - theta_offset_5;

                // calculate the total length of the arm on the RZ plane
                Radius = sqrt(pow(BigRadius_RT, 2) - pow(P_Y_offset, 2));

                // Calculate q_2 relative to q_1 using cosine law trigonometry
                q_2 = acos((pow(Radius - B_L_1 - P_L_4, 2) + pow(Z_pos - O_Z_offset - P_Z_3, 2)) / (2 * A * A) - 1);
                
                // Calculate q_1 relative to horizontal using 2 tan inverse trigonometry
                q_1 = atan2((Z_pos - O_Z_offset - P_Z_3), (Radius - B_L_1 - P_L_4)) - atan2(A * sin(q_2), A + A * cos(q_2));
                
                // Convert the q angles to theta angles
                theta_1 = q_2 + q_1;
                theta_2 = q_1;

                // Constrain upper arm to not exceed physical limitations
                theta_1 = constrain(theta_1, 0, theta_offset_1);

                // Calculate theta_2 limits based on shoulder angle
                theta_2_min = -M_PI + theta_1 + theta_offset_2;  // lower arm lower limit relative to the upper arm (avoids over-flexing elbow)
                theta_2_max = theta_1 + theta_offset_3;          // lower arm upper bound relative to the upper arm (avoids linkage snapping)

                // Clamp theta_2 to stay within mechanical limits
                theta_2 = constrain(theta_2, theta_2_min, theta_2_max);

                // Final target angles for motors, relative to the Zero offsets
                targetAngleShoulder = theta_1 - theta_offset_1;
                targetAngleElbow = theta_offset_4 + theta_2;
                
                // Convert from radians to degrees
                targetAngleWaist *= radToDeg;
                targetAngleShoulder *= radToDeg;
                targetAngleElbow *= radToDeg;

                // Send Serial Command String
                command = "point waist " + String(targetAngleWaist, 2) + ", point shoulder " + String(targetAngleShoulder, 2) + ", point elbow " + String(targetAngleElbow, 2);
                ivkFlag = true;
                currentState = READY;
                break;
            case MOVING:
                // Convert angles to stepper positions and move motors
                waist.moveTo(((-targetAngleWaist * WAIST_GEAR_RATIO) * (200L * WAIST_MICROSTEP_FACTOR)) / 360L);
                shoulder.moveTo(((targetAngleShoulder * SHOULDER_GEAR_RATIO) * (200L * ARM_MICROSTEP_FACTOR)) / 360L);
                elbow.moveTo(((-targetAngleElbow * ELBOW_GEAR_RATIO) * (200L * ARM_MICROSTEP_FACTOR)) / 360L);
                waist.run();
                shoulder.run();
                elbow.run();
                // Check if movement is complete
                if (waist.distanceToGo() == 0 && shoulder.distanceToGo() == 0 && elbow.distanceToGo() == 0) {
                    Serial.println("Arm movement complete");
                    currentState = READY;
                }
                break;
            case SYRINGE:
                // Move the syinge to inject or draw the desired amount
                //syringe.moveTo(((targetPositionSyringe * SYRING_GEAR_RATIO) * (200L * SYRINGE_MICROSTEP_FACTOR)) / 360L);
                syringe.run();
                // check if movement is complete
                if (syringe.distanceToGo() == 0) {
                    Serial.println("Syringe movement complete");
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
        targetAngleWaist = (-waist.currentPosition() * 360.0) / (200L * WAIST_MICROSTEP_FACTOR * WAIST_GEAR_RATIO);
        shoulder.stop();
        shoulder.setSpeed(0);
        targetAngleShoulder = (shoulder.currentPosition() * 360.0) / (200L * ARM_MICROSTEP_FACTOR * SHOULDER_GEAR_RATIO);
        elbow.stop();
        elbow.setSpeed(0);
        targetAngleElbow = (-elbow.currentPosition() * 360.0) / (200L * ARM_MICROSTEP_FACTOR * ELBOW_GEAR_RATIO);
        currentState = READY;
        Serial.println("System reset");
    }
    else if (cmd == "home") {
        currentState = HOMING;
    }
    else if (cmd.startsWith("move waist")) {
        cmd = cmd.substring(10);
        newTarget = cmd.toFloat();
        currentAngleWaist = (-waist.currentPosition() * 360.0) / (200L * WAIST_MICROSTEP_FACTOR * WAIST_GEAR_RATIO);
        targetAngleWaist = currentAngleWaist + newTarget;
        currentState = MOVING;
        Serial.print("New absolute waist angle: ");
        Serial.println(targetAngleWaist, 2);
    }
    else if (cmd.startsWith("move shoulder")) {
        cmd = cmd.substring(13);
        newTarget = cmd.toFloat();
        currentAngleShoulder = (shoulder.currentPosition() * 360.0) / (200L * ARM_MICROSTEP_FACTOR * SHOULDER_GEAR_RATIO);
        targetAngleShoulder = currentAngleShoulder + newTarget;
        //targetAngleShoulder = constrain(targetAngleShoulder, radToDeg * -theta_offset_1, 0);
        currentState = MOVING;
        Serial.print("New absolute shoulder angle: ");
        Serial.println(targetAngleShoulder, 2);
    }
    else if (cmd.startsWith("move elbow")) {
        cmd = cmd.substring(10);
        newTarget = cmd.toFloat();
        currentAngleElbow = (-elbow.currentPosition() * 360.0) / (200L * ARM_MICROSTEP_FACTOR * ELBOW_GEAR_RATIO);
        //currentAngleShoulder = (shoulder.currentPosition() * 360.0) / (200L * ARM_MICROSTEP_FACTOR * SHOULDER_GEAR_RATIO)
        targetAngleElbow = currentAngleElbow + newTarget;
        //targetAngleElbow = constrain(targetAngleElbow, (-180 + -currentAngleShoulder + (radToDeg * theta_offset_2)), (currentAngleShoulder + (radToDeg * theta_offset_3)))
        currentState = MOVING;
        Serial.print("New absolute elbow angle: ");
        Serial.println(targetAngleElbow, 2);
    }
    else if (cmd.startsWith("point waist")) {
        cmd = cmd.substring(11);
        newTarget = cmd.toFloat();
        currentAngleWaist = (-waist.currentPosition() * 360.0) / (200L * WAIST_MICROSTEP_FACTOR * WAIST_GEAR_RATIO);
        targetAngleWaist = newTarget; // Absolute positioning
        currentState = MOVING;
        Serial.print("Target waist angle set to: ");
        Serial.println(targetAngleWaist, 2);
    }
    else if (cmd.startsWith("point shoulder")) {
        cmd = cmd.substring(14);
        newTarget = cmd.toFloat();
        currentAngleShoulder = (shoulder.currentPosition() * 360.0) / (200L * ARM_MICROSTEP_FACTOR * SHOULDER_GEAR_RATIO);
        targetAngleShoulder = newTarget; // Absolute positioning
        currentState = MOVING;
        Serial.print("Target shoulder angle set to: ");
        Serial.println(targetAngleShoulder, 2);
    }
    else if (cmd.startsWith("point elbow")) {
        cmd = cmd.substring(11);
        newTarget = cmd.toFloat();
        currentAngleElbow = (-elbow.currentPosition() * 360.0) / (200L * ARM_MICROSTEP_FACTOR * ELBOW_GEAR_RATIO);
        targetAngleElbow = newTarget; // Absolute positioning
        currentState = MOVING;
        Serial.print("Target elbow angle set to: ");
        Serial.println(targetAngleElbow, 2);
    }
    else if (cmd.startsWith("inject syringe")) {
        cmd = cmd.substring(13);
        newTarget = abs(cmd.toFloat() / crossSectionArea);
        currentPositionSyringe = //(syringe.currentPosition() * 360.0) / (200L * SYRINGE_MICROSTEP_FACTOR * SYRINGE_GEAR_RATIO);
        targetPositionSyringe = currentPositionSyringe + newTarget;
        currentState = SYRINGE;
        Serial.print("Syringe injecting: ");
        Serial.print(targetPositionSyringe, 2);
        Serial.println(" ul");    
    }
    else if (cmd.startsWith("draw syringe")) {
        cmd = cmd.substring(13);
        newTarget = abs(cmd.toFloat() / crossSectionArea);
        currentPositionSyringe = //(syringe.currentPosition() * 360.0) / (200L * SYRINGE_MICROSTEP_FACTOR * SYRINGE_GEAR_RATIO);
        targetPositionSyringe = currentPositionSyringe - newTarget;
        currentState = SYRINGE;
        Serial.print("Syringe Drawing: ");
        Serial.print(targetPositionSyringe, 2);
        Serial.println(" ul");    
    }
    else if (cmd == "help") {
        currentState = HELP;
    }
    else if (cmd != "") {
        Serial.println("Command not recognised");
    }
}

//********* Stepper Motor Homing Sequence Function *********//

bool homeAxis(AccelStepper &motor, int microSteppingFactor, int limitSwitchPin) {

    switch (phase) {
        case MOVE_AWAY_1:
            motor.setMaxSpeed(200 * microSteppingFactor/16);       // low speed for safety
            motor.setAcceleration(3000 * microSteppingFactor/16);  // high acceleration for near instantaneous stopping
            motor.stop();
            motor.move((-200 * microSteppingFactor/16)); // Move away slightly
            phase = MOVE_TOWARDS_SWITCH;
            break;
        
        case MOVE_TOWARDS_SWITCH:
            if (motor.distanceToGo() == 0) {
            motor.move(50000 * microSteppingFactor/16); // Arbitrary large negative move
            phase = WAIT_FOR_TRIGGER;
            }
            break;

        case WAIT_FOR_TRIGGER:
            // Check if the switch has been pressed
            if (digitalRead(limitSwitchPin) == HIGH) {
                motor.stop();
                motor.move(-200 * microSteppingFactor/16); // Move away slightly
                phase = MOVE_AWAY_2;
            }
            break;

        case MOVE_AWAY_2:
            if (motor.distanceToGo() == 0) {
                motor.setMaxSpeed(100 * microSteppingFactor/16);
                motor.move(50000 * microSteppingFactor/16); // Move slowly back
                phase = FINAL_APPROACH;
            }
            break;

        case FINAL_APPROACH:
            // Check again if the switch has been triggered
            if (digitalRead(limitSwitchPin) == HIGH) {
                motor.stop();
                motor.setCurrentPosition(0);
                motor.setMaxSpeed(1000 * microSteppingFactor/16);        // Revert to default speed
                motor.setAcceleration(500 * microSteppingFactor/16);     // Revert to default acceleration
                phase = MOVE_AWAY_1; // Reset for next motor
                return true; // Homing done
            }
            break;
    }
    motor.run();
    return false; // Still homing
}