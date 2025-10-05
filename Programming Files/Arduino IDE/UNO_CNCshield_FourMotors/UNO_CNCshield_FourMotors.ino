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
#define SHOULDER_GEAR_RATIO 50.0     // Shoulder gear ratio multiplier 60:12 * 10:1 = 
#define ELBOW_GEAR_RATIO 50.0        // Elbow gear ratio multiplier 60:12 * 10:1 = 
#define SYRINGE_GEAR_RATIO 8        // Syringe threaded rod gear ratio XX:XX

#define ARM_MICROSTEP_FACTOR 2     // Microstepping factor for shoulder and elbow
#define WAIST_MICROSTEP_FACTOR 16   // Microstepping factor for waist
#define SYRINGE_MICROSTEP_FACTOR 16  // Microstepping factor for Syringe

// State Definitions
enum State {
    READY,          // System is idle and ready for commands
    HOMING,         // System is homing (not yet implemented)
    IVK_PROCESSING, // Inverse Kinematics processing (not yet implemented)
    MOVING,         // Motors are moving to target angles
    SYRINGE,        // Move the syringe motor to inject or draw the desired volume
    SYRINGE_MIX,    // Syringe mixing protocol
};

State currentState;  // Tracks the current state of the system

enum HomingStep {
    HOMING_SHOULDER,  // Move the Shoulder first
    HOMING_ELBOW,     // Move the Elbow next
    HOMING_WAIST,     // Move the Waist next
    HOMING_SYRINGE,   // Move the syring last
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

HomingPhase phase = MOVE_AWAY_1; // Tracks phase of the homing routine

enum MixPhase {
    MIX_DRAW,
    MIX_INJECT,
    MIX_DONE
};

MixPhase mixPhase = MIX_DRAW; // Tracks phase of the mixing routine

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

float volume = 0.0;
float syringeDiameter = 4.75;     // Cross sectional area of the syringe in mm
float syringeMaxDrawVol = 1000;   // maximum capacity in uL (example: 1000 = 1 mL)
float crossSectionArea = square(syringeDiameter/2.0) * M_PI;  // Area of a circle A = pi*(r^2)
float syringeMaxDrawMM = syringeMaxDrawVol / crossSectionArea; // max travel in mm (volume / area = distance)

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

// Position adjustment variables
float cal_x = 0;
float cal_y = 0;
float cal_z = 0;

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

// ------- Synchronous Joint Interpolation -------
// (stopgap solution to smooth the output motion, but doesn't create linear pathing)

// Step Targets
long targetStepsWaist    = 0;
long targetStepsShoulder = 0;
long targetStepsElbow    = 0;

// Distances
float distWaist    = 0;
float distShoulder = 0;
float distElbow    = 0;

// Times
float timeWaist    = 0;
float timeShoulder = 0;
float timeElbow    = 0;

// Master time
float masterTime   = 0;

// Ratios
float waistRatio    = 1.0; 
float shoulderRatio = 1.0; 
float elbowRatio    = 1.0;

// Default speeds and accelerations (mirrors setup values)
const float defaultWaistSpeed   = 1000.0 * WAIST_MICROSTEP_FACTOR / 16;
const float defaultWaistAccel   = 500.0  * WAIST_MICROSTEP_FACTOR / 16;

const float defaultShoulderSpeed = 1000.0 * ARM_MICROSTEP_FACTOR / 2;
const float defaultShoulderAccel = 500.0  * ARM_MICROSTEP_FACTOR / 2;

const float defaultElbowSpeed   = 1000.0 * ARM_MICROSTEP_FACTOR / 2;
const float defaultElbowAccel   = 500.0  * ARM_MICROSTEP_FACTOR / 2;

// ------- Syringe Mixing Variables -------
int timePos = 0;

long mixCyclesRequested = 1;
long mixCyclesCompleted = 0;
long mixDrawSteps = 0;

// ------- Calibration Variables -------
// calibration mode flags
bool calibrationFlag = false;
bool calInputFlag = true;

// IVK adjustment variables
bool ch_cal_x = false;
bool ch_cal_y = false;
bool ch_cal_z = false;

float tmp_cal_x = 0;
float tmp_cal_y = 0;
float tmp_cal_z = 0;

//syringe adjustment variables
bool ch_syringeDiameter = false;
bool ch_syringeMaxDrawVol = false;

float tmp_syringeDiameter = 0.0;
float tmp_syringeMaxDrawVol = 0.0;

// ------- Other -------

// Command string
String command;

void setup() {
    Serial.begin(9600);

    // Configure motor settings
    waist.setMaxSpeed(1000 * WAIST_MICROSTEP_FACTOR/16);
    waist.setAcceleration(500 * WAIST_MICROSTEP_FACTOR/16);
    waist.setCurrentPosition(0);

    shoulder.setMaxSpeed(1000 * ARM_MICROSTEP_FACTOR/2);
    shoulder.setAcceleration(500 * ARM_MICROSTEP_FACTOR/2);
    shoulder.setCurrentPosition(0);

    elbow.setMaxSpeed(1000 * ARM_MICROSTEP_FACTOR/2);
    elbow.setAcceleration(500 * ARM_MICROSTEP_FACTOR/2);
    elbow.setCurrentPosition(0);

    syringe.setMaxSpeed(3000 * SYRINGE_MICROSTEP_FACTOR/16);
    syringe.setAcceleration(500 * SYRINGE_MICROSTEP_FACTOR/16);
    syringe.setCurrentPosition(0);

    // Configure the limit switch pins
    pinMode(WAIST_LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(SHOULDER_LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(ELBOW_LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(SYRINGE_LIMIT_SWITCH_PIN, INPUT_PULLUP);

//clear variables
    emergencyStop = false;
    ivkFlag = false;
    command = "";
    currentState = READY;
    homingStep = HOMING_SHOULDER;
    phase = MOVE_AWAY_1;

    Serial.println(F(""));
    Serial.println(F("System Ready. Type 'help' for available commands."));
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

                Serial.print(F("Moving to ("));
                Serial.print(X_pos, 2);
                Serial.print(F(", "));
                Serial.print(Y_pos, 2);
                Serial.print(F(", "));
                Serial.print(Z_pos, 2);
                Serial.println(F(")"));

                currentState = IVK_PROCESSING;
            } else {
                Serial.println(F("Invalid format. Use: goto (X, Y, Z)"));
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
                        if (homeAxis(shoulder, "shoulder", ARM_MICROSTEP_FACTOR, SHOULDER_LIMIT_SWITCH_PIN)) {
                            homingStep = HOMING_ELBOW;
                        }
                        break;
                    case HOMING_ELBOW:
                        if (homeAxis(elbow, "elbow", ARM_MICROSTEP_FACTOR, ELBOW_LIMIT_SWITCH_PIN)) {
                            homingStep = HOMING_WAIST;
                        }
                        break;
                    case HOMING_WAIST:
                        if (homeAxis(waist, "waist", WAIST_MICROSTEP_FACTOR, WAIST_LIMIT_SWITCH_PIN)) {
                            homingStep = HOMING_SYRINGE;
                        }
                        break;
                    case HOMING_SYRINGE:
                        if (homeAxis(syringe, "syringe", SYRINGE_MICROSTEP_FACTOR, SYRINGE_LIMIT_SWITCH_PIN)) {
                            homingStep = HOMING_COMPLETE;
                        }
                        break;
                    case HOMING_COMPLETE:
                        Serial.println(F("Homing Sequence Complete"));

                        //reset the position pointers
                        targetAngleWaist = 0;
                        targetAngleShoulder = 0;
                        targetAngleElbow = 0;
                        targetPositionSyringe = 0;

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
                // Convert angles to stepper positions
                targetStepsWaist = ((-targetAngleWaist * WAIST_GEAR_RATIO) * (200L * WAIST_MICROSTEP_FACTOR)) / 360L; 
                targetStepsShoulder = ((targetAngleShoulder * SHOULDER_GEAR_RATIO) * (200L * ARM_MICROSTEP_FACTOR)) / 360L; 
                targetStepsElbow = ((-targetAngleElbow * ELBOW_GEAR_RATIO) * (200L * ARM_MICROSTEP_FACTOR)) / 360L; 

                // Distances to travel
                distWaist    = abs(targetStepsWaist - waist.currentPosition()); 
                distShoulder = abs(targetStepsShoulder - shoulder.currentPosition()); 
                distElbow    = abs(targetStepsElbow - elbow.currentPosition()); 

                // Time at default speeds (t = d / v)
                timeWaist    = (distWaist    > 0) ? (float)distWaist    / defaultWaistSpeed    : 0; 
                timeShoulder = (distShoulder > 0) ? (float)distShoulder / defaultShoulderSpeed : 0; 
                timeElbow    = (distElbow    > 0) ? (float)distElbow    / defaultElbowSpeed    : 0; 

                // Master time is the slowest axis
                masterTime = max(timeWaist, max(timeShoulder, timeElbow)); 

                // Early exit: nothing to do
                if (masterTime <= 0) {
                    Serial.println(F("No arm movement required (targets already reached)."));
                    currentState = READY;
                    break;
                } else {
                    // Scale speeds and accelerations
                    waistRatio    = (timeWaist    > 0) ? (timeWaist    / masterTime) : 0; 
                    shoulderRatio = (timeShoulder > 0) ? (timeShoulder / masterTime) : 0; 
                    elbowRatio    = (timeElbow    > 0) ? (timeElbow    / masterTime) : 0; 

                    waist.setMaxSpeed(defaultWaistSpeed * waistRatio);
                    waist.setAcceleration(defaultWaistAccel * waistRatio);
                    waist.moveTo(targetStepsWaist);

                    shoulder.setMaxSpeed(defaultShoulderSpeed * shoulderRatio);
                    shoulder.setAcceleration(defaultShoulderAccel * shoulderRatio);
                    shoulder.moveTo(targetStepsShoulder);

                    elbow.setMaxSpeed(defaultElbowSpeed * elbowRatio);
                    elbow.setAcceleration(defaultElbowAccel * elbowRatio);
                    elbow.moveTo(targetStepsElbow);
                }

                // Run motors
                waist.run();
                shoulder.run();
                elbow.run();

                // Exit when all motors are at target
                if (waist.distanceToGo() == 0 && shoulder.distanceToGo() == 0 && elbow.distanceToGo() == 0) {
                    Serial.println(F("Arm movement complete"));

                    // Reset to defaults
                    waist.setMaxSpeed(defaultWaistSpeed);
                    waist.setAcceleration(defaultWaistAccel);

                    shoulder.setMaxSpeed(defaultShoulderSpeed);
                    shoulder.setAcceleration(defaultShoulderAccel);

                    elbow.setMaxSpeed(defaultElbowSpeed);
                    elbow.setAcceleration(defaultElbowAccel);

                    currentState = READY;
                }
                break;
            case SYRINGE:
                // Move the syinge to inject or draw the desired amount
                syringe.moveTo((targetPositionSyringe / SYRINGE_GEAR_RATIO) * (200L * SYRINGE_MICROSTEP_FACTOR));
                syringe.run();
                // check if movement is complete
                if (syringe.distanceToGo() == 0) {
                    Serial.println(F("Syringe movement complete"));
                    currentState = READY;
                }
                break;
            case SYRINGE_MIX:
                if (mixPhase == MIX_DRAW && syringe.distanceToGo() == 0 && mixCyclesCompleted == 0) {
                    syringe.moveTo(mixDrawSteps);
                }

                if(mixSyringe()) {
                    Serial.println(F("Syringe mixing complete"));
                    mixPhase = MIX_DRAW;
                    mixCyclesCompleted = 0;
                    currentState = READY;
                }
                break;
        }
    }
}

//********* Input Serial Command Parser w/ target angle calculations *********//

void processCommand(String cmd) {
    cmd.trim();
    
    if (cmd != "") { // have the command not recognised statement at the top of the else if hierachy so it doesn't exit early when checking calibrationFlag
        Serial.println(F("Command not recognised"));
    }
    else if (!calibrationFlag) { // Allows movement when not in callibration mode
        
        if (cmd == "!") {
            // Emergency stop all motors
            emergencyStop = true;
            waist.stop();
            shoulder.stop();
            elbow.stop();
            Serial.println(F("EMERGENCY STOP ACTIVATED"));
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
            Serial.println(F("System reset"));
        }
        else if (cmd == "home") {
            currentState = HOMING;
        }
        else if (cmd == "calibration") {
            calibrationFlag = true;
            calInputFlag   = true;
            Serial.println(F("--- Entered Calibration mode ---"));
            Serial.println(F("Use 'x <val>', 'y <val>', 'z <val>', 'syringeDiameter <val>', 'syringeMaxDrawVol <val>"));
            Serial.println(F("Use 'confirm' to review changes, or 'cancel' to exit calibration mode."));
        }
        else if (cmd.startsWith("move waist")) {
            cmd = cmd.substring(10);
            newTarget = cmd.toFloat();
            currentAngleWaist = (-waist.currentPosition() * 360.0) / (200L * WAIST_MICROSTEP_FACTOR * WAIST_GEAR_RATIO);
            targetAngleWaist = currentAngleWaist + newTarget;
            currentState = MOVING;
            Serial.print(F("New absolute waist angle: "));
            Serial.println(targetAngleWaist, 2);
        }
        else if (cmd.startsWith("move shoulder")) {
            cmd = cmd.substring(13);
            newTarget = cmd.toFloat();
            currentAngleShoulder = (shoulder.currentPosition() * 360.0) / (200L * ARM_MICROSTEP_FACTOR * SHOULDER_GEAR_RATIO);
            targetAngleShoulder = currentAngleShoulder + newTarget;
            //targetAngleShoulder = constrain(targetAngleShoulder, radToDeg * -theta_offset_1, 0);
            currentState = MOVING;
            Serial.print(F("New absolute shoulder angle: "));
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
            Serial.print(F("New absolute elbow angle: "));
            Serial.println(targetAngleElbow, 2);
        }
        else if (cmd.startsWith("point waist")) {
            cmd = cmd.substring(11);
            newTarget = cmd.toFloat();
            currentAngleWaist = (-waist.currentPosition() * 360.0) / (200L * WAIST_MICROSTEP_FACTOR * WAIST_GEAR_RATIO);
            targetAngleWaist = newTarget; // Absolute positioning
            currentState = MOVING;
            Serial.print(F("Target waist angle set to: "));
            Serial.println(targetAngleWaist, 2);
        }
        else if (cmd.startsWith("point shoulder")) {
            cmd = cmd.substring(14);
            newTarget = cmd.toFloat();
            currentAngleShoulder = (shoulder.currentPosition() * 360.0) / (200L * ARM_MICROSTEP_FACTOR * SHOULDER_GEAR_RATIO);
            targetAngleShoulder = newTarget; // Absolute positioning
            currentState = MOVING;
            Serial.print(F("Target shoulder angle set to: "));
            Serial.println(targetAngleShoulder, 2);
        }
        else if (cmd.startsWith("point elbow")) {
            cmd = cmd.substring(11);
            newTarget = cmd.toFloat();
            currentAngleElbow = (-elbow.currentPosition() * 360.0) / (200L * ARM_MICROSTEP_FACTOR * ELBOW_GEAR_RATIO);
            targetAngleElbow = newTarget; // Absolute positioning
            currentState = MOVING;
            Serial.print(F("Target elbow angle set to: "));
            Serial.println(targetAngleElbow, 2);
        }
        else if (cmd.startsWith("inject syringe")) {
            cmd = cmd.substring(15);
            volume = abs(cmd.toFloat());
            newTarget = volume / crossSectionArea;
            currentPositionSyringe = (syringe.currentPosition() * SYRINGE_GEAR_RATIO) / (200L * SYRINGE_MICROSTEP_FACTOR);

            targetPositionSyringe = currentPositionSyringe - newTarget;

            if(targetPositionSyringe < 0) {
                targetPositionSyringe = 0;
                Serial.println(F("WARNING: Command clamped to 0 (syringe lower limit)."));
            } else if (targetPositionSyringe > syringeMaxDrawMM) {
                targetPositionSyringe = syringeMaxDrawMM;
                Serial.println(F("WARNING: Command clamped to max (syringe upper limit)."));
            }

            currentState = SYRINGE;

            Serial.print(F("Syringe injecting to: "));
            Serial.print(volume, 2);
            Serial.print(F(" uL : "));
            Serial.print(targetPositionSyringe, 2);
            Serial.println(F(" mm"));
        }
        else if (cmd.startsWith("draw syringe")) {
            cmd = cmd.substring(13);
            volume = abs(cmd.toFloat());
            newTarget = volume / crossSectionArea;
            currentPositionSyringe = (syringe.currentPosition() * SYRINGE_GEAR_RATIO) / (200L * SYRINGE_MICROSTEP_FACTOR);

            targetPositionSyringe = currentPositionSyringe + newTarget;

            if(targetPositionSyringe < 0) {
                targetPositionSyringe = 0;
                Serial.println(F("WARNING: Command clamped to 0 (syringe lower limit)."));
            } else if (targetPositionSyringe > syringeMaxDrawMM) {
                targetPositionSyringe = syringeMaxDrawMM;
                Serial.println(F("WARNING: Command clamped to max (syringe upper limit)."));
            }

            currentState = SYRINGE;

            Serial.print(F("Syringe drawing to: "));
            Serial.print(volume, 2);
            Serial.print(F(" uL : "));
            Serial.print(targetPositionSyringe, 2);
            Serial.println(F(" mm"));  
        }
        else if (cmd.startsWith("mix syringe ")) {
            timePos = cmd.indexOf(" times ", 12);
            if (timePos > 0) {
                volume = abs(cmd.substring(12, timePos).toFloat());
                if (volume > syringeMaxDrawVol) volume = syringeMaxDrawVol;

                mixCyclesRequested = cmd.substring(timePos + 7).toInt();
                if (mixCyclesRequested < 1) mixCyclesRequested = 1;

                mixDrawSteps = ((volume / crossSectionArea) / SYRINGE_GEAR_RATIO) * (200L * SYRINGE_MICROSTEP_FACTOR);

                currentState = SYRINGE_MIX;

                Serial.print(F("Mixing syringe: "));
                Serial.print(volume, 1);
                Serial.print(F(" uL * "));
                Serial.print(mixCyclesRequested);
                Serial.print(F(" cycles ("));
                Serial.print(volume / crossSectionArea, 2);
                Serial.println(F(" mm travel per half-cycle)"));
            }
            else if (cmd == "help") {
                // Display available commands
                Serial.println(F("Available commands:"));
                Serial.println(F("! - Emergency Stop"));
                Serial.println(F("reset - Reset system"));
                Serial.println(F("home - home the system"));
                Serial.println(F("move <motor> <angle> - Move motor by angle"));
                Serial.println(F("point <motor> <angle> - Point motor to relative angle"));
                Serial.println(F("Available motors: waist, shoulder, elbow"));
                Serial.println(F("<action> syringe <volume> - inject or draw the sringe by a volume in microlitres"));
                Serial.println(F("mix syringe <volume> times <integer> - repeatedly draw and inject the sringe by a volume in microlitres"));
                Serial.println(F("goto (X, Y, Z) - move the end effector to a cartesian coordinate relative to (0, 0, 0)"));
            }
        }
        // ------- End of the Normal Commands -------
    }
    else if (calibrationFlag) { // prevents movement inputs being processed in calibration mode.

        if (calInputFlag) {
            // handle <axis> <value>, confirm, cancel
            if (cmd.startsWith("x ")) {
                tmp_cal_x = cmd.substring(2).toFloat();
                ch_cal_x = true;
                Serial.print(F("X set to "));
                Serial.println(tmp_cal_x, 3);
            }
            else if (cmd.startsWith("y ")) {
                tmp_cal_y = cmd.substring(2).toFloat();
                ch_cal_y = true;
                Serial.print(F("Y set to "));
                Serial.println(tmp_cal_y, 3);
            }
            else if (cmd.startsWith("z ")) {
                tmp_cal_z = cmd.substring(2).toFloat();
                ch_cal_z = true;
                Serial.print(F("Z set to "));
                Serial.println(tmp_cal_z, 3);
            }
            else if (cmd.startsWith("syringeDiameter ")) {
                tmp_syringeDiameter = cmd.substring(16).toFloat();
                ch_syringeDiameter = true;
                Serial.print(F("syringeDiameter set to "));
                Serial.print(tmp_syringeDiameter, 2);
            }
            else if (cmd.startsWith("syringeMaxDrawVol ")) {
                tmp_syringeMaxDrawVol = cmd.substring(18).toFloat();
                ch_syringeMaxDrawVol = true;
                Serial.print(F("syringeMaxDrawVol set to "));
                Serial.println(tmp_syringeMaxDrawVol, 2);
            }
            else if (cmd == "confirm") {
                calInputFlag = false;
                Serial.println(F("--- Review Pending Changes ---"));
                if (ch_cal_x) {
                    Serial.print(F("X set to "));
                    Serial.println(tmp_cal_x, 3);
                }
                if (ch_cal_y) {
                    Serial.print(F("Y set to "));
                    Serial.println(tmp_cal_y, 3);
                }
                if (ch_cal_z) {
                    Serial.print(F("Z set to "));
                    Serial.println(tmp_cal_z, 3);
                }
                if (ch_syringeDiameter) {
                    Serial.print(F("syringeDiameter set to "));
                    Serial.println(tmp_syringeDiameter, 2);
                }
                if (ch_syringeMaxDrawVol) {
                    Serial.print(F("syringeMaxDrawVol set to "));
                    Serial.println(tmp_syringeMaxDrawVol, 2);
                }
                Serial.println(F("Type 'ok' to apply or 'cancel' to re-enter values."));
            }
            else if (cmd == "cancel") {
                calibrationFlag = false;
                Serial.println(F("Calibration cancelled."));
            }
            else if (cmd == "help") {
                Serial.println(F("Calibration mode commands:"));
                Serial.println(F("x <distance>, y <distance>, z <distance>"));
                Serial.println(F("syringeDiameter <distance>, syringeMaxDrawVol <volume>"));
                Serial.println(F("confirm - review changes"));
                Serial.println(F("cancel - exit calibration"));
            }
        }
        else if (!calInputFlag) {
            // confirmation stage
            if (cmd == "ok") {
                if (ch_cal_x) {
                    cal_x = tmp_cal_x;
                    ch_cal_x = false;
                }
                if (ch_cal_y) {
                    cal_y = tmp_cal_y;
                    ch_cal_y = false;
                }
                if (ch_cal_z) {
                    cal_z = tmp_cal_z;
                    ch_cal_z = false;
                }
                if (ch_syringeDiameter) {
                    syringeDiameter = tmp_syringeDiameter;
                    ch_syringeDiameter = false;
                }
                if (ch_syringeMaxDrawVol) {
                    syringeMaxDrawVol = tmp_syringeMaxDrawVol;
                    ch_syringeMaxDrawVol = false;
                }

                crossSectionArea = square(syringeDiameter / 2.0) * M_PI;
                syringeMaxDrawMM = syringeMaxDrawVol / crossSectionArea;

                calibrationFlag = false;
                Serial.println(F("Calibration applied."));
            }
            else if (cmd == "cancel") {
                calInputFlag = true;
                Serial.println(F("Back to input stage."));
            }
                            // Display available commands
                Serial.println(F("Calibration mode commands:"));
                Serial.println(F("ok - accept changes"));
                Serial.println(F("cancel - return to calibration"));
        }
        // ------- End of the Calibration mode -------
    }
}

//********* Stepper Motor Homing Sequence Function *********//

bool homeAxis(AccelStepper &motor, String Name, int microSteppingFactor, int limitSwitchPin) {

    switch (phase) {
        case MOVE_AWAY_1:
            if (Name == "waist")
            {
                motor.setMaxSpeed(200 * microSteppingFactor/16);       // low speed for safety
                motor.setAcceleration(3000 * microSteppingFactor/16);  // high acceleration for near instantaneous stopping
                motor.stop();
                motor.move((-200 * microSteppingFactor/16)); // Move away slightly
            } else if (Name == "syringe"){
                motor.setMaxSpeed(400 * microSteppingFactor/16);       // low speed for safety
                motor.setAcceleration(3000 * microSteppingFactor/16);  // high acceleration for near instantaneous stopping
                motor.stop();
                motor.move((400 * microSteppingFactor/16)); // Move away slightly
            } else {
                motor.setMaxSpeed(200 * microSteppingFactor/2);       // low speed for safety
                motor.setAcceleration(3000 * microSteppingFactor/2);  // high acceleration for near instantaneous stopping
                motor.stop();
                motor.move((-200 * microSteppingFactor/2)); // Move away slightly
            }
            phase = MOVE_TOWARDS_SWITCH;
            break;        
        case MOVE_TOWARDS_SWITCH:
            if (motor.distanceToGo() == 0) {
                if (Name == "waist"){
                    motor.move(50000 * microSteppingFactor/16); // Arbitrary large negative move
                } else if (Name == "syringe") {
                    motor.move(-50000 * microSteppingFactor/16); // Arbitrary large negative move
                } else {
                    motor.move(50000 * microSteppingFactor/2); // Arbitrary large negative move
                }
            phase = WAIT_FOR_TRIGGER;
            }
            break;

        case WAIT_FOR_TRIGGER:
            // Check if the switch has been pressed
            if (digitalRead(limitSwitchPin) == HIGH) {
                motor.stop();
                if (Name == "waist"){
                    motor.move(-200 * microSteppingFactor/16); // Move away slightly
                } else if (Name == "syringe"){
                    motor.move(400 * microSteppingFactor/16); // Move away slightly
                } else {
                    motor.move(-200 * microSteppingFactor/2); // Move away slightly
                }
                phase = MOVE_AWAY_2;
            }
            break;

        case MOVE_AWAY_2:
            if (motor.distanceToGo() == 0) {
                if (Name == "waist") {
                    motor.setMaxSpeed(100 * microSteppingFactor/16);
                    motor.move(50000 * microSteppingFactor/16); // Move slowly back
                } else if (Name == "syringe"){
                    motor.setMaxSpeed(200 * microSteppingFactor/16);
                    motor.move(-50000 * microSteppingFactor/16); // Move slowly back
                } else {
                    motor.setMaxSpeed(100 * microSteppingFactor/2);
                    motor.move(50000 * microSteppingFactor/2); // Move slowly back
                }
                phase = FINAL_APPROACH;
            }
            break;
        case FINAL_APPROACH:
            // Check again if the switch has been triggered
            if (digitalRead(limitSwitchPin) == HIGH) {
                motor.stop();
                if (Name == "waist") {
                    motor.setCurrentPosition(0);
                    motor.setMaxSpeed(1000 * microSteppingFactor/16);        // Revert to default speed
                    motor.setAcceleration(500 * microSteppingFactor/16);     // Revert to default acceleration
                } else if (Name == "syringe") {
                    motor.setCurrentPosition(0);
                    motor.setMaxSpeed(3000 * microSteppingFactor/16);        // Revert to default speed
                    motor.setAcceleration(500 * microSteppingFactor/16);     // Revert to default acceleration
                } else {
                    motor.setCurrentPosition(0);
                    motor.setMaxSpeed(1000 * microSteppingFactor/2);        // Revert to default speed
                    motor.setAcceleration(500 * microSteppingFactor/2);     // Revert to default acceleration
                }
                phase = MOVE_AWAY_1; // Reset for next motor
                return true; // Homing done
            }
            break;
    }
    motor.run();
    return false; // Still homing
}

bool mixSyringe() {
    switch (mixPhase) {
        case MIX_DRAW:
            if (syringe.distanceToGo() == 0) {
                syringe.moveTo(0);
                mixPhase = MIX_INJECT;
            }
            break;
        case MIX_INJECT:
            if (syringe.distanceToGo() == 0 ) {
                mixCyclesCompleted++;
                if (mixCyclesCompleted >= mixCyclesRequested) {
                    mixPhase = MIX_DONE;
                } else {
                    syringe.moveTo(mixDrawSteps);
                    mixPhase = MIX_DRAW;
                }
            }
            break;
        case MIX_DONE:
            syringe.stop();
            mixPhase = MIX_DRAW; // Reset for next mix
            return true; // Mixing done
    }
    syringe.run();
    return false; // Still mixing
}