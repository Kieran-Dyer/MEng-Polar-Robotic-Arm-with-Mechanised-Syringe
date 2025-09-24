#include <Stepper.h>

// Constants
const int Steps = 2048; // Number of steps for 28BYJ-48 motor
const int PinCLK = 2;   // Generating interrupts using CLK signal
const int PinDT = 3;    // Reading DT signal
const int PinSW = 4;    // Reading Push Button switch

// Variables
volatile int RotaryPosition = 0;    // To store Rotary Encoder's Position
int lastCLK = LOW;
int currentCLK;
int currentDT;

int stepsToMove = 0;

// Setup stepper motor
Stepper stepper(Steps, 8, 10, 9, 11);  // Pins for 28BYJ-48 motor

// Interrupt service routine for rotary encoder
void doEncoder() {
  currentCLK = digitalRead(PinCLK);
  currentDT = digitalRead(PinDT);

  if (currentCLK != lastCLK) {
    if (currentDT != currentCLK) {
      RotaryPosition++;
    } else {
      RotaryPosition--;
    }

    // Ensure RotaryPosition is within bounds -19 to 20
    if (RotaryPosition > 20) {
      RotaryPosition = -19;
    } else if (RotaryPosition < -19) {
      RotaryPosition = 20;
    }
  }

  lastCLK = currentCLK;
}

void updateSpeed(int position) {
  int speed;

  if (position > 0 && position <= 19) {
    speed = map(position, 1, 19, 1, 19); // Mapping positive positions to speed 1-19
  } else if (position < 0 && position >= -19) {
    speed = map(position, -19, -1, -19, -1); // Mapping negative positions to speed -19 to -1
  } else if (position == 20 || position == 0) {
    speed = 0;
  }

  Serial.print("speed: ");
  Serial.println(speed);

  if (speed != 0) {
    stepper.setSpeed(abs(speed)); // Set speed with absolute value

    stepsToMove = (position > 0) ? abs(position) * 4 : -abs(position) * 4;
    stepper.step(stepsToMove);
  }
}

void setup() {
  // Setup Serial Monitor
  Serial.begin(9600);

  // Setup input pins for the rotary encoder
  pinMode(PinCLK, INPUT);
  pinMode(PinDT, INPUT);
  pinMode(PinSW, INPUT_PULLUP);

  // Attach interrupt to the CLK pin
  attachInterrupt(digitalPinToInterrupt(PinCLK), doEncoder, CHANGE);

  // Initialize Rotary Encoder position
  RotaryPosition = -1;
}

void loop() {
  // Print the current RotaryPosition and speed to the Serial Monitor
  Serial.print("Rotary Position: ");
  Serial.println(RotaryPosition);
  
  // Update speed based on encoder position
  updateSpeed(RotaryPosition);


  if (RotaryPosition == 0 || RotaryPosition == 20) {
    delay(100);
  }

}
