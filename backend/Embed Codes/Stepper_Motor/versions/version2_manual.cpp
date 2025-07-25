#include <Arduino.h>
#include "A4988.h"

// Pin Configurations
#define STEP_PIN 3
#define DIR_PIN 2
#define SLEEP_PIN 4
#define MS1_PIN 7
#define MS2_PIN 6
#define MS3_PIN 5

// Motor and Driver Setup
const int STEPS_PER_REV = 200;   // NEMA17 typically has 200 steps/rev
int RPM = 100;
int MICROSTEPS = 1;              // Change to 2, 4, 8, 16 if needed

A4988 stepper(STEPS_PER_REV, DIR_PIN, STEP_PIN, MS1_PIN, MS2_PIN, MS3_PIN);

void setup() {
  Serial.begin(9600);
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, HIGH); // Wake up driver
  stepper.begin(RPM, MICROSTEPS);
  Serial.println("Stepper initialized. Ready to rotate.");
}

void loop() {
  // Example calls
  rotateAngle(45);   // rotate CW by 45 degrees
  delay(1000);
  rotateAngle(-90);  // rotate CCW by 90 degrees
  delay(2000);

  // Loop only once
  while (true);
}

// 🔄 Function: Rotate motor by desired angle (+CW, -CCW)
void rotateAngle(float angleDegrees) {
  long stepsToMove = (long)((angleDegrees / 360.0) * STEPS_PER_REV * MICROSTEPS);
  Serial.print("Rotating ");
  Serial.print(angleDegrees);
  Serial.print("° => ");
  Serial.print(stepsToMove);
  Serial.println(" steps");
  
  stepper.move(stepsToMove);
}
