#include <Arduino.h>

// ── X-Axis Pin Config ──────────────────────────
#define STEP_PIN    3
#define DIR_PIN     2
#define SLEEP_PIN   4
#define MS1_PIN     7
#define MS2_PIN     6
#define MS3_PIN     5

// ── Y-Axis Pin Config ──────────────────────────
#define Y_STEP_PIN  9
#define Y_DIR_PIN   8
#define Y_SLEEP_PIN 10
#define Y_MS1_PIN   12
#define Y_MS2_PIN   11
#define Y_MS3_PIN   A0

// ── Motion Parameters ───────────────────────────
const int  STEPS_PER_REV     = 200;
const int  MICROSTEPS        = 16;
const long STEP_DELAY_US     = 500L;
const float Y_GEAR_RATIO     = 3.75;  // 16T : 60T pulley

// ── State & Mode ────────────────────────────────
char        modeX            = 'X';
char        modeY            = 'X';
float       currentAngleX    = 0.0;
float       currentAngleY    = 0.0;
const float ANGLE_STEP       = 90.0;
unsigned long lastModeChange = 0;

// ── Hardcoded Coordinates for LatLong Mode ─────
/* float lat1 = 22.670227;  // Kolkata
float lon1 = 88.442940;
float lat2 = 21.93938;  // New Delhi
float lon2 = 88.89262; */

float lat1 = 22.670227;  // Kolkata
float lon1 = 88.442940;
float lat2 = 23.6102;  // New Delhi
float lon2 = 85.2799; 

// ── Setup ───────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // X-axis setup
  pinMode(SLEEP_PIN, OUTPUT); digitalWrite(SLEEP_PIN, HIGH);
  pinMode(DIR_PIN, OUTPUT);   pinMode(STEP_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);   pinMode(MS2_PIN, OUTPUT);   pinMode(MS3_PIN, OUTPUT);
  digitalWrite(MS1_PIN, HIGH); digitalWrite(MS2_PIN, HIGH); digitalWrite(MS3_PIN, HIGH);

  // Y-axis setup
  pinMode(Y_SLEEP_PIN, OUTPUT); digitalWrite(Y_SLEEP_PIN, HIGH);
  pinMode(Y_DIR_PIN, OUTPUT);   pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_MS1_PIN, OUTPUT);   pinMode(Y_MS2_PIN, OUTPUT);   pinMode(Y_MS3_PIN, OUTPUT);
  digitalWrite(Y_MS1_PIN, HIGH); digitalWrite(Y_MS2_PIN, HIGH); digitalWrite(Y_MS3_PIN, HIGH);

  lastModeChange = millis();

  Serial.println(F("Ready. Commands: C/A/R/N... for X | CY/AY/RY/NY... for Y | T for LatLong Mode"));
}

// ── Pulse Generator ─────────────────────────────
void stepPulse(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

// ── Step-Angle Conversion — X & Y ───────────────
void angleMoveX(float angleDeg) {
  long steps = lround(fabs(angleDeg) / 360.0 * STEPS_PER_REV * MICROSTEPS);
  digitalWrite(DIR_PIN, (angleDeg >= 0) ? HIGH : LOW);
  for (long i = 0; i < steps; i++) stepPulse(STEP_PIN);
  delay(500);
}

void angleMoveY(float angleDeg) {
  long steps = lround(fabs(angleDeg) / 360.0 * STEPS_PER_REV * MICROSTEPS * (Y_GEAR_RATIO/8));
  digitalWrite(Y_DIR_PIN, (angleDeg >= 0) ? HIGH : LOW);
  for (long i = 0; i < steps; i++) stepPulse(Y_STEP_PIN);
  delay(500);
}

// ── Heading Handling — X & Y ────────────────────
float headingToAngle(String cmd) {
  if      (cmd == "N")  return 0;
  else if (cmd == "NE") return 315;
  else if (cmd == "E")  return 270;
  else if (cmd == "SE") return 225;
  else if (cmd == "S")  return 180;
  else if (cmd == "SW") return 135;
  else if (cmd == "W")  return 90;
  else if (cmd == "NW") return 45;
  else return NAN;
}

void rotateXToHeading(String cmd) {
  float target = headingToAngle(cmd);
  if (isnan(target)) return;

  float diff = target - currentAngleX;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  angleMoveX(diff);
  currentAngleX = target;
}

void rotateYToHeading(String cmd) {
  float target = headingToAngle(cmd);
  if (isnan(target)) return;

  float diff = target - currentAngleY;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  angleMoveY(diff);
  currentAngleY = target;
}

// ── Reset Functions ─────────────────────────────
void resetX() {
  angleMoveX(-currentAngleX);
  currentAngleX = 0.0;
  modeX = 'X';
  rotateXToHeading("N");
  lastModeChange = millis();
  Serial.println(F("X Reset complete."));
}

void resetY() {
  angleMoveY(-currentAngleY);
  currentAngleY = 0.0;
  modeY = 'X';
  rotateYToHeading("N");
  lastModeChange = millis();
  Serial.println(F("Y Reset complete."));
}

// ── LatLong Mode Function ───────────────────────
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  lat1 = radians(lat1); lon1 = radians(lon1);
  lat2 = radians(lat2); lon2 = radians(lon2);
  float dLon = lon2 - lon1;
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  float brng = atan2(y, x);
  brng = degrees(brng);
  return fmod((brng + 360.0), 360.0);  // Normalize to 0–360
}

void rotateXToLatLongBearing() {
  float bearing = calculateBearing(lat1, lon1, lat2, lon2);

  // Mirror bearing to align with your physical frame logic
  bearing = fmod((360.0 - bearing), 360.0);

  float diff = bearing - currentAngleX;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  Serial.print(F("LatLong Mode: Mirrored Bearing "));
  Serial.print(bearing, 2);
  Serial.print(F("°, rotating by "));
  Serial.print(diff, 2);
  Serial.println(F("° on X-axis"));

  angleMoveX(diff);
  currentAngleX = bearing;
  modeX = 'X';
}

// ── Main Loop ───────────────────────────────────
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    bool isNumeric = true;
    for (int i = 0; i < cmd.length(); i++) {
      char c = cmd.charAt(i);
      if (!isDigit(c) && c != '-' && c != '+') {
        isNumeric = false;
        break;
      }
    }

    if (isNumeric && cmd.length() > 0) {
      float targetAngle = fmod((cmd.toFloat() + 360.0), 360.0);  // Normalize to 0–359

      // Acceptable ranges:
      bool inRange = (
        (targetAngle >= 0 && targetAngle <= 75) ||
        (targetAngle >= 315 && targetAngle < 360) ||
        (targetAngle >= 105 && targetAngle <= 225)
      );

      if (!inRange) {
        Serial.println(F("❌ Invalid input: Allowed angles are 0-75, 315-360, or 105-225"));
      } else {
        float diff = targetAngle - currentAngleY;

        // Rotate shortest path
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;

        angleMoveY(diff);
        currentAngleY = targetAngle;

        Serial.print(F("✅ Y-axis rotated to "));
        Serial.print(currentAngleY);
        Serial.println(F("°"));
      }
    }

    else {
      cmd.toUpperCase();

      if (cmd == "C") modeX = 'C';
      else if (cmd == "A") modeX = 'A';
      else if (cmd == "R") modeX = 'R';
      else if (cmd == "CY") modeY = 'C';
      else if (cmd == "AY") modeY = 'A';
      else if (cmd == "RY") modeY = 'R';
      else if (cmd == "T") rotateXToLatLongBearing();
      else if (cmd.endsWith("Y")) {
        modeY = 'X';
        rotateYToHeading(cmd.substring(0, cmd.length() - 1));
      }
      else if (cmd == "N" || cmd == "NE" || cmd == "E" || cmd == "SE" ||
               cmd == "S" || cmd == "SW" || cmd == "W" || cmd == "NW") {
        modeX = 'X';
        rotateXToHeading(cmd);
      } else {
        Serial.println(F("Unknown command"));
      }
    }

    lastModeChange = millis();
  }

  if (modeX == 'C') stepPulse(STEP_PIN);
  else if (modeX == 'A') angleMoveX(ANGLE_STEP);
  else if (modeX == 'R') resetX();

  if (modeY == 'C') stepPulse(Y_STEP_PIN);
  else if (modeY == 'A') angleMoveY(ANGLE_STEP);
  else if (modeY == 'R') resetY();

  if (modeX == 'X' && millis() - lastModeChange > 60000 && fabs(currentAngleX) > 0.1)
    resetX();

  if (modeY == 'X' && millis() - lastModeChange > 60000 && fabs(currentAngleY) > 0.1)
    resetY();
}
