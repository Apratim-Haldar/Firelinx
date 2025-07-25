/*
 * Fire-suppression turret controller
 *  – X-axis on TB6600 (microstep 1/32, 6400 steps/rev)
 *  – Y-axis on A4988 (microstep 1/16 through MS1-3)
 *  – Two pumps on relays, PWM speed pin
 *  – “T” command = point by GPS bearing + ballistic elevation, then fire
 *  – “M1” manual pump, “AUTO” menu fire, compass headings, direct Y angles,
 *    continuous / auto-rotate / return-to-zero modes for both axes
 */

#include <Arduino.h>
#include <math.h>

// ─────────── Pin map ───────────────────────────────────────────────
#define STEP_PIN    3      // X (TB6600)  PUL+
#define DIR_PIN     2      // X DIR+

#define Y_STEP_PIN  9      // Y (A4988)   STEP
#define Y_DIR_PIN   8      // Y DIR
#define Y_SLEEP_PIN 10
#define Y_MS1_PIN   12
#define Y_MS2_PIN   11
#define Y_MS3_PIN   A0

#define PUMP1_RELAY_PIN 30
#define PUMP2_RELAY_PIN 31
#define MOTOR_PWM_PIN   6      // speed control (0-255)

const bool PWM_LOGIC_INVERTED = true;   // HIGH == off for your MOSFET/H-bridge?

// ─────────── Stepper parameters ────────────────────────────────────
const int   STEPS_PER_REV  = 200;
const int   MICROSTEPS_X   = 32;        // TB6600 dip-switches set to 1/32 → 6400 s/r
const int   MICROSTEPS_Y   = 16;        // A4988 1/16 (all MS pins HIGH)
const long  STEP_DELAY_US  = 500L;      // pulse width/spacing
const float Y_GEAR_RATIO   = 3.75;      // gearbox (output/input)

// ─────────── Ballistics constants ──────────────────────────────────
const float GRAVITY           = 9.81;   // m·s⁻²
const float WATER_VELOCITY_MS = 25.0;   // tune to your nozzle exit speed!
const float NOZZLE_HEIGHT_M   = 0.0;    // positive = launcher higher than target

// ─────────── Target / launcher GPS (°) – demo defaults ────────────
float lat1 = 22.6703779, lon1 = 88.4430516;   // launcher
float lat2 = 22.5754,    lon2 = 88.4798;      // target

// ─────────── Runtime state ─────────────────────────────────────────
char  modeX = 'X', modeY = 'X';          // X/Y mode flags
float currentAngleX = 0.0, currentAngleY = 0.0;
unsigned long lastModeChange = 0;

// ───────────────────────── Helper macros ───────────────────────────
inline int   pwmOut(int v)      { return PWM_LOGIC_INVERTED ? 255 - v : v; }
inline void  stepPulse(int pin) { digitalWrite(pin, HIGH); delayMicroseconds(STEP_DELAY_US);
                                  digitalWrite(pin, LOW);  delayMicroseconds(STEP_DELAY_US); }
inline float toRad(float d)     { return d * PI / 180.0; }
inline float toDeg(float r)     { return r * 180.0 / PI; }

// ─────────────────────── Precise distance/bearing ──────────────────
float haversineM(float la1,float lo1,float la2,float lo2)
{
  float dLat = toRad(la2-la1);
  float dLon = toRad(lo2-lo1);
  float a = sin(dLat/2)*sin(dLat/2) +
            cos(toRad(la1))*cos(toRad(la2))*sin(dLon/2)*sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return 6371000.0 * c;                    // Earth radius → metres
}

float bearingDeg(float la1,float lo1,float la2,float lo2)
{
  float dLon = toRad(lo2 - lo1);
  float y = sin(dLon) * cos(toRad(la2));
  float x = cos(toRad(la1)) * sin(toRad(la2)) -
            sin(toRad(la1)) * cos(toRad(la2)) * cos(dLon);
  return fmod(toDeg(atan2(y, x)) + 360.0, 360.0);   // 0° = N, cw positive
}

// Lower-trajectory solution for elevation (°)
float projectileElevation(float d, float v, float h = 0.0)
{
  float v2 = v * v;
  float term = v2 * v2 - GRAVITY * (GRAVITY * d * d + 2 * h * v2);
  if (term <= 0) return 60.0;                         // out of range → use high default
  float theta = atan( (v2 - sqrt(term)) / (GRAVITY * d) );
  return constrain(toDeg(theta), 0.0, 75.0);
}

// ───────────────────────── Stepper moves ───────────────────────────
void angleMoveX(float angle)
{
  long steps = lround(fabs(angle) / 360.0 * STEPS_PER_REV * MICROSTEPS_X);
  digitalWrite(DIR_PIN, angle >= 0 ? HIGH : LOW);
  for (long i = 0; i < steps; ++i) stepPulse(STEP_PIN);
  delay(500);
}

void angleMoveY(float angle)
{
  long steps = lround(fabs(angle) / 360.0 * STEPS_PER_REV * MICROSTEPS_Y * (Y_GEAR_RATIO / 2.0));
  digitalWrite(Y_DIR_PIN, angle >= 0 ? HIGH : LOW);    // assembly convention
  for (long i = 0; i < steps; ++i) stepPulse(Y_STEP_PIN);
  delay(500);
}

// ────────────────────── High-level axis helpers ────────────────────
void rotateYToAngle(float tgt)
{
  tgt = constrain(tgt, -45, 75);
  float diff = tgt - currentAngleY;
  angleMoveY(diff);
  currentAngleY = tgt;
  Serial.print(F("Y set → ")); Serial.print(currentAngleY); Serial.println(F(" °"));
}

float headingToAngle(String h)
{
  if (h == "N")  return 0;
  if (h == "NE") return 315;
  if (h == "E")  return 270;
  if (h == "SE") return 225;
  if (h == "S")  return 180;
  if (h == "SW") return 135;
  if (h == "W")  return 90;
  if (h == "NW") return 45;
  return NAN;
}

void rotateXToHeading(String h)
{
  float tgt = headingToAngle(h);
  if (isnan(tgt)) return;
  float diff = tgt - currentAngleX;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  angleMoveX(diff);
  currentAngleX = tgt;
  Serial.print(F("X set → ")); Serial.print(currentAngleX); Serial.println(F(" °"));
}

// ───────────────────────── Pump control ────────────────────────────
void firePump(int pin, int durationSec, int speed = 255)
{
  digitalWrite(PUMP1_RELAY_PIN, LOW);
  digitalWrite(PUMP2_RELAY_PIN, LOW);
  digitalWrite(pin, HIGH);
  analogWrite(MOTOR_PWM_PIN, pwmOut(speed));

  Serial.print(F("Pump ")); Serial.print(pin == PUMP1_RELAY_PIN ? 1 : 2);
  Serial.print(F(" firing for ")); Serial.print(durationSec); Serial.println(F(" s"));

  delay((unsigned long)durationSec * 1000UL);
  analogWrite(MOTOR_PWM_PIN, pwmOut(0));
  digitalWrite(pin, LOW);
}

// ─────────── Combined GPS-target command (“T”) ─────────────────────
void rotateXToLatLongBearing()
{
  Serial.println(F("== AUTOMATED TARGET =="));
  // 1. Bearing
  float brg = fmod(360.0 - bearingDeg(lat1, lon1, lat2, lon2), 360.0);
  float diffX = brg - currentAngleX;
  if (diffX > 180) diffX -= 360;
  if (diffX < -180) diffX += 360;
  angleMoveX(diffX);
  currentAngleX = brg;
  Serial.print(F("Bearing → ")); Serial.print(brg); Serial.println(F(" °"));

  // 2. Distance & ballistic elevation
  float distM = haversineM(lat1, lon1, lat2, lon2);
  float elev  = projectileElevation(distM, WATER_VELOCITY_MS, NOZZLE_HEIGHT_M);
  Serial.print(F("Distance ")); Serial.print(distM, 0); Serial.print(F(" m, Elev → "));
  Serial.print(elev); Serial.println(F(" °"));
  rotateYToAngle(elev);

  // 3. Fire (pump 1 by default)
  firePump(PUMP1_RELAY_PIN, 10);
}

// ───────────────────────── Manual modes ────────────────────────────
void manualPumpMode()
{
  Serial.println(F("Manual pump: enter 1 or 2"));
  while (!Serial.available());
  int pump = Serial.parseInt(); Serial.readStringUntil('\n');
  int pin = (pump == 1) ? PUMP1_RELAY_PIN :
            (pump == 2) ? PUMP2_RELAY_PIN : 0;
  if (!pin) { Serial.println(F("Invalid pump")); return; }

  Serial.println(F("Speed 0-255?"));
  while (!Serial.available());
  int spd = constrain(Serial.parseInt(), 0, 255); Serial.readStringUntil('\n');

  Serial.println(F("Duration s?"));
  while (!Serial.available());
  int dur = Serial.parseInt(); Serial.readStringUntil('\n');

  firePump(pin, dur, spd);
}

void autoFireMode()
{
  Serial.println(F("Fire type A-D:"));
  while (!Serial.available());
  char t = toupper(Serial.read()); Serial.readStringUntil('\n');

  Serial.println(F("Intensity 1-4:"));
  while (!Serial.available());
  int i = constrain(Serial.parseInt(), 1, 4); Serial.readStringUntil('\n');
  int duration = i * 10;

  int pin = (t == 'A' || t == 'C') ? PUMP1_RELAY_PIN : PUMP2_RELAY_PIN;

  Serial.println(F("Elevation 15-60 °:"));
  while (!Serial.available());
  float elev = constrain(Serial.parseFloat(), 15, 60); Serial.readStringUntil('\n');

  rotateYToAngle(elev);
  firePump(pin, duration);
}

// ───────────────────────── Angle parser ────────────────────────────
float parseAngle(String s)
{
  bool opp = s.endsWith("O") || s.endsWith("o");
  if (opp) s.remove(s.length() - 1);
  float a = s.toFloat();
  if (opp) a = -a;
  return constrain(a, -45, 75);
}

// ───────────────────────── Arduino setup ───────────────────────────
void setup()
{
  Serial.begin(9600);

  // X stepper
  pinMode(DIR_PIN, OUTPUT); pinMode(STEP_PIN, OUTPUT);

  // Y stepper
  pinMode(Y_DIR_PIN, OUTPUT); pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_SLEEP_PIN, OUTPUT); digitalWrite(Y_SLEEP_PIN, HIGH);
  pinMode(Y_MS1_PIN, OUTPUT); pinMode(Y_MS2_PIN, OUTPUT); pinMode(Y_MS3_PIN, OUTPUT);
  digitalWrite(Y_MS1_PIN, HIGH); digitalWrite(Y_MS2_PIN, HIGH); digitalWrite(Y_MS3_PIN, HIGH);

  // Pumps
  pinMode(PUMP1_RELAY_PIN, OUTPUT); digitalWrite(PUMP1_RELAY_PIN, LOW);
  pinMode(PUMP2_RELAY_PIN, OUTPUT); digitalWrite(PUMP2_RELAY_PIN, LOW);
  pinMode(MOTOR_PWM_PIN,  OUTPUT);  analogWrite(MOTOR_PWM_PIN, pwmOut(0));

  Serial.println(F("System Ready: T=Target | M1=Manual | AUTO=AutoFire"));
  Serial.println(F("Compass: N,NE,E,SE,S,SW,W,NW | Y-angle ± | X/Y modes C,A,R"));
}

// ───────────────────────── Main loop ───────────────────────────────
void loop()
{
  // ░░░░░ Monitor serial commands ░░░░░
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    Serial.print(F("CMD: ")); Serial.println(cmd);

    if (cmd == "T")            rotateXToLatLongBearing();
    else if (cmd == "M1")      manualPumpMode();
    else if (cmd == "AUTO")    autoFireMode();

    // X individual modes
    else if (cmd == "C")  { modeX = 'C'; Serial.println(F("X: continuous")); }
    else if (cmd == "A")  { modeX = 'A'; Serial.println(F("X: auto 90 °")); }
    else if (cmd == "R")  { modeX = 'R'; Serial.println(F("X: return 0")); }

    // Y individual modes
    else if (cmd == "CY") { modeY = 'C'; Serial.println(F("Y: continuous")); }
    else if (cmd == "AY") { modeY = 'A'; Serial.println(F("Y: auto 45 °")); }
    else if (cmd == "RY") { modeY = 'R'; Serial.println(F("Y: return 0")); }

    // Compass headings
    else if (cmd == "N" || cmd == "NE" || cmd == "E" || cmd == "SE" ||
             cmd == "S" || cmd == "SW" || cmd == "W" || cmd == "NW")
      rotateXToHeading(cmd);

    // Raw Y-angle
    else {
      float t = parseAngle(cmd);
      if (!isnan(t)) rotateYToAngle(t);
      else           Serial.println(F("Unknown cmd"));
    }
    lastModeChange = millis();
  }

  // ░░░░░ Mode handlers ░░░░░
  // X
  if (modeX == 'C')       stepPulse(STEP_PIN);
  else if (modeX == 'A') { angleMoveX(90.0); modeX = 'X'; }
  else if (modeX == 'R') { angleMoveX(-currentAngleX); currentAngleX = 0; modeX = 'X'; }

  // Y
  if (modeY == 'C')       stepPulse(Y_STEP_PIN);
  else if (modeY == 'A') { rotateYToAngle(45.0); modeY = 'X'; }
  else if (modeY == 'R') { rotateYToAngle(0.0);  modeY = 'X'; }

  // ░░░░░ Idle auto-home after 60 s ░░░░░
  if (modeX == 'X' && millis() - lastModeChange > 60000 && fabs(currentAngleX) > 0.1) {
    angleMoveX(-currentAngleX); currentAngleX = 0.0;
  }
  if (modeY == 'X' && millis() - lastModeChange > 60000 && fabs(currentAngleY) > 0.1) {
    rotateYToAngle(0.0);
  }
}
