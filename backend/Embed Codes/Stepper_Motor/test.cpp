// FireLinx Integrated System - Enhanced UI/UX Version with Distance-Based Pump Control, Station ID Support, Optimal Pathfinding, and Laser Targeting
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <math.h>

// ---------------- TFT Setup ------------------
#define TFT_SCLK 52
#define TFT_MOSI 51
#define TFT_CS   10
#define TFT_RST  8
#define TFT_DC   9

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Enhanced Professional Color Palette
#define COLOR_NAVY       0x000F    // Original navy (kept for compatibility)
#define COLOR_WHITE      0xFFFF    // Original white (kept for compatibility)
#define COLOR_BG_DARK    0x0841    // Dark blue background
#define COLOR_BG_LIGHT   0x2945    // Light blue accent
#define COLOR_PRIMARY    0x07FF    // Cyan primary
#define COLOR_SUCCESS    0x07E0    // Green success
#define COLOR_WARNING    0xFFE0    // Yellow warning
#define COLOR_DANGER     0xF800    // Red danger
#define COLOR_GRAY       0x8410    // Gray secondary text
#define COLOR_ORANGE     0xFD20    // Orange accent
#define COLOR_PURPLE     0x780F    // Purple accent

// ---------------- I2C Setup ------------------
#define MEGA_I2C_SLAVE_A   DDRESS 0x08
#define EXPECTED_DATA_SIZE 12
#define ACKNOWLEDGEMENT_BYTE 0x55

// ---------------- Pump Setup ------------------
#define PUMP1_RELAY_PIN 30
#define PUMP2_RELAY_PIN 31
#define MOTOR_PWM_PIN 7
const bool PWM_LOGIC_INVERTED = true;

// ---------------- Stepper Pins ------------------
#define X_STEP_PIN 3
#define X_DIR_PIN 2
#define Y_STEP_PIN 6
#define Y_DIR_PIN 5
#define Y_SLEEP_PIN A3
#define Y_MS1_PIN A0
#define Y_MS2_PIN A1
#define Y_MS3_PIN A2

// ---------------- Laser Setup ------------------
#define LASER_PIN 14  // Laser LED connected to pin 14

// ---------------- Joysticks ------------------
#define JOY_X_PIN A14
#define JOY_Y_PIN A15
const int JOY_DEADZONE = 50;

// ---------------- Touch Input ------------------
#define TOUCH_BUTTON_1 43
#define TOUCH_BUTTON_2 42
#define TOUCH_BUTTON_3 41
#define TOUCH_BUTTON_4 40
#define TOUCH_SHOOT_BUTTON 39

// ---------------- Potentiometer Setup ------------------
#define POT_PIN A7  // Potentiometer for real-time speed control

// ---------------- FireData Struct ------------------
typedef struct {
  char fireType;
  char fireIntensity;
  char stnID[6];       // Added station ID field
  float latitude;
  float longitude;
  bool dataValid;
  bool ackValue;
} FireData;

volatile FireData receivedData;
volatile bool newDataAvailable = false;

// ---------------- Pump State Management ------------------
struct {
  bool isActive;
  bool isAutomatic;
  int pin;
  unsigned long endTime;
  int durationSec;
  int initialSpeed;
} currentPump = { false, false, 0, 0, 0, 0 };

// ---------------- Laser Control ------------------
bool laserOn = false;
unsigned long lastLaserActivity = 0;
const unsigned long LASER_TIMEOUT = 10000; // 10 seconds

// ---------------- Constants ------------------
const int STEPS_PER_REV = 200;
const int MICROSTEPS_X = 32;
const int MICROSTEPS_Y = 16;
const float Y_GEAR_RATIO = 3.75;
const float DEFAULT_STEP_DELAY_US = 500.0;  // Original delay
const float FAST_STEP_DELAY_US = 150.0;     // Faster delay for when pump is active
float currentStepDelay = DEFAULT_STEP_DELAY_US; // Variable step delay that can be changed at runtime

const float GRAVITY = 9.81;
const float WATER_VELOCITY_MS = 25.0;

// Station positions (bearings in degrees, distance in meters)
const float STATION_DISTANCE = 2.0;  // All stations at 2m distance
const float STATION_BEARINGS[4] = {0.0, 270.0, 180.0, 90.0};  // N, E, S, W (corrected order)

float currentAngleX = 0.0, currentAngleY = 0.0;
bool manualModeActive = false;
unsigned long lastJoystickTime = 0;

// ================== NEW MANUAL CONTROL SETTINGS ==================
float manualXSpeed = 1.0;   // Degrees per joystick movement (X-axis)
float manualYSpeed = 1.0;   // Degrees per joystick movement (Y-axis)
// ================== END NEW SETTINGS ==================

// ================== PUMP DURATION SETTINGS ==================
const int PUMP_DURATIONS[4] = {60, 120, 180, 240}; // Intensity 1-4 durations in seconds
// ================== END PUMP DURATION SETTINGS ==================

// Motor speed multipliers - higher values = faster movement
float xMotorSpeedMultiplier = 1.0;  // Default multiplier
float yMotorSpeedMultiplier = 1.0;  // Default multiplier

// ---------------- Enhanced UI Helper Functions ------------------

void drawHeader(const char* title, uint16_t bgColor) {
  // Header background with gradient effect
  tft.fillRect(0, 0, 160, 18, bgColor);
  tft.fillRect(0, 18, 160, 2, COLOR_PRIMARY);
  
  // Title text
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(1);
  tft.setCursor(5, 5);
  tft.print(title);
  
  // Status indicator dot
  uint16_t statusColor = manualModeActive ? COLOR_WARNING : COLOR_SUCCESS;
  tft.fillCircle(150, 9, 3, statusColor);
}

void drawButton(int x, int y, int width, int height, const char* text, uint16_t color, bool selected = false) {
  // Button background
  uint16_t bgColor = selected ? color : COLOR_BG_LIGHT;
  uint16_t textColor = selected ? COLOR_BG_DARK : COLOR_WHITE;
  uint16_t borderColor = selected ? COLOR_WHITE : color;
  
  tft.fillRoundRect(x, y, width, height, 4, bgColor);
  tft.drawRoundRect(x, y, width, height, 4, borderColor);
  
  // Button text centered
  tft.setTextColor(textColor);
  tft.setTextSize(1);
  int textX = x + (width - strlen(text) * 6) / 2;
  int textY = y + (height - 8) / 2;
  tft.setCursor(textX, textY);
  tft.print(text);
}

void drawStatusCard(int x, int y, int width, int height, const char* label, const char* value, uint16_t accentColor) {
  // Card background with border
  tft.fillRoundRect(x, y, width, height, 3, COLOR_BG_LIGHT);
  tft.drawRoundRect(x, y, width, height, 3, accentColor);
  
  // Accent line at top
  tft.fillRect(x + 1, y + 1, width - 2, 2, accentColor);
  
  // Label
  tft.setTextColor(COLOR_GRAY);
  tft.setTextSize(1);
  tft.setCursor(x + 3, y + 6);
  tft.print(label);
  
  // Value
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(1);
  tft.setCursor(x + 3, y + 18);
  tft.print(value);
}

void drawIcon(int x, int y, char icon, uint16_t color) {
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.setCursor(x, y);
  tft.print(icon);
}

// ---------------- Laser Control Functions ------------------
void activateLaser() {
  if (!laserOn) {
    digitalWrite(LASER_PIN, HIGH);
    laserOn = true;
    lastLaserActivity = millis();
    Serial.println("[LASER] Activated");
  }
}

void deactivateLaser() {
  if (laserOn) {
    digitalWrite(LASER_PIN, LOW);
    laserOn = false;
    Serial.println("[LASER] Deactivated");
  }
}

// ---------------- Utilities ------------------
inline float toRad(float d) { return d * PI / 180.0; }
inline float toDeg(float r) { return r * 180.0 / PI; }
inline int pwmOut(int v) { return PWM_LOGIC_INVERTED ? 255 - v : v; }
inline void stepPulse(int pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(currentStepDelay);
  digitalWrite(pin, LOW);
  delayMicroseconds(currentStepDelay);
}

float haversineM(float la1, float lo1, float la2, float lo2) {
  float dLat = toRad(la2 - la1);
  float dLon = toRad(lo2 - lo1);
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(toRad(la1)) * cos(toRad(la2)) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371000.0 * c;
}

float bearingDeg(float la1, float lo1, float la2, float lo2) {
  float dLon = toRad(lo2 - lo1);
  float y = sin(dLon) * cos(toRad(la2));
  float x = cos(toRad(la1)) * sin(toRad(la2)) - sin(toRad(la1)) * cos(toRad(la2)) * cos(dLon);
  return fmod(toDeg(atan2(y, x)) + 360.0, 360.0);
}

float projectileElevation(float d, float v) {
  float v2 = v * v;
  float term = v2 * v2 - GRAVITY * (GRAVITY * d * d);
  if (term <= 0) return 60.0;
  float theta = atan((v2 - sqrt(term)) / (GRAVITY * d));
  return constrain(toDeg(theta), 0.0, 75.0);
}

// Optimized PWM calculation with quadratic curve
int calculatePWMByDistance(float distance) {
  const float MAX_RANGE = 6.0;   // meters at PWM 255 (full power)
  const int MIN_PWM = 150;       // empirically determined lower limit
  const int MAX_PWM = 255;       // maximum possible PWM
  const float MIN_RANGE = 1.0;   // safe minimum spray reach

  // Clamp input distance
  distance = constrain(distance, MIN_RANGE, MAX_RANGE);

  // Quadratic approximation: range = k * pwm^2 → pwm = sqrt(range / k)
  float k = MAX_RANGE / (float)(MAX_PWM * MAX_PWM);  // derive constant from max range
  int pwm = sqrt(distance / k);

  // Constrain final PWM between min and max
  return constrain(pwm, MIN_PWM, MAX_PWM);
}

// ============== OPTIMIZED MOVEMENT FUNCTIONS ==============
void moveXTo(float targetAngle) {
  activateLaser(); // Activate laser for targeting
  
  // Calculate shortest rotational path
  float diff = targetAngle - currentAngleX;
  
  // Normalize difference to shortest arc
  if (diff > 180.0) diff -= 360.0;
  else if (diff < -180.0) diff += 360.0;
  
  // Determine optimal rotation direction
  bool clockwise = (diff > 0);
  digitalWrite(X_DIR_PIN, clockwise ? HIGH : LOW);
  
  // Calculate steps needed with speed multiplier applied
  float absDiff = fabs(diff);
  long steps = lround(absDiff / 360.0 * STEPS_PER_REV * MICROSTEPS_X);
  
  // Apply the speed multiplier by adjusting the effective step count
  long effectiveSteps = steps / xMotorSpeedMultiplier;
  if (effectiveSteps < 1) effectiveSteps = 1;
  
  Serial.print("[X] Moving ");
  Serial.print(clockwise ? "CW" : "CCW");
  Serial.print(" Steps: ");
  Serial.print(steps);
  Serial.print(" (Speed Mult: ");
  Serial.print(xMotorSpeedMultiplier);
  Serial.println(")");
  
  // Execute movement
  float stepIncrement = (float)steps / effectiveSteps;
  float stepCounter = 0;
  
  for (long i = 0; i < effectiveSteps; ++i) {
    stepCounter += stepIncrement;
    while (stepCounter >= 1.0) {
      stepPulse(X_STEP_PIN);
      stepCounter -= 1.0;
    }
  }
  
  // Update current angle (considering rotation direction)
  currentAngleX = targetAngle;
  
  // Normalize angle to 0-360 range
  if (currentAngleX >= 360) currentAngleX -= 360;
  if (currentAngleX < 0) currentAngleX += 360;
  
  Serial.print("[X] Now at: ");
  Serial.println(currentAngleX);
  lastLaserActivity = millis(); // Update laser activity time
}

void moveYTo(float targetAngle) {
  activateLaser(); // Activate laser for targeting
  
  // Constrain target within physical limits
  targetAngle = constrain(targetAngle, -45, 75);
  float diff = targetAngle - currentAngleY;
  
  // Determine optimal rotation direction
  bool upDirection = (diff > 0);
  digitalWrite(Y_DIR_PIN, upDirection ? HIGH : LOW);
  
  // Calculate steps needed
  float absDiff = fabs(diff);
  long steps = lround(absDiff / 360.0 * STEPS_PER_REV * MICROSTEPS_Y * Y_GEAR_RATIO);
  
  // Apply the speed multiplier by adjusting the effective step count
  long effectiveSteps = steps / yMotorSpeedMultiplier;
  if (effectiveSteps < 1) effectiveSteps = 1;
  
  Serial.print("[Y] Moving ");
  Serial.print(upDirection ? "UP" : "DOWN");
  Serial.print(" Steps: ");
  Serial.print(steps);
  Serial.print(" (Speed Mult: ");
  Serial.print(yMotorSpeedMultiplier);
  Serial.println(")");
  
  // Execute movement
  float stepIncrement = (float)steps / effectiveSteps;
  float stepCounter = 0;
  
  for (long i = 0; i < effectiveSteps; ++i) {
    stepCounter += stepIncrement;
    while (stepCounter >= 1.0) {
      stepPulse(Y_STEP_PIN);
      stepCounter -= 1.0;
    }
  }
  
  // Update current angle
  currentAngleY = targetAngle;
  
  Serial.print("[Y] Now at: ");
  Serial.println(currentAngleY);
  lastLaserActivity = millis(); // Update laser activity time
}

// Relative movement for joystick control
void moveXRelative(float angle) {
  activateLaser(); // Activate laser for targeting
  float newAngle = currentAngleX + angle;
  moveXTo(newAngle);
}

void moveYRelative(float angle) {
  activateLaser(); // Activate laser for targeting
  float newAngle = currentAngleY + angle;
  moveYTo(newAngle);
}

// ============== END OPTIMIZED MOVEMENT ==============

// ============== REAL-TIME PUMP CONTROL ==============
void startPump(int pin, int durationSec, int speed, bool isAutomatic) {
  Serial.print("[PUMP] Activating Pump "); Serial.print((pin == PUMP1_RELAY_PIN) ? 1 : 2);
  Serial.print(" at speed "); Serial.print(speed);
  Serial.print(" for "); Serial.print(durationSec); Serial.println(" seconds");
  
  // Set pump state
  currentPump.isActive = true;
  currentPump.isAutomatic = isAutomatic;
  currentPump.pin = pin;
  currentPump.durationSec = durationSec;
  currentPump.endTime = millis() + (unsigned long)durationSec * 1000UL;
  currentPump.initialSpeed = speed;
  
  // Activate pump with proper PWM inversion
  digitalWrite(PUMP1_RELAY_PIN, pin == PUMP1_RELAY_PIN ? HIGH : LOW);
  digitalWrite(PUMP2_RELAY_PIN, pin == PUMP2_RELAY_PIN ? HIGH : LOW);
  analogWrite(MOTOR_PWM_PIN, pwmOut(speed));
  
  // Set faster motor speeds during pump operation
  currentStepDelay = FAST_STEP_DELAY_US;
  xMotorSpeedMultiplier = 3.0;  // Make X motor 3x faster
  yMotorSpeedMultiplier = 3.0;  // Make Y motor 3x faster
  
  // Activate laser for targeting
  activateLaser();
  
  // Show initial firing screen
  showFiringScreen(pin, speed, durationSec);
}

void stopPump() {
  analogWrite(MOTOR_PWM_PIN, pwmOut(0));
  digitalWrite(PUMP1_RELAY_PIN, LOW);
  digitalWrite(PUMP2_RELAY_PIN, LOW);
  currentPump.isActive = false;
  
  // Reset motor speeds to default
  currentStepDelay = DEFAULT_STEP_DELAY_US;
  xMotorSpeedMultiplier = 1.0;
  yMotorSpeedMultiplier = 1.0;
  
  // Update laser activity time (keep on for targeting after firing)
  lastLaserActivity = millis();
  
  Serial.println("[PUMP] Deactivated");
}

void showFiringScreen(int pin, int speed, int duration) {
  tft.fillScreen(COLOR_BG_DARK);
  drawHeader("FIRING IN PROGRESS", COLOR_DANGER);
  
  // Fire icon
  drawIcon(70, 35, '*', COLOR_DANGER);
  
  // Pump info
  char pumpInfo[20];
  sprintf(pumpInfo, "PUMP %d ACTIVE", (pin == PUMP1_RELAY_PIN) ? 1 : 2);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(1);
  tft.setCursor(35, 60);
  tft.print(pumpInfo);
  
  // Display PWM and time info
  tft.setCursor(30, 75);
  tft.print("PWM: ");
  tft.print(speed);
  
  tft.setCursor(80, 75);
  tft.print("Time: ");
  tft.print(duration);
  tft.print("s");
  
  // Display current angles
  tft.setCursor(10, 90);
  tft.print("X:");
  tft.print(currentAngleX, 1);
  tft.print("°");
  
  tft.setCursor(80, 90);
  tft.print("Y:");
  tft.print(currentAngleY, 1);
  tft.print("°");
  
  // Laser status
  tft.setCursor(10, 105);
  tft.print("Laser: ");
  tft.setTextColor(laserOn ? COLOR_SUCCESS : COLOR_DANGER);
  tft.print(laserOn ? "ON" : "OFF");
}

void updateFiringScreen(int speed, int remainingSec) {
  // Update PWM value
  tft.fillRect(55, 75, 30, 8, COLOR_BG_DARK); // Clear previous PWM
  tft.setTextColor(COLOR_WHITE);
  tft.setCursor(55, 75);
  tft.print(speed);
  
  // Update remaining time
  tft.fillRect(115, 75, 30, 8, COLOR_BG_DARK); // Clear previous time
  tft.setCursor(115, 75);
  tft.print(remainingSec);
  
  // Update current angles
  tft.fillRect(25, 90, 45, 8, COLOR_BG_DARK); // Clear X angle
  tft.fillRect(95, 90, 45, 8, COLOR_BG_DARK); // Clear Y angle
  
  tft.setCursor(25, 90);
  tft.print(currentAngleX, 1);
  
  tft.setCursor(95, 90);
  tft.print(currentAngleY, 1);
  
  // Update laser status
  tft.fillRect(45, 105, 30, 8, COLOR_BG_DARK); // Clear laser status
  tft.setCursor(45, 105);
  tft.setTextColor(laserOn ? COLOR_SUCCESS : COLOR_DANGER);
  tft.print(laserOn ? "ON" : "OFF");
}
// ============== END REAL-TIME PUMP CONTROL ==============

// Optimized reset function with cable management
void resetSystem() {
  Serial.println("[RESET] Optimized homing sequence");
  
  // Deactivate laser during homing
  deactivateLaser();
  
  // Always return to zero position via shortest path
  moveXTo(0.0);
  moveYTo(0.0);
  
  // Reset tracking variables
  currentAngleX = 0.0;
  currentAngleY = 0.0;
  
  Serial.println("[RESET] System reset complete");
}

// Optimized station targeting
void targetStation(const char* stnID) {
  Serial.print("Targeting station: "); Serial.println(stnID);
  
  float bearing = 0.0;
  
  // Station mapping (corrected to match bearing array)
  if (strcmp(stnID, "A/1") == 0) bearing = STATION_BEARINGS[0]; // North
  else if (strcmp(stnID, "B/2") == 0) bearing = STATION_BEARINGS[1]; // East
  else if (strcmp(stnID, "C/3") == 0) bearing = STATION_BEARINGS[2]; // South
  else if (strcmp(stnID, "D/4") == 0) bearing = STATION_BEARINGS[3]; // West
  else {
    Serial.println("Unknown station ID");
    return;
  }
  
  // Calculate elevation for fixed distance
  float elevation = projectileElevation(STATION_DISTANCE, WATER_VELOCITY_MS);
  
  // Move with optimal pathfinding
  moveXTo(bearing);
  moveYTo(elevation);
  
  Serial.print("Station bearing: "); Serial.print(bearing);
  Serial.print(" Elevation: "); Serial.println(elevation);
}

void processFireAlert() {
  // Declare distance variable once at the top
  float dist = 0.0;
  
  // ================== CORRECTED TARGETING PRIORITY ==================
  // 1. Always try to use GPS coordinates first if valid
  if (fabs(receivedData.latitude) > 0.0001 && fabs(receivedData.longitude) > 0.0001) {
    Serial.println("[ALERT] Using GPS coordinates for targeting");
    
    // Enhanced targeting display
    tft.fillScreen(COLOR_BG_DARK);
    drawHeader("AUTO TARGETING", COLOR_DANGER);
    
    // Calculate distance
    dist = haversineM(22.6703779, 88.4430516, receivedData.latitude, receivedData.longitude);
    
    // Calculate PWM based on distance
    int speed = calculatePWMByDistance(dist);
    
    // Display distance and PWM info
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1);
    tft.setCursor(10, 30);
    tft.print("Distance: ");
    tft.print(dist, 1);
    tft.print("m");
    
    tft.setCursor(10, 45);
    tft.print("Pump PWM: ");
    tft.print(speed);
    
    // Display station if available (informational only)
    if (strlen(receivedData.stnID) > 0) {
      tft.setCursor(10, 60);
      tft.print("Station: ");
      tft.print((const char*)receivedData.stnID);
    }
    
    // Target icon
    drawIcon(70, 75, '+', COLOR_DANGER);
    
    // Fire info
    tft.setCursor(10, 90);
    tft.print("Type: ");
    tft.setTextColor(COLOR_WARNING);
    tft.print(receivedData.fireType);
    
    tft.setTextColor(COLOR_WHITE);
    tft.setCursor(10, 105);
    tft.print("Intensity: ");
    tft.setTextColor(COLOR_DANGER);
    tft.print(receivedData.fireIntensity);
    
    tft.setTextColor(COLOR_PRIMARY);
    tft.setCursor(40, 120);
    tft.print("ENGAGING...");
    
    delay(2000); // Show targeting screen
    
    Serial.println("[ALERT] Processing incoming fire alert...");

    // Calculate bearing using optimal path
    float brg = bearingDeg(22.6703779, 88.4430516, receivedData.latitude, receivedData.longitude);
    brg = fmod(360.0 - brg, 360.0);  // Convert to compass bearing
    
    // Move using optimal pathfinding
    moveXTo(brg);

    float elev = projectileElevation(dist, WATER_VELOCITY_MS);
    moveYTo(elev);

    // USE NEW DURATION MAPPING
    int intensityIndex = receivedData.fireIntensity - '1'; // Convert char to index (0-3)
    intensityIndex = constrain(intensityIndex, 0, 3); // Ensure valid range
    int duration = PUMP_DURATIONS[intensityIndex];
    
    int pin = (receivedData.fireType == 'A' || receivedData.fireType == 'C') ? PUMP1_RELAY_PIN : PUMP2_RELAY_PIN;
    
    // Start pump with real-time control capability
    startPump(pin, duration, speed, true);
    return;
  }
  // 2. Fallback to station ID if GPS is invalid but station ID is valid
  else if (strlen(receivedData.stnID) > 0) {
    bool validStation = false;
    if (strcmp(receivedData.stnID, "A/1") == 0 ||
        strcmp(receivedData.stnID, "B/2") == 0 ||
        strcmp(receivedData.stnID, "C/3") == 0 ||
        strcmp(receivedData.stnID, "D/4") == 0) {
      validStation = true;
    }
    
    if (validStation) {
      Serial.println("[ALERT] Falling back to station ID targeting (GPS invalid)");
      dist = STATION_DISTANCE;
      
      // Enhanced targeting display
      tft.fillScreen(COLOR_BG_DARK);
      drawHeader("AUTO TARGETING", COLOR_DANGER);
      
      // Display station info
      tft.setTextColor(COLOR_WHITE);
      tft.setTextSize(1);
      tft.setCursor(10, 30);
      tft.print("Station: ");
      tft.print((const char*)receivedData.stnID);
      
      // Calculate PWM for fixed distance
      int speed = calculatePWMByDistance(dist);
      
      tft.setCursor(10, 45);
      tft.print("Distance: ");
      tft.print(dist, 1);
      tft.print("m");
      
      tft.setCursor(10, 60);
      tft.print("Pump PWM: ");
      tft.print(speed);
      
      // Target icon
      drawIcon(70, 75, '+', COLOR_DANGER);
      
      // Fire info
      tft.setCursor(10, 90);
      tft.print("Type: ");
      tft.setTextColor(COLOR_WARNING);
      tft.print(receivedData.fireType);
      
      tft.setTextColor(COLOR_WHITE);
      tft.setCursor(10, 105);
      tft.print("Intensity: ");
      tft.setTextColor(COLOR_DANGER);
      tft.print(receivedData.fireIntensity);
      
      tft.setTextColor(COLOR_PRIMARY);
      tft.setCursor(40, 120);
      tft.print("ENGAGING...");
      
      delay(2000); // Show targeting screen
      
      // Target station
      targetStation(receivedData.stnID);
      
      // USE NEW DURATION MAPPING
      int intensityIndex = receivedData.fireIntensity - '1';
      intensityIndex = constrain(intensityIndex, 0, 3);
      int duration = PUMP_DURATIONS[intensityIndex];
      
      int pin = (receivedData.fireType == 'A' || receivedData.fireType == 'C') ? PUMP1_RELAY_PIN : PUMP2_RELAY_PIN;
      
      startPump(pin, duration, speed, true);
      return;
    }
  }
  // ================== END CORRECTED PRIORITY ==================

  // 3. If both GPS and station ID are invalid, prompt for manual input
  Serial.println("[ALERT] Invalid coordinates and station ID. Please enter manually:");
  while (true) {
    Serial.print("Latitude: ");
    while (!Serial.available());
    receivedData.latitude = Serial.parseFloat();
    Serial.read();
    
    Serial.print("Longitude: ");
    while (!Serial.available());
    receivedData.longitude = Serial.parseFloat();
    Serial.read();
    
    if (fabs(receivedData.latitude) > 0.0001 && fabs(receivedData.longitude) > 0.0001) break;
    Serial.println("Invalid again. Re-enter.");
  }

  // Restart processing with valid coordinates
  processFireAlert();
}

void receiveEvent(int howMany) {
  if (howMany != EXPECTED_DATA_SIZE) {
    while (Wire.available()) Wire.read(); // flush garbage
    Serial.println("[I2C] Incorrect byte count received.");
    return;
  }

  uint8_t buffer[EXPECTED_DATA_SIZE];
  for (int i = 0; i < EXPECTED_DATA_SIZE; i++) {
    buffer[i] = Wire.read();
  }

  receivedData.fireType = (buffer[0] >= 1 && buffer[0] <= 4) ? 'A' + (buffer[0] - 1) : '?';
  receivedData.fireIntensity = (buffer[1] >= 1 && buffer[1] <= 4) ? '0' + buffer[1] : '?';

  // Parse station ID
  uint8_t zoneVal = buffer[2];
  if (zoneVal >= 1 && zoneVal <= 26) {
    snprintf(receivedData.stnID, sizeof(receivedData.stnID), "%c/%d", 'A' + zoneVal - 1, zoneVal);
  } else {
    snprintf(receivedData.stnID, sizeof(receivedData.stnID), "?/?");
  }

  float lat, lon;
  memcpy(&lat, &buffer[3], sizeof(float));
  memcpy(&lon, &buffer[7], sizeof(float));
  receivedData.latitude = lat;
  receivedData.longitude = lon;

  receivedData.dataValid = true;
  receivedData.ackValue = true;
  newDataAvailable = true;

  Serial.print("[I2C] Received Lat: "); Serial.println(receivedData.latitude, 6);
  Serial.print("[I2C] Received Lon: "); Serial.println(receivedData.longitude, 6);
  Serial.print("[I2C] Type: "); Serial.print(receivedData.fireType);
  Serial.print(" Intensity: "); Serial.println(receivedData.fireIntensity);
  Serial.print("[I2C] Station: "); Serial.println((const char*)receivedData.stnID);
}

void requestEvent() {
  Wire.write(receivedData.ackValue ? ACKNOWLEDGEMENT_BYTE : 0x00);
}

// ================== UPDATED JOYSTICK HANDLER ==================
void handleJoystick() {
  // Always allow joystick movement regardless of any other operations
  int xVal = analogRead(JOY_X_PIN);
  int yVal = analogRead(JOY_Y_PIN);
  bool moved = false;

  if (abs(xVal - 512) > JOY_DEADZONE) {
    // Use configurable X speed
    moveXRelative(xVal > 512 ? manualXSpeed : -manualXSpeed);
    moved = true;
  }
  if (abs(yVal - 512) > JOY_DEADZONE) {
    // Use configurable Y speed
    moveYRelative(yVal < 512 ? manualYSpeed : -manualYSpeed);
    moved = true;
  }

  if (moved) {
    manualModeActive = true;
    lastJoystickTime = millis();
    
    // Update display based on current state - ENHANCED FOR PUMP OPERATION
    if (currentPump.isActive) {
      // Update firing screen with new motor positions during pump operation
      int currentSpeed = analogRead(POT_PIN) / 4;
      int remaining = (currentPump.endTime - millis()) / 1000;
      if (remaining < 0) remaining = 0;
      updateFiringScreen(currentSpeed, remaining);
    }
  } 
  // Only reset manual mode if no active operations and timeout exceeded
  else if (manualModeActive && !currentPump.isActive && 
          (millis() - lastJoystickTime > 60000)) {
    manualModeActive = false;
    Serial.println("[MODE] Manual control timeout. Returning to home position.");
    resetSystem();
  }
}
// ================== END UPDATED HANDLER ==================

void setup() {
  Serial.begin(9600);
  Wire.begin(MEGA_I2C_SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  pinMode(X_DIR_PIN, OUTPUT); pinMode(X_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT); pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_SLEEP_PIN, OUTPUT); digitalWrite(Y_SLEEP_PIN, HIGH);
  pinMode(Y_MS1_PIN, OUTPUT); digitalWrite(Y_MS1_PIN, HIGH);
  pinMode(Y_MS2_PIN, OUTPUT); digitalWrite(Y_MS2_PIN, HIGH);
  pinMode(Y_MS3_PIN, OUTPUT); digitalWrite(Y_MS3_PIN, HIGH);

  pinMode(PUMP1_RELAY_PIN, OUTPUT); digitalWrite(PUMP1_RELAY_PIN, LOW);
  pinMode(PUMP2_RELAY_PIN, OUTPUT); digitalWrite(PUMP2_RELAY_PIN, LOW);
  pinMode(MOTOR_PWM_PIN, OUTPUT); analogWrite(MOTOR_PWM_PIN, pwmOut(0));
  
  // Initialize laser pin
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  pinMode(TOUCH_BUTTON_1, INPUT);
  pinMode(TOUCH_BUTTON_2, INPUT);
  pinMode(TOUCH_BUTTON_3, INPUT);
  pinMode(TOUCH_BUTTON_4, INPUT);
  pinMode(TOUCH_SHOOT_BUTTON, INPUT);

  pinMode(POT_PIN, INPUT);  // Initialize potentiometer pin

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);

  // Enhanced boot screen
  tft.fillScreen(COLOR_BG_DARK);

  // Logo and title
  tft.setTextColor(COLOR_PRIMARY);
  tft.setTextSize(2);
  tft.setCursor(25, 30);
  tft.print("FIRELINX");

  // Subtitle
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(1);
  tft.setCursor(30, 55);
  tft.print("Fire Suppression");
  tft.setCursor(45, 65);
  tft.print("System v2.1");

  // Status indicator
  tft.fillCircle(80, 85, 4, COLOR_SUCCESS);
  tft.setCursor(90, 82);
  tft.print("READY");

  // Laser indicator
  tft.fillCircle(80, 100, 4, COLOR_DANGER);
  tft.setCursor(90, 98);
  tft.print("LASER READY");

  // Border decoration
  tft.drawRect(0, 0, 160, 128, COLOR_PRIMARY);
  tft.drawRect(1, 1, 158, 126, COLOR_PRIMARY);

  // Initialize at home position
  currentAngleX = 0.0;
  currentAngleY = 0.0;
  Serial.println("[BOOT] FireLinx System Initialized");
}

bool manualPumpModeActive = false;
unsigned long shootButtonHoldStart = 0;
int selectedType = 0;
int selectedIntensity = 0;
int manualPumpSpeed = 220;  // Default manual pump speed

void drawManualPumpUI() {
  tft.fillScreen(COLOR_BG_DARK);
  drawHeader("MANUAL PUMP CONTROL", COLOR_WARNING);

  if (selectedType == 0) {
    // Fire type selection with enhanced UI
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1);
    tft.setCursor(10, 25);
    tft.print("Select Fire Type:");
    
    // Enhanced fire type buttons
    drawButton(10, 40, 30, 18, "A", COLOR_DANGER, false);
    drawButton(45, 40, 30, 18, "B", COLOR_WARNING, false);
    drawButton(80, 40, 30, 18, "C", COLOR_PRIMARY, false);
    drawButton(115, 40, 30, 18, "D", COLOR_SUCCESS, false);
    
    // Type descriptions with colors
    tft.setTextColor(COLOR_DANGER);
    tft.setTextSize(1);
    tft.setCursor(5, 65);
    tft.print("A:Solid");
    
    tft.setTextColor(COLOR_WARNING);
    tft.setCursor(5, 75);
    tft.print("B:Liquid");
    
    tft.setTextColor(COLOR_PRIMARY);
    tft.setCursor(5, 85);
    tft.print("C:Gas");
    
    tft.setTextColor(COLOR_SUCCESS);
    tft.setCursor(5, 95);
    tft.print("D:Metal");
    
  } else if (selectedIntensity == 0) {
    // Intensity selection with enhanced UI
    char typeStr[25];
    sprintf(typeStr, "Fire Type: %c Selected", 'A' + selectedType - 1);
    
    // Selected type display
    drawStatusCard(10, 25, 140, 20, "SELECTED", typeStr, COLOR_SUCCESS);
    
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1);
    tft.setCursor(10, 50);
    tft.print("Select Intensity Level:");
    
    // Enhanced intensity buttons
    drawButton(5, 65, 35, 18, "LOW", COLOR_SUCCESS, false);
    drawButton(45, 65, 35, 18, "MED", COLOR_WARNING, false);
    drawButton(85, 65, 35, 18, "HIGH", COLOR_ORANGE, false);
    drawButton(125, 65, 30, 18, "MAX", COLOR_DANGER, false);
    
    // Intensity descriptions - UPDATED DURATIONS
    tft.setTextColor(COLOR_GRAY);
    tft.setTextSize(1);
    tft.setCursor(10, 90);
    tft.print("1:60s  2:120s  3:180s  4:240s");
    
  } else {
    // Ready to fire with enhanced display
    char typeStr[5], intensityStr[5], speedStr[10];
    sprintf(typeStr, "%c", 'A' + selectedType - 1);
    sprintf(intensityStr, "%d", selectedIntensity);
    sprintf(speedStr, "%d", manualPumpSpeed);
    
    // Status cards
    drawStatusCard(5, 25, 45, 30, "TYPE", typeStr, COLOR_PRIMARY);
    drawStatusCard(55, 25, 45, 30, "LEVEL", intensityStr, COLOR_WARNING);
    drawStatusCard(105, 25, 50, 30, "SPEED", speedStr, COLOR_SUCCESS);
    
    // Enhanced fire button with pulsing effect
    static unsigned long lastPulse = 0;
    static bool pulseState = false;
    
    if (millis() - lastPulse > 500) {
      pulseState = !pulseState;
      lastPulse = millis();
    }
    
    uint16_t fireColor = pulseState ? COLOR_WHITE : COLOR_DANGER;
    tft.fillRoundRect(25, 70, 110, 25, 5, COLOR_DANGER);
    tft.drawRoundRect(25, 70, 110, 25, 5, fireColor);
    
    // Fire icon and text
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1);
    tft.setCursor(35, 75);
    tft.print("*");
    tft.setCursor(50, 75);
    tft.print("PRESS SHOOT");
    tft.setCursor(55, 85);
    tft.print("TO FIRE!");
  }
}

void handleTouchPumpControl() {
  if (digitalRead(TOUCH_SHOOT_BUTTON) == HIGH) {
    if (!manualPumpModeActive && shootButtonHoldStart == 0) shootButtonHoldStart = millis();
    else if (!manualPumpModeActive && millis() - shootButtonHoldStart >= 2000) {
      manualPumpModeActive = true;
      Serial.println("[PUMP] Manual pump mode activated.");
      drawManualPumpUI();
      shootButtonHoldStart = 0;
    }
  } else {
    shootButtonHoldStart = 0;
  }

  if (!manualPumpModeActive) return;

  if (selectedType == 0) {
    if (digitalRead(TOUCH_BUTTON_1) == HIGH) selectedType = 1;
    else if (digitalRead(TOUCH_BUTTON_2) == HIGH) selectedType = 2;
    else if (digitalRead(TOUCH_BUTTON_3) == HIGH) selectedType = 3;
    else if (digitalRead(TOUCH_BUTTON_4) == HIGH) selectedType = 4;
    if (selectedType) {
      Serial.print("[PUMP] Type selected: "); Serial.println((char)('A'+selectedType-1));
      drawManualPumpUI();
    }
  } else if (selectedIntensity == 0) {
    if (digitalRead(TOUCH_BUTTON_1) == HIGH) selectedIntensity = 1;
    else if (digitalRead(TOUCH_BUTTON_2) == HIGH) selectedIntensity = 2;
    else if (digitalRead(TOUCH_BUTTON_3) == HIGH) selectedIntensity = 3;
    else if (digitalRead(TOUCH_BUTTON_4) == HIGH) selectedIntensity = 4;
    if (selectedIntensity) {
      Serial.print("[PUMP] Intensity selected: "); Serial.println(selectedIntensity);
      drawManualPumpUI();
    }
  } else {
    if (digitalRead(TOUCH_SHOOT_BUTTON) == HIGH) {
      int pin = 0;
      if(selectedType == 1 || selectedType == 2){
        pin = PUMP1_RELAY_PIN;
      }
      else pin = PUMP2_RELAY_PIN;
      
      // USE NEW DURATION MAPPING
      int duration = PUMP_DURATIONS[selectedIntensity - 1];
      
      Serial.println("[PUMP] Firing manually...");
      startPump(pin, duration, manualPumpSpeed, false);  // Manual mode
      manualPumpModeActive = false;
      selectedType = 0;
      selectedIntensity = 0;
    }
  }
}

void loop() {
  // ALWAYS handle joystick first - regardless of any other operations
  handleJoystick();

  // Laser timeout management
  if (laserOn && !currentPump.isActive && (millis() - lastLaserActivity >= LASER_TIMEOUT)) {
    deactivateLaser();
  }

  // Handle pump operations
  if (currentPump.isActive) {
    // Read potentiometer and convert to PWM value (0-255)
    int newSpeed = analogRead(POT_PIN) / 4;
    
    // Apply PWM with proper inversion logic
    analogWrite(MOTOR_PWM_PIN, pwmOut(newSpeed));
    
    // Calculate remaining time
    int remaining = (currentPump.endTime - millis()) / 1000;
    if (remaining < 0) remaining = 0;
    
    // Update display every 200ms - REMOVED REDUNDANT UPDATE (handled in joystick function)
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate >= 500) { // Reduced frequency to prevent conflicts
      updateFiringScreen(newSpeed, remaining);
      lastDisplayUpdate = millis();
    }
    
    // Check if pump operation should end
    if (millis() >= currentPump.endTime) {
      stopPump();
      if (currentPump.isAutomatic) {
        receivedData.dataValid = false;
        newDataAvailable = false;
        resetSystem();
      }
    }
  }  
  else {
    // Handle manual pump control when pump is not active
    manualPumpSpeed = analogRead(POT_PIN) / 4;
    
    // Update manual pump UI if in speed selection stage
    static int lastDisplayedSpeed = -1;
    if (manualPumpModeActive && selectedType && selectedIntensity) {
      if (manualPumpSpeed != lastDisplayedSpeed) {
        tft.fillRect(108, 43, 44, 10, COLOR_BG_LIGHT);
        tft.setTextColor(COLOR_WHITE);
        tft.setCursor(108, 43);
        tft.print(manualPumpSpeed);
        lastDisplayedSpeed = manualPumpSpeed;
      }
    }
    
    handleTouchPumpControl();

    // Process fire alerts if not in manual mode
    if (!manualModeActive && receivedData.dataValid) {
      Serial.println("[AUTO] Fire alert pending. Executing auto response.");
      processFireAlert();
    }
  }
}