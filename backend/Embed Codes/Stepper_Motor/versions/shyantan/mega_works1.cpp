#include <Arduino.h>
#include <Wire.h>               // For ESP32 ↔ Mega I2C
#include <SoftwareWire.h>       // For OLED on custom pins
#include <U8g2lib.h>            // OLED library that supports Software I2C

// ──────────────────────────────────────────────
// OLED CONFIGURATION
// ──────────────────────────────────────────────
#define OLED_SDA_PIN 52
#define OLED_SCL_PIN 51

U8G2_SH1106_128X64_NONAME_F_SW_I2C display(U8G2_R0, OLED_SCL_PIN, OLED_SDA_PIN, U8X8_PIN_NONE);

// ──────────────────────────────────────────────
// FIRE DATA STRUCT & I2C SETTINGS
// ──────────────────────────────────────────────
typedef struct {
  char fireType;
  char fireIntensity;
  char stnID[6];
  float latitude;
  float longitude;
  bool dataValid;
  bool ackValue;
} FireData;

#define I2C_ADDRESS            0x08
#define EXPECTED_DATA_SIZE     12
#define ACKNOWLEDGEMENT_BYTE   0x55
#define ALERT_DISPLAY_DURATION 20000UL

volatile FireData receivedData;
volatile bool newDataAvailable = false;
uint8_t i2c_buffer[EXPECTED_DATA_SIZE];

// ──────────────────────────────────────────────
// CONTROL MODES & TIMERS
// ──────────────────────────────────────────────
#define MODE_ESP32_DIALPAD 1
#define MODE_SMS           2
#define MODE_WEB_CONTROL   3
#define MODE_MANUAL        4

const uint8_t modePriorities[] = {0, 3, 2, 1, 0};

uint8_t activeMode = MODE_MANUAL;
bool manualInputDetected = false;
bool tickVisible = true;
unsigned long lastBlinkTime = 0;
unsigned long modeStartTime = 0;
unsigned long lastDataDisplayTime = 0;

// ──────────────────────────────────────────────
// FORWARD DECLARATIONS
// ──────────────────────────────────────────────
void receiveEvent(int bytesReceived);
void requestEvent();
void drawSystemInfo();
void drawControlModes(bool redraw = false);
void drawStandbyStatus();
void drawAlertInfo(const FireData &data);
void requestModeSwitch(uint8_t newMode);
void switchToManualMode();
void handlePrioritySwitching();
void onManualInputDetected();

// ──────────────────────────────────────────────
// SETUP
// ──────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x12_tr);
  display.drawStr(0, 12, "FireLinx Central Init...");
  display.sendBuffer();

  delay(1000);

  noInterrupts();
  memset(&receivedData, 0, sizeof(receivedData));
  receivedData.dataValid = false;
  receivedData.ackValue = false;
  newDataAvailable = false;
  interrupts();

  drawSystemInfo();
  Serial.println(F("✅ Mega2560 Ready w/ OLED + I2C ESP32"));
}

// ──────────────────────────────────────────────
// LOOP
// ──────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  bool displayNeedsUpdate = false;

  noInterrupts();
  bool hasNew = newDataAvailable;
  if (hasNew) newDataAvailable = false;
  interrupts();

  if (hasNew) {
    noInterrupts();
    bool valid = receivedData.dataValid;
    interrupts();
    if (valid) {
      lastDataDisplayTime = now;
      displayNeedsUpdate = true;
      requestModeSwitch(MODE_ESP32_DIALPAD);
    }
  }

  noInterrupts();
  bool stillValid = receivedData.dataValid;
  interrupts();

  if (stillValid) {
    if (now - lastDataDisplayTime > ALERT_DISPLAY_DURATION) {
      noInterrupts();
      receivedData.dataValid = false;
      interrupts();
      drawStandbyStatus();
      displayNeedsUpdate = false;
    } else if (displayNeedsUpdate) {
      FireData snapshot;
      noInterrupts();
      memcpy(&snapshot, &receivedData, sizeof(snapshot));
      interrupts();
      drawAlertInfo(snapshot);
    }
  } else if (displayNeedsUpdate) {
    drawStandbyStatus();
  }

  if (now - lastBlinkTime > 500) {
    tickVisible = !tickVisible;
    drawControlModes(true);
    lastBlinkTime = now;
  }

  handlePrioritySwitching();
  delay(50);
}

// ──────────────────────────────────────────────
// I2C RECEIVE ISR
// ──────────────────────────────────────────────
void receiveEvent(int howMany) {
  if (howMany != EXPECTED_DATA_SIZE) {
    while (Wire.available()) Wire.read();
    return;
  }
  Wire.readBytes(i2c_buffer, EXPECTED_DATA_SIZE);

  uint8_t chk = 0;
  for (int i = 0; i < EXPECTED_DATA_SIZE - 1; i++) chk ^= i2c_buffer[i];

  noInterrupts();
  if (chk == i2c_buffer[EXPECTED_DATA_SIZE - 1]) {
    uint8_t t = i2c_buffer[0];
    receivedData.fireType = (t >= 1 && t <= 4) ? ('A' + t - 1) : '?';
    uint8_t inten = i2c_buffer[1];
    receivedData.fireIntensity = (inten >= 1 && inten <= 4) ? ('0' + inten) : '?';
    uint8_t zone = i2c_buffer[2];
    if (zone >= 1 && zone <= 26)
      snprintf(receivedData.stnID, sizeof(receivedData.stnID), "%c/%d", 'A' + zone - 1, zone);
    else strcpy(receivedData.stnID, "?/?");
    memcpy(&receivedData.latitude, &i2c_buffer[3], sizeof(float));
    memcpy(&receivedData.longitude, &i2c_buffer[7], sizeof(float));
    receivedData.dataValid = true;
    receivedData.ackValue = true;
    newDataAvailable = true;
  }
  interrupts();
}

// ──────────────────────────────────────────────
// I2C REQUEST ISR
// ──────────────────────────────────────────────
void requestEvent() {
  Wire.write(receivedData.ackValue ? ACKNOWLEDGEMENT_BYTE : 0x00);
  receivedData.ackValue = false;
}

// ──────────────────────────────────────────────
// UI DRAWING
// ──────────────────────────────────────────────
void drawSystemInfo() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x12_tr);
  display.drawStr(0, 0, "FireLinx Central Control");
  drawControlModes(false);
  drawStandbyStatus();
}

void drawControlModes(bool redraw) {
  if (redraw) {
    display.clearBuffer();
    display.setFont(u8g2_font_6x12_tr);
    display.drawStr(0, 0, "FireLinx Central Control");
  }
  const char *modes[4] = {"ESP32", "SMS", "Web", "Manual"};
  for (int i = 0; i < 4; i++) {
    char label[16];
    snprintf(label, sizeof(label), "%s%s", (i + 1 == activeMode && tickVisible) ? ">" : " ", modes[i]);
    display.drawStr((i % 2) * 64, 12 + (i / 2) * 12, label);
  }
  display.sendBuffer();
}

void drawStandbyStatus() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x12_tr);
  display.drawStr(0, 32, "STANDBY");
  display.drawStr(0, 44, "Awaiting Fire Alert...");
  display.sendBuffer();
}

void drawAlertInfo(const FireData &d) {
  display.clearBuffer();
  display.setFont(u8g2_font_6x12_tr);
  display.drawStr(0, 0, "!! FIRE ALERT !!");

  char line[32];
  snprintf(line, sizeof(line), "Stn: %s", d.stnID);
  display.drawStr(0, 12, line);

  snprintf(line, sizeof(line), "T:%c I:%c", d.fireType, d.fireIntensity);
  display.drawStr(0, 24, line);

  snprintf(line, sizeof(line), "Lat: %.2f", d.latitude);
  display.drawStr(0, 36, line);

  snprintf(line, sizeof(line), "Lon: %.2f", d.longitude);
  display.drawStr(0, 48, line);

  display.sendBuffer();
}

// ──────────────────────────────────────────────
// MODE CONTROL
// ──────────────────────────────────────────────
void requestModeSwitch(uint8_t m) {
  if (modePriorities[m] > modePriorities[activeMode]) {
    activeMode = m;
    modeStartTime = millis();
    tickVisible = true;
    drawControlModes(true);
  } else if (m == activeMode) {
    modeStartTime = millis();
  }
}

void switchToManualMode() {
  if (activeMode != MODE_MANUAL) {
    activeMode = MODE_MANUAL;
    modeStartTime = millis();
    tickVisible = true;
    drawControlModes(true);
    noInterrupts();
    bool valid = receivedData.dataValid;
    interrupts();
    if (!valid) drawStandbyStatus();
  }
}

void handlePrioritySwitching() {
  if (manualInputDetected) {
    manualInputDetected = false;
    modeStartTime = millis();
    return;
  }
  if (activeMode != MODE_MANUAL && millis() - modeStartTime >= 420000UL) {
    switchToManualMode();
  }
}

void onManualInputDetected() {
  manualInputDetected = true;
}
