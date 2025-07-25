#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// TFT Pin Configuration
#define TFT_SCLK 52
#define TFT_MOSI 51
#define TFT_CS   10
#define TFT_RST  8
#define TFT_DC   9

#define TFT_WIDTH  160
#define TFT_HEIGHT 128

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// I2C Configuration
#define MEGA_I2C_SLAVE_ADDRESS 0x08
#define EXPECTED_DATA_SIZE 12
#define ACKNOWLEDGEMENT_BYTE 0x55

// === PUMP CONTROL DEFINITIONS ===
const int PUMP1_RELAY_PIN = 30;
const int PUMP2_RELAY_PIN = 31;
const int MOTOR_PWM_PIN = 6;

// --- PWM Configuration ---
const bool PWM_LOGIC_INVERTED = true; // True: 0 is MAX speed, 255 is OFF. False: 0 is OFF, 255 is MAX speed.
const int INTUITIVE_PWM_MIN_OPERATIONAL = 150; // User-perceived minimum operational speed
const int INTUITIVE_PWM_DEFAULT = 220;         // User-perceived default speed
const int INTUITIVE_PWM_MAX_OPERATIONAL = 255; // User-perceived maximum speed

// Touch Input Pins
#define TOUCH_BUTTON_1 43
#define TOUCH_BUTTON_2 42
#define TOUCH_BUTTON_3 41
#define TOUCH_BUTTON_4 40
#define TOUCH_SHOOT_BUTTON 39

// Add these constants near top of file
#define MANUAL_ACTIVATION_TIME 5000  // 5 seconds for manual mode activation
#define TOUCH_DEBOUNCE_TIME 50       // 50ms debounce time (increased from 20 for stability)
#define SPEED_ADJUST_INTERVAL 300    // 300ms between speed adjustments

// Structure to hold processed alert data
typedef struct {
    char fireType;
    char fireIntensity;
    char stnID[6];
    float latitude;
    float longitude;
    bool dataValid; // Is the data structure itself filled and checksummed correctly by ISR?
    bool ackValue;  // For I2C master acknowledgement
} FireData;

volatile FireData receivedData;     // Data received from I2C ISR
volatile bool newDataAvailable = false; // Flag set by ISR when new data is ready

// Pump Operation State Variables
volatile bool pumpSystemActive = false;                 // True if pumps are in a timed operational cycle
volatile unsigned long pumpOperationEndTime = 0;        // Absolute end time for pump operation

// Manual Control State
enum ManualControlState {
  MANUAL_INACTIVE,
  MANUAL_SELECT_TYPE,
  MANUAL_SELECT_INTENSITY,
  MANUAL_READY_TO_ACTIVATE,
  MANUAL_OPERATION_ACTIVE
};

volatile ManualControlState manualState = MANUAL_INACTIVE;
volatile char manualSelectedFireType = 'A'; // Default to A
volatile char manualSelectedIntensity = '3'; // Default to medium intensity
volatile int manualCurrentSpeed = INTUITIVE_PWM_DEFAULT; // Default speed
volatile unsigned long manualTouchStartTime = 0; // For long press detection
volatile bool manualTouchActive = false; // True if shoot button is currently pressed (for long/short press logic)
volatile unsigned long lastSpeedAdjustTime = 0; // For debouncing speed adjustments
volatile bool manualModeJustActivated = false; // Flag to trigger initial manual UI draw

// Color Definitions
#define COLOR_BLACK     0x0000
#define COLOR_NAVY      0x000F
#define COLOR_DARKGREEN 0x03E0
#define COLOR_DARKCYAN  0x03EF
#define COLOR_MAROON    0x7800
#define COLOR_PURPLE    0x780F
#define COLOR_OLIVE     0x7BE0
#define COLOR_LIGHTGREY 0xC618
#define COLOR_DARKGREY  0x7BEF
#define COLOR_BLUE      0x001F
#define COLOR_GREEN     0x07E0
#define COLOR_CYAN      0x07FF
#define COLOR_RED       0xF800
#define COLOR_MAGENTA   0xF81F
#define COLOR_YELLOW    0xFFE0
#define COLOR_WHITE     0xFFFF
#define COLOR_ORANGE    0xFD20
#define COLOR_GREENYELLOW 0xAFE5
#define COLOR_PINK      0xF81F

// Mode Control
#define MODE_ESP32_DIALPAD 1
#define MODE_SMS           2
#define MODE_WEB_CONTROL   3
#define MODE_MANUAL        4

uint8_t activeMode = MODE_MANUAL; // Start in Manual Inactive
bool tickVisible = true;
unsigned long lastBlinkTime = 0;
unsigned long modeStartTime = 0;
bool manualInputDetected = false; // Used for priority switching timeout reset
const uint8_t modePriorities[] = {0, 3, 2, 1, 0}; // 0=placeholder, ESP32=3, SMS=2, Web=1, Manual=0 (for auto-switch out)

uint8_t i2c_buffer[EXPECTED_DATA_SIZE]; // Buffer for I2C data

// Layout Defines
#define TITLE_Y 5
#define MODES_Y (TITLE_Y + 15)
#define ALERT_AREA_Y (MODES_Y + 45)
#define ALERT_AREA_HEIGHT (TFT_HEIGHT - ALERT_AREA_Y)

// --- Function Prototypes ---
void drawSystemInfo();
void drawControlModes(bool redraw);
void drawStandbyStatus();
void drawAlertInfo(const FireData& data);
void handlePrioritySwitching();
void requestModeSwitch(uint8_t newMode);
void switchToManualMode(); 
void receiveEvent(int bytesReceived);
void printReceivedData();
void requestEvent();

void initializePumps();
void initializeTouchInputs();
void activatePumpSystem(const FireData& alertData, unsigned long durationMs);
void deactivatePumpSystem();
unsigned long calculateOperationDurationMs(char fireType, char intensity);
int getIntuitivePwmForIntensity(char intensity);
int getActualPwmToWrite(int intuitiveSpeed);

// Manual Control Specific Function Prototypes
void resetManualControlState();
void enterManualMode(); 
void handleManualTouchInputs();
void processManualButton(int buttonNum);
int calculateInitialSpeed(char fireType, char intensity);
void processShootButtonPress();
void drawManualControlInterface();


void setup() {
    Serial.begin(115200);
    
    Wire.begin(MEGA_I2C_SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    noInterrupts();
    memset((void*)&receivedData, 0, sizeof(receivedData));
    receivedData.dataValid = false;
    receivedData.ackValue = false;
    newDataAvailable = false;

    pumpSystemActive = false;
    pumpOperationEndTime = 0;
    
    manualState = MANUAL_INACTIVE; 
    manualModeJustActivated = true; 
    interrupts();

    tft.initR(INITR_BLACKTAB);
    tft.setRotation(1); 
    tft.setTextWrap(false);

    initializePumps();
    initializeTouchInputs();
    drawSystemInfo(); 
    modeStartTime = millis();
    Serial.println(F("ST7735 Initialized"));
    Serial.println(F("STAFERB Mega (I2C Slave) Ready (Addr 0x08)"));
    Serial.println("PWM Logic is " + String(PWM_LOGIC_INVERTED ? "INVERTED (0=MAX)" : "NORMAL (255=MAX)"));
}

void loop() {
    unsigned long currentTime = millis();
    bool displayNeedsUpdate = false;

    noInterrupts();
    bool processDataNow = newDataAvailable;
    if (processDataNow) {
        newDataAvailable = false;
    }
    interrupts();

    if (processDataNow) {
        Serial.println("Loop: New data flag. Processing...");
        FireData localAlertData;
        bool isDataStructValid;

        noInterrupts();
        memcpy(&localAlertData, (const void*)&receivedData, sizeof(FireData));
        isDataStructValid = receivedData.dataValid;
        interrupts();

        if (isDataStructValid) {
            printReceivedData();
            unsigned long calculatedDuration = calculateOperationDurationMs(localAlertData.fireType, localAlertData.fireIntensity);

            if (calculatedDuration > 0) {
                Serial.print("Loop: Valid alert. Duration: ");
                Serial.print(calculatedDuration / 1000); Serial.println("s. Activating pumps.");
                activatePumpSystem(localAlertData, calculatedDuration);
                requestModeSwitch(MODE_ESP32_DIALPAD); 
                displayNeedsUpdate = true;
            } else {
                Serial.println("Loop: Valid alert data, but duration is 0. Pumps NOT activated/stopped.");
                deactivatePumpSystem(); 
                displayNeedsUpdate = true; 
            }
        } else {
            Serial.println("Loop: ISR signaled new data, BUT struct invalid. Ensuring pumps OFF.");
            deactivatePumpSystem();
            noInterrupts();
            receivedData.dataValid = false; 
            interrupts();
        }
    }

    noInterrupts();
    bool pumpsShouldBeActive = pumpSystemActive;
    unsigned long currentPumpOpEndTime = pumpOperationEndTime; // Local copy
    interrupts();

    if (pumpsShouldBeActive) {
        if (currentTime >= currentPumpOpEndTime) {
            Serial.println(F("Pump operation EXPIRED. Deactivating."));
            deactivatePumpSystem();
            if (activeMode == MODE_MANUAL) {
                resetManualControlState(); 
                manualState = MANUAL_INACTIVE; 
                drawManualControlInterface(); 
            }
            noInterrupts();
            // Consider if receivedData.dataValid should be cleared here.
            // If an alert caused this, it's now "handled".
            // For now, let it persist for display until a new event.
            // receivedData.dataValid = false;
            interrupts();
            displayNeedsUpdate = true; 
        }
    }

    if (displayNeedsUpdate) { 
        bool alertIsCurrentlyValidForDisplay;
        FireData dataToDisplay;

        noInterrupts();
        alertIsCurrentlyValidForDisplay = receivedData.dataValid;
        memcpy(&dataToDisplay, (const void*)&receivedData, sizeof(FireData));
        interrupts();

        if (activeMode != MODE_MANUAL) { 
            if (alertIsCurrentlyValidForDisplay) {
                drawAlertInfo(dataToDisplay);
            } else {
                drawStandbyStatus();
            }
        }
    }
    
    if (activeMode == MODE_MANUAL && manualModeJustActivated) {
        drawManualControlInterface();
        manualModeJustActivated = false;
    }

    if (currentTime - lastBlinkTime > 500) {
        tickVisible = !tickVisible;
        bool alertIsOnScreen; 
        noInterrupts();
        alertIsOnScreen = receivedData.dataValid;
        interrupts();
 
        if (!alertIsOnScreen || activeMode == MODE_MANUAL) { // Update modes if no alert OR in manual mode (modes are above manual UI)
             drawControlModes(true); 
        }
        lastBlinkTime = currentTime;
    }

    handlePrioritySwitching();
    handleManualTouchInputs(); 
}

void initializePumps() {
    pinMode(PUMP1_RELAY_PIN, OUTPUT);
    pinMode(PUMP2_RELAY_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    deactivatePumpSystem();
    Serial.println("Pumps initialized and OFF.");
}

void initializeTouchInputs() {
    pinMode(TOUCH_BUTTON_1, INPUT);
    pinMode(TOUCH_BUTTON_2, INPUT);
    pinMode(TOUCH_BUTTON_3, INPUT);
    pinMode(TOUCH_BUTTON_4, INPUT);
    pinMode(TOUCH_SHOOT_BUTTON, INPUT);
    Serial.println("Touch inputs initialized");
}

int getActualPwmToWrite(int intuitiveSpeed) {
    int speed = constrain(intuitiveSpeed, 0, 255); 
    if (PWM_LOGIC_INVERTED) {
        return 255 - speed;
    } else {
        return speed;
    }
}

void deactivatePumpSystem() {
    digitalWrite(PUMP1_RELAY_PIN, LOW);
    digitalWrite(PUMP2_RELAY_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, getActualPwmToWrite(0)); 

    noInterrupts();
    pumpSystemActive = false;
    pumpOperationEndTime = 0;
    interrupts();

    Serial.println("Pump system DEACTIVATED.");
}

int getIntuitivePwmForIntensity(char intensity) {
    switch (intensity) {
        case '1': return INTUITIVE_PWM_MIN_OPERATIONAL;
        case '2': return (INTUITIVE_PWM_MIN_OPERATIONAL + INTUITIVE_PWM_DEFAULT) / 2;
        case '3': return INTUITIVE_PWM_DEFAULT;
        case '4': return INTUITIVE_PWM_MAX_OPERATIONAL;
        case '?': default:
            Serial.println("getIntuitivePwmForIntensity: '?' or unknown intensity, using default.");
            return INTUITIVE_PWM_DEFAULT;
    }
}

unsigned long calculateOperationDurationMs(char fireType, char intensity) {
    int baseSeconds = 0;

    if (fireType == '?' || intensity == '?') {
        Serial.print("CalcDuration: Type '"); Serial.print(fireType);
        Serial.print("' or Intensity '"); Serial.print(intensity);
        Serial.println("' is '?', duration 0ms.");
        return 0;
    }

    switch (fireType) {
        case 'A':
            switch (intensity) {
                case '1': baseSeconds = 15; break; case '2': baseSeconds = 25; break;
                case '3': baseSeconds = 35; break; case '4': baseSeconds = 40; break;
                default: Serial.print("CalcDuration: Unknown intensity '"); Serial.print(intensity); Serial.println("' for Type A. Duration 0ms."); return 0;
            } break;
        case 'B':
            switch (intensity) {
                case '1': baseSeconds = 10; break; case '2': baseSeconds = 20; break;
                case '3': baseSeconds = 30; break; case '4': baseSeconds = 35; break;
                default: Serial.print("CalcDuration: Unknown intensity '"); Serial.print(intensity); Serial.println("' for Type B. Duration 0ms."); return 0;
            } break;
        case 'C': 
            switch (intensity) {
                case '1': baseSeconds = 10; break; case '2': baseSeconds = 15; break;
                case '3': baseSeconds = 25; break; case '4': baseSeconds = 30; break;
                default: Serial.print("CalcDuration: Unknown intensity '"); Serial.print(intensity); Serial.println("' for Type C. Duration 0ms."); return 0;
            } break;
        case 'D': 
            switch (intensity) {
                case '1': baseSeconds = 15; break; case '2': baseSeconds = 25; break;
                case '3': baseSeconds = 35; break; case '4': baseSeconds = 40; break;
                default: Serial.print("CalcDuration: Unknown intensity '"); Serial.print(intensity); Serial.println("' for Type D. Duration 0ms."); return 0;
            } break;
        default:
            Serial.print("CalcDuration: Unknown FireType '"); Serial.print(fireType); Serial.println("'. Duration 0ms.");
            return 0;
    }
    return (unsigned long)baseSeconds * 1000;
}

void activatePumpSystem(const FireData& alertData, unsigned long durationMs) {
    int intuitiveSpeed;
    // For I2C alerts, or if manual mode is not yet in OPERATION_ACTIVE (e.g. initial activation)
    if (!(activeMode == MODE_MANUAL && manualState == MANUAL_OPERATION_ACTIVE)) {
        intuitiveSpeed = getIntuitivePwmForIntensity(alertData.fireIntensity);
    } else {
        // If pumps are being (re)activated or speed adjusted while already in manual operation, use manualCurrentSpeed
        intuitiveSpeed = manualCurrentSpeed;
    }
    
    int actualPwm = getActualPwmToWrite(intuitiveSpeed);
    int selectedPumpRelay = 0;

    switch (alertData.fireType) {
        case 'A': case 'C': 
            selectedPumpRelay = PUMP1_RELAY_PIN;
            Serial.print("ActivatePump: Type "); Serial.print(alertData.fireType); Serial.println(" -> Pump 1.");
            break;
        case 'B': case 'D': 
            selectedPumpRelay = PUMP2_RELAY_PIN;
            Serial.print("ActivatePump: Type "); Serial.print(alertData.fireType); Serial.println(" -> Pump 2.");
            break;
        default:
            Serial.print("ActivatePump: ERROR - Invalid FireType '"); Serial.print(alertData.fireType);
            Serial.println("'. Deactivating.");
            deactivatePumpSystem();
            return;
    }

    if (selectedPumpRelay == PUMP1_RELAY_PIN) {
        digitalWrite(PUMP2_RELAY_PIN, LOW); 
        digitalWrite(PUMP1_RELAY_PIN, HIGH);
        Serial.print("Pump 1 ON. ");
    } else if (selectedPumpRelay == PUMP2_RELAY_PIN) {
        digitalWrite(PUMP1_RELAY_PIN, LOW); 
        digitalWrite(PUMP2_RELAY_PIN, HIGH);
        Serial.print("Pump 2 ON. ");
    } else {
        Serial.println("ActivatePump: ERROR - No pump relay selected. Deactivating.");
        deactivatePumpSystem();
        return;
    }

    analogWrite(MOTOR_PWM_PIN, actualPwm);
    Serial.print("Intuitive Speed: "); Serial.print(intuitiveSpeed);
    Serial.print(" -> Actual PWM: "); Serial.println(actualPwm);

    noInterrupts();
    pumpOperationEndTime = millis() + durationMs;
    pumpSystemActive = true;
    interrupts();

    Serial.print("Pump system ACTIVATED until ");
    Serial.print(pumpOperationEndTime);
    Serial.print(" ms (");
    Serial.print(durationMs / 1000);
    Serial.println(" seconds).");
}

void receiveEvent(int bytesReceived) {
    noInterrupts();
    receivedData.ackValue = false; // Assume failure until success
    bool success = false;

    if (bytesReceived == EXPECTED_DATA_SIZE) {
        int bytesRead = Wire.readBytes(i2c_buffer, EXPECTED_DATA_SIZE);
        if (bytesRead == EXPECTED_DATA_SIZE) {
            uint8_t calculatedChecksum = 0;
            for (int i = 0; i < EXPECTED_DATA_SIZE - 1; i++) {
                calculatedChecksum ^= i2c_buffer[i];
            }

            if (calculatedChecksum == i2c_buffer[EXPECTED_DATA_SIZE - 1]) {
                uint8_t typeVal = i2c_buffer[0];
                if (typeVal >= 1 && typeVal <= 4) receivedData.fireType = 'A' + typeVal - 1;
                else receivedData.fireType = '?'; 

                uint8_t intensityVal = i2c_buffer[1];
                if (intensityVal >= 1 && intensityVal <= 4) receivedData.fireIntensity = '0' + intensityVal;
                else receivedData.fireIntensity = '?'; 

                uint8_t zoneVal = i2c_buffer[2]; 
                if (zoneVal >= 0 && zoneVal <= 25) { 
                     snprintf(receivedData.stnID, sizeof(receivedData.stnID), "%c/%d", 'A' + zoneVal, zoneVal + 1);
                } else {
                     snprintf(receivedData.stnID, sizeof(receivedData.stnID), "??");
                }
                receivedData.stnID[sizeof(receivedData.stnID)-1] = '\0';

                memcpy((void*)&receivedData.latitude, &i2c_buffer[3], sizeof(float));
                memcpy((void*)&receivedData.longitude, &i2c_buffer[7], sizeof(float));

                receivedData.dataValid = true;
                receivedData.ackValue = true; 
                newDataAvailable = true;      
                success = true;
                Serial.println(F("I2C RX: Valid data received & parsed."));
            } else {
                Serial.println(F("I2C RX: Checksum FAILED!"));
            }
        } else {
            Serial.println(F("I2C RX: ReadBytes count mismatch."));
        }
    } else {
        Serial.print(F("I2C RX: Unexpected byte count: ")); Serial.println(bytesReceived);
        while (Wire.available()) Wire.read(); 
    }
    
    if (!success) {
        receivedData.dataValid = false; 
    }
    interrupts();
}

void requestEvent() {
    noInterrupts();
    bool ackToSend = receivedData.ackValue;
    interrupts();
    Wire.write(ackToSend ? ACKNOWLEDGEMENT_BYTE : 0x00);
}

void printReceivedData() {
    FireData dataToPrint; 
    noInterrupts();
    memcpy(&dataToPrint, (const void*)&receivedData, sizeof(FireData));
    bool isValid = receivedData.dataValid; 
    interrupts();

    if (isValid) {
        Serial.println(F("\n--- Parsed I2C Data (Alert) ---"));
        Serial.print(F("Fire Type: ")); Serial.println(dataToPrint.fireType);
        Serial.print(F("Intensity: ")); Serial.println(dataToPrint.fireIntensity);
        Serial.print(F("Station ID: ")); Serial.println(dataToPrint.stnID);
        Serial.print(F("Latitude: ")); Serial.println(dataToPrint.latitude, 6);
        Serial.print(F("Longitude: ")); Serial.println(dataToPrint.longitude, 6);
        Serial.println(F("-------------------------------"));
    } else {
        Serial.println(F("PrintReceivedData: Data invalid or not available."));
    }
}

void drawSystemInfo() {
    tft.fillScreen(COLOR_NAVY);
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1);
    tft.setCursor(5, TITLE_Y);
    tft.print("FireLinx Central Control");
    tft.drawFastHLine(0, TITLE_Y + 10, TFT_WIDTH, COLOR_DARKGREY);
    drawControlModes(false); 

    bool initialAlertState;
    noInterrupts();
    initialAlertState = receivedData.dataValid;
    interrupts();

    if (activeMode != MODE_MANUAL) {
        if (!initialAlertState) {
            drawStandbyStatus();
        } else {
            FireData initialData;
            noInterrupts();
            memcpy(&initialData, (const void*)&receivedData, sizeof(FireData));
            interrupts();
            drawAlertInfo(initialData);
        }
    }
    // If activeMode is MODE_MANUAL, manualModeJustActivated flag will trigger drawManualControlInterface in loop()
}

void drawControlModes(bool redraw) {
    int16_t startY = MODES_Y;
    int16_t areaHeight = 40; 
    int16_t col1X = 5;
    int16_t col2X = TFT_WIDTH / 2 + 5; 
    int16_t row1Y = startY + 12;
    int16_t row2Y = row1Y + 12;
    int16_t tickOffsetX = 60; // Adjusted for typical text length

    if (redraw) { 
        tft.fillRect(0, startY, TFT_WIDTH, areaHeight, COLOR_NAVY);
    }
    tft.setTextSize(1);
    tft.setTextColor(COLOR_WHITE);
    tft.setCursor(col1X, startY); 
    tft.print("Modes:");
    
    const char* modes[] = {"ESP32 Alert", "SMS Cmd", "Web Control", "Manual Ctrl"};
    int16_t xPos[] = {col1X, col2X, col1X, col2X}; 
    int16_t yPos[] = {row1Y, row1Y, row2Y, row2Y}; 

    for (uint8_t i = 0; i < 4; i++) {
        tft.setCursor(xPos[i], yPos[i]);
        tft.setTextColor((i + 1 == activeMode) ? COLOR_YELLOW : COLOR_WHITE);
        tft.print(modes[i]);
        if (i + 1 == activeMode && tickVisible) {
            tft.setTextColor(COLOR_WHITE); 
            tft.setCursor(xPos[i] + tickOffsetX, yPos[i]); 
            tft.print("<=");
        }
    }
    if (redraw) { 
      tft.drawFastHLine(0, startY + areaHeight -1 , TFT_WIDTH, COLOR_DARKGREY);
    }
}

void drawStandbyStatus() {
    tft.fillRect(0, ALERT_AREA_Y, TFT_WIDTH, ALERT_AREA_HEIGHT, COLOR_NAVY);
    tft.setCursor(5, ALERT_AREA_Y + 5);
    tft.setTextColor(COLOR_GREEN);
    tft.setTextSize(2);
    tft.println(" STANDBY");
    tft.setCursor(5, ALERT_AREA_Y + 25);
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1);
    tft.println("Awaiting Fire Alert...");
}

void drawAlertInfo(const FireData& data) {
    uint16_t bgColor;
    uint16_t textColor = COLOR_BLACK; 

    switch(data.fireIntensity) {
        case '1': bgColor = COLOR_GREEN; textColor = COLOR_BLACK; break;
        case '2': bgColor = COLOR_YELLOW; textColor = COLOR_BLACK; break;
        case '3': bgColor = COLOR_ORANGE; textColor = COLOR_WHITE; break;
        case '4': bgColor = COLOR_RED; textColor = COLOR_WHITE; break;
        case '?': 
        default:  bgColor = COLOR_MAGENTA; textColor = COLOR_WHITE; break;
    }

    tft.fillRect(0, ALERT_AREA_Y, TFT_WIDTH, ALERT_AREA_HEIGHT, bgColor);
    tft.setTextColor(textColor);

    int y = ALERT_AREA_Y + 5;
    int x = 5;
    tft.setCursor(x, y); tft.setTextSize(2); tft.print("FIRE ALERT!"); tft.setTextSize(1); y += 20;

    tft.setCursor(x, y); tft.print("Stn:"); tft.print(data.stnID);
    tft.print(" Typ:"); tft.print(data.fireType);
    tft.print(" Int:"); tft.println(data.fireIntensity); y += 12;

    tft.setCursor(x, y);
    if (abs(data.latitude) > 0.0001 || abs(data.longitude) > 0.0001) { 
        tft.print("Lat: "); tft.print(data.latitude, 4);
        tft.setCursor(x + TFT_WIDTH/2 -5, y); 
        tft.print("Lon: "); tft.println(data.longitude, 4);
    } else {
         tft.println("Location: N/A");
    }
}

void handlePrioritySwitching() {
    if (activeMode == MODE_MANUAL) {
        return;
    }

    unsigned long elapsedTime = millis() - modeStartTime;
    const unsigned long MODE_TIMEOUT_MS = 420000; // 7 minutes

    if (elapsedTime >= MODE_TIMEOUT_MS) {
        if (!manualInputDetected) { 
            Serial.print(F("Mode ")); Serial.print(activeMode); Serial.println(F(" Timeout -> Manual"));
            switchToManualMode(); 
        } else {
            Serial.print(F("Activity detected in Mode ")); Serial.print(activeMode);
            Serial.println(F(". Resetting mode timeout."));
            modeStartTime = millis(); 
            manualInputDetected = false; 
        }
    }
}

void requestModeSwitch(uint8_t newMode) {
    if (newMode < 1 || newMode > 4) {
         Serial.print(F("Invalid mode switch requested: ")); Serial.println(newMode);
         return;
    }
    
    if (modePriorities[newMode-1] > modePriorities[activeMode-1] || (activeMode == MODE_MANUAL && newMode != MODE_MANUAL)) {
        if (activeMode != newMode) {
            Serial.print(F("Switching from mode ")); Serial.print(activeMode);
            Serial.print(F(" to mode: ")); Serial.println(newMode);
            
            if (newMode == MODE_MANUAL) {
                switchToManualMode(); 
            } else {
                activeMode = newMode;
                manualModeJustActivated = false; 
            }
            manualInputDetected = false; 
            modeStartTime = millis();    
            drawControlModes(true);      

            if (newMode != MODE_MANUAL) {
                bool alertIsOnScreen; FireData currentAlertData;
                noInterrupts();
                alertIsOnScreen = receivedData.dataValid;
                memcpy(&currentAlertData, (const void*)&receivedData, sizeof(FireData));
                interrupts();
                if (alertIsOnScreen) drawAlertInfo(currentAlertData); else drawStandbyStatus();
            }
        } else { 
             Serial.print(F("Mode ")); Serial.print(newMode); Serial.println(F(" re-activated. Resetting timeout."));
             modeStartTime = millis();
             manualInputDetected = false;
        }
    } else {
        Serial.print(F("Ignoring lower priority mode switch: ")); Serial.print(newMode);
        Serial.print(F(" (Active: ")); Serial.print(activeMode); Serial.println(F(")"));
    }
}

void switchToManualMode() {
    if (activeMode != MODE_MANUAL) {
        Serial.println(F("Switching to Manual Mode."));
    } else {
        Serial.println(F("Already in Manual Mode. Re-initializing."));
    }
    enterManualMode(); 
    manualInputDetected = false; 
    modeStartTime = millis();    
    drawControlModes(true); 
    // manualModeJustActivated is set by enterManualMode(), loop() will draw the UI.
}

void resetManualControlState() {
  manualState = MANUAL_INACTIVE;
  manualSelectedFireType = 'A'; 
  manualSelectedIntensity = '3';
  manualCurrentSpeed = INTUITIVE_PWM_DEFAULT; 
  manualTouchActive = false;    
  Serial.println("Manual control state reset.");
}

void enterManualMode() {
  resetManualControlState(); 
  manualState = MANUAL_SELECT_TYPE; 
  
  if (activeMode != MODE_MANUAL) { 
    Serial.println("Entering Manual Mode...");
  }
  activeMode = MODE_MANUAL;
  manualModeJustActivated = true; 
}

void handleManualTouchInputs() {
  static unsigned long lastNumberButtonPressTime = 0;
  static unsigned long lastShootButtonPressTime = 0;

  if (digitalRead(TOUCH_SHOOT_BUTTON) == HIGH) {
    if (!manualTouchActive) { 
      manualTouchActive = true;
      manualTouchStartTime = millis();
      if(activeMode != MODE_MANUAL) manualInputDetected = true; // Activity for other modes timeout
    } else { 
      if (millis() - manualTouchStartTime >= MANUAL_ACTIVATION_TIME) {
        if (activeMode != MODE_MANUAL || manualState == MANUAL_INACTIVE) {
             enterManualMode(); 
        }
        manualTouchActive = false; 
        lastShootButtonPressTime = millis(); 
        return; 
      }
    }
  } else { 
    if (manualTouchActive) { 
      if (millis() - manualTouchStartTime < MANUAL_ACTIVATION_TIME) { 
        if (activeMode == MODE_MANUAL) { 
          if (millis() - lastShootButtonPressTime > TOUCH_DEBOUNCE_TIME) { 
            processShootButtonPress(); 
            lastShootButtonPressTime = millis();
          }
        }
      }
      manualTouchActive = false; 
    }
  }

  if (activeMode != MODE_MANUAL) {
    if (digitalRead(TOUCH_BUTTON_1) == HIGH || digitalRead(TOUCH_BUTTON_2) == HIGH ||
        digitalRead(TOUCH_BUTTON_3) == HIGH || digitalRead(TOUCH_BUTTON_4) == HIGH) {
        if(activeMode != MODE_MANUAL) manualInputDetected = true;
    }
    return;
  }

  if (millis() - lastNumberButtonPressTime > TOUCH_DEBOUNCE_TIME) {
    bool numberButtonPressed = false;
    if (digitalRead(TOUCH_BUTTON_1) == HIGH) {
      processManualButton(1); numberButtonPressed = true;
    } else if (digitalRead(TOUCH_BUTTON_2) == HIGH) {
      processManualButton(2); numberButtonPressed = true;
    } else if (digitalRead(TOUCH_BUTTON_3) == HIGH) {
      processManualButton(3); numberButtonPressed = true;
    } else if (digitalRead(TOUCH_BUTTON_4) == HIGH) {
      processManualButton(4); numberButtonPressed = true;
    }
    if (numberButtonPressed) {
      lastNumberButtonPressTime = millis(); 
    }
  }
}

void processManualButton(int buttonNum) { 
  if (activeMode != MODE_MANUAL) return;

  switch (manualState) {
    case MANUAL_SELECT_TYPE:
      if (buttonNum >= 1 && buttonNum <= 4) { 
        manualSelectedFireType = 'A' + (buttonNum - 1);
        manualState = MANUAL_SELECT_INTENSITY; 
        Serial.print("Manual: Selected type: "); Serial.println(manualSelectedFireType);
      }
      break;
    case MANUAL_SELECT_INTENSITY:
      if (buttonNum >= 1 && buttonNum <= 4) { 
        manualSelectedIntensity = '0' + buttonNum;
        manualCurrentSpeed = calculateInitialSpeed(manualSelectedFireType, manualSelectedIntensity); 
        manualState = MANUAL_READY_TO_ACTIVATE; 
        Serial.print("Manual: Selected intensity: "); Serial.println(manualSelectedIntensity);
        Serial.print("Manual: Initial speed: "); Serial.println(manualCurrentSpeed);
      }
      break;
    case MANUAL_OPERATION_ACTIVE: 
      if (millis() - lastSpeedAdjustTime > SPEED_ADJUST_INTERVAL) { 
        int oldSpeed = manualCurrentSpeed;
        switch (buttonNum) { 
          case 1: manualCurrentSpeed += 10; break; 
          case 2: manualCurrentSpeed -= 10; break; 
          case 3: manualCurrentSpeed = INTUITIVE_PWM_DEFAULT; break; 
          case 4: manualCurrentSpeed = INTUITIVE_PWM_MAX_OPERATIONAL; break; 
        }
        manualCurrentSpeed = constrain(manualCurrentSpeed, INTUITIVE_PWM_MIN_OPERATIONAL, INTUITIVE_PWM_MAX_OPERATIONAL);
        if (oldSpeed != manualCurrentSpeed) {
            lastSpeedAdjustTime = millis();
            if (pumpSystemActive) { 
                analogWrite(MOTOR_PWM_PIN, getActualPwmToWrite(manualCurrentSpeed));
            }
            Serial.print("Manual: Speed adjusted to: "); Serial.println(manualCurrentSpeed);
        }
      }
      break;
    default: break;
  }
  drawManualControlInterface(); 
}

int calculateInitialSpeed(char fireType, char intensity) {
  int baseSpeed = getIntuitivePwmForIntensity(intensity); 
  // Custom adjustments per fireType can be added here if needed
  // switch (fireType) {
  //   case 'A': baseSpeed = min(INTUITIVE_PWM_MAX_OPERATIONAL, baseSpeed + 5); break;
  // }
  return constrain(baseSpeed, INTUITIVE_PWM_MIN_OPERATIONAL, INTUITIVE_PWM_MAX_OPERATIONAL);
}

void processShootButtonPress() { 
  if (activeMode != MODE_MANUAL) return;

  switch (manualState) {
    case MANUAL_READY_TO_ACTIVATE:
      {
        unsigned long duration = calculateOperationDurationMs(manualSelectedFireType, manualSelectedIntensity);
        if (duration > 0) {
          // Construct FireData for activatePumpSystem consistency (though it's manual)
          FireData manualActivationData;
          manualActivationData.fireType = manualSelectedFireType;
          manualActivationData.fireIntensity = manualSelectedIntensity; // Used for duration, speed is manualCurrentSpeed
          // activatePumpSystem will use manualCurrentSpeed if activeMode==MODE_MANUAL and manualState==MANUAL_OPERATION_ACTIVE
          // Since we are transitioning TO active, we need to handle pump relays and initial speed here.
          
          int selectedPumpRelay = 0;
          if (manualSelectedFireType == 'A' || manualSelectedFireType == 'C') selectedPumpRelay = PUMP1_RELAY_PIN;
          else if (manualSelectedFireType == 'B' || manualSelectedFireType == 'D') selectedPumpRelay = PUMP2_RELAY_PIN;

          if (selectedPumpRelay != 0) {
              if (selectedPumpRelay == PUMP1_RELAY_PIN) { digitalWrite(PUMP2_RELAY_PIN, LOW); digitalWrite(PUMP1_RELAY_PIN, HIGH); }
              else { digitalWrite(PUMP1_RELAY_PIN, LOW); digitalWrite(PUMP2_RELAY_PIN, HIGH); }
              
              analogWrite(MOTOR_PWM_PIN, getActualPwmToWrite(manualCurrentSpeed));
              
              noInterrupts();
              pumpOperationEndTime = millis() + duration;
              pumpSystemActive = true;
              interrupts();
              
              manualState = MANUAL_OPERATION_ACTIVE; 
              Serial.println("Manual: Pump activation started.");
          } else {
             Serial.println("Manual: Error - No pump for selected fire type.");
          }
        } else {
            Serial.println("Manual: Calculated duration is 0. Pumps not activated.");
        }
      }
      break;
    case MANUAL_OPERATION_ACTIVE: // SHOOT button while pumps are active could mean STOP
      Serial.println("Manual: SHOOT pressed while active. Deactivating pumps.");
      deactivatePumpSystem();
      resetManualControlState(); // Go back to inactive or select type
      manualState = MANUAL_INACTIVE; 
      break;
    default:
      Serial.print("Manual: Shoot pressed in unhandled state "); Serial.println(manualState);
      break;
  }
  drawManualControlInterface(); 
}

void drawManualControlInterface() {
  if (activeMode != MODE_MANUAL) return; 
  
  tft.fillRect(0, ALERT_AREA_Y, TFT_WIDTH, ALERT_AREA_HEIGHT, COLOR_NAVY);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(1); 
  
  int y = ALERT_AREA_Y + 5;
  int x = 5;
  tft.setCursor(x, y);
  tft.setTextSize(2); 
  tft.print("MANUAL CONTROL");
  y += 20; 
  
  tft.setTextSize(1); 
  
  switch (manualState) {
    case MANUAL_INACTIVE:
      tft.setCursor(x, y); tft.println("Press & hold SHOOT");
      tft.setCursor(x, y + 12); tft.println("for 5s to activate");
      break;
    case MANUAL_SELECT_TYPE:
      tft.setCursor(x, y); tft.println("Select Fire Type:");
      tft.setCursor(x, y + 15); tft.print("1:A "); tft.print("2:B");
      tft.setCursor(x, y + 30); tft.print("3:C "); tft.print("4:D");
      // Highlight selected type (example for 'A')
      if (manualSelectedFireType != '\0') {
          int selX = x + ((manualSelectedFireType - 'A') % 2) * 40;
          int selY = y + 15 + ((manualSelectedFireType - 'A') / 2) * 15;
          // This highlighting logic needs to be more robust for different layouts
      }
      break;
    case MANUAL_SELECT_INTENSITY:
      tft.setCursor(x, y); tft.print("Type: "); tft.print(manualSelectedFireType); tft.println(" - Select Intensity:");
      tft.setCursor(x, y + 15); tft.print("1:Low ");  tft.print("2:Med");
      tft.setCursor(x, y + 30); tft.print("3:High "); tft.print("4:Max");
      break;
    case MANUAL_READY_TO_ACTIVATE:
      tft.setCursor(x, y); tft.print("Type: "); tft.print(manualSelectedFireType);
      tft.print(" Int: "); tft.println(manualSelectedIntensity);
      tft.setCursor(x, y + 15); tft.print("Speed: "); tft.print(manualCurrentSpeed);
      tft.setCursor(x, y + 30); tft.println("Press SHOOT to activate");
      break;
    case MANUAL_OPERATION_ACTIVE:
      tft.setCursor(x, y); tft.print("ACTIVE: "); 
      tft.print(manualSelectedFireType); tft.print("-"); tft.print(manualSelectedIntensity);
      unsigned long remainingTimeS = 0;
      noInterrupts();
      if (pumpSystemActive && pumpOperationEndTime > millis()) {
          remainingTimeS = (pumpOperationEndTime - millis()) / 1000;
      }
      interrupts();
      tft.setCursor(TFT_WIDTH - 60, y); 
      tft.print(remainingTimeS); tft.println("s");
      tft.setCursor(x, y + 15); tft.print("Speed: "); tft.print(manualCurrentSpeed);
      tft.setCursor(x, y + 30); tft.println("1:+10 2:-10 3:Def 4:Max");
      tft.setCursor(x, y + 45); tft.println("SHOOT to STOP");
      break;
    default: tft.setCursor(x, y); tft.println("Unknown Manual State!"); break;
  }
}