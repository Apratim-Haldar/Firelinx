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

// --- Landscape Orientation ---
#define TFT_WIDTH  160 // Wider dimension
#define TFT_HEIGHT 128 // Shorter dimension

// Initialize TFT
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// I2C Configuration
#define I2C_ADDRESS 0x08
#define EXPECTED_DATA_SIZE 12   // ESP32 now sends 12 bytes
#define ACKNOWLEDGEMENT_BYTE 0x55 // Byte sent back to ESP32 on success

// Structure to hold processed alert data
typedef struct {
    char fireType;        // 'A', 'B', 'C', 'D', or 'A'(Auto), '?'(Unknown)
    char fireIntensity;   // '1', '2', '3', '4', or '?'(Unknown)
    char stnID[6];        // Increased size slightly (e.g., "D/4\0")
    float latitude;       // Received as float
    float longitude;      // Received as float
    bool dataValid;       // Flag indicating if the current data is valid AND should be displayed
    bool ackValue;        // What the slave should send back on request
} FireData;

// Use volatile for data shared between ISR (receiveEvent) and main loop
volatile FireData receivedData;
volatile bool newDataAvailable = false; // Flag set by ISR, cleared by loop

unsigned long lastDataDisplayTime = 0; // Track when valid data was last *displayed*
const unsigned long ALERT_DISPLAY_DURATION = 20000; // How long to show an alert (ms)

// --- Color Definitions (16-bit Hex - Use these) ---
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
#define COLOR_PINK      0xF81F // Same as MAGENTA in 565

// Mode Control Variables
#define MODE_ESP32_DIALPAD 1
#define MODE_SMS           2 // Placeholder
#define MODE_WEB_CONTROL   3 // Placeholder
#define MODE_MANUAL        4 // Placeholder (Joystick/Buttons)

uint8_t activeMode = MODE_MANUAL;  // Default to Manual Mode
bool tickVisible = true;
unsigned long lastBlinkTime = 0;
unsigned long modeStartTime = 0;
bool manualInputDetected = false; // Assume this is set by other code parts

// Priority definitions (higher number = higher priority)
// Index 0 unused, indices 1-4 match mode numbers
const uint8_t modePriorities[] = {0, 3, 2, 1, 0}; // ESP32=3, SMS=2, Web=1, Manual=0 (lowest active)

// Temporary buffer used only inside ISR
uint8_t i2c_buffer[EXPECTED_DATA_SIZE];

// --- Layout Defines ---
#define TITLE_Y 5
#define MODES_Y (TITLE_Y + 15)
#define ALERT_AREA_Y (MODES_Y + 45) // Y-coordinate where the dynamic area starts
#define ALERT_AREA_HEIGHT (TFT_HEIGHT - ALERT_AREA_Y) // Height of the dynamic area

// --- Function Prototypes ---
void drawSystemInfo();
void drawControlModes(bool redraw);
void drawStandbyStatus();
void drawAlertInfo(const FireData& data); // Using Option 1 - Landscape
void handlePrioritySwitching();
void requestModeSwitch(uint8_t newMode);
void switchToManualMode();
void receiveEvent(int bytesReceived);
void onManualInputDetected();
void printReceivedData();
void requestEvent();


void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_ADDRESS);        // Join I2C bus as slave
    // Wire.setClock(100000); // Optional: Try setting lower clock speed if issues persist
    Wire.onReceive(receiveEvent); // Register event handler for incoming data
    Wire.onRequest(requestEvent); // Register event handler for requests (ACK)

    // Initialize volatile struct safely
    noInterrupts(); // Disable interrupts during initialization
    memset((void*)&receivedData, 0, sizeof(receivedData)); // Clear the data struct
    receivedData.dataValid = false;
    receivedData.ackValue = false; // Default to NACK
    newDataAvailable = false;
    interrupts(); // Re-enable interrupts

    // TFT Initialization
    tft.initR(INITR_BLACKTAB);  // Initialize ST7735S chip, black tab
    Serial.println(F("ST7735 Initialized"));
    tft.setRotation(1); // *** SET ROTATION FOR LANDSCAPE (Try 1 or 3) ***
    tft.fillScreen(COLOR_NAVY); // Use defined hex color
    tft.setTextWrap(false);     // Don't wrap text to next line

    drawSystemInfo(); // Display static system info initially
    Serial.println(F("✅ Mega2560 I2C Slave Ready (Addr 0x08)"));
}

void loop() {
    unsigned long currentTime = millis();
    bool displayNeedsUpdate = false; // Flag if screen content changed

    // --- Process New Data Flag ---
    noInterrupts(); // Access volatile flag safely
    bool processData = newDataAvailable;
    if (processData) {
        newDataAvailable = false; // Clear the flag immediately
    }
    interrupts();

    if (processData) {
        Serial.println("Loop: Processing new data received via I2C.");
        noInterrupts(); // Access volatile data safely
        bool isValid = receivedData.dataValid;
        interrupts();

        if (isValid) {
            printReceivedData(); // Log parsed data for debugging
            lastDataDisplayTime = currentTime; // Reset display timer
            displayNeedsUpdate = true; // Need to draw the alert
            requestModeSwitch(MODE_ESP32_DIALPAD); // Switch mode on valid data
        } else {
            Serial.println("Loop: ISR indicated new data, but it was marked invalid.");
            // Optionally handle invalid data case (e.g., display error briefly)
        }
    }

    // --- Handle Display State (Timeout or New Alert) ---
    noInterrupts(); // Access volatile flag safely
    bool isDataCurrentlyValid = receivedData.dataValid;
    interrupts();

    if (isDataCurrentlyValid) {
        // Check for timeout ONLY if data is currently valid
        if (currentTime - lastDataDisplayTime > ALERT_DISPLAY_DURATION) {
            Serial.println(F("Alert display duration expired. Reverting display."));
            noInterrupts();
            receivedData.dataValid = false; // Invalidate data *after* timeout check
            interrupts();
            drawStandbyStatus(); // Redraw the standby status in the placeholder
            displayNeedsUpdate = true;
        } else if (displayNeedsUpdate) {
            // New valid data arrived OR display needs refresh but not timed out yet

            // Manually copy volatile data to a local non-volatile struct
            FireData currentAlertData; // Declare the local copy
            noInterrupts(); // Disable interrupts for safe copy
            currentAlertData.fireType = receivedData.fireType;
            currentAlertData.fireIntensity = receivedData.fireIntensity;
            // Copy string safely
            strncpy(currentAlertData.stnID, (const char*)receivedData.stnID, sizeof(currentAlertData.stnID));
            currentAlertData.stnID[sizeof(currentAlertData.stnID)-1] = '\0'; // Ensure null termination
            currentAlertData.latitude = receivedData.latitude;
            currentAlertData.longitude = receivedData.longitude;
            currentAlertData.dataValid = receivedData.dataValid; // Copy validity too
            // We don't need ackValue in the local copy for display
            interrupts(); // Re-enable interrupts

            drawAlertInfo(currentAlertData); // Draw the alert using the local copy
        }
    } else if (displayNeedsUpdate) {
         // Ensure standby is drawn if display needed update but data became invalid
         drawStandbyStatus();
    }


    // --- Handle Mode Tick Blink ---
    if (currentTime - lastBlinkTime > 500) {
        tickVisible = !tickVisible;
        // Redraw only the modes section
        drawControlModes(true);
        lastBlinkTime = currentTime;
    }

    // Handle priority-based mode switching (timeouts)
    handlePrioritySwitching(); // This might call drawSystemInfo if mode changes

    // Other loop tasks
}

// Draws static FireLinx system info (Landscape)
void drawSystemInfo() {
    tft.fillScreen(COLOR_NAVY);
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1); // Default size

    // Title Area
    tft.setCursor(5, TITLE_Y);
    tft.print("FireLinx Central Control");
    tft.drawFastHLine(0, TITLE_Y + 10, TFT_WIDTH, COLOR_DARKGREY);

    // Communication Modes Area
    drawControlModes(false); // Initial draw

    // Draw the initial standby status in the placeholder area
    drawStandbyStatus();
}

// Draw Control Modes in a 2x2 Grid (Landscape)
void drawControlModes(bool redraw) {
    int16_t startY = MODES_Y;
    int16_t areaHeight = 40; // Approx height for title + 2 rows
    int16_t col1X = 5;
    int16_t col2X = TFT_WIDTH / 2; // Start second column halfway
    int16_t row1Y = startY + 12;
    int16_t row2Y = row1Y + 12;
    int16_t tickOffsetX = 75; // How far from text start to draw tick
    int16_t tickClearWidth = 20;

    if (redraw) {
        tft.fillRect(0, startY, TFT_WIDTH, areaHeight, COLOR_NAVY);
    }

    tft.setTextSize(1);
    tft.setTextColor(COLOR_WHITE);
    tft.setCursor(col1X, startY);
    tft.print("Modes:");
    if (tickVisible) tft.print(" (<= Active)"); // Indicate blinking tick meaning

    const char* modes[] = {"ESP32 Alert", "SMS Cmd", "Web Control", "Manual Ctrl"};
    int16_t xPos[] = {col1X, col2X, col1X, col2X};
    int16_t yPos[] = {row1Y, row1Y, row2Y, row2Y};

    for (uint8_t i = 0; i < 4; i++) {
        tft.setCursor(xPos[i], yPos[i]);

        // Highlight active mode text color
        if (i + 1 == activeMode) {
            tft.setTextColor(COLOR_YELLOW);
        } else {
            tft.setTextColor(COLOR_WHITE);
        }
        tft.print(modes[i]);

        // Clear previous tick area ONLY if redrawing
        if (redraw) {
             tft.fillRect(xPos[i] + tickOffsetX, yPos[i], tickClearWidth, 8, COLOR_NAVY);
        }

        // Draw tick for active mode (always white)
        tft.setTextColor(COLOR_WHITE);
        if (i + 1 == activeMode && tickVisible) {
            tft.setCursor(xPos[i] + tickOffsetX, yPos[i]); // Position tick after text
            tft.print("<=");
        }
    }
    tft.setTextColor(COLOR_WHITE); // Reset text color
    if (redraw) { // Draw separator only on full redraw
        tft.drawFastHLine(0, startY + areaHeight - 1, TFT_WIDTH, COLOR_DARKGREY);
    }
}


// Function to draw the default standby status in the placeholder area (Landscape)
void drawStandbyStatus() {
    tft.fillRect(0, ALERT_AREA_Y, TFT_WIDTH, ALERT_AREA_HEIGHT, COLOR_NAVY); // Clear area
    tft.setCursor(5, ALERT_AREA_Y + 5); // Position text within the area
    tft.setTextColor(COLOR_GREEN);
    tft.setTextSize(2); // Larger Status Text
    tft.println(" STANDBY");
    tft.setCursor(5, ALERT_AREA_Y + 25);
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1);
    tft.println("Awaiting Fire Alert...");
}

// --- DESIGN OPTION 1: Colored Background Box (Landscape) ---
void drawAlertInfo(const FireData& data) {
    uint16_t bgColor;
    uint16_t textColor = COLOR_BLACK; // Default text

    // Choose background color based on intensity
    switch(data.fireIntensity) {
        case '1': bgColor = COLOR_GREEN; textColor = COLOR_BLACK; break;
        case '2': bgColor = COLOR_YELLOW; textColor = COLOR_BLACK; break;
        case '3': bgColor = COLOR_ORANGE; textColor = COLOR_WHITE; break;
        case '4': bgColor = COLOR_RED; textColor = COLOR_WHITE; break;
        default:  bgColor = COLOR_MAGENTA; textColor = COLOR_WHITE;
    }

    // Clear and fill only the alert area
    tft.fillRect(0, ALERT_AREA_Y, TFT_WIDTH, ALERT_AREA_HEIGHT, bgColor);
    Serial.print("DEBUG (Opt1): Filled alert area with color: 0x"); Serial.println(bgColor, HEX);

    tft.setTextColor(textColor);
    tft.setTextSize(1);

    int y = ALERT_AREA_Y + 5; // Starting Y for alert text within the area
    int x = 5;

    // Alert Header
    tft.setCursor(x, y);
    tft.setTextSize(2); // Larger header
    tft.print("FIRE ALERT!");
    tft.setTextSize(1);
    y += 20; // Move down after larger text

    // Alert Details - Line 1
    tft.setCursor(x, y);
    tft.print("Stn:"); tft.print(data.stnID);
    tft.print(" Typ:"); tft.print(data.fireType);
    tft.print(" Int:"); tft.println(data.fireIntensity);
    y += 12;

    // Location (only if valid) - Side by side
    tft.setCursor(x, y);
    if (abs(data.latitude) > 0.0001 || abs(data.longitude) > 0.0001) {
        tft.print("Lat: "); tft.print(data.latitude, 4);
        // Move cursor for Lon - adjust spacing as needed
        tft.setCursor(x + 80, y); // Approx halfway
        tft.print("Lon: "); tft.println(data.longitude, 4);
    } else {
         tft.println("Location: N/A");
    }
    y += 12;

    // Add timestamp or other info if space allows
    // tft.setCursor(x, y);
    // tft.print("Time: HH:MM"); // Example

    Serial.println("DEBUG (Opt1): Drew alert details.");
}


// --- Handle Priority-Based Mode Switching (Timeouts) ---
void handlePrioritySwitching() {
    if (activeMode == MODE_MANUAL) return;
    unsigned long elapsedTime = millis() - modeStartTime;
    bool revertToManual = false;
    const unsigned long MODE_TIMEOUT_MS = 420000; // 7 minutes

    if (elapsedTime >= MODE_TIMEOUT_MS) {
        if (!manualInputDetected) {
            Serial.print(F("⏳ Mode ")); Serial.print(activeMode);
            Serial.println(F(" Timeout -> Switching to Manual"));
            revertToManual = true;
        } else {
            Serial.print(F("🎮 Manual input in Mode ")); Serial.print(activeMode);
            Serial.println(F(". Resetting timeout."));
            modeStartTime = millis();
            manualInputDetected = false;
        }
    }
    if (revertToManual) {
        switchToManualMode();
    }
}

// --- Switch to a new mode only if it has higher priority ---
void requestModeSwitch(uint8_t newMode) {
    if (newMode < 1 || newMode > 4) {
         Serial.print(F("⚠️ Invalid mode switch: ")); Serial.println(newMode);
         return;
    }
    if (modePriorities[newMode] > modePriorities[activeMode]) {
        Serial.print(F("🔄 Switching to mode: ")); Serial.println(newMode);
        activeMode = newMode;
        modeStartTime = millis();
        manualInputDetected = false;
        drawControlModes(true); // Redraw modes immediately
    } else if (newMode == activeMode) {
        Serial.print(F("🔄 Mode ")); Serial.print(newMode);
        Serial.println(F(" re-activated."));
        modeStartTime = millis();
        manualInputDetected = false;
    } else {
        Serial.print(F("⚠️ Ignoring low priority: ")); Serial.print(newMode);
        Serial.print(F(" (Active: ")); Serial.print(activeMode); Serial.println(F(")"));
    }
}

// --- Force switch to Manual Mode ---
void switchToManualMode() {
    if (activeMode != MODE_MANUAL) {
        Serial.println(F("🔄 Switching to Manual Mode."));
        activeMode = MODE_MANUAL;
        manualInputDetected = false;
        modeStartTime = millis();
        drawControlModes(true);
        noInterrupts();
        bool stillValid = receivedData.dataValid;
        interrupts();
        if (!stillValid) {
            drawStandbyStatus(); // Redraw standby if no active alert
        }
    }
}

// --- I2C Receive Event (ISR) ---
void receiveEvent(int bytesReceived) {
    if (bytesReceived == EXPECTED_DATA_SIZE) {
        int bytesRead = Wire.readBytes(i2c_buffer, EXPECTED_DATA_SIZE);
        if (bytesRead == EXPECTED_DATA_SIZE) {
            uint8_t calculatedChecksum = 0;
            for (int i = 0; i < EXPECTED_DATA_SIZE - 1; i++) calculatedChecksum ^= i2c_buffer[i];
            if (calculatedChecksum == i2c_buffer[EXPECTED_DATA_SIZE - 1]) {
                uint8_t typeVal = i2c_buffer[0];
                if (typeVal >= 1 && typeVal <= 4) receivedData.fireType = 'A' + typeVal - 1;
                else if (typeVal == 5) receivedData.fireType = 'A';
                else receivedData.fireType = '?';
                uint8_t intensityVal = i2c_buffer[1];
                if (intensityVal >= 1 && intensityVal <= 4) receivedData.fireIntensity = '0' + intensityVal;
                else receivedData.fireIntensity = '?';
                uint8_t zoneVal = i2c_buffer[2];
                if (zoneVal >= 1 && zoneVal <= 26) snprintf((char*)receivedData.stnID, sizeof(receivedData.stnID), "%c/%d", 'A' + zoneVal - 1, zoneVal);
                else snprintf((char*)receivedData.stnID, sizeof(receivedData.stnID), "?/?");
                memcpy((void*)&receivedData.latitude, &i2c_buffer[3], sizeof(float));
                memcpy((void*)&receivedData.longitude, &i2c_buffer[7], sizeof(float));
                receivedData.dataValid = true;
                receivedData.ackValue = true;
                newDataAvailable = true;
            } else {
                receivedData.dataValid = false; receivedData.ackValue = false;
            }
        } else {
            receivedData.dataValid = false; receivedData.ackValue = false;
        }
    } else {
        while (Wire.available()) Wire.read();
        // if (bytesReceived > 0) receivedData.ackValue = false; // Optional NACK
    }
}

// --- Called by other parts when manual input occurs ---
void onManualInputDetected() {
    if (activeMode != MODE_MANUAL) manualInputDetected = true;
}

// --- Print Received Data for Debugging ---
void printReceivedData() {
    noInterrupts();
    char type = receivedData.fireType; char intensity = receivedData.fireIntensity;
    char station[sizeof(receivedData.stnID)]; strncpy(station, (const char*)receivedData.stnID, sizeof(station)); station[sizeof(station)-1] = '\0';
    float lat = receivedData.latitude; float lon = receivedData.longitude;
    interrupts();
    Serial.println(F("\n--- Parsed I2C Data ---"));
    Serial.print(F("Fire Type: ")); Serial.println(type); Serial.print(F("Intensity: ")); Serial.println(intensity);
    Serial.print(F("Station ID: ")); Serial.println(station); Serial.print(F("Latitude: ")); Serial.println(lat, 6);
    Serial.print(F("Longitude: ")); Serial.println(lon, 6); Serial.println(F("---------------------"));
}

// --- I2C Request Event (ISR) ---
void requestEvent() {
    Wire.write(receivedData.ackValue ? ACKNOWLEDGEMENT_BYTE : 0x00);
    receivedData.ackValue = false;
}