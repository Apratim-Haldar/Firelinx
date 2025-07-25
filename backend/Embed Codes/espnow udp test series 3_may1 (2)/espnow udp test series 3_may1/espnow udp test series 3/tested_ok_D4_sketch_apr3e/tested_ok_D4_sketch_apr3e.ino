#include <esp_now.h>          // For ESP-NOW
#include <WiFi.h>             // For WiFi, ESP-NOW
#include <WiFiUdp.h>          // For UDP
#include <WiFiManager.h>      // For WiFi connection setup
#include <Wire.h>             // For I2C (OLED, Keypad, RFID)
#include <Adafruit_GFX.h>     // For OLED
#include <Adafruit_SH110X.h>   // For OLED driver (Using SH1106G in your provided code)
#include <Adafruit_PN532.h>   // For RFID
#include <I2CKeyPad.h>        // For Keypad
#include <TinyGPS++.h>        // For GPS parsing
#include <SoftwareSerial.h>   // For GPS communication
#include <max6675.h>          // For Thermocouple
#include <EEPROM.h>           // For storing configuration
#include <queue>              // Potentially useful, though not strictly needed here

// --- Pin Definitions ---
#define TOUCH_SENSOR_PIN 33 // Touch sensor pin
#define BUZZER_PIN 25       // Buzzer pin
#define MAX6675_CS 5        // Thermocouple CS pin
#define MAX6675_MISO 19     // Thermocouple MISO pin
#define MAX6675_SCK 18      // Thermocouple SCK pin
#define MQ2_A0 34           // MQ2 analog pin
#define MQ2_D0 26           // MQ2 digital pin (Note: Old code reads HIGH, New code reads LOW for alert) - ADJUSTING TO MATCH OLD LOGIC INTENT
#define MQ135_A0 35         // MQ135 analog pin
#define MQ135_D0 27         // MQ135 digital pin (Note: Old code reads HIGH, New code reads LOW for alert) - ADJUSTING TO MATCH OLD LOGIC INTENT
#define FLAME_SENSOR_PIN 36 // Flame sensor pin (Assuming LOW means flame detected in both)

// --- OLED Display ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_I2C_ADDRESS 0x3C // SH1106 I2C address

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Using SH1106G as per includes

// --- Keypad Setup ---
#define PCF8574_ADDR 0x20
I2CKeyPad keypad(PCF8574_ADDR);

// --- PN532 (RFID) Setup ---
#define PN532_SDA 21
#define PN532_SCL 22
Adafruit_PN532 nfc(PN532_SDA, PN532_SCL, &Wire);

// --- GPS Setup ---
#define GPS_RX 16
#define GPS_TX 17
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
TinyGPSPlus gps;

// --- Thermocouple Setup ---
MAX6675 thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_MISO);

// --- Communication Settings ---
#define UDP_PORT 12345
WiFiUDP udp;
IPAddress udpAddress; // Will be set to broadcast address after WiFi connection

// ESP-NOW Peer Info (Master's MAC Address)
uint8_t masterMac[] = {0xAC, 0x15, 0x18, 0xD4, 0x6C, 0x9C}; // REPLACE WITH ACTUAL MASTER MAC

// --- EEPROM Addresses ---
#define EEPROM_SIZE 512
#define WIFI_CREDS_ADDR 0
#define USER_COUNT_ADDR 100
#define USERS_ARRAY_ADDR (USER_COUNT_ADDR + sizeof(int))
#define STATION_ID_ADDR 150 // Approx start after max WiFi creds size
#define COMM_MODE_EEPROM_ADDR 200

// --- Configuration & State Variables ---
enum CommMode { UDP_MODE, ESPNOW_MODE };
CommMode commMode = UDP_MODE; // Default, will be loaded from EEPROM

#define STABILIZATION_PERIOD_MS 10000 // Reduced stabilization for testing, adjust as needed (was 70000)

struct WiFiCredentials {
  char ssid[32];
  char password[64];
};
WiFiCredentials wifiCreds;
String stationID = "D/4"; // Default, will be loaded from EEPROM

// --- Fire & Manual Mode Info ---
char fireType = '0';        // 'A', 'B', 'C', 'D' or '0' (none)
char fireIntensity = '0';   // '1', '2', '3', '4' or '0' (none)
bool manualModeActive = false;
bool isDisplayingSentStatus = false; // Flag to prevent loop update during status display
unsigned long touchStartTime = 0;
unsigned long rfidScanStartTime = 0;
unsigned long verificationStartTime = 0; // For manual verification timeout
unsigned long keypadTimeoutStart = 0;    // For menu timeout
unsigned long manualModeTimeoutStart = 0; // For overall manual mode step timeout

#define MENU_TIMEOUT 7000
#define MANUAL_MODE_STEP_TIMEOUT 7000 // Timeout for each step in manual mode
#define RFID_TIMEOUT 7000
#define VERIFICATION_TIMEOUT 5000 // Timeout for '*' verification press

// --- User Management ---
#define MAX_USERS 10
#define OG_USERS 2 // Initial number of hardcoded OG users
struct User {
  uint8_t uid[4];
  char name[32]; // Use char arrays for EEPROM compatibility
  char id[10];
  bool isOG;
};
User users[MAX_USERS] = {
  {{0xEE, 0x5A, 0xB5, 0x2}, "Gobinda Sir", "D01", true}, // Ensure these match your actual OG tags
  {{0x33, 0x88, 0xA4, 0x2C}, "Sayan Sir", "D02", true}
};
int userCount = OG_USERS; // Will be loaded from EEPROM

String currentUser = "";    // Name of the currently authenticated user
String currentUserID = "";  // ID of the currently authenticated user
bool currentUserIsOG = false; // Privileges of the current user
uint8_t currentUID[4] = {0}; // UID of the currently scanned tag (used in user mgmt)

// --- GPS & Sensor Data ---
String previousLatitude = "N/A";
String previousLongitude = "N/A";
String gpsDate = "N/A";
String gpsTime = "N/A";
float gpsLat = 0.0;
float gpsLon = 0.0;
float currentTemp = 0.0;
float avgTemp = 0.0; // Moving average for temp spike detection
unsigned long lastTempUpdate = 0;

// --- Data Structure for Communication ---
// This structure MUST match the one on the Master ESP32
typedef struct struct_message {
    char fireType;        // A, B, C, D or 'A' for Auto
    char fireIntensity;   // 1-4 (Manual) or calculated severity mapped to 1-4 (Auto)
    bool verified;        // True if manual alert verified with '*'
    char user[32];        // Name of user (Manual) or "Auto Mode"
    char userID[10];      // User ID (Manual) or "AUTO"
    char stnID[10];       // Station ID (e.g., "D/4") from EEPROM
    char latitude[16];    // DDM format string
    char longitude[16];   // DDM format string
    char date[10];        // DD/MM string
    char time[10];        // HH:MM string (adjusted)
} struct_message;

struct_message fireData; // Global instance used for sending

// --- Menu System ---
#define MENU_ITEMS 4
const String menuOptions[MENU_ITEMS] = {
  "1. Manual Fire Alert",
  "2. Manage Users",
  "3. Manage WiFi",
  "4. Comm Mode" // Option to switch UDP/ESPNOW
};

// --- Function Prototypes ---
// Initialization
void loadConfigFromEEPROM();
void saveConfigToEEPROM();
void initWiFi();
void initESP_NOW();
void initPeripherals();

// Communication
void sendAlertData(bool verified);
void sendViaUDP();
void sendViaESPNOW();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status); // ESP-NOW Callback
void printCommStatus(const struct_message& data, bool success, const char* protocol, esp_now_send_status_t esp_status = ESP_NOW_SEND_FAIL);

// Core Logic
void automode(); // <<<< USING OLD LOGIC
void activateManualMode();
void handleManualMode();
void resetManualMode();

// Sensor & GPS
void updateSensorReadings(); // <<<< USING OLD LOGIC
void updateGPSData();
String convertToDDM(double decimalDegrees, bool isLatitude);
bool isRealAlert(bool mq2_alert, bool mq135_alert, bool flameDetected, bool tempSpike); // <<<< USING OLD LOGIC
int calculateSeverity(int mq2Val, int mq135Val, bool flameDetected, float tempDiff);   // <<<< USING OLD LOGIC
bool handleFalsePositives(int mq2Val, int mq135Val, float currentTemp, float avgTemp); // <<<< USING OLD LOGIC

// Menu & Configuration
void showMainMenu();
void displayMenu();
void handleUserManagement();
void addUser();
void removeUser();
void removeSelf();
void removeGuest();
void promoteUser();
void configPortal();
void switchCommMode();

// User Auth & RFID
bool checkRFID(unsigned int timeout_ms = RFID_TIMEOUT);
bool validateUID(uint8_t* uid);
int findUserIndexByUID(const uint8_t* uid_to_find);

// Display & UI
void resetDisplay();
void displayFireTypePrompt();
void displayFireIntensityPrompt();
void displayVerificationPrompt();
void displayInvalidInput(String message);
void displaySelectedFireType();
void displaySelectedFireIntensity();

// Utility
void buzzAlert(bool state);
bool keypadEntryTimeout(unsigned long timeout = MENU_TIMEOUT);
void resetKeypadTimeout();

// EEPROM User Management
void saveUsersToEEPROM();
void loadUsersFromEEPROM();

// ======================= SETUP =======================
void setup() {
    Serial.begin(115200);
    Wire.begin(); // Initialize I2C
    EEPROM.begin(EEPROM_SIZE);

    Serial.println("\n\n--- FireLinx Child Node Booting ---");

    // Load configuration from EEPROM first
    loadConfigFromEEPROM();
    Serial.printf("Loaded Config: Station ID=%s, Comm Mode=%s\n",
                  stationID.c_str(), (commMode == UDP_MODE) ? "UDP" : "ESP-NOW");

    // Initialize peripherals (OLED, Sensors, Keypad, RFID, Buzzer)
    initPeripherals();

    // Initialize WiFi (connects or starts Config Portal)
    initWiFi(); // Must be done before ESP-NOW if relying on channel

    // Initialize ESP-NOW (after WiFi is connected to get channel)
    initESP_NOW();

    // Initialize UDP (after WiFi is connected)
    if (WiFi.status() == WL_CONNECTED) {
       udp.begin(UDP_PORT);
       Serial.println("UDP Initialized.");
    } else {
       Serial.println("UDP Initialization skipped (No WiFi).");
    }

    // Initialize average temperature with the first reading
    if (thermocouple.readCelsius() > -100) { // Basic check
       avgTemp = thermocouple.readCelsius();
       currentTemp = avgTemp;
    }
    Serial.printf("Initial Temp: %.2f C, Avg Temp: %.2f C\n", currentTemp, avgTemp);


    Serial.println("--- System Ready ---");
    resetDisplay(); // Show initial idle screen
}
// ======================= END SETUP =======================

// ======================= MAIN LOOP =======================
void loop() {
    // If displaying send status, pause main loop updates
    if (isDisplayingSentStatus) {
        delay(100); // Small delay to prevent busy-waiting
        return;
    }

    // Update sensor readings and GPS data continuously
    updateGPSData();
    updateSensorReadings(); // Uses OLD logic now

    // Check touch sensor for menu activation (only if not already in manual mode)
    if (!manualModeActive && digitalRead(TOUCH_SENSOR_PIN)) {
        if (touchStartTime == 0) {
            touchStartTime = millis();
        } else if (millis() - touchStartTime > 1000) { // Hold for 1 second
            showMainMenu(); // Enter the main menu system
            touchStartTime = 0; // Reset after entering menu
        }
    } else {
        touchStartTime = 0; // Reset if touch released
    }

    // Run either manual mode or auto mode
    if (manualModeActive) {
        handleManualMode();
    } else {
        // Update display in auto mode if GPS changes (or periodically)
        static unsigned long lastAutoDisplayUpdate = 0;
        // Only update display if NOT in stabilization period and data changed/timer elapsed
        if (millis() > STABILIZATION_PERIOD_MS && (gps.location.isUpdated() || millis() - lastAutoDisplayUpdate > 5000)) {
             if (!manualModeActive) resetDisplay(); // Refresh display in auto mode
             lastAutoDisplayUpdate = millis();
        } else if (millis() <= STABILIZATION_PERIOD_MS && millis() - lastAutoDisplayUpdate > 1000) {
             // Update display during stabilization to show progress/status
             if (!manualModeActive) resetDisplay();
             lastAutoDisplayUpdate = millis();
        }
        automode(); // Uses OLD logic now
    }
}
// ======================= END MAIN LOOP =======================

// ======================= INITIALIZATION FUNCTIONS =======================
void initPeripherals() {
    // Initialize OLED
    if (!display.begin(OLED_I2C_ADDRESS, true)) {
        Serial.println(F("SH110X OLED allocation failed"));
        while (1); // Halt on critical failure
    }
    display.display(); // Adafruit logo
    delay(1000);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("OLED OK");
    display.display();
    delay(500);

    // Initialize GPS Serial
    gpsSerial.begin(9600);
    Serial.println("GPS Serial OK");

    // Initialize touch sensor and buzzer
    pinMode(TOUCH_SENSOR_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off
    Serial.println("Touch/Buzzer OK");

    // Initialize Keypad
    Wire.beginTransmission(PCF8574_ADDR);
    if (Wire.endTransmission() != 0) {
         Serial.println("Keypad Not Found!");
         display.clearDisplay(); display.setCursor(0, 10); display.println("KEYPAD ERROR!"); display.display(); delay(2000);
    } else {
         if (!keypad.begin()) {
              Serial.println("Keypad begin() failed!");
         } else {
              keypad.loadKeyMap("DCBA#9630852*741"); // Standard 4x4 mapping
              Serial.println("Keypad OK");
         }
    }

    // Initialize PN532 (RFID)
    nfc.begin();
    uint32_t versiondata = nfc.getFirmwareVersion();
    if (!versiondata) {
        Serial.println("Didn't find PN532 board. Check wiring!");
        display.clearDisplay(); display.setCursor(0, 10); display.println("RFID ERROR!"); display.display(); delay(2000);
    } else {
        Serial.print("Found PN532 Firmware V: "); Serial.print((versiondata>>16)&0xFF, DEC); Serial.print('.'); Serial.println((versiondata>>8)&0xFF, DEC);
        nfc.SAMConfig();
        Serial.println("PN532 RFID OK");
    }

     // Initialize Sensors
    pinMode(MQ2_D0, INPUT_PULLUP); // Use PULLUP if HIGH means *no* gas detected (common)
    pinMode(MQ135_D0, INPUT_PULLUP); // Use PULLUP if HIGH means *no* gas detected
    pinMode(FLAME_SENSOR_PIN, INPUT_PULLUP); // Use PULLUP if LOW means flame detected
    // MAX6675 is initialized by its constructor
    Serial.println("Other Sensors OK");
    delay(1000); // Allow sensors to stabilize
}

void loadConfigFromEEPROM() {
    EEPROM.get(STATION_ID_ADDR, stationID);
    // Basic validation: if it looks like garbage or is empty, use default
    // Note: String object handles termination automatically, but check length
    if (stationID.length() < 1 || stationID.length() > 8 || stationID[0] < ' ' || stationID[0] > '~') {
        Serial.println("Invalid Station ID found in EEPROM, using default.");
        stationID = "D/4"; // Default
        EEPROM.put(STATION_ID_ADDR, stationID); // Save default if invalid
        EEPROM.commit();
    }

    // Load Communication Mode
    uint8_t modeValue;
    EEPROM.get(COMM_MODE_EEPROM_ADDR, modeValue);
    if (modeValue == (uint8_t)UDP_MODE || modeValue == (uint8_t)ESPNOW_MODE) {
        commMode = (CommMode)modeValue;
    } else {
        commMode = UDP_MODE; // Default to UDP if invalid value
        EEPROM.put(COMM_MODE_EEPROM_ADDR, (uint8_t)commMode);
        EEPROM.commit();
    }

    // Load WiFi Credentials (used by initWiFi)
    EEPROM.get(WIFI_CREDS_ADDR, wifiCreds);
    // Basic check - might be empty if never set
    wifiCreds.ssid[sizeof(wifiCreds.ssid)-1] = '\0';
    wifiCreds.password[sizeof(wifiCreds.password)-1] = '\0';

    // Load Users
    loadUsersFromEEPROM();
}

void saveConfigToEEPROM() {
    EEPROM.put(STATION_ID_ADDR, stationID); // Saving String directly works with ESP32 EEPROM library
    EEPROM.put(COMM_MODE_EEPROM_ADDR, (uint8_t)commMode);
    EEPROM.put(WIFI_CREDS_ADDR, wifiCreds);
    saveUsersToEEPROM(); // User saving handled separately
    EEPROM.commit();
    Serial.println("Configuration saved to EEPROM.");
}

void initWiFi() {
    WiFi.mode(WIFI_STA); // Set WiFi mode to Station

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connecting WiFi...");
    display.display();
    Serial.println("Initializing WiFi...");

    // Load credentials from EEPROM (already done in loadConfig)
    // If SSID is empty, likely never configured or EEPROM wiped
    if (strlen(wifiCreds.ssid) == 0) {
        Serial.println("No WiFi credentials found. Starting Config Portal.");
        display.println("Starting Config...");
        display.display();
        configPortal(); // Run WiFiManager configuration
    } else {
        Serial.printf("Attempting connection to SSID: %s\n", wifiCreds.ssid);
        display.printf("Connecting to %s\n", wifiCreds.ssid);
        display.display();

        WiFi.begin(wifiCreds.ssid, wifiCreds.password);

        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) { // 20-second timeout
            display.print(".");
            display.display();
            Serial.print(".");
            delay(500);
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi Connected!");
            Serial.print("IP Address: "); Serial.println(WiFi.localIP());
            Serial.print("MAC Address: "); Serial.println(WiFi.macAddress());
            Serial.printf("Channel: %d\n", WiFi.channel());


            display.clearDisplay();
            display.setCursor(0,0);
            display.println("WiFi Connected!");
            display.println(WiFi.SSID());
            display.println(WiFi.localIP());
            display.display();

            // Set the UDP broadcast address based on the connected network
            udpAddress = WiFi.localIP();
            udpAddress[3] = 255; // Set last octet to 255 for broadcast
            Serial.print("UDP Broadcast Address: "); Serial.println(udpAddress);

        } else {
            Serial.println("\nWiFi Connection Failed!");
            display.clearDisplay();
            display.setCursor(0,0);
            display.println("WiFi Failed!");
            display.println("SSID: " + String(wifiCreds.ssid));
            display.println("Check Pwd/Network.");
            display.println("Starting Config...");
            display.display();
            delay(2000);
            // Optionally run config portal on failure
             configPortal(); // Start config portal if connection fails
        }
    }
     delay(1000); // Short delay after WiFi setup attempt
}

void initESP_NOW() {
    // Set channel explicitly before init ONLY if WiFi failed but we want ESP-NOW
    if (WiFi.status() != WL_CONNECTED) {
         Serial.println("WiFi not connected, setting ESP-NOW channel to default 1.");
         esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    }
    // If WiFi IS connected, ESP-NOW will use its channel automatically during init.

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        display.clearDisplay(); display.setCursor(0,10); display.println("ESPNOW INIT FAIL!"); display.display(); delay(2000);
        return; // Critical failure
    } else {
         Serial.printf("ESP-NOW Initialized. Using Channel: %d\n", WiFi.channel()); // Log the channel being used
    }

    // Register the send callback function
    esp_now_register_send_cb(OnDataSent);

    // Register the master peer
    esp_now_peer_info_t peerInfo = {}; // Initialize struct to zero
    memcpy(peerInfo.peer_addr, masterMac, 6);
    peerInfo.channel = 0; // 0 means use the current channel
    peerInfo.encrypt = false; // No encryption for simplicity

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add ESP-NOW peer (Master)");
        display.clearDisplay(); display.setCursor(0,10); display.println("ESPNOW PEER FAIL!"); display.display(); delay(2000);
    } else {
        Serial.println("ESP-NOW Peer (Master) Added Successfully.");
    }
     delay(500);
}

// ======================= COMMUNICATION FUNCTIONS =======================
void sendAlertData(bool verified) {
    // 1. Populate the fireData struct
    fireData.verified = verified;

    // Fire type and intensity are already set in fireData by automode() or handleManualMode() before calling this
    // We just need to ensure they are correctly copied if coming from manual mode globals
     if (manualModeActive) {
         fireData.fireType = fireType;
         fireData.fireIntensity = fireIntensity;
     }
     // If called from automode, fireData.fireType and fireData.fireIntensity were set there.


    // User Info (Set based on mode)
    if (manualModeActive) {
        strncpy(fireData.user, currentUser.c_str(), sizeof(fireData.user) - 1);
        fireData.user[sizeof(fireData.user) - 1] = '\0';
        strncpy(fireData.userID, currentUserID.c_str(), sizeof(fireData.userID) - 1);
        fireData.userID[sizeof(fireData.userID) - 1] = '\0';
    } else { // Auto Mode
        strncpy(fireData.user, "Auto Mode", sizeof(fireData.user) - 1);
        fireData.user[sizeof(fireData.user) - 1] = '\0';
        strncpy(fireData.userID, "AUTO", sizeof(fireData.userID) - 1);
        fireData.userID[sizeof(fireData.userID) - 1] = '\0';
        // Note: fireType and fireIntensity for Auto mode were set inside automode()
    }

    // Station ID (from EEPROM)
    strncpy(fireData.stnID, stationID.c_str(), sizeof(fireData.stnID) - 1);
    fireData.stnID[sizeof(fireData.stnID) - 1] = '\0';

    // GPS Data (use latest values)
    strncpy(fireData.latitude, previousLatitude.c_str(), sizeof(fireData.latitude) - 1);
    fireData.latitude[sizeof(fireData.latitude) - 1] = '\0';
    strncpy(fireData.longitude, previousLongitude.c_str(), sizeof(fireData.longitude) - 1);
    fireData.longitude[sizeof(fireData.longitude) - 1] = '\0';

    // Date & Time (use latest values)
    strncpy(fireData.date, gpsDate.c_str(), sizeof(fireData.date) - 1);
    fireData.date[sizeof(fireData.date) - 1] = '\0';
    strncpy(fireData.time, gpsTime.c_str(), sizeof(fireData.time) - 1);
    fireData.time[sizeof(fireData.time) - 1] = '\0';

    // 2. Log before sending
    Serial.println("\n--- Preparing to Send Alert ---");
    Serial.printf("Mode: %s\n", (commMode == UDP_MODE) ? "UDP" : "ESP-NOW");
    Serial.printf("Data: Type=%c, Intensity=%c, Verified=%d\n", fireData.fireType, fireData.fireIntensity, fireData.verified);
    Serial.printf("User: %s (%s), Station: %s\n", fireData.user, fireData.userID, fireData.stnID);
    Serial.printf("GPS: %s, %s | Time: %s, %s\n", fireData.latitude, fireData.longitude, fireData.date, fireData.time);

    // 3. Send via selected protocol
    if (commMode == UDP_MODE) {
        sendViaUDP();
    } else { // ESPNOW_MODE
        sendViaESPNOW();
    }

    // 4. Reset state if called from manual mode
    if (manualModeActive) {
       resetManualMode(); // Reset state after initiating send
    }
}

void sendViaUDP() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("UDP Send Failed: WiFi not connected.");
        printCommStatus(fireData, false, "UDP"); // Show failure status
        return;
    }

    udp.beginPacket(udpAddress, UDP_PORT);
    int bytesSent = udp.write((uint8_t *)&fireData, sizeof(fireData));
    bool success = udp.endPacket(); // endPacket() returns true on success

    Serial.printf("UDP Send: %d bytes attempt, Success: %s\n", bytesSent, success ? "Yes" : "No");
    printCommStatus(fireData, success, "UDP"); // Display status on OLED
}

void sendViaESPNOW() {
    esp_err_t result = esp_now_send(masterMac, (uint8_t *)&fireData, sizeof(fireData));

    if (result != ESP_OK) {
         Serial.printf("ESP-NOW Send Initial Error: %s\n", esp_err_to_name(result));
         printCommStatus(fireData, false, "ESP-NOW", ESP_NOW_SEND_FAIL); // Use generic fail status
    } else {
         Serial.println("ESP-NOW Send Initiated. Waiting for callback...");
         display.clearDisplay(); display.setCursor(0,10); display.println("Sending ESP-NOW..."); display.display();
         isDisplayingSentStatus = true; // Prevent updates while waiting for callback status
    }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.printf("ESP-NOW Send CB: Status = %s\n", (status == ESP_NOW_SEND_SUCCESS) ? "Success" : "Fail");
    // isDisplayingSentStatus should already be true if send was initiated
    printCommStatus(fireData, (status == ESP_NOW_SEND_SUCCESS), "ESP-NOW", status); // Display status on OLED
}

void printCommStatus(const struct_message& data, bool success, const char* protocol, esp_now_send_status_t esp_status) {
    isDisplayingSentStatus = true; // Ensure flag is set

    // --- Serial Print ---
    Serial.println("\n=== Communication Status ===");
    Serial.printf("Protocol: %s\n", protocol);
    Serial.printf("Status: %s", success ? "SUCCESS" : "FAILED");
    if (!success && strcmp(protocol, "ESP-NOW") == 0) {
        Serial.printf(" (Reason: %s)", (esp_status == ESP_NOW_SEND_SUCCESS) ? "ACK OK?" : "ACK Fail/Timeout"); // Corrected logic
    }
    Serial.println();
    Serial.printf("Fire Type: %c, Intensity: %c, Verified: %d\n", data.fireType, data.fireIntensity, data.verified);
    Serial.printf("User: %s (%s), Station: %s\n", data.user, data.userID, data.stnID);
    Serial.printf("Location: %s, %s\n", data.latitude, data.longitude);
    Serial.printf("Date: %s, Time: %s\n", data.date, data.time);
    Serial.println("===========================\n");

    // --- OLED Print ---
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.printf("%s Status:\n", protocol);
    display.printf("  %s\n", success ? ">>> SENT OK <<<" : ">>> SEND FAIL <<<");
    display.drawFastHLine(0, 18, SCREEN_WIDTH, SH110X_WHITE);
    display.setCursor(0, 21);
    display.printf("Typ:%c Int:%c Ver:%d\n", data.fireType, data.fireIntensity, data.verified);
    display.printf("Usr:%s (%s)\n", data.user, data.userID);
    display.printf("Stn:%s\n", data.stnID);
    display.printf("%s %s\n", data.date, data.time);
    // display.printf("%s\n%s\n", data.latitude, data.longitude); // Lat/Lon might crowd screen

    display.display();

    // Keep status displayed for a few seconds
    unsigned long displayStartTime = millis();
    // Use a simple delay, as ESP-NOW callback is interrupt-driven
    delay(success ? 5000 : 7000); // Show longer for failures

    isDisplayingSentStatus = false; // Allow loop to resume normal updates
    resetDisplay(); // Go back to idle/auto display
}

// ======================= AUTO MODE & SENSOR LOGIC (USING OLD CODE) =======================
void updateSensorReadings() {
    // Read Thermocouple (OLD interval, NEW checks)
    if (millis() - lastTempUpdate >= 220) { // Old 220ms interval
        lastTempUpdate = millis();
        float newTemp = thermocouple.readCelsius();
        // Basic validation for thermocouple reading
        if (!isnan(newTemp) && newTemp > -50 && newTemp < 1000) { // Keep improved checks
            currentTemp = newTemp;
            // Update moving average (OLD weighting)
            avgTemp = (avgTemp * 0.9) + (currentTemp * 0.1); // Old 0.9/0.1 weighting
        } else if (isnan(newTemp)) {
             Serial.println("Warning: Thermocouple NaN");
        } else {
             Serial.printf("Warning: Thermocouple out of range: %.2f C\n", newTemp);
        }
    }
    // Other sensor readings (MQ2, MQ135, Flame) are done directly in automode()
}

void automode() {
    // NOTE: GPS update logic might differ slightly from the second code's loop structure.
    // Ensure GPS data (previousLatitude, etc.) is available globally.
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    // --- Sensor Reading (as done in FIRST code) ---
    int mq2Val = analogRead(MQ2_A0);
    int mq135Val = analogRead(MQ135_A0);
    // --- Verify digitalRead logic matches INPUT mode and sensor output ---
    // Original code likely assumed HIGH = alert for MQ, LOW = alert for Flame
    bool mq2_alert = digitalRead(MQ2_D0);     // Assumes HIGH is alert (Confirm!)
    bool mq135_alert = digitalRead(MQ135_D0); // Assumes HIGH is alert (Confirm!)
    bool flameDetected = digitalRead(FLAME_SENSOR_PIN) == LOW; // Assumes LOW is alert
    // --- End Verification Note ---

    // Read Temperature periodically
    if (millis() - lastTempUpdate >= 220) { // Read every 220ms
        lastTempUpdate = millis();
        float newTemp = thermocouple.readCelsius();
        if (!isnan(newTemp) && newTemp > -100 && newTemp < 1000) {
            currentTemp = newTemp;
            avgTemp = (avgTemp * 0.9) + (currentTemp * 0.1); // Moving average
        }
    }

    // --- Sensor Data Processing & Decision Logic (From FIRST code) ---
    bool tempSpike = abs(currentTemp - avgTemp) > 10; // Check for >10C spike

    bool realAlert = isRealAlert(mq2_alert, mq135_alert, flameDetected, tempSpike);
    int severity = calculateSeverity(mq2Val, mq135Val, flameDetected, abs(currentTemp - avgTemp));
    bool filtered = handleFalsePositives(mq2Val, mq135Val, currentTemp, avgTemp);

    // --- Action based on Sensor Data (From FIRST code) ---
    if (realAlert && severity >= 3 && filtered) { // Original condition
        digitalWrite(BUZZER_PIN, HIGH); // Activate Buzzer

        // Populate fireData structure (as done in FIRST code)
        fireData.fireType = 'A'; // Auto-detected fire
        // Map severity score (0-8+) to intensity char ('1'-'4') - You might need to adjust this mapping
        // This mapping wasn't explicitly in the first automode, but needed for sendAlertData
        if (severity >= 7) fireData.fireIntensity = '4';
        else if (severity >= 5) fireData.fireIntensity = '3';
        else if (severity >= 3) fireData.fireIntensity = '2'; // Matches log example
        else fireData.fireIntensity = '1'; // Fallback

        fireData.verified = false; // Auto alerts are unverified

        // User/Station/GPS/Time data will be populated by sendAlertData in the second code

        // !!! CRITICAL MODIFICATION !!!
        // REPLACE the original esp_now_send line with this:
        sendAlertData(false);
        // !!! END CRITICAL MODIFICATION !!!

        // !!! REMOVE any delay() that might have been here in the original !!!

    } else {
        digitalWrite(BUZZER_PIN, LOW); // Deactivate Buzzer if conditions not met
    }
}

bool isRealAlert(bool mq2_alert, bool mq135_alert, bool flameDetected, bool tempSpike) {
    static unsigned long alertStartTime = 0; // Time when alert condition first detected
    static bool alertActive = false;         // Flag if an alert condition is currently being timed
    const unsigned long PERSISTENCE_THRESHOLD = 5000; // 5 seconds

    // Check if any sensor indicates a potential alert
    // Ensure the digitalRead logic used to generate mqX_alert matches sensor output!
    bool currentCondition = mq2_alert || mq135_alert || flameDetected || tempSpike;

    if (currentCondition) {
        if (!alertActive) {
            // Start timer only on the rising edge of an alert condition
            alertStartTime = millis();
            alertActive = true;
            // Optional: Add Serial.println("Potential alert condition started.");
        }
        // If condition persists for more than threshold, confirm it's a real alert
        if (millis() - alertStartTime > PERSISTENCE_THRESHOLD) {
             // Optional: Add Serial.println("Alert condition confirmed (persistent).");
            return true;
        }
    } else {
        // Reset the timer if no alert condition is present
        // Optional: if(alertActive) Serial.println("Potential alert condition ended.");
        alertActive = false;
        alertStartTime = 0; // Reset timer explicitly
    }
    // Return false if the condition hasn't persisted long enough or isn't active
    return false;
}

int calculateSeverity(int mq2Val, int mq135Val, bool flameDetected, float tempDiff) {
    int severity = 0;

    // Assign points based on sensor readings exceeding thresholds (Original thresholds)
    if (flameDetected) {
        severity += 3; // Flame detection is significant
    }

    if (mq2Val > 700) { // High gas reading
        severity += 2;
    } else if (mq2Val > 400) { // Moderate gas reading
        severity += 1;
    }

    if (mq135Val > 600) { // High air quality sensor reading
        severity += 2;
    } else if (mq135Val > 300) { // Moderate air quality sensor reading
        severity += 1;
    }

    if (tempDiff > 15) { // Significant temperature difference/spike
        severity += 2;
    } else if (tempDiff > 10) { // Moderate temperature difference/spike
        severity += 1;
    }

    return severity; // Return the total severity score
}

bool handleFalsePositives(int mq2Val, int mq135Val, float currentTemp, float avgTemp) {
    static float lastTemp = 0;         // Store previous temperature reading
    static int mq2Stable = 0;          // Counter for consecutive high MQ2 readings
    static int mq135Stable = 0;        // Counter for consecutive high MQ135 readings
    const int CONSECUTIVE_THRESHOLD = 3; // Needs 3 stable readings above threshold

    // Check 1: Temperature Stability (Original logic was potentially flawed)
    // If temp change is very small, original code returned false.
    // This might filter real, slow-starting fires.
    if (abs(currentTemp - lastTemp) < 3) {
        // lastTemp = currentTemp; // Update lastTemp anyway
        // return false; // Original logic returned false here. Replicating.
    }
     lastTemp = currentTemp; // Update last temperature reading

    // Check 2: Sustained Gas Levels (Counts consecutive readings above threshold)
    // Ensure the digitalRead logic used to generate mqX_alert matches sensor output!
    if (mq2Val < 350) { // If below threshold, reset counter
        mq2Stable = 0;
    } else {           // If above threshold, increment counter
        mq2Stable++;
    }

    if (mq135Val < 250) { // If below threshold, reset counter
        mq135Stable = 0;
    } else {           // If above threshold, increment counter
        mq135Stable++;
    }

    // Check 3: Require Sustained Readings
    // If neither gas sensor has had CONSECUTIVE_THRESHOLD readings above its 'stable' threshold, filter out.
    // This might filter out flame/temp alerts if gas isn't also high.
    if (mq2Stable < CONSECUTIVE_THRESHOLD && mq135Stable < CONSECUTIVE_THRESHOLD) {
        // Serial.println("FP Filter: Gas levels not sustained."); // Optional debug
        return false; // Filter out if gas levels aren't sustained high
    }

    // If none of the above conditions returned false, the filter passes.
    return true; // Assume it's not a false positive
}

// ======================= MANUAL MODE LOGIC (No changes needed here) =======================
void activateManualMode() {
    manualModeActive = true;
    touchStartTime = 0; // Reset touch timer
    fireType = '0';     // Reset selections
    fireIntensity = '0';
    currentUser = "";     // Reset user info
    currentUserID = "";
    currentUserIsOG = false;
    memset(currentUID, 0, sizeof(currentUID));
    manualModeTimeoutStart = 0; // Reset step timeout

    Serial.println("Manual Mode Activated. Requesting RFID Scan.");

    display.clearDisplay();
    display.setCursor(0, 2);
    display.println("--- Manual Mode ---");
    display.display();
    delay(1000);

    // Step 1: Authenticate via RFID
    display.clearDisplay();
    display.setCursor(0, 2);
    display.println("Scan RFID for Access");
    display.println("(Timeout: " + String(RFID_TIMEOUT/1000) + "s)");
    display.display();
    rfidScanStartTime = millis(); // Start RFID scan timer

    bool rfidSuccess = checkRFID(RFID_TIMEOUT); // checkRFID handles display updates during scan

    if (!rfidSuccess) {
        Serial.println("RFID Scan Failed or Timed Out.");
        // Display message already shown by checkRFID
        delay(2000); // Allow user to see message
        resetManualMode(); // Exit manual mode
    } else {
        Serial.printf("RFID Scan Success. User: %s (%s), OG: %d\n", currentUser.c_str(), currentUserID.c_str(), currentUserIsOG);
        display.clearDisplay();
        display.setCursor(0, 2);
        display.println("Access Granted!");
        display.println("User: " + currentUser);
        display.display();
        delay(2000);
        // Proceed to Fire Type selection
        displayFireTypePrompt();
        manualModeTimeoutStart = millis(); // Start timer for the *next* step (fire type)
    }
}

void handleManualMode() {
    // Check for timeout within the current step
    if (manualModeTimeoutStart > 0 && millis() - manualModeTimeoutStart > MANUAL_MODE_STEP_TIMEOUT) {
        Serial.println("Manual Mode Step Timed Out.");
        display.clearDisplay();
        display.setCursor(0, 2);
        display.println("Input Timeout!");
        display.println("Returning to Auto Mode...");
        display.display();
        delay(2000);

        // If timeout occurs during verification stage, send unverified data
        if (fireType != '0' && fireIntensity != '0' && verificationStartTime > 0) {
             Serial.println("Timeout during verification. Sending unverified alert.");
             sendAlertData(false); // Already resets manual mode inside
        } else {
             resetManualMode(); // Just exit if timeout in earlier stages
        }
        return; // Exit handleManualMode
    }

    // Get keypad input
    uint8_t key = keypad.getKey();
    if (key == I2C_KEYPAD_NOKEY) {
        // Check verification timeout only if keypad wasn't pressed
        // And only if we are actually in the verification stage
        if (fireType != '0' && fireIntensity != '0' && verificationStartTime > 0) {
             if (millis() - verificationStartTime > VERIFICATION_TIMEOUT) {
                  Serial.println("Verification timed out. Auto-sending unverified alert.");
                  sendAlertData(false); // Send unverified data
                  // resetManualMode() is called inside sendAlertData
                  return;
             }
        }
        return; // No key pressed, nothing more to do now
    }

    // --- Key Pressed ---
    char inputChar = keypad.getChar();
    Serial.printf("Manual Mode Keypress: %c\n", inputChar);

    // Reset the step timeout timer on any valid key press
    manualModeTimeoutStart = millis();
    verificationStartTime = 0; // Reset verification timer if any key is pressed before '*' or '#' in that stage

    // Process input based on the current stage (which selection is pending)
    if (fireType == '0') {
        // --- Stage: Select Fire Type ---
        if (inputChar == 'A' || inputChar == 'B' || inputChar == 'C' || inputChar == 'D') {
            fireType = inputChar;
            Serial.printf("Fire Type selected: %c\n", fireType);
            displaySelectedFireType(); // Shows selection, then calls displayFireIntensityPrompt()
            manualModeTimeoutStart = millis(); // Reset timer for next step
        } else if (inputChar == '#') {
             Serial.println("Manual Mode Cancelled by user (#).");
             resetManualMode();
        } else {
            displayInvalidInput("Type: A/B/C/D or #");
             delay(1000); // Short delay after invalid input message
             displayFireTypePrompt(); // Show prompt again
            manualModeTimeoutStart = millis(); // Reset timer
        }

    } else if (fireIntensity == '0') {
        // --- Stage: Select Fire Intensity ---
        if (inputChar >= '1' && inputChar <= '4') {
            fireIntensity = inputChar;
            Serial.printf("Fire Intensity selected: %c\n", fireIntensity);
            displaySelectedFireIntensity(); // Shows selection, then calls displayVerificationPrompt()
            manualModeTimeoutStart = millis(); // Reset timer for next step
            verificationStartTime = millis(); // Start the specific timer for the verification step
        } else if (inputChar == '#') {
             Serial.println("Manual Mode Cancelled by user (#).");
             resetManualMode();
        } else {
            displayInvalidInput("Intensity: 1/2/3/4 or #");
             delay(1000);
             displayFireIntensityPrompt(); // Show prompt again
            manualModeTimeoutStart = millis(); // Reset timer
        }

    } else {
        // --- Stage: Verification ---
        if (inputChar == '*') {
            Serial.println("Verification confirmed (*). Sending verified alert.");
            sendAlertData(true); // Send verified data
            // resetManualMode() is called inside sendAlertData
        } else if (inputChar == '#') {
            Serial.println("Manual Mode Cancelled by user (#).");
            resetManualMode();
        } else {
            // Any other key in verification resets the timer and shows prompt again
            displayInvalidInput("Verify: * or # only");
            delay(1000);
            displayVerificationPrompt(); // Show prompt again
            manualModeTimeoutStart = millis(); // Reset step timer
            verificationStartTime = millis(); // Restart verification timer
        }
    }
}

void resetManualMode() {
    manualModeActive = false;
    fireType = '0';
    fireIntensity = '0';
    currentUser = "";
    currentUserID = "";
    currentUserIsOG = false;
    memset(currentUID, 0, sizeof(currentUID));
    verificationStartTime = 0;
    manualModeTimeoutStart = 0;
    keypadTimeoutStart = 0; // Also reset menu timeout just in case
    Serial.println("Manual Mode Reset.");
    // Don't call resetDisplay() here if called from sendAlertData,
    // as printCommStatus will handle the display update and reset.
    // If called directly (e.g., cancellation), then reset display.
    if (!isDisplayingSentStatus) {
       resetDisplay();
    }
}

// ======================= MENU SYSTEM & CONFIGURATION (No changes needed here) =======================
void showMainMenu() {
    Serial.println("Entering Main Menu.");
    resetKeypadTimeout(); // Start the menu timeout timer

    while (true) {
        displayMenu(); // Show the menu options

        // Check for menu timeout
        if (keypadEntryTimeout(MENU_TIMEOUT)) {
            Serial.println("Main Menu Timed Out.");
            resetDisplay(); // Return to default screen
            return; // Exit menu function
        }

        // Check for keypad input
        uint8_t key = keypad.getKey();
        if (key != I2C_KEYPAD_NOKEY) {
            char input = keypad.getChar();
            Serial.printf("Menu Input: %c\n", input);
            resetKeypadTimeout(); // Reset timeout on any key press

            switch (input) {
                case '1': // Manual Fire Alert
                    activateManualMode(); // Start the manual alert process
                    return; // Exit menu function (manual mode handles flow now)
                case '2': // Manage Users
                    handleUserManagement(); // Go to user management sub-menu
                    resetKeypadTimeout(); // Reset timer after returning from submenu
                    // Loop continues, displayMenu() will redraw
                    break;
                case '3': // Manage WiFi
                    configPortal(); // Start WiFiManager config portal
                    resetKeypadTimeout();
                    // Loop continues
                    break;
                case '4': // Switch Communication Mode
                    switchCommMode();
                    resetKeypadTimeout();
                    // Loop continues
                    break;
                case '#': // Exit Menu
                    Serial.println("Exiting Main Menu (#).");
                    resetDisplay(); // Return to default screen
                    return; // Exit menu function
                default:
                    displayInvalidInput("Invalid Option!");
                    // delay(1000); // displayInvalidInput has delay
                    resetKeypadTimeout();
                    // Loop continues
                    break;
            }
        }
        delay(100); // Small delay to prevent high CPU usage
    }
}

void displayMenu() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("--- MAIN MENU ---");
    display.println("(# = Back/Exit)");
    display.drawFastHLine(0, 18, SCREEN_WIDTH, SH110X_WHITE);
    display.setCursor(0, 20); // Adjusted starting position

    // Display menu items
    for (int i = 0; i < MENU_ITEMS; i++) {
        display.println(menuOptions[i]);
    }
    // Display current Comm Mode at the bottom
    display.setCursor(0, display.getCursorY() + 2); // Add some space
    display.printf("Mode: %s", (commMode == UDP_MODE) ? "UDP" : "ESP-NOW");
    display.display();
}

void switchCommMode() {
    commMode = (commMode == UDP_MODE) ? ESPNOW_MODE : UDP_MODE; // Toggle mode
    EEPROM.put(COMM_MODE_EEPROM_ADDR, (uint8_t)commMode);
    EEPROM.commit();

    Serial.printf("Communication Mode switched to: %s\n", (commMode == UDP_MODE) ? "UDP" : "ESP-NOW");

    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("Communication Mode:");
    display.setCursor(0, 30);
    display.setTextSize(2); // Larger text for mode display
    display.println((commMode == UDP_MODE) ? "UDP" : "ESP-NOW");
    display.setTextSize(1); // Reset text size
    display.display();
    delay(2500); // Show confirmation
}

void configPortal() {
    WiFiManager wifiManager;

    // Configure WiFiManager settings
    wifiManager.setConfigPortalTimeout(20); // 3-minute timeout
    wifiManager.setShowPassword(true); // Allow seeing password

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("--- WiFi Setup ---");
    display.println("Connect Phone/PC to:");
    String apName = "FireLinx " + stationID;
    display.println("AP: " + apName);
    display.println("Pass: (none)"); // Default, can be set
    display.println("Go to 192.168.4.1");
    display.display();
    Serial.printf("Starting WiFi Config Portal (AP: %s, IP: 192.168.4.1)\n", apName.c_str());


    // Start the portal - blocking until timeout or connection
    if (!wifiManager.startConfigPortal(apName.c_str())) { // Use dynamic AP name
        Serial.println("WiFi Config Portal Failed to Connect or Timed Out.");
        display.clearDisplay();
        display.setCursor(0, 10);
        display.println("Config Failed or");
        display.println("Timed Out.");
        display.display();
    } else {
        // WiFi connected successfully via portal
        Serial.println("WiFi Configured Successfully via Portal!");
        Serial.print("Connected to SSID: "); Serial.println(WiFi.SSID());
        Serial.print("IP Address: "); Serial.println(WiFi.localIP());

        // Save the new credentials to EEPROM
        strncpy(wifiCreds.ssid, WiFi.SSID().c_str(), sizeof(wifiCreds.ssid) - 1);
        wifiCreds.ssid[sizeof(wifiCreds.ssid) - 1] = '\0';
        strncpy(wifiCreds.password, WiFi.psk().c_str(), sizeof(wifiCreds.password) - 1);
        wifiCreds.password[sizeof(wifiCreds.password) - 1] = '\0';
        EEPROM.put(WIFI_CREDS_ADDR, wifiCreds);
        if(!EEPROM.commit()){
            Serial.println("EEPROM Commit Failed!");
        } else {
            Serial.println("New WiFi credentials saved to EEPROM.");
        }


        display.clearDisplay();
        display.setCursor(0, 10);
        display.println("WiFi Connected!");
        display.println(WiFi.SSID());
        display.println(WiFi.localIP());
        display.display();

        // Re-initialize UDP and ESP-NOW with new connection details
        udpAddress = WiFi.localIP();
        udpAddress[3] = 255;
        udp.stop(); // Stop previous instance
        udp.begin(UDP_PORT); // Re-initialize UDP binding
        Serial.println("UDP Re-initialized.");

         initESP_NOW(); // Re-initialize ESP-NOW (adds peer, sets channel implicitly)

    }
    delay(3000); // Show status briefly
    resetDisplay(); // Return to normal display after portal interaction
}

// ======================= USER MANAGEMENT (No changes needed here) =======================
void handleUserManagement() {
    Serial.println("Entering User Management.");

    // Step 1: Require authentication (any valid card) to access management functions
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("--- User Admin ---");
    display.setCursor(0, 30);
    display.println("Scan Your RFID Tag");
    display.display();

    // Use a temporary variable to store auth result without clobbering globals yet
    String authUser;
    String authUserID;
    bool authUserIsOG = false;
    uint8_t authUID[4];

     unsigned int scanTimeout = 10000; // Longer timeout for admin auth
     uint8_t tempUidBuffer[7] = {0};
     uint8_t tempUidLength;
     unsigned long authStartTime = millis();
     bool authSuccess = false;

     Serial.printf("Scanning for Admin RFID tag (Timeout: %d ms)...\n", scanTimeout);

     while (millis() - authStartTime < scanTimeout) {
         bool found = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, tempUidBuffer, &tempUidLength, 50);
         if (found && tempUidLength == 4) {
             if (validateUID(tempUidBuffer)) {
                 int userIndex = findUserIndexByUID(tempUidBuffer);
                 if (userIndex != -1) {
                     // Found a registered user
                     authUser = String(users[userIndex].name);
                     authUserID = String(users[userIndex].id);
                     authUserIsOG = users[userIndex].isOG;
                     memcpy(authUID, users[userIndex].uid, 4);
                     authSuccess = true;
                     Serial.printf("Admin Auth Success: %s (%s), OG: %d\n", authUser.c_str(), authUserID.c_str(), authUserIsOG);
                     break; // Exit loop on success
                 } else {
                     // Unregistered card - show message and fail auth
                     Serial.println("Admin Auth Failed: Unregistered Card Scanned.");
                     display.clearDisplay(); display.setCursor(0, 10); display.println("Unregistered Card!"); display.display(); delay(2000);
                     authSuccess = false;
                     break; // Exit loop on finding unregistered card
                 }
             } else {
                 // Invalid UID - show message and fail auth
                 Serial.println("Admin Auth Failed: Invalid UID Scanned.");
                 display.clearDisplay(); display.setCursor(0, 10); display.println("Invalid Card UID!"); display.display(); delay(2000);
                 authSuccess = false;
                 break; // Exit loop on finding invalid card
             }
         }
         delay(50);
     }

     if (!authSuccess && (millis() - authStartTime >= scanTimeout)) {
          Serial.println("Admin Auth Failed: Timeout.");
          display.clearDisplay(); display.setCursor(0, 10); display.println("Auth Timeout!"); display.display(); delay(2000);
     }


    if (!authSuccess) {
        Serial.println("User Management Auth Failed.");
        resetDisplay(); // Go back to main display
        return; // Exit user management
    }

    // Authentication successful, NOW update global variables for the admin session
    currentUser = authUser;
    currentUserID = authUserID;
    currentUserIsOG = authUserIsOG;
    memcpy(currentUID, authUID, 4);


    // User authenticated, proceed to menu
    Serial.printf("User '%s' authenticated for User Management.\n", currentUser.c_str());
    resetKeypadTimeout(); // Start timeout for the submenu itself

    while (true) {
        // Display User Management Menu
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("--- User Admin ---");
        display.printf("User: %s%s\n", currentUser.c_str(), currentUserIsOG ? " (OG)" : "");
        display.drawFastHLine(0, 18, SCREEN_WIDTH, SH110X_WHITE);
        display.setCursor(0, 20);
        display.println("1. Add User (OG Only)"); // Add requires OG
        display.println("2. Remove User");        // Remove has sub-options
        display.println("3. Promote User (OG Only)"); // Promote requires OG
        display.println("#. Back");
        display.display();

        // Check for timeout
        if (keypadEntryTimeout(15000)) { // Longer timeout for admin menu
            Serial.println("User Management Menu Timed Out.");
            resetDisplay(); // Go back to main display
             currentUser = ""; // Clear admin user info on exit
             currentUserID = "";
             currentUserIsOG = false;
             memset(currentUID, 0, sizeof(currentUID));
            return; // Exit user management
        }

        // Check for keypad input
        uint8_t key = keypad.getKey();
        if (key != I2C_KEYPAD_NOKEY) {
            char input = keypad.getChar();
            Serial.printf("User Mgmt Input: %c\n", input);
            resetKeypadTimeout(); // Reset timer

            switch (input) {
                case '1': // Add User
                    if (currentUserIsOG) {
                        addUser();
                    } else {
                        displayInvalidInput("OG Privilege Req!");
                    }
                    resetKeypadTimeout(); // Reset timer after action
                    break;
                case '2': // Remove User (leads to sub-menu)
                    removeUser(); // Handles own auth checks inside if needed
                    resetKeypadTimeout();
                    break;
                case '3': // Promote User
                    if (currentUserIsOG) {
                        promoteUser();
                    } else {
                        displayInvalidInput("OG Privilege Req!");
                    }
                    resetKeypadTimeout();
                    break;
                case '#': // Back to Main Menu
                    Serial.println("Exiting User Management.");
                     currentUser = ""; // Clear admin user info on exit
                     currentUserID = "";
                     currentUserIsOG = false;
                     memset(currentUID, 0, sizeof(currentUID));
                     resetDisplay(); // Go back to main display
                    return; // Exit this function
                default:
                    displayInvalidInput("Invalid Option!");
                    resetKeypadTimeout();
                    break;
            }
             // After handling an action, the loop continues, redrawing the menu
        }
        delay(100); // Prevent busy-waiting
    }
}

void addUser() {
    if (!currentUserIsOG) {
         Serial.println("Add User Failed: Current user is not OG.");
         return;
    }
    if (userCount >= MAX_USERS) {
        Serial.println("Add User Failed: User database full.");
        display.clearDisplay(); display.setCursor(0,10); display.println("User Limit Reached!"); display.display(); delay(2000);
        return;
    }

    Serial.println("Attempting to Add New User.");
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("--- Add User ---");
    display.setCursor(0, 30);
    display.println("Scan NEW RFID Tag");
    display.display();

    uint8_t newUid[7] = {0}; // Buffer for scanned UID (up to 7 bytes)
    uint8_t newUidLength;
    unsigned long scanStart = millis();

    while (millis() - scanStart < RFID_TIMEOUT) {
        bool found = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, newUid, &newUidLength, 50); // 50ms timeout per attempt

        if (found && newUidLength == 4) {
             Serial.print("New Tag Scanned, UID: ");
             for(int i=0; i<4; i++) Serial.printf("%02X ", newUid[i]);
             Serial.println();

            if (!validateUID(newUid)) {
                Serial.println("Invalid UID scanned.");
                display.clearDisplay(); display.setCursor(0,10); display.println("Invalid Card UID!"); display.display(); delay(2000);
                return; // Exit add user
            }
            if (findUserIndexByUID(newUid) != -1) {
                Serial.println("User with this UID already exists.");
                display.clearDisplay(); display.setCursor(0,10); display.println("Card Already Added!"); display.display(); delay(2000);
                return; // Exit add user
            }

            User newUser;
            memcpy(newUser.uid, newUid, 4);
            String guestName = "Guest" + String(userCount + 1);
            String guestID = "G" + String(userCount + 1);
            strncpy(newUser.name, guestName.c_str(), sizeof(newUser.name) - 1);
            newUser.name[sizeof(newUser.name) - 1] = '\0';
            strncpy(newUser.id, guestID.c_str(), sizeof(newUser.id) - 1);
            newUser.id[sizeof(newUser.id) - 1] = '\0';
            newUser.isOG = false; // New users are Guests

            users[userCount] = newUser;
            userCount++;
            saveUsersToEEPROM();

            Serial.printf("User Added: %s (%s)\n", newUser.name, newUser.id);
            display.clearDisplay();
            display.setCursor(0, 10);
            display.println("--- User Added ---");
            display.setCursor(0, 30);
            display.println("Name: " + String(newUser.name));
            display.println("ID:   " + String(newUser.id));
            display.println("(Guest Status)");
            display.display();
            delay(3000);
            return; // Success

        } else if (found) {
             Serial.printf("Unsupported tag type scanned (Length: %d)\n", newUidLength);
             display.clearDisplay(); display.setCursor(0,10); display.println("Unsupported Card!"); display.display(); delay(2000);
             return;
        }
        delay(50); // Small delay between scan attempts
    }

    Serial.println("Add User Failed: Scan timed out.");
    display.clearDisplay(); display.setCursor(0,10); display.println("Scan Timed Out!"); display.display(); delay(2000);
}

void removeUser() {
    Serial.println("Entering Remove User menu.");
    resetKeypadTimeout();

    while(true) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("--- Remove User ---");
        display.drawFastHLine(0, 10, SCREEN_WIDTH, SH110X_WHITE);
        display.setCursor(0, 15);
        display.println("1. Remove MY Card");
        display.println("2. Remove Guest (OG)"); // Requires OG
        display.println("#. Back");
        display.display();

        if (keypadEntryTimeout(10000)) {
             Serial.println("Remove User menu timed out.");
             return; // Back to user mgmt menu
        }

        uint8_t key = keypad.getKey();
        if (key != I2C_KEYPAD_NOKEY) {
             char input = keypad.getChar();
             Serial.printf("Remove User Input: %c\n", input);
             resetKeypadTimeout();

             switch (input) {
                 case '1':
                     removeSelf(); // Ask to remove the card used for admin auth
                     // After removeSelf returns, loop back to user mgmt menu display
                     return; // Exit remove sub-menu
                 case '2':
                     if (currentUserIsOG) {
                         removeGuest(); // Ask to scan a guest card to remove
                     } else {
                         displayInvalidInput("OG Privilege Req!");
                         resetKeypadTimeout();
                     }
                      // Loop continues after attempt/message
                     break;
                 case '#':
                     Serial.println("Exiting Remove User menu.");
                     return; // Back to user mgmt menu
                 default:
                     displayInvalidInput("Invalid Option!");
                     resetKeypadTimeout();
                     break;
             }
        }
         delay(100);
    }
}

void removeSelf() {
    Serial.println("Attempting to Remove Self (Admin Card).");

    // Find index of the current admin user
    int indexToRemove = findUserIndexByUID(currentUID); // Use UID scanned for admin auth
    if (indexToRemove == -1) {
        Serial.println("Error: Could not find current admin user in database.");
        display.clearDisplay(); display.setCursor(0,10); display.println("Internal Error!"); display.display(); delay(2000);
        return;
    }

    // Prevent removing the last OG user
    if (users[indexToRemove].isOG) {
        int ogCount = 0;
        for (int i = 0; i < userCount; i++) {
            if (users[i].isOG) ogCount++;
        }
        if (ogCount <= 1) {
            Serial.println("Remove Self Failed: Cannot remove the last OG user.");
            display.clearDisplay(); display.setCursor(0,10); display.println("Cannot Remove"); display.println("Last OG User!"); display.display(); delay(2500);
            return;
        }
    }

    // Confirmation Step: Re-scan the same card
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("--- Confirm Remove ---");
    display.setCursor(0, 30);
    display.println("Re-Scan Your Card");
    display.println("To Confirm Removal");
    display.display();

    uint8_t confirmUid[7] = {0};
    uint8_t confirmUidLength;
    unsigned long scanStart = millis();
    bool confirmed = false;

    while(millis() - scanStart < RFID_TIMEOUT) {
         bool found = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, confirmUid, &confirmUidLength, 50);
         if (found && confirmUidLength == 4) {
              if (memcmp(confirmUid, currentUID, 4) == 0) { // Compare with admin UID
                   confirmed = true;
                   break; // Match found
              } else {
                   Serial.println("Confirmation failed: Scanned wrong card.");
                   display.clearDisplay(); display.setCursor(0,10); display.println("Wrong Card Scanned!"); display.display(); delay(2000);
                   return; // Exit removal process
              }
         }
         delay(50);
    }

    if (!confirmed) {
        Serial.println("Confirmation failed: Timeout.");
        display.clearDisplay(); display.setCursor(0,10); display.println("Confirmation Timeout!"); display.display(); delay(2000);
        return; // Exit removal process
    }

    // --- Proceed with Removal ---
    Serial.printf("Removing user: %s (%s)\n", users[indexToRemove].name, users[indexToRemove].id);

    // Shift users down
    for (int i = indexToRemove; i < userCount - 1; i++) {
        users[i] = users[i + 1];
    }
    userCount--;

    saveUsersToEEPROM(); // Save changes

    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("--- User Removed ---");
    display.setCursor(0, 30);
    display.println("Your card has been");
    display.println("removed from system.");
    display.display();
    delay(3000);

    // Clear current admin user info as their card is gone
    currentUser = "";
    currentUserID = "";
    currentUserIsOG = false;
    memset(currentUID, 0, sizeof(currentUID));
    // Return will go back to the user mgmt menu loop, which will now fail auth or show blank user
}

void removeGuest() {
    if (!currentUserIsOG) return;

    Serial.println("Attempting to Remove Guest User.");
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("--- Remove Guest ---");
    display.setCursor(0, 30);
    display.println("Scan GUEST Card");
    display.println("To Be Removed");
    display.display();

    uint8_t guestUid[7] = {0};
    uint8_t guestUidLength;
    unsigned long scanStart = millis();

    while(millis() - scanStart < RFID_TIMEOUT) {
        bool found = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, guestUid, &guestUidLength, 50);
        if (found && guestUidLength == 4) {
             Serial.print("Guest Card Scanned, UID: ");
             for(int i=0; i<4; i++) Serial.printf("%02X ", guestUid[i]);
             Serial.println();

            if (!validateUID(guestUid)) {
                 Serial.println("Invalid UID scanned.");
                 display.clearDisplay(); display.setCursor(0,10); display.println("Invalid Card UID!"); display.display(); delay(2000);
                return;
            }

            int indexToRemove = findUserIndexByUID(guestUid);

            if (indexToRemove == -1) {
                Serial.println("Remove Guest Failed: Card not found.");
                display.clearDisplay(); display.setCursor(0,10); display.println("Card Not Found!"); display.display(); delay(2000);
                return;
            }
            if (users[indexToRemove].isOG) {
                Serial.println("Remove Guest Failed: Scanned card is OG user.");
                display.clearDisplay(); display.setCursor(0,10); display.println("Cannot Remove OG User"); display.println("With This Option."); display.display(); delay(2500);
                return;
            }

            // --- Proceed with Removal ---
            Serial.printf("Removing guest user: %s (%s)\n", users[indexToRemove].name, users[indexToRemove].id);
            for (int i = indexToRemove; i < userCount - 1; i++) {
                users[i] = users[i + 1];
            }
            userCount--;
            saveUsersToEEPROM();

            display.clearDisplay();
            display.setCursor(0, 10);
            display.println("--- Guest Removed ---");
            display.setCursor(0, 30);
            display.println("Guest user removed");
            display.println("successfully.");
            display.display();
            delay(3000);
            return; // Success
        }
        delay(50);
    }

    Serial.println("Remove Guest Failed: Scan timed out.");
    display.clearDisplay(); display.setCursor(0,10); display.println("Scan Timed Out!"); display.display(); delay(2000);
}

void promoteUser() {
     if (!currentUserIsOG) return;

    Serial.println("Attempting to Promote User.");
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("--- Promote User ---");
    display.setCursor(0, 30);
    display.println("Scan GUEST Card");
    display.println("To Promote to OG");
    display.display();

    uint8_t promoteUid[7] = {0};
    uint8_t promoteUidLength;
    unsigned long scanStart = millis();

    while(millis() - scanStart < RFID_TIMEOUT) {
        bool found = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, promoteUid, &promoteUidLength, 50);
        if (found && promoteUidLength == 4) {
             Serial.print("Card to Promote Scanned, UID: ");
             for(int i=0; i<4; i++) Serial.printf("%02X ", promoteUid[i]);
             Serial.println();

            if (!validateUID(promoteUid)) {
                 Serial.println("Invalid UID scanned.");
                 display.clearDisplay(); display.setCursor(0,10); display.println("Invalid Card UID!"); display.display(); delay(2000);
                return;
            }
            int indexToPromote = findUserIndexByUID(promoteUid);

            if (indexToPromote == -1) {
                Serial.println("Promote User Failed: Card not found.");
                display.clearDisplay(); display.setCursor(0,10); display.println("Card Not Found!"); display.display(); delay(2000);
                return;
            }
            if (users[indexToPromote].isOG) {
                Serial.println("Promote User Failed: User is already OG.");
                display.clearDisplay(); display.setCursor(0,10); display.println("User Already OG!"); display.display(); delay(2000);
                return;
            }

            // --- Proceed with Promotion ---
            Serial.printf("Promoting user: %s (%s) to OG status.\n", users[indexToPromote].name, users[indexToPromote].id);
            users[indexToPromote].isOG = true;
            saveUsersToEEPROM();

            display.clearDisplay();
            display.setCursor(0, 10);
            display.println("--- User Promoted ---");
            display.setCursor(0, 30);
            display.printf("User %s\n", users[indexToPromote].name);
            display.println("Promoted to OG!");
            display.display();
            delay(3000);
            return; // Success
        }
        delay(50);
    }

    Serial.println("Promote User Failed: Scan timed out.");
    display.clearDisplay(); display.setCursor(0,10); display.println("Scan Timed Out!"); display.display(); delay(2000);
}

void loadUsersFromEEPROM() {
    EEPROM.get(USER_COUNT_ADDR, userCount);
    if (userCount < OG_USERS || userCount > MAX_USERS) {
        Serial.printf("Invalid user count (%d) loaded. Resetting to %d default users.\n", userCount, OG_USERS);
        userCount = OG_USERS;
        users[0] = {{0xEE, 0x5A, 0xB5, 0x2}, "Gobinda Sir", "D01", true};
        users[1] = {{0x33, 0x88, 0xA4, 0x2C}, "Sayan Sir", "D02", true};
        for(int i = OG_USERS; i < MAX_USERS; ++i) {
            memset(&users[i], 0, sizeof(User));
        }
        saveUsersToEEPROM();
    } else {
        EEPROM.get(USERS_ARRAY_ADDR, users);
        Serial.printf("Loaded %d users from EEPROM.\n", userCount);
        // Optional: Validate loaded data (e.g., check names are null-terminated)
         for(int i=0; i<userCount; ++i) {
             users[i].name[sizeof(users[i].name)-1] = '\0';
             users[i].id[sizeof(users[i].id)-1] = '\0';
         }
    }
}

void saveUsersToEEPROM() {
    EEPROM.put(USER_COUNT_ADDR, userCount);
    EEPROM.put(USERS_ARRAY_ADDR, users);
    if (EEPROM.commit()) {
         Serial.printf("User data (%d users) saved successfully to EEPROM.\n", userCount);
    } else {
         Serial.println("ERROR: Failed to commit user data to EEPROM!");
         display.clearDisplay(); display.setCursor(0,10); display.println("EEPROM SAVE FAIL!"); display.display(); delay(2000);
    }
}

// ======================= RFID & AUTHENTICATION (No changes needed here) =======================
bool checkRFID(unsigned int timeout_ms) {
    uint8_t uidBuffer[7] = {0};
    uint8_t uidLength;
    unsigned long startTime = millis();

    Serial.printf("Scanning for RFID tag (Timeout: %d ms)...\n", timeout_ms);
    // Display updates are handled by this function directly

    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("SCAN RFID TAG");
    display.println("(Timeout: " + String(timeout_ms/1000) + "s)");
    display.display();


    while (millis() - startTime < timeout_ms) {
        // Use the PN532 library's detection method
        bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uidBuffer, &uidLength, 50); // 50ms timeout for non-blocking check

        if (success) {
            if (uidLength == 4) {
                Serial.print("Tag Found! UID: ");
                for (uint8_t i = 0; i < uidLength; i++) { Serial.print(uidBuffer[i], HEX); Serial.print(" "); }
                Serial.println();

                if (!validateUID(uidBuffer)) {
                    Serial.println("Invalid UID detected.");
                    display.clearDisplay(); display.setCursor(0,10); display.println("Invalid Card UID!"); display.display(); delay(2000);
                    return false; // Invalid UID found
                }

                int userIndex = findUserIndexByUID(uidBuffer);

                if (userIndex != -1) {
                    // --- User Found ---
                    currentUser = String(users[userIndex].name);
                    currentUserID = String(users[userIndex].id);
                    currentUserIsOG = users[userIndex].isOG;
                    memcpy(currentUID, users[userIndex].uid, 4); // Store UID

                    Serial.printf("Registered User Found: %s (%s), OG: %d\n", currentUser.c_str(), currentUserID.c_str(), currentUserIsOG);
                    // Display success message handled by caller (activateManualMode)
                    return true; // Valid, registered user found

                } else {
                    // --- User Not Found (Unregistered Tag) ---
                    Serial.println("Unregistered Tag detected.");
                    memcpy(currentUID, uidBuffer, 4); // Store UID anyway
                    currentUser = ""; // Clear user info
                    currentUserID = "";
                    currentUserIsOG = false;

                    display.clearDisplay();
                    display.setCursor(0, 10);
                    display.println("--- New Card ---");
                    display.setCursor(0, 30);
                    display.print("UID: ");
                    for (uint8_t i = 0; i < 4; i++) { display.print(uidBuffer[i], HEX); display.print(" "); }
                    display.println("\n(Not Registered)");
                    display.display();
                    delay(2500);
                    return false; // Valid UID, but not registered
                }
            } else {
                 Serial.printf("Unsupported tag length detected: %d bytes\n", uidLength);
                 display.clearDisplay(); display.setCursor(0,10); display.println("Unsupported Card!"); display.display(); delay(2000);
                 return false; // Unsupported tag type
            }
        }
        // Update display while waiting
        display.setCursor(SCREEN_WIDTH - 6, SCREEN_HEIGHT - 8); // Bottom right corner
        display.print(".");
        display.display();
        delay(50); // Wait a bit before next scan attempt
    }

    // If loop finishes, timeout occurred
    Serial.println("RFID Scan Timed Out.");
    display.clearDisplay(); display.setCursor(0,10); display.println("Scan Timed Out!"); display.display(); // Show timeout message
    // delay handled by caller
    return false; // Timeout
}

bool validateUID(uint8_t* uid) {
    if (uid == nullptr) return false;
    bool allZeros = true;
    for (int i = 0; i < 4; i++) if (uid[i] != 0x00) allZeros = false;
    if (allZeros) return false;

    bool allFFs = true;
    for (int i = 0; i < 4; i++) if (uid[i] != 0xFF) allFFs = false;
    if (allFFs) return false;

    return true; // Looks valid
}

int findUserIndexByUID(const uint8_t* uid_to_find) {
     if (uid_to_find == nullptr) return -1;
    for (int i = 0; i < userCount; i++) {
        if (memcmp(uid_to_find, users[i].uid, 4) == 0) {
            return i; // Found at index i
        }
    }
    return -1; // Not found
}

// ======================= DISPLAY & UI FUNCTIONS (No changes needed here) =======================
void resetDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);

    // Line 1: System Title and Station ID
    display.printf("FireLinx %s\n", stationID.c_str());

    // Line 2: Date and Time
    display.printf("D:%s T:%s\n", gpsDate.c_str(), gpsTime.c_str());

    // Line 3: Status & Comm Mode
     if (millis() < STABILIZATION_PERIOD_MS) {
         display.printf("Status: STABILIZING\n");
     } else {
         display.printf("Status: %s | %s\n", manualModeActive ? "MANUAL" : "AUTO", (commMode == UDP_MODE) ? "UDP" : "ESPNOW");
     }


    // Line 4-5: GPS Coordinates (if valid)
    if (gps.location.isValid() && previousLatitude != "N/A") {
        display.printf("Lat: %s\n", previousLatitude.c_str());
        display.printf("Lon: %s\n", previousLongitude.c_str());
    } else {
        display.println("GPS: Acquiring...");
        display.println(" "); // Placeholder line
    }

    // Line 6: Sensors
    // Use digitalRead with LOW check based on assumed sensor logic
    display.printf("T:%.1fC MQ2:%d F:%d\n", currentTemp, digitalRead(MQ2_D0) == LOW, digitalRead(FLAME_SENSOR_PIN) == LOW);

    // Line 7: Prompt / Info
    if (!manualModeActive && millis() > STABILIZATION_PERIOD_MS) {
       display.println("Hold Touch for Menu");
    } else {
        display.println(" "); // Empty line otherwise
    }

    display.display();
}

void displayFireTypePrompt() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("--- Manual Alert ---");
    display.setCursor(0, 15);
    display.println("Select Fire Type:");
    display.println(" A: Class A (Solid)");
    display.println(" B: Class B (Liquid)");
    display.println(" C: Class C (Gas)");
    display.println(" D: Class D (Metal)");
    display.println(" (# = Cancel)");
    display.display();
}

void displayFireIntensityPrompt() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("--- Manual Alert ---");
    display.setCursor(0, 15);
    display.println("Select Intensity:");
    display.println(" 1: Small / Initial");
    display.println(" 2: Moderate / Growing");
    display.println(" 3: High / Spreading");
    display.println(" 4: Severe / Intense");
    display.println(" (# = Cancel)");
    display.display();
}

void displayVerificationPrompt() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("--- Confirm Alert ---");
    display.setCursor(0, 15);
    display.printf("Type: %c, Intensity: %c\n", fireType, fireIntensity);
    display.printf("User: %s\n", currentUser.c_str());
    display.println("--------------------");
    display.println(" (*) Verify & SEND");
    display.println(" (#) Cancel Alert");
    display.printf(" (Auto-Send in %lds)", VERIFICATION_TIMEOUT / 1000);
    display.display();
}

void displayInvalidInput(String message) {
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("--- Invalid Input ---");
    display.setCursor(0, 30);
    display.println(message);
    display.display();
    delay(1500); // Show message for 1.5 seconds
    // Calling function should redraw the appropriate prompt afterwards
}

void displaySelectedFireType() {
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("--- Type Selected ---");
    display.setCursor(0, 30);
    display.print("Fire Type: Class ");
    display.println(fireType);
    display.display();
    delay(1500);
    displayFireIntensityPrompt(); // Move to next step
}

void displaySelectedFireIntensity() {
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("--- Intensity Set ---");
    display.setCursor(0, 30);
    display.print("Intensity: ");
    switch (fireIntensity) {
        case '1': display.println("Small"); break;
        case '2': display.println("Moderate"); break;
        case '3': display.println("High"); break;
        case '4': display.println("Severe"); break;
        default: display.println("Unknown"); break;
    }
    display.display();
    delay(1500);
    displayVerificationPrompt(); // Move to final verification step
}

// ======================= GPS FUNCTIONS (No changes needed here) =======================
void updateGPSData() {
    bool dataUpdated = false;
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            dataUpdated = true; // Mark that new data was processed
        }
    }

    // Only update global variables if new data was parsed by gps.encode()
    if (dataUpdated) {
        if (gps.location.isValid() && gps.location.isUpdated()) {
            gpsLat = gps.location.lat();
            gpsLon = gps.location.lng();
            String newLat = convertToDDM(gpsLat, true);
            String newLon = convertToDDM(gpsLon, false);
            // Only update if changed to prevent unnecessary redraws
            if (newLat != previousLatitude) previousLatitude = newLat;
            if (newLon != previousLongitude) previousLongitude = newLon;

        } else if (!gps.location.isValid() && previousLatitude != "N/A") {
              gpsLat = 0.0; gpsLon = 0.0;
              previousLatitude = "N/A"; previousLongitude = "N/A";
        }

        if (gps.date.isValid() && gps.date.isUpdated()) {
             char dateBuf[10];
             snprintf(dateBuf, sizeof(dateBuf), "%02d/%02d", gps.date.day(), gps.date.month());
             String newDate = String(dateBuf);
             if (newDate != gpsDate) gpsDate = newDate;
        } else if (!gps.date.isValid() && gpsDate != "N/A") {
             gpsDate = "N/A";
        }

        if (gps.time.isValid() && gps.time.isUpdated()) {
            int hour = gps.time.hour();
            int minute = gps.time.minute();
            // Adjust time for IST (UTC +5:30)
            minute += 30;
            hour += 5 + (minute / 60);
            minute %= 60;
            hour %= 24;
             char timeBuf[10];
             snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", hour, minute);
             String newTime = String(timeBuf);
             if(newTime != gpsTime) gpsTime = newTime;
        } else if (!gps.time.isValid() && gpsTime != "N/A") {
              gpsTime = "N/A";
         }
    }
}

String convertToDDM(double decimalDegrees, bool isLatitude) {
    if (!isfinite(decimalDegrees) || abs(decimalDegrees) < 0.000001) return "N/A"; // Handle zero/invalid/NaN case

    char direction;
    if (isLatitude) {
        direction = (decimalDegrees >= 0) ? 'N' : 'S';
    } else {
        direction = (decimalDegrees >= 0) ? 'E' : 'W';
    }

    decimalDegrees = fabs(decimalDegrees); // Work with positive value
    int degrees = (int)decimalDegrees;
    double minutes = (decimalDegrees - degrees) * 60.0;

    // Format the string carefully DD*MM.MMMM'D
    char buffer[20];
    // Use dtostrf for better control over minute formatting if snprintf precision is inconsistent
    // dtostrf(minutes, 7, 4, min_str); // width 7, 4 decimal places
    snprintf(buffer, sizeof(buffer), "%d*%07.4f'%c", degrees, minutes, direction); // Pad minutes with leading zero if needed

    return String(buffer);
}

// ======================= UTILITY FUNCTIONS (No changes needed here) =======================
void buzzAlert(bool state) {
    digitalWrite(BUZZER_PIN, state ? HIGH : LOW);
}

bool keypadEntryTimeout(unsigned long timeout) {
    if (keypadTimeoutStart == 0) {
         // Timer not started yet (first time in menu), start it now
         keypadTimeoutStart = millis();
         // Serial.println("Keypad timeout timer started."); // Debug
         return false;
    }
    if (millis() - keypadTimeoutStart > timeout) {
        Serial.println("Keypad/Menu Timeout Occurred!");
        // Display handled by caller (showMainMenu, handleUserManagement)
        resetKeypadTimeout(); // Reset timer after timeout detected
        return true; // Timeout occurred
    }
    return false; // No timeout yet
}

void resetKeypadTimeout() {
    keypadTimeoutStart = millis(); // Set timer start to current time
    // Serial.println("Keypad timeout timer reset."); // Debug
}