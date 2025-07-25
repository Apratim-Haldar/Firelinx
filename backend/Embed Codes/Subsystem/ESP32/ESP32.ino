#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"

// Camera pins for AI Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_GPIO_NUM       4

AsyncWebServer server(80);
bool flashState = false;
bool streaming = false;

// Performance monitoring
volatile unsigned long frameCount = 0;
volatile unsigned long lastFpsCheck = 0;
volatile float currentFps = 0;

// Streaming optimization
SemaphoreHandle_t frameMutex;

// Function declarations
void handleStreamRequest(AsyncWebServerRequest *request);
void handleFlash(AsyncWebServerRequest *request);
void handleStatus(AsyncWebServerRequest *request);
size_t optimizedStreamCallback(uint8_t *buffer, size_t maxLen, size_t index);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);
  
  // Maximum performance settings
  setCpuFrequencyMhz(240);
  
  // Create mutex for frame access
  frameMutex = xSemaphoreCreateMutex();
  
  // Ultra-optimized camera configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  
  // Professional streaming settings
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;  // 640x480 for best balance
  config.jpeg_quality = 10;           // Optimized quality
  config.fb_count = 3;                // Triple buffering for smoothness
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  // Professional sensor optimization
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    // Image quality for fire detection
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    s->set_special_effect(s, 0);
    
    // Optimized auto settings
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);
    s->set_ae_level(s, 0);
    s->set_aec_value(s, 300);
    s->set_gain_ctrl(s, 1);
    s->set_agc_gain(s, 0);
    s->set_gainceiling(s, (gainceiling_t)2);
    
    // Image processing optimization
    s->set_bpc(s, 0);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
    s->set_dcw(s, 1);
    s->set_colorbar(s, 0);
    
    // Frame rate optimization
    s->set_reg(s, 0x11, 0, 0x00);
  }

  // WiFi optimization
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(120);
  
  if (!wifiManager.autoConnect("FIRELINX-CAM")) {
    Serial.println("WiFi connection failed. Restarting...");
    ESP.restart();
  }
  
  // Create separator line
  String separator = "";
  for(int i = 0; i < 50; i++) {
    separator += "=";
  }
  
  Serial.println("\n" + separator);
  Serial.println("🔥 FIRELINX Professional Camera Module");
  Serial.println(separator);
  Serial.printf("📡 Stream URL: http://%s/stream\n", WiFi.localIP().toString().c_str());
  Serial.printf("💡 Flash URL: http://%s/flash\n", WiFi.localIP().toString().c_str());
  Serial.printf("📊 Status URL: http://%s/status\n", WiFi.localIP().toString().c_str());
  Serial.println(separator);

  // Setup optimized server routes
  server.on("/stream", HTTP_GET, handleStreamRequest);
  server.on("/flash", HTTP_GET, handleFlash);
  server.on("/status", HTTP_GET, handleStatus);
  
  // CORS and caching headers
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET");
  DefaultHeaders::Instance().addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  
  server.begin();
  Serial.println("🚀 HTTP server started");
}

void loop() {
  // Minimal loop for maximum performance
  vTaskDelay(1 / portTICK_PERIOD_MS);
  
  // Update FPS counter
  if (xSemaphoreTake(frameMutex, 10) == pdTRUE) {
    frameCount++;
    if (millis() - lastFpsCheck > 1000) {
      currentFps = frameCount;
      frameCount = 0;
      lastFpsCheck = millis();
    }
    xSemaphoreGive(frameMutex);
  }
}

void handleStreamRequest(AsyncWebServerRequest *request) {
  AsyncWebServerResponse *response = request->beginChunkedResponse(
    "multipart/x-mixed-replace; boundary=frame",
    [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      return optimizedStreamCallback(buffer, maxLen, index);
    }
  );
  
  response->addHeader("Access-Control-Allow-Origin", "*");
  response->addHeader("Cache-Control", "no-cache");
  response->addHeader("Pragma", "no-cache");
  
  request->send(response);
  streaming = true;
}

size_t optimizedStreamCallback(uint8_t *buffer, size_t maxLen, size_t index) {
  static camera_fb_t *fb = nullptr;
  static size_t fbPos = 0;
  static bool headerSent = false;
  
  // Get new frame if needed
  if (fb == nullptr) {
    fb = esp_camera_fb_get();
    if (!fb) return 0;
    fbPos = 0;
    headerSent = false;
  }
  
  size_t bytesToSend = 0;
  
  if (!headerSent) {
    // Send frame header
    String header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ";
    header += String(fb->len) + "\r\n\r\n";
    
    size_t headerLen = header.length();
    if (headerLen <= maxLen) {
      memcpy(buffer, header.c_str(), headerLen);
      bytesToSend = headerLen;
      headerSent = true;
    }
  }
  else if (fbPos < fb->len) {
    // Send frame data in optimal chunks
    size_t remainingBytes = fb->len - fbPos;
    size_t chunkSize = min(remainingBytes, maxLen);
    
    memcpy(buffer, fb->buf + fbPos, chunkSize);
    fbPos += chunkSize;
    bytesToSend = chunkSize;
    
    // Check if frame is complete
    if (fbPos >= fb->len) {
      // Add frame footer if there's space
      if (chunkSize < maxLen) {
        memcpy(buffer + chunkSize, "\r\n", 2);
        bytesToSend += 2;
      }
      
      // Release frame and reset
      esp_camera_fb_return(fb);
      fb = nullptr;
    }
  }
  
  return bytesToSend;
}

void handleFlash(AsyncWebServerRequest *request) {
  flashState = !flashState;
  digitalWrite(LED_GPIO_NUM, flashState ? HIGH : LOW);
  
  String response = "{\n";
  response += "  \"status\": \"" + String(flashState ? "ON" : "OFF") + "\",\n";
  response += "  \"timestamp\": " + String(millis()) + "\n";
  response += "}";
  
  request->send(200, "application/json", response);
}

void handleStatus(AsyncWebServerRequest *request) {
  String json = "{\n";
  json += "  \"heap\": " + String(ESP.getFreeHeap()) + ",\n";
  json += "  \"psram\": " + String(ESP.getFreePsram()) + ",\n";
  json += "  \"flash\": \"" + String(flashState ? "ON" : "OFF") + "\",\n";
  json += "  \"wifi_strength\": " + String(WiFi.RSSI()) + ",\n";
  json += "  \"fps\": " + String(currentFps) + ",\n";
  json += "  \"streaming\": " + String(streaming ? "true" : "false") + ",\n";
  json += "  \"cpu_freq\": " + String(getCpuFrequencyMhz()) + ",\n";
  json += "  \"uptime\": " + String(millis()) + "\n";
  json += "}";
  
  request->send(200, "application/json", json);
}