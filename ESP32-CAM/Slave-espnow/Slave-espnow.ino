#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <esp_wifi.h>

/*
 * ESP32-CAM WebSocket Streaming + ESP-NOW Receiver + UART to STM32
 * Receives motor commands via ESP-NOW and forwards to STM32 via UART
 */

// Motor control modes
enum MotorMode {
  MODE_FORWARD = 0,
  MODE_REVERSE = 1,
  MODE_RIGHT = 2,
  MODE_LEFT = 3,
  MODE_STOP = 4
};

// Structure to receive data - must match sender
typedef struct struct_message {
  uint8_t mode;       // Motor mode (0-4)
  uint8_t pwm_right;  // Right motor PWM (0-255)
  uint8_t pwm_left;   // Left motor PWM (0-255)
  int counter;        // Message counter
} struct_message;

struct_message incomingData;

// ESP-NOW tracking variables
int lastReceivedCounter = 0;
int totalReceived = 0;
int totalDropped = 0;

// UART to STM32 (using Serial2)
// TX = GPIO1 (U0TXD - shared with USB, be careful!)
// RX = GPIO3 (U0RXD - shared with USB, be careful!)
// Better to use GPIO 12 (TX) and GPIO 13 (RX) if available
#define STM32_SERIAL Serial  // Using default Serial for UART to STM32
#define UART_BAUD 115200

// Camera pin definitions for AI-Thinker ESP32-CAM
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

// LED Flash pin for AI-Thinker ESP32-CAM
#define LED_GPIO_NUM       4

// WiFi credentials (ESP32 only supports 2.4GHz, not 5GHz!)
const char* ssid = "TT_ED71_2.4G";
const char* password = "eh9zhYE3gY";

// WebSocket client
WebSocketsClient webSocket;

// Your PC's IP address where WebSocket server runs
const char* websocket_server = "192.168.1.16"; // Your PC's IP address
const int websocket_port = 8080;

// Frame rate tracking
unsigned long lastStatsTime = 0;
unsigned long framesSent = 0;
float currentFPS = 0;

// Store total memory values (set once at startup)
uint32_t totalSRAM = 0;
uint32_t totalPSRAM = 0;

void connectToWiFi() {
  // Try primary network first (2.4GHz)
  Serial.println("Trying primary network (2.4GHz)...");
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(100);
  
  Serial.printf("Connecting to WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
    return;
  }
  
  // Connection failed
  Serial.println("\nWiFi connection failed!");
  Serial.println("ESP32 only supports 2.4GHz WiFi networks.");
  Serial.printf("SSID: %s\n", ssid);
  Serial.println("Restarting in 10 seconds...");
  delay(10000);
  ESP.restart();
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket Connected - Ready to stream");
      sendSystemStats(); // Send initial stats
      break;
    case WStype_TEXT:
      {
        // Handle camera control commands from web interface
        DynamicJsonDocument doc(512);
        DeserializationError error = deserializeJson(doc, payload);
        
        if (error) {
          Serial.println("ERROR: JSON parse failed");
          return;
        }
        
        if (!doc.containsKey("cmd")) {
          return;
        }
        
        const char* cmd = doc["cmd"];
        
        if (strcmp(cmd, "set_camera") == 0) {
          applyCameraSettings(doc);
        }
      }
      break;
    case WStype_ERROR:
      Serial.printf("WebSocket ERROR: %s\n", payload);
      break;
    default:
      break;
  }
}

void applyCameraSettings(DynamicJsonDocument& doc) {
  sensor_t * s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("ERROR: Could not get camera sensor!");
    return;
  }
  
  if (doc.containsKey("brightness")) {
    s->set_brightness(s, doc["brightness"]);
  }
  
  if (doc.containsKey("contrast")) {
    s->set_contrast(s, doc["contrast"]);
  }
  
  if (doc.containsKey("saturation")) {
    s->set_saturation(s, doc["saturation"]);
  }
  
  if (doc.containsKey("ae_level")) {
    s->set_ae_level(s, doc["ae_level"]);
  }

  if (doc.containsKey("gainceiling")) {
    s->set_gainceiling(s, (gainceiling_t)doc["gainceiling"].as<int>());
  }

  if (doc.containsKey("quality")) {
    s->set_quality(s, doc["quality"]);
  }
  
  if (doc.containsKey("aec")) {
    s->set_exposure_ctrl(s, doc["aec"]);
  }
  
  if (doc.containsKey("aec_value")) {
    s->set_aec_value(s, doc["aec_value"]);
  }
  
  if (doc.containsKey("agc")) {
    s->set_gain_ctrl(s, doc["agc"]);
  }
  
  if (doc.containsKey("agc_gain")) {
    s->set_agc_gain(s, doc["agc_gain"]);
  }
  
  if (doc.containsKey("awb")) {
    s->set_whitebal(s, doc["awb"]);
  }
  
  // LED flash control
  if (doc.containsKey("led")) {
    int enabled = doc["led"];
    if (enabled) {
      int intensity = doc.containsKey("led_intensity") ? doc["led_intensity"] : 0;
      analogWrite(LED_GPIO_NUM, intensity);
    } else {
      analogWrite(LED_GPIO_NUM, 0);
    }
  } else if (doc.containsKey("led_intensity")) {
    analogWrite(LED_GPIO_NUM, doc["led_intensity"]);
  }
}

void sendSystemStats() {
  // Calculate memory usage
  uint32_t freeSRAM = ESP.getFreeHeap();
  uint32_t freePSRAM = ESP.getFreePsram();
  
  // Create JSON stats
  DynamicJsonDocument doc(512);
  doc["type"] = "stats";
  doc["fps"] = currentFPS;
  doc["sram_free"] = freeSRAM;
  doc["sram_total"] = totalSRAM;
  doc["sram_used"] = totalSRAM - freeSRAM;
  doc["psram_free"] = freePSRAM;
  doc["psram_total"] = totalPSRAM;
  doc["psram_used"] = totalPSRAM - freePSRAM;
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["motor_mode"] = incomingData.mode;
  doc["motor_right"] = incomingData.pwm_right;
  doc["motor_left"] = incomingData.pwm_left;
  
  String output;
  serializeJson(doc, output);
  webSocket.sendTXT(output);
}

// ESP-NOW callback function - called when data is received (new API)
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  
  // Track dropped messages
  totalReceived++;
  if (lastReceivedCounter > 0) {
    int expected = lastReceivedCounter + 1;
    if (incomingData.counter != expected) {
      int dropped = incomingData.counter - expected;
      totalDropped += dropped;
      Serial.printf("WARNING: DROPPED %d messages! (Expected #%d, got #%d)\n", 
                    dropped, expected, incomingData.counter);
    }
  }
  lastReceivedCounter = incomingData.counter;
  
  // Forward to STM32 via UART
  sendToSTM32(incomingData.mode, incomingData.pwm_right, incomingData.pwm_left);
}

// Send motor commands to STM32 via UART
void sendToSTM32(uint8_t mode, uint8_t pwm_right, uint8_t pwm_left) {
  // Protocol: <START_BYTE><MODE><PWM_RIGHT><PWM_LEFT><CHECKSUM>
  // START_BYTE = 0xFF for synchronization
  // CHECKSUM = XOR of MODE, PWM_RIGHT, PWM_LEFT
  
  uint8_t checksum = mode ^ pwm_right ^ pwm_left;
  
  STM32_SERIAL.write(0xFF);         // Start byte
  STM32_SERIAL.write(mode);         // Mode
  STM32_SERIAL.write(pwm_right);    // Right motor PWM
  STM32_SERIAL.write(pwm_left);     // Left motor PWM
  STM32_SERIAL.write(checksum);     // Checksum
  STM32_SERIAL.flush();             // Ensure data is sent immediately
}

void sendCameraFrame() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR: Failed to capture frame");
    return;
  }
  
  if (fb->len == 0) {
    Serial.println("ERROR: Frame length is 0");
    esp_camera_fb_return(fb);
    return;
  }
  
  // Send frame via WebSocket
  if (webSocket.isConnected()) {
    bool sent = webSocket.sendBIN(fb->buf, fb->len);
    if (sent) {
      framesSent++;
    } else {
      Serial.println("ERROR: Failed to send frame");
    }
    
    // Calculate FPS every second
    if (millis() - lastStatsTime >= 1000) {
      currentFPS = framesSent * 1000.0 / (millis() - lastStatsTime);
      framesSent = 0;
      lastStatsTime = millis();
      sendSystemStats();
    }
  }
  
  esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(UART_BAUD);
  delay(100);
  
  Serial.println("\n\n========================================");
  Serial.println("ESP32-CAM Robot Controller");
  Serial.println("Video Stream + ESP-NOW + UART to STM32");
  Serial.println("========================================\n");
  
  // Camera configuration
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
  config.xclk_freq_hz = 18000000; // 18 MHz - balanced for stability
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Camera configuration optimized for stability (prevent FB-OVF)
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA; // VGA (640x480) - good quality
    config.jpeg_quality = 8; // Quality 8 for best quality
    config.fb_count = 2; // Use 2 buffers to prevent overflow
    config.grab_mode = CAMERA_GRAB_LATEST; // Get latest frame
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  // Get sensor and apply optimized settings
  sensor_t * s = esp_camera_sensor_get();
  
  // Optimized settings
  s->set_brightness(s, 0);     // Neutral brightness
  s->set_contrast(s, 0);       // Neutral contrast
  s->set_saturation(s, -1);    // Saturation -1
  s->set_whitebal(s, 1);       // Enable auto white balance
  s->set_awb_gain(s, 1);       // Auto white balance gain
  s->set_wb_mode(s, 0);        // Auto white balance mode
  s->set_exposure_ctrl(s, 1);  // AEC AUTO enabled
  s->set_aec2(s, 1);           // Enable AEC DSP
  s->set_ae_level(s, 1);       // AE Level 1
  s->set_gain_ctrl(s, 1);      // Enable auto gain
  s->set_agc_gain(s, 0);       // Auto AGC
  s->set_gainceiling(s, (gainceiling_t)2); // 8x gain ceiling
  s->set_bpc(s, 0);            // Disable black pixel correction
  s->set_wpc(s, 1);            // Enable white pixel correction
  s->set_raw_gma(s, 1);        // Enable gamma correction
  s->set_lenc(s, 1);           // Enable lens correction
  s->set_hmirror(s, 0);        // Disable horizontal mirror
  s->set_vflip(s, 0);          // Disable vertical flip
  s->set_dcw(s, 1);            // Enable downscale
  s->set_colorbar(s, 0);       // Disable color bar
  
  // Sensor-specific defaults
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  
  // Store total memory values once
  totalSRAM = ESP.getHeapSize();
  totalPSRAM = ESP.getPsramSize();
  
  // Initialize LED flash
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);

  // Connect to WiFi using enhanced connection function
  connectToWiFi();
  
  Serial.println("WiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  
  // Print WiFi channel for ESP-NOW master configuration
  uint8_t primary;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  Serial.print("WiFi Channel: ");
  Serial.println(primary);
  Serial.println("*** Set master ESP32 to this channel! ***");
  
  // Disable WiFi power saving for better ESP-NOW performance
  WiFi.setSleep(false);
  Serial.println("WiFi power saving disabled - better ESP-NOW reliability");
  
  // Print MAC address for ESP-NOW pairing
  Serial.print("ESP32-CAM MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("*** Use this MAC in the master ESP32 sketch! ***");

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  } else {
    Serial.println("ESP-NOW Initialized - Ready to receive");
    esp_now_register_recv_cb(OnDataRecv);
  }

  // Initialize WebSocket client
  webSocket.begin(websocket_server, websocket_port, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  
  Serial.printf("Connecting to server: %s:%d\n", websocket_server, websocket_port);
}

void loop() {
  webSocket.loop();
  
  // Send frames with delay to limit FPS and allow ESP-NOW processing
  // 40ms = ~25 FPS, good balance of speed and reliability
  if (webSocket.isConnected()) {
    sendCameraFrame();
    delay(40);  // ~25 FPS target
  }
  
  yield();
}
