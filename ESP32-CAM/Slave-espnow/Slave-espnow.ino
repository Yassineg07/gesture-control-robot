#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <esp_wifi.h>

enum MotorMode {
  MODE_FORWARD = 0,
  MODE_REVERSE = 1,
  MODE_RIGHT = 2,
  MODE_LEFT = 3,
  MODE_STOP = 4
};

typedef struct struct_message {
  uint8_t mode;
  uint8_t pwm_right;
  uint8_t pwm_left;
  int counter;
} struct_message;

struct_message incomingData;

int lastReceivedCounter = 0;
int totalReceived = 0;
int totalDropped = 0;

#define STM32_SERIAL Serial
#define UART_BAUD 115200

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

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

WebSocketsClient webSocket;

const char* websocket_server = "192.168.1.100";
const int websocket_port = 8080;

unsigned long lastStatsTime = 0;
unsigned long framesSent = 0;
float currentFPS = 0;

uint32_t totalSRAM = 0;
uint32_t totalPSRAM = 0;

void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(100);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    return;
  }
  
  Serial.println("\nWiFi connection failed");
  delay(10000);
  ESP.restart();
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket Connected");
      sendSystemStats();
      break;
    case WStype_TEXT:
      {
        DynamicJsonDocument doc(512);
        DeserializationError error = deserializeJson(doc, payload);
        
        if (error || !doc.containsKey("cmd")) return;
        
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
    Serial.println("ERROR: Camera sensor not found");
    return;
  }
  
  if (doc.containsKey("brightness")) s->set_brightness(s, doc["brightness"]);
  if (doc.containsKey("contrast")) s->set_contrast(s, doc["contrast"]);
  if (doc.containsKey("saturation")) s->set_saturation(s, doc["saturation"]);
  if (doc.containsKey("ae_level")) s->set_ae_level(s, doc["ae_level"]);
  if (doc.containsKey("gainceiling")) s->set_gainceiling(s, (gainceiling_t)doc["gainceiling"].as<int>());
  if (doc.containsKey("quality")) s->set_quality(s, doc["quality"]);
  if (doc.containsKey("aec")) s->set_exposure_ctrl(s, doc["aec"]);
  if (doc.containsKey("aec_value")) s->set_aec_value(s, doc["aec_value"]);
  if (doc.containsKey("agc")) s->set_gain_ctrl(s, doc["agc"]);
  if (doc.containsKey("agc_gain")) s->set_agc_gain(s, doc["agc_gain"]);
  if (doc.containsKey("awb")) s->set_whitebal(s, doc["awb"]);
  
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
  uint32_t freeSRAM = ESP.getFreeHeap();
  uint32_t freePSRAM = ESP.getFreePsram();
  
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

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  
  totalReceived++;
  if (lastReceivedCounter > 0) {
    int expected = lastReceivedCounter + 1;
    if (incomingData.counter != expected) {
      int dropped = incomingData.counter - expected;
      totalDropped += dropped;
      Serial.printf("DROPPED %d messages\n", dropped);
    }
  }
  lastReceivedCounter = incomingData.counter;
  
  sendToSTM32(incomingData.mode, incomingData.pwm_right, incomingData.pwm_left);
}

void sendToSTM32(uint8_t mode, uint8_t pwm_right, uint8_t pwm_left) {
  uint8_t checksum = mode ^ pwm_right ^ pwm_left;
  
  STM32_SERIAL.write(0xFF);
  STM32_SERIAL.write(mode);
  STM32_SERIAL.write(pwm_right);
  STM32_SERIAL.write(pwm_left);
  STM32_SERIAL.write(checksum);
  STM32_SERIAL.flush();
}

void sendCameraFrame() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR: Frame capture failed");
    return;
  }
  
  if (fb->len == 0) {
    Serial.println("ERROR: Frame length 0");
    esp_camera_fb_return(fb);
    return;
  }
  
  if (webSocket.isConnected()) {
    bool sent = webSocket.sendBIN(fb->buf, fb->len);
    if (sent) {
      framesSent++;
    }
    
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
  
  Serial.println("ESP32-CAM Robot Controller");
  
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
  config.xclk_freq_hz = 18000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 8;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, -1);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_ae_level(s, 1);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, (gainceiling_t)2);
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);
  
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  
  totalSRAM = ESP.getHeapSize();
  totalPSRAM = ESP.getPsramSize();
  
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);

  connectToWiFi();
  
  uint8_t primary;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  Serial.printf("WiFi Channel: %d\n", primary);
  
  WiFi.setSleep(false);
  
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
  } else {
    Serial.println("ESP-NOW Initialized");
    esp_now_register_recv_cb(OnDataRecv);
  }

  webSocket.begin(websocket_server, websocket_port, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  
  Serial.printf("Connecting to server: %s:%d\n", websocket_server, websocket_port);
}

void loop() {
  webSocket.loop();
  
  if (webSocket.isConnected()) {
    sendCameraFrame();
    delay(40);
  }
  
  yield();
}