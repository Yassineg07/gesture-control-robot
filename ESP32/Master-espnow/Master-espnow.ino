/*
 * ESP32 Master - ESP-NOW Robot Controller
 * Sends motor commands to ESP32-CAM for STM32 control
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ESP32-CAM MAC address (get from ESP32-CAM Serial Monitor on boot)
uint8_t espCamAddress[] = {0x08, 0xF9, 0xE0, 0xEC, 0xCE, 0x1C};

// Motor control modes
enum MotorMode {
  MODE_FORWARD = 0,
  MODE_REVERSE = 1,
  MODE_RIGHT = 2,
  MODE_LEFT = 3,
  MODE_STOP = 4
};

// Structure to send data - must match receiver
typedef struct struct_message {
  uint8_t mode;       // Motor mode (0-4)
  uint8_t pwm_right;  // Right motor PWM (0-255)
  uint8_t pwm_left;   // Left motor PWM (0-255)
  int counter;        // Message counter
} struct_message;

struct_message myData;
int messageCounter = 0;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // Make sure we're not connected to any network
  
  // Print MAC address
  Serial.println("ESP32 Master - ESP-NOW Sender");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  // ===== CRITICAL: SET WIFI CHANNEL TO MATCH ESP32-CAM =====
  // Check ESP32-CAM Serial Monitor on boot to see its WiFi channel
  // CHANGE THIS NUMBER to match!
  int32_t channel = 3;  // WARNING: CHANGE THIS to match ESP32-CAM's channel!
  
  Serial.println("===========================================");
  Serial.printf("WARNING: SETTING WiFi Channel to %d\n", channel);
  Serial.println("WARNING: This MUST match ESP32-CAM's WiFi channel!");
  Serial.println("WARNING: Check ESP32-CAM Serial Monitor on boot.");
  Serial.println("===========================================");
  
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  
  // Verify channel was set
  uint8_t primary;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  Serial.printf("WiFi Channel confirmed: %d\n", primary);
  // =========================================================
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW Initialized");
  
  // Register send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer (ESP32-CAM)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, espCamAddress, 6);
  peerInfo.channel = channel;  // Use same channel as WiFi
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add peer");
    return;
  }
  
  Serial.println("Peer added successfully");
  Serial.print("Target MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", espCamAddress[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  Serial.println("===========================================");
  Serial.println("Robot Controller - Testing Mode");
  Serial.println("Ready to send. Interval: 2 seconds");
  Serial.println("Ignore 'Fail' status - ESP32-CAM is busy");
  Serial.println("===========================================");
  
  randomSeed(analogRead(0));
}

// Test patterns - creative movement scenarios
void loop() {
  messageCounter++;
  
  // Cycle through different test scenarios
  int scenario = (messageCounter - 1) % 12; // 12 different test cases
  
  switch(scenario) {
    case 0:
      // Straight forward, full speed
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 255;
      myData.pwm_left = 255;
      Serial.println("Straight Forward - Full Speed");
      break;
      
    case 1:
      // Forward while turning right (right slower)
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 150;
      myData.pwm_left = 255;
      Serial.println("Forward + Turning Right (drift)");
      break;
      
    case 2:
      // Forward while turning left (left slower)
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 255;
      myData.pwm_left = 150;
      Serial.println("Forward + Turning Left (drift)");
      break;
      
    case 3:
      // Gentle forward curve
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 200;
      myData.pwm_left = 180;
      Serial.println("Gentle Forward Curve");
      break;
      
    case 4:
      // Reverse full speed
      myData.mode = MODE_REVERSE;
      myData.pwm_right = 255;
      myData.pwm_left = 255;
      Serial.println("Straight Reverse - Full Speed");
      break;
      
    case 5:
      // Reverse while turning
      myData.mode = MODE_REVERSE;
      myData.pwm_right = 200;
      myData.pwm_left = 120;
      Serial.println("Reverse + Turning");
      break;
      
    case 6:
      // Spin right (rotate in place)
      myData.mode = MODE_RIGHT;
      myData.pwm_right = 200;
      myData.pwm_left = 200;
      Serial.println("Spin Right (rotate)");
      break;
      
    case 7:
      // Slow spin right
      myData.mode = MODE_RIGHT;
      myData.pwm_right = 100;
      myData.pwm_left = 100;
      Serial.println("Slow Spin Right");
      break;
      
    case 8:
      // Spin left (rotate in place)
      myData.mode = MODE_LEFT;
      myData.pwm_right = 200;
      myData.pwm_left = 200;
      Serial.println("Spin Left (rotate)");
      break;
      
    case 9:
      // Fast spin left
      myData.mode = MODE_LEFT;
      myData.pwm_right = 255;
      myData.pwm_left = 255;
      Serial.println("Fast Spin Left");
      break;
      
    case 10:
      // Slow forward
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 80;
      myData.pwm_left = 80;
      Serial.println("Slow Forward (creep)");
      break;
      
    case 11:
      // Stop
      myData.mode = MODE_STOP;
      myData.pwm_right = 0;
      myData.pwm_left = 0;
      Serial.println("STOP");
      break;
  }
  
  myData.counter = messageCounter;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(espCamAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.printf("  Sent #%d: Mode=%d, R=%d, L=%d\n", 
                  messageCounter, myData.mode, myData.pwm_right, myData.pwm_left);
  } else {
    Serial.println("  ERROR: Error sending");
  }
  
  delay(2000); // 2 seconds between commands
}
