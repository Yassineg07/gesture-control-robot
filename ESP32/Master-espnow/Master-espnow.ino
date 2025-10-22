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

// Callback when data is sent (updated for ESP32 Arduino Core v3.x)
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
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
  int32_t channel = 9;  // WARNING: CHANGE THIS to match ESP32-CAM's channel!
  
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

// Test patterns - randomized movement scenarios
void loop() {
  messageCounter++;
  
  // Cycle through different test scenarios (30 patterns with varied movements)
  int scenario = (messageCounter - 1) % 30;
  
  switch(scenario) {
    case 0:
      // Forward full speed
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 255;
      myData.pwm_left = 255;
      Serial.println("1. Forward - Full Speed");
      break;
      
    case 1:
      // Rotate left medium
      myData.mode = MODE_LEFT;
      myData.pwm_right = 180;
      myData.pwm_left = 180;
      Serial.println("2. Rotate Left - Medium");
      break;
      
    case 2:
      // Reverse drift right
      myData.mode = MODE_REVERSE;
      myData.pwm_right = 150;
      myData.pwm_left = 220;
      Serial.println("3. Reverse - Drift Right");
      break;
      
    case 3:
      // Forward sharp left turn
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 255;
      myData.pwm_left = 80;
      Serial.println("4. Forward - Sharp Left");
      break;
      
    case 4:
      // STOP
      myData.mode = MODE_STOP;
      myData.pwm_right = 0;
      myData.pwm_left = 0;
      Serial.println("5. STOP");
      break;
      
    case 5:
      // Rotate right fast
      myData.mode = MODE_RIGHT;
      myData.pwm_right = 255;
      myData.pwm_left = 255;
      Serial.println("6. Rotate Right - Fast");
      break;
      
    case 6:
      // Forward medium speed
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 180;
      myData.pwm_left = 180;
      Serial.println("7. Forward - Medium");
      break;
      
    case 7:
      // Reverse full speed
      myData.mode = MODE_REVERSE;
      myData.pwm_right = 255;
      myData.pwm_left = 255;
      Serial.println("8. Reverse - Full Speed");
      break;
      
    case 8:
      // Forward gentle right curve
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 180;
      myData.pwm_left = 220;
      Serial.println("9. Forward - Gentle Right");
      break;
      
    case 9:
      // Rotate left slow
      myData.mode = MODE_LEFT;
      myData.pwm_right = 100;
      myData.pwm_left = 100;
      Serial.println("10. Rotate Left - Slow");
      break;
      
    case 10:
      // STOP
      myData.mode = MODE_STOP;
      myData.pwm_right = 0;
      myData.pwm_left = 0;
      Serial.println("11. STOP");
      break;
      
    case 11:
      // Reverse drift left
      myData.mode = MODE_REVERSE;
      myData.pwm_right = 220;
      myData.pwm_left = 150;
      Serial.println("12. Reverse - Drift Left");
      break;
      
    case 12:
      // Forward sharp right turn
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 80;
      myData.pwm_left = 255;
      Serial.println("13. Forward - Sharp Right");
      break;
      
    case 13:
      // Rotate right slow
      myData.mode = MODE_RIGHT;
      myData.pwm_right = 100;
      myData.pwm_left = 100;
      Serial.println("14. Rotate Right - Slow");
      break;
      
    case 14:
      // Forward slow creep
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 100;
      myData.pwm_left = 100;
      Serial.println("15. Forward - Slow Creep");
      break;
      
    case 15:
      // Reverse medium speed
      myData.mode = MODE_REVERSE;
      myData.pwm_right = 180;
      myData.pwm_left = 180;
      Serial.println("16. Reverse - Medium");
      break;
      
    case 16:
      // STOP
      myData.mode = MODE_STOP;
      myData.pwm_right = 0;
      myData.pwm_left = 0;
      Serial.println("17. STOP");
      break;
      
    case 17:
      // Forward drift left
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 255;
      myData.pwm_left = 150;
      Serial.println("18. Forward - Drift Left");
      break;
      
    case 18:
      // Rotate left fast
      myData.mode = MODE_LEFT;
      myData.pwm_right = 255;
      myData.pwm_left = 255;
      Serial.println("19. Rotate Left - Fast");
      break;
      
    case 19:
      // Forward gentle left curve
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 220;
      myData.pwm_left = 180;
      Serial.println("20. Forward - Gentle Left");
      break;
      
    case 20:
      // Reverse slow
      myData.mode = MODE_REVERSE;
      myData.pwm_right = 120;
      myData.pwm_left = 120;
      Serial.println("21. Reverse - Slow");
      break;
      
    case 21:
      // Rotate right medium
      myData.mode = MODE_RIGHT;
      myData.pwm_right = 180;
      myData.pwm_left = 180;
      Serial.println("22. Rotate Right - Medium");
      break;
      
    case 22:
      // STOP
      myData.mode = MODE_STOP;
      myData.pwm_right = 0;
      myData.pwm_left = 0;
      Serial.println("23. STOP");
      break;
      
    case 23:
      // Forward drift right
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 150;
      myData.pwm_left = 255;
      Serial.println("24. Forward - Drift Right");
      break;
      
    case 24:
      // Reverse sharp turn
      myData.mode = MODE_REVERSE;
      myData.pwm_right = 100;
      myData.pwm_left = 220;
      Serial.println("25. Reverse - Sharp Turn");
      break;
      
    case 25:
      // Forward varied speed
      myData.mode = MODE_FORWARD;
      myData.pwm_right = 200;
      myData.pwm_left = 160;
      Serial.println("26. Forward - Varied Speed");
      break;
      
    case 26:
      // Rotate left varied
      myData.mode = MODE_LEFT;
      myData.pwm_right = 150;
      myData.pwm_left = 150;
      Serial.println("27. Rotate Left - Varied");
      break;
      
    case 27:
      // Reverse gentle curve
      myData.mode = MODE_REVERSE;
      myData.pwm_right = 200;
      myData.pwm_left = 170;
      Serial.println("28. Reverse - Gentle Curve");
      break;
      
    case 28:
      // STOP
      myData.mode = MODE_STOP;
      myData.pwm_right = 0;
      myData.pwm_left = 0;
      Serial.println("29. STOP");
      break;
      
    case 29:
      // Rotate right varied
      myData.mode = MODE_RIGHT;
      myData.pwm_right = 220;
      myData.pwm_left = 220;
      Serial.println("30. Rotate Right - Varied");
      break;
  }
  
  myData.counter = messageCounter;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(espCamAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.printf("  ✓ Sent #%d: Mode=%d, R=%d, L=%d\n", 
                  messageCounter, myData.mode, myData.pwm_right, myData.pwm_left);
  } else {
    Serial.println("  ✗ ERROR: Send failed");
  }
  
  delay(1000); // 1 second between commands
}
