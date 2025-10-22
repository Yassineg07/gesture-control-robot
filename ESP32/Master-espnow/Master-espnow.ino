/*
 * ESP32 Master - ESP-NOW Robot Controller
 * Sends motor commands to ESP32-CAM for STM32 control
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
// MPU
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

static constexpr float alpha = 0.98f; // complementary filter coefficient
static constexpr float accel_weight = 1.0f - alpha;
static constexpr float GYRO_LSB_PER_DPS = 131.0f; // for FS=250dps

float angle_roll_x = 0.0f;   // roll (deg)
float angle_pitch_y = 0.0f;  // pitch (deg)

unsigned long lastMicros = 0;

static void computeAccelAnglesDeg(int16_t ax, int16_t ay, int16_t az,
                                  float &rollDeg, float &pitchDeg) {

  const float fax = static_cast<float>(ax);
  const float fay = static_cast<float>(ay);
  const float faz = static_cast<float>(az);

  rollDeg = atan2f(fay, faz) * RAD_TO_DEG;
  pitchDeg = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * RAD_TO_DEG;
}

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

  // --- MPU6050 initialization ---
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    // keep running but notify
  } else {
    // prime angles from accelerometer
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    computeAccelAnglesDeg(ax, ay, az, angle_roll_x, angle_pitch_y);
  }
  lastMicros = micros();
}

// Use MPU6050 angles to control robot proportionally
void loop() {
  // timing
  const unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  // read sensors
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // accel angles
  float accel_roll_deg, accel_pitch_deg;
  computeAccelAnglesDeg(ax, ay, az, accel_roll_deg, accel_pitch_deg);

  // gyro rates (dps)
  const float gyro_rate_x_dps = static_cast<float>(gx) / GYRO_LSB_PER_DPS;
  const float gyro_rate_y_dps = static_cast<float>(gy) / GYRO_LSB_PER_DPS;

  // complementary filter
  angle_roll_x = alpha * (angle_roll_x + gyro_rate_x_dps * dt) + accel_weight * accel_roll_deg;
  angle_pitch_y = alpha * (angle_pitch_y + gyro_rate_y_dps * dt) + accel_weight * accel_pitch_deg;

  // Map angles to motor commands
  // Configuration
  const float deadzone_pitch = 5.0f; // degrees
  const float deadzone_roll = 5.0f;  // degrees
  const float max_pitch = 40.0f;     // degrees that map to full throttle
  const float max_roll = 40.0f;      // degrees that map to full steering

  float pitch = angle_pitch_y; // forward/backward
  float roll = angle_roll_x;   // left/right tilt

  // compute normalized magnitudes
  float pitch_mag = 0.0f;
  if (fabs(pitch) > deadzone_pitch) {
    pitch_mag = (fabs(pitch) - deadzone_pitch) / (max_pitch - deadzone_pitch);
    pitch_mag = constrain(pitch_mag, 0.0f, 1.0f);
  }

  float roll_norm = 0.0f;
  if (fabs(roll) > deadzone_roll) {
    roll_norm = (fabs(roll) - deadzone_roll) / (max_roll - deadzone_roll);
    roll_norm = constrain(roll_norm, 0.0f, 1.0f);
  }

  // base PWM from pitch magnitude
  uint8_t pwm_base = static_cast<uint8_t>(pitch_mag * 255.0f);

  // decide mode and compute left/right PWM
  if (pitch_mag > 0.0f) {
    // moving forward or reverse, blend steering using roll
    if (pitch > 0) {
      myData.mode = MODE_FORWARD;
    } else {
      myData.mode = MODE_REVERSE;
    }

    // turn differential: positive roll -> steer right (left motor faster)
    float turn = (roll >= 0.0f) ? roll_norm : -roll_norm; // -1..1
    // compute differential
    int diff = static_cast<int>(turn * pwm_base);
    int pr = static_cast<int>(pwm_base) - diff; // right motor
    int pl = static_cast<int>(pwm_base) + diff; // left motor
    myData.pwm_right = (uint8_t)constrain(pr, 0, 255);
    myData.pwm_left = (uint8_t)constrain(pl, 0, 255);

  } else if (roll_norm > 0.0f) {
    // rotation in place when not moving forward/back
    if (roll > 0) {
      myData.mode = MODE_RIGHT;
    } else {
      myData.mode = MODE_LEFT;
    }
    uint8_t pwm_turn = static_cast<uint8_t>(roll_norm * 255.0f);
    myData.pwm_right = pwm_turn;
    myData.pwm_left = pwm_turn;

  } else {
    // stop
    myData.mode = MODE_STOP;
    myData.pwm_right = 0;
    myData.pwm_left = 0;
  }

  // counter and send
  messageCounter++;
  myData.counter = messageCounter;

  esp_err_t result = esp_now_send(espCamAddress, (uint8_t *)&myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.printf("Sent #%d Mode=%d R=%d L=%d | Pitch=%.2f Roll=%.2f\n",
                  messageCounter, myData.mode, myData.pwm_right, myData.pwm_left,
                  angle_pitch_y, angle_roll_x);
  } else {
    Serial.println("ERROR: Send failed");
  }

  delay(50); // small delay, send ~20Hz
}
