/*
 * ESP32 Master - ESP-NOW Robot Controller
 * Sends motor commands to ESP32-CAM for STM32 control
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
// Lightweight MPU6050 implementation using Wire library
#include <Wire.h>

// I2C pins
#define I2C_SDA 21
#define I2C_SCL 22

// MPU6050 I2C address (AD0 low = 0x68, high = 0x69)
#define MPU_ADDR 0x68

// MPU6050 registers
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_GYRO_XOUT_H 0x43
#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_CONFIG 0x1A

// Sensitivity scales
const float ACCEL_SCALE = 16384.0;  // ±2g
const float GYRO_SCALE = 131.0;     // ±250 deg/s

bool imuAvailable = false;

// Motor control modes (declare early for use in variables)
enum MotorMode {
  MODE_FORWARD = 0,
  MODE_REVERSE = 1,
  MODE_RIGHT = 2,
  MODE_LEFT = 3,
  MODE_STOP = 4
};

static constexpr float alpha = 0.85f; // complementary filter coefficient (balanced)
static constexpr float accel_weight = 1.0f - alpha;

float angle_roll_x = 0.0f;   // roll (deg)
float angle_pitch_y = 0.0f;  // pitch (deg)

unsigned long lastMicros = 0;
unsigned long lastSendTime = 0;

// Previous command state to detect changes
uint8_t prev_mode = MODE_STOP;
uint8_t prev_pwm_right = 0;
uint8_t prev_pwm_left = 0;

// MPU6050 I2C helper functions
bool mpuWriteByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool mpuReadBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  
  uint8_t got = Wire.requestFrom((int)MPU_ADDR, (int)count);
  if (got < count) return false;
  
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
  return true;
}

bool mpuReadAll(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t buf[14];
  if (!mpuReadBytes(MPU_ACCEL_XOUT_H, 14, buf)) return false;
  
  // MPU6050 mounted normally - no inversion needed
  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  // buf[6-7] is temperature, skip
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
  
  return true;
}

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
  // No logging on send failure
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
  int32_t channel = 8;  // WARNING: CHANGE THIS to match ESP32-CAM's channel!
  
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
  Serial.println("\n===========================================");
  Serial.println("Initializing MPU6050...");
  Serial.printf("I2C: SDA=%d, SCL=%d\n", I2C_SDA, I2C_SCL);
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz
  delay(100);
  
  // Wake up MPU6050 (clear sleep bit)
  if (!mpuWriteByte(MPU_PWR_MGMT_1, 0x00)) {
    Serial.println("ERROR: Could not communicate with MPU6050");
    Serial.println("Check wiring:");
    Serial.printf("  MPU6050 SDA -> ESP32 GPIO %d\n", I2C_SDA);
    Serial.printf("  MPU6050 SCL -> ESP32 GPIO %d\n", I2C_SCL);
    Serial.println("  MPU6050 VCC -> ESP32 3.3V");
    Serial.println("  MPU6050 GND -> ESP32 GND");
    Serial.println("Robot will use test movement patterns.");
    imuAvailable = false;
  } else {
    Serial.println("MPU6050 communication OK!");
    // --- IMU SETTINGS: Set to match reference ---
    // Gyro: ±250 deg/s (0x00), Accel: ±2g (0x00), DLPF: 44Hz (0x03), I2C: 400kHz
    mpuWriteByte(MPU_GYRO_CONFIG, 0x00);    // ±250 deg/s
    mpuWriteByte(MPU_ACCEL_CONFIG, 0x00);   // ±2g
    mpuWriteByte(MPU_CONFIG, 0x03);         // DLPF ~44Hz
    delay(50);
    // Test read
    int16_t ax, ay, az, gx, gy, gz;
    if (mpuReadAll(ax, ay, az, gx, gy, gz)) {
      Serial.println("SUCCESS: MPU6050 initialized!");
      Serial.println("Configuration:");
      Serial.println("  Accelerometer: ±2g");
      Serial.println("  Gyroscope: ±250°/s");
      Serial.println("  Filter: 44 Hz");
      Serial.printf("Initial reading: ax=%d ay=%d az=%d\n", ax, ay, az);
      // Prime angles from accelerometer
      computeAccelAnglesDeg(ax, ay, az, angle_roll_x, angle_pitch_y);
      imuAvailable = true;
    } else {
      Serial.println("ERROR: Cannot read from MPU6050");
      imuAvailable = false;
    }
  }
  
  lastMicros = micros();
  Serial.println("Setup complete!");
}

// Test pattern function when IMU not available
void runTestPatterns() {
  // Test pattern function removed
  // (No-op)
}

// Use IMU angles to control robot proportionally
void loop() {
  // Only use IMU if available
  if (!imuAvailable) {
    return;
  }
  
  // Timing for sensor reading (~50Hz)
  const unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  // Read sensors
  int16_t ax, ay, az, gx, gy, gz;
  if (!mpuReadAll(ax, ay, az, gx, gy, gz)) {
    delay(20);
    return;
  }


  // Convert only the axes needed for calculations
  float ax_g = -ax / ACCEL_SCALE;
  float az_g = -az / ACCEL_SCALE;
  float gx_dps = gx / GYRO_SCALE;
  float gy_dps = -gy / GYRO_SCALE;

  // Calculate accel angles (deg) using the same formulas as imu-esp32.ino
  // ay is only used in accelPitch calculation
  float accelPitch = atan2f(-ay / ACCEL_SCALE, sqrtf(ax_g * ax_g + az_g * az_g)) * 180.0f / PI;
  float accelRoll = -atan2f(-ax_g, az_g) * 180.0f / PI;

  // Complementary filter (match variable names to imu-esp32.ino)
  angle_roll_x = alpha * (angle_roll_x + gx_dps * dt) + accel_weight * accelRoll;
  angle_pitch_y = alpha * (angle_pitch_y + gy_dps * dt) + accel_weight * accelPitch;

  // Map angles to motor commands
  const float deadzone_pitch = 10.0f; // degrees
  const float max_pitch = 30.0f;      // degrees
  const float deadzone_roll = 10.0f;  // degrees
  const float max_roll = 30.0f;       // degrees

  // --- IMU axis and offset correction to match imu-esp32.ino ---
  // angle_roll_x: filtered roll (deg), angle_pitch_y: filtered pitch (deg)
  // Reference: filteredRoll = -atan2(-ax_g, az_g) * 180/PI; filteredPitch = atan2(ay_g, sqrt(ax_g^2 + az_g^2)) * 180/PI;
  // Offsets: pitchOffset = 0.4, rollOffset = 5.4
  // Swapped axis mapping: roll = forward/backward, pitch = left/right
  const float pitchOffset = 4.8f; // Compensate for observed pitch drift
  const float rollOffset = -1.3f; // Compensate for observed roll drift
  float roll = angle_pitch_y + rollOffset; // Now roll is what pitch was
  float pitch = angle_roll_x + pitchOffset; // Now pitch is what roll was

  // Calculate normalized magnitudes
  float pitch_mag = 0.0f;
  if (fabs(pitch) > deadzone_pitch) {
    pitch_mag = (fabs(pitch) - deadzone_pitch) / (max_pitch - deadzone_pitch);
    pitch_mag = constrain(pitch_mag, 0.0f, 1.0f);
    pitch_mag = pitch_mag * pitch_mag;
  }

  float roll_norm = 0.0f;
  if (fabs(roll) > deadzone_roll) {
    roll_norm = (fabs(roll) - deadzone_roll) / (max_roll - deadzone_roll);
    roll_norm = constrain(roll_norm, 0.0f, 1.0f);
  }

  // --- PWM Quantization: 3 levels (150, 200, 255) ---
  // Map normalized PWM (0-255) to 200-255 range for more control
  auto quantize_pwm = [](int pwm) -> uint8_t {
    if (pwm <= 0) return 0;
    if (pwm >= 255) return 255;
    // Map 1-255 to 200-255 linearly
    return static_cast<uint8_t>(200 + ((pwm / 255.0f) * 55));
  };

  // Priority: if roll is small, use pitch; if pitch is small, use roll
  uint8_t pwm_base = static_cast<uint8_t>(pitch_mag * 255.0f);
  // Allow drift as long as pitch is not very close to zero (e.g., |pitch| >= 10 deg)
  if (fabs(pitch) >= 10.0f && pitch_mag > 0.0f) {
    // Forward/backward with tilt drift: adjust left/right PWM by roll
    if (pitch > 0) {
      myData.mode = MODE_FORWARD;
    } else {
      myData.mode = MODE_REVERSE;
    }
  // Drift: scale by both roll and base PWM, clamp so reduced side is always lower
  float drift_scale = 1.0f; // increased for quicker drift
  int drift_pwm = static_cast<int>(drift_scale * (roll / 30.0f) * pwm_base); // 30 deg = max drift
  int pwm_right = pwm_base - drift_pwm;
  int pwm_left  = pwm_base + drift_pwm;
  // Clamp so neither exceeds 255 and neither goes below 0
  pwm_right = constrain(pwm_right, 0, pwm_base);
  pwm_left  = constrain(pwm_left, 0, 255);
  myData.pwm_right = quantize_pwm(pwm_right);
  myData.pwm_left  = quantize_pwm(pwm_left);
  } else if (fabs(pitch) < 10.0f && roll_norm > 0.0f) {
    // Only switch to right/left if pitch is nearly zero
    if (roll > 0) {
      myData.mode = MODE_RIGHT;
    } else {
      myData.mode = MODE_LEFT;
    }
    uint8_t pwm_turn = quantize_pwm(static_cast<uint8_t>(roll_norm * 255.0f));
    myData.pwm_right = pwm_turn;
    myData.pwm_left = pwm_turn;
  } else {
    // Stop
    myData.mode = MODE_STOP;
    myData.pwm_right = 0;
    myData.pwm_left = 0;
  }

  // Send strategy: Send every ~30ms (33Hz) OR when command changes significantly
  const unsigned long SEND_INTERVAL_MS = 30; // 33Hz transmission rate
  bool commandChanged = (myData.mode != prev_mode) || 
                        (abs((int)myData.pwm_right - (int)prev_pwm_right) > 10) ||
                        (abs((int)myData.pwm_left - (int)prev_pwm_left) > 10);
  
  unsigned long currentTime = millis();
  
  if (commandChanged || (currentTime - lastSendTime >= SEND_INTERVAL_MS)) {
    lastSendTime = currentTime;
    
    // Update previous state
    prev_mode = myData.mode;
    prev_pwm_right = myData.pwm_right;
    prev_pwm_left = myData.pwm_left;
    
    // Send command
    messageCounter++;
    myData.counter = messageCounter;

    esp_err_t result = esp_now_send(espCamAddress, (uint8_t *)&myData, sizeof(myData));
    
    // Minimal logging to avoid Serial bottleneck
    if (messageCounter % 5 == 0) {  // Log every 5th message
      Serial.printf("#%d M=%d R=%d L=%d P=%.1f R=%.1f\n",
                    messageCounter, myData.mode, myData.pwm_right, myData.pwm_left,
                    pitch, roll);
    }
  }

  delay(20); // 50Hz loop
}
