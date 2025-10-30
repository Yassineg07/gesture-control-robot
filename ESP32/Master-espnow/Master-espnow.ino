#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_GYRO_XOUT_H 0x43
#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_CONFIG 0x1A

enum MotorMode {
  MODE_FORWARD = 0,
  MODE_REVERSE = 1,
  MODE_RIGHT = 2,
  MODE_LEFT = 3,
  MODE_STOP = 4
};

const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE = 131.0;
bool imuAvailable = false;


static constexpr float alpha = 0.85f;
static constexpr float accel_weight = 1.0f - alpha;

float angle_roll_x = 0.0f;
float angle_pitch_y = 0.0f;
unsigned long lastMicros = 0;
unsigned long lastSendTime = 0;

uint8_t prev_mode = MODE_STOP;
uint8_t prev_pwm_right = 0;
uint8_t prev_pwm_left = 0;

uint8_t espCamAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

typedef struct struct_message {
  uint8_t mode;
  uint8_t pwm_right;
  uint8_t pwm_left;
  int counter;
} struct_message;

struct_message myData;
int messageCounter = 0;

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
  
  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
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

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  Serial.println("ESP32 Master");
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  
  int32_t channel = 8;
  
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  
  uint8_t primary;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  Serial.printf("Channel: %d\n", primary);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return;
  }
  Serial.println("ESP-NOW Initialized");
  
  esp_now_register_send_cb(OnDataSent);
  
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, espCamAddress, 6);
  peerInfo.channel = channel;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add peer");
    return;
  }
  
  Serial.println("Peer added");
  randomSeed(analogRead(0));

  Serial.println("Initializing MPU6050...");
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(100);
  
  if (!mpuWriteByte(MPU_PWR_MGMT_1, 0x00)) {
    Serial.println("ERROR: MPU6050 not found");
    imuAvailable = false;
  } else {
    mpuWriteByte(MPU_GYRO_CONFIG, 0x00);
    mpuWriteByte(MPU_ACCEL_CONFIG, 0x00);
    mpuWriteByte(MPU_CONFIG, 0x03);
    delay(50);
    
    int16_t ax, ay, az, gx, gy, gz;
    if (mpuReadAll(ax, ay, az, gx, gy, gz)) {
      Serial.println("MPU6050 OK");
      computeAccelAnglesDeg(ax, ay, az, angle_roll_x, angle_pitch_y);
      imuAvailable = true;
    } else {
      Serial.println("ERROR: MPU6050 read failed");
      imuAvailable = false;
    }
  }
  
  lastMicros = micros();
  Serial.println("Setup complete");
}

void loop() {
  if (!imuAvailable) {
    return;
  }
  
  const unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  int16_t ax, ay, az, gx, gy, gz;
  if (!mpuReadAll(ax, ay, az, gx, gy, gz)) {
    delay(20);
    return;
  }

  float ax_g = -ax / ACCEL_SCALE;
  float az_g = -az / ACCEL_SCALE;
  float gx_dps = gx / GYRO_SCALE;
  float gy_dps = -gy / GYRO_SCALE;

  float accelPitch = atan2f(-ay / ACCEL_SCALE, sqrtf(ax_g * ax_g + az_g * az_g)) * 180.0f / PI;
  float accelRoll = -atan2f(-ax_g, az_g) * 180.0f / PI;

  angle_roll_x = alpha * (angle_roll_x + gx_dps * dt) + accel_weight * accelRoll;
  angle_pitch_y = alpha * (angle_pitch_y + gy_dps * dt) + accel_weight * accelPitch;

  const float deadzone_pitch_min = -35.0f;
  const float deadzone_pitch_max = 25.0f;
  const float max_pitch = 45.0f;
  const float deadzone_roll = 15.0f;
  const float max_roll = 45.0f;

  const float pitchOffset = 4.8f;
  const float rollOffset = -1.3f;
  float roll = angle_pitch_y + rollOffset;
  float pitch = angle_roll_x + pitchOffset;

  float pitch_mag = 0.0f;
  if (pitch < deadzone_pitch_min) {
    pitch_mag = (deadzone_pitch_min - pitch) / (max_pitch - deadzone_pitch_min);
    pitch_mag = constrain(pitch_mag, 0.0f, 1.0f);
    pitch_mag = pitch_mag * pitch_mag;
  } else if (pitch > deadzone_pitch_max) {
    pitch_mag = (pitch - deadzone_pitch_max) / (max_pitch - deadzone_pitch_max);
    pitch_mag = constrain(pitch_mag, 0.0f, 1.0f);
    pitch_mag = pitch_mag * pitch_mag;
  } else {
    pitch_mag = 0.0f;
  }

  float roll_norm = 0.0f;
  if (fabs(roll) > deadzone_roll) {
    roll_norm = (fabs(roll) - deadzone_roll) / (max_roll - deadzone_roll);
    roll_norm = constrain(roll_norm, 0.0f, 1.0f);
    roll_norm = roll_norm * roll_norm;
  }

  auto quantize_pwm = [](int pwm) -> uint8_t {
    if (pwm <= 0) return 0;
    if (pwm >= 255) return 255;
    return static_cast<uint8_t>(200 + ((pwm / 255.0f) * 55));
  };

  uint8_t pwm_base = static_cast<uint8_t>(pitch_mag * 255.0f);
  
  if (fabs(pitch) >= 10.0f && pitch_mag > 0.0f) {
    if (pitch > 0) {
      myData.mode = MODE_FORWARD;
    } else {
      myData.mode = MODE_REVERSE;
    }
    
    float drift_scale = 1.7f;
    int drift_pwm = static_cast<int>(drift_scale * (roll / 45.0f) * pwm_base);
    int pwm_right = pwm_base - drift_pwm;
    int pwm_left  = pwm_base + drift_pwm;
    pwm_right = constrain(pwm_right, 0, pwm_base);
    pwm_left  = constrain(pwm_left, 0, 255);
    myData.pwm_right = quantize_pwm(pwm_right);
    myData.pwm_left  = quantize_pwm(pwm_left);
  } else if (fabs(pitch) < 25.0f && roll_norm > 0.0f) {
    if (roll > 0) {
      myData.mode = MODE_RIGHT;
    } else {
      myData.mode = MODE_LEFT;
    }
    uint8_t pwm_turn = quantize_pwm(static_cast<uint8_t>(roll_norm * 255.0f));
    myData.pwm_right = pwm_turn;
    myData.pwm_left = pwm_turn;
  } else {
    myData.mode = MODE_STOP;
    myData.pwm_right = 0;
    myData.pwm_left = 0;
  }

  const unsigned long SEND_INTERVAL_MS = 30;
  bool commandChanged = (myData.mode != prev_mode) || 
                        (abs((int)myData.pwm_right - (int)prev_pwm_right) > 10) ||
                        (abs((int)myData.pwm_left - (int)prev_pwm_left) > 10);
  
  unsigned long currentTime = millis();
  
  if (commandChanged || (currentTime - lastSendTime >= SEND_INTERVAL_MS)) {
    lastSendTime = currentTime;
    
    prev_mode = myData.mode;
    prev_pwm_right = myData.pwm_right;
    prev_pwm_left = myData.pwm_left;
    
    messageCounter++;
    myData.counter = messageCounter;

    esp_err_t result = esp_now_send(espCamAddress, (uint8_t *)&myData, sizeof(myData));
    
    if (messageCounter % 5 == 0) {
      Serial.printf("#%d M=%d R=%d L=%d P=%.1f R=%.1f\n",
                    messageCounter, myData.mode, myData.pwm_right, myData.pwm_left,
                    pitch, roll);
    }
  }

  delay(20);
}