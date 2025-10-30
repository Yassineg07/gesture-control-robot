# Gesture-Controlled Robot with Live Video Streaming

Multi-microcontroller wireless robot system combining ESP-NOW communication, live video streaming, and IMU-based gesture control.

## System Architecture

```
Master ESP32 (MPU6050)  -->  ESP32-CAM  -->  STM32F4  -->  L298N  -->  DC Motors
    (Gesture Input)         (Video/Relay)   (Motor Ctrl)  (Driver)
                                 |
                                 v
                           Web Dashboard
```

## Hardware Requirements

- **Master ESP32** - With MPU6050 IMU
- **ESP32-CAM** - AI-Thinker with OV2640
- **STM32F4** - STM32F407/F411
- **L298N** - Motor driver
- **2x DC Motors** - 6-12V
- **4x LEDs** - Direction indicators
- **Battery Pack** - 7-12V for motors

## Quick Start

### 1. Configure WiFi (ESP32-CAM)
Edit `Slave-espnow.ino`:
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_PASSWORD";
const char* websocket_server = "192.168.1.XXX";  // Your PC IP
```

### 2. Set MAC Address (Master ESP32)
1. Upload code to ESP32-CAM
2. Copy MAC address from Serial Monitor
3. Update `Master-espnow.ino` (example):
```cpp
uint8_t espCamAddress[] = {0x08, 0xF9, 0xE0, 0xEC, 0xCE, 0x1C};
```

### 3. Match WiFi Channel
Both ESP32 devices must use same channel. Update in `Master-espnow.ino`:
```cpp
int32_t channel = 8;  // Match ESP32-CAM's channel
```


### 4. Start Server
```bash
cd websocket-server
npm install
npm start
```
Open browser: `http://localhost:8080`

## Wiring Guide

### ESP32-CAM to STM32F4
| ESP32-CAM | STM32F4 |
|-----------|---------|
| GPIO1 (TX) | PA3 (RX) |
| GND | GND |

### STM32F4 to L298N
| STM32F4 | L298N | Function |
|---------|-------|----------|
| PE8 | IN1 | Right forward |
| PE10 | IN2 | Right reverse |
| PE9 | ENA | Right PWM |
| PE12 | IN3 | Left forward |
| PE13 | IN4 | Left reverse |
| PE11 | ENB | Left PWM |

### STM32F4 LEDs
| Pin | LED | Direction |
|-----|-----|-----------|
| PD12 | RIGHT | Rotating right |
| PD13 | FWD | Moving forward |
| PD14 | LEFT | Rotating left |
| PD15 | REV | Moving reverse |

### Master ESP32 MPU6050
| MPU6050 | ESP32 |
|---------|-------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

## Motor Control Modes

| Mode | Value | Behavior |
|------|-------|----------|
| FORWARD | 0 | Both motors forward |
| REVERSE | 1 | Both motors reverse |
| RIGHT | 2 | Rotate right in place |
| LEFT | 3 | Rotate left in place |
| STOP | 4 | All motors off |

## UART Protocol

**Packet Format (5 bytes):**
```
[0xFF][MODE][PWM_RIGHT][PWM_LEFT][CHECKSUM]
```

**Checksum:** `MODE ^ PWM_RIGHT ^ PWM_LEFT`

**Example:**
```
FORWARD, R=255, L=200
[0xFF][0x00][0xFF][0xC8][0x37]
```

## STM32 Configuration

### TIM1 PWM
- **Prescaler:** 15
- **Period:** 255
- **Frequency:** ~26 kHz
- **Channels:** PE9 (CH1), PE11 (CH2)

### USART2
- **Baud:** 115200
- **Mode:** Interrupt-based
- **Pins:** PA2 (TX), PA3 (RX)

## Gesture Control

The MPU6050 IMU on the Master ESP32 detects tilt gestures:

- **Tilt Forward** → Robot moves forward
- **Tilt Backward** → Robot moves reverse
- **Tilt Left** → Robot turns left
- **Tilt Right** → Robot turns right
- **Level/Neutral** → Robot stops

Speed is proportional to tilt angle. Drift correction applies subtle turning while moving forward/backward.

## Performance

| Parameter | Value |
|-----------|-------|
| Video FPS | 18-22 |
| Video Resolution | 640x480 (VGA) |
| ESP-NOW Latency | <10ms |
| UART Baud | 115200 |
| PWM Frequency | 26 kHz |
| Control Update Rate | 33Hz (30ms) |

## Troubleshooting

**ESP-NOW drops packets:**
- Verify both ESP32s use same WiFi channel
- Check MAC address is correct
- Ensure explicit channel setting

**No video stream:**
- Verify PC and ESP32-CAM on same WiFi
- Check server IP in code matches your PC
- Confirm firewall allows port 8080

**Motors not responding:**
- Check L298N power (7-12V)
- Verify ENA/ENB jumpers removed
- Test UART with logic analyzer
- Check checksum validation

**Wrong motor direction:**
- Swap motor wires (OUT1↔OUT2 or OUT3↔OUT4)
- Or modify GPIO pin assignments in code

## Project Structure

```
├── Master-espnow.ino         # ESP32 gesture controller
├── Slave-espnow.ino          # ESP32-CAM receiver/streamer
├── main.c                    # STM32F4 motor controller
├── camera-server.js          # Node.js WebSocket server
└── README.md
```

## License

MIT [LICENSE](LICENSE) - Open source project for educational and commercial use.

## Authors

- [Yassineg07](mailto:gharbiyasine040@gmail.com) 
- [LansariFedi](mailto:lansarifedi7@gmail.com) 