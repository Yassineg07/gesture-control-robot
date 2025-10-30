# Gesture-Controlled Robot with Live Video Streaming

[![Demo Video](https://img.youtube.com/vi/Xi25Kjre0I0/maxresdefault.jpg)](https://youtu.be/Xi25Kjre0I0)

Build a wireless robot that streams live video while receiving motion commands over ESP-NOW. A Master ESP32 sends mode + PWM values to an ESP32-CAM, which relays them via UART to an STM32F4 that drives motors through an L298N. It’s a compact, real-world example of multi‑MCU integration with camera streaming, WebSockets, DMA UART, and hardware PWM.

---

## What’s inside (at a glance)

Architecture: 

```
Master ESP32 (MPU6050)  -->  ESP32-CAM  -->  STM32F4  -->  L298N  -->  DC Motors
    (Gesture Input)         (Video/Relay)   (Motor Ctrl)  (Driver)
                                 |
                                 v
                           Web Dashboard
```

## Gesture Control

The MPU6050 IMU on the Master ESP32 detects tilt gestures:

- **Tilt Forward** → Robot moves forward
- **Tilt Backward** → Robot moves reverse
- **Tilt Left** → Robot turns left
- **Tilt Right** → Robot turns right
- **Level/Neutral** → Robot stops

Speed is proportional to tilt angle. Drift correction applies subtle turning while moving forward/backward.

---

## Protocols and I/O:

- ESP‑NOW (ESP32 → ESP32‑CAM), <10 ms latency
- WebSocket (ESP32‑CAM → browser), JPEG stream on port 8080
- UART (ESP32‑CAM → STM32F4) at 115200 baud with checksum
- TIM1 PWM at ~26 kHz, independent L/R motor speed

## Key features:

- Live VGA video (~18–22 FPS)
- 12 test motion patterns (forward/reverse/turns/spins)
- Direction LEDs (FWD/REV/LEFT/RIGHT)
- Server‑side image tweaks (Sharp)

---

## Hardware

- ESP32 dev board (With MPU6050 IMU)
- ESP32‑CAM (AI‑Thinker with OV2640)
- STM32F4 (e.g., F407/F411)
- L298N motor driver + 4 DC motors
- Battery 7–12 V for motors (logic power separately)
- Jumper wires, USB cables

---

## Wiring (essentials only)

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

### Power: 
L298N +12V to battery, GND shared with STM32/ESP32‑CAM

### STM32F4 LEDs (on-board LEDs)
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

Programming tip:
- Common ground between all devices
- Remove ENA/ENB jumpers on L298N (PWM control from STM32)
- Disconnect ESP32‑CAM UART during flashing (GPIO1/3 used by USB).

---

## Software setup

If using Arduino IDE:

- Boards: ESP32 (Espressif URL), STM32 (STM32Duino URL)
- Libraries: ArduinoJson, WebSocketsClient, AsyncTCP, ESPAsyncWebServer

### 1. Configure WiFi (ESP32-CAM)
Configure ESP32‑CAM WiFi in `ESP32-CAM/Slave-espnow/Slave-espnow.ino`:
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_PASSWORD";
const char* websocket_server = "192.168.1.XXX";  // Your PC IP
```

### 2. Set MAC Address (Master ESP32)
1. Upload code to ESP32-CAM
2. Copy MAC address from Serial Monitor
3. Paste into `ESP32/Master-espnow/Master-espnow.ino` (example):
```cpp
uint8_t espCamAddress[] = {0x08, 0xF9, 0xE0, 0xEC, 0xCE, 0x1C};
```

### 3. Match WiFi Channel
Both ESP32 devices must use same channel. Update in `Master-espnow.ino`:
```cpp
int32_t channel = 8;  // Match ESP32-CAM's channel
```


### 4. Start Server
> **Windows users:** Double-click `start_dashboard.bat` for easy setup!

*or*
```bash
cd websocket-server
npm install
npm start
```
Open browser: `http://localhost:8080`

---

## Upload order

1. Master ESP32 → `ESP32/Master-espnow/Master-espnow.ino` (note channel)
2. ESP32‑CAM → `ESP32-CAM/Slave-espnow/Slave-espnow.ino` (set WiFi + server IP)
3. STM32F4 → CubeIDE project (TIM1 PWM, USART2 RX DMA)

Then wire the boards and power up.

---

## Quick tests

1. ESP‑NOW: Serial monitors show sent/received packets, 0% drops.
2. UART/LEDs: With motors disconnected, LEDs change by mode every ~2 s.
3. Motors: Place robot on blocks; run 12 patterns (independent L/R PWM).
4. Web UI: Start server and open http://localhost:8080 to see live video and stats.

---

## UART protocol (5 bytes)

```
[0xFF] [MODE] [PWM_R] [PWM_L] [CHECKSUM]
checksum = MODE ^ PWM_R ^ PWM_L
```

MODE: 0=FWD, 1=REV, 2=RIGHT, 3=LEFT, 4=STOP

---

## Performance (typical)

- Video: 640×480 @ ~18–22 FPS (jpeg_quality=8, delay(40))
- ESP‑NOW latency: <10 ms; reliability: ~100% with channel match
- UART: 115200 baud; DMA on STM32
- PWM: ~26 kHz, 8‑bit (0–255)

---

## Troubleshooting (fast fixes)

- No ESP‑NOW packets: Confirm MAC, same channel, and WiFi connected on ESP32‑CAM.
- No video: Verify PC IP in ESP32‑CAM code; ensure Node server running and firewall allows :8080.
- Motors idle/wrong dir: Remove ENA/ENB jumpers; check IN1–IN4 pins; swap motor leads if reversed.


---
## License
MIT [LICENSE](LICENSE) - Open source project for educational and commercial use.

---

## Credits
- Gharbi Yassine <gharbiyasine040@gmail.com>
- Lansari Fedi <lansarifedi7@gmail.com>

---
Feel free to contact us.
