# Gesture-Controlled Robot with Live Video Streaming

[![Demo Video](https://img.youtube.com/vi/Xi25Kjre0I0/maxresdefault.jpg)](https://youtu.be/Xi25Kjre0I0)

Build a wireless robot that streams live video while receiving motion commands over ESP-NOW. A Master ESP32 sends mode + PWM values to an ESP32-CAM, which relays them via UART to an STM32F4 that drives motors through an L298N. It’s a compact, real-world example of multi‑MCU integration with camera streaming, WebSockets, DMA UART, and hardware PWM.

Estimated build time: 3–4 hours (first time), 1–2 hours (experienced)

---

## What’s inside (at a glance)

Architecture: Master ESP32 → ESP32‑CAM (video + relay) → STM32F4 (motor control) → L298N → Motors

Protocols and I/O:

- ESP‑NOW (ESP32 → ESP32‑CAM), <10 ms latency
- WebSocket (ESP32‑CAM → browser), JPEG stream on port 8080
- UART (ESP32‑CAM → STM32F4) at 115200 baud with checksum
- TIM1 PWM at ~26 kHz, independent L/R motor speed

Key features:

- Live VGA video (~18–22 FPS)
- 12 test motion patterns (forward/reverse/turns/spins)
- Direction LEDs (FWD/REV/LEFT/RIGHT)
- Server‑side image tweaks (Sharp)

---

## Hardware

- ESP32 dev board (Master)
- ESP32‑CAM (AI‑Thinker with OV2640)
- STM32F4 (e.g., F407/F411)
- L298N motor driver + 2 DC motors
- Battery 7–12 V for motors (logic power separately)
- 4 LEDs + 220–1kΩ resistors, jumper wires, USB cables

Essentials:

- Common ground between all devices
- Remove ENA/ENB jumpers on L298N (PWM control from STM32)

---

## Wiring (essentials only)

- ESP32‑CAM TX (GPIO1/U0T) → STM32 PA3 (USART2_RX)
- STM32 TIM1 PWM: PE9 → L298N ENA, PE11 → L298N ENB
- STM32 direction: PE8→IN1, PE10→IN2, PE12→IN3, PE13→IN4
- LEDs: PD12 RIGHT, PD13 FWD, PD14 LEFT, PD15 REV
- Power: L298N +12V to battery, GND shared with STM32/ESP32‑CAM

Programming tip: Disconnect ESP32‑CAM UART during flashing (GPIO1/3 used by USB).

---

## Software setup

Arduino IDE:

- Boards: ESP32 (Espressif URL), STM32 (STM32Duino URL)
- Libraries: ArduinoJson, WebSocketsClient, AsyncTCP, ESPAsyncWebServer

Node.js server:

```bash
cd websocket-server
npm install
npm start
```

Open http://localhost:8080

Configure ESP32‑CAM WiFi in `ESP32-CAM/Slave-espnow/Slave-espnow.ino`:

```cpp
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* websocket_server = "192.168.1.XXX"; // your PC’s IP
```

Set MAC and channel for Master ESP32:

1. Get ESP32‑CAM MAC from Serial Monitor.
2. Paste into `ESP32/Master-espnow/Master-espnow.ino`:

```cpp
uint8_t espCamAddress[] = {0x08,0xF9,0xE0,0xEC,0xCE,0x1C};
```

3. Ensure the same WiFi channel (example):

```cpp
esp_wifi_set_channel(3, WIFI_SECOND_CHAN_NONE);
```

STM32 (via CubeMX/IDE): TIM1 CH1/CH2 PWM (PE9/PE11), USART2 RX DMA (PA3), GPIO outs for PE8/10/12/13 and PD12–15.

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

## Project structure

```
ESP32/                 Master-espnow.ino
ESP32-CAM/             Slave-espnow.ino
STM32F4/               CubeMX project (TIM1 PWM, USART2 RX DMA)
websocket-server/      camera-server.js, package.json
```

License: Educational/hobby use. Contributions welcome via PRs.
