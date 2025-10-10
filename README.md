# 🤖 ESP32-CAM Robot Control System

A complete wireless robot control system featuring live video streaming, ESP-NOW wireless control, and multi-microcontroller motor management.

## Features

✅ **Live Video Streaming** - ESP32-CAM streams 640x480 video at ~18-22 FPS  
✅ **Wireless Control** - ESP-NOW provides reliable low-latency command transmission  
✅ **Web Interface** - Real-time video feed with motor status monitoring  
✅ **Multi-Controller** - Master ESP32 → ESP32-CAM → STM32F4 → L298N motors  
✅ **Direction LEDs** - Visual feedback showing current movement direction  
✅ **Image Processing** - Server-side enhancement with blur, brightness, and saturation adjustments

---

## System Architecture

```
┌─────────────────┐         ┌──────────────────┐         ┌──────────────┐
│  Master ESP32   │         │   ESP32-CAM      │         │  STM32F4     │
│                 │         │                  │         │              │
│  ESP-NOW TX ────┼────────→│  ESP-NOW RX      │         │              │
│  (Wireless)     │         │                  │         │              │
│                 │  WiFi   │  WebSocket ──────┼────────→│  Browser UI  │
│  Test Patterns  │         │  Video Stream    │         │  (Live View) │
│                 │         │                  │         │              │
│                 │         │  UART TX (GPIO1) ├────────→│  RX (PA3)    │
│                 │         │                  │         │  DMA Buffer  │
│                 │         │                  │         │              │
│                 │         │                  │         │  TIM1 PWM    │
│                 │         │                  │         │  GPIO Dir    │
└─────────────────┘         └──────────────────┘         └──────┬───────┘
                                                                 │
                                                                 ↓
                                                         ┌──────────────┐
                                                         │  L298N       │
                                                         │  Motor Driver│
                                                         │              │
                                                         │  OUT1/2 ─────┼──→ Right Motor
                                                         │  OUT3/4 ─────┼──→ Left Motor
                                                         └──────────────┘
```

**Data Flow:**
1. Master ESP32 generates movement commands (12 test patterns)
2. ESP-NOW transmits wirelessly to ESP32-CAM (channel 3, 100% reliability)
3. ESP32-CAM receives, displays status, forwards via UART to STM32
4. STM32F4 receives via DMA, validates checksum, controls motors
5. Direction LEDs automatically indicate movement direction
6. Web browser displays live video + motor status

---

## Hardware Requirements

### Components
- **1x Master ESP32** - Any ESP32 development board
- **1x ESP32-CAM** - AI-Thinker module with OV2640 camera
- **1x STM32F4 Board** - STM32F407/STM32F411 (with PE/PD GPIO)
- **1x L298N Motor Driver** - Dual H-bridge for motor control
- **2x DC Motors** - 6-12V rated (for left and right wheels)
- **1x Battery Pack** - 7-12V for motors (separate from logic power)
- **4x LEDs** - Direction indicators (Forward, Reverse, Left, Right)
- **Resistors** - 220Ω-1kΩ for LEDs
- **Jumper Wires** - For connections
- **USB Cables** - For programming and power

---

## Wiring Guide

### 1. Master ESP32 (Sender)

**Power:** USB cable or 5V power supply  
**No wiring needed** - Communicates wirelessly via ESP-NOW

### 2. ESP32-CAM (Receiver + Video Streamer)

#### Power
| ESP32-CAM | Power Source |
|-----------|--------------|
| 5V | 5V supply (2A recommended) |
| GND | Ground |

#### UART to STM32F4
| ESP32-CAM Pin | STM32F4 Pin | Description |
|---------------|-------------|-------------|
| GPIO1 (TX) | PA3 (RX) | UART transmit |
| GND | GND | Common ground |

⚠️ **IMPORTANT:** Disconnect UART during ESP32-CAM programming! GPIO1/GPIO3 are used by USB.

### 3. STM32F4 (Motor Controller)

#### UART from ESP32-CAM
| STM32F4 Pin | ESP32-CAM Pin | Description |
|-------------|---------------|-------------|
| PA3 (USART2_RX) | GPIO1 (TX) | Receive commands |
| GND | GND | Common ground |

#### L298N Motor Driver
| STM32F4 Pin | L298N Pin | Function |
|-------------|-----------|----------|
| PE8 | IN1 | Right motor forward |
| PE10 | IN2 | Right motor reverse |
| PE9 (TIM1_CH1) | ENA | Right motor PWM speed |
| PE12 | IN3 | Left motor forward |
| PE13 | IN4 | Left motor reverse |
| PE11 (TIM1_CH2) | ENB | Left motor PWM speed |
| GND | GND | Common ground |
| 5V | +5V (logic) | Logic power |

#### Direction LEDs
| STM32F4 Pin | LED | When Active |
|-------------|-----|-------------|
| PD12 | RIGHT | Rotating right |
| PD13 | FORWARD | Moving forward |
| PD14 | LEFT | Rotating left |
| PD15 | REVERSE | Moving reverse |

*Connect LEDs with 220Ω-1kΩ resistors to ground*

### 4. L298N Motor Driver

#### Power
| L298N Pin | Connection |
|-----------|------------|
| +12V | Battery positive (7-12V) |
| GND | Battery negative + STM32 GND |
| +5V | To STM32 5V (if using onboard regulator) |

#### Motors
| L298N Pin | Motor |
|-----------|-------|
| OUT1, OUT2 | Right motor |
| OUT3, OUT4 | Left motor |

⚠️ **Remove ENA/ENB jumpers** - PWM control from STM32

---

## Software Setup

### 1. Install Dependencies

#### Arduino IDE
- ESP32 Board Support: https://dl.espressif.com/dl/package_esp32_index.json
- STM32 Board Support: https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
- Libraries: ArduinoJson, WebSocketsClient, AsyncTCP, ESPAsyncWebServer

#### Node.js Server
```bash
cd websocket-server
npm install
```

Required packages: `ws`, `sharp`, `express`

**Finding Your PC's IP Address:**
```powershell
ipconfig
```
Look for IPv4 Address (usually 192.168.x.x) - you'll need this for ESP32-CAM configuration.

### 2. Configure WiFi (ESP32-CAM)

Edit `esp32_test1.ino`:

```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* websocket_server = "192.168.1.XXX";  // Your PC's IP from ipconfig
```

⚠️ **Important:** ESP32-CAM and your PC must be on the same WiFi network!

### 3. Set ESP-NOW MAC Address (Master ESP32)

1. Upload `esp32_test1.ino` to ESP32-CAM
2. Open Serial Monitor (115200 baud)
3. Copy the MAC address shown (e.g., `08:F9:E0:EC:CE:1C`)
4. Edit `espnow/espnow.ino`:

```cpp
uint8_t esp32camAddress[] = {0x08, 0xF9, 0xE0, 0xEC, 0xCE, 0x1C};  // Your ESP32-CAM MAC
```

### 4. Match WiFi Channel

Both ESP32 devices must use the same WiFi channel:

**ESP32-CAM** (automatically uses WiFi channel from router)  
**Master ESP32** - edit `espnow/espnow.ino`:

```cpp
esp_wifi_set_channel(3, WIFI_SECOND_CHAN_NONE);  // Match ESP32-CAM's channel
```

Check ESP32-CAM's Serial Monitor for its channel, then update Master ESP32.

### 5. STM32F4 Configuration

#### Using STM32CubeMX (Recommended)
1. Create new project for your STM32F4 board
2. Configure peripherals:
   - **TIM1:** Channels 1 & 2 as PWM (PE9, PE11)
     - Prescaler: 15
     - Period: 255
     - PWM Frequency: ~26 kHz
   - **USART2:** Async mode (PA2=TX, PA3=RX)
     - Baud: 115200, 8-N-1
     - DMA: Enable RX (Stream 5, Circular, 20 bytes)
   - **GPIO Output:** PE8, PE10, PE12, PE13 (motor direction)
   - **GPIO Output:** PD12, PD13, PD14, PD15 (LEDs)
3. Generate code
4. Copy contents from `STM32F4/hand-controlled-robot/Core/Src/main.c` into generated main.c

---

## Upload Sequence

### Step 1: Upload Master ESP32
```
1. Connect Master ESP32 via USB
2. Open espnow/espnow.ino in Arduino IDE
3. Select Board: "ESP32 Dev Module"
4. Select correct COM port
5. Click Upload
6. Open Serial Monitor (115200 baud)
7. Note WiFi channel displayed
```

### Step 2: Upload ESP32-CAM
```
1. Disconnect UART from STM32 if connected!
2. Connect FTDI adapter to ESP32-CAM (RX→U0T, TX→U0R)
3. Connect IO0 to GND (programming mode)
4. Open esp32_test1.ino in Arduino IDE
5. Update WiFi SSID, password, and server IP
6. Select Board: "AI Thinker ESP32-CAM"
7. Select correct COM port
8. Click Upload
9. Disconnect IO0 from GND
10. Press RESET button
11. Open Serial Monitor (115200 baud)
12. Verify WiFi connection
13. Copy MAC address shown
```

### Step 3: Update Master ESP32 MAC
```
1. Paste ESP32-CAM MAC into espnow.ino (line ~11)
2. Verify WiFi channel matches ESP32-CAM
3. Re-upload to Master ESP32
```

### Step 4: Upload STM32F4
```
1. Connect STM32F4 via USB or ST-Link
2. Open hand-controlled-robot.ioc in STM32CubeMX
3. Generate code if needed
4. Open project in STM32CubeIDE
5. Build and upload
```

### Step 5: Connect Hardware
```
1. Power off all devices
2. Connect ESP32-CAM GPIO1 → STM32 PA3
3. Connect GND between ESP32-CAM and STM32
4. Connect STM32 to L298N (follow wiring table above)
5. Connect motors to L298N
6. Connect battery to L298N
7. Connect direction LEDs to STM32 (PD12-15)
```

### Step 6: Start Node.js Server
```bash
cd websocket-server
npm start
```

Open browser: `http://localhost:8080`

---

## Testing

### Test 1: ESP-NOW Communication ✅
**Before connecting motors**

1. Power Master ESP32 and ESP32-CAM
2. Open Serial Monitors for both (115200 baud)

**Master ESP32 should show:**
```
🤖 Robot Controller - Testing Mode
Channel: 3
MAC Target: 08:F9:E0:EC:CE:1C
Sent #1: Mode=0 (FORWARD), R=255, L=255
Sent #2: Mode=0 (FORWARD), R=150, L=255
```

**ESP32-CAM should show:**
```
=== ESP-NOW Data Received ===
Mode: FORWARD (0)
PWM Right: 255, PWM Left: 255
Counter: 1
Dropped: 0 (0.00%)
Sending to STM32: FF 00 FF FF 00
```

✅ **Success:** Counter increments, 0% dropped packets

### Test 2: UART to STM32 ✅
**Motors not connected yet**

1. Connect ESP32-CAM UART to STM32
2. Watch STM32 Serial Monitor (if enabled)
3. Direction LEDs should light according to mode:
   - PD13 (FORWARD) - Both motors forward
   - PD15 (REVERSE) - Both motors reverse
   - PD12 (RIGHT) - Rotating right
   - PD14 (LEFT) - Rotating left

✅ **Success:** LEDs change every 2 seconds with test patterns

### Test 3: Motor Control ⚠️
**IMPORTANT: Place robot on blocks! Wheels must not touch ground!**

1. Connect all motor wiring
2. Connect 7-12V battery to L298N
3. Power on system
4. Observe test sequence (12 patterns, 2 seconds each)

**Test Patterns:**
| # | Mode | Right PWM | Left PWM | Expected Behavior |
|---|------|-----------|----------|-------------------|
| 1 | FORWARD | 255 | 255 | Full speed forward |
| 2 | FORWARD | 150 | 255 | Gentle right curve |
| 3 | FORWARD | 255 | 150 | Gentle left curve |
| 4 | FORWARD | 200 | 180 | Slight right drift |
| 5 | REVERSE | 255 | 255 | Full speed reverse |
| 6 | REVERSE | 200 | 120 | Reverse with drift |
| 7 | RIGHT | 200 | 200 | Rotate right (medium) |
| 8 | RIGHT | 100 | 100 | Rotate right (slow) |
| 9 | LEFT | 255 | 255 | Rotate left (fast) |
| 10 | LEFT | 150 | 150 | Rotate left (medium) |
| 11 | FORWARD | 80 | 80 | Slow creep forward |
| 12 | STOP | 0 | 0 | All motors off |

✅ **Success:** Motors respond to all 12 patterns, LEDs indicate direction

### Test 4: Web Interface 🌐

1. Start the Node.js server:
   ```bash
   cd websocket-server
   npm start
   ```
2. Open browser: `http://localhost:8080`

**You should see:**
- ✅ Live video stream from ESP32-CAM (~18-22 FPS)
- ✅ FPS counter
- ✅ System stats (SRAM, PSRAM, WiFi RSSI)
- ✅ Motor status: Mode (FORWARD/REVERSE/RIGHT/LEFT/STOP)
- ✅ Right Motor PWM: XXX / 255
- ✅ Left Motor PWM: XXX / 255
- ✅ Camera control sliders

**Server Features:**
- Real-time video streaming via WebSocket
- Automatic reconnection handling
- Multiple browser clients supported
- Image processing (blur, brightness, saturation adjustments)
- Clean, responsive web interface

**Troubleshooting Web Interface:**
- Ensure ESP32-CAM and PC are on same WiFi network
- Check firewall isn't blocking port 8080
- Verify IP address in `esp32_test1.ino` matches your PC's IP (use `ipconfig`)
- Check Serial Monitor for WebSocket connection messages
- Open browser console (F12) for WebSocket errors

---

## UART Protocol Specification

### Packet Format (5 bytes)
```
[START] [MODE] [PWM_RIGHT] [PWM_LEFT] [CHECKSUM]
 0xFF    0-4      0-255       0-255    XOR of bytes 1-3
```

### Mode Values
| Value | Mode | Motor Behavior |
|-------|------|----------------|
| 0 | FORWARD | Both forward |
| 1 | REVERSE | Both reverse |
| 2 | RIGHT | Right reverse, left forward (rotate right) |
| 3 | LEFT | Right forward, left reverse (rotate left) |
| 4 | STOP | Both stop |

### Checksum Calculation
```c
uint8_t checksum = mode ^ pwm_right ^ pwm_left;
```

### Example Packet
```
Command: FORWARD, Right=255, Left=200
Packet: [0xFF] [0x00] [0xFF] [0xC8] [0x37]
                                      └─ 0x00 ^ 0xFF ^ 0xC8 = 0x37
```

---

## STM32F4 Pin Configuration

### TIM1 PWM (26 kHz)
| Pin | Channel | Connection | Prescaler | Period |
|-----|---------|------------|-----------|--------|
| PE9 | TIM1_CH1 | L298N ENA | 15 | 255 |
| PE11 | TIM1_CH2 | L298N ENB | 15 | 255 |

### Motor Direction (GPIO Output)
| Pin | Label | L298N Pin | Motor Function |
|-----|-------|-----------|----------------|
| PE8 | RIGHT_FWD | IN1 | Right forward |
| PE10 | RIGHT_REV | IN2 | Right reverse |
| PE12 | LEFT_FWD | IN3 | Left forward |
| PE13 | LEFT_REV | IN4 | Left reverse |

### Direction LEDs (GPIO Output)
| Pin | LED | Active When |
|-----|-----|-------------|
| PD12 | RIGHT | Mode = 2 (rotating right) |
| PD13 | FWD | Mode = 0 (forward) |
| PD14 | LEFT | Mode = 3 (rotating left) |
| PD15 | REV | Mode = 1 (reverse) |

### USART2 with DMA
| Pin | Function | Baud | DMA |
|-----|----------|------|-----|
| PA2 | TX | 115200 | - |
| PA3 | RX | 115200 | Stream 5, Circular, 20 bytes |

---

## Troubleshooting

### ESP-NOW Issues

**Problem:** Dropped packets (>10%)  
**Solution:**
- Verify both ESP32s use the same WiFi channel
- Check MAC address is correct
- Ensure channel is set explicitly: `esp_wifi_set_channel(3, WIFI_SECOND_CHAN_NONE)`

**Problem:** No packets received  
**Solution:**
- Verify MAC address with `esp_wifi_get_mac(WIFI_IF_STA, mac)`
- Check WiFi is connected on ESP32-CAM before ESP-NOW init
- Use `OnDataRecv` callback to verify reception

### Video Stream Issues

**Problem:** Low FPS (<10 FPS)  
**Solution:**
- Reduce image quality: `config.jpeg_quality = 8`
- Lower resolution: `config.frame_size = FRAMESIZE_VGA`
- Increase delay: `delay(40)` for ~25 FPS
- Check WiFi signal strength

**Problem:** No video in browser  
**Solution:**
- Verify WebSocket server IP is correct in esp32_test1.ino
- Check Node.js server is running: `npm start`
- Open browser console for WebSocket errors
- Ensure PC firewall allows port 8080

### UART Communication Issues

**Problem:** STM32 not receiving  
**Solution:**
- Verify baud rate: 115200 on both devices
- Check wiring: ESP32-CAM TX → STM32 RX
- Ensure common ground connection
- Monitor with logic analyzer if available

**Problem:** Checksum errors  
**Solution:**
- Verify XOR calculation: `mode ^ pwm_right ^ pwm_left`
- Check for electrical noise on UART lines
- Add pull-up resistors if needed

### Motor Control Issues

**Problem:** Motors not running  
**Solution:**
- Check L298N power: 7-12V battery connected
- Verify ENA/ENB jumpers are **removed**
- Test PWM with multimeter/oscilloscope (should see ~26 kHz)
- Check motor driver IN1-4 connections

**Problem:** Motors run in wrong direction  
**Solution:**
- Swap motor wires (OUT1↔OUT2 or OUT3↔OUT4)
- Or modify code: swap FWD/REV pin assignments

**Problem:** Weak motor performance  
**Solution:**
- Check battery voltage (>7V recommended)
- Increase PWM values in test patterns
- Verify motor driver cooling (L298N can overheat)

---

## Performance Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Video Resolution | 640x480 (VGA) | Configurable in code |
| Video FPS | 18-22 FPS | With delay(40) |
| JPEG Quality | 8 | Range: 0-63 (lower = better) |
| ESP-NOW Reliability | 100% | With channel matching |
| ESP-NOW Latency | <10ms | Wireless transmission |
| UART Baud Rate | 115200 | ESP32-CAM to STM32 |
| UART Latency | <1ms | DMA reception |
| PWM Frequency | ~26 kHz | TIM1, Prescaler=15, Period=255 |
| PWM Resolution | 8-bit | 0-255 levels |
| WiFi Channel | 3 | Configurable |
| Motor Voltage | 7-12V | L298N input |
| Logic Voltage | 3.3V/5V | ESP32: 3.3V, STM32: 3.3V |

---

## Project Structure

```
esp32_test1/
├── esp32_test1.ino                 # ESP32-CAM (receiver + video streamer)
├── espnow/
│   └── espnow.ino                  # Master ESP32 (sender)
├── STM32F4/
│   └── hand-controlled-robot/
│       ├── hand-controlled-robot.ioc   # STM32CubeMX project file
│       └── Core/Src/main.c             # STM32F4 motor controller (HAL)
├── websocket-server/
│   ├── camera-server.js            # Node.js WebSocket server
│   └── package.json                # Node.js dependencies
└── README.md                       # This file (complete documentation)
```

---

## Key Code Files

### 1. `espnow/espnow.ino`
- Master ESP32 controller
- Generates 12 test movement patterns
- Sends ESP-NOW packets every 2 seconds
- Explicit WiFi channel setting

### 2. `esp32_test1.ino`
- ESP32-CAM receiver
- ESP-NOW callback handler
- UART transmission to STM32
- Video streaming via WebSocket
- System stats reporting

### 3. `STM32F4/hand-controlled-robot/Core/Src/main.c`
- STM32F4 HAL-based motor controller
- DMA UART reception (20-byte circular buffer)
- Command processing with checksum validation
- TIM1 PWM motor control
- Automatic direction LED updates
- RX-only (no serial debug output)

### 4. `websocket-server/camera-server.js`
- Node.js WebSocket server
- JPEG frame reception and relay
- Image processing (blur, brightness, saturation)
- Web UI serving
- Real-time motor status display

---

## License

This project is provided as-is for educational and hobbyist purposes.

---

## Credits

Built using:
- ESP32 Arduino Core
- STM32 HAL Libraries
- Node.js + WebSocket (ws)
- Sharp (image processing)
- Express (web server)

---

## Support

For issues or questions:
1. Check Serial Monitor outputs (115200 baud)
2. Verify wiring against tables above
3. Test each component independently
4. Review troubleshooting section
5. Check ESP-NOW channel matching

**Happy Building! 🤖🎉**
