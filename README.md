# ArduinoIDE — ESP32-S3 Onboard Code

This directory contains the onboard ESP32-S3 firmware for the autonomous RC car project.

## Directory Structure

```
ArduinoIDE/
├── README.md          (this file)
├── ESP32Code/
│   ├── RC-Car.ino     — main sketch
│   ├── README.md      — build, wiring, API reference
│   └── swagger.md     — full REST API documentation
└── status.json        — telemetry format reference
```

## One-Time Setup

1. **Arduino IDE 2.x** — arduino.cc
2. **ESP32 board package:**
   - `File → Preferences → Additional Board Manager URLs`:
     `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   - `Tools → Board → Boards Manager` → install **"esp32" (Espressif)**
3. **Libraries** (`Tools → Manage Libraries`):
   - `ESP32Servo` (Kevin Harrington)
4. **Board:** `Tools → Board → ESP32 Arduino → ESP32S3 Dev Module`
5. **CDCOnBoot:** `Tools → USB CDC On Boot → Enabled`
6. **Port:** `Tools → Port` — select the ESP32-S3 USB port
7. **Baud:** Serial Monitor → **115200**

> If the port doesn't appear: use a data-capable USB cable. Hold the BOOT button during upload if needed.

## Build & Upload (arduino-cli)

```bash
# Compile
arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" ESP32Code/

# Upload
arduino-cli upload -p /dev/cu.usbmodem1101 \
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" ESP32Code/
```

## Pin Map

| Signal | GPIO |
|--------|------|
| Servo (steering) | 4 |
| ESC (throttle) | 5 |
| Left ultrasonic TRIG / ECHO | 10 / 11 |
| Right ultrasonic TRIG / ECHO | 12 / 13 |
| MPU6050 SDA / SCL | 6 / 7 |
| TF-Luna RX←TX / TX→RX | 16 / 17 |
| ARM button (NO, to GND) | 14 |
| E-STOP button (NO, to GND) | 18 |
| ARM LED (220Ω series) | 21 |

## Wiring Guide

### Power Architecture

```
LiPo
 ├──► ESC (motor power — direct)
 │      └── BEC output (5V) ──► Servo VCC
 │                           └──► ESP32 5V (optional)
 └── COMMON GND ──► ESC ──► Servo ──► ESP32 ──► IMU
```

**Rule:** ESP32 provides signal + GND only. Never power servo/ESC from ESP32 — use the ESC's BEC output.

### Servo → GPIO 4

| Wire | Colour | Connect to |
|------|--------|------------|
| Signal | Yellow / Orange | GPIO 4 |
| VCC | Red | ESC BEC 5V |
| GND | Brown / Black | GND (common) |

### ESC → GPIO 5

| Wire | Colour | Connect to |
|------|--------|------------|
| Signal | Yellow / Orange | GPIO 5 |
| BEC+ | Red | ESP32 5V (optional) |
| GND | Black | GND (common) |

### MPU6050 → GPIO 6 / 7

| Pin | Connect to |
|-----|------------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 6 |
| SCL | GPIO 7 |
| AD0 | GND (sets I²C address to 0x68) |

### Ultrasonic sensors → GPIO 10–13

| Sensor | TRIG | ECHO |
|--------|------|------|
| Left | GPIO 10 | GPIO 11 |
| Right | GPIO 12 | GPIO 13 |

Mount angle: 30° — firmware computes forward and lateral projections automatically.

### TF-Luna LiDAR → GPIO 16 / 17 (UART1)

| Pin | Connect to |
|-----|------------|
| TX | GPIO 16 (ESP RX) |
| RX | GPIO 17 (ESP TX) |
| VCC | 5V |
| GND | GND |

## Safety

- **Wheels off the ground** during all initial tests.
- Common GND is mandatory across all components.
- System boots **DISARMED** — motors do not move until explicitly armed.
- E-STOP button (GPIO 18) uses a hardware interrupt — instant response regardless of loop blocking.
- Watchdog: command timeout → motor neutral + DISARMED lock.

## WiFi & Web UI

- **SSID:** `Hezer` / **Password:** `burakhezer`
- **Web UI:** `http://esp.local:5000` or `http://<IP>:5000`
- **Find IP:** `arp -a | grep -i "dc:b4:d9"`

See `ESP32Code/README.md` for full REST API reference and `ESP32Code/swagger.md` for detailed endpoint documentation.
