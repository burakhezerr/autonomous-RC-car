# ArduinoIDE — ESP32-S3 Onboard Code

Onboard ESP32-S3 firmware for the autonomous RC car. Main sketch: `ESP32Code/ESP32Code.ino`.

```
ArduinoIDE/
├── README.md              (this file — toolchain setup)
├── status.json            (telemetry format reference with comments)
├── session.md             (REST API integration guide)
├── ESP32Code/
│   ├── ESP32Code.ino      — main sketch (manual + auto mode)
│   ├── README.md          — pins, PWM values, API reference
│   └── swagger.md         — full REST API documentation
└── ESP32Code-Yusuf/
    └── ESP32Code-Yusuf.ino — multi-car variant with RL policy
```

---

## arduino-cli

### Installation

**macOS (Homebrew):**

```bash
brew install arduino-cli
```

**Windows (winget):**

```powershell
winget install ArduinoSA.arduino-cli
```

Verify:

```bash
arduino-cli version
# arduino-cli Version: 1.x.x ...
```

---

## First-Time Board Setup

Run these once after installing arduino-cli:

```bash
# 1. Create config file
arduino-cli config init

# 2. Add Espressif board index
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json

# 3. Fetch index
arduino-cli core update-index

# 4. Install ESP32 core (~200 MB)
arduino-cli core install esp32:esp32

# 5. Install ESP32Servo library
arduino-cli lib install "ESP32Servo"

# 6. Confirm
arduino-cli core list
```

---

## Finding the Port

**macOS:**

```bash
ls /dev/cu.*
# or
arduino-cli board list
```

ESP32-S3 with CDC-on-boot enabled typically appears as `/dev/cu.usbmodem1101`.

**Windows:**

```powershell
arduino-cli board list
# or: Device Manager → Ports (COM & LPT)
```

> If nothing appears: use a data-capable USB cable. Hold the BOOT button during upload if the port disappears.

---

## Compile & Upload

```bash
# Compile only
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" \
  ESP32Code/

# Upload
arduino-cli upload \
  -p /dev/cu.usbmodem1101 \
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" \
  ESP32Code/

# Compile + upload together
arduino-cli compile --upload \
  -p /dev/cu.usbmodem1101 \
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" \
  ESP32Code/

# Serial monitor (115200 baud)
arduino-cli monitor -p /dev/cu.usbmodem1101 --config baudrate=115200
```

### Flag Reference

| Flag | Meaning |
|------|---------|
| `--fqbn esp32:esp32:esp32s3` | Fully Qualified Board Name — Espressif ESP32-S3 |
| `CDCOnBoot=cdc` | Enables USB-Serial via CDC — required for Serial Monitor |
| `-p /dev/cu.usbmodem1101` | Serial port |
| `ESP32Code/` | Sketch folder — must match the `.ino` filename inside |

---

## Safety

- **Wheels off the ground** during all initial tests.
- Common GND is mandatory across ESP32, ESC, servo, and all sensors.
- System boots **DISARMED** — motors do not move until explicitly armed via web UI or ARM button.
- Watchdog: command timeout → motor neutral + DISARMED lock.
