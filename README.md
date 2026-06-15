# ArduinoIDE — ESP32-S3 Onboard Code

Onboard ESP32-S3 firmware for the autonomous RC car. Code lives in `ESP32Code/RC-Car.ino`.

```
ArduinoIDE/
├── README.md          (this file — toolchain setup)
├── ESP32Code/
│   ├── RC-Car.ino     — main sketch
│   ├── README.md      — pins, PWM values, API reference
│   └── swagger.md     — full REST API documentation
└── status.json        — telemetry format reference
```

---

## arduino-cli

Official docs: [https://arduino.github.io/arduino-cli/](https://arduino.github.io/arduino-cli/)

### Installation

**macOS (Homebrew — recommended):**

```bash
brew install arduino-cli
```

**macOS (manual):**

```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/
```

**Windows (winget):**

```powershell
winget install ArduinoSA.arduino-cli
```

**Windows (manual):**

1. Go to [https://arduino.github.io/arduino-cli/](https://arduino.github.io/arduino-cli/) → Installation → download the Windows ZIP
2. Extract `arduino-cli.exe` to a folder, e.g. `C:\tools\arduino-cli\`
3. Add that folder to PATH: Start → "Environment Variables" → `Path` → Edit → New → paste the path

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

You need the port before uploading.

**macOS:**

```bash
ls /dev/cu.*
# or
arduino-cli board list
```

ESP32-S3 with CDC-on-boot enabled typically appears as `/dev/cu.usbmodem1101` (the number varies).

**Windows:**

```powershell
arduino-cli board list
# or: Device Manager → Ports (COM & LPT)
```

Shows as `COM3`, `COM4`, etc.

> If nothing appears: use a data-capable USB cable (not charge-only). Hold the BOOT button during upload if the port disappears.

---

## Compile & Upload

Replace the port with your actual value from above.

```bash
# Compile
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

# Windows port example
arduino-cli compile --upload \
  -p COM4 \
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" \
  ESP32Code/

# Serial monitor (115200 baud)
arduino-cli monitor -p /dev/cu.usbmodem1101 --config baudrate=115200
```

### Flag Reference

| Flag | Meaning |
|------|---------|
| `--fqbn esp32:esp32:esp32s3` | Fully Qualified Board Name — Espressif core, ESP32 family, S3 variant |
| `CDCOnBoot=cdc` | Enables USB-Serial via built-in CDC — required for Serial Monitor without a separate USB-UART chip |
| `-p /dev/cu.usbmodem1101` | Serial port the board is on |
| `ESP32Code/` | Sketch folder — must match the `.ino` filename inside |

---

## Safety

- **Wheels off the ground** during all initial tests.
- Common GND is mandatory across ESP32, ESC, servo, and all sensors.
- System boots **DISARMED** — motors do not move until explicitly armed via web UI.
- Watchdog: command timeout → motor neutral + DISARMED lock.
