# Phase 0.5 — Sensor Dashboard & Manual Control

Web-based sensor dashboard and manual driving interface running on ESP32-S3.

## Features

- **Sensor Dashboard** — HC-SR04 ultrasonic (×2), TF-Luna LiDAR, MPU6050 IMU — real-time display
- **ARM / DISARM / E-STOP** — web UI buttons and physical GPIO buttons; E-STOP via hardware interrupt (instant response regardless of loop blocking)
- **Manual control** — steering (servo) and throttle (ESC) sliders with 80ms throttle for real-time driving
- **REST API** — any device on the same network can POST to `/control` (CORS enabled)
- **`/status` endpoint** — full telemetry in `status.json` format
- **`/api/docs`** — Swagger UI (requires internet for CDN assets)

---

## Hardware Connections

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

**Power note:** Do NOT power servo/ESC from ESP32 — use the ESC's BEC output. Common GND is required.

**Ultrasonic mount angle:** 30° — code computes `lFwd = cm × cos(30°)`, `lLat = cm × sin(30°)`.

---

## Build & Upload

```bash
# Compile
arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" Faz0_5_WebTest/

# Upload
arduino-cli upload -p /dev/cu.usbmodem1101 \
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" Faz0_5_WebTest/
```

### Overriding PWM Limits

```bash
arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" \
  --build-property \
  "build.extra_flags=-DSERVO_MIN_US=1700 -DSERVO_MID_US=2000 -DSERVO_MAX_US=2300 \
   -DESC_MIN_US=1000 -DESC_MID_US=1500 -DESC_MAX_US=2000 \
   -DTHROTTLE_LIMIT_PCT=40 -DAUTO_THROTTLE_PCT=25" \
  Faz0_5_WebTest/
```

| Define | Default | Description |
|--------|---------|-------------|
| `SERVO_MIN_US` | 1700 | Servo left limit (µs) |
| `SERVO_MID_US` | 2000 | Servo centre / neutral (µs) |
| `SERVO_MAX_US` | 2300 | Servo right limit (µs) |
| `ESC_MIN_US` | 1000 | ESC lower limit (µs) |
| `ESC_MID_US` | 1500 | ESC neutral (µs) |
| `ESC_MAX_US` | 1650 | ESC upper limit (µs) |
| `THROTTLE_LIMIT_PCT` | 30 | Max throttle % above neutral |
| `AUTO_THROTTLE_PCT` | 20 | Auto-mode throttle % (future phases) |

---

## WiFi & Access

- **SSID:** `Hezer` / **Password:** `burakhezer`
- **Web UI:** `http://esp.local:5000` or `http://<IP>:5000`
- **Find IP:** `arp -a | grep -i "dc:b4:d9"`

> iPhone hotspot: Settings → Personal Hotspot → enable **"Maximise Compatibility"** (forces 2.4GHz).

---

## REST API

All POST endpoints have CORS enabled — accessible from any device on the same network.

### Arm Control

```bash
curl -X POST http://esp.local:5000/arm
curl -X POST http://esp.local:5000/disarm
curl -X POST http://esp.local:5000/estop
```

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/arm` | POST | Arm motors (blocked in EMERGENCY) |
| `/disarm` | POST | Disarm, set servo/ESC to neutral |
| `/estop` | POST | Emergency stop — requires `/disarm` to exit |

### Manual Control

```bash
# steer: 1000 (full left) – 2000 (neutral) – 2300 (full right)
# thr:   1000 (reverse/min) – 1500 (neutral) – max: neutral + range × THROTTLE_LIMIT_PCT
curl -X POST "http://esp.local:5000/control?steer=1900&thr=1560"
```

Response: `{"ok":true,"servoUs":1900,"escUs":1560}`

### Telemetry

```bash
curl http://esp.local:5000/status   # full state (status.json format)
curl http://esp.local:5000/data     # lightweight polling data
```

### API Docs

```
http://esp.local:5000/api/docs          # Swagger UI
http://esp.local:5000/api/openapi.json  # OpenAPI 3.0 spec
```

---

## ARM State Machine

```
DISARMED ──[ARM]──► ARMED ──[DISARM]──► DISARMED
                      │
                  [E-STOP]
                      │
                  EMERGENCY  (ARM blocked — must DISARM first)
```

- Always starts **DISARMED**
- To exit EMERGENCY: POST `/disarm` or press ARM button (goes to DISARMED, not ARMED)
- Physical ARM button (GPIO 14): short press = toggle (DISARMED↔ARMED)
- Physical E-STOP button (GPIO 18): **hardware interrupt** — fires instantly, no loop() latency
- ARM LED (GPIO 21): HIGH when ARMED, LOW otherwise

---

## IMU Coordinate System (NED)

```
X+ = vehicle forward
Y+ = vehicle right
Z+ = down (gravity direction)
```

On flat ground with correct mounting: `az ≈ +1g`. Calibration runs automatically at startup (~1.2s, 300 samples).

---

## Performance Notes

- **E-STOP latency:** Hardware interrupt on GPIO 18 — response is immediate regardless of `pulseIn` or HTTP handler blocking.
- **Ultrasonic timeout:** 6ms per sensor (~1m range). Reduces worst-case blocking from 60ms to 12ms per `/data` call.
- **Slider limits:** Fetched from `/status` on page load — changing `#define` values and re-uploading updates the UI automatically.

---

## Required Libraries

- **Board:** `esp32:esp32` (Espressif) — Arduino IDE Board Manager
- **ESP32Servo** (Kevin Harrington) — Library Manager
- `WiFi`, `WebServer`, `Wire`, `ESPmDNS` — included with board package
