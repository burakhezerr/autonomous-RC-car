# RC-Car — Sensor Dashboard & Manual Control

Web-based sensor dashboard and manual driving interface running on ESP32-S3.

## Features

- **Sensor Dashboard** — HC-SR04 ultrasonic (×2), TF-Luna LiDAR, MPU6050 IMU — real-time display
- **ARM / DISARM / E-STOP** — web UI buttons and physical GPIO buttons; E-STOP via hardware interrupt (instant response regardless of loop blocking)
- **Manual control** — steering (servo) and throttle (ESC) sliders with 80ms debounce for real-time driving
- **Runtime limits** — throttle and steering limits adjustable from web UI without re-uploading
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
arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" ESP32Code/

# Upload
arduino-cli upload -p /dev/cu.usbmodem1101 \
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" ESP32Code/
```

### Overriding PWM Limits at Compile Time

```bash
arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" \
  --build-property \
  "build.extra_flags=-DSERVO_MIN_US=1730 -DSERVO_MID_US=2030 -DSERVO_MAX_US=2330 \
   -DESC_MIN_US=1370 -DESC_MID_US=1540 -DESC_MAX_US=1570 \
   -DTHROTTLE_LIMIT_PCT=30 -DAUTO_THROTTLE_PCT=20" \
  ESP32Code/
```

| Define | Default | Description |
|--------|---------|-------------|
| `SERVO_MIN_US` | 1730 | Servo left limit (µs) |
| `SERVO_MID_US` | 2030 | Servo centre / neutral (µs) |
| `SERVO_MAX_US` | 2330 | Servo right limit (µs) |
| `ESC_MIN_US` | 1370 | ESC lower limit / reverse (µs) |
| `ESC_MID_US` | 1540 | ESC neutral (µs) |
| `ESC_MAX_US` | 1570 | ESC upper limit / forward (µs) |
| `THROTTLE_LIMIT_PCT` | 30 | Max throttle % above neutral (compile-time default) |
| `AUTO_THROTTLE_PCT` | 20 | Auto-mode throttle % (future use) |

> Runtime limits can also be changed without re-uploading via `POST /set?thr_limit=40&steer_limit=80`.

---

## WiFi & Access

- **SSID:** `Hezer` / **Password:** `burakhezer`
- **Web UI:** `http://esp.local:5000` or `http://<IP>:5000`
- **Find IP:** `arp -a | grep -i "dc:b4:d9"`

> iPhone hotspot: Settings → Personal Hotspot → enable **"Maximise Compatibility"** (forces 2.4GHz).

---

## REST API

All POST endpoints have CORS enabled — accessible from any device on the same network.  
Full endpoint documentation: `swagger.md`

### Quick Reference

```bash
# Arm control
curl -X POST http://esp.local:5000/arm
curl -X POST http://esp.local:5000/disarm
curl -X POST http://esp.local:5000/estop

# Manual control (ARMED state required)
curl -X POST "http://esp.local:5000/control?steer=2000&thr=1550"

# Runtime limit update
curl -X POST "http://esp.local:5000/set?thr_limit=40&steer_limit=80"

# Telemetry
curl http://esp.local:5000/status
curl http://esp.local:5000/data

# API docs
http://esp.local:5000/api/docs
```

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web dashboard (HTML) |
| `/data` | GET | Lightweight sensor JSON (500ms polling) |
| `/status` | GET | Full system state (status.json format) |
| `/api/docs` | GET | Swagger UI |
| `/api/openapi.json` | GET | OpenAPI 3.0 spec |
| `/arm` | POST | DISARMED → ARMED |
| `/disarm` | POST | Any state → DISARMED, servo/ESC to neutral |
| `/estop` | POST | Any state → EMERGENCY, immediate motor stop |
| `/control` | POST | Set `steer` and/or `thr` (µs) — ARMED only |
| `/set` | POST | Update `thr_limit` and/or `steer_limit` (0–100%) |

---

## ARM State Machine

```
DISARMED ──[POST /arm]──► ARMED ──[POST /disarm]──► DISARMED
                            │
                     [POST /estop]
                     [GPIO 18 btn]
                            │
                        EMERGENCY  (must /disarm before re-arming)
```

- Always starts **DISARMED**
- Physical ARM button (GPIO 14): toggles DISARMED ↔ ARMED
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

- **E-STOP latency:** Hardware interrupt on GPIO 18 — immediate regardless of `pulseIn` or HTTP handler blocking.
- **Ultrasonic timeout:** 6ms per sensor (~1m range). Worst-case loop block: 12ms per `/data` call.
- **Slider limits:** Fetched from `/status` on page load — updating `#define` values and re-uploading refreshes the UI automatically.
- **IMU loop:** 100Hz complementary filter running independently of HTTP requests.

---

## Required Libraries

- **Board:** `esp32:esp32` (Espressif) — Arduino IDE Board Manager
- **ESP32Servo** (Kevin Harrington) — Library Manager
- `WiFi`, `WebServer`, `Wire`, `ESPmDNS` — included with board package
