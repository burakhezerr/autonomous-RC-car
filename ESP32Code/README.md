# RC-Car.ino — Code Reference

Sensor dashboard and manual control firmware for ESP32-S3.  
For toolchain setup and arduino-cli installation see `../README.md`.

---

## Pin Map

| Signal | GPIO |
|--------|------|
| Servo (steering) | 4 |
| ESC (throttle) | 5 |
| Left ultrasonic TRIG / ECHO | 10 / 11 |
| Right ultrasonic TRIG / ECHO | 12 / 13 |
| MPU6050 SDA / SCL | 6 / 7 |
| TF-Luna RX←TX / TX→RX | 16 / 17 |

**Power:** Do NOT power servo/ESC from ESP32 — use the ESC's BEC output. Common GND required.

**Ultrasonic mount angle:** 30° — firmware computes `lFwd = cm × cos(30°)`, `lLat = cm × sin(30°)`.

---

## PWM Values

Current defaults compiled into the sketch:

| Define | Value | Description |
|--------|-------|-------------|
| `SERVO_MIN_US` | 1730 | Servo left limit (µs) |
| `SERVO_MID_US` | 2030 | Servo neutral (µs) |
| `SERVO_MAX_US` | 2330 | Servo right limit (µs) |
| `ESC_MIN_US` | 1370 | ESC reverse / lower limit (µs) |
| `ESC_MID_US` | 1540 | ESC neutral (µs) |
| `ESC_MAX_US` | 1570 | ESC forward / upper limit (µs) |
| `THROTTLE_LIMIT_PCT` | 30 | Max throttle % above neutral at boot |
| `AUTO_THROTTLE_PCT` | 20 | Auto-mode throttle % (future use) |

Override at compile time:

```bash
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" \
  --build-property \
  "build.extra_flags=-DSERVO_MID_US=2030 -DESC_MID_US=1540 -DESC_MAX_US=1570" \
  ESP32Code/
```

Runtime limits (no re-upload needed):

```bash
curl -X POST "http://esp.local:5000/set?thr_limit=40&steer_limit=80"
```

---

## WiFi

- **SSID:** `Hezer` / **Password:** `burakhezer`
- **Web UI:** `http://esp.local:5000` or `http://<IP>:5000`
- **Find IP:** `arp -a | grep -i "dc:b4:d9"`

> iPhone hotspot: Settings → Personal Hotspot → **"Maximise Compatibility"** (forces 2.4 GHz).

---

## REST API

CORS enabled on all POST endpoints — any device on the same network can call them.  
Full documentation: `swagger.md`

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web dashboard |
| `/data` | GET | Lightweight sensor JSON (for 500 ms polling) |
| `/status` | GET | Full system state |
| `/api/docs` | GET | Swagger UI |
| `/api/openapi.json` | GET | OpenAPI 3.0 spec |
| `/arm` | POST | DISARMED → ARMED |
| `/disarm` | POST | Any state → DISARMED, outputs to neutral |
| `/estop` | POST | Any state → EMERGENCY, immediate motor stop |
| `/control` | POST | Set `steer` and/or `thr` in µs — ARMED only |
| `/set` | POST | Update `thr_limit` and/or `steer_limit` (0–100%) |

```bash
curl -X POST http://esp.local:5000/arm
curl -X POST "http://esp.local:5000/control?steer=2000&thr=1550"
curl -X POST "http://esp.local:5000/set?thr_limit=40&steer_limit=80"
curl http://esp.local:5000/status
```

---

## ARM State Machine

```
DISARMED ──[/arm]──► ARMED ──[/disarm]──► DISARMED
                       │
                   [/estop]
                       │
                   EMERGENCY  ──[/disarm]──► DISARMED
```

Always boots **DISARMED**. Cannot re-arm from EMERGENCY without disarming first.

---

## IMU — NED Coordinate System

```
X+ = forward   Y+ = right   Z+ = down
```

`az ≈ +1g` on flat ground. Auto-calibration runs at startup (~1.2 s, 300 samples).

---

## Notes

- Ultrasonic `pulseIn` timeout: 6 ms (~1 m range), max loop block 12 ms per `/data` call.
- IMU complementary filter runs at 100 Hz independently of HTTP requests.
- Slider min/max/neutral fetched from `/status` on page load — re-uploading with new `#define` values updates the UI automatically.
