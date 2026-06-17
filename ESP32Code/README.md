# ESP32Code — Code Reference

Sensor dashboard, manual control, and autonomous mode firmware for ESP32-S3.  
For toolchain setup and arduino-cli installation see `../README.md`.

---

## Pin Map

| Signal | GPIO |
|--------|------|
| Servo (steering) | 4 |
| ESC (throttle) | 5 |
| Right ultrasonic TRIG / ECHO | 10 / 11 |
| Left ultrasonic TRIG / ECHO | 12 / 13 |
| MPU6050 SDA / SCL | 6 / 7 |
| TF-Luna RX←TX / TX→RX | 16 / 17 |
| ARM button (NO, to GND) | 14 |
| E-STOP button (NO, to GND) | 18 |
| ARM LED (220Ω series) | 21 |

**Power:** Do NOT power servo/ESC from ESP32 — use the ESC's BEC output. Common GND required.

**Ultrasonic mount angle:** 30° — firmware computes `lFwd = cm × cos(30°)`, `lLat = cm × sin(30°)`.

---

## PWM Values

| Define | Value | Description |
|--------|-------|-------------|
| `SERVO_MIN_US` | 1820 | Servo left limit (µs) |
| `SERVO_MID_US` | 2040 | Servo neutral / straight (µs) |
| `SERVO_MAX_US` | 2260 | Servo right limit (µs) |
| `ESC_MIN_US` | 1430 | ESC full forward (µs) |
| `ESC_MID_US` | 1495 | ESC neutral / stop (µs) |
| `ESC_MAX_US` | 1560 | ESC full reverse (µs) |
| `THROTTLE_LIMIT_PCT` | 30 | Max throttle % above neutral at boot |
| `AUTO_THROTTLE_PCT` | 20 | Auto-mode throttle % (reserved) |

Override at compile time:

```bash
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" \
  --build-property \
  "build.extra_flags=-DSERVO_MID_US=2040 -DESC_MID_US=1495 -DESC_MAX_US=1560" \
  ESP32Code/
```

---

## Autonomous Mode — Runtime Thresholds

All threshold values are global `int` variables — changeable at runtime without re-upload.

### Sonar proximity levels (`POST /set-prox`)

| Variable | Default | Description |
|----------|---------|-------------|
| `sonarCloseCm` | 10 | ≤ this → emergency stop + full steer away (both auto and manual) |
| `proxLv1` | 35 | ≤ this → Level 1 (critical danger) |
| `proxLv2` | 60 | ≤ this → Level 2 (close) |
| `proxLv3` | 85 | ≤ this → Level 3 (moderate) |
| `proxLv4` | 115 | ≤ this → Level 4 (safe approach); above → Level 5 (clear) |

Validation rules: `close ≥ 10`, `lv1 ≥ 25`, each consecutive gap `≥ 20 cm`.  
Only settable when DISARMED + MANUAL mode.

### LiDAR speed thresholds (`POST /set-lidar`)

| Variable | Default | Description |
|----------|---------|-------------|
| `lidarStopCm` | 50 | ≤ this → EMERGENCY (wall detected, auto re-arm after 2 s if clear) |
| `lidarSlowCm` | 100 | ≤ this → reduced speed (1550 µs); above → full speed (1560 µs) |

Validation rules: `stop ≥ 40`, `slow ≥ stop × 2`.  
Only settable when DISARMED + MANUAL mode.

```bash
curl -X POST "http://esp.local:5000/set-prox?lv1=35&lv2=60&lv3=85&lv4=115&close=10"
curl -X POST "http://esp.local:5000/set-lidar?stop=50&slow=100"
```

---

## RL Policy (`policy_weights.h`)

Used in **multi-car AUTO mode** when a moving obstacle is detected within `OBSTACLE_RANGE_CM = 150 cm`.

**Architecture:** MLP `7 → 128 → 64 → 4`  
**Hidden activation:** tanh  
**Output activation:** softmax → argmax selects strategy

### Observation vector (7 inputs)

| Index | Input | Range |
|-------|-------|-------|
| 0 | Throttle (normalised) | 0.0 – 1.16 |
| 1 | LiDAR distance (m) | 0.0 – 3.0 |
| 2 | Left sonar (m) | 0.02 – 1.5 |
| 3 | Right sonar (m) | 0.02 – 1.5 |
| 4 | Heading (reserved) | 0.0 – 1.0 |
| 5 | Yaw rate gz (deg/s) | −5.0 – 5.0 |
| 6 | Spare / context | 0.0 – 10.0 |

### Strategies (4 outputs)

| Index | Name | Behaviour |
|-------|------|-----------|
| 0 | `safe` | ESC neutral — hold |
| 1 | `normal` | ESC 1557 µs — proceed |
| 2 | `pass_left` | ESC 1557 µs + servo −80 µs offset — overtake left |
| 3 | `pass_right` | ESC 1557 µs + servo +80 µs offset — overtake right |

RL is **bypassed** when obstacle is static (`isDynamic()` returns false) or LiDAR > 150 cm; falls back to `autoControl()`.

---

## WiFi

- **SSID:** `Ando` / **Password:** `andrey12`
- **Web UI:** `http://esp.local:5000` or `http://<IP>:5000`
- **Find IP:** `arp -a | grep -i "dc:b4:d9"` (MAC: `dc:b4:d9:0c:6a:b8`)

> iPhone hotspot: Settings → Personal Hotspot → **"Maximise Compatibility"** (forces 2.4 GHz).

---

## REST API

CORS enabled on all POST endpoints. Full documentation: `swagger.md`

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web dashboard |
| `/data` | GET | Lightweight sensor JSON (500 ms polling) |
| `/status` | GET | Full system state + active thresholds |
| `/sensors` | GET | Raw sensor snapshot (sonar + LiDAR + IMU) |
| `/rl` | GET | RL state: active flag, current strategy, LiDAR distance |
| `/api/docs` | GET | Swagger UI (requires internet for CDN) |
| `/api/openapi.json` | GET | OpenAPI 3.0 spec |
| `/arm` | POST | DISARMED → ARMED |
| `/disarm` | POST | Any state → DISARMED, outputs to neutral |
| `/estop` | POST | Any state → EMERGENCY, immediate motor stop |
| `/control` | POST | Set `steer` and/or `thr` in µs — ARMED + MANUAL only |
| `/set` | POST | Update `thr_limit` and/or `steer_limit` (0–100%) |
| `/set-prox` | POST | Update sonar proximity thresholds — DISARMED + MANUAL only |
| `/set-lidar` | POST | Update LiDAR stop/slow thresholds — DISARMED + MANUAL only |
| `/mode` | POST | Switch `auto` / `manual` run mode |
| `/auto-version` | POST | Switch `single` / `multi` autonomous version |

```bash
curl -X POST http://esp.local:5000/arm
curl -X POST "http://esp.local:5000/control?steer=2040&thr=1430"
curl -X POST "http://esp.local:5000/set-prox?lv1=35&lv2=60&lv3=85&lv4=115&close=10"
curl -X POST "http://esp.local:5000/set-lidar?stop=50&slow=100"
curl -X POST "http://esp.local:5000/auto-version?version=multi"
curl http://esp.local:5000/status
curl http://esp.local:5000/rl
```

---

## ARM State Machine

```
DISARMED ──[/arm]──► ARMED ──[/disarm]──► DISARMED
                       │
               [/estop or wall detected in AUTO]
                       │
                   EMERGENCY ──[/disarm]──► DISARMED
                       │
               [wall cleared, AUTO mode only]
                       │
                     ARMED  (auto re-arm after 2 s)
```

Always boots **DISARMED**. Cannot re-arm from EMERGENCY without disarming first,  
except in AUTO mode wall recovery: waits 2 s, re-arms automatically if LiDAR > 50 cm.
In EMERGENCY the servo stays steerable for the first 1 s (escape maneuver), then re-centres; the ESC is neutral throughout.

---

## IMU — Coordinate System

```
X+ = forward   Y+ = right   Z+ = down
```

`az ≈ +1g` on flat ground. Auto-calibration runs at startup (~1.2 s, 300 samples).

---

## Notes

- Ultrasonic `pulseIn` timeout: 15 ms (~2.6 m range).
- IMU complementary filter runs at 100 Hz; `autoControl` / `manualSafety` run at 20 Hz.
- Slider min/max/neutral fetched from `/status` on page load — new `#define` values update the UI automatically after re-upload.
- Active threshold values are always reflected in `/status` under `sonar.thresholds` and `lidar.thresholds`.
