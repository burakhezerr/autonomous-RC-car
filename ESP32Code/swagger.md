# RC Car ESP32-S3 — API Reference

**Base URL:** `http://esp.local:5000`  (veya `http://<IP>:5000`)  
**Version:** 0.5.0  
**CORS:** Tüm POST endpoint'leri `Access-Control-Allow-Origin: *` döner — aynı ağdaki her cihazdan erişilebilir.

---

## GET Endpoints

---

### `GET /`

Web dashboard (HTML sayfası). Tarayıcıdan açılır.

**Response:** `200 text/html` — Sensor dashboard + manual control UI

```
http://esp.local:5000/
```

### `GET /status`

Tam sistem durumu. `status.json` formatıyla config + telemetry.

**Response:** `200 application/json`

```json
{
  "config": {
    "servo": { "minUs": 1730, "neutralUs": 2030, "maxUs": 2330 },
    "esc":   { "minUs": 1430, "neutralUs": 1490, "maxUs": 1550 }
  },
  "telemetry": {
    "sonar": {
      "lCm": 45.2, "rCm": 38.7,
      "lLv": 2,    "rLv": 1,
      "lFwd": 39.1, "lLat": 22.6,
      "rFwd": 33.5, "rLat": 19.4,
      "angle": 30.0
    },
    "lidar": { "ok": true, "cm": 120.0 },
    "imu": {
      "ok": true, "calibrated": true,
      "ax": 0.012, "ay": -0.034, "az": 9.810,
      "gx": 0.010, "gy": -0.020, "gz": 0.000,
      "roll": 1.20, "pitch": -0.50, "yaw": 45.30,
      "temp": 32.1
    },
    "odometry": { "x": null, "y": null },
    "control": {
      "steer": null, "steerPct": 12,
      "servoUs": 2060, "thrPct": 5, "escUs": 1476
    },
    "battery": { "v": null, "pct": null },
    "system": {
      "mstate": "ARMED",
      "armPct": 100,
      "disarmReason": "none",
      "mode": "MANUAL",
      "emergency": false
    },
    "wireless": { "wifiOk": true, "rssi": -62 },
    "limits": { "throttleLimit": 30, "autoThrottle": 20 }
  }
}
```

| Alan | Açıklama |
|------|----------|
| `config.servo.{minUs,neutralUs,maxUs}` | Derleme zamanı servo PWM limitleri (µs) |
| `config.esc.{minUs,neutralUs,maxUs}` | Derleme zamanı ESC PWM limitleri (µs) |
| `sonar.lLv / rLv` | Yakınlık seviyesi 0–4 (4=çok yakın <20cm) |
| `sonar.lFwd / lLat` | İleri + yanal mesafe bileşenleri (cm) |
| `imu.roll/pitch/yaw` | Complementary filter çıkışı (°) |
| `system.mstate` | `"DISARMED"` / `"ARMED"` / `"EMERGENCY"` |
| `system.disarmReason` | Son disarm nedeni (`"none"`, `"btn_manual"`, `"web_manual"`, `"emergency_stop"`) |
| `limits.throttleLimit` | Runtime gaz limiti (%) — `POST /set` ile değiştirilebilir |
| `limits.autoThrottle` | Oto mod gaz limiti (%, şimdilik statik) |

---

### `GET /api/docs`

Swagger UI. CDN'den yükler — internet gerektirir.

```
http://esp.local:5000/api/docs
```

---

### `GET /api/openapi.json`

OpenAPI 3.0 spec (JSON). Swagger UI buradan yükler.

**Response:** `200 application/json` — OpenAPI spec

---

## POST Endpoints

---

### `POST /arm`

Motoru ARM eder. `DISARMED → ARMED` geçişi yapar.

**Kural:** `EMERGENCY` durumundaysa önce `/disarm` çağrılmalı.

**Request body:** yok  
**Query params:** yok

**Responses:**

| HTTP | Durum | Body |
|------|-------|------|
| 200 | Başarılı | `{"ok":true,"state":"ARMED"}` |
| 403 | E-STOP aktif | `{"ok":false,"msg":"E-STOP active, disarm first"}` |

**Örnek:**
```bash
curl -X POST http://esp.local:5000/arm
```

---

### `POST /disarm`

Motoru DISARM eder. Her durumdan `DISARMED`'a geçer. Servo ve ESC nötr'e alınır.

**Request body:** yok  
**Query params:** yok

**Response:** `200`

```json
{"ok": true, "state": "DISARMED"}
```

**Örnek:**
```bash
curl -X POST http://esp.local:5000/disarm
```

---

### `POST /estop`

Acil durdurma. `EMERGENCY` durumuna geçer, motor anında nötr'e alınır. Çıkış için `/disarm` zorunlu — otomatik ARM'a dönüş yoktur.

**Request body:** yok  
**Query params:** yok

**Response:** `200`

```json
{"ok": true, "state": "EMERGENCY"}
```

**Örnek:**
```bash
curl -X POST http://esp.local:5000/estop
```

---

### `POST /control`

Direksiyon (servo) ve gaz (ESC) değeri yazar. **Yalnızca `ARMED` durumunda çalışır.**

**Query params:**

| Parametre | Tip | Zorunlu | Açıklama |
|-----------|-----|---------|----------|
| `steer` | int (µs) | hayır | Servo PWM değeri. `steerLimitPct` ile kırpılır. |
| `thr` | int (µs) | hayır | ESC PWM değeri. `throttleLimitPct` ile kırpılır. |

**Varsayılan PWM aralıkları (derleme sabitleri):**

| Sinyal | Min | Nötr | Max |
|--------|-----|------|-----|
| Servo (steer) | 1730 µs (sol) | 2030 µs (düz) | 2330 µs (sağ) |
| ESC (thr) | 1430 µs (ileri) | 1490 µs (nötr) | 1550 µs (geri) |

**Sunucu tarafı sınır hesabı:**
```
steer → [MID - (MID-MIN)*steerLimitPct/100,  MID + (MAX-MID)*steerLimitPct/100]
thr   → [ESC_MIN,  ESC_MID + (ESC_MAX-ESC_MID)*throttleLimitPct/100]
```

**Responses:**

| HTTP | Durum | Body |
|------|-------|------|
| 200 | Değer yazıldı | `{"ok":true,"servoUs":1900,"escUs":1480}` |
| 403 | ARM değil | `{"ok":false,"msg":"ARM degil"}` |

**Örnekler:**
```bash
# Düz, hafif ileri (1430=tam ileri, 1490=nötr)
curl -X POST "http://esp.local:5000/control?steer=2030&thr=1460"

# Sola dön, nötr gaz
curl -X POST "http://esp.local:5000/control?steer=1830"

# Sadece gaz (tam ileri)
curl -X POST "http://esp.local:5000/control?thr=1430"
```

---

### `POST /set`

Runtime parametre güncelleme. Yeniden yüklemeye gerek kalmadan limit değiştirir.

**Query params:**

| Parametre | Tip | Aralık | Açıklama |
|-----------|-----|--------|----------|
| `thr_limit` | int | 0–100 | Gaz limiti (%). `100` = tam ESC aralığı, `30` = max 1600'ın %30'u kadar üstü |
| `steer_limit` | int | 0–100 | Direksiyon limiti (%). `100` = tam servo aralığı, `50` = yarı açı |

En az bir parametre zorunlu. İkisi birden gönderilebilir.

**Responses:**

| HTTP | Durum | Body |
|------|-------|------|
| 200 | Güncellendi | `{"ok":true,"throttleLimit":40,"steerLimit":80}` |
| 400 | Parametre yok | `{"ok":false,"msg":"no param"}` |

**Örnekler:**
```bash
# Gaz limitini %40'a çek
curl -X POST "http://esp.local:5000/set?thr_limit=40"

# Direksiyonu %60 aralığa kısıtla
curl -X POST "http://esp.local:5000/set?steer_limit=60"

# İkisini birden güncelle
curl -X POST "http://esp.local:5000/set?thr_limit=40&steer_limit=60"
```

---

## ARM State Machine

```
               ┌─────────────────────────────────┐
               │                                 │
   [power on]  ▼        POST /arm               │
  ──────────► DISARMED ──────────────────► ARMED │
               ▲   ▲                       │  │  │
               │   │    POST /disarm       │  │  │
               │   └───────────────────────┘  │  │
               │                              │  │
               │         POST /estop          │  │
               │   ┌──────────────────────────┘  │
               │   │  (physical btn GPIO 18)      │
               │   ▼                              │
               │ EMERGENCY ◄──────────────────────┘
               │     │
               └─────┘  POST /disarm (→ DISARMED, not ARMED)
```

- `EMERGENCY`'den çıkış: yalnızca `/disarm`
- Fiziksel E-STOP butonu (GPIO 18): **hardware interrupt** — loop() bloke olsa bile anında tetiklenir
- `ARMED` durumunda `/arm` çağırmak → hata vermez, state değişmez

---

## Python Örneği

```python
import urllib.request, json

BASE = "http://esp.local:5000"

def post(path):
    req = urllib.request.Request(f"{BASE}{path}", method="POST", data=b"")
    with urllib.request.urlopen(req) as r:
        return json.loads(r.read())

def get_status():
    with urllib.request.urlopen(f"{BASE}/status") as r:
        return json.loads(r.read())

post("/arm")
post("/control?steer=2030&thr=1460")
post("/set?thr_limit=35&steer_limit=70")
post("/disarm")
```
