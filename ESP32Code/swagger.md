# RC Car ESP32-S3 — API Reference

**Base URL:** `http://esp.local:5000`  (veya `http://<IP>:5000`)
**Version:** 0.5.0
**CORS:** Tüm POST endpoint'leri `Access-Control-Allow-Origin: *` döner ve `OPTIONS` preflight'ı yanıtlar — aynı ağdaki her cihazdan erişilebilir.

> Bu doküman firmware'in **gerçek davranışını** yansıtır. Tarayıcı içi Swagger UI (`/api/docs`) ise koda gömülü `OPENAPI_JSON` spesinden üretilir; ikisi arasında küçük farklar olabilir — bu dosya ground-truth'tur.

---

## PWM Referansı (derleme sabitleri)

| Sinyal | Min | Nötr | Max |
|--------|-----|------|-----|
| Servo (steer) | `1820` µs (sol) | `2040` µs (düz) | `2260` µs (sağ) |
| ESC (thr) | `1430` µs (tam ileri) | `1495` µs (nötr/stop) | `1560` µs (tam geri) |

Bu değerler `#define` ile gelir, derlemede `-DSERVO_MID_US=...` gibi override edilebilir. Web arayüzü slider sınırlarını açılışta `/status`'tan çeker, dolayısıyla yeniden yükleme sonrası UI otomatik güncellenir.

Diğer sabitler: `THROTTLE_LIMIT_PCT=30`, `AUTO_THROTTLE_PCT=20`.

---

## GET Endpoints

---

### `GET /`

Web dashboard (HTML). Tarayıcıdan açılır — sensör paneli + manuel/oto kontrol UI.

**Response:** `200 text/html`

```
http://esp.local:5000/
```

---

### `GET /status`

Tam sistem durumu (`status.json` formatı): config + telemetry.

**Response:** `200 application/json`

```json
{
  "config": {
    "servo": { "minUs": 1820, "neutralUs": 2040, "maxUs": 2260 },
    "esc":   { "minUs": 1430, "neutralUs": 1495, "maxUs": 1560 }
  },
  "telemetry": {
    "sonar": {
      "lCm": 45.2, "rCm": 38.7,
      "lLv": 2,    "rLv": 1,
      "lFwd": 39.1, "lLat": 22.6,
      "rFwd": 33.5, "rLat": 19.4,
      "angle": 30.0,
      "thresholds": { "close": 10, "lv1": 35, "lv2": 60, "lv3": 85, "lv4": 115 }
    },
    "lidar": { "ok": true, "cm": 120.0, "thresholds": { "stop": 50, "slow": 100 } },
    "imu": {
      "ok": true, "calibrated": true,
      "ax": 0.012, "ay": -0.034, "az": 0.998,
      "gx": 0.010, "gy": -0.020, "gz": 0.000,
      "roll": 1.20, "pitch": -0.50, "yaw": 45.30,
      "temp": 32.1
    },
    "odometry": { "x": null, "y": null },
    "control": {
      "steer": null, "steerPct": 0,
      "servoUs": 2040, "thrPct": 50, "escUs": 1495
    },
    "battery": { "v": null, "pct": null },
    "system": {
      "mstate": "ARMED",
      "armPct": 100,
      "disarmReason": "none",
      "mode": "MANUAL",
      "auto-mode-enabled": false,
      "auto-mode-status": "single-car",
      "emergency": false
    },
    "wireless": { "wifiOk": true, "rssi": -62 },
    "limits": { "throttleLimit": 100, "autoThrottle": 20 }
  }
}
```

| Alan | Açıklama |
|------|----------|
| `config.servo/esc.{minUs,neutralUs,maxUs}` | Derleme zamanı PWM limitleri (µs) |
| `sonar.lLv / rLv` | Yakınlık seviyesi 1–5 (1=en yakın/kritik, 5=açık) |
| `sonar.lFwd / lLat` | İleri + yanal mesafe bileşenleri (cm), 30° montaj açısından |
| `sonar.thresholds` | Aktif sonar eşikleri (runtime, `/set-prox` ile değişir) |
| `lidar.cm` | TF-Luna mesafesi (cm); `ok=false` ise geçersiz |
| `lidar.thresholds` | Aktif LiDAR eşikleri (`/set-lidar` ile değişir) |
| `imu.ax/ay/az` | İvme (g); düz zeminde `az ≈ +1.0` |
| `imu.roll/pitch/yaw` | Complementary filter çıkışı (°) |
| `control.steerPct` | Servo yüzdesi −100…+100; `thrPct` 0…100 |
| `system.mstate` | `"DISARMED"` / `"ARMED"` / `"EMERGENCY"` |
| `system.disarmReason` | `"none"`, `"btn_manual"`, `"web_manual"`, `"emergency_stop"`, `"wall_detected"` |
| `system.mode` | `"MANUAL"` / `"AUTO"` |
| `system.auto-mode-status` | `"single-car"` / `"multi-car"` |
| `limits.throttleLimit` | **Firmware'de sabit `100` döner** (runtime `throttleLimitPct` burada raporlanmaz) |
| `limits.autoThrottle` | `AUTO_THROTTLE_PCT` (statik) |

---

### `GET /api/docs`

Swagger UI. CDN'den yükler — internet gerektirir.

```
http://esp.local:5000/api/docs
```

---

### `GET /api/openapi.json`

OpenAPI 3.0 spec (JSON). Swagger UI buradan yükler.

**Response:** `200 application/json`

---

## POST Endpoints

---

### `POST /arm`

Motoru ARM eder. `DISARMED → ARMED`.

**Kural:** `EMERGENCY` durumundaysa önce `/disarm` çağrılmalı.

| HTTP | Durum | Body |
|------|-------|------|
| 200 | Başarılı | `{"ok":true,"state":"ARMED"}` |
| 403 | E-STOP aktif | `{"ok":false,"msg":"E-STOP active, disarm first"}` |

```bash
curl -X POST http://esp.local:5000/arm
```

---

### `POST /disarm`

Her durumdan `DISARMED`'a geçer. Servo ve ESC nötr'e alınır. `disarmReason` = `"web_manual"`.

**Response:** `200` → `{"ok":true,"state":"DISARMED"}`

```bash
curl -X POST http://esp.local:5000/disarm
```

---

### `POST /estop`

Acil durdurma. `EMERGENCY`'ye geçer, motor anında nötr. Çıkış için `/disarm` zorunlu (otomatik ARM'a dönüş yok; AUTO modundaki duvar kurtarma istisnası hariç).

**Response:** `200` → `{"ok":true,"state":"EMERGENCY"}`

> Fiziksel E-STOP butonu (GPIO 18) **hardware interrupt** ile çalışır — `loop()` bloke olsa bile anında tetiklenir.

```bash
curl -X POST http://esp.local:5000/estop
```

---

### `POST /mode`

Çalışma modunu değiştirir.

| Parametre | Tip | Zorunlu | Değer |
|-----------|-----|---------|-------|
| `mode` | string | evet | `auto` \| `manual` |

`manual`'a geçişte ARMED ise servo/ESC nötr'e alınır.

| HTTP | Body |
|------|------|
| 200 | `{"ok":true,"mode":"AUTO"}` |
| 400 | `{"ok":false,"msg":"no mode param"}` |

```bash
curl -X POST "http://esp.local:5000/mode?mode=auto"
```

---

### `POST /auto-version`

AUTO modunun sürümünü seçer.

| Parametre | Tip | Zorunlu | Değer |
|-----------|-----|---------|-------|
| `version` | string | evet | `single` \| `multi` |

- `single` — yalnızca ultrasonik + LiDAR tabanlı `autoControl` (RL kapalı).
- `multi` — hareketli engel (`isDynamic()`) tespitinde RL politikası devreye girer; statik/uzak engelde `autoControl`'e düşer.

`single`'a geçiş RL durumunu ve LiDAR geçmişini sıfırlar.

| HTTP | Body |
|------|------|
| 200 | `{"ok":true,"autoVersion":"multi-car"}` |
| 400 | `{"ok":false,"msg":"no version param"}` |

```bash
curl -X POST "http://esp.local:5000/auto-version?version=multi"
```

---

### `POST /control`

Direksiyon (servo) ve gaz (ESC) yazar. **Yalnızca `ARMED` + `MANUAL` modunda çalışır.**

| Parametre | Tip | Zorunlu | Açıklama |
|-----------|-----|---------|----------|
| `steer` | int (µs) | hayır | Servo PWM. `steerLimitPct` ile kırpılır. |
| `thr` | int (µs) | hayır | ESC PWM. `[1430, 1560]` aralığına kırpılır. |

**Sunucu tarafı sınır hesabı:**
```
steer → [ 2040 - (2040-1820)*steerLimitPct/100 ,  2040 + (2260-2040)*steerLimitPct/100 ]
thr   → [ 1430 , 1560 ]
```

> **Not:** `thr`, `throttleLimitPct` ile sınırlanmaz — yalnızca tam ESC aralığına (`1430…1560`) kırpılır. Yazıldıktan sonra `manualSafety()` çalışır: sonar `≤ sonarCloseCm` veya LiDAR `≤ 25 cm` ise gaz nötr'e zorlanır.

| HTTP | Durum | Body |
|------|-------|------|
| 200 | Yazıldı | `{"ok":true,"servoUs":1900,"escUs":1480}` |
| 403 | ARM değil | `{"ok":false,"msg":"ARM degil"}` |
| 403 | AUTO modunda | `{"ok":false,"msg":"AUTO mode"}` |

```bash
# Düz, hafif ileri (1495=nötr, <1495 ileri)
curl -X POST "http://esp.local:5000/control?steer=2040&thr=1470"

# Sola dön, gaz değişmeden
curl -X POST "http://esp.local:5000/control?steer=1900"
```

---

### `POST /set`

Runtime limit güncelleme (yeniden yükleme gerekmez). En az bir parametre zorunlu.

| Parametre | Tip | Aralık | Açıklama |
|-----------|-----|--------|----------|
| `thr_limit` | int | 0–100 | `throttleLimitPct`'i günceller. **Uyarı:** bu değer şu an `/control`'de uygulanmıyor (yalnızca saklanır). |
| `steer_limit` | int | 0–100 | `steerLimitPct` — `/control` direksiyon kırpmasını doğrudan etkiler. |

| HTTP | Body |
|------|------|
| 200 | `{"ok":true,"throttleLimit":40,"steerLimit":80}` |
| 400 | `{"ok":false,"msg":"no param"}` |

```bash
curl -X POST "http://esp.local:5000/set?steer_limit=60"
curl -X POST "http://esp.local:5000/set?thr_limit=40&steer_limit=60"
```

---

### `POST /set-prox`

Sonar yakınlık eşiklerini günceller. **Yalnızca DISARMED + MANUAL modunda.** Beş parametre de zorunlu.

| Parametre | Açıklama | Kural |
|-----------|----------|-------|
| `close` | Acil dur + tam kır eşiği (cm) | `≥ 10` |
| `lv1` | Seviye 1 üst sınırı (cm) | `≥ 25` |
| `lv2` | Seviye 2 üst sınırı (cm) | `lv2 − lv1 ≥ 20` |
| `lv3` | Seviye 3 üst sınırı (cm) | `lv3 − lv2 ≥ 20` |
| `lv4` | Seviye 4 üst sınırı (cm) | `lv4 − lv3 ≥ 20` |

| HTTP | Body |
|------|------|
| 200 | `{"ok":true,"proxThresholds":{"lv1":35,"lv2":60,"lv3":85,"lv4":115,"close":10}}` |
| 400 | Doğrulama hatası — ör. `{"ok":false,"msg":"Each consecutive level must have >= 20 cm gap (lv1<lv2<lv3<lv4)"}` |
| 403 | ARMED veya AUTO — `{"ok":false,"msg":"Cannot change thresholds while ARMED"}` |

```bash
curl -X POST "http://esp.local:5000/set-prox?lv1=35&lv2=60&lv3=85&lv4=115&close=10"
```

---

### `POST /set-lidar`

LiDAR hız/dur eşiklerini günceller. **Yalnızca DISARMED + MANUAL modunda.** İki parametre de zorunlu.

| Parametre | Açıklama | Kural |
|-----------|----------|-------|
| `stop` | EMERGENCY tetik mesafesi (cm) | `≥ 40` |
| `slow` | Yavaşlama eşiği (cm) | `≥ stop × 2` |

| HTTP | Body |
|------|------|
| 200 | `{"ok":true,"lidarThresholds":{"stop":50,"slow":100}}` |
| 400 | Doğrulama hatası — ör. `{"ok":false,"msg":"slow must be >= 2x stop (min 100 cm)"}` |
| 403 | ARMED veya AUTO — `{"ok":false,"msg":"Cannot change thresholds in AUTO mode"}` |

```bash
curl -X POST "http://esp.local:5000/set-lidar?stop=50&slow=100"
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
               │   POST /estop  ·  AUTO duvar │  │
               │   ┌──────────────────────────┘  │
               │   │  (fiziksel btn GPIO 18, ISR) │
               │   ▼                              │
               │ EMERGENCY ◄──────────────────────┘
               │     │
               └─────┘  POST /disarm (→ DISARMED, ARMED değil)
```

- `EMERGENCY`'den çıkış: yalnızca `/disarm`.
- `ARMED` iken `/arm` çağırmak → hata vermez, state değişmez.
- AUTO modunda duvar tespiti EMERGENCY'ye geçirir; 2 sn sonra LiDAR > 50 cm ise **otomatik** yeniden ARMED olur. EMERGENCY'de servo ilk 1 sn kaçış manevrası için serbest, sonra nötr; ESC sürekli nötr.

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
post("/control?steer=2040&thr=1470")
post("/set?steer_limit=70")
post("/disarm")
```
