# ArduinoIDE — ESP32-S3 Gömülü Kod

Bu dizin, otonom RC araba projesinin **araç üstü (onboard) ESP32-S3 yazılımını** içerir. Her geliştirme fazı kendi sketch klasöründe durur. Donanım yönü ve neden ESP32-S3 tek kart seçildiği için bkz. `../ReadMEs/ESP32_YOL_HARITASI.md`.

> **Çalışma felsefesi:** "Yürüyen iskelet" — en basit çalışan döngüyü kur, sonra her fazda **bir** katman ekle. Sketch'ler bu sırayı izler.

---

## 1. Tek Seferlik Kurulum

1. **Arduino IDE 2.x** kur (arduino.cc).
2. **ESP32 board paketi:**
   - `File → Preferences → Additional Board Manager URLs` alanına ekle:
     `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   - `Tools → Board → Boards Manager` → **"esp32" (Espressif)** kur.
3. **Kütüphaneler** (`Tools → Manage Libraries`):
   - `ESP32Servo` (Kevin Harrington) — servo/ESC PWM için.
   - *(Faz 3+)* `Adafruit MPU6050` veya `Adafruit BNO055` — IMU için.
4. **Kart seçimi:** `Tools → Board → ESP32 Arduino → ESP32S3 Dev Module`.
5. **Port:** ESP32-S3'ü USB ile bağla, `Tools → Port` altından seç.
6. **Baud:** Serial Monitor sağ alt → **115200**.

> İlk yüklemede port görünmezse: USB-veri kablosu kullandığından emin ol (bazı kablolar sadece şarj). Gerekirse karttaki BOOT tuşunu basılı tutup yükle.

---

## 2. Klasör Konvansiyonu

Arduino IDE kuralı: **her sketch kendi klasöründe olmalı ve klasör adı `.ino` dosyasıyla aynı olmalı.**

```
ArduinoIDE/
├── README.md                      # (bu dosya)
├── Faz1_AktuatorTest/
│   ├── Faz1_AktuatorTest.ino
│   └── README.md                  # faza özel hızlı başlangıç
├── Faz2_WifiTeleop/               # (sonraki)
│   └── ...
└── ...
```

Yeni faza başlarken: aynı isimde klasör + `.ino` aç, başına amaç/güvenlik/pin yorum bloğu koy (Faz 1'deki gibi).

---

## 3. Sketch İndeksi

| Sketch | Faz | Amaç | Durum |
|---|---|---|---|
| `Faz1_AktuatorTest` | 1 ⭐ | Servo sweep + motor ESC PWM testi (tekerler havada) | ✅ Hazır |
| `Faz2_WifiTeleop` | 2 | WiFi'den elle gaz+direksiyon + fail-safe watchdog | ⏳ Planlı |
| `Faz3_Sensorler` | 3 | IMU (I²C) + encoder (interrupt) + pil voltajı okuma | ⏳ Planlı |
| `Faz4_HizPID` | 4 | Encoder geri besleme → hız PID → ESC | ⏳ Planlı |
| `Faz5_PoseEKF` | 5 | UDP pose alımı + IMU/encoder füzyonu (EKF) | ⏳ Planlı |
| `Faz6_YolTakibi` | 6 | Pure Pursuit / Stanley ile yol takibi | ⏳ Planlı |
| `Faz7_LQR` | 7 | LQR + offline hız profili (MPC alternatifi) | ⏳ Planlı |

---

## 4. Fazlar — Açıklamalar

Her faz, bir öncekinin üstüne **çalışan** bir katman ekler. Tam detay (sık hatalar dahil) için: `../ReadMEs/ESP32_YOL_HARITASI.md`.

### Faz 0 — Tanıma & Hazırlık
ESC/servo sinyal kablolarını bul, Arduino IDE + board paketi + `ESP32Servo` kur, eksik parçaları sipariş et.
**Başarı:** IDE derleme yapıyor, sinyal yolları belli.

### Faz 1 ⭐ — Aktüatör Testi *(`Faz1_AktuatorTest/`)*
Kendi kodunla servoyu süpürt, motoru kısa süre döndür (tekerler havada). PWM (1000–1500–2000 µs), ESC arming.
**Başarı:** `s` ile servo süpürüyor; `a`+`g` ile motor dönüyor.

### Faz 2 — WiFi Teleop *(`Faz2_WifiTeleop/`)*
ESP32-S3 WiFi'den gaz+direksiyon alır, Faz 1 PWM çıkışına bağlar. **Watchdog:** komut/bağlantı kopunca motor 0.
**Başarı:** Telefonla araba elle sürülüyor; WiFi kesilince duruyor.

### Faz 3 — Sensörler *(`Faz3_Sensorler/`)*
IMU I²C ile (~200 Hz), encoder interrupt ile pulse sayımı → hız, pil voltajı ADC.
**Başarı:** Serial'de IMU + hız + voltaj canlı akıyor.

### Faz 4 — Hız PID *(`Faz4_HizPID/`)*
İlk gerçek kapalı döngü: encoder hızı → `u = Kp·e + Ki·∫e + Kd·de/dt` → ESC. Ayarı önce simülasyonda yap.
**Başarı:** Hedef hız verince araba o hızda sabitleniyor.

### Faz 5 — Pose + EKF *(`Faz5_PoseEKF/`)*
Offboard IR kameradan UDP ile pose; IMU/encoder ile füzyon (EKF, 6-state). Pose'u **ayrı core/thread**'de al — kontrolü kilitlemesin.
**Başarı:** ESP32 `(x, y, ψ, v)` durumunu güncel tutuyor, WiFi gecikmesinde dead-reckoning yapıyor.

### Faz 6 — Yol Takibi *(`Faz6_YolTakibi/`)*
Pure Pursuit `δ = atan2(2·L·sin(α), ld)` veya Stanley ile referans yolu takip. Önce `../Simulation/bicycle_pure_pursuit.py`'de ayarlanır.
**Başarı:** Pisti <10 cm sapma ile tamamlıyor.

### Faz 7 — LQR + Hız Profili *(`Faz7_LQR/`)* — MPC alternatifi
Offline hız profili (`v_max = sqrt(μ·g/κ)`) + feedforward (`δ_ff = atan(L·κ)`) + LQR (`u = -K·x`, gain scheduling) + friction-circle. Optimizasyon laptopta, araçta sadece takip.
**Başarı:** Yarış çizgisini optimal hıza yakın tamamlıyor.

---

## 5. Ortak Pin Haritası

Tüm sketch'lerde tutarlı kalması için referans atama (faz ilerledikçe genişler):

| Sinyal | GPIO | Not |
|---|---|---|
| Servo (direksiyon) | 4 | PWM, 50 Hz |
| ESC (gaz) | 5 | PWM, 50 Hz |
| IMU SDA / SCL | 8 / 9 | I²C *(Faz 3)* |
| Encoder A / B | 6 / 7 | interrupt'lı *(Faz 3)* |
| Pil voltaj ADC | 1 | bölücü ile *(Faz 3)* |

> Strapping pinlerinden (ör. GPIO0, 45, 46) ve USB pinlerinden kaçın. Yukarıdaki atamalar tipik ESP32-S3 DevKit için güvenli; kendi kartının pinout'unu doğrula.

---

## 6. Güvenlik (her sketch için geçerli)

- **Faz 1-2:** araba sehpa üstünde, **tekerler havada**.
- **Ortak GND zorunlu:** ESP32 GND ↔ ESC GND ↔ servo GND.
- **Servo/motoru ESP32'den besleme:** ESP32 yalnız sinyal + GND verir; güç ESC BEC / bataryadan.
- **Watchdog:** Faz 2'den itibaren komut/WiFi kopunca motor 0.
- **LiPo:** safe-bag'de şarj, voltaj izle.

---

**İlişkili:** `../ReadMEs/ESP32_YOL_HARITASI.md` (yol haritası), `../Simulation/bicycle_pure_pursuit.py` (algoritma simülasyonu), `../ReadMEs/ONBOARD_MIMARI_SECIMI.md` (donanım gerekçesi).
