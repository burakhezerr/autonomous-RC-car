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
| `Faz2_WifiTeleop` | 2 | WiFi'den elle gaz+direksiyon + fail-safe watchdog | ✅ Hazır |
| `Faz3_Sensorler` | 3 | IMU (I²C) + encoder (interrupt) + pil voltajı okuma | ✅ Hazır |
| `Faz4_HizPID` | 4 | Encoder geri besleme → hız PID → ESC | ✅ Hazır |
| `Faz5_PoseEKF` | 5 | UDP pose alımı + IMU/encoder füzyonu (EKF) | ✅ Hazır |
| `Faz6_YolTakibi` | 6 | Pure Pursuit ile yol takibi | ✅ Hazır |
| `Faz7_LQR` | 7 | LQR + offline hız profili (MPC alternatifi) | ✅ Hazır |

> **Önemli:** "Hazır" = kod yazıldı, ama **donanımda doğrulanmadı** (arduino-cli yok). Test **sıralı/alttan-üste** yapılır: Faz 3 sensörleri gerçek donanımda doğrulanmadan Faz 5 EKF'ine, o doğrulanmadan Faz 6/7'ye güvenme. Faz 5 (EKF) ve Faz 7 (LQR + hız profili) matematiği önce `../Simulation/` altında Python ile doğrulandı, sonra C'ye port edildi.

### Faz 5 & 7 — Offline/Python eşlikçileri (`../Simulation/`)
| Dosya | Ne işe yarar |
|---|---|
| `ekf_localization.py` | Faz 5 EKF referansı — sentetik veriyle doğrulandı (RMS ~5 cm) |
| `fake_pose_sender.py` | Faz 5 testi: gerçek tracker yokken UDP'den sahte pose gönderir |
| `offline_design.py` | Faz 7: hız profili + LQR kazançlarını hesaplar → `Faz7_LQR/track_data.h` üretir |
| `bicycle_pure_pursuit.py` | Faz 6 referansı — Pure Pursuit (kararlı durum ~5.8 cm) |

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

## 6. Fiziksel Kablolama Rehberi

> Bu bölüm "hangi kabloyu nereye takacağım?" sorusuna adım adım cevap verir.  
> Faz 1–2 için Servo + ESC yeterli. Faz 3+ için IMU, encoder ve voltaj bölücü eklenir.

---

### 6.1 Güç Mimarisi (önce oku)

```
LiPo/Akü
   │
   ├──► ESC (motor gücü — doğrudan)
   │      └── BEC çıkışı (5 V, kırmızı tel) ──► Servo VCC
   │                                         └──► ESP32 5 V pini  ← opsiyonel
   │
   └── ORTAK GND ──► ESC GND ──► Servo GND ──► ESP32 GND ──► IMU GND ──► Encoder GND
```

**Kural:** ESP32 yalnızca **sinyal + GND** verir, akım çekmez.  
Servo ve motor gücünü ESP32'den **kesinlikle alma** — BEC veya ayrı regülatörden besle.

---

### 6.2 Servo (Direksiyon) → GPIO 4

| Servo kablosu | Renk (standart) | ESP32-S3'e bağla |
|---|---|---|
| Sinyal | Sarı / Turuncu | **GPIO 4** |
| VCC | Kırmızı | ESC BEC 5 V çıkışı (ESP32 değil) |
| GND | Kahverengi / Siyah | **GND** (ortak hat) |

```
Servo fiş (3 pin)
  [GND]──────────────────────► ESP32 GND
  [VCC]──────────────────────► BEC 5V
  [SIG]──────────────────────► GPIO 4
```

---

### 6.3 ESC (Gaz / Motor) → GPIO 5

RC ESC'nin kumanda tarafında 3 pinli bir sinyal fişi bulunur:

| ESC sinyal fişi | Renk (standart) | ESP32-S3'e bağla |
|---|---|---|
| Sinyal | Sarı / Turuncu | **GPIO 5** |
| BEC + | Kırmızı | ESP32 **5V** pinine bağlayabilirsin (opsiyonel — USB varsa gerekmez) |
| GND | Siyah | **GND** (ortak hat) |

> ESC'nin motor tarafındaki kalın kablolar (akü + ve akü −) direkt LiPo'ya gider, ESP32'ye gelmiyor.

---

### 6.4 IMU (MPU-6050) → GPIO 8 / 9 *(Faz 3+)*

MPU-6050 breakout modülü I²C ile bağlanır:

| MPU-6050 pini | ESP32-S3'e bağla |
|---|---|
| VCC | **3.3 V** (ESP32'nin 3V3 pini) |
| GND | **GND** |
| SDA | **GPIO 8** |
| SCL | **GPIO 9** |
| AD0 | GND'e bağla (I²C adresi 0x68 olur) |
| INT | bağlanmayabilir (polling kullanıyoruz) |

> MPU-6050 modülünde genellikle dahili 3.3 V regülatör vardır; 5 V'a da bağlanabilir — modülün datasheeti'ne bak.

---

### 6.5 Encoder → GPIO 6 / 7 *(Faz 3+)*

Kuadratür encoder (A/B kanalı) ya da tek kanallı Hall sensörü:

| Encoder pini | ESP32-S3'e bağla |
|---|---|
| VCC | **3.3 V** |
| GND | **GND** |
| Kanal A | **GPIO 6** (interrupt) |
| Kanal B | **GPIO 7** (interrupt, kuadratür için) |

> Encoder tekerlek miline mi yoksa motor miline mi bağlı? `GEAR_RATIO` sabitini buna göre ayarla — `// CONFIRM:` yorumuna bak.

---

### 6.6 Pil Voltaj Bölücü → GPIO 1 *(Faz 3+)*

ESP32 ADC girişleri en fazla **3.3 V** okuyabilir. 2S LiPo (~8.4 V) için voltaj bölücü şart:

```
LiPo (+) ──┬── R1 (10 kΩ) ──┬── R2 (3.3 kΩ) ── GND
           │                 │
           └─── (akü güç)    └──► GPIO 1  (ADC)
```

**Bölücü oranı:** `V_adc = V_bat × R2 / (R1 + R2)`  
→ 10 kΩ + 3.3 kΩ ile 8.4 V → **2.1 V** (güvenli).  
Sketch'teki `VDIV_RATIO` sabitini `(R1+R2)/R2` formülüyle hesapla ve gir: `(10+3.3)/3.3 ≈ 4.03`.

---

### 6.7 Genel Kontrol Listesi (ilk bağlantı öncesi)

- [ ] Tüm GND'ler birbirine bağlı (ESP32 ↔ ESC ↔ servo ↔ IMU ↔ encoder)
- [ ] Servo VCC → BEC 5V (ESP32 değil)
- [ ] Motor güç kabloları polarite doğru (ESC ↔ akü)
- [ ] IMU 3.3V, encoder 3.3V aldığından emin ol
- [ ] Voltaj bölücü dirençleri lehimli, GPIO1 3.3V'ı geçmiyor
- [ ] USB bağlı, Faz 1 sketch yüklü, **tekerler havada**, Serial Monitor açık

---

## 7. Güvenlik (her sketch için geçerli)

- **Faz 1-2:** araba sehpa üstünde, **tekerler havada**.
- **Ortak GND zorunlu:** ESP32 GND ↔ ESC GND ↔ servo GND.
- **Servo/motoru ESP32'den besleme:** ESP32 yalnız sinyal + GND verir; güç ESC BEC / bataryadan.
- **Watchdog:** Faz 2'den itibaren komut/WiFi kopunca motor 0.
- **LiPo:** safe-bag'de şarj, voltaj izle.

---

**İlişkili:** `../ReadMEs/ESP32_YOL_HARITASI.md`
 (yol haritası), `../Simulation/bicycle_pure_pursuit.py` (algoritma simülasyonu), `../ReadMEs/ONBOARD_MIMARI_SECIMI.md` (donanım gerekçesi).
