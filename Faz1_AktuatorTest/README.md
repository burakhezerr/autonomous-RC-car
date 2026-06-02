# Faz 1 — Aktüatör Testi

**Amaç:** Kendi kodunla direksiyon servosunu oynatmak ve motor ESC'sini döndürmek. Projenin ilk ve en kritik kilometre taşı — bu çalışmadan PID/Kalman/LQR'a geçmek anlamsız.

> Sketch içindeki yorum bloğu tüm detayı içerir. Bu dosya **hızlı başvuru** içindir.

---

## ⚠ Önce Güvenlik
- **TEKERLER HAVADA** — araba sehpa/kutu üstünde, yere değmesin.
- Motor açılışta **DİSARMED** (durur); döndürme yalnız açık komutla, kısa süreli.
- Elini ESC güç anahtarında / batarya fişinde tut.

---

## Kablolama (en sık hatalar burada)

| Bağlantı | Nereye | Not |
|---|---|---|
| Servo sinyal | GPIO **4** | turuncu/beyaz tel |
| ESC sinyal (gaz) | GPIO **5** | ESC'den çıkan servo-tipi 3'lü konnektörün sinyali |
| **Ortak GND** | ESP32 GND ↔ ESC GND ↔ servo GND | **zorunlu** — yoksa hiçbir şey çalışmaz |
| Servo/motor gücü | ESC BEC / batarya | **ESP32'den ASLA besleme** (board yanar) |

> "ESC" ve "speed controller" aynı parçadır (tek motor hız kontrolcüsü). Bu sketch tek ESC + tek PWM kanalı varsayar.

---

## Yükleme
1. Kart: **ESP32S3 Dev Module**, Port seç, Baud **115200**.
2. `ESP32Servo` kütüphanesi kurulu olmalı (bkz. `../README.md`).
3. Upload → Serial Monitor'ü aç.

## Serial Komutları (115200 baud)

| Tuş | Etki | Güvenli mi? |
|---|---|---|
| `s` | Servo sweep (sol↔sağ süpürür) | ✅ motor dönmez |
| `c` | Servoyu merkeze al | ✅ |
| `a` | Motoru ARM et (~3 sn nötr sinyal, ESC bip öter) | motor hazırlanır |
| `g` | Motoru ~1 sn hafif döndür, sonra nötr (önce `a`) | ⚠ tekerler havada! |
| `x` | Acil dur / DISARM | ✅ |

---

## Hızlı Sorun Giderme

| Belirti | Olası sebep |
|---|---|
| Hiçbir şey tepki vermiyor / rastgele oynuyor | **Ortak GND yok** (1 numaralı hata) |
| Servo titriyor ama dönmüyor | güç yetersiz — servoyu ESP32'den besliyor olabilirsin |
| Motor sadece bip ötüyor, dönmüyor | ESC "throttle range calibration" gerekir — ESC kılavuzu |
| Motor düşük gazda tekleyerek dönüyor | **normal** (fırçasız cogging), hata değil |
| Port görünmüyor | veri kablosu kullan; gerekirse BOOT tuşuyla yükle |

---

**Sonraki:** Faz 2 — WiFi teleop (`../Faz2_WifiTeleop/`). Tam yol haritası: `../../ReadMEs/ESP32_YOL_HARITASI.md`.
