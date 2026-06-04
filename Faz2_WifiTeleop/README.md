# Faz 2 — WiFi Teleop

**Amaç:** ESP32-S3 bir WiFi ağına bağlanır, gömülü bir web arayüzü sunar; tarayıcıdan (telefon/laptop) gaz + direksiyon ile arabayı **elle** sürersin. Faz 1 serial komutlarının web karşılığı + gerçek teleop.

> Tüm detay sketch içindeki yorum bloğunda. Bu dosya hızlı başvuru.

---

## ⚠ Güvenlik (Faz 2'ye özel)
- **Dead-man kontrol:** Tarayıcıda gaz/direksiyon **bırakılınca sıfıra/merkeze döner**. Sürmek için basılı tut.
- **Fail-safe watchdog:** ~500 ms komut gelmezse (WiFi koptu / sekme arka plana alındı / telefon uyudu) motor nötr olur **ve sistem DISARMED'a kilitlenir**. Bağlantı geri gelse bile motor kendiliğinden devam etmez → **tekrar ARM** gerekir.
- **Gaz limiti:** `THROTTLE_LIMIT_PCT` (varsayılan **%30**) — tam slider ≠ tam hız. İlk testlerde düşük tut.
- **Tekerler havada** ilk testte. Ortak GND + motoru/servoyu ESP32'den besleme (bkz. Faz 1).

---

## Kurulum
1. Sketch'in başındaki **`WIFI_SSID` / `WIFI_PASS`** değerlerini kendi ağınla doldur.
2. Telefon/laptop **aynı WiFi ağında** olmalı.
3. `ESP32Servo` kütüphanesi kurulu olsun (WiFi/WebServer/ESPmDNS çekirdekte dahili).
4. Yükle, Serial Monitor (115200) aç → bağlanınca **IP** ve **`http://arac.local/`** yazar.
5. Tarayıcıdan o adrese git.

> `WEB_PORT` varsayılan **80** — bu durumda tarayıcıda port yazmana gerek yok. Değiştirirsen `http://arac.local:PORT/`.

---

## Kullanım

| Kontrol | Etki |
|---|---|
| Gaz slider (dikey) | İleri/geri (yalnız ARMED iken, limitli) |
| Direksiyon slider (yatay) | Sol/sağ |
| Klavye `W`/`S` veya ↑/↓ | Gaz |
| Klavye `A`/`D` veya ←/→ | Direksiyon |
| **ARM** | ~3 sn ESC hazırlanır → sonra gaz çalışır |
| **DISARM** | Motoru pasifleştir |
| **■ E-STOP** | Anında durdur + merkeze al |

Akış: **ARM** → durum `ARMING` → ~3 sn sonra `ARMED` → gaz uygula.

---

## Sorun Giderme

| Belirti | Olası sebep |
|---|---|
| `http://arac.local/` açılmıyor | mDNS desteklemeyen ağ/cihaz → Serial'deki **IP**'yi kullan |
| Bağlanıyor ama gaz çalışmıyor | Motor `DISARMED` — önce **ARM** (ve `ARMED` olmasını bekle) |
| Araba sürerken aniden durdu | Watchdog tetiklendi (WiFi/sekme) — bu **güvenlik**, tekrar ARM |
| Sekme değişince duruyor | Tarayıcı zamanlayıcıyı yavaşlatır → watchdog. **Beklenen**, hata değil |
| WiFi'ye hiç bağlanmıyor | SSID/şifre yanlış veya 5 GHz-only ağ (ESP32 2.4 GHz ister) |

---

**Not:** arduino-cli olmadığı için bu sketch derlenerek/donanımda doğrulanmadı; mantık okuma ile doğrulandı. İlk yüklemede Serial loglarını izle.
**Sonraki:** Faz 3 — sensörler (IMU + encoder). Yol haritası: `../../ReadMEs/ESP32_YOL_HARITASI.md`.
