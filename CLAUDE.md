# CLAUDE.md — ArduinoIDE / ESP32-S3 Gömülü Kod

Bu dizin otonom RC arabanın **araç üstü yazılımını** içerir. Proje geneli: `../CLAUDE.md`. Kurulum ve pin haritası: `README.md`. Tam yol haritası: `../ReadMEs/ESP32_YOL_HARITASI.md`.

## Mimari (kısa)
- **ESP32-S3 tek kart** — Raspberry Pi yok → **online MPC yok**.
- Faz 7: offline hız profili + LQR gain-schedule + feedforward + Pure Pursuit + hız PID (optimizasyon `../Simulation/offline_design.py`'de).

## Sketch'ler (sırayla test et)
| Klasör | Faz | Not |
|--------|-----|-----|
| `Faz1_AktuatorTest` | 1 | Servo sweep + ESC, tekerler havada |
| `Faz2_WifiTeleop` | 2 | WiFi teleop + watchdog |
| `Faz3_Sensorler` | 3 | IMU + encoder ISR + pil ADC |
| `Faz4_HizPID` | 4 | Hız PID, arming korunur |
| `Faz5_PoseEKF` | 5 | UDP pose + EKF (önce `../Simulation/ekf_localization.py`) |
| `Faz6_YolTakibi` | 6 | Pure Pursuit + geofence |
| `Faz7_LQR` | 7 | LQR + `track_data.h` (önce `offline_design.py`) |

**Durum:** Faz 1–7 kodu yazıldı; **donanımda ve derlemede doğrulanmadı**. "Hazır" ≠ bitti — Faz 3 ölçülmeden 5/6/7'ye güvenme.

## Güvenlik (her motor-süren sketch)
- Açılış **DISARMED**; gaz yalnız ARMED.
- Watchdog: komut/pose timeout → motor nötr + DISARMED kilit (otomatik devam yok).
- Loop/handler'da `delay()` yok (E-STOP anında).
- Faz 6–7: geofence + pose timeout.
- `THROTTLE_LIMIT_PCT=30`. İlk testler **tekerler havada**. Ortak GND; ESP32 motor/servo **beslemez**.

## `// CONFIRM:` — donanımda ölç
`ENC_PPR`, `WHEEL_DIAMETER_M`, **`GEAR_RATIO`** (encoder yeri hızı değiştirir), `VDIV_RATIO`. Faz 7: `e_cte`/`e_psi` işareti ilk testte tersse çevir.

## Kütüphane
- Board: ESP32 Arduino → **ESP32S3 Dev Module**.
- Elle: **`ESP32Servo`** (Kevin Harrington). `WiFi`, `WebServer`, `Wire` board paketinde.

## Pin referansı (tüm fazlar)
| Sinyal | GPIO |
|--------|------|
| Servo | 4 |
| ESC | 5 |
| Encoder A/B | 6 / 7 |
| IMU SDA/SCL | 8 / 9 |
| Pil ADC | 1 |

## Simülasyon eşlikçileri (`../Simulation/`)
- `ekf_localization.py` (Faz5), `bicycle_pure_pursuit.py` (Faz6), `offline_design.py` → `Faz7_LQR/track_data.h`, `fake_pose_sender.py` (UDP test).
- Python: `../.capstoneenv/bin/python` (stdlib only).

## Sonraki adımlar
1. `arduino-cli` + esp32 core → tüm sketch'leri derle.
2. Faz 1 → 2 → … donanımda; CONFIRM sabitlerini ölç.
3. Pist ölçüsüne göre `offline_design.py` → `track_data.h` yenile.
