/*
 * ESC Direct Test — ESP32-S3
 * ─────────────────────────────────────────────────────────────────────────────
 * Güvenlik önlemi YOK. Sadece ESC'nin çalışıp çalışmadığını test eder.
 * WiFi yok, watchdog yok, ARM/DISARM yok.
 *
 * KABLOLAMA:
 *   ESC sinyal fişi (3 pin):
 *     Beyaz/Sarı (sinyal)  →  ESP32 GPIO 5
 *     Siyah (GND)          →  ESP32 GND
 *     Kırmızı (BEC 5V)     →  BAĞLAMA (USB varken)
 *
 *   Batarya → ESC (kalın kırmızı/siyah kablo) — ZORUNLU
 *
 * KULLANIM (Serial Monitor, 115200 baud):
 *   f  →  ileri  (+%30 throttle)
 *   b  →  geri   (-%30 throttle)  * ESC reverse destekliyorsa
 *   n  →  nötr   (dur)
 *   +  →  throttle +10%
 *   -  →  throttle -10%
 *   0  →  throttle sıfırla
 *   a  →  ESC arming (3sn nötr sinyal)
 *
 * ESC İLK AÇILIŞTA:
 *   1. Bataryayı bağla
 *   2. 'a' tuşuna bas — ESC 3 sn nötr sinyal alır, bip sesi duyulur
 *   3. Hazır, 'f' ile ileri ver
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <ESP32Servo.h>

// ---- Pin ----
const int PIN_ESC = 5;   // ESC sinyal teli buraya

// ---- PWM değerleri (µs) ----
const int ESC_NEUTRAL  = 1500;
const int ESC_MAX_FWD  = 2000;  // tam ileri
const int ESC_MAX_REV  = 1000;  // tam geri
const int STEP         = 50;    // +/- tuşu adımı

int currentUs = ESC_NEUTRAL;
Servo esc;

void writeUs(int us) {
  us = constrain(us, ESC_MAX_REV, ESC_MAX_FWD);
  esc.writeMicroseconds(us);
  currentUs = us;
  Serial.printf("[ESC] %d µs  (%.0f%%)\n", us, (us - ESC_NEUTRAL) / 5.0f);
}

void armEsc() {
  Serial.println("[ARM] Sending neutral for 3 seconds...");
  esc.writeMicroseconds(ESC_NEUTRAL);
  for (int i = 3; i > 0; i--) {
    Serial.printf("[ARM] %d...\n", i);
    delay(1000);
  }
  Serial.println("[ARM] Done — ESC should beep. Try 'f' now.");
  currentUs = ESC_NEUTRAL;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  ESP32PWM::allocateTimer(0);
  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC, ESC_MAX_REV, ESC_MAX_FWD);

  writeUs(ESC_NEUTRAL);

  Serial.println("\n=== ESC Direct Test ===");
  Serial.println("Commands: f=forward  b=backward  n=neutral  +=more  -=less  0=zero  a=arm");
  Serial.println("Run 'a' first to arm the ESC, then try 'f'.");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case 'f': writeUs(ESC_NEUTRAL + 150); break;  // ~+30% ileri
    case 'b': writeUs(ESC_NEUTRAL - 150); break;  // ~-30% geri
    case 'n': writeUs(ESC_NEUTRAL);       break;
    case '0': writeUs(ESC_NEUTRAL);       break;
    case '+': writeUs(currentUs + STEP);  break;
    case '-': writeUs(currentUs - STEP);  break;
    case 'a': armEsc();                   break;
    default:  break;
  }
}
