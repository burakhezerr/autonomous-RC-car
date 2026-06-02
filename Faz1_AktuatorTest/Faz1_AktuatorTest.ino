/*
 * ============================================================================
 *  Faz 1 — Aktuator Test  (ESP32-S3)
 *  Autonomous RC Car Racing Platform / Kontrol Katmani
 * ============================================================================
 *
 *  AMAC:
 *    Kendi kodunla direksiyon servosunu ve motor ESC'sini kontrol edebildigini
 *    dogrulamak. Bu, projenin "yuruyen iskelet" baslangici. Bu calismadan
 *    Kalman/PID/LQR'a gecmek anlamsiz.
 *
 *  ----------------------------------------------------------------------------
 *  >>> GUVENLIK — OKUMADAN CALISTIRMA <<<
 *  ----------------------------------------------------------------------------
 *    1) TEKERLER HAVADA OLSUN. Araci bir kutu/sehpa uzerine koy, tekerler
 *       yere degmesin. Motor testinde araba masadan firlayabilir.
 *    2) Motor varsayilan olarak DURUR (DISARMED). Kod acilista motoru
 *       DONDURMEZ. Donme sadece sen klavyeden komut verince, kisa sureli olur.
 *    3) Elini ESC guc anahtarinda / batarya fisinde tut. Beklenmedik harekette
 *       gucu kes.
 *
 *  ----------------------------------------------------------------------------
 *  >>> KABLOLAMA — EN SIK HATALAR BURADA <<<
 *  ----------------------------------------------------------------------------
 *    A) ORTAK GROUND (GND) ZORUNLU:
 *         ESP32 GND  <--->  ESC GND  <--->  Servo GND  (hepsi ayni hatta)
 *       Ortak GND yoksa PWM sinyalinin referansi olmaz -> "hicbir sey tepki
 *       vermiyor / rastgele oynuyor" sikayetinin 1 numarali sebebi budur.
 *
 *    B) SERVO/MOTOR GUCUNU ESP32'DEN ALMA:
 *         ESP32 sadece SINYAL (GPIO) + GND verir.
 *         Servo ve motor gucu ESC'nin BEC cikisindan veya bataryadan gelir.
 *         (Tipik olarak ESC'nin 5V BEC'i ESP32'yi besler, tersi DEGIL.)
 *         Servo stall akimi ESP32'nin 3.3V/5V pinini yakar veya brown-out yapar.
 *
 *    C) ESP32-S3 GPIO sinyali 3.3V mantik seviyesidir. Neredeyse tum hobi
 *       servo/ESC'leri bunu sorunsuz okur — bu yuzden burada sorun arama.
 *
 *    D) "ESC" ve "speed controller" senin listende ayri gozukuyor; standart bir
 *       RC arabada bunlar AYNI parcadir (motor hiz kontrolcusu). Bu kod tek bir
 *       ESC + tek PWM kanali varsayar. ESC'den cikan, servo-tipi 3'lu (sinyal/
 *       +/GND) konnektor = gaz sinyalidir. Servoyu ondan ayirt et.
 *
 *  ----------------------------------------------------------------------------
 *  >>> PIN BAGLANTILARI <<<
 *  ----------------------------------------------------------------------------
 *    Servo sinyal  -> GPIO 4   (turuncu/beyaz tel)
 *    ESC   sinyal  -> GPIO 5   (gaz sinyali)
 *    GND           -> ortak GND (yukaridaki A maddesi)
 *
 *  ----------------------------------------------------------------------------
 *  >>> KURULUM <<<
 *  ----------------------------------------------------------------------------
 *    1) Arduino IDE -> Boards Manager -> "esp32" (Espressif) paketini kur.
 *    2) Library Manager -> "ESP32Servo" (Kevin Harrington) kur.
 *    3) Kart: "ESP32S3 Dev Module" sec. Baud: 115200.
 *
 *  ----------------------------------------------------------------------------
 *  >>> KULLANIM (Serial Monitor, 115200 baud) <<<
 *  ----------------------------------------------------------------------------
 *    Komutlar:
 *      s  -> Servo SWEEP testi (sol<->sag suprur).  GUVENLI, motor donmez.
 *      c  -> Servoyu merkeze al.
 *      a  -> Motoru ARM et (notr sinyali ~3 sn gonder, ESC bip oter).
 *      g  -> Motoru kisa sure (~1 sn) HAFIF dondurup notr'a don. (once 'a' yap)
 *      x  -> Her seyi durdur / DISARM (acil).
 *
 *    NOT (motor): Fircasiz motor cok dusuk gazda DUZGUN donmez; teklerek
 *    (cogging) baslar. Bu normaldir, hata degildir.
 *    NOT (arming): ESC modeline gore degisir. Sadece bip oter, donmezse
 *    muhtemelen "throttle range calibration" gerekir — ESC kilavuzuna bak.
 * ============================================================================
 */

#include <ESP32Servo.h>

// ---- Pin tanimlari ----
const int PIN_SERVO = 4;
const int PIN_ESC   = 5;

// ---- PWM darbe genislikleri (mikrosaniye) ----
// Standart hobi araligi: 1000 (min) - 1500 (notr) - 2000 (maks)
const int SERVO_MIN    = 1000;  // tam sol
const int SERVO_CENTER = 1500;  // duz
const int SERVO_MAX    = 2000;  // tam sag

const int ESC_NEUTRAL  = 1500;  // cift yonlu (forward/reverse) RC ESC notru
const int ESC_TEST_FWD = 1560;  // COK hafif ileri (sadece test). Yukseltme!

Servo servo;
Servo esc;

bool motorArmed = false;

void setServo(int us) { servo.writeMicroseconds(us); }
void setEsc(int us)   { esc.writeMicroseconds(us); }

void printMenu() {
  Serial.println(F("\n================ Faz 1 Aktuator Test ================"));
  Serial.println(F(" TEKERLER HAVADA MI? Degilse simdi kaldir."));
  Serial.println(F(" s = servo sweep (guvenli) | c = servo merkez"));
  Serial.println(F(" a = motoru ARM et         | g = motoru kisa nudge"));
  Serial.println(F(" x = ACIL DURDUR / disarm"));
  Serial.print  (F(" Motor durumu: "));
  Serial.println(motorArmed ? F("ARMED") : F("DISARMED (guvenli)"));
  Serial.println(F("====================================================="));
}

void servoSweep() {
  Serial.println(F("[servo] sweep basliyor (sol<->sag)..."));
  for (int us = SERVO_MIN; us <= SERVO_MAX; us += 10) { setServo(us); delay(15); }
  for (int us = SERVO_MAX; us >= SERVO_MIN; us -= 10) { setServo(us); delay(15); }
  setServo(SERVO_CENTER);
  Serial.println(F("[servo] sweep bitti, merkeze alindi."));
}

void armMotor() {
  Serial.println(F("[esc] ARM: notr sinyali 3 sn gonderiliyor (bip bekle)..."));
  setEsc(ESC_NEUTRAL);
  delay(3000);
  motorArmed = true;
  Serial.println(F("[esc] ARMED. Artik 'g' ile kisa nudge yapabilirsin."));
}

void motorNudge() {
  if (!motorArmed) {
    Serial.println(F("[esc] ONCE 'a' ile ARM et!"));
    return;
  }
  Serial.println(F("[esc] NUDGE: ~1 sn hafif ileri, sonra notr."));
  setEsc(ESC_TEST_FWD);
  delay(1000);
  setEsc(ESC_NEUTRAL);
  Serial.println(F("[esc] notr'a donuldu."));
}

void disarmAll() {
  setEsc(ESC_NEUTRAL);
  setServo(SERVO_CENTER);
  motorArmed = false;
  Serial.println(F("[ACIL] Motor DISARMED, servo merkez. Guvenli."));
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // ESP32 PWM zamanlayicilarini ayir (ESP32Servo gereksinimi)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  servo.setPeriodHertz(50);                 // hobi servo standardi 50 Hz
  servo.attach(PIN_SERVO, SERVO_MIN, SERVO_MAX);

  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC, 1000, 2000);

  // GUVENLI BASLANGIC: motor notr (durur), servo merkez. Motor ARMED degil.
  disarmAll();

  printMenu();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 's': servoSweep();  break;
      case 'c': setServo(SERVO_CENTER); Serial.println(F("[servo] merkez.")); break;
      case 'a': armMotor();    break;
      case 'g': motorNudge();  break;
      case 'x': disarmAll();   break;
      case '\n': case '\r': break;          // satir sonlarini yut
      default:  printMenu();   break;
    }
  }
}
