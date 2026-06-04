/*
 * ============================================================================
 *  Faz 4 — Hiz PID  (ESP32-S3)
 *  Autonomous RC Car Racing Platform / Kontrol Katmani
 * ============================================================================
 *
 *  AMAC:
 *    Ilk gercek kapali dongu kontrolcusu. Encoder hizini geri besleme alip
 *    PID ile ESC gazini ayarlayarak HEDEF HIZI sabit tutmak.
 *      u = Kp*e + Ki*Integral(e) + Kd*de/dt   (slaytlardaki formul)
 *
 *  GUVENLIK (Faz 2'den DEVAM EDER — gerilemez):
 *    - Motor DISARMED baslar; PID cikisi yalnizca ARMED iken ESC'ye gider.
 *    - Watchdog: CMD_TIMEOUT_MS boyunca hedef hiz komutu gelmezse DISARMED.
 *    - THROTTLE_LIMIT_PCT ile ust sinir. TEKERLER HAVADA test et.
 *    - Anti-windup: integral clamp.
 *
 *  KULLANIM (Serial Monitor, 115200):
 *    a       -> ARM (ESC arming, bloklamayan)
 *    x       -> DISARM / dur
 *    0..9    -> hedef hiz = rakam * 0.5 m/s  (orn '4' -> 2.0 m/s)
 *    +/-     -> hedef hizi 0.1 m/s artir/azalt
 *    Her komut watchdog'u besler.
 *
 *  CONFIRM: encoder/hiz sabitleri Faz 3 ile AYNI olmali (PPR, capi, gear).
 *  NOT: arduino-cli yok -> derlenmeden, okuma ile dogrulandi.
 * ============================================================================
 */

#include <ESP32Servo.h>

// ---- Pinler ----
const int PIN_ESC   = 5;
const int PIN_ENC_A = 6;

// ---- ESC PWM ----
const int ESC_MIN_US  = 1000;
const int ESC_MAX_US  = 2000;
const int ESC_NEUTRAL = 1500;

// ---- Guvenlik ----
const int THROTTLE_LIMIT_PCT = 30;
const unsigned long ARM_DURATION_MS = 3000;
const unsigned long CMD_TIMEOUT_MS  = 2000;   // bench testte serial; biraz genis

// ---- Encoder/hiz (Faz 3 ile AYNI - CONFIRM) ----
const float ENC_PPR          = 360.0;
const float WHEEL_DIAMETER_M = 0.065;
const float GEAR_RATIO       = 1.0;
const float WHEEL_CIRC_M     = 3.14159265f * WHEEL_DIAMETER_M;

// ---- PID kazanclari (TUNE: once dusuk basla) ----
float Kp = 80.0, Ki = 120.0, Kd = 2.0;   // cikis birimi: ESC mikro-saniye sapmasi/(m/s)
const float I_CLAMP = 300.0;              // anti-windup (us)

// ---- Durum ----
enum MotorState { DISARMED, ARMING, ARMED };
MotorState motorState = DISARMED;
unsigned long armStartMs = 0, lastCmdMs = 0, lastLoop = 0;
float targetSpeed = 0.0;       // m/s
float integ = 0.0, prevErr = 0.0;

volatile long encCount = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR encISR() { portENTER_CRITICAL_ISR(&encMux); encCount++; portEXIT_CRITICAL_ISR(&encMux); }
long readAndResetEnc() { long c; portENTER_CRITICAL(&encMux); c = encCount; encCount = 0; portEXIT_CRITICAL(&encMux); return c; }

Servo esc;

void escNeutral() { esc.writeMicroseconds(ESC_NEUTRAL); }

// us sapmasini limitli sekilde ESC'ye yaz (yalniz ARMED)
void escApply(float deltaUs) {
  if (motorState != ARMED) { escNeutral(); return; }
  float maxDelta = (ESC_MAX_US - ESC_NEUTRAL) * THROTTLE_LIMIT_PCT / 100.0;
  deltaUs = constrain(deltaUs, -maxDelta, maxDelta);
  esc.writeMicroseconds((int)(ESC_NEUTRAL + deltaUs));
}

void disarm(const char* why) {
  motorState = DISARMED; escNeutral();
  integ = 0; prevErr = 0; targetSpeed = 0;
  Serial.printf("[STATE] DISARMED (%s)\n", why);
}
void beginArming() {
  if (motorState != DISARMED) return;
  motorState = ARMING; armStartMs = millis(); lastCmdMs = millis();
  escNeutral(); integ = 0; prevErr = 0;
  Serial.println("[STATE] ARMING...");
}

void setup() {
  Serial.begin(115200); delay(300);
  ESP32PWM::allocateTimer(0);
  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC, ESC_MIN_US, ESC_MAX_US);
  escNeutral();

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, RISING);

  lastLoop = millis();
  Serial.println("[Faz4] Hiz PID. 'a'=ARM 'x'=DUR  rakam=hedef*0.5 m/s  +/- ince ayar. TEKERLER HAVADA.");
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'a') beginArming();
    else if (c == 'x') disarm("serial x");
    else if (c >= '0' && c <= '9') { targetSpeed = (c - '0') * 0.5; lastCmdMs = millis(); }
    else if (c == '+') { targetSpeed += 0.1; lastCmdMs = millis(); }
    else if (c == '-') { targetSpeed = max(0.0f, targetSpeed - 0.1f); lastCmdMs = millis(); }
    else continue;
    if (c != '\n' && c != '\r')
      Serial.printf("[cmd] hedef=%.2f m/s state=%d\n", targetSpeed, motorState);
  }
}

void loop() {
  handleSerial();

  // Arming durum makinesi (bloklamayan)
  if (motorState == ARMING && millis() - armStartMs >= ARM_DURATION_MS) {
    motorState = ARMED; Serial.println("[STATE] ARMED.");
  }
  // Watchdog
  if (motorState != DISARMED && millis() - lastCmdMs > CMD_TIMEOUT_MS)
    disarm("watchdog");

  // PID dongusu — sabit periyot (50 Hz)
  unsigned long now = millis();
  if (now - lastLoop < 20) return;
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;

  long counts = readAndResetEnc();
  float wheelRevs = (counts / ENC_PPR) / GEAR_RATIO;
  float speed = (wheelRevs * WHEEL_CIRC_M) / dt;

  float err = targetSpeed - speed;
  integ += err * dt;
  integ = constrain(integ, -I_CLAMP / (Ki + 1e-6f), I_CLAMP / (Ki + 1e-6f));  // anti-windup
  float deriv = (err - prevErr) / dt;
  prevErr = err;
  float u = Kp * err + Ki * integ + Kd * deriv;   // ESC us sapmasi
  escApply(u);

  static unsigned long lastP = 0;
  if (now - lastP > 200) {   // 5 Hz log
    lastP = now;
    Serial.printf("hedef=%.2f olcum=%.2f hata=%.2f u=%.0f st=%d\n",
                  targetSpeed, speed, err, u, motorState);
  }
}
