/*
 * ============================================================================
 *  Faz 5 — Pose + EKF  (ESP32-S3)
 *  Autonomous RC Car Racing Platform / Kontrol Katmani
 * ============================================================================
 *
 *  AMAC:
 *    Aracin nerede oldugunu bilmesi. Sensorleri birlestirir (EKF):
 *      - gyro (MPU6050)        -> tahmin girdisi, yuksek frekans
 *      - encoder hizi          -> hiz olcumu, yuksek frekans
 *      - kamera pose (UDP)     -> konum olcumu, dusuk frekans (offboard tracker)
 *    Durum: [x, y, psi, v].
 *
 *  >>> BU KOD `Simulation/ekf_localization.py` ILE AYNI YAPIDADIR <<<
 *    O Python referansi sentetik veriyle DOGRULANDI (RMS ~5 cm). Buradaki C
 *    matematigi onunla birebir ayni; donanimda once Faz 3 (IMU/encoder)
 *    dogrulanmadan bu filtreye guvenme.
 *
 *  UDP POSE PAKETI (offboard tracker -> arac):
 *    UDP port 4210'a metin: "x,y,psi"  (metre, metre, radyan)
 *    Test icin: Simulation/fake_pose_sender.py
 *
 *  KRITIK: psi yeniligi [-pi,pi]'ye sarilir (wrap), yoksa filtre patlar.
 *  NOT: arduino-cli yok -> derlenmeden, okuma ile dogrulandi (Python referansi calisir).
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>

// ---- WiFi ----
const char* WIFI_SSID = "WIFI_ADINI_YAZ";
const char* WIFI_PASS = "WIFI_SIFRENI_YAZ";
const uint16_t UDP_PORT = 4210;

// ---- Pinler ----
const int PIN_SDA = 8, PIN_SCL = 9, PIN_ENC_A = 6;

// ---- MPU6050 ----
const uint8_t MPU_ADDR = 0x68;
const float GYRO_SCALE = 131.0;          // LSB/(deg/s) @ +/-250dps
const float DEG2RAD = 3.14159265f / 180.0f;

// ---- Encoder/hiz (Faz 3 ile AYNI - CONFIRM) ----
const float ENC_PPR = 360.0, WHEEL_DIAMETER_M = 0.065, GEAR_RATIO = 1.0;
const float WHEEL_CIRC_M = 3.14159265f * WHEEL_DIAMETER_M;

WiFiUDP udp;
char udpBuf[64];

volatile long encCount = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR encISR() { portENTER_CRITICAL_ISR(&encMux); encCount++; portEXIT_CRITICAL_ISR(&encMux); }
long readEnc() { long c; portENTER_CRITICAL(&encMux); c = encCount; encCount = 0; portEXIT_CRITICAL(&encMux); return c; }

// ============================ EKF ===========================================
float X[4] = {0, 0, 0, 0};                 // x, y, psi, v
float P[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
const float Q[4] = {0.01, 0.01, 0.005, 0.05};   // kosegen surec gurultusu
const float R_POS = 0.0025, R_PSI = 0.0025, R_V = 0.01;

float wrapPi(float a) {
  while (a >  PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

void ekfPredict(float omega, float dt) {
  float x = X[0], y = X[1], psi = X[2], v = X[3];
  X[0] = x + v * cosf(psi) * dt;
  X[1] = y + v * sinf(psi) * dt;
  X[2] = wrapPi(psi + omega * dt);
  X[3] = v;
  // F = df/dx
  float F[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  F[0][2] = -v * sinf(psi) * dt;  F[0][3] = cosf(psi) * dt;
  F[1][2] =  v * cosf(psi) * dt;  F[1][3] = sinf(psi) * dt;
  // P = F P F^T + Q  (kosegen Q)
  float FP[4][4], Pn[4][4];
  for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
    float s = 0; for (int k = 0; k < 4; k++) s += F[i][k] * P[k][j]; FP[i][j] = s;
  }
  for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
    float s = 0; for (int k = 0; k < 4; k++) s += FP[i][k] * F[j][k]; Pn[i][j] = s;
  }
  for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P[i][j] = Pn[i][j] + (i == j ? Q[i] : 0);
}

// Sirali skaler olcum guncellemesi (matris ters cevirme yok)
void ekfUpdateScalar(const float H[4], float z, float r, bool isAngle) {
  float Hx = 0; for (int i = 0; i < 4; i++) Hx += H[i] * X[i];
  float y = z - Hx; if (isAngle) y = wrapPi(y);
  float PHt[4];
  for (int i = 0; i < 4; i++) { float s = 0; for (int k = 0; k < 4; k++) s += P[i][k] * H[k]; PHt[i] = s; }
  float S = r; for (int i = 0; i < 4; i++) S += H[i] * PHt[i];
  float K[4]; for (int i = 0; i < 4; i++) K[i] = PHt[i] / S;
  for (int i = 0; i < 4; i++) X[i] += K[i] * y;
  X[2] = wrapPi(X[2]);
  // P = (I - K H) P
  float IKH[4][4], Pn[4][4];
  for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) IKH[i][j] = (i == j ? 1.0 : 0.0) - K[i] * H[j];
  for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
    float s = 0; for (int k = 0; k < 4; k++) s += IKH[i][k] * P[k][j]; Pn[i][j] = s;
  }
  for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P[i][j] = Pn[i][j];
}

void ekfUpdateSpeed(float v) { float H[4] = {0,0,0,1}; ekfUpdateScalar(H, v, R_V, false); }
void ekfUpdatePose(float mx, float my, float mpsi) {
  float Hx[4] = {1,0,0,0}, Hy[4] = {0,1,0,0}, Hp[4] = {0,0,1,0};
  ekfUpdateScalar(Hx, mx, R_POS, false);
  ekfUpdateScalar(Hy, my, R_POS, false);
  ekfUpdateScalar(Hp, mpsi, R_PSI, true);
}

// ============================ Sensorler =====================================
void mpuWake() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
}
float readGyroZ_rad() {   // yaw rate
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);   // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom((int)MPU_ADDR, 2) != 2) return 0;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return (raw / GYRO_SCALE) * DEG2RAD;
}

void connectWifi() {
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(250); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] IP: %s  UDP pose port: %u\n", WiFi.localIP().toString().c_str(), UDP_PORT);
    udp.begin(UDP_PORT);
  } else Serial.println("[WiFi] baglanamadi.");
}

unsigned long lastLoop = 0, lastPrint = 0;

void setup() {
  Serial.begin(115200); delay(300);
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000); mpuWake();
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, RISING);
  connectWifi();
  lastLoop = millis();
  Serial.println("[Faz5] EKF basladi. Pose icin UDP'den 'x,y,psi' bekleniyor.");
}

void loop() {
  // --- UDP pose olcumu (varsa) ---
  int sz = udp.parsePacket();
  if (sz > 0) {
    int n = udp.read(udpBuf, sizeof(udpBuf) - 1); udpBuf[n] = 0;
    float mx, my, mpsi;
    if (sscanf(udpBuf, "%f,%f,%f", &mx, &my, &mpsi) == 3)
      ekfUpdatePose(mx, my, wrapPi(mpsi));
  }

  // --- predict + speed update, sabit 50 Hz ---
  unsigned long now = millis();
  if (now - lastLoop < 20) return;
  float dt = (now - lastLoop) / 1000.0; lastLoop = now;

  float omega = readGyroZ_rad();
  long counts = readEnc();
  float speed = ((counts / ENC_PPR) / GEAR_RATIO) * WHEEL_CIRC_M / dt;

  ekfPredict(omega, dt);
  ekfUpdateSpeed(speed);

  if (now - lastPrint > 200) {
    lastPrint = now;
    Serial.printf("EKF: x=%.2f y=%.2f psi=%.0fdeg v=%.2f\n",
                  X[0], X[1], X[2] * 180.0 / PI, X[3]);
  }
}
