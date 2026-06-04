/*
 * ============================================================================
 *  Faz 3 — Sensorler  (ESP32-S3)
 *  Autonomous RC Car Racing Platform / Kontrol Katmani
 * ============================================================================
 *
 *  AMAC:
 *    Araca "duyu" kazandirmak. Uc sensoru oku ve Serial'e bas:
 *      1) IMU (MPU6050, I2C)        -> ivme + acisal hiz (gyro)
 *      2) Tekerlek encoder (interrupt) -> dogrusal hiz
 *      3) Pil voltaji (ADC + bolucu)
 *    Bu degerler ileride Kalman/PID/LQR'in girdisidir.
 *
 *  KUTUPHANE: ESP32Servo gerekmez. MPU6050 dogrudan Wire (I2C) ile, ek
 *  kutuphane olmadan okunur. (BNO055 kullanacaksan Adafruit_BNO055 gerekir.)
 *
 *  ----------------------------------------------------------------------------
 *  >>> KABLOLAMA <<<
 *  ----------------------------------------------------------------------------
 *    MPU6050:  VCC->3.3V  GND->GND  SDA->GPIO8  SCL->GPIO9
 *    Encoder:  A->GPIO6   (B->GPIO7 opsiyonel)  VCC->3.3V/5V  GND->GND
 *    Pil:      bolucu ortasindan -> GPIO1 (ADC).  ASLA ham LiPo'yu ADC'ye verme!
 *
 *  >>> CONFIRM (donanimina gore DOGRULA — buradan tahmin edilemez) <<<
 *    - ENC_PPR: encoder tur basina darbe (tek kanal yukselen kenar sayimi).
 *    - WHEEL_DIAMETER_M: tekerlek capi.
 *    - GEAR_RATIO: encoder MOTOR milinde mi (sanziman ONCESI) yoksa TEKER'de mi?
 *      Motor milindeyse hiz hesabini gear_ratio'ya boler. Yanlissa hiz tamamen yanlis cikar.
 *    - VDIV_RATIO: voltaj bolucu orani (orn. R1=10k, R2=2.2k -> (10+2.2)/2.2).
 *
 *  NOT: arduino-cli yok -> derlenmeden, okuma ile dogrulandi.
 * ============================================================================
 */

#include <Wire.h>

// ---- Pinler ----
const int PIN_SDA = 8;
const int PIN_SCL = 9;
const int PIN_ENC_A = 6;     // encoder A kanali (interrupt)
const int PIN_VBAT  = 1;     // pil voltaji ADC

// ---- MPU6050 I2C ----
const uint8_t MPU_ADDR       = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_XOUT = 0x3B;   // 0x3B..0x48: accel(6) temp(2) gyro(6)
const float ACCEL_SCALE = 16384.0;     // +/-2g  -> LSB/g
const float GYRO_SCALE  = 131.0;       // +/-250 dps -> LSB/(deg/s)
const float G = 9.81;

// ---- Encoder / hiz (CONFIRM!) ----
const float ENC_PPR          = 360.0;   // CONFIRM
const float WHEEL_DIAMETER_M = 0.065;   // CONFIRM (1/10 ~6.5 cm)
const float GEAR_RATIO       = 1.0;     // CONFIRM: motor milinde ise >1 yaz
const float WHEEL_CIRC_M     = 3.14159265f * WHEEL_DIAMETER_M;

// ---- Pil (CONFIRM!) ----
const float VDIV_RATIO = (10.0 + 2.2) / 2.2;  // CONFIRM bolucu orani

// ---- Encoder ISR durumu ----
volatile long encCount = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR encISR() {
  portENTER_CRITICAL_ISR(&encMux);
  encCount++;
  portEXIT_CRITICAL_ISR(&encMux);
}

long readAndResetEnc() {
  long c;
  portENTER_CRITICAL(&encMux);     // torn-read'i onle
  c = encCount;
  encCount = 0;
  portEXIT_CRITICAL(&encMux);
  return c;
}

// ---- MPU6050 ----
bool mpuWake() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);                // uyku bitini temizle -> SART (yoksa hep 0 okur)
  return Wire.endTransmission() == 0;
}

bool mpuRead(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();        // sicaklik (atla)
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();
  ax = rawAx / ACCEL_SCALE * G;  ay = rawAy / ACCEL_SCALE * G;  az = rawAz / ACCEL_SCALE * G;
  gx = rawGx / GYRO_SCALE;       gy = rawGy / GYRO_SCALE;       gz = rawGz / GYRO_SCALE;
  return true;
}

// ---- Pil ----
float readBatteryV() {
  float vAdc = analogReadMilliVolts(PIN_VBAT) / 1000.0;  // ESP32 kalibre mV
  return vAdc * VDIV_RATIO;
}

// ---- Zamanlama ----
unsigned long lastPrint = 0;
const unsigned long PRINT_PERIOD_MS = 100;   // 10 Hz cikti

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Serial.println(mpuWake() ? "[MPU] uyandirildi." : "[MPU] BULUNAMADI! kablolama/adres kontrol.");

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, RISING);

  analogReadResolution(12);
  Serial.println("[Faz3] sensor okuma basladi (10 Hz). CONFIRM sabitlerini dogrula.");
}

void loop() {
  unsigned long now = millis();
  if (now - lastPrint < PRINT_PERIOD_MS) return;
  float dt = (now - lastPrint) / 1000.0;
  lastPrint = now;

  // Encoder -> hiz
  long counts = readAndResetEnc();
  float wheelRevs = (counts / ENC_PPR) / GEAR_RATIO;
  float speed = (wheelRevs * WHEEL_CIRC_M) / dt;   // m/s

  // IMU
  float ax, ay, az, gx, gy, gz;
  bool imuOk = mpuRead(ax, ay, az, gx, gy, gz);

  float vbat = readBatteryV();

  Serial.printf("v=%.2f m/s | ", speed);
  if (imuOk) Serial.printf("acc[%.2f %.2f %.2f] gyro[%.1f %.1f %.1f] | ",
                           ax, ay, az, gx, gy, gz);
  else       Serial.print("IMU? | ");
  Serial.printf("vbat=%.2f V\n", vbat);
}
