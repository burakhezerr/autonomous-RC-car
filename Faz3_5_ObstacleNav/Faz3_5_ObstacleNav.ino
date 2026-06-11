/*
 * ============================================================================
 *  Faz 3.5 — Çift Ultrasonik Engel Algılama + MPU6050 IMU + Motor Kontrol
 *  (ESP32-S3)
 * ============================================================================
 *  KABLOLAMA:
 *    MPU6050:      SDA→GPIO8   SCL→GPIO9   VCC→3.3V  GND→GND
 *                  AD0→GND (0x68) veya AD0→3.3V (0x69) — otomatik algılanır
 *    Sol HC-SR04:  TRIG→GPIO10 ECHO→GPIO11 VCC→5V    GND→GND
 *    Sağ HC-SR04:  TRIG→GPIO12 ECHO→GPIO13 VCC→5V    GND→GND
 *    ⚠ ECHO 5V → voltaj bölücü: 1kΩ seri + 2kΩ GND'e
 *    Servo:        GPIO4   (sinyal)
 *    ESC:          GPIO5   (sinyal)
 *
 *  MİMARİ:
 *    Core 0 — sonarTask (FreeRTOS, pulseIn güvenli)
 *    Core 1 — loop: IMU + WiFi + WebServer + otonom sürüş
 *  WEB UI:  http://<IP>:5003/
 *
 *  GÜVENLİK:
 *    - Açılışta DISARMED; web'den ARM edilmeden motor çalışmaz
 *    - L1 engel (<8 cm) → E-STOP → DISARMED (re-ARM gerekir)
 *    - Sonar verisi 1 s gelmezse → DISARMED
 *    - THROTTLE_LIMIT_PCT: web slider'dan değiştirilebilir (max %30 başlangıç)
 * ============================================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <math.h>

// ========================= WiFi / Web =======================================
const char* WIFI_SSID = "Hezer";
const char* WIFI_PASS = "burakhezer";
const int   WEB_PORT  = 5003;

// ========================= Motor Pinler / PWM ================================
const int PIN_SERVO    = 4;
const int PIN_ESC      = 5;
const int SERVO_MIN    = 1000;
const int SERVO_CENTER = 1500;
const int SERVO_MAX    = 2000;
const int ESC_MIN_US   = 1000;
const int ESC_MAX_US   = 2000;
const int ESC_NEUTRAL  = 1500;

const unsigned long ARM_DURATION_MS  = 3000;
const unsigned long SONAR_WDOG_MS    = 1000;  // sonar dursa disarm
int throttleLimit = 30;   // web'den ayarlanabilir, max gaz yüzdesi
int autoThrottle  = 20;   // otonom sürüş baz gazı (%)

// ========================= IMU Pinler / Sabitler ============================
const int PIN_SDA = 8;
const int PIN_SCL = 9;

uint8_t         MPU_ADDR         = 0x68;   // setup'ta 0x69'a düşebilir
const uint8_t   REG_PWR_MGMT_1  = 0x6B;
const uint8_t   REG_ACCEL_XOUT  = 0x3B;
const float     ACCEL_SCALE     = 16384.0f;
const float     GYRO_SCALE      = 131.0f;
const float     G_MS2           = 9.81f;
const unsigned long READ_INTERVAL_MS = 20;
const int           CAL_SAMPLES      = 300;
const float         CF_ALPHA         = 0.98f;

// ========================= Sonar Sabitler ===================================
const int   PIN_TRIG_L = 10;
const int   PIN_ECHO_L = 11;
const int   PIN_TRIG_R = 12;
const int   PIN_ECHO_R = 13;

const float PROX_L1       =  8.0f;
const float PROX_L2       = 20.0f;
const float PROX_L3       = 50.0f;
const float PROX_L4       = 75.0f;
const float MAX_DIST      = 150.0f;
const float STEER_GAIN    =  0.4f;
const float MAX_STEER_DEG = 30.0f;
const float DEADBAND_CM   =  8.0f;
const unsigned long SONAR_TASK_MS = 60;

// ========================= IMU Durum ========================================
struct ImuRaw { float ax,ay,az,gx,gy,gz,temp; bool ok; };
ImuRaw  imu        = {};
bool    mpuFound   = false;
bool    calibrated = false;
float   bias_ax=0, bias_ay=0, bias_az=0;
float   bias_gx=0, bias_gy=0, bias_gz=0;
float   roll=0, pitch=0, yaw=0;
float   dr_x=0, dr_y=0, dr_vx=0, dr_vy=0;
unsigned long lastReadMs = 0;
unsigned long lastLoopUs = 0;

// ========================= Sonar Durum (mutex korumalı) =====================
portMUX_TYPE sonarMux = portMUX_INITIALIZER_UNLOCKED;
float leftCm   = -1.0f;
float rightCm  = -1.0f;
int   leftLevel  = 0;
int   rightLevel = 0;
float steerDeg   = 0.0f;
unsigned long sonarLastUpdMs = 0;  // watchdog için

const int LOG_SIZE = 12;
String logBuf[LOG_SIZE];
int    logIdx = 0;

// ========================= Motor Durum ======================================
enum MotorState { DISARMED, ARMING, ARMED };
volatile MotorState motorState = DISARMED;
unsigned long armStartMs  = 0;
int curSteerPct    = 0;
int curThrottlePct = 0;
int appliedEscUs   = ESC_NEUTRAL;
int appliedServoUs = SERVO_CENTER;

Servo servo;
Servo esc;

WebServer server(WEB_PORT);

// ========================= IMU Fonksiyonları ================================
bool mpuWake() {
  // AD0 GND → 0x68, AD0 3.3V → 0x69 — ikisini dene
  for (uint8_t addr : {(uint8_t)0x68, (uint8_t)0x69}) {
    Wire.beginTransmission(addr);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x00);
    if (Wire.endTransmission() == 0) {
      MPU_ADDR = addr;
      Serial.printf("[IMU] 0x%02X adresinde bulundu\n", addr);
      return true;
    }
  }
  // I2C tarama — hangi adresler var?
  Serial.print("[IMU] Tarama: ");
  bool any = false;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf("0x%02X ", a); any = true; }
  }
  if (!any) Serial.print("HİÇBİR CIHAZ YOK");
  Serial.println();
  return false;
}

bool mpuReadRaw(ImuRaw& d) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(REG_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) { d.ok=false; return false; }
  if (Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14) < 14) { d.ok=false; return false; }
  auto rw = []()->int16_t{ return (int16_t)((Wire.read()<<8)|Wire.read()); };
  d.ax=rw()/ACCEL_SCALE*G_MS2; d.ay=rw()/ACCEL_SCALE*G_MS2; d.az=rw()/ACCEL_SCALE*G_MS2;
  d.temp=rw()/340.0f+36.53f;
  d.gx=rw()/GYRO_SCALE; d.gy=rw()/GYRO_SCALE; d.gz=rw()/GYRO_SCALE;
  d.ok=true; return true;
}

void calibrate() {
  Serial.printf("[CAL] Sabit tut — %d örnek...\n", CAL_SAMPLES);
  double sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0; int n=0; ImuRaw d;
  while (n < CAL_SAMPLES) {
    if (mpuReadRaw(d)) { sax+=d.ax; say+=d.ay; saz+=d.az; sgx+=d.gx; sgy+=d.gy; sgz+=d.gz; n++; }
    delay(READ_INTERVAL_MS);
  }
  float mx=sax/n, my=say/n, mz=saz/n;
  float abx=fabsf(mx), aby=fabsf(my), abz=fabsf(mz);
  float gsx=0, gsy=0, gsz=0;
  if (abx>=aby && abx>=abz)      gsx = (mx>=0)?1:-1;
  else if (aby>=abx && aby>=abz) gsy = (my>=0)?1:-1;
  else                            gsz = (mz>=0)?1:-1;
  bias_ax=mx-gsx*G_MS2; bias_ay=my-gsy*G_MS2; bias_az=mz-gsz*G_MS2;
  bias_gx=sgx/n; bias_gy=sgy/n; bias_gz=sgz/n;
  float ax0=mx-bias_ax, ay0=my-bias_ay, az0=mz-bias_az;
  roll  = atan2f(ay0,az0)*180.0f/M_PI;
  pitch = atan2f(-ax0,sqrtf(ay0*ay0+az0*az0))*180.0f/M_PI;
  yaw=0; dr_x=dr_y=dr_vx=dr_vy=0;
  calibrated = true;
  Serial.printf("[CAL] Tamam. bias_a=(%.3f,%.3f,%.3f) bias_g=(%.2f,%.2f,%.2f)\n",
                bias_ax,bias_ay,bias_az,bias_gx,bias_gy,bias_gz);
}

void updateFusion(float dt) {
  float ax=imu.ax, ay=imu.ay, az=imu.az, gx=imu.gx, gy=imu.gy, gz=imu.gz;
  float ra = atan2f(ay,az)*180.0f/M_PI;
  float pa = atan2f(-ax,sqrtf(ay*ay+az*az))*180.0f/M_PI;
  roll  = CF_ALPHA*(roll +gx*dt) + (1-CF_ALPHA)*ra;
  pitch = CF_ALPHA*(pitch+gy*dt) + (1-CF_ALPHA)*pa;
  yaw  += gz*dt;
  if (yaw> 180) yaw-=360;
  if (yaw<-180) yaw+=360;
  float yr = yaw*M_PI/180.0f;
  float axw = ax*cosf(yr)-ay*sinf(yr);
  float ayw = ax*sinf(yr)+ay*cosf(yr);
  if (fabsf(axw)<0.15f) axw=0;
  if (fabsf(ayw)<0.15f) ayw=0;
  dr_vx+=axw*dt; dr_vy+=ayw*dt; dr_vx*=0.98f; dr_vy*=0.98f;
  dr_x+=dr_vx*dt; dr_y+=dr_vy*dt;
}

// ========================= Sonar Yardımcılar ================================
int proxLevel(float cm) {
  if (cm<0)       return 0;
  if (cm<PROX_L1) return 1;
  if (cm<PROX_L2) return 2;
  if (cm<PROX_L3) return 3;
  if (cm<PROX_L4) return 4;
  return 0;
}
float computeSteering(float lCm, float rCm) {
  float l = (lCm>0 && lCm<MAX_DIST) ? lCm : MAX_DIST;
  float r = (rCm>0 && rCm<MAX_DIST) ? rCm : MAX_DIST;
  if (l>=MAX_DIST && r>=MAX_DIST) return 0.0f;
  float diff = r - l;
  if (fabsf(diff)<DEADBAND_CM) return 0.0f;
  float angle = STEER_GAIN * diff;
  if (angle >  MAX_STEER_DEG) angle =  MAX_STEER_DEG;
  if (angle < -MAX_STEER_DEG) angle = -MAX_STEER_DEG;
  return angle;
}
String levelName(int lv) {
  const char* n[]={"--","L1","L2","L3","L4"};
  return (lv>=0&&lv<=4) ? n[lv] : "--";
}
void addLog(const String& msg) {
  logBuf[logIdx % LOG_SIZE] = msg;
  logIdx++;
}
float measureSonar(int trig, int echo) {
  digitalWrite(trig,LOW);  delayMicroseconds(2);
  digitalWrite(trig,HIGH); delayMicroseconds(10);
  digitalWrite(trig,LOW);
  unsigned long dur = pulseIn(echo, HIGH, 20000);
  if (dur==0) return -1.0f;
  return dur * 0.01716f;
}

// ========================= FreeRTOS Sonar Task (Core 0) =====================
void sonarTask(void* param) {
  for (;;) {
    float lCm = measureSonar(PIN_TRIG_L, PIN_ECHO_L);
    vTaskDelay(pdMS_TO_TICKS(10));
    float rCm = measureSonar(PIN_TRIG_R, PIN_ECHO_R);
    int lLv = proxLevel(lCm);
    int rLv = proxLevel(rCm);
    float st = computeSteering(lCm, rCm);

    portENTER_CRITICAL(&sonarMux);
    float prevSteer = steerDeg;
    leftCm=lCm; rightCm=rCm; leftLevel=lLv; rightLevel=rLv; steerDeg=st;
    sonarLastUpdMs = millis();
    portEXIT_CRITICAL(&sonarMux);

    if (lLv==1 || rLv==1) {
      addLog("TEHLİKE! Sol:L"+String(lLv)+" Sağ:L"+String(rLv)+
             " → "+(st>0?"Sağa ":st<0?"Sola ":"DUR ")+String(st,1)+"°");
    } else if (fabsf(st-prevSteer)>1.5f) {
      String lStr = lCm<0?"---":String(lCm,1);
      String rStr = rCm<0?"---":String(rCm,1);
      String dir  = fabsf(st)<0.5f ? "Düz git" :
                    (st>0 ? "→ Sağa "+String(st,1)+"°" : "← Sola "+String(fabsf(st),1)+"°");
      addLog("Sol:"+lStr+"["+levelName(lLv)+"] Sağ:"+rStr+"["+levelName(rLv)+"] "+dir);
    }
    Serial.printf("Sol:%-7s Sağ:%-7s %+6.1f°\n",
      (lCm<0?"---":(String(lCm,1)+"cm")).c_str(),
      (rCm<0?"---":(String(rCm,1)+"cm")).c_str(), st);

    vTaskDelay(pdMS_TO_TICKS(SONAR_TASK_MS));
  }
}

// ========================= Motor Kontrol =====================================
void writeServoPct(int pct) {
  pct = constrain(pct, -100, 100);
  int us = SERVO_CENTER + (int)((long)pct * (SERVO_MAX - SERVO_MIN) / 2 / 100);
  us = constrain(us, SERVO_MIN, SERVO_MAX);
  servo.writeMicroseconds(us);
  curSteerPct    = pct;
  appliedServoUs = us;
}
void writeEscNeutral() {
  esc.writeMicroseconds(ESC_NEUTRAL);
  appliedEscUs   = ESC_NEUTRAL;
  curThrottlePct = 0;
}
void writeThrottlePct(int pct) {
  pct = constrain(pct, -100, 100);
  curThrottlePct = pct;
  if (motorState != ARMED) { writeEscNeutral(); return; }
  int eff = pct * throttleLimit / 100;
  int us  = ESC_NEUTRAL + (int)((long)eff * (ESC_MAX_US - ESC_MIN_US) / 2 / 100);
  us = constrain(us, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(us);
  appliedEscUs = us;
}
void disarm(const char* reason) {
  motorState = DISARMED;
  writeEscNeutral();
  writeServoPct(0);
  Serial.printf("[STATE] DISARMED | %s\n", reason);
}
void beginArming() {
  if (motorState != DISARMED) return;
  motorState = ARMING;
  armStartMs = millis();
  writeEscNeutral();
  Serial.printf("[STATE] ARMING | %lu ms nötr sinyal...\n", ARM_DURATION_MS);
}
const char* stateName() {
  switch (motorState) {
    case DISARMED: return "DISARMED";
    case ARMING:   return "ARMING";
    case ARMED:    return "ARMED";
  }
  return "?";
}

// ========================= Otonom Sürüş =====================================
void autonomousDrive() {
  if (motorState != ARMED) return;

  float lCm, rCm, st; int lLv, rLv;
  unsigned long sonarAge;
  portENTER_CRITICAL(&sonarMux);
  lCm=leftCm; rCm=rightCm; lLv=leftLevel; rLv=rightLevel; st=steerDeg;
  sonarAge = millis() - sonarLastUpdMs;
  portEXIT_CRITICAL(&sonarMux);

  // Sonar watchdog
  if (sonarAge > SONAR_WDOG_MS) { disarm("sonar-timeout"); return; }

  // L1 acil durum durdur
  int maxLv = max(lLv, rLv);
  if (maxLv == 1) { disarm("L1-engel"); writeServoPct(0); return; }

  // Direksiyon: steerDeg → servo yüzdesi
  int steerPct = constrain((int)(st / MAX_STEER_DEG * 100), -100, 100);
  writeServoPct(steerPct);

  // Hız: yakınlık seviyesine göre kısalt
  int speedPct;
  if (maxLv == 0 || maxLv == 4) speedPct = autoThrottle;
  else if (maxLv == 3)          speedPct = autoThrottle * 65 / 100;
  else                          speedPct = autoThrottle * 25 / 100;  // L2
  writeThrottlePct(speedPct);
}

// ========================= HTTP =============================================
void handleArm()    { beginArming(); server.send(200,"application/json","{\"ok\":true}"); }
void handleDisarm() { disarm("web"); server.send(200,"application/json","{\"ok\":true}"); }
void handleSetLimit() {
  if (server.hasArg("v")) throttleLimit = constrain(server.arg("v").toInt(), 5, 50);
  if (server.hasArg("a")) autoThrottle  = constrain(server.arg("a").toInt(), 5, 40);
  server.send(200,"application/json","{\"ok\":true}");
}
void handleStatus() {
  float lCm, rCm, st; int lLv, rLv;
  portENTER_CRITICAL(&sonarMux);
  lCm=leftCm; rCm=rightCm; lLv=leftLevel; rLv=rightLevel; st=steerDeg;
  portEXIT_CRITICAL(&sonarMux);

  String logs = "[";
  for (int i=0; i<LOG_SIZE; i++) {
    int idx = ((logIdx-1-i) % LOG_SIZE + LOG_SIZE) % LOG_SIZE;
    if (logBuf[idx].length()==0) continue;
    if (logs.length()>1) logs += ",";
    logs += "\"" + logBuf[idx] + "\"";
  }
  logs += "]";

  char buf[1024];
  snprintf(buf, sizeof(buf),
    "{\"lCm\":%.1f,\"rCm\":%.1f,\"lLv\":%d,\"rLv\":%d,\"steer\":%.2f,"
    "\"ok\":%s,\"cal\":%s,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
    "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
    "\"x\":%.4f,\"y\":%.4f,\"vx\":%.4f,\"vy\":%.4f,"
    "\"temp\":%.1f,"
    "\"mstate\":\"%s\",\"steerPct\":%d,\"thrPct\":%d,\"escUs\":%d,"
    "\"limit\":%d,\"auto\":%d,"
    "\"logs\":%s}",
    lCm,rCm,lLv,rLv,st,
    imu.ok?"true":"false", calibrated?"true":"false",
    imu.ax,imu.ay,imu.az,
    imu.gx,imu.gy,imu.gz,
    roll,pitch,yaw,
    dr_x,dr_y,dr_vx,dr_vy,
    imu.temp,
    stateName(), curSteerPct, curThrottlePct, appliedEscUs,
    throttleLimit, autoThrottle,
    logs.c_str());
  server.send(200, "application/json", buf);
}
void handleReset() {
  yaw=0; dr_x=0; dr_y=0; dr_vx=0; dr_vy=0;
  server.send(200, "application/json", "{\"ok\":true}");
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>ObstacleNav + IMU — Faz 3.5</title>
<style>
:root{--bg:#0a0e13;--card:#111820;--line:#1e2d3d;--txt:#dde6f0;--mut:#5a7a96;
      --ok:#22c55e;--bad:#ef4444;--warn:#f59e0b;--cal:#a78bfa;
      --p0:#5a7a96;--p1:#ef4444;--p2:#f97316;--p3:#eab308;--p4:#22c55e}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font:13px/1.5 system-ui,sans-serif;
     padding:14px;max-width:660px;margin:0 auto}
h1{font-size:16px;font-weight:700;margin-bottom:2px}
.sub{font-size:11px;color:var(--mut);margin-bottom:10px}
.badges{display:flex;gap:8px;margin-bottom:12px;flex-wrap:wrap;align-items:center}
.badge{padding:3px 10px;border-radius:99px;font-size:11px;font-weight:700}
.okb{background:#14532d;color:#86efac}.badb{background:#450a0a;color:#fca5a5}
.calok{background:#2e1065;color:#c4b5fd}.calno{background:#1c1917;color:#78716c}
.armed{background:#1e3a1e;color:#86efac}.arming{background:#3a2e00;color:#fde68a}
.disarmed{background:#1c1917;color:#78716c}
.btn{background:#1e2d3d;border:1px solid var(--line);color:var(--txt);
     padding:4px 12px;border-radius:6px;font-size:12px;cursor:pointer}
.btn:hover{background:#253545}
.btn-arm{background:#14532d;border-color:#166534;color:#86efac}
.btn-arm:hover{background:#166534}
.btn-disarm{background:#450a0a;border-color:#7f1d1d;color:#fca5a5}
.btn-disarm:hover{background:#7f1d1d}
section{margin-bottom:12px}
.stitle{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;
        letter-spacing:.8px;margin-bottom:8px}
.motor-card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
.motor-card h2{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;letter-spacing:.8px;margin-bottom:10px}
.motor-row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-bottom:8px}
.motor-stat{display:flex;gap:18px;flex-wrap:wrap;margin-top:8px}
.ms-item{text-align:center}
.ms-lbl{font-size:10px;color:var(--mut)}
.ms-val{font-size:18px;font-weight:700;font-family:monospace}
.slider-row{display:flex;align-items:center;gap:8px;margin-top:6px}
.slider-row label{font-size:11px;color:var(--mut);white-space:nowrap;min-width:110px}
input[type=range]{flex:1;accent-color:#3b82f6}
.gauges{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.gauge{background:var(--card);border:1px solid var(--line);border-radius:12px;
       padding:10px;text-align:center}
.gauge .gl{font-size:10px;color:var(--mut);text-transform:uppercase;letter-spacing:.8px}
.gauge .gv{font-size:20px;font-weight:700;font-family:monospace;margin:4px 0}
.gauge .gu{font-size:10px;color:var(--mut)}
.rc{color:#3b82f6}.pc{color:#8b5cf6}.yc{color:#22c55e}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}
.card h2{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;
         letter-spacing:.8px;margin-bottom:8px}
.row{display:flex;justify-content:space-between;padding:4px 0;
     border-bottom:1px solid var(--line)}
.row:last-child{border-bottom:none}
.lbl{font-size:11px;color:var(--mut)}
.val{font-size:14px;font-weight:700;font-family:monospace}
.unit{font-size:10px;color:var(--mut);margin-left:3px}
.sensors{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.sensor-card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px}
.sensor-card h2{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;
                letter-spacing:.8px;margin-bottom:10px;text-align:center}
.arc-wrap{display:flex;justify-content:center;margin-bottom:10px}
.dist-num{font-size:28px;font-weight:700;font-family:monospace;text-align:center;transition:color .2s}
.dist-unit{font-size:11px;color:var(--mut);text-align:center}
.level-label{font-size:12px;font-weight:700;text-align:center;margin-top:6px;transition:color .2s}
.bars{display:flex;justify-content:center;gap:5px;align-items:flex-end;height:40px;margin-top:8px}
.lb{width:18px;border-radius:3px 3px 0 0;transition:opacity .15s}
.lb-4{height:20px;background:var(--p4)}.lb-3{height:27px;background:var(--p3)}
.lb-2{height:33px;background:var(--p2)}.lb-1{height:40px;background:var(--p1)}
.lb.off{opacity:.12}
.steer-card{background:var(--card);border:1px solid var(--line);border-radius:12px;
            padding:16px}
.steer-card h2{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;
               letter-spacing:.8px;margin-bottom:12px;text-align:center}
.steer-inner{display:flex;align-items:center;justify-content:center;gap:24px;flex-wrap:wrap}
.steer-info{text-align:center}
.steer-deg{font-size:42px;font-weight:700;font-family:monospace;transition:color .2s}
.steer-label{font-size:13px;font-weight:700;margin-top:4px;transition:color .2s}
.steer-note{font-size:10px;color:var(--mut);margin-top:6px}
.map-card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px}
.map-warn{font-size:10px;color:var(--warn);margin-top:6px}
.log-card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px}
.log-list{font-family:monospace;font-size:11px;max-height:150px;overflow-y:auto}
.log-entry{padding:3px 0;border-bottom:1px solid var(--line);color:var(--mut)}
.log-entry:last-child{border:none}
.log-entry.right{color:#3b82f6}.log-entry.left{color:#8b5cf6}
.log-entry.danger{color:var(--p1)}
canvas{display:block}
</style></head><body>
<h1>ObstacleNav + IMU — Faz 3.5</h1>
<div class="sub">Sol/Sağ HC-SR04 (GPIO10-13) · MPU6050 IMU (GPIO8-9) · Servo GPIO4 · ESC GPIO5</div>
<div class="badges">
  <span id="badge" class="badge badb">NO DATA</span>
  <span id="calbadge" class="badge calno">CALIBRATING...</span>
  <span id="mstatebadge" class="badge disarmed">DISARMED</span>
  <button class="btn" onclick="resetPos()">Reset Yaw + Konum</button>
</div>

<div class="motor-card">
  <h2>Motor Kontrol — Otonom Sürüş</h2>
  <div class="motor-row">
    <button class="btn btn-arm"    onclick="armCar()">▶ ARM</button>
    <button class="btn btn-disarm" onclick="disarmCar()">■ DISARM</button>
    <span style="font-size:11px;color:var(--mut)" id="motorNote">DISARMED — ARM için tıkla</span>
  </div>
  <div class="slider-row">
    <label>Max Gaz (ThrottleLimit) <span id="limVal">30</span>%</label>
    <input type="range" min="5" max="50" value="30" id="limSlider" oninput="updateLimits()">
  </div>
  <div class="slider-row">
    <label>Otonom Baz Gaz <span id="autoVal">20</span>%</label>
    <input type="range" min="5" max="40" value="20" id="autoSlider" oninput="updateLimits()">
  </div>
  <div class="motor-stat">
    <div class="ms-item"><div class="ms-lbl">Direksiyon</div><div class="ms-val" id="msteer">0%</div></div>
    <div class="ms-item"><div class="ms-lbl">Gaz</div><div class="ms-val" id="mthr">0%</div></div>
    <div class="ms-item"><div class="ms-lbl">ESC µs</div><div class="ms-val" id="mesc">1500</div></div>
  </div>
</div>

<section>
  <div class="stitle">IMU — Oryantasyon</div>
  <div class="gauges">
    <div class="gauge"><div class="gl">Roll</div><div class="gv rc" id="groll">—</div><div class="gu">°</div></div>
    <div class="gauge"><div class="gl">Pitch</div><div class="gv pc" id="gpitch">—</div><div class="gu">°</div></div>
    <div class="gauge"><div class="gl">Yaw</div><div class="gv yc" id="gyaw">—</div><div class="gu">°</div></div>
  </div>
</section>

<section>
  <div class="stitle">IMU — Ham Veri</div>
  <div class="grid2">
    <div class="card"><h2>Accelerometer (m/s²)</h2>
      <div class="row"><span class="lbl">X</span><span><span class="val" id="ax">—</span><span class="unit">m/s²</span></span></div>
      <div class="row"><span class="lbl">Y</span><span><span class="val" id="ay">—</span><span class="unit">m/s²</span></span></div>
      <div class="row"><span class="lbl">Z</span><span><span class="val" id="az">—</span><span class="unit">m/s²</span></span></div>
    </div>
    <div class="card"><h2>Gyroscope (°/s)</h2>
      <div class="row"><span class="lbl">X</span><span><span class="val" id="gx">—</span><span class="unit">°/s</span></span></div>
      <div class="row"><span class="lbl">Y</span><span><span class="val" id="gy">—</span><span class="unit">°/s</span></span></div>
      <div class="row"><span class="lbl">Z</span><span><span class="val" id="gz">—</span><span class="unit">°/s</span></span></div>
    </div>
  </div>
</section>

<section>
  <div class="stitle">Ultrasonik Sensörler</div>
  <div class="sensors">
    <div class="sensor-card">
      <h2>◀ Sol Sensör</h2>
      <div class="arc-wrap"><canvas id="arcL" width="150" height="85"></canvas></div>
      <div class="dist-num" id="ldist">—</div>
      <div class="dist-unit">cm</div>
      <div class="level-label" id="llabel">—</div>
      <div class="bars">
        <div class="lb lb-4 off" id="ll4"></div>
        <div class="lb lb-3 off" id="ll3"></div>
        <div class="lb lb-2 off" id="ll2"></div>
        <div class="lb lb-1 off" id="ll1"></div>
      </div>
    </div>
    <div class="sensor-card">
      <h2>Sağ Sensör ▶</h2>
      <div class="arc-wrap"><canvas id="arcR" width="150" height="85"></canvas></div>
      <div class="dist-num" id="rdist">—</div>
      <div class="dist-unit">cm</div>
      <div class="level-label" id="rlabel">—</div>
      <div class="bars">
        <div class="lb lb-4 off" id="rl4"></div>
        <div class="lb lb-3 off" id="rl3"></div>
        <div class="lb lb-2 off" id="rl2"></div>
        <div class="lb lb-1 off" id="rl1"></div>
      </div>
    </div>
  </div>
</section>

<section>
  <div class="steer-card">
    <h2>Yön Kararı</h2>
    <div class="steer-inner">
      <canvas id="steer" width="200" height="110"></canvas>
      <div class="steer-info">
        <div class="steer-deg" id="sdeg">0.0°</div>
        <div class="steer-label" id="slabel">Düz Git</div>
        <div class="steer-note">+ = Sağa · − = Sola</div>
      </div>
    </div>
  </div>
</section>

<section>
  <div class="map-card">
    <div class="stitle">Dead-Reckoning  <span style="color:var(--mut);font-weight:400;text-transform:none;letter-spacing:0" id="xyval"></span></div>
    <canvas id="map" width="620" height="180" style="border-radius:8px;width:100%"></canvas>
    <div class="map-warn">⚠ Drift uyarısı — gerçek konum için Faz 5 EKF gerekir.</div>
  </div>
</section>

<section>
  <div class="log-card">
    <div class="stitle">Engel Logu</div>
    <div class="log-list" id="logList"></div>
  </div>
</section>

<script>
const LV_NAMES=['Cisim yok','● L1 Çok yakın','● L2 Yakın','● L3 Orta','● L4 Uzak'];
const LV_COLS=['var(--mut)','var(--p1)','var(--p2)','var(--p3)','var(--p4)'];
const ARC_COLORS=['var(--p4)','var(--p3)','var(--p2)','var(--p1)'];
let limitTimer=null;

function drawArc(id,dist,level){
  const cv=document.getElementById(id),c=cv.getContext('2d');
  const w=cv.width,h=cv.height,cx=w/2,cy=h-8,maxR=h-12;
  c.clearRect(0,0,w,h); c.fillStyle='#0d1520'; c.fillRect(0,0,w,h);
  const radii=[maxR,maxR*.72,maxR*.50,maxR*.30,maxR*.14];
  for(let i=0;i<4;i++){
    c.beginPath(); c.moveTo(cx,cy);
    c.arc(cx,cy,radii[i],Math.PI,2*Math.PI);
    c.arc(cx,cy,radii[i+1],2*Math.PI,Math.PI,true);
    c.closePath();
    c.fillStyle=level>=(4-i)?ARC_COLORS[i]:'#1a2535'; c.fill();
    c.strokeStyle='#0a0e13'; c.lineWidth=1.5; c.stroke();
  }
  c.beginPath(); c.arc(cx,cy,5,0,2*Math.PI); c.fillStyle='#3b82f6'; c.fill();
}
function drawSteer(deg){
  const cv=document.getElementById('steer'),c=cv.getContext('2d');
  const w=cv.width,h=cv.height,cx=w/2,cy=h-10,R=h-20;
  c.clearRect(0,0,w,h); c.fillStyle='#0d1520'; c.fillRect(0,0,w,h);
  c.beginPath(); c.arc(cx,cy,R,Math.PI,2*Math.PI);
  c.strokeStyle='#1e2d3d'; c.lineWidth=12; c.stroke();
  c.beginPath(); c.arc(cx,cy,R,Math.PI,Math.PI*1.5);
  c.strokeStyle='#7c3aed'; c.lineWidth=12; c.stroke();
  c.beginPath(); c.arc(cx,cy,R,Math.PI*1.5,2*Math.PI);
  c.strokeStyle='#2563eb'; c.lineWidth=12; c.stroke();
  const maxD=30,a=Math.PI+(Math.max(-maxD,Math.min(maxD,deg))/maxD)*(Math.PI/2);
  const nx=cx+R*Math.cos(a),ny=cy+R*Math.sin(a);
  const col=deg===0?'#5a7a96':deg>0?'#3b82f6':'#8b5cf6';
  c.beginPath(); c.moveTo(cx,cy); c.lineTo(nx,ny);
  c.strokeStyle=col; c.lineWidth=3; c.stroke();
  c.beginPath(); c.arc(cx,cy,5,0,2*Math.PI); c.fillStyle=col; c.fill();
  c.fillStyle='#5a7a96'; c.font='10px system-ui'; c.textAlign='center';
  c.fillText('-30°',cx-R+6,cy+2); c.fillText('0°',cx,cy-R-4); c.fillText('+30°',cx+R-6,cy+2);
}
function updateSensor(p,dist,level){
  drawArc('arc'+p.toUpperCase(),dist,level);
  const dn=document.getElementById(p+'dist');
  dn.textContent=dist<0?'—':dist.toFixed(1); dn.style.color=LV_COLS[level];
  const lb=document.getElementById(p+'label');
  lb.textContent=LV_NAMES[level]; lb.style.color=LV_COLS[level];
  for(let i=1;i<=4;i++) document.getElementById(p+'l'+i).classList.toggle('off',level<i);
}
const MC=document.getElementById('map');
const mctx=MC.getContext('2d');
let trail=[{x:0,y:0}]; let scale=50;
function drawMap(curX,curY,yawDeg){
  const w=MC.width,h=MC.height,cx=w/2,cy=h/2;
  mctx.clearRect(0,0,w,h); mctx.fillStyle='#0d1520'; mctx.fillRect(0,0,w,h);
  mctx.strokeStyle='#1e2d3d'; mctx.lineWidth=1;
  for(let g=-8;g<=8;g++){
    mctx.beginPath();mctx.moveTo(cx+g*scale,0);mctx.lineTo(cx+g*scale,h);mctx.stroke();
    mctx.beginPath();mctx.moveTo(0,cy+g*scale);mctx.lineTo(w,cy+g*scale);mctx.stroke();
  }
  mctx.strokeStyle='#2d4a63';mctx.lineWidth=1.5;
  mctx.beginPath();mctx.moveTo(0,cy);mctx.lineTo(w,cy);mctx.stroke();
  mctx.beginPath();mctx.moveTo(cx,0);mctx.lineTo(cx,h);mctx.stroke();
  if(trail.length>1){
    mctx.beginPath();
    mctx.moveTo(cx+trail[0].x*scale,cy-trail[0].y*scale);
    for(let i=1;i<trail.length;i++) mctx.lineTo(cx+trail[i].x*scale,cy-trail[i].y*scale);
    mctx.strokeStyle='#3b82f6';mctx.lineWidth=2;mctx.stroke();
  }
  const px=cx+curX*scale,py=cy-curY*scale,yr=yawDeg*Math.PI/180;
  mctx.beginPath();mctx.arc(px,py,6,0,2*Math.PI);mctx.fillStyle='#22c55e';mctx.fill();
  mctx.beginPath();mctx.moveTo(px,py);mctx.lineTo(px+18*Math.sin(yr),py-18*Math.cos(yr));
  mctx.strokeStyle='#22c55e';mctx.lineWidth=2.5;mctx.stroke();
  mctx.beginPath();mctx.arc(cx,cy,4,0,2*Math.PI);mctx.fillStyle='#f59e0b';mctx.fill();
  mctx.fillStyle='#5a7a96';mctx.font='10px system-ui';mctx.textAlign='left';
  mctx.fillText('1grid='+(100/scale*10).toFixed(0)+'cm',6,h-5);
}
let lastLogs=[];
function updateLog(logs){
  if(JSON.stringify(logs)===JSON.stringify(lastLogs)) return;
  lastLogs=logs;
  document.getElementById('logList').innerHTML=logs.map(l=>{
    let cls='';
    if(l.includes('Sağa')) cls='right';
    else if(l.includes('Sola')) cls='left';
    else if(l.includes('TEHLİKE')) cls='danger';
    return '<div class="log-entry '+cls+'">'+l+'</div>';
  }).join('');
}
function updateMotorUI(d){
  const mb=document.getElementById('mstatebadge');
  const note=document.getElementById('motorNote');
  mb.textContent=d.mstate;
  mb.className='badge '+(d.mstate==='ARMED'?'armed':d.mstate==='ARMING'?'arming':'disarmed');
  if(d.mstate==='ARMED') note.textContent='ARMED — otonom sürüş aktif';
  else if(d.mstate==='ARMING') note.textContent='ARM ediliyor... ESC nötr sinyali';
  else note.textContent='DISARMED — ARM için tıkla';
  document.getElementById('msteer').textContent=(d.steerPct>=0?'+':'')+d.steerPct+'%';
  document.getElementById('mthr').textContent=(d.thrPct>=0?'+':'')+d.thrPct+'%';
  document.getElementById('mesc').textContent=d.escUs+'µs';
}
function updateLimits(){
  const lv=document.getElementById('limSlider').value;
  const av=document.getElementById('autoSlider').value;
  document.getElementById('limVal').textContent=lv;
  document.getElementById('autoVal').textContent=av;
  clearTimeout(limitTimer);
  limitTimer=setTimeout(()=>fetch('/setlimit?v='+lv+'&a='+av,{cache:'no-store'}),300);
}
async function armCar()   { await fetch('/arm',{cache:'no-store'}); }
async function disarmCar(){ await fetch('/disarm',{cache:'no-store'}); }
async function tick(){
  try{
    const d=await(await fetch('/status',{cache:'no-store'})).json();
    document.getElementById('badge').textContent=d.ok?'IMU OK':'IMU ERR';
    document.getElementById('badge').className='badge '+(d.ok?'okb':'badb');
    document.getElementById('calbadge').textContent=d.cal?'CALIBRATED':'CALIBRATING...';
    document.getElementById('calbadge').className='badge '+(d.cal?'calok':'calno');
    document.getElementById('ax').textContent=d.ax.toFixed(2);
    document.getElementById('ay').textContent=d.ay.toFixed(2);
    document.getElementById('az').textContent=d.az.toFixed(2);
    document.getElementById('gx').textContent=d.gx.toFixed(1);
    document.getElementById('gy').textContent=d.gy.toFixed(1);
    document.getElementById('gz').textContent=d.gz.toFixed(1);
    document.getElementById('groll').textContent=d.roll.toFixed(1);
    document.getElementById('gpitch').textContent=d.pitch.toFixed(1);
    document.getElementById('gyaw').textContent=d.yaw.toFixed(1);
    document.getElementById('xyval').textContent='x='+d.x.toFixed(3)+'m  y='+d.y.toFixed(3)+'m';
    updateSensor('l',d.lCm,d.lLv);
    updateSensor('r',d.rCm,d.rLv);
    drawSteer(d.steer);
    const sdeg=document.getElementById('sdeg'),slbl=document.getElementById('slabel');
    sdeg.textContent=(d.steer>=0?'+':'')+d.steer.toFixed(1)+'°';
    if(Math.abs(d.steer)<0.5){sdeg.style.color='var(--mut)';slbl.textContent='Düz Git';slbl.style.color='var(--mut)';}
    else if(d.steer>0){sdeg.style.color='#3b82f6';slbl.textContent='→ Sağa Dön';slbl.style.color='#3b82f6';}
    else{sdeg.style.color='#8b5cf6';slbl.textContent='← Sola Dön';slbl.style.color='#8b5cf6';}
    const last=trail[trail.length-1];
    if(Math.abs(d.x-last.x)>0.001||Math.abs(d.y-last.y)>0.001) trail.push({x:d.x,y:d.y});
    if(trail.length>400) trail.shift();
    const maxD=trail.reduce((m,p)=>Math.max(m,Math.abs(p.x),Math.abs(p.y)),0.1);
    scale=Math.min(150,(MC.height/2-20)/maxD);
    drawMap(d.x,d.y,d.yaw);
    updateLog(d.logs);
    updateMotorUI(d);
  }catch(e){}
}
async function resetPos(){ await fetch('/reset',{cache:'no-store'}); trail=[{x:0,y:0}]; }
setInterval(tick,120);
</script>
</body></html>
)HTML";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

// ========================= SETUP ============================================
void setup() {
  Serial.begin(115200); delay(300);
  Serial.println("\n========================================");
  Serial.println("  Faz 3.5 — ObstacleNav + IMU + Motor");
  Serial.println("========================================");

  // Servo / ESC başlat — açılışta nötr
  servo.attach(PIN_SERVO, SERVO_MIN, SERVO_MAX);
  esc.attach(PIN_ESC, ESC_MIN_US, ESC_MAX_US);
  servo.writeMicroseconds(SERVO_CENTER);
  esc.writeMicroseconds(ESC_NEUTRAL);
  Serial.printf("[MOTOR] Servo→GPIO%d  ESC→GPIO%d  DISARMED\n", PIN_SERVO, PIN_ESC);

  // IMU başlat — 0x68 veya 0x69 otomatik algıla
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000);
  mpuFound = mpuWake();
  if (mpuFound) {
    Serial.printf("[IMU] 0x%02X — kalibrasyon başlıyor...\n", MPU_ADDR);
    calibrate();
  } else {
    Serial.println("[IMU] BULUNAMADI — devam ediliyor (sonar + motor aktif)");
  }

  // Sonar pinler
  pinMode(PIN_TRIG_L, OUTPUT); digitalWrite(PIN_TRIG_L, LOW);
  pinMode(PIN_TRIG_R, OUTPUT); digitalWrite(PIN_TRIG_R, LOW);
  pinMode(PIN_ECHO_L, INPUT);
  pinMode(PIN_ECHO_R, INPUT);
  xTaskCreatePinnedToCore(sonarTask, "sonar", 4096, NULL, 1, NULL, 0);
  Serial.printf("[SONAR] Sol TRIG→GPIO%d ECHO→GPIO%d\n", PIN_TRIG_L, PIN_ECHO_L);
  Serial.printf("[SONAR] Sağ TRIG→GPIO%d ECHO→GPIO%d\n", PIN_TRIG_R, PIN_ECHO_R);

  // WiFi — ağ taraması + bağlantı
  WiFi.mode(WIFI_STA);
  Serial.println("[WiFi] Yakındaki ağlar taranıyor...");
  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("[WiFi] Hiç ağ bulunamadı!");
  } else {
    Serial.printf("[WiFi] %d ağ:\n", n);
    bool found = false;
    for (int i = 0; i < n; i++) {
      bool match = (WiFi.SSID(i) == String(WIFI_SSID));
      Serial.printf("  [%d] %-28s %4d dBm ch%2d %s%s\n",
        i+1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
        WiFi.encryptionType(i)==WIFI_AUTH_OPEN?"OPEN":"ENC",
        match?" <<<< HEDEF":"");
      if (match) found = true;
    }
    if (!found) Serial.printf("[WiFi] UYARI: '%s' listede YOK!\n", WIFI_SSID);
  }
  WiFi.scanDelete();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] '%s' bağlanılıyor...\n", WIFI_SSID);
  unsigned long t0 = millis();
  int lastSt = -1;
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<15000) {
    delay(500);
    int st = WiFi.status();
    if (st != lastSt) {
      const char* sn = st==WL_NO_SSID_AVAIL?"NO_SSID":st==WL_CONNECT_FAILED?"FAILED":
                       st==WL_DISCONNECTED?"DISCONNECTED":"...";
      Serial.printf("[WiFi] %s (%d)  %.1fs\n", sn, st, (millis()-t0)/1000.0f);
      lastSt = st;
    } else { Serial.print("."); }
  }
  Serial.println();
  if (WiFi.status()==WL_CONNECTED)
    Serial.printf("[WiFi] BAĞLANDI! http://%s:%d/\n", WiFi.localIP().toString().c_str(), WEB_PORT);
  else
    Serial.printf("[WiFi] BAŞARISIZ — son durum: %d\n", WiFi.status());

  server.on("/",        handleRoot);
  server.on("/status",  handleStatus);
  server.on("/reset",   handleReset);
  server.on("/arm",     handleArm);
  server.on("/disarm",  handleDisarm);
  server.on("/setlimit",handleSetLimit);
  server.onNotFound([](){ server.send(404,"text/plain","not found"); });
  server.begin();
  Serial.println("[HAZIR] — Web UI aktif, DISARMED");
  lastLoopUs = micros();
}

// ========================= LOOP (Core 1) ====================================
void loop() {
  server.handleClient();

  // Arming state machine
  if (motorState == ARMING && millis()-armStartMs >= ARM_DURATION_MS) {
    motorState = ARMED;
    Serial.println("[STATE] ARMED — otonom sürüş başladı");
  }

  // Otonom sürüş
  autonomousDrive();

  // IMU okuma
  unsigned long now = millis();
  if (now - lastReadMs >= READ_INTERVAL_MS) {
    lastReadMs = now;
    unsigned long nowUs = micros();
    float dt = (nowUs - lastLoopUs) * 1e-6f;
    lastLoopUs = nowUs;
    if (dt > 0.1f) dt = 0.02f;
    if (!mpuFound) { mpuFound = mpuWake(); if (mpuFound) calibrate(); return; }
    ImuRaw raw;
    if (mpuReadRaw(raw)) {
      raw.ax-=bias_ax; raw.ay-=bias_ay; raw.az-=bias_az;
      raw.gx-=bias_gx; raw.gy-=bias_gy; raw.gz-=bias_gz;
      imu = raw;
      if (calibrated) updateFusion(dt);
    } else { imu.ok=false; mpuFound=false; }
  }
}
