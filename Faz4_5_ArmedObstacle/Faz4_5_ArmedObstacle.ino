/*
 * ============================================================================
 *  Faz 4.5 — Armed Obstacle Navigation + Manual Drive  (ESP32-S3)
 * ============================================================================
 *  KABLOLAMA:
 *    MPU6050:   SDA→GPIO8  SCL→GPIO9  VCC→3.3V  GND→GND
 *    Sol SR04:  TRIG→GPIO10  ECHO→GPIO11  VCC→5V  GND→GND
 *    Sağ SR04:  TRIG→GPIO12  ECHO→GPIO13  VCC→5V  GND→GND
 *    Servo:     Sinyal→GPIO4   GND→GND   VCC→ESC BEC (5V)
 *    ESC:       Sinyal→GPIO5   GND→GND   (güç bataryadan)
 *    ⚠ ECHO 5V→ESP32: voltaj bölücü (1kΩ + 2kΩ)
 *    ⚠ Ortak GND: batarya(-) + ESC siyah + ESP32 GND hepsi birleşmeli
 *
 *  MODLAR:
 *    AUTO   — sonar verisiyle otonom engel kaçınma
 *    MANUAL — web UI'dan klavye / joystick ile kullanıcı kontrolü
 *
 *  GÜVENLİK:
 *    - Açılışta DISARMED
 *    - 3 sn nötr → ARMED (ESC arming)
 *    - L1 engel (<8 cm) → DISARMED (AUTO modda)
 *    - MANUAL modda 1.5 s komut gelmezse → motor nötr (ARMED kalır)
 *    - Sonar 1 s susarsa → DISARMED
 *  WEB:  http://<IP>:5003/
 * ============================================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <math.h>

// ========================= WiFi =============================================
const char* WIFI_SSID = "Hezer";
const char* WIFI_PASS = "burakhezer";
const int   WEB_PORT  = 5003;

// ========================= Motor Pinler / PWM ================================
const int PIN_SERVO    = 4;
const int PIN_ESC      = 5;
const int SERVO_MIN    = 2000;
const int SERVO_CENTER = 2500;
const int SERVO_MAX    = 3000;
// Sağa basınca ters dönüyorsa -1 yap
const int SERVO_DIR    = 1;
const int ESC_MIN_US   = 1000;
const int ESC_MAX_US   = 1650;
const int ESC_NEUTRAL  = 1500;
// İleri bastığında geri dönüyorsa -1 yap, doğruysa +1 bırak
const int ESC_DIR      = -1;

const unsigned long ARM_DURATION_MS = 2000;
const unsigned long MANUAL_WDOG_MS  = 1500;  // komut gelmezse nötr
const unsigned long SONAR_WDOG_MS   = 2000;  // WiFi gecikmelerine karşı geniş tutuluyor

int throttleLimit = 40;   // her iki modda da hard cap (%)
int autoThrottle  = 30;   // AUTO modda sabit ileri hız — DOĞRUDAN PWM % → 1500+150=1650µs

// ========================= IMU Pinler / Sabitler ============================
const int PIN_SDA = 8;
const int PIN_SCL = 9;

uint8_t       MPU_ADDR        = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_XOUT = 0x3B;
const float   ACCEL_SCALE    = 16384.0f;
const float   GYRO_SCALE     = 131.0f;
const float   G_MS2          = 9.81f;
const unsigned long READ_INTERVAL_MS = 20;
const int           CAL_SAMPLES      = 300;
const float         CF_ALPHA         = 0.98f;

// ========================= Sonar Sabitler ===================================
const int   PIN_TRIG_L = 10, PIN_ECHO_L = 11;
const int   PIN_TRIG_R = 12, PIN_ECHO_R = 13;
const float PROX_L1 =  8.0f, PROX_L2 = 20.0f;
const float PROX_L3 = 50.0f, PROX_L4 = 75.0f;
const float MAX_DIST = 150.0f, STEER_GAIN = 0.4f;
const float MAX_STEER_DEG = 30.0f, DEADBAND_CM = 8.0f;
const unsigned long SONAR_TASK_MS = 60;

// ========================= IMU Durum ========================================
struct ImuRaw { float ax,ay,az,gx,gy,gz,temp; bool ok; };
ImuRaw imu = {};
bool   mpuFound=false, calibrated=false;
float  bias_ax=0,bias_ay=0,bias_az=0,bias_gx=0,bias_gy=0,bias_gz=0;
float  roll=0,pitch=0,yaw=0,dr_x=0,dr_y=0,dr_vx=0,dr_vy=0;
unsigned long lastReadMs=0, lastLoopUs=0;

// ========================= Sonar Durum ======================================
portMUX_TYPE sonarMux = portMUX_INITIALIZER_UNLOCKED;
float leftCm=-1, rightCm=-1; int leftLevel=0, rightLevel=0; float steerDeg=0;
unsigned long sonarLastUpdMs=0;

const int LOG_SIZE=12; String logBuf[LOG_SIZE]; int logIdx=0;

// ========================= Motor / Mod Durum ================================
enum MotorState { DISARMED, ARMING, ARMED };
enum DriveMode  { AUTO, MANUAL };
volatile MotorState motorState = DISARMED;
DriveMode driveMode = AUTO;
unsigned long armStartMs=0, lastManualCmdMs=0;
bool     pendingAutoRearm=false;
unsigned long autoDisarmMs=0;
const unsigned long AUTO_REARM_DELAY_MS=2000;
int curSteerPct=0, curThrottlePct=0, appliedEscUs=ESC_NEUTRAL, appliedServoUs=SERVO_CENTER;

Servo servo; Servo esc;
WebServer server(WEB_PORT);

// ========================= IMU =============================================
bool mpuWake() {
  for (uint8_t addr : {(uint8_t)0x68,(uint8_t)0x69}) {
    Wire.beginTransmission(addr); Wire.write(REG_PWR_MGMT_1); Wire.write(0x00);
    if (Wire.endTransmission()==0) { MPU_ADDR=addr; Serial.printf("[IMU] 0x%02X bulundu\n",addr); return true; }
  }
  Serial.print("[IMU] Bulunamadı. I2C tarama: ");
  bool any=false;
  for (uint8_t a=1;a<127;a++) { Wire.beginTransmission(a); if(Wire.endTransmission()==0){Serial.printf("0x%02X ",a);any=true;} }
  if(!any) Serial.print("HİÇ CİHAZ YOK");
  Serial.println(); return false;
}
bool mpuReadRaw(ImuRaw& d) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(REG_ACCEL_XOUT);
  if (Wire.endTransmission(false)!=0){d.ok=false;return false;}
  if (Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14)<14){d.ok=false;return false;}
  auto rw=[]()->int16_t{return(int16_t)((Wire.read()<<8)|Wire.read());};
  d.ax=rw()/ACCEL_SCALE*G_MS2; d.ay=rw()/ACCEL_SCALE*G_MS2; d.az=rw()/ACCEL_SCALE*G_MS2;
  d.temp=rw()/340.0f+36.53f;
  d.gx=rw()/GYRO_SCALE; d.gy=rw()/GYRO_SCALE; d.gz=rw()/GYRO_SCALE;
  d.ok=true; return true;
}
void calibrate() {
  Serial.printf("[CAL] %d örnek...\n",CAL_SAMPLES);
  double sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0; int n=0; ImuRaw d;
  while(n<CAL_SAMPLES){if(mpuReadRaw(d)){sax+=d.ax;say+=d.ay;saz+=d.az;sgx+=d.gx;sgy+=d.gy;sgz+=d.gz;n++;}delay(READ_INTERVAL_MS);}
  float mx=sax/n,my=say/n,mz=saz/n,abx=fabsf(mx),aby=fabsf(my),abz=fabsf(mz);
  float gsx=0,gsy=0,gsz=0;
  if(abx>=aby&&abx>=abz) gsx=(mx>=0)?1:-1;
  else if(aby>=abx&&aby>=abz) gsy=(my>=0)?1:-1;
  else gsz=(mz>=0)?1:-1;
  bias_ax=mx-gsx*G_MS2; bias_ay=my-gsy*G_MS2; bias_az=mz-gsz*G_MS2;
  bias_gx=sgx/n; bias_gy=sgy/n; bias_gz=sgz/n;
  float ax0=mx-bias_ax,ay0=my-bias_ay,az0=mz-bias_az;
  roll=atan2f(ay0,az0)*180.0f/M_PI;
  pitch=atan2f(-ax0,sqrtf(ay0*ay0+az0*az0))*180.0f/M_PI;
  yaw=0; dr_x=dr_y=dr_vx=dr_vy=0; calibrated=true;
  Serial.printf("[CAL] Tamam. bias_a=(%.3f,%.3f,%.3f)\n",bias_ax,bias_ay,bias_az);
}
void updateFusion(float dt) {
  float ax=imu.ax,ay=imu.ay,az=imu.az,gx=imu.gx,gy=imu.gy,gz=imu.gz;
  roll =CF_ALPHA*(roll +gx*dt)+(1-CF_ALPHA)*atan2f(ay,az)*180.0f/M_PI;
  pitch=CF_ALPHA*(pitch+gy*dt)+(1-CF_ALPHA)*atan2f(-ax,sqrtf(ay*ay+az*az))*180.0f/M_PI;
  yaw+=gz*dt; if(yaw>180)yaw-=360; if(yaw<-180)yaw+=360;
  float yr=yaw*M_PI/180.0f;
  float axw=ax*cosf(yr)-ay*sinf(yr), ayw=ax*sinf(yr)+ay*cosf(yr);
  if(fabsf(axw)<0.15f)axw=0; if(fabsf(ayw)<0.15f)ayw=0;
  dr_vx+=axw*dt; dr_vy+=ayw*dt; dr_vx*=0.98f; dr_vy*=0.98f;
  dr_x+=dr_vx*dt; dr_y+=dr_vy*dt;
}

// ========================= Sonar ============================================
int proxLevel(float cm){if(cm<0)return 0;if(cm<PROX_L1)return 1;if(cm<PROX_L2)return 2;if(cm<PROX_L3)return 3;if(cm<PROX_L4)return 4;return 0;}
float computeSteering(float l,float r){
  float lv=(l>0&&l<MAX_DIST)?l:MAX_DIST, rv=(r>0&&r<MAX_DIST)?r:MAX_DIST;
  if(lv>=MAX_DIST&&rv>=MAX_DIST)return 0;
  float diff=rv-lv; if(fabsf(diff)<DEADBAND_CM)return 0;
  return constrain(STEER_GAIN*diff,-MAX_STEER_DEG,MAX_STEER_DEG);
}
String levelName(int lv){const char* n[]={"--","L1","L2","L3","L4"};return(lv>=0&&lv<=4)?n[lv]:"--";}
void addLog(const String& msg){logBuf[logIdx%LOG_SIZE]=msg;logIdx++;}
float measureSonar(int trig,int echo){
  digitalWrite(trig,LOW);delayMicroseconds(2);
  digitalWrite(trig,HIGH);delayMicroseconds(10);digitalWrite(trig,LOW);
  unsigned long dur=pulseIn(echo,HIGH,20000);
  return dur==0?-1.0f:dur*0.01716f;
}
void sonarTask(void*){
  for(;;){
    float lCm=measureSonar(PIN_TRIG_L,PIN_ECHO_L); vTaskDelay(pdMS_TO_TICKS(10));
    float rCm=measureSonar(PIN_TRIG_R,PIN_ECHO_R);
    int lLv=proxLevel(lCm),rLv=proxLevel(rCm); float st=computeSteering(lCm,rCm);
    portENTER_CRITICAL(&sonarMux);
    float prevSt=steerDeg; leftCm=lCm;rightCm=rCm;leftLevel=lLv;rightLevel=rLv;steerDeg=st;sonarLastUpdMs=millis();
    portEXIT_CRITICAL(&sonarMux);
    if(lLv==1||rLv==1) addLog("TEHLİKE! Sol:L"+String(lLv)+" Sağ:L"+String(rLv)+" → "+(st>0?"Sağa ":st<0?"Sola ":"DUR ")+String(st,1)+"°");
    else if(fabsf(st-prevSt)>1.5f){
      String dir=fabsf(st)<0.5f?"Düz git":(st>0?"→ Sağa "+String(st,1)+"°":"← Sola "+String(fabsf(st),1)+"°");
      addLog("Sol:"+(lCm<0?String("---"):String(lCm,1))+"["+levelName(lLv)+"] Sağ:"+(rCm<0?String("---"):String(rCm,1))+"["+levelName(rLv)+"] "+dir);
    }
    vTaskDelay(pdMS_TO_TICKS(SONAR_TASK_MS));
  }
}

// ========================= Motor Kontrol =====================================
void writeServoPct(int pct){
  pct=constrain(pct,-100,100);
  int us=SERVO_CENTER+SERVO_DIR*(int)((long)pct*(SERVO_MAX-SERVO_MIN)/2/100);
  us=constrain(us,SERVO_MIN,SERVO_MAX);
  servo.writeMicroseconds(us);
  curSteerPct=pct; appliedServoUs=us;
}
void writeEscNeutral(){esc.writeMicroseconds(ESC_NEUTRAL);appliedEscUs=ESC_NEUTRAL;curThrottlePct=0;}
void writeThrottlePct(int pct){
  pct=constrain(pct,-throttleLimit,throttleLimit);
  curThrottlePct=pct;
  if(motorState!=ARMED){writeEscNeutral();return;}

  // ESC yön düzeltmesi: ESC_DIR=-1 ise ileri/geri tersine çevrilir
  int dirPct = pct * ESC_DIR;

  // Araba ESC geri protokolü: ilk geri sinyali=fren, 200ms sonra gerçek geri
  static bool brakePhase = false;
  static unsigned long brakeStartMs = 0;
  if(dirPct < 0){
    if(!brakePhase){ brakePhase=true; brakeStartMs=millis(); esc.writeMicroseconds(ESC_NEUTRAL); appliedEscUs=ESC_NEUTRAL; return; }
    if(millis()-brakeStartMs < 200){ esc.writeMicroseconds(ESC_NEUTRAL); appliedEscUs=ESC_NEUTRAL; return; }
    // 200ms geçti → geri sinyal gönder
  } else {
    brakePhase=false;
  }

  int us=ESC_NEUTRAL+(int)((long)dirPct*(ESC_MAX_US-ESC_MIN_US)/2/100);
  esc.writeMicroseconds(constrain(us,ESC_MIN_US,ESC_MAX_US));
  appliedEscUs=us;
}
void disarm(const char* r){
  motorState=DISARMED; writeEscNeutral(); writeServoPct(0);
  Serial.printf("[STATE] DISARMED | %s\n",r);
  // Manuel disarm veya sonar timeout dışında: engel geçince otomatik re-arm
  if(driveMode==AUTO && strcmp(r,"web")!=0 && strcmp(r,"sonar-timeout")!=0){
    pendingAutoRearm=true; autoDisarmMs=millis();
  } else {
    pendingAutoRearm=false;
  }
}
void beginArming(){
  if(motorState!=DISARMED)return;
  motorState=ARMING;armStartMs=millis();writeEscNeutral();
  Serial.printf("[STATE] ARMING | %lu ms nötr...\n",ARM_DURATION_MS);
}
const char* stateName(){switch(motorState){case DISARMED:return"DISARMED";case ARMING:return"ARMING";case ARMED:return"ARMED";}return"?";}
const char* modeName(){return driveMode==AUTO?"AUTO":"MANUAL";}

// ========================= Otonom Sürüş (AUTO modu) =========================
void autonomousDrive(){
  if(motorState!=ARMED||driveMode!=AUTO)return;

  // 50 ms'den daha sık ESC yazma — gereksiz çağrıları filtreler
  static unsigned long lastAutoMs=0;
  if(millis()-lastAutoMs < 50)return;
  lastAutoMs=millis();

  float lCm,rCm,st;int lLv,rLv;unsigned long age;
  portENTER_CRITICAL(&sonarMux);lCm=leftCm;rCm=rightCm;lLv=leftLevel;rLv=rightLevel;st=steerDeg;age=millis()-sonarLastUpdMs;portEXIT_CRITICAL(&sonarMux);

  if(age>SONAR_WDOG_MS){disarm("sonar-timeout");return;}

  // Her iki taraf L1 = gerçekten sıkışık, dur
  if(lLv==1 && rLv==1){ disarm("L1-her-iki-yan"); writeServoPct(0); return; }

  // Tek taraf L1 = o taraftan kaç, yavaş devam et (disarm etme)
  if(lLv==1 || rLv==1){
    writeServoPct(lLv==1 ? 100 : -100);          // L1 olan tarafın tersine steer
    writeThrottlePct(max(autoThrottle*40/100,15)); // düşük hız ama deadband üstünde
    return;
  }

  // Normal sürüş: sonar açısıyla steer, sabit hız
  // (L4/L2 farkı olsa da durmaz — steer yeterli)
  writeServoPct(constrain((int)(st/MAX_STEER_DEG*100),-100,100));
  writeThrottlePct(autoThrottle);
}

// ========================= HTTP =============================================
void handleArm(){beginArming();server.send(200,"application/json","{\"ok\":true}");}
void handleDisarm(){disarm("web");server.send(200,"application/json","{\"ok\":true}");}
void handleMode(){
  if(server.hasArg("m")){
    String m=server.arg("m");
    driveMode=(m=="manual")?MANUAL:AUTO;
    if(driveMode==MANUAL)lastManualCmdMs=millis();
    else{writeEscNeutral();writeServoPct(0);}
    Serial.printf("[MODE] %s\n",modeName());
  }
  server.send(200,"application/json","{\"ok\":true}");
}
void handleControl(){
  if(motorState==ARMED&&driveMode==MANUAL){
    lastManualCmdMs=millis();
    int steer=server.hasArg("s")?server.arg("s").toInt():0;
    int thr  =server.hasArg("t")?server.arg("t").toInt():0;
    writeServoPct(steer);
    writeThrottlePct(thr);
  }
  server.send(200,"application/json","{\"ok\":true}");
}
void handleSetLimit(){
  if(server.hasArg("v"))throttleLimit=constrain(server.arg("v").toInt(),5,60);
  if(server.hasArg("a"))autoThrottle =constrain(server.arg("a").toInt(),5,50);
  server.send(200,"application/json","{\"ok\":true}");
}
void handleStatus(){
  float lCm,rCm,st;int lLv,rLv;
  portENTER_CRITICAL(&sonarMux);lCm=leftCm;rCm=rightCm;lLv=leftLevel;rLv=rightLevel;st=steerDeg;portEXIT_CRITICAL(&sonarMux);
  String logs="[";
  for(int i=0;i<LOG_SIZE;i++){int idx=((logIdx-1-i)%LOG_SIZE+LOG_SIZE)%LOG_SIZE;if(logBuf[idx].length()==0)continue;if(logs.length()>1)logs+=",";logs+="\""+logBuf[idx]+"\"";}
  logs+="]";
  char buf[1100];
  snprintf(buf,sizeof(buf),
    "{\"lCm\":%.1f,\"rCm\":%.1f,\"lLv\":%d,\"rLv\":%d,\"steer\":%.2f,"
    "\"imuOk\":%s,\"cal\":%s,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
    "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
    "\"x\":%.4f,\"y\":%.4f,\"temp\":%.1f,"
    "\"mstate\":\"%s\",\"mode\":\"%s\","
    "\"steerPct\":%d,\"thrPct\":%d,\"escUs\":%d,\"servoUs\":%d,"
    "\"limit\":%d,\"auto\":%d,"
    "\"logs\":%s}",
    lCm,rCm,lLv,rLv,st,
    imu.ok?"true":"false",calibrated?"true":"false",
    imu.ax,imu.ay,imu.az,imu.gx,imu.gy,imu.gz,
    roll,pitch,yaw,dr_x,dr_y,imu.temp,
    stateName(),modeName(),
    curSteerPct,curThrottlePct,appliedEscUs,appliedServoUs,
    throttleLimit,autoThrottle,
    logs.c_str());
  server.send(200,"application/json",buf);
}
void handleReset(){yaw=0;dr_x=dr_y=dr_vx=dr_vy=0;server.send(200,"application/json","{\"ok\":true}");}

// ========================= HTML =============================================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Armed ObstacleNav — Faz 4.5</title>
<style>
:root{--bg:#0a0e13;--card:#111820;--line:#1e2d3d;--txt:#dde6f0;--mut:#5a7a96;
      --ok:#22c55e;--bad:#ef4444;--warn:#f59e0b;
      --p0:#5a7a96;--p1:#ef4444;--p2:#f97316;--p3:#eab308;--p4:#22c55e}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font:13px/1.5 system-ui,sans-serif;padding:14px;max-width:680px;margin:0 auto}
h1{font-size:16px;font-weight:700;margin-bottom:2px}
.sub{font-size:11px;color:var(--mut);margin-bottom:10px}
.row-wrap{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:10px}
.badge{padding:3px 10px;border-radius:99px;font-size:11px;font-weight:700}
.okb{background:#14532d;color:#86efac}.badb{background:#450a0a;color:#fca5a5}
.calok{background:#2e1065;color:#c4b5fd}.calno{background:#1c1917;color:#78716c}
.armed{background:#14532d;color:#86efac}.arming{background:#3a2e00;color:#fde68a}
.disarmed{background:#1c1917;color:#78716c}
.mode-auto{background:#1e3a5f;color:#93c5fd}.mode-manual{background:#3b1f5e;color:#c4b5fd}
.btn{background:#1e2d3d;border:1px solid var(--line);color:var(--txt);padding:5px 14px;border-radius:6px;font-size:12px;cursor:pointer;font-weight:600}
.btn:hover{background:#253545}
.btn-arm{background:#14532d;border-color:#166534;color:#86efac}
.btn-arm:hover{background:#166534}
.btn-disarm{background:#450a0a;border-color:#7f1d1d;color:#fca5a5}
.btn-disarm:hover{background:#7f1d1d}
.btn-auto{background:#1e3a5f;border-color:#1d4ed8;color:#93c5fd}
.btn-manual{background:#3b1f5e;border-color:#6d28d9;color:#c4b5fd}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:10px}
.card-title{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;letter-spacing:.8px;margin-bottom:10px}
.stat-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
.stat{text-align:center;background:#0d1520;border-radius:8px;padding:8px 4px}
.stat-lbl{font-size:10px;color:var(--mut)}
.stat-val{font-size:20px;font-weight:700;font-family:monospace;margin:2px 0}
.stat-unit{font-size:10px;color:var(--mut)}
/* Manual joystick */
.joy-wrap{display:flex;gap:16px;align-items:flex-start;flex-wrap:wrap}
.joy-area{position:relative;width:180px;height:180px;background:#0d1520;border-radius:12px;border:1px solid var(--line);touch-action:none;flex-shrink:0}
.joy-dot{position:absolute;width:44px;height:44px;border-radius:50%;background:#6d28d9;transform:translate(-50%,-50%);top:50%;left:50%;transition:background .15s}
.joy-cross{position:absolute;inset:0;pointer-events:none}
.key-hints{font-size:11px;color:var(--mut);line-height:1.9}
.key-hints kbd{background:#1e2d3d;border:1px solid #2d4a63;border-radius:4px;padding:1px 6px;font-family:monospace;font-size:11px;color:var(--txt)}
/* Sliders */
.slider-row{display:flex;align-items:center;gap:8px;margin-top:6px}
.slider-row label{font-size:11px;color:var(--mut);white-space:nowrap;min-width:130px}
input[type=range]{flex:1;accent-color:#6d28d9}
/* Sensors */
.sensors{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.sensor-card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px;text-align:center}
.sensor-card h2{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;letter-spacing:.8px;margin-bottom:8px}
.dist-num{font-size:26px;font-weight:700;font-family:monospace;transition:color .2s}
.level-lbl{font-size:11px;font-weight:700;margin-top:4px;transition:color .2s}
.bars{display:flex;justify-content:center;gap:4px;align-items:flex-end;height:36px;margin-top:6px}
.lb{width:16px;border-radius:2px 2px 0 0;transition:opacity .15s}
.lb-4{height:18px;background:var(--p4)}.lb-3{height:24px;background:var(--p3)}
.lb-2{height:30px;background:var(--p2)}.lb-1{height:36px;background:var(--p1)}.lb.off{opacity:.12}
.gauges3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.gauge{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px;text-align:center}
.gauge .gl{font-size:10px;color:var(--mut);text-transform:uppercase}
.gauge .gv{font-size:18px;font-weight:700;font-family:monospace;margin:4px 0}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.irow{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px solid var(--line);font-size:12px}
.irow:last-child{border:none}.ilbl{color:var(--mut)}.ival{font-family:monospace;font-weight:700}
.steer-card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:10px}
.steer-inner{display:flex;align-items:center;justify-content:center;gap:20px;flex-wrap:wrap}
.sdeg{font-size:38px;font-weight:700;font-family:monospace;transition:color .2s}
.slbl{font-size:13px;font-weight:700;margin-top:4px;transition:color .2s}
.map-card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px;margin-bottom:10px}
.map-warn{font-size:10px;color:var(--warn);margin-top:5px}
.log-card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}
.log-list{font-family:monospace;font-size:11px;max-height:130px;overflow-y:auto}
.log-entry{padding:2px 0;border-bottom:1px solid var(--line);color:var(--mut)}
.log-entry.right{color:#3b82f6}.log-entry.left{color:#8b5cf6}.log-entry.danger{color:var(--p1)}
canvas{display:block}
</style></head><body>

<h1>Armed ObstacleNav — Faz 4.5</h1>
<div class="sub">HC-SR04 GPIO10-13 · MPU6050 GPIO8-9 · Servo GPIO4 · ESC GPIO5</div>

<div class="row-wrap">
  <span id="imuBadge" class="badge badb">IMU ?</span>
  <span id="calBadge" class="badge calno">CAL?</span>
  <span id="mstateBadge" class="badge disarmed">DISARMED</span>
  <span id="modeBadge"  class="badge mode-auto">AUTO</span>
</div>

<!-- Motor + Mod -->
<div class="card">
  <div class="card-title">Motor Kontrol</div>
  <div class="row-wrap">
    <button class="btn btn-arm"    onclick="armCar()">▶ ARM (3 sn)</button>
    <button class="btn btn-disarm" onclick="disarmCar()">■ DISARM</button>
    <button id="modeBtn" class="btn btn-auto" onclick="toggleMode()">⇄ AUTO MOD</button>
    <button class="btn" onclick="resetPos()">↺ Reset IMU</button>
  </div>
  <div class="stat-grid">
    <div class="stat"><div class="stat-lbl">Direksiyon</div><div class="stat-val" id="mSteer">0</div><div class="stat-unit">%</div></div>
    <div class="stat"><div class="stat-lbl">Gaz</div><div class="stat-val" id="mThr">0</div><div class="stat-unit">%</div></div>
    <div class="stat"><div class="stat-lbl">ESC µs</div><div class="stat-val" id="mEsc">1500</div><div class="stat-unit">µs</div></div>
    <div class="stat"><div class="stat-lbl">Servo µs</div><div class="stat-val" id="mServo">2500</div><div class="stat-unit">µs</div></div>
    <div class="stat"><div class="stat-lbl">Sıcaklık</div><div class="stat-val" id="mTemp">—</div><div class="stat-unit">°C</div></div>
  </div>
  <div class="slider-row"><label>Maks Gaz Limiti <span id="limVal">35</span>%</label><input type="range" min="5" max="60" value="35" id="limSlider" oninput="sendLimits()"></div>
  <div class="slider-row"><label>AUTO Baz Gaz <span id="autoVal">25</span>%</label><input type="range" min="5" max="50" value="25" id="autoSlider" oninput="sendLimits()"></div>
</div>

<!-- Manuel Joystick -->
<div class="card" id="manualCard" style="display:none">
  <div class="card-title">Manuel Kontrol — Joystick veya Klavye</div>
  <div class="joy-wrap">
    <div class="joy-area" id="joy">
      <svg class="joy-cross" viewBox="0 0 180 180"><line x1="90" y1="10" x2="90" y2="170" stroke="#1e2d3d" stroke-width="1"/><line x1="10" y1="90" x2="170" y2="90" stroke="#1e2d3d" stroke-width="1"/></svg>
      <div class="joy-dot" id="joyDot"></div>
    </div>
    <div class="key-hints">
      <b style="color:var(--txt)">Klavye:</b><br>
      <kbd>W</kbd> / <kbd>↑</kbd> — İleri<br>
      <kbd>S</kbd> / <kbd>↓</kbd> — Geri<br>
      <kbd>A</kbd> / <kbd>←</kbd> — Sol<br>
      <kbd>D</kbd> / <kbd>→</kbd> — Sağ<br>
      <kbd>Space</kbd> — Dur<br>
      <br>
      <span style="color:var(--warn)">⚠ ARM olmadan motor çalışmaz</span>
    </div>
  </div>
</div>

<!-- Sensörler -->
<div class="steer-card">
  <div class="card-title">Yön Kararı (Sonar)</div>
  <div class="steer-inner">
    <canvas id="steerCv" width="190" height="105"></canvas>
    <div style="text-align:center">
      <div class="sdeg" id="sdeg">0.0°</div>
      <div class="slbl" id="slbl">Düz Git</div>
      <div style="font-size:10px;color:var(--mut)">+ = Sağa · − = Sola</div>
    </div>
  </div>
</div>
<div class="sensors" style="margin-bottom:10px">
  <div class="sensor-card"><h2>◀ Sol</h2><div class="dist-num" id="ldist">—</div><div style="font-size:10px;color:var(--mut)">cm</div><div class="level-lbl" id="llbl">—</div><div class="bars"><div class="lb lb-4 off" id="ll4"></div><div class="lb lb-3 off" id="ll3"></div><div class="lb lb-2 off" id="ll2"></div><div class="lb lb-1 off" id="ll1"></div></div></div>
  <div class="sensor-card"><h2>Sağ ▶</h2><div class="dist-num" id="rdist">—</div><div style="font-size:10px;color:var(--mut)">cm</div><div class="level-lbl" id="rlbl">—</div><div class="bars"><div class="lb lb-4 off" id="rl4"></div><div class="lb lb-3 off" id="rl3"></div><div class="lb lb-2 off" id="rl2"></div><div class="lb lb-1 off" id="rl1"></div></div></div>
</div>

<!-- IMU -->
<div class="gauges3" style="margin-bottom:10px">
  <div class="gauge"><div class="gl">Roll</div><div class="gv" style="color:#3b82f6" id="groll">—</div><div style="font-size:10px;color:var(--mut)">°</div></div>
  <div class="gauge"><div class="gl">Pitch</div><div class="gv" style="color:#8b5cf6" id="gpitch">—</div><div style="font-size:10px;color:var(--mut)">°</div></div>
  <div class="gauge"><div class="gl">Yaw</div><div class="gv" style="color:#22c55e" id="gyaw">—</div><div style="font-size:10px;color:var(--mut)">°</div></div>
</div>
<div class="g2" style="margin-bottom:10px">
  <div class="card" style="padding:10px"><div class="card-title" style="margin-bottom:6px">Accel m/s²</div>
    <div class="irow"><span class="ilbl">X</span><span class="ival" id="ax">—</span></div>
    <div class="irow"><span class="ilbl">Y</span><span class="ival" id="ay">—</span></div>
    <div class="irow"><span class="ilbl">Z</span><span class="ival" id="az">—</span></div>
  </div>
  <div class="card" style="padding:10px"><div class="card-title" style="margin-bottom:6px">Gyro °/s</div>
    <div class="irow"><span class="ilbl">X</span><span class="ival" id="gx">—</span></div>
    <div class="irow"><span class="ilbl">Y</span><span class="ival" id="gy">—</span></div>
    <div class="irow"><span class="ilbl">Z</span><span class="ival" id="gz">—</span></div>
  </div>
</div>

<!-- Harita -->
<div class="map-card">
  <div class="card-title">Dead-Reckoning <span style="color:var(--mut);font-weight:400;text-transform:none;letter-spacing:0" id="xyval"></span></div>
  <canvas id="map" width="640" height="170" style="border-radius:8px;width:100%"></canvas>
  <div class="map-warn">⚠ Drift — gerçek konum için Faz 5 EKF gerekir</div>
</div>

<!-- Log -->
<div class="log-card">
  <div class="card-title">Engel Logu</div>
  <div class="log-list" id="logList"></div>
</div>

<script>
const LV_COLS=['var(--mut)','var(--p1)','var(--p2)','var(--p3)','var(--p4)'];
const LV_NAMES=['—','● L1 Çok yakın','● L2 Yakın','● L3 Orta','● L4 Uzak'];
let currentMode='AUTO', limitTimer=null;

// ---- Steer canvas ----
function drawSteer(deg){
  const cv=document.getElementById('steerCv'),c=cv.getContext('2d');
  const w=cv.width,h=cv.height,cx=w/2,cy=h-8,R=h-18;
  c.clearRect(0,0,w,h);c.fillStyle='#0d1520';c.fillRect(0,0,w,h);
  c.beginPath();c.arc(cx,cy,R,Math.PI,2*Math.PI);c.strokeStyle='#1e2d3d';c.lineWidth=12;c.stroke();
  c.beginPath();c.arc(cx,cy,R,Math.PI,Math.PI*1.5);c.strokeStyle='#7c3aed';c.lineWidth=12;c.stroke();
  c.beginPath();c.arc(cx,cy,R,Math.PI*1.5,2*Math.PI);c.strokeStyle='#2563eb';c.lineWidth=12;c.stroke();
  const a=Math.PI+(Math.max(-30,Math.min(30,deg))/30)*(Math.PI/2);
  const col=deg===0?'#5a7a96':deg>0?'#3b82f6':'#8b5cf6';
  c.beginPath();c.moveTo(cx,cy);c.lineTo(cx+R*Math.cos(a),cy+R*Math.sin(a));c.strokeStyle=col;c.lineWidth=3;c.stroke();
  c.beginPath();c.arc(cx,cy,4,0,2*Math.PI);c.fillStyle=col;c.fill();
  c.fillStyle='#5a7a96';c.font='10px system-ui';c.textAlign='center';
  c.fillText('-30°',cx-R+8,cy+2);c.fillText('0°',cx,cy-R-4);c.fillText('+30°',cx+R-8,cy+2);
}

// ---- Sensor ----
function updateSensor(p,d,lv){
  const dn=document.getElementById(p+'dist');dn.textContent=d<0?'—':d.toFixed(1);dn.style.color=LV_COLS[lv];
  const lb=document.getElementById(p+'lbl');lb.textContent=LV_NAMES[lv];lb.style.color=LV_COLS[lv];
  for(let i=1;i<=4;i++)document.getElementById(p+'l'+i).classList.toggle('off',lv<i);
}

// ---- Map ----
const MC=document.getElementById('map'),mctx=MC.getContext('2d');
let trail=[{x:0,y:0}],scale=50;
function drawMap(x,y,yDeg){
  const w=MC.width,h=MC.height,cx=w/2,cy=h/2;
  mctx.clearRect(0,0,w,h);mctx.fillStyle='#0d1520';mctx.fillRect(0,0,w,h);
  mctx.strokeStyle='#1e2d3d';mctx.lineWidth=1;
  for(let g=-8;g<=8;g++){mctx.beginPath();mctx.moveTo(cx+g*scale,0);mctx.lineTo(cx+g*scale,h);mctx.stroke();mctx.beginPath();mctx.moveTo(0,cy+g*scale);mctx.lineTo(w,cy+g*scale);mctx.stroke();}
  mctx.strokeStyle='#2d4a63';mctx.lineWidth=1.5;
  mctx.beginPath();mctx.moveTo(0,cy);mctx.lineTo(w,cy);mctx.stroke();
  mctx.beginPath();mctx.moveTo(cx,0);mctx.lineTo(cx,h);mctx.stroke();
  if(trail.length>1){mctx.beginPath();mctx.moveTo(cx+trail[0].x*scale,cy-trail[0].y*scale);for(let i=1;i<trail.length;i++)mctx.lineTo(cx+trail[i].x*scale,cy-trail[i].y*scale);mctx.strokeStyle='#3b82f6';mctx.lineWidth=2;mctx.stroke();}
  const px=cx+x*scale,py=cy-y*scale,yr=yDeg*Math.PI/180;
  mctx.beginPath();mctx.arc(px,py,5,0,2*Math.PI);mctx.fillStyle='#22c55e';mctx.fill();
  mctx.beginPath();mctx.moveTo(px,py);mctx.lineTo(px+16*Math.sin(yr),py-16*Math.cos(yr));mctx.strokeStyle='#22c55e';mctx.lineWidth=2.5;mctx.stroke();
  mctx.beginPath();mctx.arc(cx,cy,4,0,2*Math.PI);mctx.fillStyle='#f59e0b';mctx.fill();
  mctx.fillStyle='#5a7a96';mctx.font='10px system-ui';mctx.textAlign='left';mctx.fillText('1grid='+(100/scale*10).toFixed(0)+'cm',5,h-4);
}

// ---- Log ----
let lastLogs=[];
function updateLog(logs){
  if(JSON.stringify(logs)===JSON.stringify(lastLogs))return;
  lastLogs=logs;
  document.getElementById('logList').innerHTML=logs.map(l=>{
    let c='';if(l.includes('Sağa'))c='right';else if(l.includes('Sola'))c='left';else if(l.includes('TEHLİKE'))c='danger';
    return'<div class="log-entry '+c+'">'+l+'</div>';
  }).join('');
}

// ---- Limits ----
function sendLimits(){
  const lv=document.getElementById('limSlider').value,av=document.getElementById('autoSlider').value;
  document.getElementById('limVal').textContent=lv;document.getElementById('autoVal').textContent=av;
  clearTimeout(limitTimer);limitTimer=setTimeout(()=>fetch('/setlimit?v='+lv+'&a='+av),300);
}

// ---- ARM / DISARM / MODE ----
async function armCar(){await fetch('/arm');}
async function disarmCar(){await fetch('/disarm');}
async function toggleMode(){
  currentMode=currentMode==='AUTO'?'MANUAL':'AUTO';
  await fetch('/mode?m='+currentMode.toLowerCase());
  document.getElementById('manualCard').style.display=currentMode==='MANUAL'?'block':'none';
  const mb=document.getElementById('modeBtn');
  mb.className='btn '+(currentMode==='AUTO'?'btn-auto':'btn-manual');
  mb.textContent='⇄ '+(currentMode==='AUTO'?'AUTO':'MANUAL')+' MOD';
}
async function resetPos(){await fetch('/reset');trail=[{x:0,y:0}];}

// ---- Manual joystick ----
let joyActive=false,joyX=0,joyY=0;
const joy=document.getElementById('joy'),joyDot=document.getElementById('joyDot');
const JR=68;
function joyPos(el,e){const r=el.getBoundingClientRect();const cx=r.left+r.width/2,cy=r.top+r.height/2;const px=(e.touches?e.touches[0].clientX:e.clientX)-cx,py=(e.touches?e.touches[0].clientY:e.clientY)-cy;const dist=Math.sqrt(px*px+py*py);const f=dist>JR?JR/dist:1;return{x:px*f,y:py*f};}
function updateJoy(x,y){joyDot.style.left=(90+x)+'px';joyDot.style.top=(90+y)+'px';joyX=Math.round(x/JR*100);joyY=-Math.round(y/JR*100);}
joy.addEventListener('mousedown',e=>{joyActive=true;const p=joyPos(joy,e);updateJoy(p.x,p.y);});
joy.addEventListener('touchstart',e=>{e.preventDefault();joyActive=true;const p=joyPos(joy,e);updateJoy(p.x,p.y);},{passive:false});
document.addEventListener('mousemove',e=>{if(!joyActive)return;const p=joyPos(joy,e);updateJoy(p.x,p.y);});
document.addEventListener('touchmove',e=>{if(!joyActive)return;e.preventDefault();const p=joyPos(joy,e);updateJoy(p.x,p.y);},{passive:false});
document.addEventListener('mouseup',()=>{if(joyActive){joyActive=false;joyX=0;joyY=0;updateJoy(0,0);}});
document.addEventListener('touchend',()=>{if(joyActive){joyActive=false;joyX=0;joyY=0;updateJoy(0,0);}});

// ---- Keyboard ----
const keys={};
document.addEventListener('keydown',e=>{keys[e.key]=true;});
document.addEventListener('keyup',e=>{keys[e.key]=false;});
function keysToControl(){
  let s=0,t=0;
  if(keys['ArrowUp']||keys['w']||keys['W'])t=80;
  if(keys['ArrowDown']||keys['s']||keys['S'])t=-40;
  if(keys['ArrowLeft']||keys['a']||keys['A'])s=-80;
  if(keys['ArrowRight']||keys['d']||keys['D'])s=80;
  if(keys[' ']){t=0;s=0;}
  return{s,t};
}

// ---- Send manual cmd ----
let cmdTimer=null;
function sendManualCmd(s,t){
  if(currentMode!=='MANUAL')return;
  clearTimeout(cmdTimer);
  cmdTimer=setTimeout(()=>fetch('/control?s='+s+'&t='+t),0);
}

// ---- Main tick ----
async function tick(){
  try{
    const d=await(await fetch('/status',{cache:'no-store'})).json();
    document.getElementById('imuBadge').textContent=d.imuOk?'IMU OK':'IMU ERR';
    document.getElementById('imuBadge').className='badge '+(d.imuOk?'okb':'badb');
    document.getElementById('calBadge').textContent=d.cal?'CALIBRATED':'CALIBRATING...';
    document.getElementById('calBadge').className='badge '+(d.cal?'calok':'calno');
    const ms=document.getElementById('mstateBadge');ms.textContent=d.mstate;ms.className='badge '+(d.mstate==='ARMED'?'armed':d.mstate==='ARMING'?'arming':'disarmed');
    const mb=document.getElementById('modeBadge');mb.textContent=d.mode;mb.className='badge '+(d.mode==='AUTO'?'mode-auto':'mode-manual');
    if(d.mode!==currentMode){currentMode=d.mode;document.getElementById('manualCard').style.display=d.mode==='MANUAL'?'block':'none';}
    document.getElementById('mSteer').textContent=(d.steerPct>=0?'+':'')+d.steerPct;
    document.getElementById('mThr').textContent=(d.thrPct>=0?'+':'')+d.thrPct;
    document.getElementById('mEsc').textContent=d.escUs;
    document.getElementById('mServo').textContent=d.servoUs;
    document.getElementById('mTemp').textContent=d.temp.toFixed(1);
    document.getElementById('ax').textContent=d.ax.toFixed(2);document.getElementById('ay').textContent=d.ay.toFixed(2);document.getElementById('az').textContent=d.az.toFixed(2);
    document.getElementById('gx').textContent=d.gx.toFixed(1);document.getElementById('gy').textContent=d.gy.toFixed(1);document.getElementById('gz').textContent=d.gz.toFixed(1);
    document.getElementById('groll').textContent=d.roll.toFixed(1);document.getElementById('gpitch').textContent=d.pitch.toFixed(1);document.getElementById('gyaw').textContent=d.yaw.toFixed(1);
    document.getElementById('xyval').textContent='x='+d.x.toFixed(3)+'m  y='+d.y.toFixed(3)+'m';
    updateSensor('l',d.lCm,d.lLv);updateSensor('r',d.rCm,d.rLv);
    drawSteer(d.steer);
    const sd=document.getElementById('sdeg'),sl=document.getElementById('slbl');
    sd.textContent=(d.steer>=0?'+':'')+d.steer.toFixed(1)+'°';
    if(Math.abs(d.steer)<0.5){sd.style.color='var(--mut)';sl.textContent='Düz Git';sl.style.color='var(--mut)';}
    else if(d.steer>0){sd.style.color='#3b82f6';sl.textContent='→ Sağa Dön';sl.style.color='#3b82f6';}
    else{sd.style.color='#8b5cf6';sl.textContent='← Sola Dön';sl.style.color='#8b5cf6';}
    const last=trail[trail.length-1];
    if(Math.abs(d.x-last.x)>0.001||Math.abs(d.y-last.y)>0.001)trail.push({x:d.x,y:d.y});
    if(trail.length>400)trail.shift();
    const maxD=trail.reduce((m,p)=>Math.max(m,Math.abs(p.x),Math.abs(p.y)),0.1);
    scale=Math.min(160,(MC.height/2-16)/maxD);
    drawMap(d.x,d.y,d.yaw);updateLog(d.logs);
  }catch(e){}
}

// ---- Control loop (manual) ----
setInterval(()=>{
  if(currentMode!=='MANUAL')return;
  const kc=keysToControl();
  const s=joyActive?joyX:kc.s, t=joyActive?joyY:kc.t;
  sendManualCmd(s,t);
},120);

setInterval(tick,150);
</script>
</body></html>
)HTML";

void handleRoot(){server.send_P(200,"text/html",INDEX_HTML);}

// ========================= SETUP ============================================
void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("\n========================================");
  Serial.println("  Faz 4.5 — Armed Obstacle Navigation");
  Serial.println("========================================");

  servo.attach(PIN_SERVO,SERVO_MIN,SERVO_MAX); servo.writeMicroseconds(SERVO_CENTER);
  esc.attach(PIN_ESC,ESC_MIN_US,ESC_MAX_US);   esc.writeMicroseconds(ESC_NEUTRAL);
  Serial.printf("[MOTOR] Servo→GPIO%d  ESC→GPIO%d  DISARMED\n",PIN_SERVO,PIN_ESC);

  Wire.begin(PIN_SDA,PIN_SCL); Wire.setClock(400000);
  mpuFound=mpuWake();
  if(mpuFound){Serial.printf("[IMU] 0x%02X — kalibrasyon...\n",MPU_ADDR);calibrate();}
  else Serial.println("[IMU] Bulunamadı — sonar+motor aktif");

  pinMode(PIN_TRIG_L,OUTPUT);digitalWrite(PIN_TRIG_L,LOW);
  pinMode(PIN_TRIG_R,OUTPUT);digitalWrite(PIN_TRIG_R,LOW);
  pinMode(PIN_ECHO_L,INPUT);pinMode(PIN_ECHO_R,INPUT);
  xTaskCreatePinnedToCore(sonarTask,"sonar",4096,NULL,1,NULL,0);
  Serial.printf("[SONAR] Sol TRIG%d/ECHO%d  Sağ TRIG%d/ECHO%d\n",PIN_TRIG_L,PIN_ECHO_L,PIN_TRIG_R,PIN_ECHO_R);

  WiFi.mode(WIFI_STA);
  Serial.println("[WiFi] Taranıyor...");
  int n=WiFi.scanNetworks(); bool found=false;
  for(int i=0;i<n;i++){bool m=(WiFi.SSID(i)==String(WIFI_SSID));Serial.printf("  [%d] %-26s %4ddBm ch%d%s\n",i+1,WiFi.SSID(i).c_str(),WiFi.RSSI(i),WiFi.channel(i),m?" <<HEDEF":"");if(m)found=true;}
  if(!found)Serial.printf("[WiFi] UYARI: '%s' bulunamadı!\n",WIFI_SSID);
  WiFi.scanDelete();
  WiFi.begin(WIFI_SSID,WIFI_PASS);
  unsigned long t0=millis(); int lastSt=-1;
  while(WiFi.status()!=WL_CONNECTED&&millis()-t0<15000){
    delay(500);int st=WiFi.status();
    if(st!=lastSt){const char*sn=st==WL_NO_SSID_AVAIL?"NO_SSID":st==WL_CONNECT_FAILED?"FAILED":"...";Serial.printf("[WiFi] %s %.1fs\n",sn,(millis()-t0)/1000.0f);lastSt=st;}else Serial.print(".");
  }
  Serial.println();
  if(WiFi.status()==WL_CONNECTED)Serial.printf("[WiFi] BAĞLANDI → http://%s:%d/\n",WiFi.localIP().toString().c_str(),WEB_PORT);
  else Serial.printf("[WiFi] BAŞARISIZ (%d)\n",WiFi.status());

  server.on("/",handleRoot);
  server.on("/status",handleStatus);
  server.on("/reset",handleReset);
  server.on("/arm",handleArm);
  server.on("/disarm",handleDisarm);
  server.on("/mode",handleMode);
  server.on("/control",handleControl);
  server.on("/setlimit",handleSetLimit);
  server.onNotFound([](){ server.send(404,"text/plain","not found"); });
  server.begin();
  Serial.println("[HAZIR] — DISARMED, AUTO mod");
  lastLoopUs=micros();
}

// ========================= LOOP =============================================
void loop(){
  server.handleClient();

  // Arming state machine
  if(motorState==ARMING&&millis()-armStartMs>=ARM_DURATION_MS){
    motorState=ARMED; Serial.println("[STATE] ARMED");
    if(driveMode==MANUAL)lastManualCmdMs=millis();
  }

  // MANUAL watchdog: komut gelmezse nötr (ama ARMED kalır)
  if(motorState==ARMED&&driveMode==MANUAL&&millis()-lastManualCmdMs>MANUAL_WDOG_MS){
    writeEscNeutral(); writeServoPct(0);
  }

  // AUTO re-arm: engel geçtikten 2 sn sonra otomatik yeniden arm
  if(driveMode==AUTO && motorState==DISARMED && pendingAutoRearm &&
     millis()-autoDisarmMs > AUTO_REARM_DELAY_MS){
    int lLv,rLv; unsigned long age;
    portENTER_CRITICAL(&sonarMux); lLv=leftLevel; rLv=rightLevel; age=millis()-sonarLastUpdMs; portEXIT_CRITICAL(&sonarMux);
    if(age<SONAR_WDOG_MS && lLv!=1 && rLv!=1){
      Serial.println("[AUTO-REARM] Engel geçti — yeniden arming");
      pendingAutoRearm=false;
      beginArming();
    }
  }

  // AUTO otonom sürüş
  autonomousDrive();

  // IMU okuma
  unsigned long now=millis();
  if(now-lastReadMs>=READ_INTERVAL_MS){
    lastReadMs=now;
    unsigned long nowUs=micros(); float dt=(nowUs-lastLoopUs)*1e-6f; lastLoopUs=nowUs;
    if(dt>0.1f)dt=0.02f;
    if(!mpuFound){mpuFound=mpuWake();if(mpuFound)calibrate();return;}
    ImuRaw raw;
    if(mpuReadRaw(raw)){raw.ax-=bias_ax;raw.ay-=bias_ay;raw.az-=bias_az;raw.gx-=bias_gx;raw.gy-=bias_gy;raw.gz-=bias_gz;imu=raw;if(calibrated)updateFusion(dt);}
    else{imu.ok=false;mpuFound=false;}
  }
}
