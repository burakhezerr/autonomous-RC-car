/*
 * ============================================================================
 *  Faz 6 — Yol Takibi (Pure Pursuit)  (ESP32-S3)
 *  Autonomous RC Car Racing Platform / Kontrol Katmani
 * ============================================================================
 *
 *  AMAC:
 *    Ilk OTONOM surus. EKF durumunu (Faz 5) kullanarak referans yolu
 *    Pure Pursuit ile takip et. Direksiyon = atan2(2 L sin(alpha), ld).
 *    Bu algoritma `Simulation/bicycle_pure_pursuit.py`'de DOGRULANDI
 *    (kararli durum ~5.8 cm sapma).
 *
 *  GUVENLIK — OTONOM oldugu icin fail-safe DAHA kritik (volanda insan yok):
 *    - ARMED degilse surmez ('a' ile arm). Watchdog korunur.
 *    - POSE TIMEOUT: kameradan pose ~POSE_TIMEOUT_MS gelmezse GUVENLI DUR
 *      (kor navigasyon yapma). EKF kisa sure dead-reckoning yapsa da
 *      konum guvenligi icin duruyoruz.
 *    - GEOFENCE: tahmini konum parkur sinirinin disinda ise DUR (Section 3.6).
 *    - THROTTLE_LIMIT_PCT korunur. TEKERLER HAVADA ilk testte.
 *
 *  EKF bolumu Faz 5 ile AYNIDIR (Python referansiyla dogrulandi).
 *  NOT: arduino-cli yok -> derlenmeden, okuma ile dogrulandi.
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// ---- WiFi / UDP ----
const char* WIFI_SSID = "WIFI_ADINI_YAZ";
const char* WIFI_PASS = "WIFI_SIFRENI_YAZ";
const uint16_t UDP_PORT = 4210;

// ---- Pinler ----
const int PIN_SERVO = 4, PIN_ESC = 5, PIN_SDA = 8, PIN_SCL = 9, PIN_ENC_A = 6;

// ---- PWM ----
const int SERVO_MIN = 1000, SERVO_CENTER = 1500, SERVO_MAX = 2000;
const int ESC_MIN_US = 1000, ESC_MAX_US = 2000, ESC_NEUTRAL = 1500;

// ---- Arac / kontrol ----
const float L = 0.25;                  // dingil mesafesi (m)
const float MAX_STEER = 30.0 * PI / 180.0;
const float TARGET_SPEED = 1.5;        // m/s (sabit; degisken profil Faz 7'de)
const float LD_GAIN = 0.4, LD_MIN = 0.4;   // lookahead = LD_GAIN*v + LD_MIN
const int   THROTTLE_LIMIT_PCT = 30;
const float SPEED_KP_US = 150.0;       // hiz hatasi (m/s) -> ESC us

// ---- Sensor sabitleri (Faz 3/5 ile AYNI - CONFIRM) ----
const float ENC_PPR = 360.0, WHEEL_DIAMETER_M = 0.065, GEAR_RATIO = 1.0;
const float WHEEL_CIRC_M = PI * WHEEL_DIAMETER_M;
const float GYRO_SCALE = 131.0, DEG2RAD = PI / 180.0;
const uint8_t MPU_ADDR = 0x68;

// ---- Guvenlik ----
const unsigned long ARM_DURATION_MS = 3000;
const unsigned long POSE_TIMEOUT_MS = 1000;   // bu sure pose yoksa dur
const float GEOFENCE_MARGIN = 1.0;            // parkur etrafi guvenli pay (m)

// ---- Referans yol (oval, setup'ta uretilir) ----
const int NWP = 60;
float pathX[NWP], pathY[NWP];
float trackMinX, trackMaxX, trackMinY, trackMaxY;   // geofence kutusu

// ---- Durum ----
enum MotorState { DISARMED, ARMING, ARMED };
MotorState motorState = DISARMED;
unsigned long armStartMs = 0, lastLoop = 0, lastPose = 0, lastPrint = 0;

// ---- EKF (Faz 5 ile AYNI) ----
float X[4] = {5, 0, PI/2, 0};
float P[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
const float Q[4] = {0.01,0.01,0.005,0.05};
const float R_POS = 0.0025, R_PSI = 0.0025, R_V = 0.01;
float wrapPi(float a){ while(a>PI)a-=2*PI; while(a<-PI)a+=2*PI; return a; }
void ekfPredict(float omega,float dt){
  float x=X[0],y=X[1],psi=X[2],v=X[3];
  X[0]=x+v*cosf(psi)*dt; X[1]=y+v*sinf(psi)*dt; X[2]=wrapPi(psi+omega*dt); X[3]=v;
  float F[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  F[0][2]=-v*sinf(psi)*dt; F[0][3]=cosf(psi)*dt; F[1][2]=v*cosf(psi)*dt; F[1][3]=sinf(psi)*dt;
  float FP[4][4],Pn[4][4];
  for(int i=0;i<4;i++)for(int j=0;j<4;j++){float s=0;for(int k=0;k<4;k++)s+=F[i][k]*P[k][j];FP[i][j]=s;}
  for(int i=0;i<4;i++)for(int j=0;j<4;j++){float s=0;for(int k=0;k<4;k++)s+=FP[i][k]*F[j][k];Pn[i][j]=s;}
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)P[i][j]=Pn[i][j]+(i==j?Q[i]:0);
}
void ekfUpd(const float H[4],float z,float r,bool ang){
  float Hx=0;for(int i=0;i<4;i++)Hx+=H[i]*X[i];
  float y=z-Hx; if(ang)y=wrapPi(y);
  float PHt[4];for(int i=0;i<4;i++){float s=0;for(int k=0;k<4;k++)s+=P[i][k]*H[k];PHt[i]=s;}
  float S=r;for(int i=0;i<4;i++)S+=H[i]*PHt[i];
  float K[4];for(int i=0;i<4;i++)K[i]=PHt[i]/S;
  for(int i=0;i<4;i++)X[i]+=K[i]*y; X[2]=wrapPi(X[2]);
  float IKH[4][4],Pn[4][4];
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)IKH[i][j]=(i==j?1.0:0.0)-K[i]*H[j];
  for(int i=0;i<4;i++)for(int j=0;j<4;j++){float s=0;for(int k=0;k<4;k++)s+=IKH[i][k]*P[k][j];Pn[i][j]=s;}
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)P[i][j]=Pn[i][j];
}
void ekfSpeed(float v){float H[4]={0,0,0,1};ekfUpd(H,v,R_V,false);}
void ekfPose(float mx,float my,float mp){
  float Hx[4]={1,0,0,0},Hy[4]={0,1,0,0},Hp[4]={0,0,1,0};
  ekfUpd(Hx,mx,R_POS,false); ekfUpd(Hy,my,R_POS,false); ekfUpd(Hp,mp,R_PSI,true);
}

// ---- Aktuator/sensor ----
Servo servo, esc; WiFiUDP udp; char udpBuf[64];
volatile long encCount=0; portMUX_TYPE encMux=portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR encISR(){portENTER_CRITICAL_ISR(&encMux);encCount++;portEXIT_CRITICAL_ISR(&encMux);}
long readEnc(){long c;portENTER_CRITICAL(&encMux);c=encCount;encCount=0;portEXIT_CRITICAL(&encMux);return c;}
void mpuWake(){Wire.beginTransmission(MPU_ADDR);Wire.write(0x6B);Wire.write(0x00);Wire.endTransmission();}
float readGyroZ(){
  Wire.beginTransmission(MPU_ADDR);Wire.write(0x47);
  if(Wire.endTransmission(false)!=0)return 0;
  if(Wire.requestFrom((int)MPU_ADDR,2)!=2)return 0;
  int16_t raw=(Wire.read()<<8)|Wire.read();
  return (raw/GYRO_SCALE)*DEG2RAD;
}
void servoSteer(float delta){   // radyan -> servo us
  delta=constrain(delta,-MAX_STEER,MAX_STEER);
  int us=SERVO_CENTER+(int)(delta/MAX_STEER*(SERVO_MAX-SERVO_MIN)/2);
  servo.writeMicroseconds(constrain(us,SERVO_MIN,SERVO_MAX));
}
void escSpeed(float targetV,float measV){  // basit P + limit, yalniz ARMED
  if(motorState!=ARMED){esc.writeMicroseconds(ESC_NEUTRAL);return;}
  float du=SPEED_KP_US*(targetV-measV);
  float maxD=(ESC_MAX_US-ESC_NEUTRAL)*THROTTLE_LIMIT_PCT/100.0;
  du=constrain(du,-maxD,maxD);
  esc.writeMicroseconds((int)(ESC_NEUTRAL+du));
}
void safeStop(const char* why){
  motorState=DISARMED; esc.writeMicroseconds(ESC_NEUTRAL); servoSteer(0);
  Serial.printf("[FAIL-SAFE] DUR (%s)\n",why);
}

// ---- Pure Pursuit (bicycle_pure_pursuit.py portu) ----
float purePursuit(float ld){
  int ni=0; float nd=1e9;
  for(int i=0;i<NWP;i++){float d=hypotf(pathX[i]-X[0],pathY[i]-X[1]); if(d<nd){nd=d;ni=i;}}
  float tx=pathX[ni],ty=pathY[ni];
  for(int k=0;k<NWP;k++){int idx=(ni+k)%NWP;
    if(hypotf(pathX[idx]-X[0],pathY[idx]-X[1])>=ld){tx=pathX[idx];ty=pathY[idx];break;}}
  float dx=tx-X[0],dy=ty-X[1];
  float alpha=atan2f(dy,dx)-X[2];
  float actLd=hypotf(dx,dy); if(actLd<1e-3)return 0;
  return atan2f(2.0*L*sinf(alpha),actLd);
}

void beginArming(){ if(motorState!=DISARMED)return; motorState=ARMING; armStartMs=millis(); lastPose=millis();
  esc.writeMicroseconds(ESC_NEUTRAL); Serial.println("[STATE] ARMING..."); }

void connectWifi(){
  WiFi.mode(WIFI_STA);WiFi.begin(WIFI_SSID,WIFI_PASS);
  unsigned long t0=millis();
  while(WiFi.status()!=WL_CONNECTED&&millis()-t0<15000){delay(250);Serial.print(".");}
  Serial.println();
  if(WiFi.status()==WL_CONNECTED){Serial.printf("[WiFi] IP:%s UDP:%u\n",WiFi.localIP().toString().c_str(),UDP_PORT);udp.begin(UDP_PORT);}
  else Serial.println("[WiFi] baglanamadi.");
}

void setup(){
  Serial.begin(115200);delay(300);
  // oval yol + geofence kutusu
  trackMinX=trackMinY=1e9; trackMaxX=trackMaxY=-1e9;
  for(int i=0;i<NWP;i++){float t=2*PI*i/NWP; pathX[i]=5*cosf(t); pathY[i]=3*sinf(t);
    trackMinX=min(trackMinX,pathX[i]);trackMaxX=max(trackMaxX,pathX[i]);
    trackMinY=min(trackMinY,pathY[i]);trackMaxY=max(trackMaxY,pathY[i]);}
  ESP32PWM::allocateTimer(0);ESP32PWM::allocateTimer(1);
  servo.setPeriodHertz(50);servo.attach(PIN_SERVO,SERVO_MIN,SERVO_MAX);
  esc.setPeriodHertz(50);esc.attach(PIN_ESC,ESC_MIN_US,ESC_MAX_US);
  servoSteer(0);esc.writeMicroseconds(ESC_NEUTRAL);
  Wire.begin(PIN_SDA,PIN_SCL);Wire.setClock(400000);mpuWake();
  pinMode(PIN_ENC_A,INPUT_PULLUP);attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),encISR,RISING);
  connectWifi();
  lastLoop=millis();
  Serial.println("[Faz6] Pure Pursuit otonom. 'a'=ARM 'x'=DUR. TEKERLER HAVADA ilk testte.");
}

void loop(){
  while(Serial.available()){char c=Serial.read(); if(c=='a')beginArming(); else if(c=='x')safeStop("serial x");}

  // UDP pose
  int sz=udp.parsePacket();
  if(sz>0){int n=udp.read(udpBuf,sizeof(udpBuf)-1);udpBuf[n]=0; float mx,my,mp;
    if(sscanf(udpBuf,"%f,%f,%f",&mx,&my,&mp)==3){ekfPose(mx,my,wrapPi(mp));lastPose=millis();}}

  if(motorState==ARMING&&millis()-armStartMs>=ARM_DURATION_MS){motorState=ARMED;Serial.println("[STATE] ARMED.");}

  // Fail-safe: pose timeout
  if(motorState!=DISARMED&&millis()-lastPose>POSE_TIMEOUT_MS){safeStop("pose timeout");}

  unsigned long now=millis(); if(now-lastLoop<20)return;
  float dt=(now-lastLoop)/1000.0;lastLoop=now;

  float omega=readGyroZ();
  long counts=readEnc();
  float speed=((counts/ENC_PPR)/GEAR_RATIO)*WHEEL_CIRC_M/dt;
  ekfPredict(omega,dt); ekfSpeed(speed);

  // Geofence
  if(motorState==ARMED&&(X[0]<trackMinX-GEOFENCE_MARGIN||X[0]>trackMaxX+GEOFENCE_MARGIN||
                         X[1]<trackMinY-GEOFENCE_MARGIN||X[1]>trackMaxY+GEOFENCE_MARGIN)){
    safeStop("geofence"); }

  // Kontrol
  float ld=LD_GAIN*X[3]+LD_MIN;
  float delta=purePursuit(ld);
  servoSteer(delta);
  escSpeed(TARGET_SPEED,speed);

  if(now-lastPrint>200){lastPrint=now;
    Serial.printf("x=%.2f y=%.2f psi=%.0f v=%.2f delta=%.0fdeg st=%d\n",
                  X[0],X[1],X[2]*180/PI,speed,delta*180/PI,motorState);}
}
