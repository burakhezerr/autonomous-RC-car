/*
 * ============================================================================
 *  Faz 7 — LQR + Offline Hiz Profili  (ESP32-S3)   [MPC ALTERNATIFI]
 *  Autonomous RC Car Racing Platform / Kontrol Katmani
 * ============================================================================
 *
 *  AMAC:
 *    MPC'nin yarista verdiginin %80-90'i, MPC'siz. "Optimizasyon" laptopta
 *    (offline_design.py) yapildi ve `track_data.h`'ye gomuldu. Araçta sadece:
 *      - V_REF tablosu araması (offline optimal hiz profili)
 *      - Feedforward direksiyon: delta_ff = atan(L * kappa)
 *      - LQR geri besleme: delta_fb = -(k1*e_cte + k2*e_psi), hiza gore gain-schedule
 *    => mikro-saniye seviyesinde, MPC'nin online QP'si olmadan.
 *
 *  >>> track_data.h `offline_design.py` ile uretildi ve sim'de dogrulandi:
 *      hiz profili (viraj yavas/duz hizli) + LQR kazanclari pozitif/sonlu (kararli). <<<
 *
 *  GUVENLIK: Faz 6 ile AYNI (otonom -> fail-safe kritik): ARM + pose timeout
 *    -> guvenli dur + geofence + throttle limit. Friction-circle limiti zaten
 *    OFFLINE hiz profiline gomulu (viraj hizi sqrt(mu*g/kappa) ile sinirli).
 *
 *  >>> SIGN UYARISI (donanimda dogrulanmadi): e_cte / e_psi / delta isaret
 *    konvansiyonu ilk gercek testte ters cikarsa ilgili terimin isaretini cevir.
 *    Konvansiyon: e_cte sola pozitif, e_psi = wrap(psi - yol_yonu),
 *    delta_fb = -(k1*e_cte + k2*e_psi). EKF bolumu Faz 5 ile ayni (dogrulandi).
 *  NOT: arduino-cli yok -> derlenmeden, okuma ile dogrulandi.
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>
#include "track_data.h"     // offline uretildi: PATH_X/Y/KAPPA, V_REF, GAIN_*

// ---- WiFi / UDP ----
const char* WIFI_SSID = "WIFI_ADINI_YAZ";
const char* WIFI_PASS = "WIFI_SIFRENI_YAZ";
const uint16_t UDP_PORT = 4210;

// ---- Pinler / PWM ----
const int PIN_SERVO=4, PIN_ESC=5, PIN_SDA=8, PIN_SCL=9, PIN_ENC_A=6;
const int SERVO_MIN=1000, SERVO_CENTER=1500, SERVO_MAX=2000;
const int ESC_MIN_US=1000, ESC_MAX_US=2000, ESC_NEUTRAL=1500;

// ---- Arac / kontrol ----
const float L=0.25, MAX_STEER=30.0*PI/180.0;
const int   THROTTLE_LIMIT_PCT=30;
const float SPEED_KP_US=150.0;

// ---- Sensor (Faz 3/5 ile AYNI - CONFIRM) ----
const float ENC_PPR=360.0, WHEEL_DIAMETER_M=0.065, GEAR_RATIO=1.0;
const float WHEEL_CIRC_M=PI*WHEEL_DIAMETER_M, GYRO_SCALE=131.0, DEG2RAD=PI/180.0;
const uint8_t MPU_ADDR=0x68;

// ---- Guvenlik ----
const unsigned long ARM_DURATION_MS=3000, POSE_TIMEOUT_MS=1000;
const float GEOFENCE_MARGIN=1.0;

// ---- EKF (Faz 5 ile AYNI) ----
float X[4]={5,0,PI/2,0};
float P[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
const float Q[4]={0.01,0.01,0.005,0.05};
const float R_POS=0.0025, R_PSI=0.0025, R_V=0.01;
float wrapPi(float a){while(a>PI)a-=2*PI;while(a<-PI)a+=2*PI;return a;}
void ekfPredict(float omega,float dt){
  float x=X[0],y=X[1],psi=X[2],v=X[3];
  X[0]=x+v*cosf(psi)*dt;X[1]=y+v*sinf(psi)*dt;X[2]=wrapPi(psi+omega*dt);X[3]=v;
  float F[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  F[0][2]=-v*sinf(psi)*dt;F[0][3]=cosf(psi)*dt;F[1][2]=v*cosf(psi)*dt;F[1][3]=sinf(psi)*dt;
  float FP[4][4],Pn[4][4];
  for(int i=0;i<4;i++)for(int j=0;j<4;j++){float s=0;for(int k=0;k<4;k++)s+=F[i][k]*P[k][j];FP[i][j]=s;}
  for(int i=0;i<4;i++)for(int j=0;j<4;j++){float s=0;for(int k=0;k<4;k++)s+=FP[i][k]*F[j][k];Pn[i][j]=s;}
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)P[i][j]=Pn[i][j]+(i==j?Q[i]:0);
}
void ekfUpd(const float H[4],float z,float r,bool ang){
  float Hx=0;for(int i=0;i<4;i++)Hx+=H[i]*X[i];
  float y=z-Hx;if(ang)y=wrapPi(y);
  float PHt[4];for(int i=0;i<4;i++){float s=0;for(int k=0;k<4;k++)s+=P[i][k]*H[k];PHt[i]=s;}
  float S=r;for(int i=0;i<4;i++)S+=H[i]*PHt[i];
  float K[4];for(int i=0;i<4;i++)K[i]=PHt[i]/S;
  for(int i=0;i<4;i++)X[i]+=K[i]*y;X[2]=wrapPi(X[2]);
  float IKH[4][4],Pn[4][4];
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)IKH[i][j]=(i==j?1.0:0.0)-K[i]*H[j];
  for(int i=0;i<4;i++)for(int j=0;j<4;j++){float s=0;for(int k=0;k<4;k++)s+=IKH[i][k]*P[k][j];Pn[i][j]=s;}
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)P[i][j]=Pn[i][j];
}
void ekfSpeed(float v){float H[4]={0,0,0,1};ekfUpd(H,v,R_V,false);}
void ekfPose(float mx,float my,float mp){float Hx[4]={1,0,0,0},Hy[4]={0,1,0,0},Hp[4]={0,0,1,0};
  ekfUpd(Hx,mx,R_POS,false);ekfUpd(Hy,my,R_POS,false);ekfUpd(Hp,mp,R_PSI,true);}

// ---- HW ----
Servo servo,esc; WiFiUDP udp; char udpBuf[64];
volatile long encCount=0; portMUX_TYPE encMux=portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR encISR(){portENTER_CRITICAL_ISR(&encMux);encCount++;portEXIT_CRITICAL_ISR(&encMux);}
long readEnc(){long c;portENTER_CRITICAL(&encMux);c=encCount;encCount=0;portEXIT_CRITICAL(&encMux);return c;}
void mpuWake(){Wire.beginTransmission(MPU_ADDR);Wire.write(0x6B);Wire.write(0x00);Wire.endTransmission();}
float readGyroZ(){Wire.beginTransmission(MPU_ADDR);Wire.write(0x47);
  if(Wire.endTransmission(false)!=0)return 0; if(Wire.requestFrom((int)MPU_ADDR,2)!=2)return 0;
  int16_t raw=(Wire.read()<<8)|Wire.read(); return (raw/GYRO_SCALE)*DEG2RAD;}
void servoSteer(float d){d=constrain(d,-MAX_STEER,MAX_STEER);
  int us=SERVO_CENTER+(int)(d/MAX_STEER*(SERVO_MAX-SERVO_MIN)/2);
  servo.writeMicroseconds(constrain(us,SERVO_MIN,SERVO_MAX));}

enum MotorState{DISARMED,ARMING,ARMED}; MotorState motorState=DISARMED;
unsigned long armStartMs=0,lastLoop=0,lastPose=0,lastPrint=0;
void escSpeed(float tv,float mv){if(motorState!=ARMED){esc.writeMicroseconds(ESC_NEUTRAL);return;}
  float du=SPEED_KP_US*(tv-mv); float md=(ESC_MAX_US-ESC_NEUTRAL)*THROTTLE_LIMIT_PCT/100.0;
  du=constrain(du,-md,md); esc.writeMicroseconds((int)(ESC_NEUTRAL+du));}
void safeStop(const char* w){motorState=DISARMED;esc.writeMicroseconds(ESC_NEUTRAL);servoSteer(0);
  Serial.printf("[FAIL-SAFE] DUR (%s)\n",w);}
void beginArming(){if(motorState!=DISARMED)return;motorState=ARMING;armStartMs=millis();lastPose=millis();
  esc.writeMicroseconds(ESC_NEUTRAL);Serial.println("[STATE] ARMING...");}

// ---- Hiza gore LQR kazanc interpolasyonu ----
void lqrGain(float v, float& k1, float& k2){
  if(v<=GAIN_V[0]){k1=GAIN_K1[0];k2=GAIN_K2[0];return;}
  if(v>=GAIN_V[N_GAINS-1]){k1=GAIN_K1[N_GAINS-1];k2=GAIN_K2[N_GAINS-1];return;}
  for(int i=0;i<N_GAINS-1;i++) if(v>=GAIN_V[i]&&v<GAIN_V[i+1]){
    float t=(v-GAIN_V[i])/(GAIN_V[i+1]-GAIN_V[i]);
    k1=GAIN_K1[i]+t*(GAIN_K1[i+1]-GAIN_K1[i]);
    k2=GAIN_K2[i]+t*(GAIN_K2[i+1]-GAIN_K2[i]); return;}
}

// ---- En yakin nokta + isaretli hatalar + isaretli egrilik ----
int nearestWp(){int ni=0;float nd=1e9;
  for(int i=0;i<NWP;i++){float d=hypotf(PATH_X[i]-X[0],PATH_Y[i]-X[1]);if(d<nd){nd=d;ni=i;}}return ni;}

void connectWifi(){WiFi.mode(WIFI_STA);WiFi.begin(WIFI_SSID,WIFI_PASS);
  unsigned long t0=millis();while(WiFi.status()!=WL_CONNECTED&&millis()-t0<15000){delay(250);Serial.print(".");}
  Serial.println();
  if(WiFi.status()==WL_CONNECTED){Serial.printf("[WiFi] IP:%s UDP:%u\n",WiFi.localIP().toString().c_str(),UDP_PORT);udp.begin(UDP_PORT);}
  else Serial.println("[WiFi] baglanamadi.");}

void setup(){
  Serial.begin(115200);delay(300);
  ESP32PWM::allocateTimer(0);ESP32PWM::allocateTimer(1);
  servo.setPeriodHertz(50);servo.attach(PIN_SERVO,SERVO_MIN,SERVO_MAX);
  esc.setPeriodHertz(50);esc.attach(PIN_ESC,ESC_MIN_US,ESC_MAX_US);
  servoSteer(0);esc.writeMicroseconds(ESC_NEUTRAL);
  Wire.begin(PIN_SDA,PIN_SCL);Wire.setClock(400000);mpuWake();
  pinMode(PIN_ENC_A,INPUT_PULLUP);attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),encISR,RISING);
  connectWifi();lastLoop=millis();
  Serial.println("[Faz7] LQR + offline hiz profili. 'a'=ARM 'x'=DUR. TEKERLER HAVADA ilk testte.");
}

void loop(){
  while(Serial.available()){char c=Serial.read();if(c=='a')beginArming();else if(c=='x')safeStop("serial x");}
  int sz=udp.parsePacket();
  if(sz>0){int n=udp.read(udpBuf,sizeof(udpBuf)-1);udpBuf[n]=0;float mx,my,mp;
    if(sscanf(udpBuf,"%f,%f,%f",&mx,&my,&mp)==3){ekfPose(mx,my,wrapPi(mp));lastPose=millis();}}
  if(motorState==ARMING&&millis()-armStartMs>=ARM_DURATION_MS){motorState=ARMED;Serial.println("[STATE] ARMED.");}
  if(motorState!=DISARMED&&millis()-lastPose>POSE_TIMEOUT_MS)safeStop("pose timeout");

  unsigned long now=millis();if(now-lastLoop<20)return;
  float dt=(now-lastLoop)/1000.0;lastLoop=now;

  float omega=readGyroZ(); long counts=readEnc();
  float speed=((counts/ENC_PPR)/GEAR_RATIO)*WHEEL_CIRC_M/dt;
  ekfPredict(omega,dt);ekfSpeed(speed);

  // en yakin segment, isaretli cross-track + heading hatasi
  int ni=nearestWp(); int nj=(ni+1)%NWP;
  float dx=PATH_X[nj]-PATH_X[ni], dy=PATH_Y[nj]-PATH_Y[ni];
  float segLen=hypotf(dx,dy)+1e-6;
  float th=atan2f(dy,dx);                                   // yol yonu
  float ex=X[0]-PATH_X[ni], ey=X[1]-PATH_Y[ni];
  float eCte=(dx*ey-dy*ex)/segLen;                          // sola pozitif
  float ePsi=wrapPi(X[2]-th);                               // heading hatasi
  // isaretli egrilik (feedforward icin) komsu yol yonlerinden
  int nk=(nj+1)%NWP;
  float th2=atan2f(PATH_Y[nk]-PATH_Y[nj],PATH_X[nk]-PATH_X[nj]);
  float kappaSigned=wrapPi(th2-th)/segLen;

  // geofence
  static float mnx=1e9,mxx=-1e9,mny=1e9,mxy=-1e9; if(mxx<mnx){for(int i=0;i<NWP;i++){mnx=min(mnx,PATH_X[i]);mxx=max(mxx,PATH_X[i]);mny=min(mny,PATH_Y[i]);mxy=max(mxy,PATH_Y[i]);}}
  if(motorState==ARMED&&(X[0]<mnx-GEOFENCE_MARGIN||X[0]>mxx+GEOFENCE_MARGIN||X[1]<mny-GEOFENCE_MARGIN||X[1]>mxy+GEOFENCE_MARGIN))safeStop("geofence");

  // ---- KONTROL: feedforward + LQR ----
  float k1,k2; lqrGain(speed,k1,k2);
  float deltaFf=atanf(L*kappaSigned);                      // viraj onsezisi
  float deltaFb=-(k1*eCte+k2*ePsi);                        // LQR geri besleme
  float delta=deltaFf+deltaFb;
  servoSteer(delta);

  // ---- hiz: offline profil tablosu (friction-circle gomulu) ----
  float vref=V_REF[ni];
  escSpeed(vref,speed);

  if(now-lastPrint>200){lastPrint=now;
    Serial.printf("x=%.2f y=%.2f v=%.2f vref=%.2f eCte=%.2f ePsi=%.0f delta=%.0f st=%d\n",
                  X[0],X[1],speed,vref,eCte,ePsi*180/PI,delta*180/PI,motorState);}
}
