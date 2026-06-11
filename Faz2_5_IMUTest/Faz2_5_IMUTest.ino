/*
 * ============================================================================
 *  Faz 2.5 — MPU6050 IMU Test + Orientation + Dead-Reckoning  (ESP32-S3)
 * ============================================================================
 *
 *  KABLOLAMA:  SDA→GPIO8  SCL→GPIO9  VCC→3.3V  GND→GND
 *  WEB UI:     http://<IP>:5002/
 *
 *  AÇILAR (complementary filter):
 *    roll, pitch  → accel ağırlıklı, gyro hız düzeltmesi, drift yok
 *    yaw          → yalnızca gyro integrali, zamanla drift eder
 *
 *  POZİSYON (dead-reckoning):
 *    Gravity çıkarılmış accel → hız integrali → konum integrali.
 *    UYARI: birkaç saniyede kayar — sadece kısa süreli gösterge.
 *    Gerçek konum için Faz 5 EKF gerekir.
 * ============================================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

const char* WIFI_SSID = "Hezer";
const char* WIFI_PASS = "burakhezer";
const int   WEB_PORT  = 5002;

const int     PIN_SDA        = 8;
const int     PIN_SCL        = 9;
const uint8_t MPU_ADDR       = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_XOUT = 0x3B;
const float   ACCEL_SCALE    = 16384.0f;
const float   GYRO_SCALE     = 131.0f;
const float   G_MS2          = 9.81f;

const unsigned long READ_INTERVAL_MS = 20;
const int           CAL_SAMPLES      = 300;
const float         CF_ALPHA         = 0.98f;

struct ImuRaw { float ax,ay,az,gx,gy,gz,temp; bool ok; };

ImuRaw  imu       = {};
bool    mpuFound  = false;
bool    calibrated= false;

float bias_ax=0,bias_ay=0,bias_az=0;
float bias_gx=0,bias_gy=0,bias_gz=0;
float roll=0,pitch=0,yaw=0;
float dr_x=0,dr_y=0,dr_vx=0,dr_vy=0;

unsigned long lastReadMs  = 0;
unsigned long lastLoopUs  = 0;

WebServer server(WEB_PORT);

bool mpuWake(){
  Wire.beginTransmission(MPU_ADDR); Wire.write(REG_PWR_MGMT_1); Wire.write(0x00);
  return Wire.endTransmission()==0;
}
bool mpuReadRaw(ImuRaw& d){
  Wire.beginTransmission(MPU_ADDR); Wire.write(REG_ACCEL_XOUT);
  if(Wire.endTransmission(false)!=0){d.ok=false;return false;}
  if(Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14)<14){d.ok=false;return false;}
  auto rw=[]()->int16_t{return(int16_t)((Wire.read()<<8)|Wire.read());};
  d.ax=rw()/ACCEL_SCALE*G_MS2; d.ay=rw()/ACCEL_SCALE*G_MS2; d.az=rw()/ACCEL_SCALE*G_MS2;
  d.temp=rw()/340.0f+36.53f;
  d.gx=rw()/GYRO_SCALE; d.gy=rw()/GYRO_SCALE; d.gz=rw()/GYRO_SCALE;
  d.ok=true; return true;
}
void calibrate(){
  Serial.printf("[CAL] Keep still — %d samples...\n",CAL_SAMPLES);
  double sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0; int n=0; ImuRaw d;
  while(n<CAL_SAMPLES){
    if(mpuReadRaw(d)){sax+=d.ax;say+=d.ay;saz+=d.az;sgx+=d.gx;sgy+=d.gy;sgz+=d.gz;n++;}
    delay(READ_INTERVAL_MS);
  }
  float mx=sax/n,my=say/n,mz=saz/n;
  float abx=fabsf(mx),aby=fabsf(my),abz=fabsf(mz);
  float gsx=0,gsy=0,gsz=0;
  if(abx>=aby&&abx>=abz)      gsx=(mx>=0)?1:-1;
  else if(aby>=abx&&aby>=abz) gsy=(my>=0)?1:-1;
  else                         gsz=(mz>=0)?1:-1;
  bias_ax=mx-gsx*G_MS2; bias_ay=my-gsy*G_MS2; bias_az=mz-gsz*G_MS2;
  bias_gx=sgx/n; bias_gy=sgy/n; bias_gz=sgz/n;
  float ax0=mx-bias_ax,ay0=my-bias_ay,az0=mz-bias_az;
  roll=atan2f(ay0,az0)*180.0f/M_PI;
  pitch=atan2f(-ax0,sqrtf(ay0*ay0+az0*az0))*180.0f/M_PI;
  yaw=0; dr_x=dr_y=dr_vx=dr_vy=0;
  calibrated=true;
  Serial.printf("[CAL] Done. bias_a=(%.3f,%.3f,%.3f) g=(%.2f,%.2f,%.2f)\n",
                bias_ax,bias_ay,bias_az,bias_gx,bias_gy,bias_gz);
}
void updateFusion(float dt){
  float ax=imu.ax,ay=imu.ay,az=imu.az,gx=imu.gx,gy=imu.gy,gz=imu.gz;
  float ra=atan2f(ay,az)*180.0f/M_PI;
  float pa=atan2f(-ax,sqrtf(ay*ay+az*az))*180.0f/M_PI;
  roll =CF_ALPHA*(roll +gx*dt)+(1-CF_ALPHA)*ra;
  pitch=CF_ALPHA*(pitch+gy*dt)+(1-CF_ALPHA)*pa;
  yaw+=gz*dt;
  if(yaw>180)yaw-=360; if(yaw<-180)yaw+=360;
  float yr=yaw*M_PI/180.0f;
  float axw=ax*cosf(yr)-ay*sinf(yr), ayw=ax*sinf(yr)+ay*cosf(yr);
  if(fabsf(axw)<0.15f)axw=0; if(fabsf(ayw)<0.15f)ayw=0;
  dr_vx+=axw*dt; dr_vy+=ayw*dt; dr_vx*=0.98f; dr_vy*=0.98f;
  dr_x+=dr_vx*dt; dr_y+=dr_vy*dt;
}

void handleStatus(){
  char buf[512];
  snprintf(buf,sizeof(buf),
    "{\"ok\":%s,\"cal\":%s,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
    "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
    "\"x\":%.4f,\"y\":%.4f,\"vx\":%.4f,\"vy\":%.4f,"
    "\"temp\":%.1f}",
    imu.ok?"true":"false",calibrated?"true":"false",
    imu.ax,imu.ay,imu.az,imu.gx,imu.gy,imu.gz,
    roll,pitch,yaw,dr_x,dr_y,dr_vx,dr_vy,imu.temp);
  server.send(200,"application/json",buf);
}
void handleReset(){
  yaw=0;dr_x=0;dr_y=0;dr_vx=0;dr_vy=0;
  server.send(200,"application/json","{\"ok\":true}");
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>IMU — Faz 2.5</title>
<style>
:root{--bg:#0a0e13;--card:#111820;--line:#1e2d3d;--txt:#dde6f0;--mut:#5a7a96;
      --ok:#22c55e;--bad:#ef4444;--warn:#f59e0b;--cal:#a78bfa}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font:13px/1.5 system-ui,sans-serif;
     padding:12px;max-width:600px;margin:0 auto}
h1{font-size:16px;font-weight:700;margin-bottom:2px}
.sub{font-size:11px;color:var(--mut);margin-bottom:10px}
.badges{display:flex;gap:8px;margin-bottom:12px;flex-wrap:wrap;align-items:center}
.badge{padding:3px 10px;border-radius:99px;font-size:11px;font-weight:700}
.ok{background:#14532d;color:#86efac}.bad{background:#450a0a;color:#fca5a5}
.calok{background:#2e1065;color:#c4b5fd}.calno{background:#1c1917;color:#78716c}
.btn{background:#1e2d3d;border:1px solid var(--line);color:var(--txt);padding:4px 12px;
     border-radius:6px;font-size:12px;cursor:pointer}
.btn:hover{background:#253545}
.gauges{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:8px}
.gauge{background:var(--card);border:1px solid var(--line);border-radius:12px;
       padding:10px;text-align:center}
.gauge .gl{font-size:10px;color:var(--mut);text-transform:uppercase;letter-spacing:.8px}
.gauge .gv{font-size:20px;font-weight:700;font-family:monospace;margin:4px 0}
.gauge .gu{font-size:10px;color:var(--mut)}
.rc{color:#3b82f6}.pc{color:#8b5cf6}.yc{color:#22c55e}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}
.card h2{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;
         letter-spacing:.8px;margin-bottom:8px}
.row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid var(--line)}
.row:last-child{border-bottom:none}
.lbl{font-size:11px;color:var(--mut)}
.val{font-size:14px;font-weight:700;font-family:monospace}
.unit{font-size:10px;color:var(--mut);margin-left:3px}
.map-card{background:var(--card);border:1px solid var(--line);border-radius:12px;
          padding:12px;margin-bottom:8px}
.map-card h2{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;
             letter-spacing:.8px;margin-bottom:8px}
.warn{font-size:10px;color:var(--warn);margin-top:6px}
canvas{display:block;margin:0 auto;border-radius:8px}
</style></head><body>
<h1>IMU Test — Faz 2.5</h1>
<div class="sub">MPU6050 · complementary filter · dead-reckoning</div>
<div class="badges">
  <span id="badge" class="badge bad">NO DATA</span>
  <span id="calbadge" class="badge calno">CALIBRATING...</span>
  <button class="btn" onclick="resetPos()">Reset Yaw + Konum</button>
</div>
<div class="gauges">
  <div class="gauge"><div class="gl">Roll</div>
    <div class="gv rc" id="groll">—</div><div class="gu">°</div></div>
  <div class="gauge"><div class="gl">Pitch</div>
    <div class="gv pc" id="gpitch">—</div><div class="gu">°</div></div>
  <div class="gauge"><div class="gl">Yaw</div>
    <div class="gv yc" id="gyaw">—</div><div class="gu">°</div></div>
</div>
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
<div class="map-card">
  <h2>Dead-Reckoning  <span style="color:var(--mut);font-weight:400" id="xyval"></span></h2>
  <canvas id="map" width="560" height="200"></canvas>
  <div class="warn">⚠ Drift uyarısı — gerçek konum için Faz 5 EKF gerekir.</div>
</div>
<script>
const MC=document.getElementById('map'); const mctx=MC.getContext('2d');
let trail=[{x:0,y:0}]; let scale=50;
function drawMap(curX,curY,yawDeg){
  const w=MC.width,h=MC.height,cx=w/2,cy=h/2;
  mctx.clearRect(0,0,w,h); mctx.fillStyle='#0d1520'; mctx.fillRect(0,0,w,h);
  mctx.strokeStyle='#1e2d3d'; mctx.lineWidth=1;
  for(let g=-6;g<=6;g++){
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
  mctx.beginPath();mctx.moveTo(px,py);mctx.lineTo(px+20*Math.sin(yr),py-20*Math.cos(yr));
  mctx.strokeStyle='#22c55e';mctx.lineWidth=2.5;mctx.stroke();
  mctx.beginPath();mctx.arc(cx,cy,4,0,2*Math.PI);mctx.fillStyle='#f59e0b';mctx.fill();
  mctx.fillStyle='#5a7a96';mctx.font='10px system-ui';mctx.textAlign='left';
  mctx.fillText('1 grid='+(100/scale*10).toFixed(0)+'cm',8,h-6);
}
async function tick(){
  try{
    const d=await(await fetch('/status',{cache:'no-store'})).json();
    document.getElementById('badge').textContent=d.ok?'CONNECTED':'IMU ERROR';
    document.getElementById('badge').className='badge '+(d.ok?'ok':'bad');
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
    const last=trail[trail.length-1];
    if(Math.abs(d.x-last.x)>0.001||Math.abs(d.y-last.y)>0.001) trail.push({x:d.x,y:d.y});
    if(trail.length>400) trail.shift();
    const maxD=trail.reduce((m,p)=>Math.max(m,Math.abs(p.x),Math.abs(p.y)),0.1);
    scale=Math.min(180,(MC.height/2-20)/maxD);
    drawMap(d.x,d.y,d.yaw);
  }catch(e){}
}
async function resetPos(){ await fetch('/reset',{cache:'no-store'}); trail=[{x:0,y:0}]; }
setInterval(tick,100);
</script>
</body></html>
)HTML";

void handleRoot(){ server.send_P(200,"text/html",INDEX_HTML); }

void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("\n========================================");
  Serial.println("  Faz 2.5 — IMU + Orientation + DR");
  Serial.println("========================================");
  Wire.begin(PIN_SDA,PIN_SCL); Wire.setClock(400000);
  mpuFound=mpuWake();
  if(mpuFound){ Serial.printf("[IMU] Found at 0x%02X\n",MPU_ADDR); calibrate(); }
  else         { Serial.printf("[IMU] NOT found!\n"); }
  Serial.printf("[WiFi] Connecting to '%s'...\n",WIFI_SSID);
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID,WIFI_PASS);
  unsigned long t0=millis();
  while(WiFi.status()!=WL_CONNECTED&&millis()-t0<15000){delay(250);Serial.print(".");}
  Serial.println();
  if(WiFi.status()==WL_CONNECTED)
    Serial.printf("[WiFi] http://%s:%d/\n",WiFi.localIP().toString().c_str(),WEB_PORT);
  else Serial.println("[WiFi] FAILED");
  server.on("/",       handleRoot);
  server.on("/status", handleStatus);
  server.on("/reset",  handleReset);
  server.onNotFound([](){server.send(404,"text/plain","not found");});
  server.begin();
  Serial.println("[READY]");
  lastLoopUs=micros();
}

void loop(){
  server.handleClient();
  unsigned long now=millis();
  if(now-lastReadMs>=READ_INTERVAL_MS){
    lastReadMs=now;
    unsigned long nowUs=micros();
    float dt=(nowUs-lastLoopUs)*1e-6f;
    lastLoopUs=nowUs;
    if(dt>0.1f) dt=0.02f;
    if(!mpuFound){ mpuFound=mpuWake(); if(mpuFound) calibrate(); return; }
    ImuRaw raw;
    if(mpuReadRaw(raw)){
      raw.ax-=bias_ax;raw.ay-=bias_ay;raw.az-=bias_az;
      raw.gx-=bias_gx;raw.gy-=bias_gy;raw.gz-=bias_gz;
      imu=raw;
      if(calibrated) updateFusion(dt);
    } else { imu.ok=false; mpuFound=false; }
  }
}
