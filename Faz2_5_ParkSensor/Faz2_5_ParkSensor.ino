/*
 * ============================================================================
 *  Faz 2.5 — MPU6050 IMU + HC-SR04 Park Sensörü  (ESP32-S3)
 * ============================================================================
 *
 *  KABLOLAMA:
 *    MPU6050  VCC→3.3V  GND→GND  SDA→GPIO8  SCL→GPIO9
 *    HC-SR04  VCC→5V    GND→GND  TRIG→GPIO10  ECHO→GPIO11
 *
 *    ⚠ ECHO 5V çıkış yapar!  Güvenli bağlantı:
 *       ECHO → 1kΩ → GPIO11 → 2kΩ → GND  (voltaj bölücü)
 *
 *  WEB UI:  http://<IP>:5002/
 * ============================================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

const char* WIFI_SSID = "Hezer";
const char* WIFI_PASS = "burakhezer";
const int   WEB_PORT  = 5002;

// ---- IMU ----
const int     PIN_SDA        = 8;
const int     PIN_SCL        = 9;
const uint8_t MPU_ADDR       = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_XOUT = 0x3B;
const float   ACCEL_SCALE    = 16384.0f;
const float   GYRO_SCALE     = 131.0f;
const float   G_MS2          = 9.81f;
const float   CF_ALPHA       = 0.98f;

// ---- HC-SR04 ----
const int   PIN_TRIG = 10;
const int   PIN_ECHO = 11;
// Mesafe eşikleri (cm) — araç boyutuna göre ayarla
const float PROX_L1 =  8.0f;   //  0-8 cm  → 1. derece (kırmızı)
const float PROX_L2 = 20.0f;   //  8-20 cm → 2. derece (turuncu)
const float PROX_L3 = 50.0f;   // 20-50 cm → 3. derece (sarı)
const float PROX_L4 = 75.0f;   // 50-75 cm → 4. derece (yeşil)
                                // > 75 cm  → cisim yok

const unsigned long IMU_INTERVAL_MS   = 20;   // 50 Hz
const unsigned long SONAR_INTERVAL_MS = 80;   // ~12 Hz
const int           CAL_SAMPLES       = 300;

// ============================ DURUM =========================================
struct ImuRaw { float ax,ay,az,gx,gy,gz,temp; bool ok; };
ImuRaw imu = {};
bool   mpuFound=false, calibrated=false;

float bias_ax=0,bias_ay=0,bias_az=0;
float bias_gx=0,bias_gy=0,bias_gz=0;
float roll=0,pitch=0,yaw=0;
float dr_x=0,dr_y=0,dr_vx=0,dr_vy=0;

float sonarCm    = -1.0f;   // -1 = cisim yok / timeout
int   proxLevel  = 0;       // 0=yok, 1-4

unsigned long lastImuMs   = 0;
unsigned long lastSonarMs = 0;
unsigned long lastLoopUs  = 0;

WebServer server(WEB_PORT);

// ============================ MPU6050 =======================================
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
    delay(IMU_INTERVAL_MS);
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
  dr_vx+=axw*dt; dr_vy+=ayw*dt;
  dr_vx*=0.98f; dr_vy*=0.98f;
  dr_x+=dr_vx*dt; dr_y+=dr_vy*dt;
}

// ============================ HC-SR04 =======================================
float measureSonar(){
  digitalWrite(PIN_TRIG,LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG,HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG,LOW);
  long dur=pulseIn(PIN_ECHO,HIGH,20000);  // 20 ms timeout → ~340 cm max
  if(dur==0) return -1.0f;
  return dur*0.01716f;  // cm
}
int calcProxLevel(float cm){
  if(cm<0)           return 0;
  if(cm<PROX_L1)     return 1;
  if(cm<PROX_L2)     return 2;
  if(cm<PROX_L3)     return 3;
  if(cm<PROX_L4)     return 4;
  return 0;
}

// ============================ HTTP ==========================================
void handleStatus(){
  char buf[640];
  snprintf(buf,sizeof(buf),
    "{\"ok\":%s,\"cal\":%s,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
    "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
    "\"x\":%.4f,\"y\":%.4f,"
    "\"dist\":%.1f,\"prox\":%d,"
    "\"temp\":%.1f}",
    imu.ok?"true":"false",calibrated?"true":"false",
    imu.ax,imu.ay,imu.az,imu.gx,imu.gy,imu.gz,
    roll,pitch,yaw,dr_x,dr_y,
    sonarCm,proxLevel,imu.temp);
  server.send(200,"application/json",buf);
}
void handleReset(){
  yaw=0;dr_x=0;dr_y=0;dr_vx=0;dr_vy=0;
  server.send(200,"application/json","{\"ok\":true}");
}

// ============================ WEB UI ========================================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>IMU + Park — Faz 2.5</title>
<style>
:root{--bg:#0a0e13;--card:#111820;--line:#1e2d3d;--txt:#dde6f0;--mut:#5a7a96;
      --ok:#22c55e;--bad:#ef4444;--warn:#f59e0b;--cal:#a78bfa;
      --p1:#ef4444;--p2:#f97316;--p3:#eab308;--p4:#22c55e}
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

/* ── PARK SENSOR ── */
.park-card{background:var(--card);border:1px solid var(--line);border-radius:12px;
           padding:16px;margin-bottom:10px}
.park-card h2{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;
              letter-spacing:.8px;margin-bottom:12px}
.park-display{display:flex;flex-direction:column;align-items:center;gap:12px}
canvas#park{display:block}
.park-info{display:flex;align-items:center;gap:20px}
.park-dist-box{text-align:center}
.park-dist-num{font-size:36px;font-weight:700;font-family:monospace;transition:color .2s}
.park-dist-unit{font-size:12px;color:var(--mut)}
.level-bars{display:flex;gap:6px;align-items:flex-end;height:48px}
.lb{width:22px;border-radius:4px 4px 0 0;transition:opacity .2s,background .2s}
.lb-4{height:24px;background:var(--p4)}.lb-3{height:32px;background:var(--p3)}
.lb-2{height:40px;background:var(--p2)}.lb-1{height:48px;background:var(--p1)}
.lb.off{opacity:.15}
.park-label{font-size:13px;font-weight:700;letter-spacing:.3px;transition:color .2s}

/* ── GAUGES ── */
.gauges{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:8px}
.gauge{background:var(--card);border:1px solid var(--line);border-radius:12px;
       padding:10px;text-align:center}
.gauge .gl{font-size:10px;color:var(--mut);text-transform:uppercase;letter-spacing:.8px}
.gauge .gv{font-size:20px;font-weight:700;font-family:monospace;margin:4px 0}
.gauge .gu{font-size:10px;color:var(--mut)}
.rc{color:#3b82f6}.pc{color:#8b5cf6}.yc{color:#22c55e}

/* ── RAW ── */
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}
.card h2{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;
         letter-spacing:.8px;margin-bottom:8px}
.row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid var(--line)}
.row:last-child{border-bottom:none}
.lbl{font-size:11px;color:var(--mut)}
.val{font-size:14px;font-weight:700;font-family:monospace}
.unit{font-size:10px;color:var(--mut);margin-left:3px}

/* ── MAP ── */
.map-card{background:var(--card);border:1px solid var(--line);border-radius:12px;
          padding:12px;margin-bottom:8px}
.map-card h2{font-size:10px;font-weight:700;color:var(--mut);text-transform:uppercase;
             letter-spacing:.8px;margin-bottom:8px}
.warn{font-size:10px;color:var(--warn);margin-top:6px}
canvas{display:block;margin:0 auto;border-radius:8px}
</style></head><body>

<h1>IMU + Park Sensörü — Faz 2.5</h1>
<div class="sub">MPU6050 · HC-SR04 · ESP32-S3</div>
<div class="badges">
  <span id="badge" class="badge bad">NO DATA</span>
  <span id="calbadge" class="badge calno">CALIBRATING...</span>
  <button class="btn" onclick="resetPos()">Reset Yaw + Konum</button>
</div>

<!-- PARK SENSOR -->
<div class="park-card">
  <h2>Park Sensörü (HC-SR04)</h2>
  <div class="park-display">
    <canvas id="park" width="340" height="180"></canvas>
    <div class="park-info">
      <div class="park-dist-box">
        <div class="park-dist-num" id="pdist">—</div>
        <div class="park-dist-unit">cm</div>
      </div>
      <div class="level-bars">
        <div class="lb lb-4 off" id="lb4"></div>
        <div class="lb lb-3 off" id="lb3"></div>
        <div class="lb lb-2 off" id="lb2"></div>
        <div class="lb lb-1 off" id="lb1"></div>
      </div>
      <div class="park-label" id="plabel" style="color:var(--mut)">Cisim yok</div>
    </div>
  </div>
</div>

<!-- AÇILAR -->
<div class="gauges">
  <div class="gauge"><div class="gl">Roll</div>
    <div class="gv rc" id="groll">—</div><div class="gu">°</div></div>
  <div class="gauge"><div class="gl">Pitch</div>
    <div class="gv pc" id="gpitch">—</div><div class="gu">°</div></div>
  <div class="gauge"><div class="gl">Yaw</div>
    <div class="gv yc" id="gyaw">—</div><div class="gu">°</div></div>
</div>

<!-- RAW -->
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

<!-- MAP -->
<div class="map-card">
  <h2>Dead-Reckoning  <span style="color:var(--mut);font-weight:400" id="xyval"></span></h2>
  <canvas id="map" width="560" height="200"></canvas>
  <div class="warn">⚠ Drift uyarısı — gerçek konum için Faz 5 EKF gerekir.</div>
</div>

<script>
// ── Park sensor canvas ──────────────────────────────────────────────────────
const PC = document.getElementById('park');
const pctx = PC.getContext('2d');
const COLORS = ['#22c55e','#eab308','#f97316','#ef4444'];
const THRESHOLDS = [90, 60, 35, 15]; // L4,L3,L2,L1 dış kenarı (cm)

function drawPark(dist, level) {
  const w=PC.width, h=PC.height;
  const cx=w/2, cy=h-20;
  const maxR=h-30;
  pctx.clearRect(0,0,w,h);
  pctx.fillStyle='#0d1520'; pctx.fillRect(0,0,w,h);

  // Arcs (dıştan içe: L4→L3→L2→L1)
  const radii = [maxR, maxR*0.72, maxR*0.50, maxR*0.30, maxR*0.14];
  for(let i=0;i<4;i++){
    const active = level >= (4-i);
    pctx.beginPath();
    pctx.moveTo(cx,cy);
    pctx.arc(cx,cy,radii[i],Math.PI,2*Math.PI);
    pctx.arc(cx,cy,radii[i+1],2*Math.PI,Math.PI,true);
    pctx.closePath();
    pctx.fillStyle = active ? COLORS[i] : '#1a2535';
    pctx.fill();
    pctx.strokeStyle='#0a0e13'; pctx.lineWidth=2; pctx.stroke();
  }

  // Threshold labels
  const labels=['75','50','20','8'];
  pctx.fillStyle='#5a7a96'; pctx.font='10px system-ui'; pctx.textAlign='center';
  for(let i=0;i<4;i++){
    const r=radii[i]-8;
    pctx.fillText(labels[i]+'cm', cx+r*0.85, cy-4);
  }

  // Sensor icon
  pctx.beginPath(); pctx.arc(cx,cy,8,0,2*Math.PI);
  pctx.fillStyle='#3b82f6'; pctx.fill();

  // Object marker (if detected)
  if(dist>0 && dist<340){
    const ratio = Math.max(0, Math.min(1, dist/75));
    const r = ratio*maxR;
    pctx.beginPath(); pctx.arc(cx, cy-r, 5, 0, 2*Math.PI);
    pctx.fillStyle='#fff'; pctx.fill();
  }
}

// ── Dead-reckoning map ──────────────────────────────────────────────────────
const MC=document.getElementById('map'); const mctx=MC.getContext('2d');
let trail=[{x:0,y:0}]; let scale=50;
function drawMap(curX,curY,yawDeg){
  const w=MC.width,h=MC.height,cx=w/2,cy=h/2;
  mctx.clearRect(0,0,w,h);
  mctx.fillStyle='#0d1520'; mctx.fillRect(0,0,w,h);
  mctx.strokeStyle='#1e2d3d'; mctx.lineWidth=1;
  for(let g=-6;g<=6;g++){
    const gx=cx+g*scale,gy=cy+g*scale;
    mctx.beginPath();mctx.moveTo(gx,0);mctx.lineTo(gx,h);mctx.stroke();
    mctx.beginPath();mctx.moveTo(0,gy);mctx.lineTo(w,gy);mctx.stroke();
  }
  mctx.strokeStyle='#2d4a63';mctx.lineWidth=1.5;
  mctx.beginPath();mctx.moveTo(0,cy);mctx.lineTo(w,cy);mctx.stroke();
  mctx.beginPath();mctx.moveTo(cx,0);mctx.lineTo(cx,h);mctx.stroke();
  if(trail.length>1){
    mctx.beginPath();
    mctx.moveTo(cx+trail[0].x*scale, cy-trail[0].y*scale);
    for(let i=1;i<trail.length;i++) mctx.lineTo(cx+trail[i].x*scale,cy-trail[i].y*scale);
    mctx.strokeStyle='#3b82f6';mctx.lineWidth=2;mctx.stroke();
  }
  const px=cx+curX*scale, py=cy-curY*scale;
  const yr=yawDeg*Math.PI/180;
  mctx.beginPath();mctx.arc(px,py,6,0,2*Math.PI);mctx.fillStyle='#22c55e';mctx.fill();
  mctx.beginPath();mctx.moveTo(px,py);
  mctx.lineTo(px+20*Math.sin(yr),py-20*Math.cos(yr));
  mctx.strokeStyle='#22c55e';mctx.lineWidth=2.5;mctx.stroke();
  mctx.beginPath();mctx.arc(cx,cy,4,0,2*Math.PI);mctx.fillStyle='#f59e0b';mctx.fill();
  mctx.fillStyle='#5a7a96';mctx.font='10px system-ui';mctx.textAlign='left';
  mctx.fillText('1 grid='+(100/scale*10).toFixed(0)+'cm',8,h-6);
}

// ── Proximity UI update ─────────────────────────────────────────────────────
const PROX_COLORS=['var(--mut)','var(--p1)','var(--p2)','var(--p3)','var(--p4)'];
const PROX_LABELS=['Cisim yok','● 1. Derece — Çok yakın!','● 2. Derece — Yakın','● 3. Derece — Orta','● 4. Derece — Uzak'];
function updateProx(dist, level){
  const el=document.getElementById('pdist');
  el.textContent = dist<0 ? '—' : dist.toFixed(1);
  el.style.color = level>0 ? PROX_COLORS[level] : 'var(--txt)';
  document.getElementById('plabel').textContent=PROX_LABELS[level];
  document.getElementById('plabel').style.color=PROX_COLORS[level];
  for(let i=1;i<=4;i++){
    const lb=document.getElementById('lb'+i);
    lb.classList.toggle('off', level<i);
  }
  drawPark(dist, level);
}

// ── Main poll ───────────────────────────────────────────────────────────────
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
    document.getElementById('xyval').textContent=
      'x='+d.x.toFixed(3)+'m  y='+d.y.toFixed(3)+'m';
    updateProx(d.dist, d.prox);
    const last=trail[trail.length-1];
    if(Math.abs(d.x-last.x)>0.001||Math.abs(d.y-last.y)>0.001)
      trail.push({x:d.x,y:d.y});
    if(trail.length>400) trail.shift();
    const maxD=trail.reduce((m,p)=>Math.max(m,Math.abs(p.x),Math.abs(p.y)),0.1);
    scale=Math.min(180,(MC.height/2-20)/maxD);
    drawMap(d.x,d.y,d.yaw);
  }catch(e){}
}
async function resetPos(){
  await fetch('/reset',{cache:'no-store'});
  trail=[{x:0,y:0}];
}
setInterval(tick,100);
</script>
</body></html>
)HTML";

void handleRoot(){ server.send_P(200,"text/html",INDEX_HTML); }

// ============================ SETUP =========================================
void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("\n========================================");
  Serial.println("  Faz 2.5 — IMU + Park Sensörü");
  Serial.println("========================================");

  pinMode(PIN_TRIG,OUTPUT); pinMode(PIN_ECHO,INPUT);
  digitalWrite(PIN_TRIG,LOW);
  Serial.printf("[SONAR] HC-SR04 TRIG→GPIO%d  ECHO→GPIO%d\n",PIN_TRIG,PIN_ECHO);

  Wire.begin(PIN_SDA,PIN_SCL); Wire.setClock(400000);
  mpuFound=mpuWake();
  if(mpuFound){ Serial.printf("[IMU] MPU6050 found at 0x%02X\n",MPU_ADDR); calibrate(); }
  else         { Serial.printf("[IMU] NOT found at 0x%02X!\n",MPU_ADDR); }

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

// ============================ LOOP ==========================================
void loop(){
  server.handleClient();
  unsigned long now=millis();

  // IMU — 50 Hz
  if(now-lastImuMs>=IMU_INTERVAL_MS){
    lastImuMs=now;
    unsigned long nowUs=micros();
    float dt=(nowUs-lastLoopUs)*1e-6f;
    lastLoopUs=nowUs;
    if(dt>0.1f) dt=0.02f;
    if(!mpuFound){ mpuFound=mpuWake(); if(mpuFound) calibrate(); }
    else{
      ImuRaw raw;
      if(mpuReadRaw(raw)){
        raw.ax-=bias_ax;raw.ay-=bias_ay;raw.az-=bias_az;
        raw.gx-=bias_gx;raw.gy-=bias_gy;raw.gz-=bias_gz;
        imu=raw;
        if(calibrated) updateFusion(dt);
      } else { imu.ok=false; mpuFound=false; }
    }
  }

  // Sonar — ~12 Hz
  if(now-lastSonarMs>=SONAR_INTERVAL_MS){
    lastSonarMs=now;
    sonarCm   = measureSonar();
    proxLevel = calcProxLevel(sonarCm);
  }
}
