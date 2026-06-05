/*
 * Faz2 Direct Drive — ESC + Servo (ESP32-S3)
 * ─────────────────────────────────────────────────────────────────────────────
 * ARM/DISARM/watchdog yok — slider = direkt çıkış.
 * Port: 5001
 *
 * ── KABLOLAMA ─────────────────────────────────────────────────────────────
 *
 *  ESP32-S3 GPIO bağlantıları:
 *  ┌─────────────────────────────┬──────────────┐
 *  │ ESC sinyal (beyaz/sarı tel) │ → GPIO 5     │
 *  │ Servo sinyal (sarı/turuncu) │ → GPIO 4     │
 *  │ ESC GND (siyah)             │ → GND        │
 *  │ Servo GND (siyah/kahverengi)│ → GND        │
 *  │ ESC kırmızı (BEC 5V)        │ BAĞLAMA*     │
 *  │ Servo kırmızı (VCC)         │ BAĞLAMA*     │
 *  └─────────────────────────────┴──────────────┘
 *  * USB ile besleniyorsa kırmızı telleri ESP32'ye bağlama.
 *    Kırmızıları birbirine bağla (ESC BEC → Servo VCC).
 *
 *  Güç bağlantıları:
 *  ┌──────────────────────────────────────────────┐
 *  │ LiPo (+) → ESC kalın kırmızı kablo           │
 *  │ LiPo (-) → ESC kalın siyah kablo             │
 *  │ ESC BEC kırmızı → Servo kırmızı (5V güç)     │
 *  │ USB → ESP32 (kod + serial monitor için)       │
 *  └──────────────────────────────────────────────┘
 *
 * KULLANIM:
 *   Yükle → Serial Monitor 115200 → IP'yi gör
 *   Tarayıcı: http://<IP>:5001/
 *   W/S = ileri/geri, A/D = direksiyon, Space = dur
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>

// ============================ AYARLAR =======================================
const char* WIFI_SSID = "Hezer";
const char* WIFI_PASS = "burakhezer";
const char* MDNS_HOST = "arac";
const int   WEB_PORT  = 5001;

// ---- Pinler ----
const int PIN_SERVO = 4;
const int PIN_ESC   = 5;

// ---- Servo PWM (µs) ----
const int SERVO_MIN    = 1000;
const int SERVO_CENTER = 2200;
const int SERVO_MAX    = 4000;
const int SERVO_DIR    = 1;    // 1 = normal, -1 = direksiyon ters ise

// ---- ESC PWM (µs) ----
const int ESC_MIN_US  = 1200;
const int ESC_MAX_US  = 1800;
const int ESC_NEUTRAL = 1500;
const int MOTOR_DIR   = -1;   // 1 = normal, -1 = motor yönü ters ise

// ---- Durum ----
int throttleLimit  = 30;
int curThrottlePct = 0;
int curSteerPct    = 0;
int appliedEscUs   = ESC_NEUTRAL;
int appliedServoUs = SERVO_CENTER;

Servo servo;
Servo esc;
WebServer server(WEB_PORT);

// ============================ AKTÜATÖR ======================================
void writeServoPct(int pct) {
  pct = constrain(pct, -100, 100) * SERVO_DIR;
  int us = SERVO_CENTER + (int)((long)pct * (SERVO_MAX - SERVO_MIN) / 2 / 100);
  us = constrain(us, SERVO_MIN, SERVO_MAX);
  servo.writeMicroseconds(us);
  curSteerPct    = pct * SERVO_DIR;  // ekranda orijinal yönü göster
  appliedServoUs = us;
}

// Çoğu ESC'de geri vites için: fren → nötr → geri sırası gerekir
static int prevThrottlePct = 0;

void writeThrottlePct(int pct) {
  pct = constrain(pct, -100, 100);

  // İleri/nötrden geriye geçiş: ESC reverse unlock sırası
  if (pct < 0 && prevThrottlePct >= 0) {
    Serial.println("[ESC] Reverse sequence: brake -> neutral -> reverse");
    esc.writeMicroseconds(1300);
    delay(200);
    esc.writeMicroseconds(ESC_NEUTRAL);
    delay(60);
  }

  prevThrottlePct = pct;
  curThrottlePct  = pct;
  int eff = pct * throttleLimit / 100 * MOTOR_DIR;
  int us  = ESC_NEUTRAL + (int)((long)eff * (ESC_MAX_US - ESC_MIN_US) / 2 / 100);
  us = constrain(us, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(us);
  appliedEscUs = us;
  if (pct != 0)
    Serial.printf("[ESC] throttle:%3d%%  eff:%3d%%  us:%d | steer:%3d%%  servo:%d us\n",
                  pct, eff, us, curSteerPct, appliedServoUs);
}

// ============================ JSON ==========================================
String statusJson() {
  return "{\"throttle\":" + String(curThrottlePct) +
         ",\"steer\":"    + String(curSteerPct)    +
         ",\"escUs\":"    + String(appliedEscUs)   +
         ",\"servoUs\":"  + String(appliedServoUs) +
         ",\"limit\":"    + String(throttleLimit)  + "}";
}
void sendStatus() { server.send(200, "application/json", statusJson()); }

// ============================ HTTP ==========================================
void handleControl() {
  int thr   = server.hasArg("throttle") ? server.arg("throttle").toInt() : 0;
  int steer = server.hasArg("steer")    ? server.arg("steer").toInt()    : 0;
  writeServoPct(steer);
  writeThrottlePct(thr);
  sendStatus();
}

void handleSetLimit() {
  if (server.hasArg("v")) {
    int prev = throttleLimit;
    throttleLimit = constrain(server.arg("v").toInt(), 1, 100);
    Serial.printf("[LIMIT] %d%% -> %d%%\n", prev, throttleLimit);
  }
  sendStatus();
}

void handleRoot();

// ============================ WiFi ==========================================
void connectWifi() {
  Serial.printf("\n[WiFi] Connecting to '%s'...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected! IP: %s | RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    if (MDNS.begin(MDNS_HOST)) MDNS.addService("http", "tcp", WEB_PORT);
    Serial.printf("[WiFi] URL: http://%s:%d/\n",
                  WiFi.localIP().toString().c_str(), WEB_PORT);
  } else {
    Serial.println("[WiFi] FAILED. Will retry.");
  }
}

// ============================ SETUP / LOOP ==================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Faz2 Direct Drive — ESC + Servo (port 5001) ===");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, SERVO_MIN, SERVO_MAX);
  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC, ESC_MIN_US, ESC_MAX_US);

  writeServoPct(0);
  esc.writeMicroseconds(ESC_NEUTRAL);
  Serial.printf("[INIT] Servo GPIO%d | ESC GPIO%d | limit %d%%\n",
                PIN_SERVO, PIN_ESC, throttleLimit);

  connectWifi();

  server.on("/",         handleRoot);
  server.on("/control",  handleControl);
  server.on("/setlimit", handleSetLimit);
  server.on("/status",   []() { sendStatus(); });
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
  server.begin();
  Serial.printf("[HTTP] Server on port %d\n", WEB_PORT);
  Serial.println("[READY] Open browser → URL above.");
}

void loop() {
  server.handleClient();
  static unsigned long lastRetry = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastRetry > 5000) {
    lastRetry = millis();
    Serial.println("[WiFi] Reconnecting...");
    WiFi.reconnect();
  }
}

// ============================ WEB UI ========================================
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no"/>
<title>RC Direct Drive</title>
<style>
:root{--bg:#0a0e13;--card:#111820;--card2:#161e28;--line:#1e2d3d;
      --txt:#dde6f0;--mut:#5a7a96;--ok:#22c55e;--warn:#f59e0b;--bad:#ef4444;--accent:#3b82f6;}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font:14px/1.5 system-ui,sans-serif;
     padding:12px;max-width:580px;margin:0 auto;min-height:100vh}
.header{display:flex;align-items:center;justify-content:space-between;margin-bottom:6px}
.header h1{font-size:17px;font-weight:700}
.subtitle{font-size:11px;color:var(--mut);margin-bottom:12px}
.badge{padding:4px 12px;border-radius:999px;font-weight:700;font-size:12px;background:#14532d;color:#86efac}
.statusbar{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:6px;margin-bottom:10px}
.scard{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:8px 10px;text-align:center}
.scard .lbl{font-size:10px;color:var(--mut);text-transform:uppercase;letter-spacing:.7px;margin-bottom:3px}
.scard .val{font-size:16px;font-weight:700}
.ctrlcard{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:10px}
.card-title{font-size:11px;font-weight:700;color:var(--mut);text-transform:uppercase;letter-spacing:.8px;margin-bottom:10px}
.slider-row{display:flex;align-items:center;gap:12px;margin-bottom:10px}
.slider-row:last-of-type{margin-bottom:0}
.slabel{width:82px;font-size:11px;color:var(--mut);text-transform:uppercase;letter-spacing:.5px;flex-shrink:0}
.slider-row input[type=range]{flex:1;height:38px;cursor:pointer;accent-color:var(--accent)}
#steer{accent-color:#a78bfa}
#limitSlider{accent-color:var(--warn)}
.sval{width:52px;text-align:right;font-size:13px;font-weight:700;flex-shrink:0}
.info-box{background:var(--card2);border:1px solid var(--line);border-radius:8px;padding:10px 12px;
          margin-top:10px;font-size:12px;color:var(--mut);line-height:1.6}
.info-box b{color:var(--txt)}
.btn-stop{border:0;border-radius:10px;padding:17px;font-size:16px;font-weight:700;color:#fff;cursor:pointer;
          width:100%;background:linear-gradient(135deg,#dc2626,#b91c1c);transition:opacity .1s}
.btn-stop:active{opacity:.7}
.kbdtag{background:#1a2535;border:1px solid #2d4a63;border-radius:4px;padding:1px 6px;
        font:700 10px monospace;color:var(--txt)}
.keyhints{display:flex;flex-wrap:wrap;gap:6px 14px;margin-top:10px}
.kh{font-size:11px;color:var(--mut)}
.logcard{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:10px}
.log-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:8px}
.log-list{max-height:160px;overflow-y:auto;font-size:12px;font-family:monospace}
.log-list::-webkit-scrollbar{width:4px}.log-list::-webkit-scrollbar-thumb{background:var(--line);border-radius:2px}
.log-entry{display:flex;gap:8px;padding:4px 0;border-bottom:1px solid var(--line)}
.log-entry:last-child{border-bottom:none}
.log-ts{color:var(--mut);flex-shrink:0;min-width:56px}
.log-entry.stop .log-msg{color:var(--bad);font-weight:700}
.log-entry.limit .log-msg{color:var(--accent)}
.log-entry.wifi .log-msg{color:#a78bfa}
.log-clear{background:none;border:1px solid var(--line);color:var(--mut);border-radius:6px;
           padding:2px 8px;font-size:11px;cursor:pointer}
</style></head><body>

<div class="header">
  <h1>RC Direct Drive</h1>
  <span class="badge">LIVE</span>
</div>
<div class="subtitle">ESC + Servo · no ARM/DISARM · dead-man · port 5001 · ESP32-S3</div>

<div class="statusbar">
  <div class="scard"><div class="lbl">Throttle</div><div class="val" id="thval">0%</div></div>
  <div class="scard"><div class="lbl">Steering</div><div class="val" id="stval">0</div></div>
  <div class="scard"><div class="lbl">ESC (µs)</div><div class="val" id="escval">1500</div></div>
  <div class="scard"><div class="lbl">Limit</div><div class="val" id="limitval">30%</div></div>
</div>

<div class="ctrlcard">
  <div class="card-title">Controls</div>
  <div class="slider-row">
    <span class="slabel">Throttle</span>
    <input id="throttle" type="range" min="-100" max="100" value="0"/>
    <span class="sval" id="thval2">0%</span>
  </div>
  <div class="slider-row">
    <span class="slabel">Steering</span>
    <input id="steer" type="range" min="-100" max="100" value="0"/>
    <span class="sval" id="stval2">0</span>
  </div>
  <div class="info-box">
    <b>Dead-man:</b> release slider or key → instantly returns to zero.
  </div>
</div>

<div class="ctrlcard">
  <div class="card-title">Throttle Limit</div>
  <div class="slider-row">
    <span class="slabel">Max power</span>
    <input id="limitSlider" type="range" min="1" max="100" value="30"/>
    <span class="sval" id="limitval2">30%</span>
  </div>
  <div class="info-box">
    Caps maximum ESC output. At 30%, slider 100% → ESC gets 30% of full range.
    <b>Keep low for first tests.</b>
  </div>
</div>

<div class="ctrlcard">
  <button class="btn-stop" onclick="doStop()">
    STOP &nbsp;<span class="kbdtag">SPACE</span>
  </button>
  <div class="keyhints">
    <span class="kh"><span class="kbdtag">W</span>/<span class="kbdtag">↑</span> forward</span>
    <span class="kh"><span class="kbdtag">S</span>/<span class="kbdtag">↓</span> reverse</span>
    <span class="kh"><span class="kbdtag">A</span>/<span class="kbdtag">←</span> steer left</span>
    <span class="kh"><span class="kbdtag">D</span>/<span class="kbdtag">→</span> steer right</span>
    <span class="kh"><span class="kbdtag">SPACE</span> stop all</span>
  </div>
</div>

<div class="logcard">
  <div class="log-header">
    <span class="card-title" style="margin-bottom:0">Activity Log</span>
    <button class="log-clear" onclick="clearLog()">Clear</button>
  </div>
  <div class="log-list" id="logList">
    <div class="log-entry wifi"><span class="log-ts">--:--:--</span><span class="log-msg">Connecting...</span></div>
  </div>
</div>

<script>
let throttle=0,steer=0,sending=false,pendingLimit=null,firstStatus=true;
const $=id=>document.getElementById(id);

function ts(){return new Date().toTimeString().slice(0,8);}
function addLog(msg,type=''){
  const list=$('logList');
  if(list.children.length===1&&list.children[0].querySelector('.log-msg').textContent==='Connecting...')
    list.innerHTML='';
  const el=document.createElement('div');
  el.className='log-entry'+(type?' '+type:'');
  el.innerHTML=`<span class="log-ts">${ts()}</span><span class="log-msg">${msg}</span>`;
  list.prepend(el);
  while(list.children.length>80) list.removeChild(list.lastChild);
}
function clearLog(){$('logList').innerHTML='';}

function setThrottle(v){
  throttle=Math.max(-100,Math.min(100,v|0));
  $('throttle').value=throttle;
  $('thval').textContent=throttle+'%';
  $('thval2').textContent=throttle+'%';
}
function setSteer(v){
  steer=Math.max(-100,Math.min(100,v|0));
  $('steer').value=steer;
  $('stval').textContent=steer;
  $('stval2').textContent=steer;
}
function doStop(){setThrottle(0);setSteer(0);addLog('STOP — all zeroed','stop');}

$('throttle').addEventListener('input',e=>setThrottle(+e.target.value));
$('steer').addEventListener('input',e=>setSteer(+e.target.value));
['pointerup','pointercancel','touchend'].forEach(ev=>{
  $('throttle').addEventListener(ev,()=>setThrottle(0));
  $('steer').addEventListener(ev,()=>setSteer(0));
});

function applyLimitUI(v){$('limitval').textContent=v+'%';$('limitval2').textContent=v+'%';$('limitSlider').value=v;}
$('limitSlider').addEventListener('input',e=>{applyLimitUI(+e.target.value);pendingLimit=+e.target.value;});
['pointerup','touchend'].forEach(ev=>$('limitSlider').addEventListener(ev,()=>{
  if(pendingLimit!==null){
    const v=pendingLimit; pendingLimit=null;
    addLog(`Limit → ${v}%`,'limit');
    fetch('/setlimit?v='+v,{cache:'no-store'}).then(r=>r.json()).then(applyStatus);
  }
}));

const keys={};
addEventListener('keydown',e=>{
  const k=e.key.toLowerCase();
  if(k===' '){e.preventDefault();doStop();return;}
  keys[k]=true;applyKeys();
});
addEventListener('keyup',e=>{keys[e.key.toLowerCase()]=false;applyKeys();});
function applyKeys(){
  let t=0,s=0;
  if(keys['w']||keys['arrowup'])t=100;
  if(keys['s']||keys['arrowdown'])t=-100;
  if(keys['d']||keys['arrowright'])s=100;
  if(keys['a']||keys['arrowleft'])s=-100;
  setThrottle(t);setSteer(s);
}

function applyStatus(st){
  applyLimitUI(st.limit);
  $('escval').textContent=st.escUs;
  if(firstStatus){addLog('Connected to ESP32','wifi');firstStatus=false;}
}

async function tick(){
  if(sending)return;
  sending=true;
  try{
    const r=await fetch(`/control?throttle=${throttle}&steer=${steer}`,{cache:'no-store'});
    if(r.ok)applyStatus(await r.json());
  }catch(e){}finally{sending=false;}
}
setInterval(tick,120);
</script>
</body></html>
)HTMLPAGE";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}
