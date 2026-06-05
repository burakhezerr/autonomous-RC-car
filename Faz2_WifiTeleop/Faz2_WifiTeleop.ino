/*
 * ============================================================================
 *  Faz 2 — WiFi Teleop  (ESP32-S3)
 *  Otonom RC Araba Yarış Platformu / Kontrol Katmanı
 * ============================================================================
 *
 *  AMAÇ:
 *    ESP32-S3'ü bir WiFi ağına bağlar; içine gömülü web arayüzü sunar.
 *    Tarayıcıdan (telefon/laptop) gaz + direksiyon ile araç EL ile sürülür.
 *    Faz 1 serial komutlarının web karşılığı + gerçek teleop kontrolleri.
 *
 *  GÜVENLİK MİMARİSİ:
 *    1) Motor açılışta DISARMED; gaz yalnızca ARMED iken uygulanır.
 *    2) DEAD-MAN: tarayıcıda tuş/slider bırakılınca değer sıfıra döner.
 *    3) WATCHDOG: CMD_TIMEOUT_MS boyunca komut gelmezse motor nötr + DISARMED
 *       kilidine girer — bağlantı geri gelse bile kendiliğinden devam etmez.
 *    4) HTTP handler'larda delay() YOK → E-STOP her an işlenir.
 *    5) Gaz limiti (throttleLimit) web UI'dan anlık değiştirilebilir.
 *    6) İlk testler TEKERLER HAVADA. Ortak GND; ESP32 motor/servo beslemez.
 *
 *  KURULUM:
 *    - WIFI_SSID / WIFI_PASS değerlerini kendi ağınla doldur.
 *    - Kart: ESP32S3 Dev Module. Kütüphane: ESP32Servo.
 *    - Yükle, Serial Monitor (115200) aç. Bağlanınca IP adresi yazılır.
 *    - Telefon/laptop AYNI ağda olmalı → tarayıcıdan http://<IP>:5000/ git.
 * ============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>

// ============================ AYARLAR =======================================
const char* WIFI_SSID = "Hezer";
const char* WIFI_PASS = "burakhezer";
const char* MDNS_HOST = "arac";    // mDNS adı: http://arac.local:5000/
const int   WEB_PORT  = 5000;

// ---- Pin tanımları (Faz 1 ile aynı) ----
const int PIN_SERVO = 4;
const int PIN_ESC   = 5;

// ---- PWM aralıkları (mikrosaniye) ----
const int SERVO_MIN    = 1000;
const int SERVO_CENTER = 1500;
const int SERVO_MAX    = 2000;
const int ESC_MIN_US   = 1000;
const int ESC_MAX_US   = 2000;
const int ESC_NEUTRAL  = 1500;   // çift yönlü RC ESC nötrü

// ---- Güvenlik parametreleri ----
int throttleLimit = 30;                       // web UI'dan anlık değiştirilebilir (1-100)
const unsigned long ARM_DURATION_MS = 3000;  // ESC arming için nötr sinyal süresi
const unsigned long CMD_TIMEOUT_MS  = 1500;  // bu süre komut gelmezse fail-safe tetiklenir (mobil tarayıcı throttle payı)
const unsigned long WIFI_RETRY_MS   = 5000;  // WiFi kopunca yeniden deneme aralığı

// ============================ DURUM =========================================
enum MotorState { DISARMED, ARMING, ARMED };
volatile MotorState motorState = DISARMED;

unsigned long armStartMs   = 0;   // ARMING başlangıç zamanı
unsigned long lastCmdMs    = 0;   // son /control isteği (watchdog referansı)
unsigned long lastWifiTry  = 0;   // son WiFi reconnect denemesi
bool commLost = false;            // watchdog tetiklendi mi (UI'ya bildirilir)

int curSteerPct    = 0;           // -100..100 (son uygulanan direksiyon)
int curThrottlePct = 0;           // -100..100 (kullanıcıdan gelen ham gaz)
int appliedEscUs   = ESC_NEUTRAL; // gerçekte ESC'ye yazılan µs değeri
int appliedServoUs = SERVO_CENTER; // gerçekte servo'ya yazılan µs değeri

Servo servo;
Servo esc;
WebServer server(WEB_PORT);

// ============================ AKTÜATÖR ======================================

// Direksiyon yüzdesini (-100..100) servo µs değerine çevirir ve yazar
void writeServoPct(int steerPct) {
  steerPct = constrain(steerPct, -100, 100);
  int us = SERVO_CENTER + (int)((long)steerPct * (SERVO_MAX - SERVO_MIN) / 2 / 100);
  us = constrain(us, SERVO_MIN, SERVO_MAX);
  servo.writeMicroseconds(us);
  curSteerPct    = steerPct;
  appliedServoUs = us;
}

// ESC'yi kesin nötr konumuna alır (güvenli bekleme pozisyonu)
void writeEscNeutral() {
  esc.writeMicroseconds(ESC_NEUTRAL);
  appliedEscUs = ESC_NEUTRAL;
}

// Gaz yüzdesini ESC'ye yazar; yalnızca ARMED iken gerçek gaz uygulanır
void writeThrottlePct(int throttlePct) {
  throttlePct = constrain(throttlePct, -100, 100);
  curThrottlePct = throttlePct;
  if (motorState != ARMED) { writeEscNeutral(); return; }
  int eff = throttlePct * throttleLimit / 100;   // kullanıcı limitini uygula
  int us  = ESC_NEUTRAL + (int)((long)eff * (ESC_MAX_US - ESC_MIN_US) / 2 / 100);
  us = constrain(us, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(us);
  appliedEscUs = us;
}

// ============================ DURUM GEÇİŞLERİ ===============================

void disarm(const char* reason) {
  motorState = DISARMED;
  writeEscNeutral();
  curThrottlePct = 0;
  Serial.printf("[STATE] DISARMED | reason: %s | ESC -> neutral (%d us) | uptime: %lu s\n",
                reason, ESC_NEUTRAL, millis() / 1000);
}

// E-STOP: motoru kapat + direksiyonu merkeze al
void estop() {
  disarm("E-STOP");
  writeServoPct(0);
  Serial.println("[ESTOP] Steering centered. System locked until re-ARM.");
}

// Arming sürecini başlatır; ESC ARM_DURATION_MS boyunca nötr sinyal alır
void beginArming() {
  if (motorState != DISARMED) return;
  motorState = ARMING;
  armStartMs = millis();
  lastCmdMs  = millis();
  commLost   = false;
  writeEscNeutral();
  Serial.printf("[STATE] ARMING | sending neutral signal for %lu ms | throttleLimit: %d%%\n",
                ARM_DURATION_MS, throttleLimit);
}

const char* stateName() {
  switch (motorState) {
    case DISARMED: return "DISARMED";
    case ARMING:   return "ARMING";
    case ARMED:    return "ARMED";
  }
  return "?";
}

// ============================ JSON DURUM ====================================
String statusJson() {
  String j = "{";
  j += "\"state\":\"" + String(stateName()) + "\",";
  j += "\"steer\":" + String(curSteerPct) + ",";
  j += "\"throttle\":" + String(curThrottlePct) + ",";
  j += "\"escUs\":" + String(appliedEscUs) + ",";
  j += "\"commLost\":" + String(commLost ? "true" : "false") + ",";
  j += "\"limit\":" + String(throttleLimit);
  j += "}";
  return j;
}

void sendStatus() {
  server.send(200, "application/json", statusJson());
}

// ============================ HTTP HANDLER'LAR ==============================

// Periyodik heartbeat: ~8 Hz tarayıcıdan gelir; steer + throttle değerlerini uygular
void handleControl() {
  lastCmdMs = millis();
  commLost  = false;
  int steer = server.hasArg("steer")    ? server.arg("steer").toInt()    : 0;
  int thr   = server.hasArg("throttle") ? server.arg("throttle").toInt() : 0;
  writeServoPct(steer);   // direksiyon her durumda uygulanır (güvenli)
  writeThrottlePct(thr);  // gaz yalnızca ARMED iken ESC'ye iletilir
  sendStatus();
}

void handleArm()    { beginArming();        sendStatus(); }
void handleDisarm() { disarm("web-disarm"); sendStatus(); }
void handleEstop()  { estop();              sendStatus(); }
void handleStatus() { sendStatus(); }

// /setlimit?v=<1-100> — gaz limitini anlık değiştirir, ARMED iken hemen uygular
void handleSetLimit() {
  if (server.hasArg("v")) {
    int prev = throttleLimit;
    int v    = server.arg("v").toInt();
    throttleLimit = constrain(v, 1, 100);
    Serial.printf("[LIMIT] Throttle limit changed: %d%% -> %d%% | state: %s\n",
                  prev, throttleLimit, stateName());
    if (motorState == ARMED) writeThrottlePct(curThrottlePct);
  }
  sendStatus();
}

void handleRoot();   // (HTML aşağıda tanımlı)

// ============================ KURULUM =======================================

// WiFi bağlantısını kurar; setup()'ta çağrılır (sadece burada delay kullanılır)
void connectWifi() {
  Serial.printf("\n[WiFi] Connecting to SSID: '%s' ...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected! IP: %s | RSSI: %d dBm | channel: %d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.channel());
    if (MDNS.begin(MDNS_HOST)) {
      MDNS.addService("http", "tcp", WEB_PORT);
      Serial.printf("[WiFi] mDNS: http://%s.local:%d/\n", MDNS_HOST, WEB_PORT);
    }
    Serial.printf("[WiFi] Direct URL: http://%s:%d/\n",
                  WiFi.localIP().toString().c_str(), WEB_PORT);
  } else {
    Serial.printf("[WiFi] FAILED to connect to '%s' after 15 s. Will retry every %lu s.\n",
                  WIFI_SSID, WIFI_RETRY_MS / 1000);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n========================================");
  Serial.println("  Faz 2 — WiFi Teleop  (ESP32-S3)");
  Serial.println("========================================");

  // PWM zamanlayıcıları ve servo/ESC başlatma
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, SERVO_MIN, SERVO_MAX);
  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC, ESC_MIN_US, ESC_MAX_US);
  Serial.printf("[INIT] Servo -> GPIO%d (%d-%d us) | ESC -> GPIO%d (%d-%d us)\n",
                PIN_SERVO, SERVO_MIN, SERVO_MAX, PIN_ESC, ESC_MIN_US, ESC_MAX_US);

  // Güvenli açılış: motor nötr, sistem kilitli
  writeServoPct(0);
  writeEscNeutral();
  motorState = DISARMED;
  Serial.printf("[INIT] Safe start — state: DISARMED | ESC neutral: %d us | throttleLimit: %d%%\n",
                ESC_NEUTRAL, throttleLimit);

  connectWifi();

  // HTTP rotaları
  server.on("/", handleRoot);
  server.on("/control",  handleControl);
  server.on("/arm",      handleArm);
  server.on("/disarm",   handleDisarm);
  server.on("/estop",    handleEstop);
  server.on("/status",   handleStatus);
  server.on("/setlimit", handleSetLimit);
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
  server.begin();
  Serial.printf("[HTTP] Web server started on port %d\n", WEB_PORT);
  Serial.println("[READY] System ready. Open browser and navigate to the URL above.");
  Serial.println("----------------------------------------");
}

// ============================ ANA DÖNGÜ =====================================
void loop() {
  server.handleClient();

  // WiFi kopuksa bloklamadan yeniden bağlan; watchdog zaten motoru durdurur
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > WIFI_RETRY_MS) {
    lastWifiTry = millis();
    Serial.printf("[WiFi] Link lost — attempting reconnect (uptime: %lu s)\n",
                  millis() / 1000);
    WiFi.reconnect();
  }

  // Arming durum makinesi: ARM_DURATION_MS nötr sinyal tamamlandıktan sonra ARMED
  if (motorState == ARMING && millis() - armStartMs >= ARM_DURATION_MS) {
    motorState = ARMED;
    lastCmdMs  = millis();  // watchdog'u sıfırla — ARMED olur olmaz timeout'u engelle
    Serial.printf("[STATE] ARMED | throttleLimit: %d%% | dead-man + watchdog active\n",
                  throttleLimit);
  }

  // Fail-safe watchdog: yalnızca ARMED iken aktif — ARMING sırasında tetiklenmez
  if (motorState == ARMED && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    commLost = true;
    Serial.printf("[WATCHDOG] No command for >%lu ms — triggering fail-safe\n",
                  CMD_TIMEOUT_MS);
    disarm("watchdog-timeout");
  }
}

// ============================ WEB UI ========================================
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no"/>
<title>RC Teleop</title>
<style>
:root{
  --bg:#0a0e13;--card:#111820;--card2:#161e28;--line:#1e2d3d;
  --txt:#dde6f0;--mut:#5a7a96;--ok:#22c55e;--warn:#f59e0b;--bad:#ef4444;--accent:#3b82f6;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font:14px/1.5 system-ui,sans-serif;
     padding:12px;max-width:620px;margin:0 auto;min-height:100vh}
.header{display:flex;align-items:center;justify-content:space-between;margin-bottom:6px}
.header h1{font-size:17px;font-weight:700;letter-spacing:.3px}
.subtitle{font-size:11px;color:var(--mut);margin-bottom:12px}
.badge{display:inline-flex;align-items:center;padding:4px 12px;border-radius:999px;font-weight:700;font-size:12px;letter-spacing:.5px}
.b-dis{background:#1e2d3d;color:var(--mut)}.b-ing{background:#78350f;color:#fde68a}.b-arm{background:#14532d;color:#86efac}
/* STATUS BAR */
.statusbar{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:10px}
.scard{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px 12px;text-align:center}
.scard .lbl{font-size:10px;color:var(--mut);text-transform:uppercase;letter-spacing:.8px;margin-bottom:4px}
.scard .val{font-size:18px;font-weight:700}
/* CARDS */
.ctrlcard{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:10px}
.card-title{font-size:11px;font-weight:700;color:var(--mut);text-transform:uppercase;letter-spacing:.8px;margin-bottom:10px}
/* SLIDERS */
.slider-row{display:flex;align-items:center;gap:12px;margin-bottom:10px}
.slider-row:last-of-type{margin-bottom:0}
.slabel{width:82px;font-size:11px;color:var(--mut);text-transform:uppercase;letter-spacing:.5px;flex-shrink:0}
.slider-row input[type=range]{flex:1;height:36px;cursor:pointer;accent-color:var(--accent)}
#limitSlider{accent-color:var(--warn)}
.sval{width:48px;text-align:right;font-size:13px;font-weight:700;flex-shrink:0}
.info-box{background:var(--card2);border:1px solid var(--line);border-radius:8px;padding:10px 12px;margin-top:10px;font-size:12px;color:var(--mut);line-height:1.6}
.info-box b{color:var(--txt)}
/* BUTTONS */
.btnrow{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px}
.btn{border:0;border-radius:10px;padding:13px;font-size:14px;font-weight:700;color:#fff;cursor:pointer;
     display:flex;align-items:center;justify-content:center;gap:7px;transition:opacity .1s}
.btn:active{opacity:.7}
.btn-arm{background:linear-gradient(135deg,#16a34a,#15803d)}
.btn-disarm{background:var(--card2);border:1px solid var(--line);color:var(--mut)}
.btn-estop{background:linear-gradient(135deg,#dc2626,#b91c1c);grid-column:1/-1;padding:16px;font-size:16px}
.kbdtag{background:#1a2535;border:1px solid #2d4a63;border-radius:4px;padding:1px 6px;
        font:700 10px monospace;color:var(--txt)}
.btn-arm .kbdtag{background:#166534;border-color:#15803d}
.btn-estop .kbdtag{background:#7f1d1d;border-color:#991b1b}
/* KEYHINTS */
.keyhints{display:flex;flex-wrap:wrap;gap:6px 14px;margin-top:10px}
.kh{font-size:11px;color:var(--mut)}
/* WARN */
.commwarn{background:#431407;border:1px solid #7c2d12;border-radius:8px;padding:8px 12px;
          color:#fca5a5;font-size:12px;font-weight:600;margin-bottom:10px;display:none}
/* ACTIVITY LOG */
.logcard{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:10px}
.log-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:8px}
.log-list{max-height:200px;overflow-y:auto;font-size:12px;font-family:monospace}
.log-list::-webkit-scrollbar{width:4px}.log-list::-webkit-scrollbar-track{background:transparent}
.log-list::-webkit-scrollbar-thumb{background:var(--line);border-radius:2px}
.log-entry{display:flex;gap:8px;padding:4px 0;border-bottom:1px solid var(--line)}
.log-entry:last-child{border-bottom:none}
.log-ts{color:var(--mut);flex-shrink:0;min-width:56px}
.log-msg{flex:1}
.log-entry.estop .log-msg{color:var(--bad);font-weight:700}
.log-entry.armed .log-msg{color:var(--ok)}
.log-entry.arming .log-msg{color:var(--warn)}
.log-entry.disarm .log-msg{color:var(--mut)}
.log-entry.watchdog .log-msg{color:#fb923c;font-weight:700}
.log-entry.limit .log-msg{color:var(--accent)}
.log-entry.wifi .log-msg{color:#a78bfa}
.log-clear{background:none;border:1px solid var(--line);color:var(--mut);border-radius:6px;
           padding:2px 8px;font-size:11px;cursor:pointer}
.log-clear:hover{border-color:var(--mut)}
</style></head><body>

<div class="header">
  <h1>RC Teleop — Phase 2</h1>
  <span id="state" class="badge b-dis">DISARMED</span>
</div>
<div class="subtitle">WiFi teleoperation · dead-man control · fail-safe watchdog · ESP32-S3</div>

<div id="commLost" class="commwarn">
  Connection lost — motor stopped and system locked. Re-ARM required to resume.
</div>

<div class="statusbar">
  <div class="scard"><div class="lbl">Throttle</div><div class="val" id="thval">0%</div></div>
  <div class="scard"><div class="lbl">Steering</div><div class="val" id="stval">0</div></div>
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
    <b>Dead-man control:</b> releasing the slider or key instantly returns the value to zero —
    the car stops on its own. You must actively hold input to keep moving.
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
    <b>Throttle Limit</b> caps the maximum ESC output as a percentage of full power.
    At 30%, even if the slider is pushed to 100%, the ESC only receives 30% of the full
    PWM range. <b>Start low (10–30%) when wheels are on the ground for the first time.</b>
    The limit is sent to the ESP32 immediately when you release the slider.
  </div>
</div>

<div class="ctrlcard">
  <div class="card-title">Arm / Disarm / Emergency Stop</div>
  <div class="btnrow">
    <button class="btn btn-arm" onclick="cmd('arm')">
      ARM <span class="kbdtag">E</span>
    </button>
    <button class="btn btn-disarm" onclick="cmd('disarm')">
      DISARM <span class="kbdtag">Q</span>
    </button>
  </div>
  <button class="btn btn-estop" onclick="cmd('estop')">
    EMERGENCY STOP <span class="kbdtag">SPACE</span>
  </button>
  <div class="info-box" style="margin-top:10px">
    <b>ARM</b> — starts a ~3 s ESC initialization sequence (neutral signal). Throttle only
    activates after arming completes. Status shows <span style="color:#86efac">ARMING</span>
    then <span style="color:#86efac">ARMED</span>.<br><br>
    <b>DISARM</b> — safely cuts throttle and locks the system. Must re-ARM to drive again.<br><br>
    <b>E-STOP (Emergency Stop)</b> — immediately sets motor to neutral and centers steering.
    System is locked and <b>will not resume</b> even if connection is restored — manual
    re-ARM is required. Also triggered automatically if no command is received for
    500 ms (WiFi drop, tab closed, phone screen off).
  </div>
  <div class="keyhints">
    <span class="kh"><span class="kbdtag">W</span><span class="kbdtag">S</span> throttle fwd/rev</span>
    <span class="kh"><span class="kbdtag">A</span><span class="kbdtag">D</span> steering</span>
    <span class="kh"><span class="kbdtag">↑↓←→</span> arrows also work</span>
    <span class="kh"><span class="kbdtag">E</span> ARM</span>
    <span class="kh"><span class="kbdtag">Q</span> DISARM</span>
    <span class="kh"><span class="kbdtag">SPACE</span> E-STOP</span>
  </div>
</div>

<div class="logcard">
  <div class="log-header">
    <span class="card-title" style="margin-bottom:0">Activity Log</span>
    <button class="log-clear" onclick="clearLog()">Clear</button>
  </div>
  <div class="log-list" id="logList">
    <div class="log-entry wifi"><span class="log-ts">--:--:--</span><span class="log-msg">Waiting for first status...</span></div>
  </div>
</div>

<script>
let steer=0,throttle=0,sending=false,pendingLimit=null;
let prevState='',prevCommLost=false,prevLimit=-1;
const $=id=>document.getElementById(id);
const MAX_LOG=80;

function ts(){const d=new Date();return d.toTimeString().slice(0,8);}

function addLog(msg,type=''){
  const list=$('logList');
  // İlk "waiting" satırını kaldır
  if(list.children.length===1&&list.children[0].querySelector('.log-msg').textContent.startsWith('Waiting'))
    list.innerHTML='';
  const el=document.createElement('div');
  el.className='log-entry'+(type?' '+type:'');
  el.innerHTML=`<span class="log-ts">${ts()}</span><span class="log-msg">${msg}</span>`;
  list.prepend(el);  // en yeni üstte
  while(list.children.length>MAX_LOG) list.removeChild(list.lastChild);
}

function clearLog(){$('logList').innerHTML='';}

function setSteer(v){steer=Math.max(-100,Math.min(100,v|0));$('steer').value=steer;$('stval').textContent=steer;$('stval2').textContent=steer;}
function setThrottle(v){throttle=Math.max(-100,Math.min(100,v|0));$('throttle').value=throttle;$('thval').textContent=throttle+'%';$('thval2').textContent=throttle+'%';}

$('steer').addEventListener('input',e=>setSteer(+e.target.value));
$('throttle').addEventListener('input',e=>setThrottle(+e.target.value));
['pointerup','pointercancel','touchend'].forEach(ev=>{
  $('steer').addEventListener(ev,()=>setSteer(0));
  $('throttle').addEventListener(ev,()=>setThrottle(0));
});

function applyLimitUI(v){$('limitval').textContent=v+'%';$('limitval2').textContent=v+'%';$('limitSlider').value=v;}

$('limitSlider').addEventListener('input',e=>{applyLimitUI(+e.target.value);pendingLimit=+e.target.value;});
['pointerup','touchend'].forEach(ev=>$('limitSlider').addEventListener(ev,()=>{
  if(pendingLimit!==null){
    const v=pendingLimit; pendingLimit=null;
    fetch('/setlimit?v='+v,{cache:'no-store'}).then(r=>r.json()).then(st=>{
      addLog(`Throttle limit set to ${v}%`,'limit');
      applyStatus(st);
    });
  }
}));

const keys={};
addEventListener('keydown',e=>{
  const k=e.key.toLowerCase();
  if(k===' '){e.preventDefault();cmd('estop','user key [SPACE]');return;}
  if(k==='e'&&!keys[k]){cmd('arm','user key [E]');}
  if(k==='q'&&!keys[k]){cmd('disarm','user key [Q]');}
  keys[k]=true;applyKeys();
});
addEventListener('keyup',e=>{keys[e.key.toLowerCase()]=false;applyKeys();});
function applyKeys(){
  let t=0,s=0;
  if(keys['w']||keys['arrowup'])t=100; if(keys['s']||keys['arrowdown'])t=-100;
  if(keys['d']||keys['arrowright'])s=100; if(keys['a']||keys['arrowleft'])s=-100;
  setThrottle(t);setSteer(s);
}

function applyStatus(st){
  const b=$('state');
  b.textContent=st.state;
  b.className='badge '+(st.state==='ARMED'?'b-arm':st.state==='ARMING'?'b-ing':'b-dis');
  applyLimitUI(st.limit);
  $('commLost').style.display=st.commLost?'block':'none';

  // --- state transition logging (watchdog / unexpected changes) ---
  if(st.state!==prevState){
    if(st.state==='ARMING')       addLog('ARMING — ESC initializing (~3 s neutral signal)','arming');
    else if(st.state==='ARMED')   addLog('ARMED — throttle active','armed');
    else if(st.state==='DISARMED'&&prevState==='ARMED')
                                  addLog('DISARMED by watchdog — connection timeout','watchdog');
    else if(st.state==='DISARMED'&&prevState==='ARMING')
                                  addLog('DISARMED — arming cancelled','disarm');
    prevState=st.state;
  }

  // --- commLost transition ---
  if(st.commLost&&!prevCommLost)  addLog('CONNECTION LOST — watchdog triggered, system locked','watchdog');
  if(!st.commLost&&prevCommLost)  addLog('Connection restored (re-ARM required)','wifi');
  prevCommLost=st.commLost;
}

async function tick(){
  if(sending)return;
  sending=true;
  try{const r=await fetch(`/control?steer=${steer}&throttle=${throttle}`,{cache:'no-store'});
      if(r.ok)applyStatus(await r.json());}
  catch(e){}finally{sending=false;}
}
setInterval(tick,120);

async function cmd(name,src){
  const label={arm:'ARM requested',disarm:'DISARM requested',estop:'EMERGENCY STOP'}[name]||name;
  const type={arm:'arming',disarm:'disarm',estop:'estop'}[name]||'';
  addLog(label+(src?' ('+src+')':''),type);
  try{const r=await fetch('/'+name,{cache:'no-store'});if(r.ok)applyStatus(await r.json());}catch(e){}
}
</script>
</body></html>
)HTMLPAGE";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}
