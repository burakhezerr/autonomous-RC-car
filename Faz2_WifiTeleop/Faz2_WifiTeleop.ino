/*
 * ============================================================================
 *  Faz 2 — WiFi Teleop  (ESP32-S3)
 *  Autonomous RC Car Racing Platform / Kontrol Katmani
 * ============================================================================
 *
 *  AMAC:
 *    ESP32-S3'u bir WiFi agina baglar, kendi icine gomulu bir web arayuzu
 *    sunar. Tarayicidan (telefon/laptop) gaz + direksiyon ile araci ELLE
 *    surersin. Faz 1'deki serial komutlarinin (s/c/a/g/x) web karsiligi +
 *    gercek teleop kontrolleri.
 *
 *  ----------------------------------------------------------------------------
 *  >>> GUVENLIK MIMARISI (oku) <<<
 *  ----------------------------------------------------------------------------
 *    1) Motor acilista DISARMED. Gaz yalnizca ARMED iken uygulanir.
 *    2) DEAD-MAN kontrol: tarayicida gaz/direksiyon BIRAKILINCA sifira/merkeze
 *       doner. Surmek icin basili tutmak gerekir.
 *    3) FAIL-SAFE WATCHDOG: ~CMD_TIMEOUT_MS boyunca komut gelmezse (WiFi koptu,
 *       sekme kapandi, telefon uyudu) motor notr'a alinir VE sistem DISARMED'a
 *       kilitlenir. Baglanti geri gelse bile motor KENDILIGINDEN DEVAM ETMEZ;
 *       tekrar ARM gerekir. (Elin gazda iken kopup geri gelirse surpriz
 *       kalkis olmasin diye.)
 *    4) Hicbir HTTP handler'da bloklama (delay) YOK -> E-STOP her an islenir.
 *       Arming, bloklamayan zamanli bir durum makinesidir.
 *    5) THROTTLE_LIMIT_PCT ile tam slider != tam hiz. Ilk testlerde dusuk tut.
 *    6) TEKERLER HAVADA ilk testte. Ortak GND + ESP32'den motor/servo BESLEME.
 *       (Detay: Faz 1 sketch yorumlari.)
 *
 *  ----------------------------------------------------------------------------
 *  >>> KURULUM <<<
 *  ----------------------------------------------------------------------------
 *    - Asagidaki WIFI_SSID / WIFI_PASS degerlerini KENDI aginla doldur.
 *    - Kart: ESP32S3 Dev Module. Kutuphane: ESP32Servo (+ cekirdek WiFi/WebServer/ESPmDNS dahili).
 *    - Yukle, Serial Monitor (115200) ac. Baglaninca IP + http://arac.local/ yazar.
 *    - Telefon/laptop AYNI WiFi aginda olmali. Tarayicidan o adrese git.
 *
 *  NOT: arduino-cli olmadigi icin bu kod DERLENEREK/DONANIMDA dogrulanmadi;
 *  mantik okuma ile dogrulanmistir. Ilk yuklemede Serial loglarini izle.
 * ============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>

// ============================ AYARLAR =======================================
const char* WIFI_SSID = "WIFI_ADINI_YAZ";       // <-- KENDI aginla doldur
const char* WIFI_PASS = "WIFI_SIFRENI_YAZ";     // <-- KENDI aginla doldur
const char* MDNS_HOST = "arac";                  // http://arac.local/
const int   WEB_PORT  = 80;                      // 80 ise tarayicida port yazmana gerek yok

// ---- Pin tanimlari (Faz 1 ile ayni) ----
const int PIN_SERVO = 4;
const int PIN_ESC   = 5;

// ---- PWM araliklari (mikrosaniye) ----
const int SERVO_MIN    = 1000;
const int SERVO_CENTER = 1500;
const int SERVO_MAX    = 2000;
const int ESC_MIN_US   = 1000;
const int ESC_MAX_US   = 2000;
const int ESC_NEUTRAL  = 1500;   // cift yonlu RC ESC notru

// ---- Guvenlik parametreleri ----
const int  THROTTLE_LIMIT_PCT = 30;    // tam slider'in % kaci uygulanir (dusuk basla!)
const unsigned long ARM_DURATION_MS = 3000;  // ESC arming icin notr sinyal suresi
const unsigned long CMD_TIMEOUT_MS  = 500;   // bu sure komut gelmezse fail-safe
const unsigned long WIFI_RETRY_MS   = 5000;  // WiFi kopunca yeniden deneme araligi

// ============================ DURUM =========================================
enum MotorState { DISARMED, ARMING, ARMED };
volatile MotorState motorState = DISARMED;

unsigned long armStartMs   = 0;   // ARMING baslangici
unsigned long lastCmdMs    = 0;   // son /control komutu (watchdog)
unsigned long lastWifiTry  = 0;
bool commLost = false;            // watchdog tetiklendi mi (UI bilgisi)

int curSteerPct    = 0;           // -100..100 (son uygulanan)
int curThrottlePct = 0;           // -100..100 (kullanicidan gelen, ham)
int appliedEscUs   = ESC_NEUTRAL; // gercekte ESC'ye giden

Servo servo;
Servo esc;
WebServer server(WEB_PORT);

// ============================ AKTUATOR ======================================
void writeServoPct(int steerPct) {
  steerPct = constrain(steerPct, -100, 100);
  int us = SERVO_CENTER + (int)((long)steerPct * (SERVO_MAX - SERVO_MIN) / 2 / 100);
  us = constrain(us, SERVO_MIN, SERVO_MAX);
  servo.writeMicroseconds(us);
  curSteerPct = steerPct;
}

void writeEscNeutral() {
  esc.writeMicroseconds(ESC_NEUTRAL);
  appliedEscUs = ESC_NEUTRAL;
}

// Yalnizca ARMED iken gercek gaz uygular; aksi halde notr.
void writeThrottlePct(int throttlePct) {
  throttlePct = constrain(throttlePct, -100, 100);
  curThrottlePct = throttlePct;
  if (motorState != ARMED) { writeEscNeutral(); return; }
  int eff = throttlePct * THROTTLE_LIMIT_PCT / 100;          // limit uygula
  int us  = ESC_NEUTRAL + (int)((long)eff * (ESC_MAX_US - ESC_MIN_US) / 2 / 100);
  us = constrain(us, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(us);
  appliedEscUs = us;
}

// ============================ DURUM GECISLERI ===============================
void disarm(const char* reason) {
  motorState = DISARMED;
  writeEscNeutral();
  curThrottlePct = 0;
  Serial.printf("[STATE] DISARMED (%s)\n", reason);
}

void estop() {
  disarm("E-STOP");
  writeServoPct(0);   // direksiyonu da merkeze al
}

void beginArming() {
  if (motorState != DISARMED) return;
  motorState = ARMING;
  armStartMs = millis();
  lastCmdMs  = millis();
  commLost   = false;
  writeEscNeutral();
  Serial.println("[STATE] ARMING (notr sinyal gonderiliyor)...");
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
  j += "\"limit\":" + String(THROTTLE_LIMIT_PCT);
  j += "}";
  return j;
}

void sendStatus() {
  server.send(200, "application/json", statusJson());
}

// ============================ HTTP HANDLER'LAR ==============================
void handleControl() {
  // /control?steer=<-100..100>&throttle=<-100..100>  (periyodik heartbeat)
  lastCmdMs = millis();
  commLost  = false;
  int steer = server.hasArg("steer") ? server.arg("steer").toInt() : 0;
  int thr   = server.hasArg("throttle") ? server.arg("throttle").toInt() : 0;
  writeServoPct(steer);          // direksiyon her durumda uygulanir (zararsiz)
  writeThrottlePct(thr);         // gaz yalnizca ARMED iken
  sendStatus();
}

void handleArm()    { beginArming();        sendStatus(); }
void handleDisarm() { disarm("web disarm"); sendStatus(); }
void handleEstop()  { estop();              sendStatus(); }
void handleStatus() { sendStatus(); }

void handleRoot();   // (HTML asagida)

// ============================ KURULUM =======================================
void connectWifi() {
  Serial.printf("[WiFi] '%s' agina baglaniliyor...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);                  // sadece KURULUMDA; loop'ta delay yok
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Baglandi. IP: %s\n", WiFi.localIP().toString().c_str());
    if (MDNS.begin(MDNS_HOST)) {
      MDNS.addService("http", "tcp", WEB_PORT);
      Serial.printf("[WiFi] Adres: http://%s.local/  (veya http://%s/)\n",
                    MDNS_HOST, WiFi.localIP().toString().c_str());
    }
  } else {
    Serial.println("[WiFi] BAGLANAMADI. loop'ta yeniden denenecek.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, SERVO_MIN, SERVO_MAX);
  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC, ESC_MIN_US, ESC_MAX_US);

  // Guvenli baslangic
  writeServoPct(0);
  writeEscNeutral();
  motorState = DISARMED;

  connectWifi();

  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.on("/arm", handleArm);
  server.on("/disarm", handleDisarm);
  server.on("/estop", handleEstop);
  server.on("/status", handleStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "yok"); });
  server.begin();
  Serial.printf("[HTTP] Sunucu port %d'de basladi.\n", WEB_PORT);
}

// ============================ ANA DONGU =====================================
void loop() {
  server.handleClient();

  // --- WiFi yeniden baglanma (bloklamadan) ---
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > WIFI_RETRY_MS) {
    lastWifiTry = millis();
    Serial.println("[WiFi] kopuk -> reconnect deneniyor");
    WiFi.reconnect();
    // Not: WiFi kopuksa watchdog zaten motoru notr'a alir; iki katman birlikte calisir.
  }

  // --- Arming durum makinesi (bloklamayan) ---
  if (motorState == ARMING && millis() - armStartMs >= ARM_DURATION_MS) {
    motorState = ARMED;
    Serial.println("[STATE] ARMED. Artik gaz uygulanir (dead-man + watchdog aktif).");
  }

  // --- Fail-safe watchdog ---
  if (motorState != DISARMED && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    commLost = true;
    disarm("watchdog timeout");   // DISARMED'a kilitlenir; tekrar ARM gerekir
  }
}

// ============================ WEB ARAYUZU ===================================
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html><html lang="tr"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no"/>
<title>RC Araba Teleop</title>
<style>
  :root{--bg:#0f1419;--card:#1b232c;--line:#2c3946;--txt:#e6edf3;--mut:#8b97a3;
        --ok:#2ea043;--warn:#d29922;--bad:#f85149;--accent:#388bfd;}
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  body{margin:0;background:var(--bg);color:var(--txt);font:15px/1.4 system-ui,sans-serif;
       padding:14px;max-width:560px;margin:0 auto}
  h1{font-size:18px;margin:4px 0 2px}
  .sub{color:var(--mut);font-size:12px;margin-bottom:12px}
  .card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
  .row{display:flex;gap:10px}
  .pill{flex:1;text-align:center;padding:8px;border-radius:8px;border:1px solid var(--line);
        background:#0d1117;font-weight:600;font-size:13px}
  .badge{display:inline-block;padding:2px 10px;border-radius:999px;font-weight:700;font-size:13px}
  .b-dis{background:#30363d;color:var(--mut)} .b-ing{background:var(--warn);color:#1b1b00}
  .b-arm{background:var(--ok);color:#04210c}
  label{display:block;color:var(--mut);font-size:12px;margin:2px 0 6px}
  input[type=range]{width:100%;height:42px}
  .vbox{display:flex;gap:16px;align-items:stretch}
  .vthrottle{writing-mode:vertical-lr;direction:rtl;height:200px;width:48px}
  .steerwrap{flex:1;display:flex;flex-direction:column;justify-content:flex-end}
  button{border:0;border-radius:10px;padding:14px;font-size:15px;font-weight:700;color:#fff;width:100%}
  .arm{background:var(--ok)} .disarm{background:#30363d} .estop{background:var(--bad);font-size:18px;padding:18px}
  .grid3{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}
  .hint{color:var(--mut);font-size:11px;margin-top:8px}
  .warn{color:var(--warn);font-weight:700}
</style></head><body>

<h1>RC Araba — WiFi Teleop</h1>
<div class="sub">Dead-man kontrol: birakinca durur. WiFi/sekme koparsa otomatik durur.</div>

<div class="card">
  <div class="row">
    <div class="pill">Durum: <span id="state" class="badge b-dis">DISARMED</span></div>
    <div class="pill">Gaz limiti: <span id="limit">?</span>%</div>
  </div>
  <div id="commLost" class="hint warn" style="display:none">⚠ Baglanti koptu — motor durduruldu, yeniden ARM gerekir.</div>
</div>

<div class="card vbox">
  <div style="text-align:center">
    <label>GAZ</label>
    <input id="throttle" class="vthrottle" type="range" min="-100" max="100" value="0"/>
    <div id="thval" class="pill" style="margin-top:6px">0%</div>
  </div>
  <div class="steerwrap">
    <label>DIREKSIYON</label>
    <input id="steer" type="range" min="-100" max="100" value="0"/>
    <div id="stval" class="pill" style="margin-top:6px">0</div>
    <div class="hint">Klavye: W/S veya yon tuslari = gaz, A/D = direksiyon. Birakinca sifir.</div>
  </div>
</div>

<div class="card">
  <button class="estop" onclick="cmd('estop')">■ ACIL DUR (E-STOP)</button>
  <div class="grid3">
    <button class="arm" onclick="cmd('arm')">ARM</button>
    <button class="disarm" onclick="cmd('disarm')">DISARM</button>
  </div>
  <div class="hint">Once <b>ARM</b> → ~3 sn ESC hazirlanir → sonra gaz calisir. Tekerler havada test et.</div>
</div>

<script>
let steer=0, throttle=0, sending=false;
const $=id=>document.getElementById(id);

function setSteer(v){steer=Math.max(-100,Math.min(100,v|0));$('steer').value=steer;$('stval').textContent=steer;}
function setThrottle(v){throttle=Math.max(-100,Math.min(100,v|0));$('throttle').value=throttle;$('thval').textContent=throttle+'%';}

$('steer').addEventListener('input',e=>setSteer(+e.target.value));
$('throttle').addEventListener('input',e=>setThrottle(+e.target.value));
// Birakinca sifira/merkeze don (dead-man)
['pointerup','pointercancel','mouseleave','touchend'].forEach(ev=>{
  $('steer').addEventListener(ev,()=>setSteer(0));
  $('throttle').addEventListener(ev,()=>setThrottle(0));
});

// Klavye
const keys={};
addEventListener('keydown',e=>{keys[e.key.toLowerCase()]=true;applyKeys();});
addEventListener('keyup',e=>{keys[e.key.toLowerCase()]=false;applyKeys();});
function applyKeys(){
  let t=0,s=0;
  if(keys['w']||keys['arrowup'])t=100; if(keys['s']||keys['arrowdown'])t=-100;
  if(keys['d']||keys['arrowright'])s=100; if(keys['a']||keys['arrowleft'])s=-100;
  setThrottle(t);setSteer(s);
}

function applyStatus(st){
  const b=$('state'); b.textContent=st.state;
  b.className='badge '+(st.state==='ARMED'?'b-arm':st.state==='ARMING'?'b-ing':'b-dis');
  $('limit').textContent=st.limit;
  $('commLost').style.display=st.commLost?'block':'none';
}

// Periyodik heartbeat — in-flight guard ile (eskimis komut ezilmesin)
async function tick(){
  if(sending)return;             // onceki istek bitmediyse atla
  sending=true;
  try{
    const r=await fetch(`/control?steer=${steer}&throttle=${throttle}`,{cache:'no-store'});
    if(r.ok)applyStatus(await r.json());
  }catch(e){/* ag hatasi: ESP32 watchdog'u zaten durduracak */}
  finally{sending=false;}
}
setInterval(tick,120);           // ~8 Hz heartbeat (watchdog 500ms'den kisa)

// Buton komutlari
async function cmd(name){
  try{const r=await fetch('/'+name,{cache:'no-store'}); if(r.ok)applyStatus(await r.json());}catch(e){}
}
// Not: sekme arka plana alininca tarayici zamanlayiciyi yavaslatir -> watchdog
// tetiklenir ve araba guvenle durur. Bu beklenen davranistir, hata degil.
</script>
</body></html>
)HTMLPAGE";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}
