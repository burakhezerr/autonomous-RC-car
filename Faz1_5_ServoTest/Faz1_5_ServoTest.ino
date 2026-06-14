/*
 * Faz 1.5 — Servo + ESC Manuel Test (ESP32-S3)
 * Servo: GPIO4  |  ESC: GPIO5
 * Web UI: http://<IP>:5004/
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

const char* WIFI_SSID = "Hezer";
const char* WIFI_PASS = "burakhezer";
const int   WEB_PORT  = 5004;

const int PIN_SERVO    = 38;
const int SERVO_MIN    = 1700;
const int SERVO_CENTER = 2000;
const int SERVO_MAX    = 2300;

const int PIN_ESC    = 41;
const int ESC_MIN    = 1000;
const int ESC_NEUTRAL= 1500;
const int ESC_MAX    = 1650;

Servo servo;
Servo esc;
WebServer server(WEB_PORT);

int servoUs = SERVO_CENTER;
int escUs   = ESC_NEUTRAL;

// ── HTML ────────────────────────────────────────────────────────────────────
const char PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head><meta charset="utf-8">
<title>Servo + ESC Test</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
  body{font-family:sans-serif;background:#111;color:#eee;display:flex;flex-direction:column;align-items:center;padding:24px;gap:16px}
  h2{margin:0}
  .card{background:#1e1e1e;border-radius:12px;padding:24px;width:320px;text-align:center}
  .lbl{font-size:0.85em;color:#888;margin-bottom:4px}
  .val{font-size:2.2em;font-weight:bold;margin:6px 0}
  .servo-val{color:#4fc3f7}
  .esc-val{color:#81c784}
  .unit{font-size:0.85em;color:#888}
  input[type=range]{width:100%;margin:12px 0}
  input.servo-slider{accent-color:#4fc3f7}
  input.esc-slider{accent-color:#81c784}
  .btns{display:flex;gap:8px;justify-content:center;margin-top:8px}
  button{padding:9px 16px;border:none;border-radius:8px;cursor:pointer;font-size:0.9em;background:#2a2a2a;color:#eee}
  button:hover{filter:brightness(1.4)}
  .btn-s{background:#1a3a4a}
  .btn-e{background:#1a3a1a}
</style></head><body>
<h2>Servo + ESC Test</h2>

<div class="card">
  <div class="lbl">SERVO (Yön)</div>
  <div class="val servo-val" id="sVal">2500</div>
  <div class="unit">µs</div>
  <input class="servo-slider" type="range" min="2400" max="2600" step="5" value="2500" id="sSlider"
    oninput="setServo(this.value)">
  <div class="btns">
    <button class="btn-s" onclick="setServo(2400)">SOL<br><small>2400</small></button>
    <button class="btn-s" onclick="setServo(2500)">ORTA<br><small>2500</small></button>
    <button class="btn-s" onclick="setServo(2600)">SAĞ<br><small>2600</small></button>
  </div>
</div>

<div class="card">
  <div class="lbl">ESC (Hız)</div>
  <div class="val esc-val" id="eVal">1500</div>
  <div class="unit">µs</div>
  <input class="esc-slider" type="range" min="1000" max="1650" step="5" value="1500" id="eSlider"
    oninput="setEsc(this.value)">
  <div class="btns">
    <button class="btn-e" onclick="setEsc(1000)">MIN<br><small>1000</small></button>
    <button class="btn-e" onclick="setEsc(1500)">NÖTR<br><small>1500</small></button>
    <button class="btn-e" onclick="setEsc(1650)">MAX<br><small>1650</small></button>
  </div>
</div>

<script>
function setServo(us){
  document.getElementById('sVal').textContent=us;
  document.getElementById('sSlider').value=us;
  fetch('/set?servo='+us);
}
function setEsc(us){
  document.getElementById('eVal').textContent=us;
  document.getElementById('eSlider').value=us;
  fetch('/set?esc='+us);
}
</script>
</body></html>
)rawhtml";

void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleSet() {
  if (server.hasArg("servo")) {
    servoUs = constrain(server.arg("servo").toInt(), SERVO_MIN, SERVO_MAX);
    servo.writeMicroseconds(servoUs);
    Serial.printf("[SERVO] %d µs\n", servoUs);
  }
  if (server.hasArg("esc")) {
    escUs = constrain(server.arg("esc").toInt(), ESC_MIN, ESC_MAX);
    esc.writeMicroseconds(escUs);
    Serial.printf("[ESC]   %d µs\n", escUs);
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  servo.attach(PIN_SERVO, SERVO_MIN, SERVO_MAX);
  servo.writeMicroseconds(SERVO_CENTER);
  Serial.printf("[SERVO] Center (%d µs)\n", SERVO_CENTER);

  esc.attach(PIN_ESC, ESC_MIN, ESC_MAX);
  esc.writeMicroseconds(ESC_NEUTRAL);
  Serial.printf("[ESC]   Nötr (%d µs) — ESC arming için 2 sn bekle\n", ESC_NEUTRAL);
  delay(2000);
  Serial.println("[ESC]   Hazır");

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Bağlanıyor");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("[Web]  http://%s:%d/\n", WiFi.localIP().toString().c_str(), WEB_PORT);

  server.on("/",    handleRoot);
  server.on("/set", handleSet);
  server.begin();
}

void loop() {
  server.handleClient();
}
