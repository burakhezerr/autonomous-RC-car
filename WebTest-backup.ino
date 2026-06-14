/*
 * ============================================================================
 *  Faz 0.5 — WiFi Web Test  (ESP32-S3)
 *  Autonomous RC Car Racing Platform / Kontrol Katmani
 * ============================================================================
 *
 *  KULLANIM:
 *    1) Kodu yukle. Bilgisayari Hezer WiFi'ina bagla.
 *    2) Tarayicida: http://172.20.10.4:5000  (veya esp.local:5000)
 *
 *  PINLER:
 *    Sag ultrasonik TRIG/ECHO : 10 / 11
 *    Sol ultrasonik TRIG/ECHO : 12 / 13
 *    MPU6050 SDA/SCL           :  6 /  7  (encoder -> 8/9 ile takas)
 *    TF-Luna TX->RX / RX->TX  : 16 / 17  (UART1, 115200)
 *
 *  Board: ESP32S3 Dev Module  |  CDCOnBoot: Enabled
 * ============================================================================
 */

 #include <WiFi.h>
 #include <WebServer.h>
 #include <ESPmDNS.h>
 #include <Wire.h>
 
 // ---------------------------------------------------------------------------
 // WiFi
 // ---------------------------------------------------------------------------
 const char* SSID     = "Hezer";
 const char* PASSWORD = "burakhezer";
 const int   WEB_PORT  = 5000;
 
 // ---------------------------------------------------------------------------
 // Pinler
 // ---------------------------------------------------------------------------
 const int TRIG_R = 10;
 const int ECHO_R = 11;
 const int TRIG_L = 12;
 const int ECHO_L = 13;
 
 const int SDA_PIN   = 6;
 const int SCL_PIN   = 7;
 
 const int LUNA_RX   = 16;   // TF-Luna TX -> buraya
 const int LUNA_TX   = 17;   // TF-Luna RX -> buraya (opsiyonel)
 
 // ---------------------------------------------------------------------------
 // MPU6050
 // ---------------------------------------------------------------------------
 #define MPU_ADDR   0x68
 #define PWR_MGMT_1 0x6B
 #define ACCEL_OUT  0x3B
 #define GYRO_OUT   0x43
 #define TEMP_OUT   0x41
 
 struct ImuData {
   float ax, ay, az;   // g
   float gx, gy, gz;   // deg/s
   float temp;          // °C
   bool  ok;
 };
 
 void mpuInit() {
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(PWR_MGMT_1);
   Wire.write(0x00);  // uyandır
   Wire.endTransmission(true);
 }
 
 int16_t readWord(uint8_t reg) {
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(reg);
   Wire.endTransmission(false);
   Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true);
   return (Wire.read() << 8) | Wire.read();
 }
 
 ImuData mpuRead() {
   ImuData d;
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(ACCEL_OUT);
   Wire.endTransmission(false);
   uint8_t n = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
   if (n < 14) { d.ok = false; return d; }
 
   auto rd = []() -> int16_t {
     return (Wire.read() << 8) | Wire.read();
   };
 
   d.ax   = rd() / 16384.0f;
   d.ay   = rd() / 16384.0f;
   d.az   = rd() / 16384.0f;
   d.temp = rd() / 340.0f + 36.53f;
   d.gx   = rd() / 131.0f;
   d.gy   = rd() / 131.0f;
   d.gz   = rd() / 131.0f;
   d.ok   = true;
   return d;
 }
 
 // ---------------------------------------------------------------------------
 // Web Sunucusu
 // ---------------------------------------------------------------------------
 WebServer server(WEB_PORT);
 
 // ---------------------------------------------------------------------------
 // Ultrasonik olcum
 // ---------------------------------------------------------------------------
 float readCm(uint8_t trig, uint8_t echo) {
   digitalWrite(trig, LOW);  delayMicroseconds(2);
   digitalWrite(trig, HIGH); delayMicroseconds(10);
   digitalWrite(trig, LOW);
   long dur = pulseIn(echo, HIGH, 30000);
   return dur == 0 ? -1 : dur / 58.0f;
 }
 
 // ---------------------------------------------------------------------------
 // HTML
 // ---------------------------------------------------------------------------
 const char INDEX_HTML[] PROGMEM = R"rawliteral(
 <!DOCTYPE html>
 <html lang="tr">
 <head>
   <meta charset="UTF-8">
   <meta name="viewport" content="width=device-width, initial-scale=1.0">
   <title>ESP32 — RC Car</title>
   <style>
     *{box-sizing:border-box;margin:0;padding:0}
     body{font-family:'Segoe UI',sans-serif;background:#0f0f0f;color:#e0e0e0;
          display:flex;justify-content:center;padding:32px 16px}
     .page{max-width:560px;width:100%}
     .badge{display:inline-block;background:#00c853;color:#000;font-size:11px;
            font-weight:700;letter-spacing:1px;padding:4px 10px;border-radius:99px;margin-bottom:16px}
     h1{font-size:24px;font-weight:600;margin-bottom:4px}
     .sub{color:#666;font-size:13px;margin-bottom:28px}
 
     h2{font-size:11px;letter-spacing:1.5px;color:#555;text-transform:uppercase;
        margin:24px 0 10px}
 
     .grid2{display:grid;grid-template-columns:1fr 1fr;gap:12px}
     .grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}
 
     .tile{background:#1a1a1a;border:1px solid #222;border-radius:12px;
           padding:16px 10px;text-align:center}
     .tile .lbl{font-size:10px;letter-spacing:1px;color:#555;text-transform:uppercase;margin-bottom:8px}
     .tile .val{font-size:28px;font-weight:700;font-family:monospace;color:#82b1ff;line-height:1}
     .tile .unit{font-size:11px;color:#444;margin-top:4px}
     .tile.warn .val{color:#ff6b6b}
     .tile.err  .val{color:#555;font-size:18px}
 
     #upd{font-size:11px;color:#333;margin-top:24px;text-align:center}
   </style>
 </head>
 <body>
 <div class="page">
   <div class="badge">ONLINE</div>
   <h1>RC Car — ESP32-S3</h1>
   <p class="sub">Sensor Dashboard / Faz 0.5</p>
 
   <h2>Ultrasonik</h2>
   <div class="grid2">
     <div class="tile" id="box-l">
       <div class="lbl">Sol</div>
       <div class="val" id="val-l">—</div>
       <div class="unit">cm</div>
     </div>
     <div class="tile" id="box-r">
       <div class="lbl">Sag</div>
       <div class="val" id="val-r">—</div>
       <div class="unit">cm</div>
     </div>
   </div>
 
   <h2>Ivmeolcer (g)</h2>
   <div class="grid3">
     <div class="tile" id="box-ax"><div class="lbl">X</div><div class="val" id="val-ax">—</div><div class="unit">g</div></div>
     <div class="tile" id="box-ay"><div class="lbl">Y</div><div class="val" id="val-ay">—</div><div class="unit">g</div></div>
     <div class="tile" id="box-az"><div class="lbl">Z</div><div class="val" id="val-az">—</div><div class="unit">g</div></div>
   </div>
 
   <h2>Jiroskop (°/s)</h2>
   <div class="grid3">
     <div class="tile" id="box-gx"><div class="lbl">X</div><div class="val" id="val-gx">—</div><div class="unit">°/s</div></div>
     <div class="tile" id="box-gy"><div class="lbl">Y</div><div class="val" id="val-gy">—</div><div class="unit">°/s</div></div>
     <div class="tile" id="box-gz"><div class="lbl">Z</div><div class="val" id="val-gz">—</div><div class="unit">°/s</div></div>
   </div>
 
   <h2>Sicaklik</h2>
   <div class="grid2">
     <div class="tile" id="box-tmp">
       <div class="lbl">MPU6050</div>
       <div class="val" id="val-tmp">—</div>
       <div class="unit">°C</div>
     </div>
   </div>
 
   <div id="upd">bekleniyor...</div>
 </div>
 
 <script>
 const WARN_US = 20;
 
 function setTile(valId, boxId, v, decimals, warnFn) {
   const ve = document.getElementById(valId);
   const be = document.getElementById(boxId);
   if (v === null || v === undefined) { ve.textContent='ERR'; be.className='tile err'; return; }
   ve.textContent = v.toFixed(decimals);
   be.className = 'tile' + (warnFn && warnFn(v) ? ' warn' : '');
 }
 
 async function poll() {
   try {
     const d = await fetch('/data').then(r=>r.json());
 
     setTile('val-l','box-l', d.left,  1, v=>v>0&&v<WARN_US);
     setTile('val-r','box-r', d.right, 1, v=>v>0&&v<WARN_US);
 
     const imu = d.imu_ok;
     setTile('val-ax','box-ax', imu ? d.ax : null, 2);
     setTile('val-ay','box-ay', imu ? d.ay : null, 2);
     setTile('val-az','box-az', imu ? d.az : null, 2);
     setTile('val-gx','box-gx', imu ? d.gx : null, 1);
     setTile('val-gy','box-gy', imu ? d.gy : null, 1);
     setTile('val-gz','box-gz', imu ? d.gz : null, 1);
     setTile('val-tmp','box-tmp', imu ? d.temp : null, 1);
 
     document.getElementById('upd').textContent =
       'Son guncelleme: ' + new Date().toLocaleTimeString('tr-TR');
   } catch(e) {
     document.getElementById('upd').textContent = 'Baglanti hatasi...';
   }
 }
 poll();
 setInterval(poll, 500);
 </script>
 </body>
 </html>
 )rawliteral";
 
 // ---------------------------------------------------------------------------
 // Handler'lar
 // ---------------------------------------------------------------------------
 void handleRoot() {
   server.send(200, "text/html", INDEX_HTML);
 }
 
 void handleData() {
   float l = readCm(TRIG_L, ECHO_L);
   float r = readCm(TRIG_R, ECHO_R);
   ImuData imu = mpuRead();
 
   char buf[256];
   if (imu.ok) {
     snprintf(buf, sizeof(buf),
       "{\"left\":%.1f,\"right\":%.1f,\"imu_ok\":true,"
       "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
       "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"temp\":%.1f}",
       l, r, imu.ax, imu.ay, imu.az,
       imu.gx, imu.gy, imu.gz, imu.temp);
   } else {
     snprintf(buf, sizeof(buf),
       "{\"left\":%.1f,\"right\":%.1f,\"imu_ok\":false}", l, r);
   }
   server.send(200, "application/json", buf);
 }
 
 void handleNotFound() {
   server.send(404, "text/plain", "Not found");
 }
 
 // ---------------------------------------------------------------------------
 // Setup
 // ---------------------------------------------------------------------------
 void setup() {
   Serial.begin(115200);
   delay(500);
 
   for (uint8_t pin : {2, 21, 35, 38, 48}) neopixelWrite(pin, 0, 0, 0);
 
   pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
   pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
 
   Wire.begin(SDA_PIN, SCL_PIN);
   mpuInit();
   Serial.println("MPU6050 baslatildi.");
 
   Serial.println("\n=== Faz 0.5 — Sensor Dashboard ===");
 
   WiFi.mode(WIFI_STA);
   WiFi.begin(SSID, PASSWORD);
   Serial.print("WiFi baglaniyor");
 
   unsigned long t0 = millis();
   while (WiFi.status() != WL_CONNECTED) {
     if (millis() - t0 > 15000) {
       Serial.println("\nERROR: WiFi timeout, yeniden baslatiliyor...");
       delay(1000);
       ESP.restart();
     }
     delay(500);
     Serial.print(".");
   }
 
   Serial.printf("\nWiFi BAGLANDI  IP: %s\n", WiFi.localIP().toString().c_str());
 
   if (MDNS.begin("esp")) MDNS.addService("http", "tcp", WEB_PORT);
 
   server.on("/",     handleRoot);
   server.on("/data", handleData);
   server.onNotFound(handleNotFound);
   server.begin();
 
   Serial.println("Sunucu baslatildi -> http://esp.local:5000");
   Serial.println("===================================");
 }
 
 // ---------------------------------------------------------------------------
 // Loop
 // ---------------------------------------------------------------------------
 void loop() {
   server.handleClient();
 }
 