/*
 * ============================================================================
 *  Phase 0.5 — Sensor Dashboard  (ESP32-S3)
 *  Autonomous RC Car Racing Platform
 * ============================================================================
 *
 *  USAGE:
 *    1) Upload sketch. Connect computer to Hezer WiFi.
 *    2) Browser: http://esp.local:5000  or  http://<IP>:5000
 *
 *  PINS:
 *    Left  ultrasonic TRIG/ECHO : 10 / 11
 *    Right ultrasonic TRIG/ECHO : 12 / 13
 *    MPU6050 SDA/SCL            :  6 /  7
 *    TF-Luna TX->RX / RX->TX    : 16 / 17  (UART1)
 *    ARM button (NO, to GND)    : 14
 *    E-STOP button (NO, to GND) : 18
 *    ARM LED (220Ω series)      : 21
 *
 *  Board: ESP32S3 Dev Module  |  CDCOnBoot: Enabled
 * ============================================================================
 */

 #include <WiFi.h>
 #include <WebServer.h>
 #include <ESPmDNS.h>
 #include <Wire.h>
 #include <ESP32Servo.h>
 
 // Placed at top to avoid Arduino preprocessor prototype ordering issue
 typedef struct {
   float ax, ay, az, gx, gy, gz, temp;
   bool ok;
 } ImuData;
 
 // ---------------------------------------------------------------------------
 // WiFi
 // ---------------------------------------------------------------------------
 const char* SSID     = "Hezer";
 const char* PASSWORD = "burakhezer";

 // ---------------------------------------------------------------------------
 // Pins
 // ---------------------------------------------------------------------------
 const int SERVO_PIN  = 4;
 const int ESC_PIN    = 5;
 const int TRIG_R     = 10;
 const int ECHO_R     = 11;
 const int TRIG_L     = 12;
 const int ECHO_L     = 13;
 const int SDA_PIN    = 6;
 const int SCL_PIN    = 7;
 const int LUNA_RX    = 16;
 const int LUNA_TX    = 17;
 const int BTN_ARM    = 14;   // Momentary NO button, to GND — INPUT_PULLUP
 const int BTN_ESTOP  = 18;   // Momentary NO button, to GND — INPUT_PULLUP
 const int LED_ARMED  = 21;   // ARM status LED (220Ω series)

 // PWM limits (µs) — overridable via build flag:
 //   arduino-cli compile ... --build-property \
 //     "build.extra_flags=-DSERVO_MIN_US=900 -DSERVO_MID_US=1480 -DSERVO_MAX_US=2100"

 #ifndef SERVO_MIN_US
 #define SERVO_MIN_US  1730
 #endif
 #ifndef SERVO_MID_US
 #define SERVO_MID_US  2030
 #endif
 #ifndef SERVO_MAX_US
 #define SERVO_MAX_US  2330
 #endif
 #ifndef ESC_MIN_US
 #define ESC_MIN_US    1370
 #endif
 #ifndef ESC_MID_US
 #define ESC_MID_US    1540
 #endif
 #ifndef ESC_MAX_US
 #define ESC_MAX_US    1570
 #endif
 
 // Throttle limits (percent, 0-100) — overridable via build flag:
 //   --build-property "build.extra_flags=-DTHROTTLE_LIMIT_PCT=40 -DAUTO_THROTTLE_PCT=25"
 #ifndef THROTTLE_LIMIT_PCT
 #define THROTTLE_LIMIT_PCT 30
 #endif
 #ifndef AUTO_THROTTLE_PCT
 #define AUTO_THROTTLE_PCT  20
 #endif
 
 // ---------------------------------------------------------------------------
 // Servo / ESC objects and current µs values
 // ---------------------------------------------------------------------------
 Servo steerServo;
 Servo esc;
 int   servoUs = SERVO_MID_US;
 int   escUs   = ESC_MID_US;

 // Runtime-adjustable limits (default from compile-time defines)
 int throttleLimitPct = THROTTLE_LIMIT_PCT;
 int steerLimitPct    = 100;  // 100% = full servo range, lower = narrower steering
 
 // ---------------------------------------------------------------------------
 // Arm / E-Stop state machine
 // ---------------------------------------------------------------------------
 enum ArmState { DISARMED, ARMED, EMERGENCY };
 ArmState armState      = DISARMED;
 String   disarmReason  = "none";

 // Button debounce
 unsigned long btnArmLast   = 0;
 unsigned long btnEstopLast = 0;
 #define DEBOUNCE_MS 50

 // E-STOP via hardware interrupt — fires instantly regardless of loop() blocking
 volatile bool estopISRFlag = false;
 void IRAM_ATTR onEstopISR() { estopISRFlag = true; }
 
 void doDisarm(const String& reason) {
   armState     = DISARMED;
   disarmReason = reason;
   escUs        = ESC_MID_US;
   servoUs      = SERVO_MID_US;
   esc.writeMicroseconds(ESC_MID_US);
   steerServo.writeMicroseconds(SERVO_MID_US);
   digitalWrite(LED_ARMED, LOW);
   Serial.println("[ARM] DISARMED: " + reason);
 }
 
 void doArm() {
   if (armState == EMERGENCY) return;  // must disarm before re-arming after e-stop
   armState     = ARMED;
   disarmReason = "none";
   digitalWrite(LED_ARMED, HIGH);
   Serial.println("[ARM] ARMED");
 }
 
 void doEstop() {
   armState     = EMERGENCY;
   disarmReason = "emergency_stop";
   escUs        = ESC_MID_US;
   servoUs      = SERVO_MID_US;
   esc.writeMicroseconds(ESC_MID_US);
   steerServo.writeMicroseconds(SERVO_MID_US);
   digitalWrite(LED_ARMED, LOW);
   Serial.println("[ARM] EMERGENCY STOP");
 }
 
 void checkButtons() {
   unsigned long now = millis();

   // E-STOP interrupt flag — handled first, regardless of what loop() was doing
   if (estopISRFlag) {
     estopISRFlag = false;
     doEstop();
     return;
   }

   // ARM button (toggle: DISARMED->ARMED, ARMED->DISARMED)
   if (digitalRead(BTN_ARM) == LOW && now - btnArmLast > DEBOUNCE_MS) {
     btnArmLast = now;
     if      (armState == DISARMED) doArm();
     else if (armState == ARMED)    doDisarm("btn_manual");
   }
 }
 
 // ---------------------------------------------------------------------------
 // Constants
 // ---------------------------------------------------------------------------
 #define SONAR_ANGLE_DEG 30.0f
 const float SONAR_COS = 0.8660f;   // cos(30°)
 const float SONAR_SIN = 0.5000f;   // sin(30°)

 // ---------------------------------------------------------------------------
 // Global sensor state (last read values)
 // ---------------------------------------------------------------------------
 ImuData lastImu = {0, 0, 0, 0, 0, 0, 0, false};
 float   lastLeft  = -1;
 float   lastRight = -1;

 // ---------------------------------------------------------------------------
 // IMU calibration offsets
 // ---------------------------------------------------------------------------
 float imuOffAx = 0, imuOffAy = 0, imuOffAz = 0;
 float imuOffGx = 0, imuOffGy = 0, imuOffGz = 0;
 bool  imuCalibrated = false;
 
 // ---------------------------------------------------------------------------
 // Complementary filter — roll / pitch / yaw
 // ---------------------------------------------------------------------------
 float cfRoll  = 0, cfPitch = 0, cfYaw = 0;
 unsigned long cfLastUs = 0;
 #define CF_ALPHA 0.96f   // gyro weight (0.96 = trust gyro more)
 
 // ---------------------------------------------------------------------------
 // TF-Luna
 // ---------------------------------------------------------------------------
 int   lunaDistCm = -1;
 int   lunaStr    = 0;
 float lunaTemp   = 0;
 
 void lunaUpdate() {
   while (Serial1.available() >= 9) {
     if (Serial1.read() != 0x59) continue;
     if (Serial1.peek() != 0x59) continue;
     Serial1.read();
     uint8_t buf[7];
     if (Serial1.readBytes(buf, 7) < 7) break;
     uint8_t chk = 0x59 + 0x59;
     for (int i = 0; i < 6; i++) chk += buf[i];
     if ((chk & 0xFF) != buf[6]) continue;
     lunaStr    = buf[2] | (buf[3] << 8);
     lunaTemp   = (buf[4] | (buf[5] << 8)) / 8.0f - 256.0f;
     lunaDistCm = (lunaStr < 100) ? -1 : (buf[0] | (buf[1] << 8));
   }
 }
 
 // ---------------------------------------------------------------------------
 // MPU6050
 // ---------------------------------------------------------------------------
 #define MPU_ADDR 0x68
 
 void mpuInit() {
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x6B); Wire.write(0x00);
   Wire.endTransmission(true);
 }
 
 // Collects n samples, computes mean, fills calibration offsets
 void mpuCalibrate(int n = 300) {
   double sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0;
   for (int i = 0; i < n; i++) {
     Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
     Wire.endTransmission(false);
     Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
     auto rd = []() -> int16_t { return (Wire.read() << 8) | Wire.read(); };
     sax += rd()/16384.0f;
     say += rd()/16384.0f;
     saz += rd()/16384.0f;
     rd();  // skip temp register
     sgx += rd()/131.0f;
     sgy += rd()/131.0f;
     sgz += rd()/131.0f;
     delay(4);
   }
   imuOffAx = sax / n;
   imuOffAy = say / n;
   // NED: Z+ down, az = +1g expected on flat ground.
   // If sensor is mounted Z+ up (az = -1g), auto-detected and offset to +1g ref.
   float avgAz = saz / n;
   imuOffAz = avgAz - (avgAz > 0 ? +1.0f : -1.0f);
   imuOffGx = sgx / n;
   imuOffGy = sgy / n;
   imuOffGz = sgz / n;
   imuCalibrated = true;
   Serial.printf("[IMU] Calibration done | offAx=%.4f offAy=%.4f offAz=%.4f\n",
                 imuOffAx, imuOffAy, imuOffAz);
   Serial.printf("                      | offGx=%.4f offGy=%.4f offGz=%.4f\n",
                 imuOffGx, imuOffGy, imuOffGz);
 }
 
 ImuData mpuRead() {
   ImuData d;
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x3B);
   Wire.endTransmission(false);
   if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true) < 14) {
     d.ok = false; return d;
   }
   auto rd = []() -> int16_t { return (Wire.read() << 8) | Wire.read(); };
   d.ax = rd()/16384.0f - imuOffAx;
   d.ay = rd()/16384.0f - imuOffAy;
   d.az = rd()/16384.0f - imuOffAz;
   d.temp = rd()/340.0f + 36.53f;
   d.gx = rd()/131.0f - imuOffGx;
   d.gy = rd()/131.0f - imuOffGy;
   d.gz = rd()/131.0f - imuOffGz;
   d.ok = true;
   return d;
 }
 
 // Complementary filter — NED coordinate system
 //   X+ = forward  |  Y+ = right  |  Z+ = down (gravity direction)
 //
 // Expected on flat ground (correct mounting):
 //   ax~0, ay~0, az~+1g  |  roll=0, pitch=0
 //
 // Note: currently Z+ up on desk (az=-1g). Formulas work correctly
 //       once sensor is mounted Z+ down in the car.
 void cfUpdate(const ImuData& d) {
   if (!d.ok) return;
 
   unsigned long now = micros();
   float dt = cfLastUs == 0 ? 0.01f : (now - cfLastUs) * 1e-6f;
   cfLastUs = now;
   if (dt > 0.5f) dt = 0.01f;
 
   // NED atan2 (Z+ down, az = +1g on flat ground)
   // roll : right side down = positive
   float aRoll  = atan2f(d.ay, d.az) * 57.2958f;
   // pitch: nose up = positive
   float aPitch = atan2f(-d.ax, sqrtf(d.ay*d.ay + d.az*d.az)) * 57.2958f;

   // Gyro sign convention for NED:
   //   gx: right-hand rule around X+ → right side down = positive roll rate ✓
   //   gy: right-hand rule around Y+ → nose down = positive; we want nose up
   //       positive → negate: -gy
   //   gz: right-hand rule around Z+ down → clockwise (right turn) = positive ✓
   cfRoll  = CF_ALPHA * (cfRoll  + d.gx  * dt) + (1.0f - CF_ALPHA) * aRoll;
   cfPitch = CF_ALPHA * (cfPitch - d.gy  * dt) + (1.0f - CF_ALPHA) * aPitch;
   cfYaw  += d.gz * dt;
 }
 
 // ---------------------------------------------------------------------------
 // Ultrasonic
 // ---------------------------------------------------------------------------
 float readCm(uint8_t trig, uint8_t echo) {
   digitalWrite(trig, LOW);  delayMicroseconds(2);
   digitalWrite(trig, HIGH); delayMicroseconds(10);
   digitalWrite(trig, LOW);
   long dur = pulseIn(echo, HIGH, 6000);  // 6ms = ~1m range
   return dur == 0 ? -1 : dur / 58.0f;
 }
 
 // ---------------------------------------------------------------------------
 // Web server
 // ---------------------------------------------------------------------------
 WebServer server(5000);
 
 // ---------------------------------------------------------------------------
 // HTML
 // ---------------------------------------------------------------------------
 const char INDEX_HTML[] PROGMEM = R"rawliteral(
 <!DOCTYPE html>
 <html lang="en">
 <head>
   <meta charset="UTF-8">
   <meta name="viewport" content="width=device-width,initial-scale=1">
   <title>ESP32 — RC Car</title>
   <style>
     *{box-sizing:border-box;margin:0;padding:0}
     body{font-family:'Segoe UI',sans-serif;background:#0a0a0a;color:#ddd;
          display:flex;justify-content:center;padding:24px 12px}
     .page{max-width:620px;width:100%}
     .badge{display:inline-block;background:#00c853;color:#000;font-size:10px;
            font-weight:700;letter-spacing:1px;padding:3px 9px;border-radius:99px;margin-bottom:12px}
     h1{font-size:22px;font-weight:600;margin-bottom:3px}
     .sub{color:#555;font-size:12px;margin-bottom:22px}
     h2{font-size:10px;letter-spacing:1.5px;color:#444;text-transform:uppercase;margin:20px 0 8px}
 
     .grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}
     .grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
     .tile{background:#141414;border:1px solid #1e1e1e;border-radius:10px;
           padding:14px 8px;text-align:center}
     .tile .lbl{font-size:9px;letter-spacing:1px;color:#444;text-transform:uppercase;margin-bottom:6px}
     .tile .val{font-size:26px;font-weight:700;font-family:monospace;color:#82b1ff;line-height:1}
     .tile .unit{font-size:10px;color:#333;margin-top:3px}
     .tile.warn .val{color:#ff6b6b}
     .tile.err  .val{color:#333;font-size:16px}
 
     #chart-wrap{background:#0d1117;border:1px solid #1e1e1e;border-radius:12px;
                 padding:12px;margin-top:8px}
     #chart{display:block;width:100%;height:160px}
     #chart-info{font-size:9px;color:#2a2a2a;margin-top:6px;text-align:right}
 
     #upd{font-size:10px;color:#2a2a2a;margin-top:18px;text-align:center}
 
   /* --- Control Panel --- */
   .ctrl{background:#141414;border:1px solid #1e1e1e;border-radius:10px;padding:16px;margin-top:8px}
   .btn-row{display:flex;gap:8px;margin-bottom:12px}
   .btn{flex:1;padding:10px 0;border:none;border-radius:8px;font-size:13px;
        font-weight:700;letter-spacing:.5px;cursor:pointer;transition:opacity .1s}
   .btn:active{opacity:.7}
   .btn-arm  {background:#00c853;color:#000}
   .btn-disarm{background:#555;color:#fff}
   .btn-estop{background:#f44336;color:#fff;flex:2}
   .btn:disabled{opacity:.3;cursor:not-allowed}
   .slider-lbl{font-size:9px;letter-spacing:1px;color:#444;text-transform:uppercase;margin-bottom:4px}
   .slider-row{margin-bottom:10px}
   input[type=range]{width:100%;accent-color:#82b1ff}
   .slider-val{font-size:11px;color:#82b1ff;font-family:monospace;text-align:right}
   #state-badge{display:inline-block;padding:3px 10px;border-radius:99px;font-size:10px;
                font-weight:700;letter-spacing:1px;margin-bottom:10px}
   .state-disarmed{background:#333;color:#888}
   .state-armed   {background:#00c853;color:#000}
   .state-emergency{background:#f44336;color:#fff}
   </style>
 </head>
 <body>
 <div class="page">
   <div class="badge">ONLINE</div>
   <h1>RC Car — ESP32-S3</h1>
   <p class="sub">Sensor Dashboard / Phase 0.5</p>

   <h2>TF-Luna — Distance History</h2>
   <div id="chart-wrap">
     <canvas id="chart"></canvas>
     <div id="chart-info">—</div>
   </div>
 
   <h2>LiDAR Live</h2>
   <div class="grid3">
     <div class="tile" id="box-luna"><div class="lbl">Distance</div><div class="val" id="val-luna">—</div><div class="unit">cm</div></div>
     <div class="tile" id="box-lstr"><div class="lbl">Signal</div><div class="val" id="val-lstr">—</div><div class="unit">/65535</div></div>
     <div class="tile" id="box-ltmp"><div class="lbl">Chip</div><div class="val" id="val-ltmp">—</div><div class="unit">°C</div></div>
   </div>

   <h2>Ultrasonic</h2>
   <div class="grid2">
     <div class="tile" id="box-l"><div class="lbl">Left</div><div class="val" id="val-l">—</div><div class="unit">cm</div></div>
     <div class="tile" id="box-r"><div class="lbl">Right</div><div class="val" id="val-r">—</div><div class="unit">cm</div></div>
   </div>
 
   <h2>IMU</h2>
   <div class="grid3">
     <div class="tile"><div class="lbl">Accel X</div><div class="val" id="val-ax">—</div><div class="unit">g</div></div>
     <div class="tile"><div class="lbl">Accel Y</div><div class="val" id="val-ay">—</div><div class="unit">g</div></div>
     <div class="tile"><div class="lbl">Accel Z</div><div class="val" id="val-az">—</div><div class="unit">g</div></div>
   </div>
 
   <h2>Control</h2>
   <div class="ctrl">
     <div id="state-badge" class="state-disarmed">DISARMED</div>
     <div class="btn-row">
       <button class="btn btn-arm"    id="btn-arm"    onclick="sendArm()">ARM</button>
       <button class="btn btn-disarm" id="btn-disarm" onclick="sendDisarm()">DISARM</button>
       <button class="btn btn-estop"  id="btn-estop"  onclick="sendEstop()">E-STOP</button>
     </div>
     <div class="slider-row">
       <div class="slider-lbl">Steering (µs)</div>
       <input type="range" id="sl-steer" min="1000" max="2000" value="1500" step="10"
              oninput="steerVal(this.value); scheduleControl()">
       <div class="slider-val" id="steer-val">1500 µs</div>
     </div>
     <div class="slider-row">
       <div class="slider-lbl" style="display:flex;justify-content:space-between;align-items:center">
         <span>Throttle (µs)</span>
         <span style="display:flex;align-items:center;gap:4px">
           Limit:
           <input type="number" id="thr-limit-input" min="0" max="100" value="30" step="1"
             style="width:48px;background:#1e1e1e;border:1px solid #333;color:#82b1ff;
                    font-size:11px;font-family:monospace;text-align:center;border-radius:4px;padding:2px"
             onchange="setThrLimit(this.value)">%
         </span>
       </div>
       <input type="range" id="sl-thr" min="1000" max="2000" value="1500" step="10"
              oninput="thrVal(this.value); scheduleControl()">
       <div class="slider-val" id="thr-val">1500 µs</div>
     </div>
   </div>

   <div id="upd">waiting...</div>
 </div>
 
 <script>
 const MAX_PTS = 60;
 const MAX_CM  = 500;
 const history = [];
 
 const canvas = document.getElementById('chart');
 const ctx    = canvas.getContext('2d');
 
 function drawChart() {
   const W = canvas.offsetWidth, H = canvas.offsetHeight;
   canvas.width = W; canvas.height = H;
   ctx.clearRect(0,0,W,H);
 
   // izgara cizgileri
   [100,200,300,400].forEach(v=>{
     const y = H - (v/MAX_CM)*H;
     ctx.strokeStyle='#1a1a1a'; ctx.lineWidth=1;
     ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke();
     ctx.fillStyle='#2a2a2a'; ctx.font='9px monospace';
     ctx.fillText(v+'cm',4,y-2);
   });
 
   if(history.length<2) return;
 
   // cizgi
   ctx.beginPath();
   history.forEach((v,i)=>{
     const x = (i/(MAX_PTS-1))*W;
     const y = v<0 ? H : H-(Math.min(v,MAX_CM)/MAX_CM)*H;
     i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
   });
   ctx.strokeStyle='#00e676'; ctx.lineWidth=2; ctx.stroke();
 
   // son nokta vurgusu
   const last = history[history.length-1];
   if(last>=0){
     const x=(history.length-1)/(MAX_PTS-1)*W;
     const y=H-(Math.min(last,MAX_CM)/MAX_CM)*H;
     ctx.beginPath(); ctx.arc(x,y,4,0,2*Math.PI);
     ctx.fillStyle= last<30?'#ff6b6b':'#00e676'; ctx.fill();
   }
 }
 
 function setTile(vi,bi,v,dec,warnFn){
   const ve=document.getElementById(vi), be=document.getElementById(bi);
   if(!be||!ve) return;
   if(v===null||v===undefined||v<0){ve.textContent='ERR';be.className='tile err';return;}
   ve.textContent=typeof dec==='number'?v.toFixed(dec):v;
   be.className='tile'+(warnFn&&warnFn(v)?' warn':'');
 }
 
 async function poll(){
   try{
     const d=await fetch('/data').then(r=>r.json());
 
     // Grafige ekle
     history.push(d.luna);
     if(history.length>MAX_PTS) history.shift();
     drawChart();
     document.getElementById('chart-info').textContent=
       'Last: '+(d.luna>=0?d.luna+'cm':'invalid')+
       ' | Signal: '+d.lstr;
 
     setTile('val-luna','box-luna', d.luna, 0, v=>v<25);
     setTile('val-lstr','box-lstr', d.lstr>0?d.lstr:null, 0, v=>v<200);
     setTile('val-ltmp','box-ltmp', d.ltmp, 1);
     setTile('val-l','box-l', d.left,  1, v=>v<20);
     setTile('val-r','box-r', d.right, 1, v=>v<20);
 
     if(d.imu_ok){
       document.getElementById('val-ax').textContent=d.ax.toFixed(2);
       document.getElementById('val-ay').textContent=d.ay.toFixed(2);
       document.getElementById('val-az').textContent=d.az.toFixed(2);
     }
     document.getElementById('upd').textContent=
       'Last update: '+new Date().toLocaleTimeString('en-US');
   } catch(e){
     document.getElementById('upd').textContent='Connection error...';
   }
 }
 
 // --- Arm state UI ---
 let currentState = 'DISARMED';
 function updateStateBadge(state){
   currentState = state;
   const b = document.getElementById('state-badge');
   b.textContent = state;
   b.className = 'state-' + state.toLowerCase();
   const armed = state === 'ARMED';
   document.getElementById('sl-steer').disabled = !armed;
   document.getElementById('sl-thr').disabled   = !armed;
 }
 
 function steerVal(v){ document.getElementById('steer-val').textContent = v + ' µs'; }
 function thrVal(v)  { document.getElementById('thr-val').textContent   = v + ' µs'; }
 
 async function setThrLimit(v){
   v = Math.min(100, Math.max(0, parseInt(v)||0));
   document.getElementById('thr-limit-input').value = v;
   try{
     const r = await fetch('/set?thr_limit='+v,{method:'POST'});
     const d = await r.json();
     if(d.ok){
       // update slider max
       const eMid = parseInt(document.getElementById('sl-thr').value)||1470;
       const curMid = 1470; // ESC_MID_US
       const curMax = 1800; // ESC_MAX_US
       const maxUs = curMid + Math.round((curMax - curMid) * v / 100);
       document.getElementById('sl-thr').max = maxUs;
     }
   } catch(e){}
 }

 // 80ms throttle on slider oninput — avoids flooding ESP32
 let _ctrlTimer = null;
 function scheduleControl(){
   if(_ctrlTimer) return;
   _ctrlTimer = setTimeout(()=>{ _ctrlTimer=null; sendControl(); }, 80);
 }
 
 async function sendArm(){
   try{ const r=await fetch('/arm',{method:'POST'}); const d=await r.json();
     if(d.ok) updateStateBadge('ARMED');
   } catch(e){ alert('ARM error'); }
 }
 async function sendDisarm(){
   try{ const r=await fetch('/disarm',{method:'POST'}); const d=await r.json();
     if(d.ok){ updateStateBadge('DISARMED');
       document.getElementById('sl-steer').value=1500; steerVal(1500);
       document.getElementById('sl-thr').value=1500;   thrVal(1500); }
   } catch(e){ alert('DISARM error'); }
 }
 async function sendEstop(){
   try{ const r=await fetch('/estop',{method:'POST'}); const d=await r.json();
     if(d.ok){ updateStateBadge('EMERGENCY');
       document.getElementById('sl-steer').value=1500; steerVal(1500);
       document.getElementById('sl-thr').value=1500;   thrVal(1500); }
   } catch(e){ alert('E-STOP error'); }
 }
 async function sendControl(){
   if(currentState !== 'ARMED') return;
   const steer = document.getElementById('sl-steer').value;
   const thr   = document.getElementById('sl-thr').value;
   try{ await fetch('/control?steer='+steer+'&thr='+thr,{method:'POST'}); }
   catch(e){}
 }
 
 // Sayfa yuklenince slider'lari disable et + limit bilgisini cek
 updateStateBadge('DISARMED');
 fetch('/status').then(r=>r.json()).then(d=>{
   const cfg=d.config, tel=d.telemetry;
   if(cfg){
     const sMin=cfg.servo?.minUs, sMid=cfg.servo?.neutralUs, sMax=cfg.servo?.maxUs;
     if(sMin!=null) document.getElementById('sl-steer').min=sMin;
     if(sMax!=null) document.getElementById('sl-steer').max=sMax;
     if(sMid!=null){ document.getElementById('sl-steer').value=sMid; steerVal(sMid); }
     const eMin=cfg.esc?.minUs, eMid=cfg.esc?.neutralUs, eMax=cfg.esc?.maxUs??2000;
     if(eMin!=null) document.getElementById('sl-thr').min=eMin;
     if(eMid!=null){ document.getElementById('sl-thr').value=eMid; thrVal(eMid); }
     const lim=tel?.limits?.throttleLimit;
     if(lim!=null){
       document.getElementById('thr-limit-input').value=lim;
       const maxUs=(eMid??1470)+Math.round((eMax-(eMid??1470))*lim/100);
       document.getElementById('sl-thr').max=maxUs;
     }
   }
 }).catch(()=>{});
 
 poll();
 setInterval(poll, 500);
 window.addEventListener('resize', drawChart);
 </script>
 </body>
 </html>
 )rawliteral";
 
// ---------------------------------------------------------------------------
// Swagger UI HTML (CDN'den yuklenir — internet gerekir)
// ---------------------------------------------------------------------------
const char SWAGGER_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>RC Car API Docs</title>
  <link rel="stylesheet" href="https://unpkg.com/swagger-ui-dist@5/swagger-ui.css">
  <style>body{margin:0}.swagger-ui .topbar{background:#0a0a0a}</style>
</head>
<body>
<div id="swagger-ui"></div>
<script src="https://unpkg.com/swagger-ui-dist@5/swagger-ui-bundle.js"></script>
<script>
window.onload=()=>SwaggerUIBundle({
  url:"/api/openapi.json",
  dom_id:"#swagger-ui",
  presets:[SwaggerUIBundle.presets.apis,SwaggerUIBundle.SwaggerUIStandalonePreset],
  layout:"BaseLayout",
  tryItOutEnabled:true
});
</script>
</body>
</html>
)rawliteral";

// ---------------------------------------------------------------------------
// OpenAPI 3.0 spec
// ---------------------------------------------------------------------------
const char OPENAPI_JSON[] PROGMEM = R"rawliteral(
{
  "openapi":"3.0.0",
  "info":{
    "title":"RC Car ESP32-S3 API",
    "version":"0.5.0",
    "description":"Phase 0.5 — Sensor Dashboard & Manual Control. /control requires ARMED state."
  },
  "servers":[{"url":"http://esp.local:5000","description":"ESP32-S3 (mDNS)"}],
  "tags":[
    {"name":"Telemetry","description":"Sensor read endpoints"},
    {"name":"Arm","description":"Motor arm / disarm / emergency stop"},
    {"name":"Control","description":"Servo and ESC control"}
  ],
  "paths":{
    "/status":{
      "get":{
        "tags":["Telemetry"],
        "summary":"Full system state",
        "description":"Sonar, lidar, imu, control, system and limits in status.json format.",
        "responses":{"200":{"description":"Telemetry JSON","content":{"application/json":{"schema":{"$ref":"#/components/schemas/Status"}}}}}
      }
    },
    "/data":{
      "get":{
        "tags":["Telemetry"],
        "summary":"Lightweight polling data",
        "description":"Compact JSON optimised for 500ms polling.",
        "responses":{"200":{"description":"Sensor JSON","content":{"application/json":{"schema":{"$ref":"#/components/schemas/Data"}}}}}
      }
    },
    "/arm":{
      "post":{
        "tags":["Arm"],
        "summary":"Arm the motors",
        "description":"DISARMED -> ARMED. Cannot arm from EMERGENCY state — disarm first.",
        "responses":{
          "200":{"description":"ARM successful","content":{"application/json":{"example":{"ok":true,"state":"ARMED"}}}},
          "403":{"description":"E-STOP active","content":{"application/json":{"example":{"ok":false,"msg":"E-STOP active, disarm first"}}}}
        }
      }
    },
    "/disarm":{
      "post":{
        "tags":["Arm"],
        "summary":"Disarm the motors",
        "description":"Transitions to DISARMED from any state. Servo and ESC are set to neutral.",
        "responses":{"200":{"description":"DISARM successful","content":{"application/json":{"example":{"ok":true,"state":"DISARMED"}}}}}
      }
    },
    "/estop":{
      "post":{
        "tags":["Arm"],
        "summary":"Emergency stop",
        "description":"Transitions to EMERGENCY, motor stops immediately. Requires /disarm to exit.",
        "responses":{"200":{"description":"E-STOP active","content":{"application/json":{"example":{"ok":true,"state":"EMERGENCY"}}}}}
      }
    },
    "/control":{
      "post":{
        "tags":["Control"],
        "summary":"Set steering and throttle",
        "description":"Only works in ARMED state. Throttle is clamped server-side by THROTTLE_LIMIT_PCT (default 30% = max 1650 us).",
        "parameters":[
          {
            "name":"steer","in":"query","required":false,
            "description":"Servo µs: 1000=full left | 1500=centre | 2000=full right",
            "schema":{"type":"integer","minimum":1000,"maximum":2000,"example":1500}
          },
          {
            "name":"thr","in":"query","required":false,
            "description":"ESC µs: 1500=neutral | max=(neutral + range * PCT/100)",
            "schema":{"type":"integer","minimum":1000,"maximum":2000,"example":1500}
          }
        ],
        "responses":{
          "200":{"description":"Control applied","content":{"application/json":{"example":{"ok":true,"servoUs":1400,"escUs":1560}}}},
          "403":{"description":"Not armed","content":{"application/json":{"example":{"ok":false,"msg":"NOT ARMED"}}}}
        }
      }
    }
  },
  "components":{
    "schemas":{
      "Data":{
        "type":"object",
        "properties":{
          "left":{"type":"number","description":"Left sonar distance (cm)"},
          "right":{"type":"number","description":"Right sonar distance (cm)"},
          "luna":{"type":"integer","description":"TF-Luna distance (cm, -1=invalid)"},
          "lstr":{"type":"integer","description":"TF-Luna signal strength"},
          "ltmp":{"type":"number","description":"TF-Luna chip temperature (C)"},
          "imu_ok":{"type":"boolean"},
          "ax":{"type":"number","description":"Acceleration X (g)"},
          "ay":{"type":"number","description":"Acceleration Y (g)"},
          "az":{"type":"number","description":"Acceleration Z (g)"}
        }
      },
      "Status":{
        "type":"object",
        "properties":{
          "config":{"type":"object","description":"Servo ve ESC PWM limitleri"},
          "telemetry":{"type":"object","description":"Sonar, lidar, imu, kontrol, sistem, wireless, limits"}
        }
      }
    }
  }
}
)rawliteral";

 // ---------------------------------------------------------------------------
 // Handlers
 // ---------------------------------------------------------------------------

 // Allow cross-origin POST from external clients (laptop app, curl, Python)
 void addCORS() {
   server.sendHeader("Access-Control-Allow-Origin",  "*");
   server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
   server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
 }
 
 // OPTIONS preflight for browser CORS
 void handleOptions() { addCORS(); server.send(204); }
 
 void handleRoot()    { server.send(200, "text/html",       INDEX_HTML);   }
void handleApiDocs() { server.send(200, "text/html",       SWAGGER_HTML); }
void handleOpenApi() { addCORS(); server.send(200, "application/json", OPENAPI_JSON); }
 
 void handleData() {
   lastLeft  = readCm(TRIG_L, ECHO_L);
   lastRight = readCm(TRIG_R, ECHO_R);
   lastImu   = mpuRead();
   cfUpdate(lastImu);
   float l = lastLeft, r = lastRight;
   ImuData imu = lastImu;
   char buf[256];
   if (imu.ok) {
     snprintf(buf, sizeof(buf),
       "{\"left\":%.1f,\"right\":%.1f,"
       "\"luna\":%d,\"lstr\":%d,\"ltmp\":%.1f,"
       "\"imu_ok\":true,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
       "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"temp\":%.1f}",
       l, r, lunaDistCm, lunaStr, lunaTemp,
       imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.temp);
   } else {
     snprintf(buf, sizeof(buf),
       "{\"left\":%.1f,\"right\":%.1f,"
       "\"luna\":%d,\"lstr\":%d,\"ltmp\":%.1f,\"imu_ok\":false}",
       l, r, lunaDistCm, lunaStr, lunaTemp);
   }
   addCORS();
   server.send(200, "application/json", buf);
 }
 
 // /status — full system state in status.json format
 void handleStatus() {
   // Sonar geometry
   float lFwd = lastLeft  > 0 ? lastLeft  * SONAR_COS : -1;
   float lLat = lastLeft  > 0 ? lastLeft  * SONAR_SIN : -1;
   float rFwd = lastRight > 0 ? lastRight * SONAR_COS : -1;
   float rLat = lastRight > 0 ? lastRight * SONAR_SIN : -1;
 
   auto proxLv = [](float cm) -> int {
     if (cm < 0)   return 0;
     if (cm < 20)  return 4;
     if (cm < 50)  return 3;
     if (cm < 100) return 2;
     if (cm < 200) return 1;
     return 0;
   };
 
   bool lidarOk = lunaDistCm > 0;
 
   String j = "{";
   j += "\"config\":{"
        "\"servo\":{\"minUs\":" + String(SERVO_MIN_US) +
        ",\"neutralUs\":" + String(SERVO_MID_US) +
        ",\"maxUs\":"     + String(SERVO_MAX_US) + "},"
        "\"esc\":{\"minUs\":" + String(ESC_MIN_US) +
        ",\"neutralUs\":" + String(ESC_MID_US) +
        ",\"maxUs\":"     + String(ESC_MAX_US) + "}},";
   j += "\"telemetry\":{";
 
   // sonar
   char buf[512];
   snprintf(buf, sizeof(buf),
     "\"sonar\":{"
       "\"lCm\":%.1f,\"rCm\":%.1f,"
       "\"lLv\":%d,\"rLv\":%d,"
       "\"lFwd\":%.1f,\"lLat\":%.1f,"
       "\"rFwd\":%.1f,\"rLat\":%.1f,"
       "\"angle\":%.1f"
     "},",
     lastLeft, lastRight,
     proxLv(lastLeft), proxLv(lastRight),
     lFwd, lLat, rFwd, rLat,
     SONAR_ANGLE_DEG);
   j += buf;
 
   // lidar
   snprintf(buf, sizeof(buf),
     "\"lidar\":{\"ok\":%s,\"cm\":%.1f},",
     lidarOk ? "true" : "false",
     (float)lunaDistCm);
   j += buf;
 
   // imu
   if (lastImu.ok) {
     snprintf(buf, sizeof(buf),
       "\"imu\":{"
         "\"ok\":true,\"calibrated\":%s,"
         "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
         "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
         "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
         "\"temp\":%.1f"
       "},",
       imuCalibrated ? "true" : "false",
       lastImu.ax, lastImu.ay, lastImu.az,
       lastImu.gx, lastImu.gy, lastImu.gz,
       cfRoll, cfPitch, cfYaw,
       lastImu.temp);
   } else {
     snprintf(buf, sizeof(buf),
       "\"imu\":{\"ok\":false,\"calibrated\":false,"
       "\"ax\":null,\"ay\":null,\"az\":null,"
       "\"gx\":null,\"gy\":null,\"gz\":null,"
       "\"roll\":null,\"pitch\":null,\"yaw\":null,\"temp\":null},");
   }
   j += buf;
 
   // odometry, control, battery — not yet implemented
   j += "\"odometry\":{\"x\":null,\"y\":null},";
   {
     int steerPct = map(servoUs, SERVO_MIN_US, SERVO_MAX_US, -100, 100);
     int thrPct   = map(escUs,   ESC_MIN_US,   ESC_MAX_US,     0, 100);
     char cbuf[128];
     snprintf(cbuf, sizeof(cbuf),
       "\"control\":{\"steer\":null,\"steerPct\":%d,"
       "\"servoUs\":%d,\"thrPct\":%d,\"escUs\":%d},",
       steerPct, servoUs, thrPct, escUs);
     j += cbuf;
   }
   j += "\"battery\":{\"v\":null,\"pct\":null},";
 
   // system
   {
     const char* mstate = (armState == ARMED) ? "ARMED"
                        : (armState == EMERGENCY) ? "EMERGENCY" : "DISARMED";
     char sbuf[128];
     snprintf(sbuf, sizeof(sbuf),
       "\"system\":{\"mstate\":\"%s\",\"armPct\":%d,"
       "\"disarmReason\":\"%s\",\"mode\":\"MANUAL\","
       "\"emergency\":%s},",
       mstate,
       armState == ARMED ? 100 : 0,
       disarmReason.c_str(),
       armState == EMERGENCY ? "true" : "false");
     j += sbuf;
   }
 
   // wireless
   snprintf(buf, sizeof(buf),
     "\"wireless\":{\"wifiOk\":true,\"rssi\":%d},",
     WiFi.RSSI());
   j += buf;
 
   // limits
   {
     char lbuf[64];
     snprintf(lbuf, sizeof(lbuf),
       "\"limits\":{\"throttleLimit\":%d,\"autoThrottle\":%d}",
       throttleLimitPct, AUTO_THROTTLE_PCT);
     j += lbuf;
   }
 
   j += "}}";
   addCORS();
   server.send(200, "application/json", j);
 }
 
 // POST /arm
 void handleArm() {
   addCORS();
   if (armState == EMERGENCY) {
     server.send(403, "application/json", "{\"ok\":false,\"msg\":\"E-STOP active, disarm first\"}");
     return;
   }
   doArm();
   server.send(200, "application/json", "{\"ok\":true,\"state\":\"ARMED\"}");
 }
 
 // POST /disarm
 void handleDisarm() {
   addCORS();
   doDisarm("web_manual");
   server.send(200, "application/json", "{\"ok\":true,\"state\":\"DISARMED\"}");
 }
 
 // POST /estop
 void handleEstop() {
   addCORS();
   doEstop();
   server.send(200, "application/json", "{\"ok\":true,\"state\":\"EMERGENCY\"}");
 }
 
 // POST /control?steer=<us>&thr=<us>
 // Throttle, THROTTLE_LIMIT_PCT ile sinirlenir.
 void handleControl() {
   addCORS();
   if (armState != ARMED) {
     server.send(403, "application/json", "{\"ok\":false,\"msg\":\"ARM degil\"}");
     return;
   }
   if (server.hasArg("steer")) {
     int sMinLim = SERVO_MID_US - (int)((SERVO_MID_US - SERVO_MIN_US) * steerLimitPct / 100.0f);
     int sMaxLim = SERVO_MID_US + (int)((SERVO_MAX_US - SERVO_MID_US) * steerLimitPct / 100.0f);
     int us = constrain(server.arg("steer").toInt(), sMinLim, sMaxLim);
     servoUs = us;
     steerServo.writeMicroseconds(us);
   }
   if (server.hasArg("thr")) {
     int maxUs = ESC_MID_US + (int)((ESC_MAX_US - ESC_MID_US) * throttleLimitPct / 100.0f);
     int us    = constrain(server.arg("thr").toInt(), ESC_MIN_US, maxUs);
     escUs = us;
     esc.writeMicroseconds(us);
   }
   char rbuf[64];
   snprintf(rbuf, sizeof(rbuf),
     "{\"ok\":true,\"servoUs\":%d,\"escUs\":%d}", servoUs, escUs);
   server.send(200, "application/json", rbuf);
 }
 
 // POST /set?thr_limit=40  — runtime parameter update
void handleSet() {
  addCORS();
  bool changed = false;
  if (server.hasArg("thr_limit")) {
    int v = constrain(server.arg("thr_limit").toInt(), 0, 100);
    throttleLimitPct = v;
    changed = true;
    Serial.printf("[SET] throttleLimitPct = %d%%\n", v);
  }
  if (server.hasArg("steer_limit")) {
    int v = constrain(server.arg("steer_limit").toInt(), 0, 100);
    steerLimitPct = v;
    changed = true;
    Serial.printf("[SET] steerLimitPct = %d%%\n", v);
  }
  if (!changed) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"no param\"}");
    return;
  }
  char buf[80];
  snprintf(buf, sizeof(buf), "{\"ok\":true,\"throttleLimit\":%d,\"steerLimit\":%d}",
           throttleLimitPct, steerLimitPct);
  server.send(200, "application/json", buf);
}

void handleNotFound() { server.send(404, "text/plain", "Not found"); }
 
 // ---------------------------------------------------------------------------
 // Setup
 // ---------------------------------------------------------------------------
 void setup() {
   Serial.begin(115200);
   delay(500);
 
   for (uint8_t pin : {2, 21, 35, 38, 48}) neopixelWrite(pin, 0, 0, 0);
 
   pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
   pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
 
   // ARM button and LED
   pinMode(BTN_ARM,   INPUT_PULLUP);
   pinMode(BTN_ESTOP, INPUT_PULLUP);
   pinMode(LED_ARMED, OUTPUT);
   digitalWrite(LED_ARMED, LOW);
   attachInterrupt(BTN_ESTOP, onEstopISR, FALLING);  // anlik E-STOP
 
   // Servo ve ESC — acilista neutral pozisyona al
   steerServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
   esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
   steerServo.writeMicroseconds(SERVO_MID_US);
   esc.writeMicroseconds(ESC_MID_US);
   Serial.println("Servo ve ESC neutral konumda.");
 
   Wire.begin(SDA_PIN, SCL_PIN);
   mpuInit();
   delay(200);
   Serial.println("IMU kalibrasyonu basliyor — 2 sn hareketsiz tut...");
   mpuCalibrate(300);  // ~1.2 sn (300 x 4ms)
 
   Serial1.begin(115200, SERIAL_8N1, LUNA_RX, LUNA_TX);
 
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
   if (MDNS.begin("esp")) MDNS.addService("http", "tcp", 5000);
 
   server.on("/",                 handleRoot);
   server.on("/data",             handleData);
   server.on("/status",           handleStatus);
   server.on("/api/docs",         handleApiDocs);
   server.on("/api/openapi.json", handleOpenApi);
   server.on("/arm",     HTTP_POST,    handleArm);
   server.on("/arm",     HTTP_OPTIONS, handleOptions);
   server.on("/disarm",  HTTP_POST,    handleDisarm);
   server.on("/disarm",  HTTP_OPTIONS, handleOptions);
   server.on("/estop",   HTTP_POST,    handleEstop);
   server.on("/estop",   HTTP_OPTIONS, handleOptions);
   server.on("/control", HTTP_POST,    handleControl);
   server.on("/control", HTTP_OPTIONS, handleOptions);
   server.on("/set",     HTTP_POST,    handleSet);
   server.on("/set",     HTTP_OPTIONS, handleOptions);
   server.onNotFound(handleNotFound);
   server.begin();
   Serial.println("http://esp.local:5000");
 }
 
 // ---------------------------------------------------------------------------
 // Loop
 // ---------------------------------------------------------------------------
 void loop() {
   checkButtons();
   lunaUpdate();
   server.handleClient();
 
   // IMU'yu surekli oku — filter dt dogru calisson diye
   static unsigned long lastImuMs = 0;
   if (millis() - lastImuMs >= 10) {   // 100 Hz
     lastImuMs = millis();
     ImuData d = mpuRead();
     if (d.ok) { lastImu = d; cfUpdate(d); }
   }
 }
 