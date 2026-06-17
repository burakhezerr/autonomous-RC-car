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
 #include "policy_weights.h"
 
 struct ImuData {
   float ax, ay, az, gx, gy, gz, temp;
   bool ok;
 };
 
 // ---------------------------------------------------------------------------
 // WiFi
 // ---------------------------------------------------------------------------
 const char* SSID     = "Ando";
 const char* PASSWORD = "andrey12";
 const int   WEB_PORT = 5000;

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
 #define SERVO_MIN_US  1820
 #endif
 #ifndef SERVO_MID_US
 #define SERVO_MID_US  2040
 #endif
 #ifndef SERVO_MAX_US
 #define SERVO_MAX_US  2260
 #endif
 #ifndef ESC_MIN_US
 #define ESC_MIN_US    1430
 #endif
 #ifndef ESC_MID_US
 #define ESC_MID_US    1495
 #endif
 #ifndef ESC_MAX_US
 #define ESC_MAX_US    1560
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
 // Run mode
 // ---------------------------------------------------------------------------
 enum RunMode { MODE_MANUAL, MODE_AUTO };
 RunMode runMode = MODE_MANUAL;
 String lastAutoLog = "";
 bool   autoWallRecover = false;
 unsigned long wallEmergencyMs = 0;

 // --- Auto mode version ---
 enum AutoVersion { AUTO_SINGLE, AUTO_MULTI };
 AutoVersion   autoVersion     = AUTO_SINGLE;

 // --- Emergency servo timer ---
 unsigned long emergencyStartMs = 0;

 // --- RL (ESP32 içi model, sadece AUTO_MULTI'de aktif) ---
 bool          rlActive         = false;
 int           rlStrategy       = 0;
 #define OBSTACLE_RANGE_CM  150
 #define CLEAR_RANGE_CM     300
 #define STATIC_THRESHOLD   0.15f
 #define CLASSIFY_SAMPLES   3
 float         lunaHist[CLASSIFY_SAMPLES];
 unsigned long lunaHistMs[CLASSIFY_SAMPLES];
 int           lunaHistIdx  = 0;
 bool          lunaHistFull = false;
 
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
   armState        = EMERGENCY;
   emergencyStartMs = millis();
   disarmReason    = "emergency_stop";
   escUs           = ESC_MID_US;
   servoUs         = SERVO_MID_US;
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
 // Autonomous control helpers
 // ---------------------------------------------------------------------------
 int proxLv1 = 35;
 int proxLv2 = 60;
 int proxLv3 = 85;
 int proxLv4 = 115;

 int lidarStopCm = 50;    // ≤ this → EMERGENCY in auto mode
 int lidarSlowCm = 100;   // ≤ this → reduced speed in auto mode
 int sonarCloseCm = 10;   // ≤ this → stop + full steer away (auto & manual)

 int proximityLevel(float cm) {
   if (cm < 0)        return 5;
   if (cm <= proxLv1) return 1;
   if (cm <= proxLv2) return 2;
   if (cm <= proxLv3) return 3;
   if (cm <= proxLv4) return 4;
   return 5;
 }

 void autoControl() {
   if (armState != ARMED) return;

   // LiDAR wall detection — takes priority
   if (lunaDistCm > 0 && lunaDistCm <= lidarStopCm) {
     escUs = ESC_MID_US; servoUs = SERVO_MID_US;
     esc.writeMicroseconds(ESC_MID_US);
     steerServo.writeMicroseconds(SERVO_MID_US);
     armState          = EMERGENCY;
     emergencyStartMs  = millis();
     disarmReason      = "wall_detected";
     autoWallRecover   = true;
     wallEmergencyMs   = millis();
     lastAutoLog       = "Wall detected";
     Serial.println("[AUTO] Wall detected — EMERGENCY");
     return;
   }

   // Sonar ≤10cm: stop + full steer away (4-level equivalent)
   bool leftClose  = (lastLeft  > 0 && lastLeft  <= sonarCloseCm);
   bool rightClose = (lastRight > 0 && lastRight <= sonarCloseCm);
   if (leftClose || rightClose) {
     escUs = ESC_MID_US;
     esc.writeMicroseconds(ESC_MID_US);
     if (leftClose && !rightClose)       servoUs = SERVO_MAX_US;
     else if (rightClose && !leftClose)  servoUs = SERVO_MIN_US;
     else                                servoUs = SERVO_MID_US;
     steerServo.writeMicroseconds(servoUs);
     lastAutoLog = "Sonar emergency — steering away";
     return;
   }

   // Ultrasonic proximity steering
   int lvL  = proximityLevel(lastLeft);
   int lvR  = proximityLevel(lastRight);
   int diff = lvR - lvL;
   if (diff == 0) {
     servoUs = SERVO_MID_US;
   } else {
     int   ad  = abs(diff);
     float pct = (ad == 1) ? 0.25f : (ad == 2) ? 0.50f : (ad == 3) ? 0.75f : 1.00f;
     servoUs = (diff > 0)
       ? SERVO_MID_US + (int)((SERVO_MAX_US - SERVO_MID_US) * pct)
       : SERVO_MID_US - (int)((SERVO_MID_US - SERVO_MIN_US) * pct);
   }
   servoUs = constrain(servoUs, SERVO_MIN_US, SERVO_MAX_US);
   steerServo.writeMicroseconds(servoUs);

   // LiDAR-based speed: >100cm=max, 25-100cm=medium
   int autoUs;
   if (lunaDistCm < 0 || lunaDistCm > lidarSlowCm) {
     autoUs      = 1560;
     lastAutoLog = "Full speed";
   } else {
     autoUs      = 1550;
     lastAutoLog = "Slowing — obstacle ahead";
   }
   escUs = autoUs;
   esc.writeMicroseconds(escUs);
 }

 void manualSafety() {
   if (armState != ARMED) return;
   bool sonarDanger = (lastLeft  >= 0 && lastLeft  <= sonarCloseCm)
                   || (lastRight >= 0 && lastRight <= sonarCloseCm);
   bool lidarDanger = (lunaDistCm > 0 && lunaDistCm <= 25);
   if (sonarDanger || lidarDanger) {
     escUs = ESC_MID_US;
     esc.writeMicroseconds(ESC_MID_US);
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
   long dur = pulseIn(echo, HIGH, 15000);  // 15ms = ~2.6m range
   return dur == 0 ? -1 : dur / 58.0f;
 }
 
 // ---------------------------------------------------------------------------
 // Web server
 // ---------------------------------------------------------------------------
 WebServer server(WEB_PORT);
 
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

   <h2>Joystick Control</h2>
   <div class="ctrl">
     <div id="gp-status" style="font-size:10px;color:#555;margin-bottom:8px">No gamepad — press any button on controller</div>
     <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:10px">
       <div>
         <div class="slider-lbl">Servo µs — min / mid / max</div>
         <div style="display:flex;gap:4px">
           <input type="number" id="gp-smin" value="1820" style="width:56px;background:#1e1e1e;border:1px solid #333;color:#82b1ff;font-size:11px;font-family:monospace;text-align:center;border-radius:4px;padding:2px">
           <input type="number" id="gp-smid" value="2040" style="width:56px;background:#1e1e1e;border:1px solid #333;color:#82b1ff;font-size:11px;font-family:monospace;text-align:center;border-radius:4px;padding:2px">
           <input type="number" id="gp-smax" value="2260" style="width:56px;background:#1e1e1e;border:1px solid #333;color:#82b1ff;font-size:11px;font-family:monospace;text-align:center;border-radius:4px;padding:2px">
         </div>
       </div>
       <div>
         <div class="slider-lbl">ESC µs — min / mid / max</div>
         <div style="display:flex;gap:4px">
           <input type="number" id="gp-emin" value="1070" style="width:56px;background:#1e1e1e;border:1px solid #333;color:#82b1ff;font-size:11px;font-family:monospace;text-align:center;border-radius:4px;padding:2px">
           <input type="number" id="gp-emid" value="1540" style="width:56px;background:#1e1e1e;border:1px solid #333;color:#82b1ff;font-size:11px;font-family:monospace;text-align:center;border-radius:4px;padding:2px">
           <input type="number" id="gp-emax" value="1750" style="width:56px;background:#1e1e1e;border:1px solid #333;color:#82b1ff;font-size:11px;font-family:monospace;text-align:center;border-radius:4px;padding:2px">
         </div>
       </div>
     </div>
     <div style="display:flex;gap:10px;align-items:center;margin-bottom:10px;font-size:11px;color:#666;flex-wrap:wrap">
       <label>Steer axis: <select id="gp-sa" style="background:#1e1e1e;color:#82b1ff;border:1px solid #333;border-radius:4px;padding:2px;font-size:11px"><option value="0" selected>L-X (0)</option><option value="1">L-Y (1)</option><option value="2">R-X (2)</option><option value="3">R-Y (3)</option></select></label>
       <label>Thr axis: <select id="gp-ta" style="background:#1e1e1e;color:#82b1ff;border:1px solid #333;border-radius:4px;padding:2px;font-size:11px"><option value="0">L-X (0)</option><option value="1" selected>L-Y (1)</option><option value="2">R-X (2)</option><option value="3">R-Y (3)</option></select></label>
       <label>Deadzone: <input type="number" id="gp-dz" value="0.08" step="0.01" min="0" max="0.5" style="width:44px;background:#1e1e1e;border:1px solid #333;color:#82b1ff;border-radius:4px;padding:2px;font-size:11px;font-family:monospace;text-align:center"></label>
     </div>
     <div style="display:flex;justify-content:center;margin:12px 0;gap:16px;align-items:center">
       <div id="joy-outer" touch-action="none"
         style="width:160px;height:160px;border-radius:50%;border:2px solid #444;background:#111;position:relative;cursor:grab;user-select:none;flex-shrink:0"
         onpointerdown="joyDown(event)" onpointermove="joyMove(event)" onpointerup="joyUp(event)" onpointercancel="joyUp(event)">
         <div style="position:absolute;top:50%;left:0;right:0;height:1px;background:#222;transform:translateY(-50%)"></div>
         <div style="position:absolute;left:50%;top:0;bottom:0;width:1px;background:#222;transform:translateX(-50%)"></div>
         <div id="joy-inner" style="width:48px;height:48px;border-radius:50%;background:#82b1ff;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);pointer-events:none"></div>
       </div>
       <div style="display:flex;flex-direction:column;gap:6px">
         <div class="tile" style="min-width:72px"><div class="lbl">Steer</div><div class="val" id="gp-sa-val" style="font-size:18px">—</div></div>
         <div class="tile" style="min-width:72px"><div class="lbl">Throttle</div><div class="val" id="gp-ta-val" style="font-size:18px">—</div></div>
       </div>
     </div>
     <button id="btn-gp" class="btn" onclick="toggleGp()" disabled style="background:#333;color:#555">GAMEPAD OFF</button>
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
   <div style="display:flex;gap:12px;margin-top:10px">
     <div style="flex:1">
       <div style="font-size:10px;color:#666;margin-bottom:4px;text-align:center">LEFT PROXIMITY</div>
       <div id="prox-l" style="display:flex;gap:3px;justify-content:center"></div>
     </div>
     <div style="flex:1">
       <div style="font-size:10px;color:#666;margin-bottom:4px;text-align:center">RIGHT PROXIMITY</div>
       <div id="prox-r" style="display:flex;gap:3px;justify-content:center"></div>
     </div>
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
       <button class="btn" id="btn-auto" onclick="sendMode()" style="background:#333;color:#aaa">AUTO OFF</button>
       <button class="btn" id="btn-av" onclick="sendAutoVersion()" style="background:#1a1a2e;color:#555;display:none">SINGLE-CAR</button>
     </div>
     <div id="auto-info" style="display:none;margin-top:10px">
       <div class="grid2">
         <div class="tile"><div class="lbl">Left Level</div><div class="val" id="lv-l" style="font-size:28px">—</div></div>
         <div class="tile"><div class="lbl">Right Level</div><div class="val" id="lv-r" style="font-size:28px">—</div></div>
       </div>
       <div style="font-size:10px;color:#555;margin-top:6px;text-align:center">Lv1≤45 · Lv2≤60 · Lv3≤75 · Lv4≤95 · sonar≤10cm→stop+steer · LiDAR≤25cm→EMERGENCY · LiDAR>100cm→full</div>
       <div id="auto-log" style="margin-top:8px;padding:6px 10px;background:#111;border:1px solid #333;border-radius:6px;font-size:12px;color:#00e676;font-family:monospace;min-height:24px;text-align:center">—</div>
       <div id="rl-badge" style="display:none;margin-top:6px;padding:4px 10px;background:#4a0080;border-radius:6px;font-size:11px;color:#ce93d8;font-family:monospace;text-align:center">RL ACTIVE</div>
     </div>
     <div class="slider-row">
       <div class="slider-lbl">Steering (µs)</div>
       <input type="range" id="sl-steer" min="1000" max="2000" value="1500" step="10"
              oninput="steerVal(this.value); scheduleControl()">
       <div class="slider-val" id="steer-val">1500 µs</div>
     </div>
     <div class="slider-row">
       <div class="slider-lbl">Throttle (µs)</div>
       <input type="range" id="sl-thr" min="1430" max="1560" value="1495" step="1"
              oninput="thrVal(this.value); scheduleControl()">
       <div class="slider-val" id="thr-val">1500 µs</div>
     </div>
   </div>



   <div id="upd">waiting...</div>
 </div>
 
 <script>

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
 
     setTile('val-luna','box-luna', d.luna, 0, v=>v<25);
     setTile('val-lstr','box-lstr', d.lstr>0?d.lstr:null, 0, v=>v<200);
     setTile('val-ltmp','box-ltmp', d.ltmp, 1);
     setTile('val-l','box-l', d.left,  1, v=>v<20);
     setTile('val-r','box-r', d.right, 1, v=>v<20);
     drawProx('prox-l', d.lLv);
     drawProx('prox-r', d.rLv);
     if(d.mstate && d.mstate !== currentState) updateStateBadge(d.mstate);
     if(autoMode){
       const lvlColor=v=>v<=2;
       const el=document.getElementById('lv-l');
       if(el){ el.textContent=d.lLv??'—'; el.style.color=lvlColor(d.lLv)?'#ff5252':'#69f0ae'; }
       const er=document.getElementById('lv-r');
       if(er){ er.textContent=d.rLv??'—'; er.style.color=lvlColor(d.rLv)?'#ff5252':'#69f0ae'; }
       const al=document.getElementById('auto-log');
       if(al&&d.alog!=null){
         al.textContent=d.alog||'—';
         const isRL=d.alog.startsWith('RL:');
         al.style.color=d.alog==='Wall detected'?'#ff1744':d.alog.startsWith('Wall cleared')?'#69f0ae':isRL?'#ce93d8':'#00e676';
         const rb=document.getElementById('rl-badge');
         if(rb) rb.style.display=(isRL&&autoVersionMode==='multi')?'block':'none';
       }
     }
 
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
 let neutralSteer = 2040, neutralThr = 1540;
 let autoMode = false;
 let autoVersionMode = 'single';
 function updateStateBadge(state){
   currentState = state;
   const b = document.getElementById('state-badge');
   b.textContent = state;
   b.className = 'state-' + state.toLowerCase();
   const armed = state === 'ARMED' && !autoMode;
   document.getElementById('sl-steer').disabled = !armed;
   document.getElementById('sl-thr').disabled   = !armed;
 }
 
 function steerVal(v){ document.getElementById('steer-val').textContent = v + ' µs'; }
 function thrVal(v)  { document.getElementById('thr-val').textContent   = v + ' µs'; }
 const PROX_COLORS=['#ff1744','#ff5722','#ffc107','#8bc34a','#00e676'];
 function drawProx(id, lv){
   const el=document.getElementById(id); if(!el)return;
   if(!el.children.length){
     for(let i=1;i<=5;i++){
       const d=document.createElement('div');
       d.style.cssText='width:28px;height:28px;border-radius:4px;border:1px solid #333;transition:background 0.2s;display:flex;align-items:center;justify-content:center;font-size:10px;color:#fff;font-weight:bold';
       d.textContent=i; el.appendChild(d);
     }
   }
   Array.from(el.children).forEach((d,i)=>{
     const active=lv!=null&&(i+1)===lv;
     d.style.background=active?PROX_COLORS[i]:'#1a1a1a';
     d.style.borderColor=active?PROX_COLORS[i]:'#333';
   });
 }
 


 // 80ms throttle on slider oninput — avoids flooding ESP32
 let _ctrlTimer = null;
 function scheduleControl(){
   if(_ctrlTimer) return;
   _ctrlTimer = setTimeout(()=>{ _ctrlTimer=null; sendControl(); }, 150);
 }
 
 async function sendArm(){
   try{ const r=await fetch('/arm',{method:'POST'}); const d=await r.json();
     if(d.ok) updateStateBadge('ARMED');
   } catch(e){ alert('ARM error'); }
 }
 async function sendDisarm(){
   try{ const r=await fetch('/disarm',{method:'POST'}); const d=await r.json();
     if(d.ok){ updateStateBadge('DISARMED');
       document.getElementById('sl-steer').value=neutralSteer; steerVal(neutralSteer);
       document.getElementById('sl-thr').value=neutralThr;     thrVal(neutralThr); }
   } catch(e){ alert('DISARM error'); }
 }
 async function sendEstop(){
   try{ const r=await fetch('/estop',{method:'POST'}); const d=await r.json();
     if(d.ok){ updateStateBadge('EMERGENCY');
       document.getElementById('sl-steer').value=neutralSteer; steerVal(neutralSteer);
       document.getElementById('sl-thr').value=neutralThr;     thrVal(neutralThr); }
   } catch(e){ alert('E-STOP error'); }
 }
 function updateAvBtn(){
   const b=document.getElementById('btn-av');
   if(!b) return;
   const isMulti = autoVersionMode==='multi';
   b.textContent = isMulti ? 'MULTI-CAR' : 'SINGLE-CAR';
   b.style.background = isMulti ? '#6a0dad' : '#1a1a2e';
   b.style.color = isMulti ? '#ce93d8' : '#7986cb';
   b.style.display = autoMode ? 'inline-block' : 'none';
 }
 async function sendAutoVersion(){
   const next = autoVersionMode==='single' ? 'multi' : 'single';
   try{
     const r=await fetch('/auto-version?version='+next,{method:'POST'}); const d=await r.json();
     if(d.ok){
       autoVersionMode = next;
       updateAvBtn();
       if(next==='single'){ document.getElementById('rl-badge').style.display='none'; }
     }
   } catch(e){ alert('Auto version error'); }
 }
 async function sendMode(){
   const next = autoMode ? 'manual' : 'auto';
   try{
     const r=await fetch('/mode?mode='+next,{method:'POST'}); const d=await r.json();
     if(d.ok){
       autoMode=(d.mode==='AUTO');
       const b=document.getElementById('btn-auto');
       b.textContent=autoMode?'AUTO ON':'AUTO OFF';
       b.style.background=autoMode?'#00c853':'#333';
       b.style.color=autoMode?'#000':'#aaa';
       document.getElementById('auto-info').style.display=autoMode?'block':'none';
       if(!autoMode){ document.getElementById('rl-badge').style.display='none'; }
       updateAvBtn();
       if(autoMode && gpOn) toggleGp();
       updateStateBadge(currentState);
     }
   } catch(e){ alert('Mode error'); }
 }
 async function sendControl(){
   if(currentState !== 'ARMED' || autoMode) return;
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
     if(sMin!=null){ document.getElementById('sl-steer').min=sMin; document.getElementById('gp-smin').value=sMin; document.getElementById('gp-smin').min=sMin; }
     if(sMax!=null){ document.getElementById('sl-steer').max=sMax; document.getElementById('gp-smax').value=sMax; document.getElementById('gp-smax').max=sMax; }
     if(sMid!=null){ neutralSteer=sMid; document.getElementById('sl-steer').value=sMid; steerVal(sMid); document.getElementById('gp-smid').value=sMid; document.getElementById('gp-smin').max=sMid; document.getElementById('gp-smax').min=sMid; }
     const eMin=cfg.esc?.minUs, eMid=cfg.esc?.neutralUs, eMax=cfg.esc?.maxUs;
     if(eMin!=null){ document.getElementById('sl-thr').min=eMin; document.getElementById('gp-emin').value=eMin; document.getElementById('gp-emin').min=eMin; }
     if(eMax!=null){ document.getElementById('sl-thr').max=eMax; document.getElementById('gp-emax').value=eMax; document.getElementById('gp-emax').max=eMax; }
     if(eMid!=null){ neutralThr=eMid; document.getElementById('sl-thr').value=eMid; thrVal(eMid); document.getElementById('gp-emid').value=eMid; document.getElementById('gp-emin').max=eMid; document.getElementById('gp-emax').min=eMid; }
   }
 }).catch(()=>{});
 
 poll();
 setInterval(poll, 500);


 // --- Gamepad ---
 let gpOn=false,gpIdx=-1,gpTimer=null;
 window.addEventListener('gamepadconnected',e=>{
   gpIdx=e.gamepad.index;
   document.getElementById('gp-status').textContent='Connected: '+e.gamepad.id.slice(0,60);
   const b=document.getElementById('btn-gp');
   b.disabled=false;b.style.background='#333';b.style.color='#fff';
 });
 window.addEventListener('gamepaddisconnected',e=>{
   if(e.gamepad.index!==gpIdx)return;
   gpIdx=-1;gpOn=false;stopGp();
   document.getElementById('gp-status').textContent='No gamepad — press any button on controller';
   const b=document.getElementById('btn-gp');
   b.disabled=true;b.style.background='#333';b.style.color='#555';b.textContent='GAMEPAD OFF';
 });
 function toggleGp(){
   gpOn=!gpOn;
   gpOn?startGp():stopGp();
   const b=document.getElementById('btn-gp');
   b.textContent=gpOn?'GAMEPAD ON':'GAMEPAD OFF';
   b.style.background=gpOn?'#00c853':'#333';
   b.style.color=gpOn?'#000':'#fff';
 }
 function startGp(){if(!gpTimer)gpTimer=setInterval(gpPoll,50);}
 function stopGp(){if(gpTimer){clearInterval(gpTimer);gpTimer=null;}}
 function gpPoll(){
   const gp=navigator.getGamepads()[gpIdx];
   if(!gp)return;
   const dz=parseFloat(document.getElementById('gp-dz').value)||0.08;
   const saI=parseInt(document.getElementById('gp-sa').value)||0;
   const taI=parseInt(document.getElementById('gp-ta').value)||1;
   let sa=gp.axes[saI]||0,ta=gp.axes[taI]||0;
   if(Math.abs(sa)<dz)sa=0;
   if(Math.abs(ta)<dz)ta=0;
   document.getElementById('gp-sa-val').textContent=sa.toFixed(2);
   document.getElementById('gp-ta-val').textContent=ta.toFixed(2);
   if(!gpOn||currentState!=='ARMED')return;
   const sMin=parseInt(document.getElementById('gp-smin').value)||1730;
   const sMid=parseInt(document.getElementById('gp-smid').value)||2030;
   const sMax=parseInt(document.getElementById('gp-smax').value)||2330;
   const eMin=parseInt(document.getElementById('gp-emin').value)||1070;
   const eMid=parseInt(document.getElementById('gp-emid').value)||1540;
   const eMax=parseInt(document.getElementById('gp-emax').value)||1750;
   const stUs=Math.round(sa<0?sMid+sa*(sMid-sMin):sMid+sa*(sMax-sMid));
   const thUs=Math.round(ta<0?eMid-ta*(eMax-eMid):eMid-ta*(eMid-eMin));
   document.getElementById('sl-steer').value=stUs;steerVal(stUs);
   document.getElementById('sl-thr').value=thUs;thrVal(thUs);
   fetch('/control?steer='+stUs+'&thr='+thUs,{method:'POST'}).catch(()=>{});
 }

 // --- Virtual Joystick ---
 let joyActive=false,joyCX=0,joyCY=0,joyR=0,joySa=0,joyTa=0,joyTimer=null;
 function joyDown(e){
   e.currentTarget.setPointerCapture(e.pointerId);
   const rc=e.currentTarget.getBoundingClientRect();
   joyCX=rc.left+rc.width/2; joyCY=rc.top+rc.height/2; joyR=rc.width/2-24;
   joyActive=true; joyMove(e);
   if(!joyTimer)joyTimer=setInterval(joyPoll,50);
 }
 function joyMove(e){
   if(!joyActive)return;
   const dx=e.clientX-joyCX, dy=e.clientY-joyCY;
   const dist=Math.sqrt(dx*dx+dy*dy);
   const s=Math.min(dist,joyR)/Math.max(dist,0.001);
   const cx=dx*s, cy=dy*s;
   joySa=cx/joyR; joyTa=-cy/joyR;
   const half=joyR+24;
   const inn=document.getElementById('joy-inner');
   inn.style.left=(half+cx)+'px'; inn.style.top=(half+cy)+'px';
   inn.style.transform='translate(-50%,-50%)';
   document.getElementById('gp-sa-val').textContent=joySa.toFixed(2);
   document.getElementById('gp-ta-val').textContent=joyTa.toFixed(2);
 }
 function joyUp(e){
   joyActive=false; joySa=0; joyTa=0;
   const inn=document.getElementById('joy-inner');
   inn.style.left='50%'; inn.style.top='50%'; inn.style.transform='translate(-50%,-50%)';
   document.getElementById('gp-sa-val').textContent='0.00';
   document.getElementById('gp-ta-val').textContent='0.00';
   if(joyTimer){clearInterval(joyTimer);joyTimer=null;}
   if(currentState==='ARMED'){
     const sMid=parseInt(document.getElementById('gp-smid').value)||2030;
     const eMid=parseInt(document.getElementById('gp-emid').value)||1540;
     fetch('/control?steer='+sMid+'&thr='+eMid,{method:'POST'}).catch(()=>{});
   }
 }
 function joyPoll(){
   if(!joyActive||currentState!=='ARMED')return;
   const sMin=parseInt(document.getElementById('gp-smin').value)||1730;
   const sMid=parseInt(document.getElementById('gp-smid').value)||2030;
   const sMax=parseInt(document.getElementById('gp-smax').value)||2330;
   const eMin=parseInt(document.getElementById('gp-emin').value)||1070;
   const eMid=parseInt(document.getElementById('gp-emid').value)||1540;
   const eMax=parseInt(document.getElementById('gp-emax').value)||1750;
   const stUs=Math.round(joySa<0?sMid+joySa*(sMid-sMin):sMid+joySa*(sMax-sMid));
   const thUs=Math.round(joyTa>=0?eMid+joyTa*(eMax-eMid):eMid+joyTa*(eMid-eMin));
   document.getElementById('sl-steer').value=stUs; steerVal(stUs);
   document.getElementById('sl-thr').value=thUs; thrVal(thUs);
   fetch('/control?steer='+stUs+'&thr='+thUs,{method:'POST'}).catch(()=>{});
 }
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
    "/mode":{
      "post":{
        "tags":["Control"],
        "summary":"Set run mode",
        "description":"Switch between AUTO and MANUAL mode.",
        "parameters":[{"name":"mode","in":"query","required":true,"schema":{"type":"string","enum":["auto","manual"]}}],
        "responses":{"200":{"description":"Mode changed","content":{"application/json":{"example":{"ok":true,"mode":"AUTO"}}}}}
      }
    },
    "/auto-version":{
      "post":{
        "tags":["Control"],
        "summary":"Set auto mode version",
        "description":"Switch between single-car (ultrasonics+LiDAR only) and multi-car (RL enabled) autonomous modes.",
        "parameters":[{"name":"version","in":"query","required":true,"schema":{"type":"string","enum":["single","multi"]}}],
        "responses":{"200":{"description":"Version changed","content":{"application/json":{"example":{"ok":true,"autoVersion":"multi-car"}}}}}
      }
    },
    "/set-prox":{
      "post":{
        "tags":["Config"],
        "summary":"Update sonar proximity thresholds",
        "description":"Updates all 5 sonar thresholds at once. Only allowed when DISARMED and in MANUAL mode. Rules: close>=10, lv1>=25, each consecutive level must have >=20cm gap.",
        "parameters":[
          {"name":"lv1","in":"query","required":true,"description":"Level 1 upper bound (cm, min 25)","schema":{"type":"integer","example":35}},
          {"name":"lv2","in":"query","required":true,"description":"Level 2 upper bound (cm, min lv1+20)","schema":{"type":"integer","example":60}},
          {"name":"lv3","in":"query","required":true,"description":"Level 3 upper bound (cm, min lv2+20)","schema":{"type":"integer","example":85}},
          {"name":"lv4","in":"query","required":true,"description":"Level 4 upper bound (cm, min lv3+20)","schema":{"type":"integer","example":115}},
          {"name":"close","in":"query","required":true,"description":"Emergency stop+steer threshold (cm, min 10)","schema":{"type":"integer","example":10}}
        ],
        "responses":{
          "200":{"description":"Thresholds updated","content":{"application/json":{"example":{"ok":true,"proxThresholds":{"lv1":35,"lv2":60,"lv3":85,"lv4":115,"close":10}}}}},
          "400":{"description":"Validation error","content":{"application/json":{"example":{"ok":false,"msg":"Each consecutive level must have >= 20 cm gap"}}}},
          "403":{"description":"ARMED or AUTO mode","content":{"application/json":{"example":{"ok":false,"msg":"Cannot change thresholds while ARMED"}}}}
        }
      }
    },
    "/set-lidar":{
      "post":{
        "tags":["Config"],
        "summary":"Update LiDAR thresholds",
        "description":"Updates LiDAR stop and slow thresholds. Only allowed when DISARMED and in MANUAL mode. Rules: stop>=40, slow>=stop*2.",
        "parameters":[
          {"name":"stop","in":"query","required":true,"description":"EMERGENCY trigger distance (cm, min 40)","schema":{"type":"integer","example":50}},
          {"name":"slow","in":"query","required":true,"description":"Reduced speed threshold (cm, min stop*2)","schema":{"type":"integer","example":100}}
        ],
        "responses":{
          "200":{"description":"Thresholds updated","content":{"application/json":{"example":{"ok":true,"lidarThresholds":{"stop":50,"slow":100}}}}},
          "400":{"description":"Validation error","content":{"application/json":{"example":{"ok":false,"msg":"slow must be >= 2x stop (min 100 cm)"}}}},
          "403":{"description":"ARMED or AUTO mode","content":{"application/json":{"example":{"ok":false,"msg":"Cannot change thresholds in AUTO mode"}}}}
        }
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
            "description":"Servo µs: 1730=full left | 2030=centre | 2330=full right",
            "schema":{"type":"integer","minimum":1730,"maximum":2330,"example":2030}
          },
          {
            "name":"thr","in":"query","required":false,
            "description":"ESC µs: 1490=neutral | 1430=full forward | 1550=full reverse",
            "schema":{"type":"integer","minimum":1430,"maximum":1550,"example":1490}
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
   int lvL = proximityLevel(l), lvR = proximityLevel(r);
   const char* mstate = (armState == ARMED) ? "ARMED"
                      : (armState == EMERGENCY) ? "EMERGENCY" : "DISARMED";
   char buf[420];
   if (imu.ok) {
     snprintf(buf, sizeof(buf),
       "{\"left\":%.1f,\"right\":%.1f,\"lLv\":%d,\"rLv\":%d,"
       "\"luna\":%d,\"lstr\":%d,\"ltmp\":%.1f,"
       "\"imu_ok\":true,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
       "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"temp\":%.1f,"
       "\"alog\":\"%s\",\"mstate\":\"%s\"}",
       l, r, lvL, lvR, lunaDistCm, lunaStr, lunaTemp,
       imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.temp,
       lastAutoLog.c_str(), mstate);
   } else {
     snprintf(buf, sizeof(buf),
       "{\"left\":%.1f,\"right\":%.1f,\"lLv\":%d,\"rLv\":%d,"
       "\"luna\":%d,\"lstr\":%d,\"ltmp\":%.1f,\"imu_ok\":false,"
       "\"alog\":\"%s\",\"mstate\":\"%s\"}",
       l, r, lvL, lvR, lunaDistCm, lunaStr, lunaTemp,
       lastAutoLog.c_str(), mstate);
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
 
   auto proxLv = [](float cm) -> int { return proximityLevel(cm); };
 
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
   char buf[768];
   snprintf(buf, sizeof(buf),
     "\"sonar\":{"
       "\"lCm\":%.1f,\"rCm\":%.1f,"
       "\"lLv\":%d,\"rLv\":%d,"
       "\"lFwd\":%.1f,\"lLat\":%.1f,"
       "\"rFwd\":%.1f,\"rLat\":%.1f,"
       "\"angle\":%.1f,"
       "\"thresholds\":{\"close\":%d,\"lv1\":%d,\"lv2\":%d,\"lv3\":%d,\"lv4\":%d}"
     "},",
     lastLeft, lastRight,
     proxLv(lastLeft), proxLv(lastRight),
     lFwd, lLat, rFwd, rLat,
     SONAR_ANGLE_DEG,
     sonarCloseCm, proxLv1, proxLv2, proxLv3, proxLv4);
   j += buf;

   // lidar
   snprintf(buf, sizeof(buf),
     "\"lidar\":{\"ok\":%s,\"cm\":%.1f,"
       "\"thresholds\":{\"stop\":%d,\"slow\":%d}"
     "},",
     lidarOk ? "true" : "false",
     (float)lunaDistCm,
     lidarStopCm, lidarSlowCm);
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
     char sbuf[256];
     snprintf(sbuf, sizeof(sbuf),
       "\"system\":{\"mstate\":\"%s\",\"armPct\":%d,"
       "\"disarmReason\":\"%s\",\"mode\":\"%s\","
       "\"auto-mode-enabled\":%s,\"auto-mode-status\":\"%s\","
       "\"emergency\":%s},",
       mstate,
       armState == ARMED ? 100 : 0,
       disarmReason.c_str(),
       runMode == MODE_AUTO ? "AUTO" : "MANUAL",
       runMode == MODE_AUTO ? "true" : "false",
       autoVersion == AUTO_MULTI ? "multi-car" : "single-car",
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
       "\"limits\":{\"throttleLimit\":100,\"autoThrottle\":%d}",
       AUTO_THROTTLE_PCT);
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
   if (runMode == MODE_AUTO) {
     server.send(403, "application/json", "{\"ok\":false,\"msg\":\"AUTO mode\"}");
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
     int us = constrain(server.arg("thr").toInt(), ESC_MIN_US, ESC_MAX_US);
     escUs = us;
     esc.writeMicroseconds(us);
   }
   manualSafety();  // override throttle if too close
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

 // POST /set-prox?lv1=35&lv2=60&lv3=85&lv4=115&close=10
 void handleSetProx() {
   addCORS();
   if (runMode == MODE_AUTO) {
     server.send(403, "application/json",
       "{\"ok\":false,\"msg\":\"Cannot change thresholds in AUTO mode\"}");
     return;
   }
   if (armState == ARMED) {
     server.send(403, "application/json",
       "{\"ok\":false,\"msg\":\"Cannot change thresholds while ARMED\"}");
     return;
   }
   if (!server.hasArg("lv1") || !server.hasArg("lv2") ||
       !server.hasArg("lv3") || !server.hasArg("lv4") || !server.hasArg("close")) {
     server.send(400, "application/json",
       "{\"ok\":false,\"msg\":\"All params required: lv1, lv2, lv3, lv4, close\"}");
     return;
   }
   int v1    = server.arg("lv1").toInt();
   int v2    = server.arg("lv2").toInt();
   int v3    = server.arg("lv3").toInt();
   int v4    = server.arg("lv4").toInt();
   int close = server.arg("close").toInt();
   if (close < 10) {
     server.send(400, "application/json",
       "{\"ok\":false,\"msg\":\"close must be >= 10 cm\"}");
     return;
   }
   if (v1 < 25) {
     server.send(400, "application/json",
       "{\"ok\":false,\"msg\":\"lv1 must be >= 25 cm\"}");
     return;
   }
   if (v2 - v1 < 20 || v3 - v2 < 20 || v4 - v3 < 20) {
     server.send(400, "application/json",
       "{\"ok\":false,\"msg\":\"Each consecutive level must have >= 20 cm gap (lv1<lv2<lv3<lv4)\"}");
     return;
   }
   proxLv1      = v1;
   proxLv2      = v2;
   proxLv3      = v3;
   proxLv4      = v4;
   sonarCloseCm = close;
   Serial.printf("[SET-PROX] lv1=%d lv2=%d lv3=%d lv4=%d close=%d\n", v1, v2, v3, v4, close);
   char buf[128];
   snprintf(buf, sizeof(buf),
     "{\"ok\":true,\"proxThresholds\":{\"lv1\":%d,\"lv2\":%d,\"lv3\":%d,\"lv4\":%d,\"close\":%d}}",
     proxLv1, proxLv2, proxLv3, proxLv4, sonarCloseCm);
   server.send(200, "application/json", buf);
 }

 // POST /set-lidar?stop=50&slow=100
 void handleSetLidar() {
   addCORS();
   if (runMode == MODE_AUTO) {
     server.send(403, "application/json",
       "{\"ok\":false,\"msg\":\"Cannot change thresholds in AUTO mode\"}");
     return;
   }
   if (armState == ARMED) {
     server.send(403, "application/json",
       "{\"ok\":false,\"msg\":\"Cannot change thresholds while ARMED\"}");
     return;
   }
   if (!server.hasArg("stop") || !server.hasArg("slow")) {
     server.send(400, "application/json",
       "{\"ok\":false,\"msg\":\"Both params required: stop, slow\"}");
     return;
   }
   int vstop = server.arg("stop").toInt();
   int vslow = server.arg("slow").toInt();
   if (vstop < 40) {
     server.send(400, "application/json",
       "{\"ok\":false,\"msg\":\"stop must be >= 40 cm\"}");
     return;
   }
   if (vslow < vstop * 2) {
     char buf[96];
     snprintf(buf, sizeof(buf),
       "{\"ok\":false,\"msg\":\"slow must be >= 2x stop (min %d cm)\"}",
       vstop * 2);
     server.send(400, "application/json", buf);
     return;
   }
   lidarStopCm = vstop;
   lidarSlowCm = vslow;
   Serial.printf("[SET-LIDAR] stop=%d slow=%d\n", vstop, vslow);
   char buf[80];
   snprintf(buf, sizeof(buf),
     "{\"ok\":true,\"lidarThresholds\":{\"stop\":%d,\"slow\":%d}}",
     lidarStopCm, lidarSlowCm);
   server.send(200, "application/json", buf);
 }

 void handleAutoVersion() {
   addCORS();
   if (!server.hasArg("version")) {
     server.send(400, "application/json", "{\"ok\":false,\"msg\":\"no version param\"}");
     return;
   }
   String v = server.arg("version");
   if (v == "multi") {
     autoVersion = AUTO_MULTI;
   } else {
     autoVersion  = AUTO_SINGLE;
     rlActive     = false;
     lunaHistIdx  = 0;
     lunaHistFull = false;
   }
   const char* vstr = (autoVersion == AUTO_MULTI) ? "multi-car" : "single-car";
   char buf[64];
   snprintf(buf, sizeof(buf), "{\"ok\":true,\"autoVersion\":\"%s\"}", vstr);
   server.send(200, "application/json", buf);
 }

 void handleMode() {
   addCORS();
   if (!server.hasArg("mode")) {
     server.send(400, "application/json", "{\"ok\":false,\"msg\":\"no mode param\"}");
     return;
   }
   String m = server.arg("mode");
   if (m == "auto") {
     runMode         = MODE_AUTO;
     autoWallRecover = false;
     lastAutoLog     = "";
     Serial.println("[MODE] AUTO");
   } else {
     runMode = MODE_MANUAL;
     if (armState == ARMED) {
       escUs = ESC_MID_US; servoUs = SERVO_MID_US;
       esc.writeMicroseconds(ESC_MID_US);
       steerServo.writeMicroseconds(SERVO_MID_US);
     }
     Serial.println("[MODE] MANUAL");
   }
   String resp = String("{\"ok\":true,\"mode\":\"") + (runMode == MODE_AUTO ? "AUTO" : "MANUAL") + "\"}";
   server.send(200, "application/json", resp);
 }

// LiDAR geçmişini güncelle
void updateLunaHistory() {
  lunaHist[lunaHistIdx]   = lunaDistCm / 100.0f;
  lunaHistMs[lunaHistIdx] = millis();
  lunaHistIdx = (lunaHistIdx + 1) % CLASSIFY_SAMPLES;
  if (lunaHistIdx == 0) lunaHistFull = true;
}

void resetLunaHistory() {
  lunaHistIdx  = 0;
  lunaHistFull = false;
}

// Kapanma hızı oranına göre statik/dinamik karar
bool isDynamic() {
  if (!lunaHistFull) return false;
  int oldest = lunaHistIdx;  // en eski kayıt (dairesel buffer)
  float d0 = lunaHist[oldest];
  float d1 = lunaHist[(oldest + CLASSIFY_SAMPLES - 1) % CLASSIFY_SAMPLES];
  float dt  = (lunaHistMs[(oldest + CLASSIFY_SAMPLES - 1) % CLASSIFY_SAMPLES]
              - lunaHistMs[oldest]) / 1000.0f;
  if (dt < 0.05f) return false;
  float closingRate = (d0 - d1) / dt;
  float speedEst    = max((escUs - 1495) / 65.0f * 1.16f, 0.15f);
  return (closingRate / speedEst) < STATIC_THRESHOLD;
}

// RL stratejisini motora uygula
void applyRLStrategy() {
  int targetEsc = (rlStrategy == 0) ? ESC_MID_US : 1557;
  if (lunaDistCm > 0 && lunaDistCm < 40) targetEsc = ESC_MID_US;  // güvenlik
  int targetServo = SERVO_MID_US;
  if (rlStrategy == 2) targetServo = SERVO_MID_US - 80;  // sol
  if (rlStrategy == 3) targetServo = SERVO_MID_US + 80;  // sağ
  escUs   = targetEsc;
  servoUs = targetServo;
  esc.writeMicroseconds(escUs);
  steerServo.writeMicroseconds(servoUs);
  const char* names[] = {"TEMKIN","NORMAL","SOL GEC","SAG GEC"};
  lastAutoLog = String("RL:") + names[rlStrategy];
}

void handleSensors() {
  addCORS();
  char buf[96];
  snprintf(buf, sizeof(buf),
    "{\"luna\":%d,\"lstr\":%d,\"left\":%.1f,\"right\":%.1f,\"esc\":%d}",
    lunaDistCm, lunaStr, lastLeft, lastRight, escUs);
  server.send(200, "application/json", buf);
}

// GET /rl — mevcut RL durumunu döner (debug)
void handleRL() {
  addCORS();
  char buf[80];
  snprintf(buf, sizeof(buf),
    "{\"rlActive\":%s,\"strategy\":%d,\"luna\":%d}",
    rlActive ? "true" : "false", rlStrategy, lunaDistCm);
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
   server.on("/set",       HTTP_POST,    handleSet);
   server.on("/set",       HTTP_OPTIONS, handleOptions);
   server.on("/set-prox",  HTTP_POST,    handleSetProx);
   server.on("/set-prox",  HTTP_OPTIONS, handleOptions);
   server.on("/set-lidar", HTTP_POST,    handleSetLidar);
   server.on("/set-lidar", HTTP_OPTIONS, handleOptions);
   server.on("/mode",         HTTP_POST,    handleMode);
   server.on("/mode",         HTTP_OPTIONS, handleOptions);
   server.on("/auto-version", HTTP_POST,    handleAutoVersion);
   server.on("/sensors",      HTTP_GET,     handleSensors);
   server.on("/rl",           HTTP_GET,     handleRL);
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

   static unsigned long lastImuMs = 0;
   if (millis() - lastImuMs >= 10) {   // 100 Hz
     lastImuMs = millis();
     ImuData d = mpuRead();
     if (d.ok) { lastImu = d; cfUpdate(d); }
   }

   static unsigned long lastCtrlMs = 0;
   if (millis() - lastCtrlMs >= 50) {  // 20 Hz
     lastCtrlMs = millis();
     if (runMode == MODE_AUTO) {
       if (armState == DISARMED) {
         // DISARMED: her zaman neutral
         escUs   = ESC_MID_US;
         servoUs = SERVO_MID_US;
         esc.writeMicroseconds(ESC_MID_US);
         steerServo.writeMicroseconds(SERVO_MID_US);
       } else if (armState == EMERGENCY) {
         // ESC her zaman neutral
         escUs = ESC_MID_US;
         esc.writeMicroseconds(ESC_MID_US);
         // Servo: ilk 1s manevra için serbest, sonra neutral
         if (millis() - emergencyStartMs >= 1000) {
           servoUs = SERVO_MID_US;
           steerServo.writeMicroseconds(SERVO_MID_US);
         }
         // Wall recovery
         if (autoWallRecover && millis() - wallEmergencyMs >= 2000) {
           if (lunaDistCm > 50) {
             armState        = ARMED;
             autoWallRecover = false;
             lastAutoLog     = "Wall cleared — resumed";
             Serial.println("[AUTO] Wall cleared — re-ARMED");
           } else {
             wallEmergencyMs = millis();
           }
         }
       } else {
         // ARMED: single-car veya multi-car
         if (autoVersion == AUTO_MULTI) {
           if (lunaDistCm > 0 && lunaDistCm < OBSTACLE_RANGE_CM) {
             updateLunaHistory();
             if (isDynamic()) {
               float obs[7];
               obs[0] = max((escUs - 1495) / 65.0f * 1.16f, 0.0f);
               obs[1] = lunaDistCm / 100.0f;
               obs[2] = lastLeft  > 0 ? lastLeft  / 100.0f : 1.5f;
               obs[3] = lastRight > 0 ? lastRight / 100.0f : 1.5f;
               obs[4] = 0.0f;
               obs[5] = lastImu.ok ? lastImu.gz : 0.0f;
               obs[6] = 5.0f;
               float probs[NUM_STRATEGIES];
               policy_forward(obs, probs);
               rlStrategy = policy_argmax(probs);
               rlActive   = true;
             } else {
               rlActive = false;
             }
           } else {
             resetLunaHistory();
             rlActive = false;
           }
           if (rlActive) {
             applyRLStrategy();
           } else {
             autoControl();
           }
         } else {
           // AUTO_SINGLE: sadece orijinal autoControl
           rlActive = false;
           autoControl();
         }
       }
     } else {
       manualSafety();
     }
   }
 }