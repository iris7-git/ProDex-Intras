#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>

// ---------------------------------------------------------
// HARDWARE SETTINGS
// ---------------------------------------------------------
#define MOTOR_PIN 15

// ---------------------------------------------------------
// WIFI SETTINGS
// Set these to your phone hotspot credentials.
// After boot, Serial prints the IP — open it in any browser.
// ---------------------------------------------------------
const char* WIFI_SSID     = "IQOO Z10X";
const char* WIFI_PASSWORD = "iznhkdj94reqtzm";

// ---------------------------------------------------------
// CALIBRATION SETTINGS
// ---------------------------------------------------------
#define CALIB_DURATION_MS        3000
#define CALIB_SAMPLE_INTERVAL_MS 10

// ---------------------------------------------------------
// THRESHOLDS — 3 zones per metric
// YELLOW = warning (approaching danger)
// RED    = alert (dangerous, haptic fires)
// ---------------------------------------------------------

// Knee Flexion/Extension
const float HYPEREXTENSION_RED    = -5.0;   // degrees
const float FLEXION_YELLOW        = 110.0;  // degrees
const float FLEXION_RED           = 130.0;  // degrees

// Valgus Wobble
const float WOBBLE_YELLOW         = 10.0;   // degrees
const float WOBBLE_RED            = 15.0;   // degrees

// Tibial Twist
const float TWIST_YELLOW          = 12.0;   // degrees
const float TWIST_RED             = 20.0;   // degrees

// Hard Landing
const float IMPACT_RED            = 29.4;   // m/s² net (~3G)

// Combined mechanics — all three at medium threshold = critical
const float COMBO_ANGLE_THRESH    = 20.0;
const float COMBO_WOBBLE_THRESH   = 8.0;
const float COMBO_TWIST_THRESH    = 10.0;

// ---------------------------------------------------------
// SIDE-MOUNT AXIS NOTE
// MPUs mounted on the SIDE of thigh and shin:
//   Knee Flexion/Extension -> gyro.y / accel Y-axis
//   Valgus/Varus Wobble    -> gyro.x / accel X-axis
//   Tibial Twist (Yaw)     -> gyro.z
// Swap gyro.x <-> gyro.y if physical test shows mismatch.
// ---------------------------------------------------------

// ---------------------------------------------------------
// WiFi / WEB SERVER GLOBALS
// ---------------------------------------------------------
WebServer server(80);

// Simple ring buffer to store last 30 alert events
// Each entry is a plain string — readable in the browser
#define MAX_LOG_ENTRIES 30
String  alertLog[MAX_LOG_ENTRIES];
int     logHead    = 0;   // points to next write position
int     logCount   = 0;   // total entries written so far
unsigned long alertCounter = 0;

// ---------------------------------------------------------
// IMU & FILTER GLOBALS
// ---------------------------------------------------------
Adafruit_MPU6050 thighIMU;
Adafruit_MPU6050 calfIMU;

unsigned long lastTime = 0;
float alpha = 0.98;

float thighPitch = 0, thighRoll = 0;
float calfPitch  = 0, calfRoll  = 0;
float relTwist   = 0;

float biasPitchThigh = 0, biasRollThigh = 0;
float biasPitchCalf  = 0, biasRollCalf  = 0;
float biasGyroZ      = 0;

// ---------------------------------------------------------
// FUNCTION DECLARATIONS
// ---------------------------------------------------------
void triggerHaptic(int pattern);
void calibrate();
void setupWiFi();
void setupWebServer();
void logAlert(const char* zone, const char* alertType,
              float angle, float wobble,
              float twist, float impact);

// ---------------------------------------------------------
// WEB PAGE
// Auto-refreshes every 3 seconds.
// Shows all stored alert events in a clean table.
// ---------------------------------------------------------
void handleRoot() {
  String html = R"rawhtml(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'>
<meta http-equiv='refresh' content='3'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<title>ACL Brace Monitor</title>
<style>
  body { font-family: monospace; background: #0d0d0d; color: #e0e0e0; padding: 16px; }
  h2   { color: #00e5ff; margin-bottom: 4px; }
  p    { color: #888; font-size: 12px; margin-top: 0; }
  table { width: 100%; border-collapse: collapse; font-size: 13px; }
  th   { background: #1a1a1a; color: #00e5ff; padding: 8px; text-align: left; }
  td   { padding: 7px 8px; border-bottom: 1px solid #222; }
  .CRITICAL { color: #ff1744; font-weight: bold; }
  .RED      { color: #ff6d00; }
  .YELLOW   { color: #ffd600; }
  .empty    { color: #555; text-align: center; padding: 24px; }
</style>
</head><body>
<h2>ACL Brace — Alert Log</h2>
<p>Auto-refreshes every 3 seconds &nbsp;|&nbsp; )rawhtml";

  html += String(logCount) + " event(s) recorded</p>";
  html += "<table><tr><th>#</th><th>Zone</th><th>Type</th>"
          "<th>Angle</th><th>Wobble</th><th>Twist</th><th>Impact</th><th>Time(s)</th></tr>";

  if (logCount == 0) {
    html += "<tr><td colspan='8' class='empty'>No alerts yet — system monitoring...</td></tr>";
  } else {
    // Print newest first
    int entries = min(logCount, MAX_LOG_ENTRIES);
    for (int i = 0; i < entries; i++) {
      int idx = (logHead - 1 - i + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
      html += alertLog[idx];
    }
  }

  html += "</table></body></html>";
  server.send(200, "text/html", html);
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  Serial.println("\n--- Smart ACL Brace: Full Kinematics + WiFi Mode ---");

  // --- Step 1: IMUs ---
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(150);

  Serial.println("Checking Thigh Sensor (0x68)...");
  if (!thighIMU.begin(0x68)) {
    Serial.println("CRITICAL ERROR: Thigh MPU6050 not found. HALTING.");
    while (true) { delay(1000); }
  }
  Serial.println("Thigh Sensor OK!");

  Serial.println("Checking Calf Sensor (0x69)...");
  if (!calfIMU.begin(0x69)) {
    Serial.println("CRITICAL ERROR: Calf MPU6050 not found. Is AD0 on 3.3V? HALTING.");
    while (true) { delay(1000); }
  }
  Serial.println("Calf Sensor OK!");

  thighIMU.setAccelerometerRange(MPU6050_RANGE_8_G);
  thighIMU.setGyroRange(MPU6050_RANGE_500_DEG);
  calfIMU.setAccelerometerRange(MPU6050_RANGE_8_G);
  calfIMU.setGyroRange(MPU6050_RANGE_500_DEG);

  // --- Step 2: WiFi + Web Server ---
  setupWiFi();
  setupWebServer();

  // --- Step 3: Calibrate ---
  triggerHaptic(5); // 2 short buzzes: "hold still"

  Serial.print("Calibrating for ");
  Serial.print(CALIB_DURATION_MS / 1000);
  Serial.println(" seconds... Hold still in neutral stance.");

  calibrate();

  triggerHaptic(1); // 1 long buzz: "done, go"
  Serial.println("System Running.");
  Serial.println("----------------------------------------------------");

  lastTime = millis();
}

// ---------------------------------------------------------
// WIFI SETUP
// ---------------------------------------------------------
void setupWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.print(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] Open this in your browser: http://");
    Serial.println(WiFi.localIP());
  } else {
    // WiFi failed — system still works, just no remote logging
    Serial.println("\n[WiFi] Could not connect. Running Serial-only mode.");
  }
}

// ---------------------------------------------------------
// WEB SERVER SETUP
// ---------------------------------------------------------
void setupWebServer() {
  server.on("/", handleRoot);
  server.begin();
  Serial.println("[Web] Server started on port 80.");
}

// ---------------------------------------------------------
// LOG ALERT
// Stores event in ring buffer AND prints to Serial.
// Zone is "YELLOW" or "RED" or "CRITICAL".
// ---------------------------------------------------------
void logAlert(const char* zone, const char* alertType,
              float angle, float wobble,
              float twist, float impact) {
  alertCounter++;
  float timestamp = millis() / 1000.0;

  // Serial log
  Serial.printf("[%s] #%lu %s | Angle:%.1f Wobble:%.1f Twist:%.1f Impact:%.1f t=%.1fs\n",
    zone, alertCounter, alertType, angle, wobble, twist, impact, timestamp);

  // Build HTML table row for web page
  String row = "<tr class='";
  row += zone;
  row += "'><td>";
  row += alertCounter;
  row += "</td><td>";
  row += zone;
  row += "</td><td>";
  row += alertType;
  row += "</td><td>";
  row += String(angle, 1);
  row += "°</td><td>";
  row += String(wobble, 1);
  row += "°</td><td>";
  row += String(twist, 1);
  row += "°</td><td>";
  row += String(impact, 1);
  row += " m/s²</td><td>";
  row += String(timestamp, 1);
  row += "s</td></tr>";

  // Write to ring buffer
  alertLog[logHead] = row;
  logHead = (logHead + 1) % MAX_LOG_ENTRIES;
  logCount++;
}

// ---------------------------------------------------------
// CALIBRATION
// ---------------------------------------------------------
void calibrate() {
  float accumPitchThigh = 0, accumRollThigh = 0;
  float accumPitchCalf  = 0, accumRollCalf  = 0;
  float accumGyroZ      = 0;
  int   samples         = 0;

  unsigned long calibStart = millis();

  while (millis() - calibStart < CALIB_DURATION_MS) {
    sensors_event_t t_a, t_g, t_temp;
    sensors_event_t c_a, c_g, c_temp;
    thighIMU.getEvent(&t_a, &t_g, &t_temp);
    calfIMU.getEvent(&c_a, &c_g, &c_temp);

    accumPitchThigh += atan2(t_a.acceleration.y, sqrt(pow(t_a.acceleration.x, 2) + pow(t_a.acceleration.z, 2))) * 180.0 / PI;
    accumRollThigh  += atan2(-t_a.acceleration.x, t_a.acceleration.z) * 180.0 / PI;
    accumPitchCalf  += atan2(c_a.acceleration.y, sqrt(pow(c_a.acceleration.x, 2) + pow(c_a.acceleration.z, 2))) * 180.0 / PI;
    accumRollCalf   += atan2(-c_a.acceleration.x, c_a.acceleration.z) * 180.0 / PI;
    accumGyroZ      += (c_g.gyro.z - t_g.gyro.z) * 180.0 / PI;

    samples++;
    delay(CALIB_SAMPLE_INTERVAL_MS);
  }

  biasPitchThigh = accumPitchThigh / samples;
  biasRollThigh  = accumRollThigh  / samples;
  biasPitchCalf  = accumPitchCalf  / samples;
  biasRollCalf   = accumRollCalf   / samples;
  biasGyroZ      = accumGyroZ      / samples;

  thighPitch = biasPitchThigh;
  thighRoll  = biasRollThigh;
  calfPitch  = biasPitchCalf;
  calfRoll   = biasRollCalf;
  relTwist   = 0;

  Serial.print("Bias | thighPitch: "); Serial.print(biasPitchThigh, 2);
  Serial.print("  calfPitch: ");       Serial.print(biasPitchCalf,  2);
  Serial.print("  gyroZ-diff: ");      Serial.print(biasGyroZ,      3);
  Serial.println(" deg/s");

  lastTime = millis();
}

// ---------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------
void loop() {
  // Handle incoming web requests (non-blocking)
  server.handleClient();

  // 1. Time Delta
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // 2. Read both sensors
  sensors_event_t t_a, t_g, t_temp;
  sensors_event_t c_a, c_g, c_temp;
  thighIMU.getEvent(&t_a, &t_g, &t_temp);
  calfIMU.getEvent(&c_a, &c_g, &c_temp);

  // 3. Accelerometer tilt (side-mount: Y-axis = knee flexion)
  float t_accelPitch = atan2(t_a.acceleration.y, sqrt(pow(t_a.acceleration.x, 2) + pow(t_a.acceleration.z, 2))) * 180.0 / PI;
  float t_accelRoll  = atan2(-t_a.acceleration.x, t_a.acceleration.z) * 180.0 / PI;
  float c_accelPitch = atan2(c_a.acceleration.y, sqrt(pow(c_a.acceleration.x, 2) + pow(c_a.acceleration.z, 2))) * 180.0 / PI;
  float c_accelRoll  = atan2(-c_a.acceleration.x, c_a.acceleration.z) * 180.0 / PI;

  // 4. Complementary Filter
  thighPitch = alpha * (thighPitch + (t_g.gyro.y * 180.0 / PI) * dt) + (1 - alpha) * t_accelPitch;
  thighRoll  = alpha * (thighRoll  + (t_g.gyro.x * 180.0 / PI) * dt) + (1 - alpha) * t_accelRoll;
  calfPitch  = alpha * (calfPitch  + (c_g.gyro.y * 180.0 / PI) * dt) + (1 - alpha) * c_accelPitch;
  calfRoll   = alpha * (calfRoll   + (c_g.gyro.x * 180.0 / PI) * dt) + (1 - alpha) * c_accelRoll;

  // 5. Relative Tibial Twist — bias-corrected
  float gyroZDelta = ((c_g.gyro.z - t_g.gyro.z) * 180.0 / PI) - biasGyroZ;
  relTwist += gyroZDelta * dt;

  // 6. Kinematic Metrics — relative to calibrated neutral
  float kneeAngle   = (thighPitch - biasPitchThigh) - (calfPitch - biasPitchCalf);
  float wobbleIndex = abs((thighRoll - biasRollThigh) - (calfRoll - biasRollCalf));
  float twistIndex  = abs(relTwist);
  float verticalImpact = abs(c_a.acceleration.z) - 9.8;

  // 7. Danger Logic — CRITICAL > RED > YELLOW, ordered by severity
  bool danger = false;

  // --- CRITICAL: All three mechanisms at once ---
  if (kneeAngle   > COMBO_ANGLE_THRESH &&
      wobbleIndex > COMBO_WOBBLE_THRESH &&
      twistIndex  > COMBO_TWIST_THRESH) {
    Serial.print(">>> CRITICAL: Combined ACL Mechanism! <<<  ");
    logAlert("CRITICAL", "COMBINED", kneeAngle, wobbleIndex, twistIndex, verticalImpact);
    triggerHaptic(4);
    danger = true;
  }

  // --- RED ZONE alerts (haptic fires) ---
  else if (verticalImpact > IMPACT_RED && kneeAngle > 5.0) {
    Serial.print(">>> RED: Hard Landing! <<<  ");
    logAlert("RED", "HARD_LANDING", kneeAngle, wobbleIndex, twistIndex, verticalImpact);
    triggerHaptic(1);
    danger = true;
  }
  else if (kneeAngle < HYPEREXTENSION_RED) {
    Serial.print(">>> RED: Hyperextension! <<<  ");
    logAlert("RED", "HYPEREXTENSION", kneeAngle, wobbleIndex, twistIndex, verticalImpact);
    triggerHaptic(1);
    danger = true;
  }
  else if (kneeAngle > FLEXION_RED) {
    Serial.print(">>> RED: Deep Flexion! <<<  ");
    logAlert("RED", "DEEP_FLEXION", kneeAngle, wobbleIndex, twistIndex, verticalImpact);
    triggerHaptic(2);
    danger = true;
  }
  else if (wobbleIndex > WOBBLE_RED) {
    Serial.print(">>> RED: Valgus Wobble! <<<  ");
    logAlert("RED", "VALGUS_WOBBLE", kneeAngle, wobbleIndex, twistIndex, verticalImpact);
    triggerHaptic(3);
    danger = true;
  }
  else if (twistIndex > TWIST_RED) {
    Serial.print(">>> RED: Tibial Twist! <<<  ");
    logAlert("RED", "TIBIAL_TWIST", kneeAngle, wobbleIndex, twistIndex, verticalImpact);
    triggerHaptic(3);
    danger = true;
  }

  // --- YELLOW ZONE warnings (logged only, no haptic) ---
  else if (kneeAngle > FLEXION_YELLOW) {
    Serial.print(">>> YELLOW: Approaching Deep Flexion <<<  ");
    logAlert("YELLOW", "FLEXION_WARNING", kneeAngle, wobbleIndex, twistIndex, verticalImpact);
    // No haptic — yellow is a caution, not an emergency
  }
  else if (wobbleIndex > WOBBLE_YELLOW) {
    Serial.print(">>> YELLOW: Wobble Warning <<<  ");
    logAlert("YELLOW", "WOBBLE_WARNING", kneeAngle, wobbleIndex, twistIndex, verticalImpact);
  }
  else if (twistIndex > TWIST_YELLOW) {
    Serial.print(">>> YELLOW: Twist Warning <<<  ");
    logAlert("YELLOW", "TWIST_WARNING", kneeAngle, wobbleIndex, twistIndex, verticalImpact);
  }

  // 8. Live Serial Dashboard
  Serial.print("Angle: ");     Serial.print(kneeAngle,      1); Serial.print("deg");
  Serial.print(" | Wobble: "); Serial.print(wobbleIndex,    1); Serial.print("deg");
  Serial.print(" | Twist: ");  Serial.print(twistIndex,     1); Serial.print("deg");
  Serial.print(" | Impact: "); Serial.print(verticalImpact, 1); Serial.println(" m/s2");

  if (danger) { lastTime = millis(); }

  delay(20); // ~50Hz
}

// ---------------------------------------------------------
// HAPTIC FEEDBACK PATTERNS
// Yellow zone has no haptic — it is a log-only caution.
// ---------------------------------------------------------
void triggerHaptic(int pattern) {
  switch (pattern) {

    case 1: // Long single pulse — Hyperextension / Hard Landing
      digitalWrite(MOTOR_PIN, HIGH); delay(500);
      digitalWrite(MOTOR_PIN, LOW);
      break;

    case 2: // Double pulse — Deep Flexion
      digitalWrite(MOTOR_PIN, HIGH); delay(100);
      digitalWrite(MOTOR_PIN, LOW);  delay(100);
      digitalWrite(MOTOR_PIN, HIGH); delay(100);
      digitalWrite(MOTOR_PIN, LOW);
      break;

    case 3: // 3 rapid buzzes — Valgus / Twist
      for (int i = 0; i < 3; i++) {
        digitalWrite(MOTOR_PIN, HIGH); delay(50);
        digitalWrite(MOTOR_PIN, LOW);  delay(50);
      }
      break;

    case 4: // 5 urgent buzzes — Combined Critical
      for (int i = 0; i < 5; i++) {
        digitalWrite(MOTOR_PIN, HIGH); delay(80);
        digitalWrite(MOTOR_PIN, LOW);  delay(40);
      }
      break;

    case 5: // 2 pre-calibration warning buzzes
      for (int i = 0; i < 2; i++) {
        digitalWrite(MOTOR_PIN, HIGH); delay(150);
        digitalWrite(MOTOR_PIN, LOW);  delay(150);
      }
      break;
  }
}