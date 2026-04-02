# Energy-Monitoring-System
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <math.h>

// === CONFIG ===
const char* ssid     = "hello";
const char* password = "test1234";

const float ENERGY_COST_PER_KWH = 6.0f; // ₹ per kWh (user requested ₹6)
const float PF_FIXED = 0.85f;           // Fixed power factor (change if you want)

// ---- RELAY 1 ----
const int RELAY1_PIN = 4;
const bool RELAY_ACTIVE_LOW = true;
bool relay1State = false;

// ---- RELAY 2 ----
const int RELAY2_PIN = 14;
bool relay2State = false;

// Voltage sensor (ZMPT101B)
const int ADC_PIN_V = 34;
const int ADC_SAMPLES = 350;
const float VREF = 3.3f;
const float ADC_MAX = 4095.0f;
float calibrationFactorV = 500.0f; // tuning factor - adjust for correct mains voltage

// Current sensor (ZMCT103C)
const int ADC_PIN_I = 35;
const float calibrationFactorI = 0.64f; // tuning - adjust to match real current

WebServer server(80);

// Measurement state
float mainsVrms = 0.0f;
float currentA = 0.0f;
float apparentVA = 0.0f;
float realW = 0.0f;
float powerFactor = PF_FIXED;
float freqHz = 0.0f;

unsigned long startMillis = 0;
unsigned long lastMeasureMillis = 0;
const unsigned long MEASURE_INTERVAL_MS = 1000; // measure every 1 second

double energy_Wh = 0.0; // energy accumulated since boot in Wh

// ---------------- helpers ----------------
void writeRelay(int relayNum, bool on) {
  int pin = (relayNum == 1) ? RELAY1_PIN : RELAY2_PIN;

  if (RELAY_ACTIVE_LOW) digitalWrite(pin, on ? LOW : HIGH);
  else digitalWrite(pin, on ? HIGH : LOW);

  if (relayNum == 1) relay1State = on;
  else relay2State = on;

  Serial.printf("[RELAY %d] %s (pin %d = %d)\n",
                relayNum, on ? "ON" : "OFF", pin, digitalRead(pin));
}

// compute Vrms from ADC pin using sampling (same helper used for voltage & current sensors)
float measureVrmsRaw(int adcPin) {
  analogReadResolution(12);
  // attenuation already set in setup, but safe to set here as well
  analogSetPinAttenuation(adcPin, ADC_11db);

  long acc = 0;
  for (int i = 0; i < ADC_SAMPLES; ++i) {
    acc += analogRead(adcPin);
    delayMicroseconds(200);
  }
  float avg = (float)acc / ADC_SAMPLES;

  double ms = 0.0;
  for (int i = 0; i < ADC_SAMPLES; ++i) {
    int sample = analogRead(adcPin);
    double v = ((double)sample - avg) * (VREF / ADC_MAX);
    ms += v * v;
    delayMicroseconds(200);
  }
  ms /= ADC_SAMPLES;
  return sqrt(ms);
}

float measureVoltageSensorVrms() { return measureVrmsRaw(ADC_PIN_V); }
float measureCurrentSensorVrms() { return measureVrmsRaw(ADC_PIN_I); }

// Measure mains frequency using zero crossing timestamps on voltage ADC
float measureFrequencyHz(unsigned long timeoutMs = 600) {
  // We'll detect positive-going zero crossings relative to the average
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN_V, ADC_11db);

  // First get midpoint (DC offset)
  const int warmSamples = 200;
  long acc = 0;
  for (int i = 0; i < warmSamples; ++i) {
    acc += analogRead(ADC_PIN_V);
    delayMicroseconds(200);
  }
  float midpoint = (float)acc / warmSamples;

  unsigned long start = millis();
  unsigned long endTime = start + timeoutMs;
  bool prevAbove = false;
  int crossings = 0;
  unsigned long firstCrossTime = 0;
  unsigned long lastCrossTime = 0;

  while (millis() < endTime) {
    int s = analogRead(ADC_PIN_V);
    bool above = (s > midpoint);
    if (above && !prevAbove) {
      // positive-going zero crossing
      unsigned long t = micros();
      if (crossings == 0) firstCrossTime = t;
      lastCrossTime = t;
      crossings++;
      // we'll wait a bit to avoid multiple triggers within the same crossing
      delayMicroseconds(3000); // ~3 ms debounce
    }
    prevAbove = above;
    // small delay to avoid hammering ADC
    delayMicroseconds(200);
  }

  if (crossings >= 2) {
    // number of full periods = crossings - 1
    unsigned long deltaMicro = lastCrossTime - firstCrossTime;
    double periodMicro = (double)deltaMicro / (double)(crossings - 1);
    if (periodMicro > 0.0) {
      double hz = 1e6 / periodMicro;
      return (float)hz;
    }
  }
  return 0.0f; // failed to measure
}

// Single measurement update that calculates all metrics and accumulates energy
void performMeasurement() {
  unsigned long now = millis();
  static unsigned long prevMillis = 0;
  unsigned long dtMs = (prevMillis == 0) ? MEASURE_INTERVAL_MS : (now - prevMillis);
  if (dtMs == 0) dtMs = MEASURE_INTERVAL_MS;

  // Measure RMS values
  float rawVrmsV = measureVoltageSensorVrms();
  mainsVrms = rawVrmsV * calibrationFactorV;

  float rawVrmsI = measureCurrentSensorVrms();
  currentA = rawVrmsI * calibrationFactorI;

  // apparent and real power
  apparentVA = mainsVrms * currentA;
  powerFactor = PF_FIXED; // fixed PF as requested
  realW = apparentVA * powerFactor;

  // frequency (attempt)
  float f = measureFrequencyHz(600);
  if (f > 0.0f) freqHz = f;

  // accumulate energy in Wh. realW is Watts. dtMs in ms -> hours = dtMs / 3600000.0
  double dtHours = (double)dtMs / 3600000.0;
  energy_Wh += (double)realW * dtHours;

  prevMillis = now;
}

// ---------------- Web UI ----------------
String htmlPage() {
  String s = "<!doctype html><html><head>"
  "<meta charset='UTF-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'/>";
  s += "<title>ESP32 Power Monitor</title>";
  s += "<style>";
  s += "html,body{height:100%;margin:0;font-family:Inter,Segoe UI,Arial;background:#000;color:#e6eef8}";
  s += ".wrap{max-width:1000px;margin:18px auto;padding:18px;}";
  s += "h1{font-size:1.6rem;margin:6px 0;color:#b8f3ff;letter-spacing:0.6px}";
  s += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:14px;margin-top:14px}";
  s += ".card{background:linear-gradient(180deg, rgba(255,255,255,0.02), rgba(255,255,255,0.01));border:1px solid rgba(255,255,255,0.04);padding:14px;border-radius:12px;box-shadow:0 6px 18px rgba(0,0,0,0.6)}";
  s += ".val{font-size:1.6rem;font-weight:700;color:#ffffff;margin-top:6px}";
  s += ".label{font-size:0.85rem;color:#9fb6c9}";
  s += ".small{font-size:0.85rem;color:#cfe8ff}";
  s += ".btn{display:inline-block;padding:10px 14px;border-radius:10px;margin:6px 4px;background:transparent;border:1px solid rgba(255,255,255,0.08);cursor:pointer;color:#bff;}";
  s += ".btn.on{background:linear-gradient(90deg,#00ffb2,#00d4ff);color:#001}";
  s += ".muted{color:#6e8aa0;font-size:0.9rem}";
  s += ".footer{margin-top:14px;color:#7ea3b8;font-size:0.85rem;text-align:center}";
  s += "</style>";
  s += "</head><body>";
  s += "<div class='wrap'>";
  s += "<h1>ESP32 — Power & Energy Monitor</h1>";

  // Relays row
  s += "<div class='grid'>";
  s += "<div class='card'><div class='label'>Relay 1</div><div style='margin-top:8px'><span id='r1state' class='val'>-</span></div>";
  s += "<div style='margin-top:10px'><button class='btn' onclick=\"fetch('/r1/on').then(()=>update())\">ON</button><button class='btn' onclick=\"fetch('/r1/off').then(()=>update())\">OFF</button></div></div>";

  s += "<div class='card'><div class='label'>Relay 2</div><div style='margin-top:8px'><span id='r2state' class='val'>-</span></div>";
  s += "<div style='margin-top:10px'><button class='btn' onclick=\"fetch('/r2/on').then(()=>update())\">ON</button><button class='btn' onclick=\"fetch('/r2/off').then(()=>update())\">OFF</button></div></div>";

  // Voltage
  s += "<div class='card'><div class='label'>Voltage (RMS)</div><div id='voltage' class='val'>-- V</div><div class='small'>Measured from ZMPT101B</div></div>";

  // Current
  s += "<div class='card'><div class='label'>Current (RMS)</div><div id='current' class='val'>-- A</div><div class='small'>Measured from ZMCT103C</div></div>";

  s += "</div>"; // grid

  // Second row: power, PF, freq
  s += "<div class='grid' style='margin-top:12px'>";
  s += "<div class='card'><div class='label'>Apparent Power</div><div id='va' class='val'>-- VA</div><div class='small'>V × I</div></div>";
  s += "<div class='card'><div class='label'>Real Power</div><div id='w' class='val'>-- W</div><div class='small'>Using fixed PF</div></div>";
  s += "<div class='card'><div class='label'>Power Factor</div><div id='pf' class='val'>--</div><div class='small'>Fixed at " + String(PF_FIXED, 2) + "</div></div>";
  s += "<div class='card'><div class='label'>Frequency</div><div id='hz' class='val'>-- Hz</div><div class='small'>Estimated by zero-cross</div></div>";
  s += "</div>";

  // Energy row
  s += "<div class='grid' style='margin-top:12px'>";
  s += "<div class='card'><div class='label'>Energy (since boot)</div><div id='energy' class='val'>-- Wh</div><div class='small'>Accumulated on ESP32</div></div>";
  s += "<div class='card'><div class='label'>Cost (since boot)</div><div id='cost' class='val'>-- ₹</div><div class='small'>At ₹ " + String(ENERGY_COST_PER_KWH, 2) + " / kWh</div></div>";
  s += "<div class='card'><div class='label'>Monthly Energy (est.)</div><div id='month_kwh' class='val'>-- kWh</div><div class='small'>Extrapolated at current power</div></div>";
  s += "<div class='card'><div class='label'>Monthly Cost (est.)</div><div id='month_cost' class='val'>-- ₹</div><div class='small'>Extrapolated at current power</div></div>";
  s += "</div>";

  s += "<div class='footer'>Tip: Calibration factors (voltage/current) are editable in code. Adjust them for accurate readings.</div>";
  s += "</div>";

  // JS: periodic fetch /status
  s += "<script>";
  s += "async function update(){ try{ const r = await fetch('/status'); const j = await r.json();";
  s += " document.getElementById('r1state').innerText = j.relay1 ? 'ON' : 'OFF';";
  s += " document.getElementById('r2state').innerText = j.relay2 ? 'ON' : 'OFF';";
  s += " document.getElementById('voltage').innerText = j.mains_v.toFixed(1) + ' V';";
  s += " document.getElementById('current').innerText = j.current_a.toFixed(3) + ' A';";
  s += " document.getElementById('va').innerText = j.apparent_va.toFixed(1) + ' VA';";
  s += " document.getElementById('w').innerText = j.real_w.toFixed(1) + ' W';";
  s += " document.getElementById('pf').innerText = j.power_factor.toFixed(2);";
  s += " document.getElementById('hz').innerText = j.frequency_hz>0 ? j.frequency_hz.toFixed(1) + ' Hz' : '--';";
  s += " document.getElementById('energy').innerText = j.energy_wh.toFixed(3) + ' Wh';";
  s += " document.getElementById('cost').innerText = '₹ ' + j.energy_cost.toFixed(2);";
  s += " document.getElementById('month_kwh').innerText = j.monthly_kwh.toFixed(3) + ' kWh';";
  s += " document.getElementById('month_cost').innerText = '₹ ' + j.monthly_cost.toFixed(2);";
  s += " }catch(e){ console.log('update err', e); } }";
  s += "update(); setInterval(update,1000);";
  s += "</script>";

  s += "</body></html>";
  return s;
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

// RELAY 1 HANDLERS
void handleR1On() { writeRelay(1, true); server.sendHeader("Location","/"); server.send(303); }
void handleR1Off() { writeRelay(1, false); server.sendHeader("Location","/"); server.send(303); }

// RELAY 2 HANDLERS
void handleR2On() { writeRelay(2, true); server.sendHeader("Location","/"); server.send(303); }
void handleR2Off() { writeRelay(2, false); server.sendHeader("Location","/"); server.send(303); }

// status JSON API used by the web UI
void handleStatus() {
  // Prepare JSON
  float monthly_kwh = 0.0f;
  float monthly_cost = 0.0f;

  // Extrapolate monthly energy from current real power (W)
  // monthly_kwh = (realW * 24 * 30) / 1000
  monthly_kwh = (realW * 24.0f * 30.0f) / 1000.0f;
  monthly_cost = monthly_kwh * ENERGY_COST_PER_KWH;

  // cost since boot
  double energy_kwh_since_boot = energy_Wh / 1000.0;
  double energy_cost_since_boot = energy_kwh_since_boot * (double)ENERGY_COST_PER_KWH;

  String js = "{";
  js += "\"mains_v\":" + String(mainsVrms, 3);
  js += ",\"current_a\":" + String(currentA, 6);
  js += ",\"apparent_va\":" + String(apparentVA, 3);
  js += ",\"real_w\":" + String(realW, 3);
  js += ",\"power_factor\":" + String(powerFactor, 3);
  js += ",\"frequency_hz\":" + String(freqHz, 3);
  js += ",\"energy_wh\":" + String(energy_Wh, 6);
  js += ",\"energy_cost\":" + String(energy_cost_since_boot, 3);
  js += ",\"monthly_kwh\":" + String(monthly_kwh, 6);
  js += ",\"monthly_cost\":" + String(monthly_cost, 3);
  js += ",\"relay1\":" + String(relay1State ? "true" : "false");
  js += ",\"relay2\":" + String(relay2State ? "true" : "false");
  js += "}";
  server.send(200, "application/json", js);
}

void handleNotFound() {
  server.send(404, "text/plain", "404: Not found");
}

void setup() {
  Serial.begin(115200);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  writeRelay(1, false);
  writeRelay(2, false);

  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN_V, ADC_11db);
  analogSetPinAttenuation(ADC_PIN_I, ADC_11db);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);

  server.on("/r1/on", handleR1On);
  server.on("/r1/off", handleR1Off);

  server.on("/r2/on", handleR2On);
  server.on("/r2/off", handleR2Off);

  server.on("/status", handleStatus);
  server.onNotFound(handleNotFound);

  server.begin();

  startMillis = millis();
  lastMeasureMillis = 0;

  // perform an initial measurement
  performMeasurement();
}

void loop() {
  server.handleClient();

  unsigned long now = millis();
  if (now - lastMeasureMillis >= MEASURE_INTERVAL_MS) {
    // Update measurements and energy accumulation
    performMeasurement();
    lastMeasureMillis = now;
    // Debug print
    Serial.printf("V=%.1fV I=%.3fA VA=%.1fW real=%.1fW PF=%.2f F=%.1fHz E=%.4fWh\n",
                  mainsVrms, currentA, apparentVA, realW, powerFactor, freqHz, energy_Wh);
  }
}
