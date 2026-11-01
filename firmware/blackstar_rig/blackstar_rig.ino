// BLACKSTAR RIG: automated thrust sequence + HX711 logging, ESP32, Arduino IDE.
// Libraries: ESP32Servo (Library Manager). Everything else is built in.
//
// Wiring:  ESC1 signal -> GPIO18     ESC2 signal -> GPIO19     ESC grounds -> ESP32 GND
//          HX711 DT -> GPIO32        HX711 SCK -> GPIO33       HX711 VCC -> 3V3, GND -> GND
// Use:     flash, join WiFi "Blackstar" (password below), open http://192.168.4.1
//
// SIGN: tare with the model resting and flow off, then lift UNLOADS the cell. Calibration mass
// (coins on top) LOADS it. So lift and cal mass move opposite ways. The page reports
//   lift_g = (zeroRaw - raw) / calFactor
// which is POSITIVE for lift given coins-on-top calibration. If it ever reads the wrong way,
// recalibrate -- the signed factor absorbs a flipped A+/A-.
//
// The board runs the whole sequence by itself (baseline, M1 climb, settle, M2 climb, hold,
// ramp down, final baseline), samples the HX711 into RAM, and hands the data to the page.
// The page stores every run in the browser (IndexedDB) and exports CSV.

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Preferences.h>

const char* AP_SSID = "Blackstar";
const char* AP_PASS = "blackstar123";   // min 8 chars

const int PIN_M1 = 18;
const int PIN_M2 = 19;
const int PIN_HX_DOUT = 32;
const int PIN_HX_SCK = 33;

const int US_IDLE = 1000;
const int US_FULL = 2000;
const int US_SOFT_CAP = 1600;           // hard ceiling on output pulse
const uint32_t ARM_MS = 3000;
const uint32_t FAILSAFE_MS = 1000;      // manual mode: page silent this long -> idle
const uint32_t FAILSAFE_RUN_MS = 3000;  // during a run: page silent this long -> abort

// Ramp rates in us/s (1% throttle = 10 us). Set from the page.
float RAMP_UP_US_PER_S = 50.0f;         // 5 %/s
float RAMP_DOWN_US_PER_S = 1000.0f;     // 100 %/s

Servo esc1, esc2;
WebServer server(80);
Preferences prefs;
portMUX_TYPE hxMux = portMUX_INITIALIZER_UNLOCKED;

// ---------- shared state ----------
volatile int cmd1 = US_IDLE, cmd2 = US_IDLE;        // targets
volatile float out1 = US_IDLE, out2 = US_IDLE;      // what the ESCs get
volatile uint32_t lastBeatMs = 0;
volatile uint32_t srvBusyStart = 0;                 // nonzero while handleClient() is running
volatile bool armed = false;
volatile uint16_t failsafeTrips = 0;

enum { PH_IDLE = 0, PH_PRE, PH_M1, PH_SETTLE, PH_M2, PH_HOLD, PH_DOWN, PH_POST, PH_DONE, PH_ABORT };
volatile uint8_t phase = PH_IDLE;
volatile uint32_t phaseStartMs = 0, runStartMs = 0;
volatile uint32_t preMs = 3000, settleMs = 5000, holdMs = 5000, postMs = 3000;
volatile int runCmd1 = US_IDLE, runCmd2 = US_IDLE;
volatile uint8_t abortReason = 0;                   // 1 link lost, 2 buffer full, 3 stopped, 4 timeout

struct Sample {
  uint32_t t;      // ms since run start
  int32_t raw;     // HX711 counts
  uint16_t m1, m2; // actual output pulse, us
  uint8_t ph;
  uint8_t pad[3];
};
const uint32_t SAMPLE_CAP = 4000;
Sample* samples = nullptr;
volatile uint32_t nSamples = 0;
volatile bool recording = false;
volatile bool bufFull = false;

volatile int32_t lastRaw = 0;
volatile uint32_t lastHxMs = 0;
float zeroRaw = 0.0f;
volatile bool tared = false;                        // Tare done since boot (Calibrate needs it)
float calFactor = 1.0f;                             // counts per gram (signed)

volatile uint8_t avgMode = 0;                       // 0 none, 1 tare, 2 calibrate
volatile uint8_t avgResult = 0;                     // 0 none, 1 ok, 2 failed
double avgSum = 0;
uint32_t avgCnt = 0, avgUntilMs = 0;
float calMass = 0.0f;

// ---------- helpers ----------
static inline int32_t ageMs(uint32_t t) { return (int32_t)(millis() - t); }
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

int pctToUs(float pct) {
  pct = clampf(pct, 0.0f, 100.0f);
  int us = US_IDLE + (int)(pct * (US_FULL - US_IDLE) / 100.0f + 0.5f);
  return us > US_SOFT_CAP ? US_SOFT_CAP : us;
}

float slew(float cur, float target, float dt) {
  float step = (target > cur ? RAMP_UP_US_PER_S : RAMP_DOWN_US_PER_S) * dt;
  if (fabsf(target - cur) <= step) return target;
  return cur + (target > cur ? step : -step);
}

int32_t hxRead() {                                  // call only when DOUT is low
  uint32_t v = 0;
  portENTER_CRITICAL(&hxMux);
  for (int i = 0; i < 24; i++) {
    digitalWrite(PIN_HX_SCK, HIGH);
    delayMicroseconds(1);
    v = (v << 1) | (uint32_t)digitalRead(PIN_HX_DOUT);
    digitalWrite(PIN_HX_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(PIN_HX_SCK, HIGH);                   // 25th pulse: channel A, gain 128
  delayMicroseconds(1);
  digitalWrite(PIN_HX_SCK, LOW);
  portEXIT_CRITICAL(&hxMux);
  if (v & 0x800000UL) v |= 0xFF000000UL;
  return (int32_t)v;
}

void finishAvg() {
  float mean = (float)(avgSum / (double)avgCnt);
  if (avgMode == 1) {
    zeroRaw = mean;
    tared = true;
    avgResult = 1;
  } else if (avgMode == 2) {
    float f = (mean - zeroRaw) / calMass;
    if (calMass > 0.0f && fabsf(f) > 1.0f) {
      calFactor = f;
      prefs.putFloat("cal", f);
      avgResult = 1;
    } else {
      avgResult = 2;
    }
  }
  avgMode = 0;
}

void onSample(int32_t r, uint32_t now) {
  lastRaw = r;
  lastHxMs = now;
  if (avgMode) {
    avgSum += r;
    avgCnt++;
    if ((int32_t)(now - avgUntilMs) >= 0 && avgCnt >= 3) finishAvg();
  }
  if (recording) {
    if (nSamples < SAMPLE_CAP) {
      int32_t t = (int32_t)(now - runStartMs);
      if (t < 0) t = 0;
      Sample& s = samples[nSamples];
      s.t = (uint32_t)t;
      s.raw = r;
      s.m1 = (uint16_t)(out1 + 0.5f);
      s.m2 = (uint16_t)(out2 + 0.5f);
      s.ph = phase;
      nSamples++;
    } else {
      bufFull = true;
    }
  }
}

void enterPhase(uint8_t p, uint32_t now) {
  switch (p) {
    case PH_PRE:  cmd1 = US_IDLE; cmd2 = US_IDLE; break;
    case PH_M1:   cmd1 = runCmd1; break;
    case PH_M2:   cmd2 = runCmd2; break;
    case PH_DOWN: cmd1 = US_IDLE; cmd2 = US_IDLE; break;
    case PH_DONE: recording = false; break;
    default: break;
  }
  phaseStartMs = now;
  phase = p;
}

void abortRun(uint8_t reason) {
  phase = PH_ABORT;
  recording = false;
  abortReason = reason;
  cmd1 = US_IDLE;
  cmd2 = US_IDLE;
}

void runStep(uint32_t now) {
  uint8_t ph = phase;
  if (ph < PH_PRE || ph > PH_POST) return;
  int32_t el = (int32_t)(now - phaseStartMs);
  if (el < 0) el = 0;
  int32_t total = (int32_t)(now - runStartMs);
  if (bufFull) { abortRun(2); return; }
  if (total > 120000) { abortRun(4); return; }

  switch (ph) {
    case PH_PRE:
      if ((uint32_t)el >= preMs) enterPhase(PH_M1, now);
      break;
    case PH_M1:
      if (fabsf(out1 - (float)runCmd1) <= 1.0f) enterPhase(PH_SETTLE, now);
      break;
    case PH_SETTLE:
      if ((uint32_t)el >= settleMs) enterPhase(PH_M2, now);
      break;
    case PH_M2:
      if (fabsf(out2 - (float)runCmd2) <= 1.0f) enterPhase(PH_HOLD, now);
      break;
    case PH_HOLD:
      if ((uint32_t)el >= holdMs) enterPhase(PH_DOWN, now);
      break;
    case PH_DOWN:
      if (out1 <= US_IDLE + 1 && out2 <= US_IDLE + 1) enterPhase(PH_POST, now);
      break;
    case PH_POST:
      if ((uint32_t)el >= postMs) enterPhase(PH_DONE, now);
      break;
  }
}

// ---------- control task: HX711 sampling, run state machine, motor output ----------
void controlTask(void*) {
  uint32_t lastUs = micros();
  bool inFs = false;
  for (;;) {
    uint32_t nowUs = micros();
    float dt = (float)(nowUs - lastUs) / 1e6f;
    lastUs = nowUs;
    if (dt > 0.05f) dt = 0.05f;
    uint32_t now = millis();

    if (!armed && now >= ARM_MS) {
      armed = true;
      lastBeatMs = now;
      Serial.println("ARMED");
    }

    if (digitalRead(PIN_HX_DOUT) == LOW) {
      int32_t r = hxRead();
      onSample(r, now);
    }

    bool running = (phase >= PH_PRE && phase <= PH_POST);
    int32_t age = (int32_t)(now - lastBeatMs);
    uint32_t busy0 = srvBusyStart;
    bool serverStalled = (busy0 != 0 && (int32_t)(now - busy0) > 200);   // web server blocked, not a lost link
    bool hbLost = armed && !serverStalled && age > (int32_t)(running ? FAILSAFE_RUN_MS : FAILSAFE_MS);

    if (!armed) {
      cmd1 = US_IDLE; cmd2 = US_IDLE;
      out1 = US_IDLE; out2 = US_IDLE;
    } else if (hbLost) {
      if (!inFs) {
        inFs = true;
        if (running || cmd1 > US_IDLE || cmd2 > US_IDLE) {
          failsafeTrips++;
          Serial.println("FAILSAFE: heartbeat lost");
        }
      }
      if (running) abortRun(1);
      cmd1 = US_IDLE; cmd2 = US_IDLE;
      out1 = US_IDLE; out2 = US_IDLE;
    } else {
      inFs = false;
      runStep(now);
      out1 = slew(out1, (float)cmd1, dt);
      out2 = slew(out2, (float)cmd2, dt);
    }

    esc1.writeMicroseconds((int)(out1 + 0.5f));
    esc2.writeMicroseconds((int)(out2 + 0.5f));
    vTaskDelay(1);
  }
}

// ---------- web ----------
const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BLACKSTAR TERMINAL</title>
<style>
:root{--bg:#070b07;--pan:#0c120c;--ln:#1c2a1c;--fg:#4fe08a;--dim:#2f7a4e;--amb:#f4c048;--red:#ff4d4d;--blu:#58b0ff}
*{box-sizing:border-box}
html,body{margin:0}
body{font-family:'Courier New',ui-monospace,monospace;background:var(--bg);color:var(--fg);
  max-width:820px;margin:0 auto;padding:8px 12px 70px;font-size:15px;line-height:1.35;
  text-shadow:0 0 1px rgba(79,224,138,.35)}
body:before{content:"";position:fixed;inset:0;pointer-events:none;z-index:99;
  background:repeating-linear-gradient(0deg,rgba(0,0,0,0) 0,rgba(0,0,0,0) 2px,rgba(0,0,0,.14) 3px);opacity:.5}
.hdr{border:1px solid var(--ln);padding:6px 10px;margin:4px 0 8px;background:var(--pan)}
.hdr b{color:var(--fg);letter-spacing:2px}
.hdr .sub{color:var(--dim);font-size:11px;letter-spacing:1px;margin-top:2px}
.pan{border:1px solid var(--ln);background:var(--pan);padding:10px;margin:8px 0}
.pan>.t{color:var(--amb);font-size:12px;letter-spacing:2px;margin:-2px 0 8px;border-bottom:1px dashed var(--ln);padding-bottom:4px}
.row{display:flex;align-items:center;gap:8px;flex-wrap:wrap}
.top{display:flex;gap:10px;align-items:stretch;flex-wrap:wrap}
.top .lcol{flex:1 1 220px;min-width:200px}
.top .gcol{flex:1 1 300px;min-width:260px}
#phase{font-size:15px;color:var(--amb);letter-spacing:1px}
#live{font-size:52px;font-weight:bold;line-height:1.05;margin:2px 0}
#live small{font-size:20px;color:var(--dim)}
#info{color:var(--dim);font-size:11px;letter-spacing:.5px}
canvas{width:100%;height:150px;display:block;border:1px solid var(--ln);background:#040704}
.glab{display:flex;justify-content:space-between;color:var(--dim);font-size:10px;letter-spacing:1px;margin-top:3px}
select,input[type=number]{font-family:inherit;font-size:17px;padding:8px;background:#061006;color:var(--fg);
  border:1px solid var(--dim);border-radius:0;width:100%}
select:disabled{opacity:.35}
.sel3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
.sel3 label{font-size:11px;color:var(--dim);letter-spacing:1px;display:block;margin-bottom:2px}
.lblbox{margin-top:8px;border:1px dashed var(--ln);padding:6px 8px;color:var(--fg);font-size:18px;letter-spacing:1px}
.lblbox b{color:var(--amb)}
button{font-family:inherit;font-size:15px;padding:11px 14px;border:1px solid var(--dim);
  background:#0a140a;color:var(--fg);border-radius:0;letter-spacing:1px;cursor:pointer}
button:active{background:#0f1f0f}
#run{width:100%;font-size:22px;padding:16px;background:#0a2a16;border-color:var(--fg);color:var(--fg);margin-top:10px;letter-spacing:3px}
#run.abort{background:#2a0a0a;border-color:var(--red);color:var(--red)}
#nc{width:100%;margin-top:8px;border-color:var(--amb);color:var(--amb)}
#nc.on{background:#2a2206;color:var(--amb);border-color:var(--amb)}
#msg{margin-top:8px;font-size:14px;min-height:22px;color:var(--fg)}
#direct{color:var(--amb);font-size:13px;letter-spacing:1px;margin-bottom:8px;min-height:18px}
.matrix{border-collapse:collapse;width:100%;font-size:13px;table-layout:fixed}
.matrix th,.matrix td{border:1px solid var(--ln);padding:5px 2px;text-align:center}
.matrix th{color:var(--dim);font-size:11px;letter-spacing:1px}
.matrix td{color:var(--dim)}
.matrix td.done{color:var(--fg);background:#0a1a0e}
.matrix td.rep{color:var(--amb);background:#1a1606}
.matrix td.nobase{color:var(--blu)}
.bstat{color:var(--dim);font-size:12px;letter-spacing:1px;margin-top:6px}
.bstat b{color:var(--fg)}
table.log{width:100%;border-collapse:collapse;font-size:12px}
table.log td,table.log th{padding:3px 5px;border-bottom:1px solid var(--ln);text-align:right}
table.log td:first-child,table.log th:first-child,table.log td:nth-child(2),table.log th:nth-child(2){text-align:left}
table.log th{color:var(--dim);font-size:10px;letter-spacing:1px}
.bad{color:var(--red)}
.link{color:var(--red);font-size:12px;letter-spacing:1px}
details.pan summary{color:var(--amb);font-size:12px;letter-spacing:2px;cursor:pointer}
input[type=range]{width:100%;height:30px}
#st{color:var(--dim);font-size:12px;margin-top:6px}
</style></head><body>

<div class="hdr">
  <b>&#9608; BLACKSTAR &#9608; AERODYNAMIC LOAD TERMINAL</b>
  <div class="sub">SPECIMEN LOAD SERIES // CLEARANCE: INTERNAL // 130-UNIT CAMPAIGN</div>
</div>

<div class="pan">
  <div class="top">
    <div class="lcol">
      <div class="row" style="justify-content:space-between"><span id="phase">IDLE</span><span id="link" class="link"></span></div>
      <div id="live">0.00<small> g</small></div>
      <div id="info"></div>
    </div>
    <div class="gcol">
      <canvas id="g" width="440" height="150"></canvas>
      <div class="glab"><span id="gmax">--</span><span id="grate">LIVE LOAD TRACE</span><span id="gmin">--</span></div>
    </div>
  </div>
</div>

<div class="pan">
  <div class="t">[ SPECIMEN SELECT ]</div>
  <div id="direct"></div>
  <div class="sel3">
    <div><label>&#955; LAMBDA %</label>
      <select id="lam"><option>15</option><option>20</option><option>25</option><option>30</option><option>35</option></select></div>
    <div><label>&#952; THETA &#176;</label>
      <select id="th"><option>60</option><option>62.5</option><option>65</option><option>67.5</option><option>70</option></select></div>
    <div><label>&#945; AOA BLOCK &#176;</label>
      <select id="aoa"><option>0</option><option>5</option><option>10</option><option>15</option><option>20</option></select></div>
  </div>
  <div class="lblbox">SPECIMEN ID: <b id="lbl">--</b></div>
  <button id="nc">[ NO-CHINE BASELINE: OFF ]</button>
  <button id="run">RUN &#9655; (SPACE)</button>
  <div id="msg"></div>
</div>

<div class="pan">
  <div class="t">[ COVERAGE MATRIX &#8212; AOA <span id="mxaoa">0</span>&#176; ]</div>
  <table class="matrix" id="mx"></table>
  <div class="bstat" id="bstat"></div>
</div>

<div class="pan">
  <div class="t">[ RUN LOG &#8212; <span id="count">0</span> RECORDS IN THIS BROWSER ]</div>
  <div class="row">
    <button id="exps">EXPORT SUMMARY CSV</button>
    <button id="expr">EXPORT RAW CSV</button>
    <button id="dellast">DELETE LAST</button>
  </div>
  <div id="warn" class="bad" style="margin-top:6px"></div>
  <table class="log" id="tbl" style="margin-top:8px"></table>
</div>

<details class="pan"><summary>[ RUN PARAMETERS ]</summary>
  <div class="sel3" style="margin-top:8px">
    <div><label>M1 %</label><input type="number" id="p1" step="0.1" value="20"></div>
    <div><label>M2 %</label><input type="number" id="p2" step="0.1" value="25"></div>
    <div><label>RAMP UP %/s</label><input type="number" id="ru" step="1" value="5"></div>
    <div><label>RAMP DN %/s</label><input type="number" id="rd" step="1" value="100"></div>
    <div><label>BASELINE s</label><input type="number" id="tpre" step="0.5" value="3"></div>
    <div><label>M1 SETTLE s</label><input type="number" id="tset" step="0.5" value="5"></div>
    <div><label>HOLD s</label><input type="number" id="thold" step="0.5" value="5"></div>
    <div><label>POST BASE s</label><input type="number" id="tpost" step="0.5" value="3"></div>
    <div><label>AVG WIN s</label><input type="number" id="twin" step="0.5" value="2"></div>
    <div><label>DRIFT FLAG g</label><input type="number" id="dlim" step="0.1" value="1"></div>
  </div>
</details>

<details class="pan"><summary>[ SCALE: TARE / CALIBRATE ]</summary>
  <div class="row" style="margin-top:8px">
    <button id="tare">TARE</button>
    <input type="number" id="calm" step="0.01" placeholder="known mass g" style="width:150px">
    <button id="calb">CALIBRATE</button>
  </div>
  <div id="st">Tare with flow OFF and the model resting on the leg. Then place a known mass on top, enter its grams, press CALIBRATE. Once per power-up is enough; the factor is stored.</div>
</details>

<details class="pan"><summary>[ MANUAL MOTOR OVERRIDE ]</summary>
  <div class="row" style="margin-top:8px"><span>M1 %</span><input type="number" id="n1" min="0" max="100" step="0.1" value="0" style="width:110px"></div>
  <input type="range" id="s1" min="0" max="100" step="0.1" value="0">
  <div class="row"><span>M2 %</span><input type="number" id="n2" min="0" max="100" step="0.1" value="0" style="width:110px"></div>
  <input type="range" id="s2" min="0" max="100" step="0.1" value="0">
  <div class="row"><button id="sync">LINK MOTORS: OFF</button><button id="stop" style="border-color:var(--red);color:var(--red)">STOP</button></div>
</details>

<script>
const $=id=>document.getElementById(id);
const PHASES=['IDLE','BASELINE','M1 CLIMB','M1 SETTLE','M2 CLIMB','HOLD','RAMP DOWN','POST BASE','DONE (SAVING)','ABORTED'];
const PHCOL=['#2f7a4e','#58b0ff','#4fe08a','#2f7a4e','#f4c048','#f4c048','#f08040','#58b0ff','#4fe08a','#ff4d4d'];
const THS=[60,62.5,65,67.5,70],LAMS=[15,20,25,30,35];
const fmt2=n=>String(n).padStart(2,'0');

// ==ANALYSIS START==
function parseCsv(txt){
  const lines=txt.split('\n');let factor=1;const rows=[];
  for(const l of lines){
    if(l.startsWith('#cal,')){factor=parseFloat(l.slice(5));continue;}
    const p=l.split(',');
    if(p.length<5)continue;
    const t=+p[0];if(isNaN(t))continue;
    rows.push({t:t,raw:+p[1],m1:+p[2],m2:+p[3],ph:+p[4]});
  }
  return {factor:factor,rows:rows};
}
function lastWindow(rows,ph,winMs){
  const r=rows.filter(x=>x.ph===ph);
  if(!r.length)return [];
  const tEnd=r[r.length-1].t;
  return r.filter(x=>x.t>=tEnd-winMs);
}
const mean=a=>a.reduce((s,x)=>s+x,0)/a.length;
const sd=a=>{if(a.length<2)return 0;const m=mean(a);return Math.sqrt(a.reduce((s,x)=>s+(x-m)*(x-m),0)/(a.length-1));};
function analyze(parsed,winS,driftLim){
  const factor=parsed.factor,rows=parsed.rows,win=winS*1000;
  if(rows.length<10)return {ok:false,err:'too few samples'};
  const pre=lastWindow(rows,1,win),post=lastWindow(rows,7,win);
  const w1=lastWindow(rows,3,win),w2=lastWindow(rows,5,win);
  if(pre.length<3||post.length<3||w1.length<3||w2.length<3)return {ok:false,err:'phase window too short'};
  const zPre=mean(pre.map(x=>x.raw)),zPost=mean(post.map(x=>x.raw));
  const tPre=mean(pre.map(x=>x.t)),tPost=mean(post.map(x=>x.t));
  const zeroRaw=t=>tPost>tPre?zPre+(zPost-zPre)*(t-tPre)/(tPost-tPre):zPre;
  // lift is positive: (zero - raw)/factor
  const g=x=>(zeroRaw(x.t)-x.raw)/factor;
  const g1=w1.map(g),g2=w2.map(g);
  const dur=(rows[rows.length-1].t-rows[0].t)/1000;
  const s={
    m1_g:mean(g1),m1_sd:sd(g1),m12_g:mean(g2),m12_sd:sd(g2),
    delta_g:mean(g2)-mean(g1),
    drift_g:(zPre-zPost)/factor,
    pre_sd:sd(pre.map(x=>x.raw))/Math.abs(factor||1),
    n:rows.length,rate:dur>0?rows.length/dur:0
  };
  const flags=[];
  if(Math.abs(s.drift_g)>driftLim)flags.push('DRIFT');
  if(s.rate<5)flags.push('LOWRATE');
  if(factor===1)flags.push('UNCAL');
  // zeroG(t) in grams for raw export
  return {ok:true,summary:s,flags:flags,gOf:g,factor:factor};
}
// ==ANALYSIS END==

// ---------- no-chine baseline (time-interpolated per AoA) ----------
function ncBaseAt(aoa,tMs){
  const ncs=cache.filter(r=>r.nc&&r.aoa===aoa&&r.summary)
    .map(r=>({t:Date.parse(r.ts),v:r.summary.m12_g})).sort((a,b)=>a.t-b.t);
  if(!ncs.length)return null;
  if(ncs.length===1||tMs<=ncs[0].t)return ncs[0].v;
  if(tMs>=ncs[ncs.length-1].t)return ncs[ncs.length-1].v;
  for(let i=0;i<ncs.length-1;i++){
    if(tMs>=ncs[i].t&&tMs<=ncs[i+1].t){
      const f=(tMs-ncs[i].t)/(ncs[i+1].t-ncs[i].t);
      return ncs[i].v+(ncs[i+1].v-ncs[i].v)*f;
    }
  }
  return ncs[ncs.length-1].v;
}
function chineLift(r){
  if(!r.summary)return null;
  if(r.nc)return null;
  const b=ncBaseAt(r.aoa,Date.parse(r.ts));
  return b===null?{v:r.summary.m12_g,hasBase:false}:{v:r.summary.m12_g-b,hasBase:true};
}

// ---------- state ----------
let S={phase:0,hx:0,cal:1,g:0,out1:1000,out2:1000,n:0,el:0,abort:0,avgm:0,avgr:0,armed:0};
let runActive=false,runMeta=null,saving=false,needReset=false,epoch=0,polling=false,retries=0,uncalOk=false,prevAvgm=0,linkOk=true;
let db=null,cache=[],ncMode=false;
let trace=[],prevN=0,prevT=0,rateLive=0;
const TRACE_CAP=440;
const busy=()=>S.phase>=1&&S.phase<=7;
function msg(t){$('msg').textContent=t;}
function clamp(x,lo,hi){x=parseFloat(x);if(isNaN(x))x=lo;return Math.min(hi,Math.max(lo,x));}
function parseStatus(t){const o={};t.split(';').forEach(kv=>{const i=kv.indexOf('=');if(i>0)o[kv.slice(0,i)]=parseFloat(kv.slice(i+1));});return o;}

// ---------- specimen ----------
function currentSpec(){
  const aoa=+$('aoa').value;
  if(ncMode)return {nc:true,aoa:aoa,lam:null,th:null,label:'NC_A'+fmt2(aoa)};
  const lam=+$('lam').value,th=+$('th').value;
  return {nc:false,aoa:aoa,lam:lam,th:th,label:'L'+lam+'_T'+th+'_A'+fmt2(aoa)};
}
function refreshSpec(){
  const sp=currentSpec();
  $('lbl').textContent=sp.label;
  $('lam').disabled=ncMode;$('th').disabled=ncMode;
  $('nc').className=ncMode?'on':'';
  $('nc').textContent='[ NO-CHINE BASELINE: '+(ncMode?'ON':'OFF')+' ]';
  $('run').firstChild&&($('run').childNodes[0].nodeValue=busy()?'ABORT ':(ncMode?'RUN NO-CHINE ':'RUN '));
  renderMatrix();
}
$('nc').onclick=()=>{if(busy()||runActive)return;ncMode=!ncMode;refreshSpec();};
['lam','th','aoa'].forEach(id=>$(id).addEventListener('change',()=>{refreshSpec();$('mxaoa').textContent=$('aoa').value;}));

// ---------- settings ----------
const keep=['lam','th','aoa','p1','p2','ru','rd','tpre','tset','thold','tpost','twin','dlim','calm'];
try{keep.forEach(id=>{const v=localStorage.getItem('bs_'+id);if(v!==null)$(id).value=v;});}catch(e){}
keep.forEach(id=>$(id).addEventListener('change',e=>{try{localStorage.setItem('bs_'+id,e.target.value);}catch(x){}}));
function params(){
  return {p1:clamp($('p1').value,0,100),p2:clamp($('p2').value,0,100),
    ru:clamp($('ru').value,1,1000),rd:clamp($('rd').value,1,1000),
    pre:clamp($('tpre').value,1,30),settle:clamp($('tset').value,1,60),
    hold:clamp($('thold').value,1,60),post:clamp($('tpost').value,1,30),
    win:clamp($('twin').value,0.5,30),dlim:clamp($('dlim').value,0,1000)};
}

// ---------- IndexedDB ----------
function dbOpen(){return new Promise((res,rej)=>{
  const r=indexedDB.open('blackstar',1);
  r.onupgradeneeded=()=>{const d=r.result;d.createObjectStore('runs',{keyPath:'id',autoIncrement:true});d.createObjectStore('raw',{keyPath:'id'});};
  r.onsuccess=()=>res(r.result);r.onerror=()=>rej(r.error);});}
function dbAll(store){return new Promise((res,rej)=>{
  const rq=db.transaction(store,'readonly').objectStore(store).getAll();
  rq.onsuccess=()=>res(rq.result);rq.onerror=()=>rej(rq.error);});}
function dbAdd(rec,csv){return new Promise((res,rej)=>{
  const tx=db.transaction(['runs','raw'],'readwrite');let id;
  const rq=tx.objectStore('runs').add(rec);
  rq.onsuccess=()=>{id=rq.result;tx.objectStore('raw').put({id:id,csv:csv});};
  tx.oncomplete=()=>res(id);tx.onerror=()=>rej(tx.error);tx.onabort=()=>rej(tx.error);});}
function dbDel(id){return new Promise((res,rej)=>{
  const tx=db.transaction(['runs','raw'],'readwrite');
  tx.objectStore('runs').delete(id);tx.objectStore('raw').delete(id);
  tx.oncomplete=()=>res();tx.onerror=()=>rej(tx.error);tx.onabort=()=>rej(tx.error);});}

// ---------- rendering ----------
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));}
function renderMatrix(){
  const aoa=+$('aoa').value;
  $('mxaoa').textContent=aoa;
  let h='<tr><th>&#955;\\&#952;</th>';
  THS.forEach(t=>h+='<th>'+t+'</th>');
  h+='</tr>';
  for(const lam of LAMS){
    h+='<tr><th>'+lam+'</th>';
    for(const th of THS){
      const rs=cache.filter(r=>!r.nc&&r.aoa===aoa&&r.lam===lam&&r.th===th);
      if(!rs.length){h+='<td>&#183;</td>';continue;}
      const latest=rs[rs.length-1];
      const cl=chineLift(latest);
      let cls=rs.length>1?'rep':'done';
      let txt='--';
      if(cl){txt=cl.v.toFixed(1);if(!cl.hasBase){cls='nobase';}}
      if(rs.length>1)txt+='&#215;'+rs.length;
      h+='<td class="'+cls+'">'+txt+'</td>';
    }
    h+='</tr>';
  }
  $('mx').innerHTML=h;
  // block status + directive
  const ncN=cache.filter(r=>r.nc&&r.aoa===aoa).length;
  const cells=new Set(cache.filter(r=>!r.nc&&r.aoa===aoa).map(r=>r.lam+'_'+r.th)).size;
  $('bstat').innerHTML='BLOCK A'+aoa+'&#176; &#8212; NO-CHINE RUNS: <b>'+ncN+'</b> | CELLS FILLED: <b>'+cells+'</b>/25';
  let d;
  if(ncN===0)d='&#9656; MOUNT '+aoa+'&#176; GAUGE, SET SPECIMEN, REMOVE GAUGE, RUN START NO-CHINE';
  else if(cells<25)d='&#9656; BLIND-PICK SPECIMEN, SELECT &#955;/&#952;, RUN';
  else d='&#9656; BLOCK COMPLETE &#8212; RUN END NO-CHINE, THEN ADVANCE AOA';
  $('direct').innerHTML=d;
}
function renderTable(){
  $('count').textContent=cache.length;
  const rows=cache.slice(-12).reverse();
  let h='<tr><th>#</th><th>ID</th><th>rep</th><th>M1 g</th><th>M1+M2 g</th><th>chine g</th><th>drift</th><th>flags</th></tr>';
  for(const r of rows){
    const s=r.summary,cl=chineLift(r);
    const clt=r.nc?'(base)':(cl?(cl.hasBase?cl.v.toFixed(2):cl.v.toFixed(2)+'*'):'-');
    h+='<tr><td>'+r.id+'</td><td>'+esc(r.label)+'</td><td>'+r.rep+'</td><td>'+(s?s.m1_g.toFixed(2):'-')+'</td><td>'+(s?s.m12_g.toFixed(2):'-')+'</td><td>'+clt+'</td><td>'+(s?s.drift_g.toFixed(2):'-')+'</td><td class="'+(r.flags.length?'bad':'')+'">'+esc(r.flags.join(' '))+'</td></tr>';
  }
  $('tbl').innerHTML=h;
  const since=cache.length%25;
  $('warn').textContent=(cache.length>0&&since===0)?'25 RUNS SINCE LAST REMINDER: EXPORT YOUR DATA NOW.':'';
  renderMatrix();
}
function drawGraph(){
  const c=$('g');if(!c)return;const ctx=c.getContext('2d');
  const W=c.width,H=c.height;
  ctx.fillStyle='#040704';ctx.fillRect(0,0,W,H);
  ctx.strokeStyle='#102810';ctx.lineWidth=1;
  for(let i=0;i<=4;i++){const y=Math.round(H*i/4)+0.5;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();}
  if(trace.length<2){$('gmax').textContent='--';$('gmin').textContent='--';return;}
  let mn=Infinity,mx=-Infinity;
  for(const p of trace){if(p.g<mn)mn=p.g;if(p.g>mx)mx=p.g;}
  if(mx-mn<1){const m=(mx+mn)/2;mn=m-0.5;mx=m+0.5;}
  const pad=(mx-mn)*0.15;mn-=pad;mx+=pad;
  const n=trace.length,x=i=>W*i/(TRACE_CAP-1),y=v=>H-(v-mn)/(mx-mn)*H;
  // zero line
  if(mn<0&&mx>0){ctx.strokeStyle='#1c3a1c';ctx.beginPath();ctx.moveTo(0,y(0));ctx.lineTo(W,y(0));ctx.stroke();}
  for(let i=1;i<n;i++){
    ctx.strokeStyle=PHCOL[trace[i].ph|0]||'#4fe08a';
    ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x(i-1),y(trace[i-1].g));ctx.lineTo(x(i),y(trace[i].g));ctx.stroke();
  }
  $('gmax').textContent=mx.toFixed(1)+'g';$('gmin').textContent=mn.toFixed(1)+'g';
  $('grate').textContent=busy()&&rateLive>0?('REC '+rateLive.toFixed(0)+' Hz'):'LIVE LOAD TRACE';
}
function render(){
  const ph=S.phase|0;
  $('phase').textContent=S.armed?(PHASES[ph]||('?'+ph)):'ARMING...';
  $('live').innerHTML=S.g.toFixed(2)+'<small> g</small>';
  $('info').textContent='HX711 '+(S.hx?'OK':'NOT FOUND')+' | CAL '+S.cal.toFixed(3)+(S.cal===1?' (UNCAL)':'')+' | N '+S.n+(busy()?' | T '+(S.el/1000).toFixed(1)+'s':'');
  const r=$('run');
  if(busy()){r.className='abort';r.childNodes[0].nodeValue='ABORT ';}
  else{r.className='';r.childNodes[0].nodeValue=ncMode?'RUN NO-CHINE ':'RUN ';}
  $('link').textContent=linkOk?'':'LINK LOST';
}

// ---------- polling / heartbeat ----------
async function poll(){
  if(polling)return;
  polling=true;const e=epoch;
  try{
    let url='/set?hb=1';
    if(!runActive)url+='&m1='+$('s1').value+'&m2='+$('s2').value+'&ru='+clamp($('ru').value,1,1000)+'&rd='+clamp($('rd').value,1,1000);
    const ctl=new AbortController();const to=setTimeout(()=>ctl.abort(),1500);
    let t;
    try{t=await (await fetch(url,{signal:ctl.signal})).text();}finally{clearTimeout(to);}
    if(e!==epoch)return;
    S=Object.assign({phase:0,hx:0,cal:1,g:0,out1:1000,out2:1000,n:0,el:0,abort:0,avgm:0,avgr:0,armed:0},parseStatus(t));
    linkOk=true;
    // live sample rate during a run
    const nowP=performance.now();
    if(prevT&&S.n>=prevN&&busy()){const dt=(nowP-prevT)/1000;if(dt>0)rateLive=0.6*rateLive+0.4*((S.n-prevN)/dt);}
    if(!busy())rateLive=0;
    prevN=S.n;prevT=nowP;
    // trace
    trace.push({g:S.g,ph:S.phase});if(trace.length>TRACE_CAP)trace.shift();
    drawGraph();
    if(prevAvgm>0&&S.avgm===0)msg(S.avgr===1?'DONE. SCALE UPDATED.':'FAILED. CHECK MASS/SIGNAL AND RETRY.');
    prevAvgm=S.avgm;
    render();
    await handlePhase();
  }catch(err){linkOk=false;render();}
  finally{polling=false;}
}
async function handlePhase(){
  if(S.phase===8){
    if(!needReset&&!saving){
      saving=true;let ok=false;
      try{ok=await finishRun();}finally{saving=false;}
      if(ok)needReset=true;
    }
    if(needReset){
      try{await fetch('/reset');epoch++;needReset=false;}catch(e){}
    }
  }else if(S.phase===9){
    runActive=false;runMeta=null;
    const why=['','LINK LOST','BUFFER FULL','STOPPED','TIMEOUT'][S.abort]||'';
    msg('RUN ABORTED ('+why+'). NOT SAVED.');
    try{await fetch('/reset');epoch++;}catch(e){}
  }
}

// ---------- runs ----------
async function finishRun(){
  const meta=runMeta||{label:'recovered',nc:false,aoa:+$('aoa').value,lam:null,th:null,params:params()};
  let txt;
  try{
    const r=await fetch('/data');
    if(!r.ok)throw new Error('HTTP '+r.status);
    txt=await r.text();
  }catch(err){
    retries++;
    if(retries>=5){msg('COULD NOT FETCH DATA FROM BOARD. IT STAYS THERE UNTIL NEXT RUN.');runActive=false;retries=0;}
    else msg('FETCH FAILED, RETRYING...');
    return false;
  }
  retries=0;
  const parsed=parseCsv(txt);
  const a=analyze(parsed,meta.params.win,meta.params.dlim);
  const rep=cache.filter(r=>r.label===meta.label).length+1;
  const rec={ts:new Date().toISOString(),label:meta.label,nc:!!meta.nc,aoa:meta.aoa,lam:meta.lam,th:meta.th,
    rep:rep,params:meta.params,summary:a.ok?a.summary:null,flags:a.ok?a.flags:['ANALYSIS_FAIL:'+a.err],factor:parsed.factor};
  const id=await dbAdd(rec,txt);
  rec.id=id;cache.push(rec);
  runActive=false;runMeta=null;
  renderTable();
  const s=rec.summary;
  const cl=chineLift(rec);
  let tail=s?('M1 '+s.m1_g.toFixed(2)+' g, M1+M2 '+s.m12_g.toFixed(2)+' g'+(cl?(', CHINE '+cl.v.toFixed(2)+' g'+(cl.hasBase?'':'*')):'')+', drift '+s.drift_g.toFixed(2)+' g'):'analysis failed, raw saved';
  msg('REC '+id+' SAVED ['+meta.label+' #'+rep+']: '+tail+(rec.flags.length?'  FLAGS: '+rec.flags.join(' '):'')+'. LOAD NEXT.');
  return true;
}
async function startRun(){
  if(!db){msg('BROWSER STORAGE UNAVAILABLE. NOT STARTING: DATA WOULD BE LOST.');return;}
  if(busy()||runActive)return;
  const sp=currentSpec();
  if(!S.armed){msg('BOARD STILL ARMING. WAIT A FEW SECONDS.');return;}
  if(!S.hx){msg('HX711 NOT RESPONDING. CHECK WIRING.');return;}
  if(S.out1>1000||S.out2>1000||parseFloat($('s1').value)>0||parseFloat($('s2').value)>0){msg('MOTORS NOT IDLE. PRESS STOP FIRST.');return;}
  const dup=cache.filter(r=>r.label===sp.label).length;
  if(dup>0&&!sp.nc){if(!confirm(sp.label+' already has '+dup+' run(s). Log a repeat?'))return;}
  if(S.cal===1&&!uncalOk){if(!confirm('Scale not calibrated, so grams will be wrong. Run anyway?'))return;uncalOk=true;}
  const p=params();
  const q=new URLSearchParams({p1:p.p1,p2:p.p2,pre:p.pre,settle:p.settle,hold:p.hold,post:p.post,ru:p.ru,rd:p.rd});
  runMeta={label:sp.label,nc:sp.nc,aoa:sp.aoa,lam:sp.lam,th:sp.th,params:p};runActive=true;msg('STARTING '+sp.label+'...');
  try{
    const t=await (await fetch('/start?'+q.toString())).text();
    if(t.indexOf('OK')!==0){runActive=false;runMeta=null;msg('BOARD REFUSED: '+t);return;}
  }catch(e){runActive=false;runMeta=null;msg('LINK ERROR. NOT STARTED.');return;}
  trace=[];msg('RUNNING '+sp.label+'...');
}
async function abortRunNow(){try{await fetch('/stop');}catch(e){}msg('ABORTING...');}
function stopManual(){setBoth(0,0);fetch('/stop').catch(()=>{});poll();}
function mainAction(){
  if(busy()){abortRunNow();return;}
  if(S.out1>1000||S.out2>1000||parseFloat($('s1').value)>0||parseFloat($('s2').value)>0){stopManual();return;}
  startRun();
}
$('run').onclick=mainAction;
document.addEventListener('keydown',e=>{
  if(e.code!=='Space')return;
  const a=document.activeElement;
  if(a&&(a.tagName==='SELECT'||a.tagName==='INPUT'))return;
  e.preventDefault();
  if(e.repeat)return;
  if(a&&a.blur)a.blur();
  mainAction();
});
document.addEventListener('keyup',e=>{
  const a=document.activeElement;
  if(e.code==='Space'&&!(a&&(a.tagName==='SELECT'||a.tagName==='INPUT')))e.preventDefault();
});

// ---------- scale ----------
$('tare').onclick=async()=>{if(busy())return;msg('TARING: KEEP FLOW OFF, MODEL STILL...');try{await fetch('/tare');}catch(e){}};
$('calb').onclick=async()=>{
  if(busy())return;
  const m=parseFloat($('calm').value);
  if(!(m>0)){msg('ENTER THE KNOWN MASS IN GRAMS.');return;}
  msg('CALIBRATING WITH '+m+' g...');
  try{const t=await (await fetch('/cal?g='+m)).text();if(t.indexOf('OK')!==0)msg(t);}catch(e){}
};

// ---------- export ----------
function qcsv(s){s=String(s);return /[",\n]/.test(s)?'"'+s.replace(/"/g,'""')+'"':s;}
function download(name,text){
  const b=new Blob([text],{type:'text/csv'});const a=document.createElement('a');
  a.href=URL.createObjectURL(b);a.download=name;document.body.appendChild(a);a.click();
  setTimeout(()=>{URL.revokeObjectURL(a.href);a.remove();},1500);
}
$('exps').onclick=()=>{
  const out=['run_id,timestamp,specimen_id,aoa_deg,lambda_pct,theta_deg,is_no_chine,rep,M1_pct,M2_pct,M1_only_g,M1_only_sd,M1M2_g,M1M2_sd,M2_increment_g,nc_baseline_g,chine_lift_g,baseline_drift_g,baseline_noise_sd_g,samples,rate_hz,cal_counts_per_g,flags'];
  for(const r of cache){
    const s=r.summary,p=r.params;
    const base=(!r.nc&&s)?ncBaseAt(r.aoa,Date.parse(r.ts)):null;
    const chine=(base!==null&&s)?(s.m12_g-base):null;
    out.push([r.id,r.ts,qcsv(r.label),r.aoa,(r.lam==null?'':r.lam),(r.th==null?'':r.th),r.nc?1:0,r.rep,p.p1,p.p2,
      s?s.m1_g.toFixed(4):'',s?s.m1_sd.toFixed(4):'',s?s.m12_g.toFixed(4):'',s?s.m12_sd.toFixed(4):'',
      s?s.delta_g.toFixed(4):'',base!==null?base.toFixed(4):'',chine!==null?chine.toFixed(4):'',
      s?s.drift_g.toFixed(4):'',s?s.pre_sd.toFixed(4):'',s?s.n:'',s?s.rate.toFixed(1):'',r.factor,qcsv(r.flags.join(' '))].join(','));
  }
  download('blackstar_summary_'+new Date().toISOString().slice(0,10)+'.csv',out.join('\n')+'\n');
};
$('expr').onclick=async()=>{
  msg('BUILDING RAW EXPORT...');
  const raws=await dbAll('raw');const byId={};cache.forEach(r=>byId[r.id]=r);
  const out=['run_id,specimen_id,aoa_deg,lambda_pct,theta_deg,is_no_chine,rep,t_ms,phase,raw_counts,m1_us,m2_us,lift_g'];
  for(const rw of raws){
    const r=byId[rw.id];if(!r)continue;
    const parsed=parseCsv(rw.csv);
    const a=analyze(parsed,r.params.win,r.params.dlim);
    for(const x of parsed.rows){
      const g=a.ok?a.gOf(x).toFixed(4):'';
      out.push(r.id+','+qcsv(r.label)+','+r.aoa+','+(r.lam==null?'':r.lam)+','+(r.th==null?'':r.th)+','+(r.nc?1:0)+','+r.rep+','+x.t+','+x.ph+','+x.raw+','+x.m1+','+x.m2+','+g);
    }
  }
  download('blackstar_raw_'+new Date().toISOString().slice(0,10)+'.csv',out.join('\n')+'\n');
  msg('RAW EXPORT DOWNLOADED ('+raws.length+' RUNS).');
};
$('dellast').onclick=async()=>{
  if(!cache.length)return;
  const r=cache[cache.length-1];
  if(!confirm('Delete run '+r.id+' ('+r.label+' #'+r.rep+')? This cannot be undone.'))return;
  await dbDel(r.id);cache.pop();renderTable();msg('REC '+r.id+' DELETED.');
};

// ---------- manual motors ----------
const s1=$('s1'),s2=$('s2'),n1=$('n1'),n2=$('n2');
let linked=false;
function setBoth(a,b){s1.value=n1.value=a;s2.value=n2.value=b;}
function applyManual(from){
  const v=clamp((from===1?n1:n2).value,0,100);
  if(linked)setBoth(v,v);else if(from===1){s1.value=n1.value=v;}else{s2.value=n2.value=v;}
  poll();
}
s1.oninput=()=>{n1.value=s1.value;applyManual(1);};
s2.oninput=()=>{n2.value=s2.value;applyManual(2);};
n1.onchange=()=>applyManual(1);n2.onchange=()=>applyManual(2);
n1.onkeydown=e=>{if(e.key==='Enter')n1.blur();};
n2.onkeydown=e=>{if(e.key==='Enter')n2.blur();};
$('sync').onclick=function(){
  linked=!linked;this.textContent='LINK MOTORS: '+(linked?'ON':'OFF');
  if(linked){const v=clamp(n1.value,0,100);setBoth(v,v);poll();}
};
$('stop').onclick=()=>{if(busy())abortRunNow();stopManual();};

// ---------- boot ----------
(async function(){
  try{
    db=await dbOpen();
    cache=await dbAll('runs');
    cache.sort((a,b)=>a.id-b.id);
    renderTable();
  }catch(e){
    db=null;
    $('warn').textContent='BROWSER STORAGE UNAVAILABLE (PRIVATE WINDOW?). RUNS BLOCKED SO DATA IS NOT LOST.';
  }
  refreshSpec();
  $('mxaoa').textContent=$('aoa').value;
  drawGraph();
  setInterval(poll,250);
  poll();
})();
</script></body></html>
)HTML";

void handleRoot() { server.send_P(200, "text/html", PAGE); }

void statusStr(char* b, size_t n) {
  // lift is positive: (zeroRaw - raw) / calFactor
  float g = (calFactor != 0.0f) ? (zeroRaw - (float)lastRaw) / calFactor : 0.0f;
  int hx = (lastHxMs != 0 && ageMs(lastHxMs) < 1000) ? 1 : 0;
  int32_t el = (phase >= PH_PRE && phase <= PH_POST) ? ageMs(runStartMs) : 0;
  if (el < 0) el = 0;
  snprintf(b, n,
           "phase=%u;out1=%d;out2=%d;cmd1=%d;cmd2=%d;g=%.3f;raw=%ld;hx=%d;cal=%.4f;n=%lu;el=%ld;fs=%u;up=%lu;avgm=%u;avgr=%u;abort=%u;armed=%d",
           (unsigned)phase, (int)(out1 + 0.5f), (int)(out2 + 0.5f), (int)cmd1, (int)cmd2, g,
           (long)lastRaw, hx, calFactor, (unsigned long)nSamples, (long)el, (unsigned)failsafeTrips,
           (unsigned long)(millis() / 1000), (unsigned)avgMode, (unsigned)avgResult,
           (unsigned)abortReason, armed ? 1 : 0);
}

float argF(const char* name, float def) {
  return server.hasArg(name) ? server.arg(name).toFloat() : def;
}

void sendStatus() {
  char b[320];
  statusStr(b, sizeof(b));
  server.send(200, "text/plain", b);
}

bool runIdleState() { return phase == PH_IDLE || phase == PH_DONE || phase == PH_ABORT; }

void handleSet() {
  lastBeatMs = millis();
  if (runIdleState()) {
    if (server.hasArg("ru")) RAMP_UP_US_PER_S = clampf(argF("ru", 5), 1.0f, 1000.0f) * 10.0f;
    if (server.hasArg("rd")) RAMP_DOWN_US_PER_S = clampf(argF("rd", 100), 1.0f, 1000.0f) * 10.0f;
    if (armed && server.hasArg("m1") && server.hasArg("m2")) {
      cmd1 = pctToUs(argF("m1", 0));
      cmd2 = pctToUs(argF("m2", 0));
    }
  }
  sendStatus();
}

void handleStart() {
  lastBeatMs = millis();
  if (!armed) { server.send(200, "text/plain", "ERR: board still arming"); return; }
  if (!runIdleState()) { server.send(200, "text/plain", "ERR: a run is already active"); return; }
  if (lastHxMs == 0 || ageMs(lastHxMs) > 1000) { server.send(200, "text/plain", "ERR: HX711 not responding"); return; }
  if (out1 > US_IDLE + 1 || out2 > US_IDLE + 1) { server.send(200, "text/plain", "ERR: motors not idle"); return; }
  if (!samples) { server.send(200, "text/plain", "ERR: no sample buffer"); return; }

  float p1 = clampf(argF("p1", 0), 0.0f, 100.0f);
  float p2 = clampf(argF("p2", 0), 0.0f, 100.0f);
  float pre = clampf(argF("pre", 3), 1.0f, 30.0f);
  float settle = clampf(argF("settle", 5), 1.0f, 60.0f);
  float hold = clampf(argF("hold", 5), 1.0f, 60.0f);
  float post = clampf(argF("post", 3), 1.0f, 30.0f);
  if (server.hasArg("ru")) RAMP_UP_US_PER_S = clampf(argF("ru", 5), 1.0f, 1000.0f) * 10.0f;
  if (server.hasArg("rd")) RAMP_DOWN_US_PER_S = clampf(argF("rd", 100), 1.0f, 1000.0f) * 10.0f;

  int c1 = pctToUs(p1), c2 = pctToUs(p2);
  float upS = (float)(c1 - US_IDLE) / RAMP_UP_US_PER_S + (float)(c2 - US_IDLE) / RAMP_UP_US_PER_S;
  float downS = (float)((c1 > c2 ? c1 : c2) - US_IDLE) / RAMP_DOWN_US_PER_S;
  float est = pre + settle + hold + post + upS + downS + 1.0f;
  if (est * 80.0f > (float)SAMPLE_CAP) { server.send(200, "text/plain", "ERR: run too long for the sample buffer"); return; }

  runCmd1 = c1;
  runCmd2 = c2;
  preMs = (uint32_t)(pre * 1000.0f);
  settleMs = (uint32_t)(settle * 1000.0f);
  holdMs = (uint32_t)(hold * 1000.0f);
  postMs = (uint32_t)(post * 1000.0f);
  nSamples = 0;
  bufFull = false;
  abortReason = 0;
  cmd1 = US_IDLE;
  cmd2 = US_IDLE;
  runStartMs = millis();
  phaseStartMs = runStartMs;
  recording = true;
  phase = PH_PRE;                       // set last: publishes the run to the control task
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  lastBeatMs = millis();
  if (phase >= PH_PRE && phase <= PH_POST) abortRun(3);
  cmd1 = US_IDLE;
  cmd2 = US_IDLE;
  sendStatus();
}

void handleReset() {
  lastBeatMs = millis();
  if (phase == PH_DONE || phase == PH_ABORT) phase = PH_IDLE;
  server.send(200, "text/plain", "OK");
}

void handleTare() {
  lastBeatMs = millis();
  if (!runIdleState() || avgMode) { server.send(200, "text/plain", "ERR: busy"); return; }
  avgSum = 0; avgCnt = 0; avgUntilMs = millis() + 1500; avgResult = 0;
  avgMode = 1;
  server.send(200, "text/plain", "OK");
}

void handleCal() {
  lastBeatMs = millis();
  float g = argF("g", 0);
  if (!tared) { server.send(200, "text/plain", "ERR: Tare first"); return; }
  if (!runIdleState() || avgMode || g <= 0.0f) { server.send(200, "text/plain", "ERR: busy or bad mass"); return; }
  calMass = g;
  avgSum = 0; avgCnt = 0; avgUntilMs = millis() + 1500; avgResult = 0;
  avgMode = 2;
  server.send(200, "text/plain", "OK");
}

void handleData() {
  lastBeatMs = millis();
  if (phase >= PH_PRE && phase <= PH_POST) { server.send(409, "text/plain", "run in progress"); return; }
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  char buf[1200];
  int pos = snprintf(buf, sizeof(buf), "#cal,%.6f\nt_ms,raw,m1_us,m2_us,phase\n", calFactor);
  uint32_t n = nSamples;
  for (uint32_t i = 0; i < n; i++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, "%lu,%ld,%u,%u,%u\n",
                    (unsigned long)samples[i].t, (long)samples[i].raw,
                    (unsigned)samples[i].m1, (unsigned)samples[i].m2, (unsigned)samples[i].ph);
    if (pos > 1000) {
      buf[pos] = 0;
      server.sendContent(String(buf));
      pos = 0;
    }
  }
  if (pos > 0) {
    buf[pos] = 0;
    server.sendContent(String(buf));
  }
  server.sendContent("");
  lastBeatMs = millis();
}

void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc1.attach(PIN_M1, 1000, 2000);
  esc2.attach(PIN_M2, 1000, 2000);
  esc1.writeMicroseconds(US_IDLE);
  esc2.writeMicroseconds(US_IDLE);

  pinMode(PIN_HX_DOUT, INPUT);
  pinMode(PIN_HX_SCK, OUTPUT);
  digitalWrite(PIN_HX_SCK, LOW);

  prefs.begin("rig", false);
  calFactor = prefs.getFloat("cal", 1.0f);
  if (calFactor == 0.0f) calFactor = 1.0f;

  samples = (Sample*)malloc(sizeof(Sample) * SAMPLE_CAP);
  if (!samples) Serial.println("ERROR: sample buffer allocation failed");

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());      // 192.168.4.1

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/reset", handleReset);
  server.on("/tare", handleTare);
  server.on("/cal", handleCal);
  server.on("/data", handleData);
  server.begin();

  xTaskCreatePinnedToCore(controlTask, "ctl", 6144, NULL, 2, NULL, 1);
}

void loop() {
  // handleClient() can block up to 5 s on an idle browser socket. Tell the control task so
  // it does not read that as a lost link, and credit the stall back to the heartbeat clock.
  uint32_t beatBefore = lastBeatMs;
  uint32_t t0 = millis();
  srvBusyStart = t0 ? t0 : 1;
  server.handleClient();
  uint32_t spent = millis() - t0;
  if (spent > 200 && lastBeatMs == beatBefore) lastBeatMs = beatBefore + spent;
  srvBusyStart = 0;
  delay(1);
}
