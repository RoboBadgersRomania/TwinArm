// TwinArm motor test: finds drivers 1-4, homes them to the arm's zero (driver origin + HOME_OFFSET) and holds.
// All angles typed and printed are relative to that arm zero.
// Serial monitor (send with newline):
//   x = release all   h = home all   d = toggle raw CAN frame dump   s = toggle gripper sweep
//   <id> a<deg>  go to angle      1 a90
//   <id> r<deg>  move relative    1 r-30
//   <id> v<rpm>  max move speed   1 v30
//   <id> h       home one         1 h
//   <id> o       release one      1 o
//   <id> c       clear fault/jam  1 c   (shows the fault code left afterwards)
//   <id> z       arm zero here    4 z   (where the joint is now becomes its 0, kept in EEPROM for every homing;
//                answer Z,id,ok,offset from the driver origin)
//   <id> g [pos kp ki vel kp ki]  read, or write then read, the driver's loop gains   1 g 20 0.5 3 0.1
//   <id> m<amps> max current      1 m8
//   <id> s<deg>  step test: move by deg from where it is, sample every 10 ms for 2 s, send the curve   1 s5
//   t <deg>      turret angle     t -30  (0 = centre)
//   p <%>        gripper opening  p 40   (0 = closed, 100 = open, inside the hard limits)
//   j [deg ms]   jam detection: read, or set tolerance and time (ms 0 = off)   j 4 400
// Teach and replay, the take living in RAM and saved on the SD card in the Teensy's slot:
//   rec          record what the arm is told (joints, turret, gripper) 100 times a second, up to 5 min
//   stop         end the recording (saved as TAKEnnn.BIN) or the replay (the arm holds where it is)
//   play <%>     replay the loaded take at this speed (10-200, 100 = as taught): a minimum-jerk move to its
//                first pose, then the take, smoothed. A manual move (a r s h o t p x) or a jam stops it
//   load <n>     load TAKEn from the card      takes   list the takes on the card      del <n>  delete TAKEn
// MPC for the replay, always on for joints with a model (JointMPC.h; the maths and a simulated comparison in
// InterfaceApp/mpc.py). Models and lead are kept in EEPROM:
//   mpc <id> <wn> <zeta>   the joint's model, from its step-test fit (the app sends model_fit.json)   mpc 2 14.9 0.52
//   mpc lead <deg>         how far a setpoint may lead the take (default 25)      mpc test   self-test and timing
//   mpc                    status:  Q,on,lead,slowest tick us,id:wn:zeta,...  (wn 0 = no model: plain replay)
// The same commands arrive from the LightGateway on Serial2 (WiFi app), which gets one machine-readable
// line per motor every TELEMETRY_MS:  T,id,ok,deg,target,amps,tempC,volts,busAmps,mode,fault,jammed
// and the recorder's state:  R,idle|rec|move|play,take number (0 = not saved),seconds in,take seconds,sd ok
// and answers to g and j:  P,id,ok,posKp,posKi,velKp,velKi   J,deg,ms   to takes: K,n,seconds ... K,end,count
// and to s:  S,id,begin,start,target,sampleMs  then S,id,i,deg,deg,... (10 samples from index i per line)
//            then S,id,end  (or S,id,fail)
// A joint that stops short of its target (see checkJam) holds where it stopped and ignores a/r until cleared.
#include "GDZ.h"
#include "JointMPC.h"
#include <EEPROM.h>
#include <Servo.h>
#include <SD.h>

Servo gripper;                      // DS 40 kg servo, signal on pin 6, powered from its own supply
const int GRIPPER_PIN = 6;
const int GRIPPER_MID_US = 1500;    // idle position
const int GRIPPER_CLOSED_US = 1350, GRIPPER_OPEN_US = 2000;  // hard limits: every pulse is clamped to these
bool sweeping = false;              // 's' toggles; stopping returns to idle
const float GRIP_MID_PCT = 100.0f * (GRIPPER_MID_US - GRIPPER_CLOSED_US) / (GRIPPER_OPEN_US - GRIPPER_CLOSED_US);
float gripPct = GRIP_MID_PCT;       // last commanded opening, %, for the recorder

void grip(int us) { gripper.writeMicroseconds(constrain(us, GRIPPER_CLOSED_US, GRIPPER_OPEN_US)); }

void openTo(float pct) {            // 0 = closed, 100 = open, inside the hard limits
  gripPct = constrain(pct, 0.0f, 100.0f);
  grip(lroundf(GRIPPER_CLOSED_US + gripPct * (GRIPPER_OPEN_US - GRIPPER_CLOSED_US) / 100));
}

Servo turret;                       // base turret: DS5180 80 kg servo, signal on pin 29, powered from its own supply
const int TURRET_PIN = 29;          // held at the centre (1500 us, turn(0)) from boot until the app moves it
const float TURRET_SPAN_DEG = 180;  // calibrate: degrees the turret turns between the 500 us and 2500 us pulses
const int TURRET_MIN_US = 500, TURRET_MAX_US = 2500;  // hard limits: every pulse is clamped to these
float turretDeg = 0;                // last commanded angle, for the recorder

void turn(float deg) {
  turretDeg = deg;
  int us = lroundf(1500 + deg * 2000 / TURRET_SPAN_DEG);
  turret.writeMicroseconds(constrain(us, TURRET_MIN_US, TURRET_MAX_US));
}

auto &gateway = Serial2;             // pin 7 RX2 <- LightGateway TXD, pin 8 TX2 -> LightGateway RXD, GND - GND
const uint32_t GATEWAY_BAUD = 115200;  // same as UART_BAUD in lightgateway.ino
const int TELEMETRY_MS = 100;
uint8_t gatewayRx[512];             // extra RX buffer: command bursts keep arriving while a telemetry sweep runs

GDZBus<CAN1> bus;                   // CAN1: pin 22 -> TJA1051 TXD, pin 23 <- RXD
const uint8_t IDS[] = {1, 2, 3, 4};
const uint32_t CAN_BAUD = 1000000;  // driver default; must match the CAN baud set in the ZE300 GUI
const float HOME_RPM = 10;          // calibrate: speed limit for the homing move (driver forgets it at power-off)
const int MAX_ID = 4;
// [id]: degrees from driver origin to arm zero, by default, where the arm is straight (2026-11-03: with the zeros
// before, the app's alignment dials read shoulder -23.3, elbow -0.6, wrist pitch -26.6 there; those dials show the
// drawing's direction, sign x motor degrees, and shoulder and wrist pitch run inverted)
const float HOME_OFFSET[MAX_ID + 1] = {0, 5.7f, 3.7f, 9.9f, 0.0f};  // then shoulder -18.3, elbow -0.7 (3D view); wrist roll at its driver origin
float homeOffset[MAX_ID + 1];       // the offsets in use: HOME_OFFSET, or what '<id> z' measured (kept in EEPROM)
float originDeg[MAX_ID + 1];        // driver angle where this boot's homing found the origin
bool homed[MAX_ID + 1];             // the origin above is valid
float zeroDeg[MAX_ID + 1];          // driver angle of each arm zero, set by homing
float target[MAX_ID + 1];           // last commanded angle relative to the arm zero, reported as telemetry

float jamErrDeg = 4;                // calibrate, or 'j' from the app: further than this from the target = it should be moving
int jamMs = 400;                    // calibrate, or 'j': not moving for this long = jammed; 0 = detection off
bool jammed[MAX_ID + 1];
float lastDeg[MAX_ID + 1], rpmSet[MAX_ID + 1];  // angle at the last sweep, speed limit last sent to the driver
int stuckMs[MAX_ID + 1];

const char *MODES[] = {"off", "volt", "current", "speed", "pos"};

// Jammed = in position mode, far from the target, yet it moved less than a quarter of what its speed limit
// allows in one sweep (and under 1 deg, so a joint that is slow but moving is not a jam), for jamMs.
// Then it holds where it stopped instead of pushing on.
// ponytail: position-only stall check; add a phase-current threshold if slow moves under heavy load trip it.
void checkJam(uint8_t id, float deg) {
  const GDZMotor &s = bus.m[id];
  float allowed = rpmSet[id] * 6 * TELEMETRY_MS / 1000.0f;  // deg per sweep at the speed limit
  bool stuck = s.mode == 4 && fabsf(target[id] - deg) > jamErrDeg && fabsf(deg - lastDeg[id]) < min(0.25f * allowed, 1.0f);
  stuckMs[id] = stuck ? stuckMs[id] + TELEMETRY_MS : 0;
  lastDeg[id] = deg;
  if (jammed[id] || !jamMs || stuckMs[id] < jamMs) return;
  jammed[id] = true;
  target[id] = deg;
  bus.moveTo(id, s.deg);
  Serial.printf("motor %d: JAMMED at %.1f deg, holding there; '%d c' clears it\n", id, deg, id);
}

// Polls until the joint stops moving (< 0.1 deg in 200 ms), max 10 s.
bool waitStill(uint8_t id) {
  float last = NAN;
  for (elapsedMillis t; t < 10000; delay(200)) {
    if (!bus.readPosition(id)) return false;
    if (fabsf(bus.m[id].deg - last) < 0.1f) return true;
    last = bus.m[id].deg;
  }
  return false;
}

// Driver homes to its origin; once it has settled, move on by the offset to the arm zero.
void homeOne(uint8_t id) {
  bus.status(id);                   // fresh fault state; a driver that was still booting at ping gets homed too
  if (bus.m[id].fault) bus.clearFault(id);
  jammed[id] = false;
  stuckMs[id] = 0;
  rpmSet[id] = HOME_RPM;
  bus.setMaxSpeed(id, HOME_RPM);
  bool ok = bus.home(id) && waitStill(id);
  homed[id] = ok;
  if (ok) {
    originDeg[id] = bus.m[id].deg;
    zeroDeg[id] = originDeg[id] + homeOffset[id];
    target[id] = 0;
    ok = bus.moveTo(id, zeroDeg[id]);
  }
  Serial.printf("motor %d: home %s, arm zero %.2f deg from the driver origin\n", id, ok ? "ok" : "FAILED", homeOffset[id]);
}

// ---------- arm zero calibration: '<id> z' makes where the joint is now its arm zero ----------
// The offset from the driver origin is kept in EEPROM (after the MPC models) and used by every later homing,
// so a gripper remounted or slipped on its shaft is calibrated once, without editing HOME_OFFSET.
const int ZERO_ADDR = 128;
const uint32_t ZERO_MAGIC = 0x314F525A;     // "ZRO1"
struct ZeroStore { uint32_t magic; float off[MAX_ID + 1]; };

void zeroLoad() {
  ZeroStore st;
  EEPROM.get(ZERO_ADDR, st);
  for (uint8_t id : IDS)
    homeOffset[id] = st.magic == ZERO_MAGIC && isfinite(st.off[id]) && fabsf(st.off[id]) < 720 ? st.off[id] : HOME_OFFSET[id];
  if (st.magic == ZERO_MAGIC) Serial.println("arm zeros from EEPROM");
}

void zeroHere(uint8_t id) {                 // answer: Z,id,ok,offset from the driver origin
  bool ok = homed[id] && !jammed[id] && bus.readPosition(id);
  if (ok) {
    homeOffset[id] = bus.m[id].deg - originDeg[id];
    zeroDeg[id] = bus.m[id].deg;
    target[id] = 0;
    lastDeg[id] = 0;
    ok = bus.moveTo(id, zeroDeg[id]);         // hold right here: it is the new 0
    ZeroStore st = {ZERO_MAGIC, {}};
    for (uint8_t i : IDS) st.off[i] = homeOffset[i];
    EEPROM.put(ZERO_ADDR, st);
  }
  gateway.printf("Z,%d,%d,%.2f\n", id, ok, homeOffset[id]);
  Serial.printf("motor %d: %s\n", id, ok ? "arm zero set here, kept for every homing" :
                homed[id] ? "zero not set (no reply, or jammed)" : "zero not set: home it first");
}

// Position and velocity loop gains straight from the driver, to the app (P line) and the console.
void reportGains(uint8_t id, bool echo) {
  float g[4] = {NAN, NAN, NAN, NAN};
  bool ok = true;
  for (int k = 0; k < 4; k++) ok = bus.getGain(id, bus.POS_KP + k, g[k]) && ok;
  gateway.printf("P,%d,%d,%.4f,%.4f,%.4f,%.4f\n", id, ok, g[0], g[1], g[2], g[3]);
  if (echo) Serial.printf("motor %d gains: pos kp %.4f ki %.4f, vel kp %.4f ki %.4f%s\n", id, g[0], g[1], g[2], g[3],
                          ok ? "" : "  (no reply)");
}

// Step test for tuning: a step from where the joint is now, sampled far faster than the 10 Hz telemetry.
// Blocks the loop for STEP_MS (no telemetry meanwhile), then sends the curve to the app.
const int STEP_SAMPLE_MS = 10, STEP_MS = 2000;
void stepTest(uint8_t id, float deg, bool echo) {
  static float y[STEP_MS / STEP_SAMPLE_MS];
  float start = bus.readPosition(id) ? bus.m[id].deg - zeroDeg[id] : target[id];
  target[id] = start + deg;
  if (!bus.moveTo(id, zeroDeg[id] + target[id])) { gateway.printf("S,%d,fail\n", id); return; }
  int n = 0;
  for (elapsedMillis t; n < STEP_MS / STEP_SAMPLE_MS;)
    if (t >= (uint32_t)(n * STEP_SAMPLE_MS)) y[n++] = bus.readPosition(id) ? bus.m[id].deg - zeroDeg[id] : NAN;
  gateway.printf("S,%d,begin,%.3f,%.3f,%d\n", id, start, target[id], STEP_SAMPLE_MS);
  for (int i = 0; i < n; i += 10) {   // 10 samples a line: 20 WiFi datagrams instead of 200, fewer to lose
    gateway.printf("S,%d,%d", id, i);
    for (int k = i; k < i + 10 && k < n; k++) gateway.printf(",%.2f", y[k]);
    gateway.print('\n');
  }
  gateway.printf("S,%d,end\n", id);
  lastDeg[id] = y[n - 1];             // no false jam from the jump on the next sweep
  if (echo) Serial.printf("motor %d: step %+.1f deg captured, %d samples\n", id, deg, n);
}

// Every joint homes at the same time: all drivers get the home command together, the gripper opens and closes
// once meanwhile, and once every joint has settled (or after 10 s) they all move on to their arm zero together.
// The turret is not touched: it holds where it was told (the centre after boot). '<id> h' still homes one joint.
void homeAll() {
  bool started[MAX_ID + 1] = {}, still[MAX_ID + 1] = {};
  float last[MAX_ID + 1];
  for (uint8_t id : IDS) {
    bus.status(id);                 // fresh fault state; a driver that was still booting at ping gets homed too
    if (bus.m[id].fault) bus.clearFault(id);
    jammed[id] = homed[id] = false;
    stuckMs[id] = 0;
    rpmSet[id] = HOME_RPM;
    bus.setMaxSpeed(id, HOME_RPM);
    last[id] = NAN;
  }
  for (uint8_t id : IDS) started[id] = bus.home(id);  // back to back: they start within a few ms of each other
  grip(GRIPPER_OPEN_US);
  int gripPhase = 0;
  for (elapsedMillis t; t < 10000; delay(200)) {     // poll like waitStill: still = under 0.1 deg in 200 ms
    if (gripPhase == 0 && t >= 1000) { grip(GRIPPER_CLOSED_US); gripPhase = 1; }
    if (gripPhase == 1 && t >= 2000) { openTo(GRIP_MID_PCT); gripPhase = 2; }
    bool all = true;
    for (uint8_t id : IDS) {
      if (!started[id] || still[id]) continue;
      if (!bus.readPosition(id)) { started[id] = false; continue; }
      if (fabsf(bus.m[id].deg - last[id]) < 0.1f) still[id] = true;
      else all = false;
      last[id] = bus.m[id].deg;
    }
    if (all && gripPhase == 2) break;
  }
  if (gripPhase < 2) openTo(GRIP_MID_PCT);
  for (uint8_t id : IDS) {
    bool ok = started[id] && still[id];
    homed[id] = ok;
    if (ok) {
      originDeg[id] = last[id];
      zeroDeg[id] = originDeg[id] + homeOffset[id];
      target[id] = 0;
      ok = bus.moveTo(id, zeroDeg[id]);
    }
    Serial.printf("motor %d: home %s, arm zero %.2f deg from the driver origin\n", id, ok ? "ok" : "FAILED", homeOffset[id]);
  }
}

// ---------- teach and replay ----------
// A take is the commanded pose every TAKE_MS: joints 1-4 (deg from the arm zero), turret (deg), gripper (%),
// as hundredths in int16. On the card: TAKEnnn.BIN = 3 x uint32 header (TAKE_MAGIC, TAKE_MS, samples), then the samples.
const int TAKE_MS = 10, PLAY_MS = 20;       // record at 100 Hz, send the replay at 50 Hz
const int TAKE_MAX = 30000, NV = 6;         // 5 min of samples, 360 kB in RAM2
const uint32_t TAKE_MAGIC = 0x31414B54;     // "TKA1"
const float MOVE_IN_DPS = 20;               // speed of the move to a take's first pose; it takes 1 to 8 s
DMAMEM int16_t take[TAKE_MAX][NV];
int takeLen = 0, takeNo = 0;                // samples in RAM, its number on the card (0 = not saved)
enum { IDLE, RECORDING, MOVING_IN, PLAYING } rec = IDLE;
const char *REC_NAMES[] = {"idle", "rec", "move", "play"};
float playSpeed = 1, playPos = 0, startPose[NV];
uint32_t moveInMs = 0;
elapsedMillis recClock;                     // since the recording, move-in or replay started
bool sdOk = false;

bool replaying() { return rec == MOVING_IN || rec == PLAYING; }

// ---------- MPC for the replay: always on ----------
// Sees the take MPC_N ticks (0.5 s) ahead and sends setpoints that lead it, so the joints stop trailing the take
// by their loop's lag. Every setpoint stays within the lead of what the plain replay would send. A joint replays
// through the MPC once it has a model (mpc <id> <wn> <zeta>, from its step-test fit; the app sends them); the models
// and the lead are kept in EEPROM, so a reboot keeps them. A joint without a model replays plainly.
const int MPC_N = 25;
static_assert(PLAY_MS == 20, "JointMPC::DT is the replay tick");
JointMPC<MPC_N> mpc[MAX_ID + 1];
uint32_t mpcUs = 0;                         // the slowest MPC tick of this replay: every joint's read, observe, solve
const uint32_t MPC_MAGIC = 0x3143504D;      // "MPC1"
struct MpcStore { uint32_t magic; float lead, wn[MAX_ID + 1], zeta[MAX_ID + 1]; };

bool mpcValid(float wn, float z) {          // inside what JointMPC's 1 ms substeps integrate stably (kv h < 2)
  return wn >= 0.5f && wn <= 150 && z >= 0.1f && z <= 4;
}

void mpcSave() {
  MpcStore st = {MPC_MAGIC, mpc[1].lead, {}, {}};
  for (uint8_t id : IDS) {
    st.wn[id] = mpc[id].ready ? mpc[id].wn : 0;
    st.zeta[id] = mpc[id].ready ? mpc[id].zeta : 0;
  }
  EEPROM.put(0, st);
}

void mpcLoad() {
  MpcStore st;
  EEPROM.get(0, st);
  if (st.magic != MPC_MAGIC) {
    Serial.println("mpc: no joint models stored yet (the app sends them)");
    return;
  }
  for (uint8_t id : IDS) {
    if (st.lead >= 2 && st.lead <= 40) mpc[id].lead = st.lead;
    if (mpcValid(st.wn[id], st.zeta[id])) mpc[id].setModel(st.wn[id], st.zeta[id]);
  }
  Serial.print("mpc: models from EEPROM ");
  for (uint8_t id : IDS) Serial.printf(" %d:%.2f:%.2f", id, mpc[id].ready ? mpc[id].wn : 0, mpc[id].ready ? mpc[id].zeta : 0);
  Serial.println();
}

void mpcStart() {                           // the replay proper begins: each observer from where its joint is
  mpcUs = 0;
  for (uint8_t id : IDS) {
    if (!mpc[id].ready) continue;
    bool ok = bus.readPosition(id);
    mpc[id].reset(ok ? bus.m[id].deg - zeroDeg[id] : target[id]);
  }
}

void sampleAt(float pos, float *q);

void mpcPose(float *q) {                    // q's CAN joints become the MPC's setpoints, on joints with a model
  uint32_t t0 = micros();
  static float ref[4][MPC_N];
  float s[NV], ahead = PLAY_MS * playSpeed / TAKE_MS;  // take samples per tick
  for (int k = 0; k < MPC_N; k++) {
    sampleAt(min(playPos + (k + 1) * ahead, (float)(takeLen - 1)), s);
    for (int i = 0; i < 4; i++) ref[i][k] = s[i];
  }
  for (int i = 0; i < 4; i++) {
    uint8_t id = IDS[i];
    if (!mpc[id].ready || jammed[id]) continue;
    bool ok = bus.readPosition(id);
    mpc[id].observe(ok ? bus.m[id].deg - zeroDeg[id] : 0, ok);
    q[i] = mpc[id].step(ref[i]);
  }
  mpcUs = max(mpcUs, micros() - t0);
}

void mpcCommand(String a) {
  a.trim();
  int id;
  float wn, z;
  if (a.startsWith("lead ")) {
    float l = a.substring(5).toFloat();
    if (l >= 2 && l <= 40) {
      for (auto &m : mpc) m.lead = l;
      mpcSave();
    }
  } else if (a == "test") {
    uint32_t us;
    bool ok = JointMPC<MPC_N>::selfTest(Serial, us);
    gateway.printf("Q,test,%d,%lu\n", ok, (unsigned long)us);
    return;
  } else if (sscanf(a.c_str(), "%d %f %f", &id, &wn, &z) == 3 && id >= 1 && id <= MAX_ID) {
    if (!mpcValid(wn, z)) {
      Serial.printf("mpc: motor %d model wn %.2f zeta %.2f refused (wn 0.5-150, zeta 0.1-4)\n", id, wn, z);
    } else if (!mpc[id].ready || fabsf(mpc[id].wn - wn) > 1e-3f || fabsf(mpc[id].zeta - z) > 1e-3f) {
      mpc[id].setModel(wn, z);
      if (rec == PLAYING) mpc[id].reset(target[id]);
      mpcSave();
    }
  } else if (a.length()) {
    Serial.println("mpc: <id> <wn> <zeta> | lead <deg> | test");
  }
  char line[160];
  int n = snprintf(line, sizeof line, "Q,on,%.1f,%lu", mpc[1].lead, (unsigned long)mpcUs);
  for (uint8_t i : IDS)
    n += snprintf(line + n, sizeof line - n, ",%d:%.2f:%.2f", i, mpc[i].ready ? mpc[i].wn : 0, mpc[i].ready ? mpc[i].zeta : 0);
  gateway.println(line);
  Serial.printf("mpc: lead %.1f deg, slowest tick %lu us; models (id:wn:zeta) %s\n", mpc[1].lead,
                (unsigned long)mpcUs, line + 5);
}

void say(const char *msg) { Serial.printf("take: %s\n", msg); }

// The take at sample position pos, smoothed by a centred moving average (zero phase, so no lag) over +-50 ms,
// which rounds off the 20 Hz steps of the app's commands, and interpolated between samples.
void sampleAt(float pos, float *q) {
  const int W = 5;
  int i0 = (int)pos;
  float f = pos - i0;
  for (int k = 0; k < NV; k++) {
    float a = 0, b = 0;
    for (int j = -W; j <= W; j++) {
      a += take[constrain(i0 + j, 0, takeLen - 1)][k];
      b += take[constrain(i0 + 1 + j, 0, takeLen - 1)][k];
    }
    q[k] = (a + (b - a) * f) / (2 * W + 1) / 100.0f;
  }
}

// The replay drives the arm as the app's a / t / p commands would. u: the setpoints sent (the MPC's, or the take's),
// ref: the take, reported as the target, so telemetry and jam detection measure against the take in both modes.
void applyPose(const float *u, const float *ref) {
  for (int i = 0; i < 4; i++) {
    uint8_t id = IDS[i];
    if (jammed[id]) continue;
    target[id] = ref[i];
    bus.moveTo(id, zeroDeg[id] + u[i]);
  }
  turn(u[4]);
  openTo(u[5]);
}

bool saveTake() {
  if (!sdOk || !takeLen) return false;
  char name[16];
  int n = 1;
  while (snprintf(name, sizeof name, "TAKE%03d.BIN", n), SD.exists(name) && n < 999) n++;
  File f = SD.open(name, FILE_WRITE);
  if (!f) return false;
  uint32_t hdr[3] = {TAKE_MAGIC, TAKE_MS, (uint32_t)takeLen};
  size_t bytes = takeLen * sizeof take[0];
  bool ok = f.write((uint8_t *)hdr, sizeof hdr) == sizeof hdr && f.write((uint8_t *)take, bytes) == bytes;
  f.close();
  if (ok) takeNo = n;
  return ok;
}

bool takeHeader(File &f, uint32_t *hdr) {   // true if f is a take this sketch can play
  return f.read(hdr, 12) == 12 && hdr[0] == TAKE_MAGIC && hdr[1] == TAKE_MS && hdr[2] && hdr[2] <= TAKE_MAX;
}

bool loadTake(int n) {
  char name[16];
  snprintf(name, sizeof name, "TAKE%03d.BIN", n);
  File f = sdOk ? SD.open(name) : File();
  uint32_t hdr[3];
  if (!f || !takeHeader(f, hdr)) return false;
  size_t bytes = hdr[2] * sizeof take[0];
  bool ok = f.read(take, bytes) == bytes;
  f.close();
  takeLen = ok ? hdr[2] : 0;                // a half-read take is no take
  takeNo = ok ? n : 0;
  return ok;
}

void listTakes() {                          // K,n,seconds for every take on the card, then K,end,count
  int count = 0;
  File dir = sdOk ? SD.open("/") : File();
  for (File e; dir && (e = dir.openNextFile()); e.close()) {
    int n;
    uint32_t hdr[3];
    if (sscanf(e.name(), "TAKE%3d.BIN", &n) != 1 || !takeHeader(e, hdr)) continue;
    gateway.printf("K,%d,%.2f\n", n, hdr[2] * TAKE_MS / 1000.0f);
    Serial.printf("take %d: %.1f s\n", n, hdr[2] * TAKE_MS / 1000.0f);
    count++;
  }
  gateway.printf("K,end,%d\n", count);
}

void deleteTake(int n) {                    // del <n>: TAKEn off the card, then the list again
  char name[16];
  snprintf(name, sizeof name, "TAKE%03d.BIN", n);
  bool ok = sdOk && rec == IDLE && SD.exists(name) && SD.remove(name);
  if (ok && takeNo == n) takeNo = 0;        // its copy in RAM stays, as a take not saved
  Serial.printf("take %d: %s\n", n, ok ? "deleted from the card" : rec != IDLE ? "not deleted while recording or replaying"
                                                                             : "not found on the card");
  listTakes();
}

void stopTake() {                           // ends a recording (and saves it) or a replay (the arm holds)
  if (rec == RECORDING) {
    rec = IDLE;
    bool saved = saveTake();
    Serial.printf("take: recorded %.1f s, %s\n", takeLen * TAKE_MS / 1000.0f,
                  saved ? "saved on the card" : sdOk ? "SAVING FAILED, kept in RAM" : "no card, kept in RAM");
  } else if (replaying()) {
    bool viaMpc = rec == PLAYING;
    rec = IDLE;
    if (viaMpc)  // an MPC setpoint can lead the take by up to the lead: hold on the take, as a plain replay would
      for (uint8_t id : IDS)
        if (mpc[id].ready && !jammed[id]) bus.moveTo(id, zeroDeg[id] + target[id]);
    say("replay stopped, holding");
  }
}

void startRecording() {
  stopTake();
  takeLen = takeNo = 0;
  recClock = 0;
  rec = RECORDING;
  say("recording");
}

void startReplay(float speedPct) {
  if (!takeLen) return say("nothing to play: record or load a take");
  stopTake();
  float first[NV], far = 0;
  sampleAt(0, first);
  for (int i = 0; i < 4; i++) {             // the move-in starts from where the joints really are
    uint8_t id = IDS[i];
    startPose[i] = bus.readPosition(id) ? bus.m[id].deg - zeroDeg[id] : target[id];
    far = max(far, fabsf(first[i] - startPose[i]));
  }
  startPose[4] = turretDeg;
  startPose[5] = gripPct;
  far = max(far, fabsf(first[4] - turretDeg));
  moveInMs = constrain(far / MOVE_IN_DPS * 1000, 1000.0f, 8000.0f);
  playSpeed = constrain(speedPct, 10.0f, 200.0f) / 100;
  playPos = 0;
  recClock = 0;
  rec = MOVING_IN;
  Serial.printf("take: moving to the first pose in %.1f s, then replaying at %.0f %%\n", moveInMs / 1000.0f,
                playSpeed * 100);
}

float minJerk(float s) {                    // 0..1 -> 0..1 with zero speed and acceleration at both ends
  s = constrain(s, 0.0f, 1.0f);
  return s * s * s * (10 - 15 * s + 6 * s * s);
}

void takeTick() {
  if (rec == RECORDING) {                   // every TAKE_MS of time gets a sample, even across a slow loop
    int upTo = min((int)(recClock / TAKE_MS) + 1, TAKE_MAX);
    float q[NV] = {target[1], target[2], target[3], target[4], turretDeg, gripPct};
    for (; takeLen < upTo; takeLen++)
      for (int k = 0; k < NV; k++) take[takeLen][k] = lroundf(q[k] * 100);
    if (takeLen >= TAKE_MAX) stopTake();    // full: 5 min
    return;
  }
  static elapsedMillis tick;
  if (!replaying() || tick < PLAY_MS) return;
  tick = 0;
  for (uint8_t id : IDS)
    if (jammed[id]) return stopTake();      // something is in the way
  float q[NV];
  bool viaMpc = false;
  if (rec == MOVING_IN) {
    float first[NV], s = minJerk(recClock / (float)moveInMs);
    sampleAt(0, first);
    for (int k = 0; k < NV; k++) q[k] = startPose[k] + (first[k] - startPose[k]) * s;
    if (recClock >= moveInMs) {
      rec = PLAYING;
      recClock = 0;
      mpcStart();
    }
  } else {
    playPos = recClock * playSpeed / TAKE_MS;  // from the clock, so a slow loop does not stretch the take
    if (playPos >= takeLen - 1) {
      playPos = takeLen - 1;
      rec = IDLE;
      say("replay done");
    }
    sampleAt(playPos, q);
    viaMpc = rec == PLAYING;                // the last pose goes out plain, so the arm ends where the take does
  }
  float u[NV];
  memcpy(u, q, sizeof u);
  if (viaMpc) mpcPose(u);                   // the drivers get the MPC's setpoints; q stays the take
  applyPose(u, q);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.printf("decoder self-test: %s\n", bus.selfTest() ? "ok" : "FAILED");
  gateway.begin(GATEWAY_BAUD);
  gateway.addMemoryForRead(gatewayRx, sizeof gatewayRx);
  gripper.attach(GRIPPER_PIN);
  grip(GRIPPER_MID_US);
  turret.attach(TURRET_PIN);
  turn(0);
  mpcLoad();
  zeroLoad();
  sdOk = SD.begin(BUILTIN_SDCARD);
  Serial.printf("SD card: %s\n", sdOk ? "ok, takes are saved there" : "NOT FOUND, takes only live in RAM");
  bus.begin(CAN_BAUD);

  for (uint8_t id : IDS) {
    if (!bus.ping(id)) { Serial.printf("motor %d: no reply\n", id); continue; }
    bus.status(id);
    bus.readMotorInfo(id);
    const GDZMotor &s = bus.m[id];
    Serial.printf("motor %d: fw %d, %.1f V, %d C, gear %d, kt %.3f Nm/A, fault 0x%02X\n",
                  id, s.fw, s.volts, s.tempC, s.gear, s.kt, s.fault);
  }
  homeAll();
}

// echo = false for gateway commands: the app streams them, so the USB console would drown in replies.
void command(String s, bool echo) {
  s.trim();
  if (s == "rec") return startRecording();
  if (s == "stop") return stopTake();
  if (s.startsWith("play")) return startReplay(s.length() > 4 ? s.substring(4).toFloat() : 100);
  if (s.startsWith("load ")) {
    int n = s.substring(5).toInt();
    Serial.printf("take %d: %s\n", n, loadTake(n) ? "loaded" : "not found on the card");
    return;
  }
  if (s == "takes") return listTakes();
  if (s.startsWith("del ")) return deleteTake(s.substring(4).toInt());
  if (s == "mpc" || s.startsWith("mpc ")) return mpcCommand(s.substring(3));
  int id; char op; float v = 0;
  bool joint = sscanf(s.c_str(), "%d %c%f", &id, &op, &v) >= 2;
  if (replaying() && (s.startsWith("t ") || s.startsWith("p ") || s == "x" || s == "h" ||
                      (joint && strchr("arshoz", op)))) {
    stopTake();                     // a manual move takes over from the replay
  }
  if (s.startsWith("t ")) {
    float deg = s.substring(2).toFloat();
    turn(deg);
    if (echo) Serial.printf("turret %.1f deg\n", deg);
    return;
  }
  if (s.startsWith("p ")) {
    sweeping = false;
    openTo(s.substring(2).toFloat());
    if (echo) Serial.printf("gripper %.0f %% open\n", gripPct);
    return;
  }
  if (s == "s") {
    sweeping = !sweeping;
    if (!sweeping) grip(GRIPPER_MID_US);
    Serial.printf("gripper sweep %s\n", sweeping ? "on" : "off");
    return;
  }
  if (s == "j" || s.startsWith("j ")) {
    float deg;
    int ms;
    if (sscanf(s.c_str(), "j %f %d", &deg, &ms) == 2 && deg > 0 && ms >= 0) { jamErrDeg = deg; jamMs = ms; }
    gateway.printf("J,%.2f,%d\n", jamErrDeg, jamMs);
    if (echo) Serial.printf("jam detection: %.2f deg for %d ms%s\n", jamErrDeg, jamMs, jamMs ? "" : " (off)");
    return;
  }
  if (s == "x") { bus.offAll(); Serial.println("all motors released"); return; }
  if (s == "h") { homeAll(); return; }
  if (s == "d") { bus.dump = !bus.dump; Serial.printf("frame dump %s\n", bus.dump ? "on" : "off"); return; }
  if (!joint || id < 1 || id > MAX_ID) { Serial.println("? see commands at top of sketch"); return; }
  if (op == 'h') { homeOne(id); return; }
  if (op == 'z') { zeroHere(id); return; }
  if (op == 'g') {
    float k[4];
    if (sscanf(s.c_str(), "%*d g %f %f %f %f", &k[0], &k[1], &k[2], &k[3]) == 4)
      for (int i = 0; i < 4; i++) bus.setGain(id, bus.POS_KP + i, k[i]);
    reportGains(id, echo);          // read back, so the app shows what the driver really holds
    return;
  }
  if ((op == 'a' || op == 'r' || op == 's') && jammed[id]) {
    if (echo) Serial.printf("motor %d is jammed: '%d c' clears it\n", id, id);
    return;
  }
  if (op == 's') { stepTest(id, v, echo); return; }
  if (op == 'c') { jammed[id] = false; stuckMs[id] = 0; }
  if (op == 'v') rpmSet[id] = v;
  if (op == 'a') target[id] = v;
  if (op == 'r') target[id] += v;
  bool ok = op == 'a' ? bus.moveTo(id, zeroDeg[id] + v)
          : op == 'r' ? bus.moveBy(id, v)
          : op == 'v' ? bus.setMaxSpeed(id, v)
          : op == 'm' ? bus.setMaxCurrent(id, v)
          : op == 'o' ? bus.off(id)
          : op == 'c' ? bus.clearFault(id) && bus.status(id)
          : false;
  if (echo) Serial.printf("motor %d %c %.2f: %s, fault 0x%02X\n", id, op, v, ok ? "ok" : "failed", bus.m[id].fault);
}

void loop() {
  if (Serial.available()) command(Serial.readStringUntil('\n'), true);
  if (gateway.available()) command(gateway.readStringUntil('\n'), false);
  takeTick();

  static elapsedMillis st;          // gripper sweep: 10 us every 20 ms, about 1.3 s end to end
  static int us = GRIPPER_MID_US, dir = 10;
  if (sweeping && st >= 20) {
    st = 0;
    us += dir;
    if (us >= GRIPPER_OPEN_US || us <= GRIPPER_CLOSED_US) dir = -dir;
    grip(us);
  }

  static elapsedMillis t;
  if (t < TELEMETRY_MS) return;
  t = 0;
  for (uint8_t id : IDS) {
    bool ok = bus.readPosition(id) && bus.readCurrent(id) && bus.status(id);
    const GDZMotor &s = bus.m[id];
    if (ok) checkJam(id, s.deg - zeroDeg[id]);
    else stuckMs[id] = 0;
    gateway.printf("T,%d,%d,%.2f,%.2f,%.3f,%d,%.2f,%.2f,%d,%d,%d\n", id, ok, s.deg - zeroDeg[id], target[id], s.amps,
                   s.tempC, s.volts, s.busAmps, s.mode, s.fault, jammed[id]);
    if (!ok) { Serial.printf("%d: --  ", id); continue; }
    Serial.printf("%d: %7.2f deg %5.2f A %s f%02X  ", id, s.deg - zeroDeg[id], s.amps, s.mode < 5 ? MODES[s.mode] : "?", s.fault);
  }
  Serial.println();
  float in = rec == RECORDING ? takeLen * TAKE_MS / 1000.0f : rec == PLAYING ? playPos * TAKE_MS / 1000.0f : 0;
  gateway.printf("R,%s,%d,%.2f,%.2f,%d\n", REC_NAMES[rec], takeNo, in, takeLen * TAKE_MS / 1000.0f, sdOk);
}
