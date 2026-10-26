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
//   <id> g [pos kp ki vel kp ki]  read, or write then read, the driver's loop gains   1 g 20 0.5 3 0.1
//   <id> m<amps> max current      1 m8
//   <id> s<deg>  step test: move by deg from where it is, sample every 10 ms for 2 s, send the curve   1 s5
//   t <deg>      turret angle     t -30  (0 = centre)
//   p <%>        gripper opening  p 40   (0 = closed, 100 = open, inside the hard limits)
//   j [deg ms]   jam detection: read, or set tolerance and time (ms 0 = off)   j 4 400
// The same commands arrive from the LightGateway on Serial2 (WiFi app), which gets one machine-readable
// line per motor every TELEMETRY_MS:  T,id,ok,deg,target,amps,tempC,volts,busAmps,mode,fault,jammed
// and answers to g and j:  P,id,ok,posKp,posKi,velKp,velKi   J,deg,ms
// and to s:  S,id,begin,start,target,sampleMs  then S,id,i,deg,deg,... (10 samples from index i per line)
//            then S,id,end  (or S,id,fail)
// A joint that stops short of its target (see checkJam) holds where it stopped and ignores a/r until cleared.
#include "GDZ.h"
#include <Servo.h>

Servo gripper;                      // DS 40 kg servo, signal on pin 6, powered from its own supply
const int GRIPPER_PIN = 6;
const int GRIPPER_MID_US = 1500;    // idle position
const int GRIPPER_CLOSED_US = 1350, GRIPPER_OPEN_US = 2000;  // hard limits: every pulse is clamped to these
bool sweeping = false;              // 's' toggles; stopping returns to idle

void grip(int us) { gripper.writeMicroseconds(constrain(us, GRIPPER_CLOSED_US, GRIPPER_OPEN_US)); }

Servo turret;                       // base turret: DS5180 80 kg servo, signal on pin 9, powered from its own supply
const int TURRET_PIN = 9;
const float TURRET_SPAN_DEG = 180;  // calibrate: degrees the turret turns between the 500 us and 2500 us pulses
const int TURRET_MIN_US = 500, TURRET_MAX_US = 2500;  // hard limits: every pulse is clamped to these

void turn(float deg) {
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
const float HOME_OFFSET[MAX_ID + 1] = {0,0,5, 0, 45};  // [id]: degrees from driver origin to arm zero
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
  if (ok) {
    zeroDeg[id] = bus.m[id].deg + HOME_OFFSET[id];
    target[id] = 0;
    ok = bus.moveTo(id, zeroDeg[id]);
  }
  Serial.printf("motor %d: home %s\n", id, ok ? "ok" : "FAILED");
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

// Gripper opens and closes once, settles at idle, then the motors home.
void homeAll() {
  grip(GRIPPER_OPEN_US);   delay(1000);
  grip(GRIPPER_CLOSED_US); delay(1000);
  grip(GRIPPER_MID_US);
  for (uint8_t id : IDS) homeOne(id);
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
  if (s.startsWith("t ")) {
    float deg = s.substring(2).toFloat();
    turn(deg);
    if (echo) Serial.printf("turret %.1f deg\n", deg);
    return;
  }
  if (s.startsWith("p ")) {
    float pct = constrain(s.substring(2).toFloat(), 0.0f, 100.0f);
    sweeping = false;
    grip(lroundf(GRIPPER_CLOSED_US + pct * (GRIPPER_OPEN_US - GRIPPER_CLOSED_US) / 100));
    if (echo) Serial.printf("gripper %.0f %% open\n", pct);
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
  int id; char op; float v = 0;
  if (sscanf(s.c_str(), "%d %c%f", &id, &op, &v) < 2 || id < 1 || id > MAX_ID) { Serial.println("? see commands at top of sketch"); return; }
  if (op == 'h') { homeOne(id); return; }
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
}
