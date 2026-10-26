// TwinArm motor test: finds drivers 1-4, homes them to the arm's zero (driver origin + HOME_OFFSET) and holds.
// All angles typed and printed are relative to that arm zero.
// Serial monitor (send with newline):
//   x = release all   h = home all   d = toggle raw CAN frame dump   s = toggle gripper sweep
//   <id> a<deg>  go to angle      1 a90
//   <id> r<deg>  move relative    1 r-30
//   <id> v<rpm>  max move speed   1 v30
//   <id> h       home one         1 h
//   <id> o       release one      1 o
//   <id> c       clear fault      1 c   (shows the fault code left afterwards)
#include "GDZ.h"
#include <Servo.h>

Servo gripper;                      // DS 40 kg servo, signal on pin 6, powered from its own supply
const int GRIPPER_PIN = 6;
const int GRIPPER_MID_US = 1500;    // calibrate: pulse width for the middle position
const int GRIPPER_MIN_US = 1000, GRIPPER_MAX_US = 2000;  // calibrate: sweep ends, widen carefully (gripper may bind)
bool sweeping = true;               // 's' toggles; stopping returns to middle

GDZBus<CAN1> bus;                   // CAN1: pin 22 -> TJA1051 TXD, pin 23 <- RXD
const uint8_t IDS[] = {1, 2, 3, 4};
const uint32_t CAN_BAUD = 1000000;  // driver default; must match the CAN baud set in the ZE300 GUI
const float HOME_RPM = 10;          // calibrate: speed limit for the homing move (driver forgets it at power-off)
const int MAX_ID = 4;
const float HOME_OFFSET[MAX_ID + 1] = {0,0,5, 0, 45};  // [id]: degrees from driver origin to arm zero
float zeroDeg[MAX_ID + 1];          // driver angle of each arm zero, set by homing

const char *MODES[] = {"off", "volt", "current", "speed", "pos"};

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
  bus.setMaxSpeed(id, HOME_RPM);
  bool ok = bus.home(id) && waitStill(id);
  if (ok) {
    zeroDeg[id] = bus.m[id].deg + HOME_OFFSET[id];
    ok = bus.moveTo(id, zeroDeg[id]);
  }
  Serial.printf("motor %d: home %s\n", id, ok ? "ok" : "FAILED");
}

void homeAll() {
  for (uint8_t id : IDS) homeOne(id);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.printf("decoder self-test: %s\n", bus.selfTest() ? "ok" : "FAILED");
  gripper.attach(GRIPPER_PIN);
  gripper.writeMicroseconds(GRIPPER_MID_US);
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

void command(String s) {
  s.trim();
  if (s == "s") {
    sweeping = !sweeping;
    if (!sweeping) gripper.writeMicroseconds(GRIPPER_MID_US);
    Serial.printf("gripper sweep %s\n", sweeping ? "on" : "off");
    return;
  }
  if (s == "x") { bus.offAll(); Serial.println("all motors released"); return; }
  if (s == "h") { homeAll(); return; }
  if (s == "d") { bus.dump = !bus.dump; Serial.printf("frame dump %s\n", bus.dump ? "on" : "off"); return; }
  int id; char op; float v = 0;
  if (sscanf(s.c_str(), "%d %c%f", &id, &op, &v) < 2 || id < 1 || id > MAX_ID) { Serial.println("? see commands at top of sketch"); return; }
  if (op == 'h') { homeOne(id); return; }
  bool ok = op == 'a' ? bus.moveTo(id, zeroDeg[id] + v)
          : op == 'r' ? bus.moveBy(id, v)
          : op == 'v' ? bus.setMaxSpeed(id, v)
          : op == 'o' ? bus.off(id)
          : op == 'c' ? bus.clearFault(id) && bus.status(id)
          : false;
  Serial.printf("motor %d %c %.2f: %s, fault 0x%02X\n", id, op, v, ok ? "ok" : "failed", bus.m[id].fault);
}

void loop() {
  if (Serial.available()) command(Serial.readStringUntil('\n'));

  static elapsedMillis st;          // gripper sweep: 10 us every 20 ms, about 2 s end to end
  static int us = GRIPPER_MID_US, dir = 10;
  if (sweeping && st >= 20) {
    st = 0;
    us += dir;
    if (us >= GRIPPER_MAX_US || us <= GRIPPER_MIN_US) dir = -dir;
    gripper.writeMicroseconds(us);
  }

  static elapsedMillis t;
  if (t < 200) return;
  t = 0;
  for (uint8_t id : IDS) {
    if (!bus.readPosition(id) || !bus.readCurrent(id) || !bus.status(id)) { Serial.printf("%d: --  ", id); continue; }
    const GDZMotor &s = bus.m[id];
    Serial.printf("%d: %7.2f deg %5.2f A %s f%02X  ", id, s.deg - zeroDeg[id], s.amps, s.mode < 5 ? MODES[s.mode] : "?", s.fault);
  }
  Serial.println();
}
