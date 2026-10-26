// GDZ.h - SteadyWin GDZ (ZE300 firmware) motor drivers on one CAN bus, "custom CAN protocol" v3.07.
// Teensy 4.x + FlexCAN_T4 + a CAN transceiver (TJA1051). Standard 11-bit frames, little-endian.
// Each call sends one frame to driver `id` and waits for its reply; decoded values land in bus.m[id].
// Units: angles in degrees (driver counts 16384 per turn), speed in RPM, current in A (torque = A * kt).
#pragma once
#include <FlexCAN_T4.h>

struct GDZMotor {                  // last values heard from one driver
  bool     online;                 // has replied at least once
  uint16_t fw;                     // app firmware, e.g. 3028 (readState needs >= 3028, MIT >= 3026)
  float    deg, singleDeg;         // multi-turn angle (resets at power-up) / single-turn 0..360
  float    rpm, amps;              // speed, Q-axis current
  float    volts, busAmps;
  uint8_t  tempC, mode, fault;     // mode: 0 off 1 voltage 2 current 3 speed 4 position
                                   // fault bits: 0 voltage 1 current 2 temp 3 encoder 6 hardware 7 software
  uint8_t  poles, gear;
  float    kt;                     // Nm/A
  float    mitPos, mitVel, mitTorque;  // MIT feedback: rad, rad/s, Nm
  uint8_t  mitStatus;                  // bit0 in MIT mode, bit1 fault
  float    pMax = 95.5f, vMax = 45.0f, tMax = 18.0f;  // MIT scaling, refresh with readMitLimits()
};

template <CAN_DEV_TABLE BUS>       // CAN1 = pins 22/23, CAN2 = 1/0, CAN3 = 31/30 (TX/RX)
class GDZBus {
 public:
  GDZMotor m[256];                 // indexed by driver address 1..254
  CAN_message_t rx;                // last frame received
  bool dump = false;               // print every frame sent/received to Serial

  void begin(uint32_t baud = 1000000) { can.begin(); can.setBaudRate(baud); can.enableFIFO(); }
  void poll() { while (can.read(rx)) parse(rx); }   // picks up the 200 ms fault reports

  // reads
  bool ping(uint8_t id)          { return req(id, 0xA0); }  // fw
  bool status(uint8_t id)        { return req(id, 0xAE); }  // volts, busAmps, tempC, mode, fault
  bool readPosition(uint8_t id)  { return req(id, 0xA3); }  // deg, singleDeg
  bool readSpeed(uint8_t id)     { return req(id, 0xA2); }  // rpm
  bool readCurrent(uint8_t id)   { return req(id, 0xA1); }  // amps
  bool readState(uint8_t id)     { return req(id, 0xA4); }  // tempC, amps, rpm, singleDeg in one frame
  bool readMotorInfo(uint8_t id) { return req(id, 0xB0); }  // poles, kt, gear

  // control: each command switches the driver into that mode and powers the motor
  bool current(uint8_t id, float a)  { return req32(id, 0xC0, a * 1000); }
  bool speed(uint8_t id, float rpm)  { return req32(id, 0xC1, rpm * 100); }
  bool moveTo(uint8_t id, float deg) { return req32(id, 0xC2, deg * CNT); }  // absolute, multi-turn
  bool moveBy(uint8_t id, float deg) { return req32(id, 0xC3, deg * CNT); }  // relative to now
  bool home(uint8_t id)              { return req(id, 0xC4); }  // shortest way (<= 180 deg) to origin, then holds
  bool off(uint8_t id)               { return req(id, 0xCF); }  // free-wheel, same as after power-up
  void offAll()                      { send(0, 0xCF); }         // broadcast address 0: all obey, none reply

  // settings: lost at power-off, except setZeroHere which is saved in the driver
  bool setMaxSpeed(uint8_t id, float rpm)     { return req32(id, 0xB2, rpm * 100); }   // position moves
  bool setMaxCurrent(uint8_t id, float a)     { return req32(id, 0xB3, a * 1000); }    // position/speed modes
  bool setCurrentSlope(uint8_t id, float aps) { return req32(id, 0xB4, aps * 1000); }  // current mode, 0 = instant
  bool setAccel(uint8_t id, float rpmps)      { return req32(id, 0xB5, rpmps * 100); } // speed mode, 0 = max
  bool setZeroHere(uint8_t id)                { return req(id, 0xB1, nullptr, 0, 100); }  // ~35 ms with 2nd encoder
  bool clearFault(uint8_t id)                 { return req(id, 0xAF); }
  bool brake(uint8_t id, uint8_t op)          { return req(id, 0xCE, &op, 1); }  // 0 open, 1 close, 0xFF read; state in rx.buf[1]
  void reboot(uint8_t id) { static const uint8_t k[7] = {0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF}; send(id, 0x00, k, 7); }

  enum { POS_KP = 0xB6, POS_KI, VEL_KP, VEL_KI };
  bool setGain(uint8_t id, uint8_t which, float v) { return req(id, which, &v, 4); }
  bool getGain(uint8_t id, uint8_t which, float &v) {
    if (!req(id, which)) return false;
    memcpy(&v, rx.buf + 1, 4);
    return true;
  }

  // MIT impedance mode (fw >= 3026): torque = kp*(pos - p) + kd*(vel - v) + t, in rad, rad/s, Nm.
  // Any other control command (or off) leaves it. Gains: kp 0..500, kd 0..5.
  bool readMitLimits(uint8_t id) { return req(id, 0xF0); }  // pMax, vMax, tMax
  bool readMit(uint8_t id)       { return req(id, 0xF1); }  // mitPos, mitVel, mitTorque, mitStatus
  bool mit(uint8_t id, float pos, float vel, float kp, float kd, float t) {
    const GDZMotor &s = m[id];
    uint16_t p = toU(pos, -s.pMax, s.pMax, 16), v = toU(vel, -s.vMax, s.vMax, 12),
             P = toU(kp, 0, 500, 12), D = toU(kd, 0, 5, 12), T = toU(t, -s.tMax, s.tMax, 12);
    CAN_message_t f;
    f.id = 0x400 | id;             // bit 10 marks an MIT frame (it has no command byte)
    f.buf[0] = p >> 8; f.buf[1] = p;
    f.buf[2] = v >> 4; f.buf[3] = v << 4 | P >> 8; f.buf[4] = P;
    f.buf[5] = D >> 4; f.buf[6] = D << 4 | T >> 8; f.buf[7] = T;
    return xfer(f, id, 0xF1);
  }

  // Decodes the example replies from the protocol PDF; false means the decoding is broken.
  bool selfTest() {
    static const uint8_t ex[4][8] = {{0xC0, 0xF5, 0x03},                 // 1.013 A
                                     {0xC1, 0x38, 0x27},                 // 100.40 rpm
                                     {0xC3, 0x00, 0x40, 0x00, 0x40},     // 360 deg single and multi
                                     {0xCF, 0x7C, 0x09, 0x01, 0, 0x26}}; // 24.28 V, 38 C
    for (auto &e : ex) { CAN_message_t f; f.id = 0; memcpy(f.buf, e, 8); parse(f); }
    const GDZMotor &s = m[0];      // address 0 never replies, so it's free scratch space
    return fabsf(s.amps - 1.013f) < 1e-4f && fabsf(s.rpm - 100.4f) < 1e-3f && fabsf(s.deg - 360) < 1e-3f &&
           fabsf(s.singleDeg - 360) < 1e-3f && fabsf(s.volts - 24.28f) < 1e-3f && s.tempC == 38 && s.mode == 0;
  }

 private:
  FlexCAN_T4<BUS> can;
  static constexpr float CNT = 16384 / 360.0f;  // counts per degree

  template <class T> static T get(const uint8_t *p) { T v; memcpy(&v, p, sizeof v); return v; }
  static uint16_t toU(float x, float lo, float hi, int bits) { return (constrain(x, lo, hi) - lo) * ((1 << bits) - 1) / (hi - lo); }
  static float toF(uint16_t u, float lo, float hi, int bits) { return u * (hi - lo) / ((1 << bits) - 1) + lo; }

  static CAN_message_t frame(uint8_t id, uint8_t cmd, const void *data, uint8_t len) {
    CAN_message_t f;
    f.id = id; f.len = len + 1; f.buf[0] = cmd;
    if (len) memcpy(f.buf + 1, data, len);
    return f;
  }
  void send(uint8_t id, uint8_t cmd, const void *data = nullptr, uint8_t len = 0) { can.write(frame(id, cmd, data, len)); }
  bool req(uint8_t id, uint8_t cmd, const void *data = nullptr, uint8_t len = 0, uint16_t ms = 10) {
    return xfer(frame(id, cmd, data, len), id, cmd, ms);
  }
  bool req32(uint8_t id, uint8_t cmd, float v) { int32_t x = lroundf(v); return req(id, cmd, &x, 4); }

  // Replies come back on the driver's own address with the same command byte.
  bool xfer(const CAN_message_t &f, uint8_t id, uint8_t want, uint16_t ms = 10) {
    poll();
    if (dump) print("TX", f);
    can.write(f);
    for (elapsedMillis t; t < ms;)
      while (can.read(rx)) {
        parse(rx);
        if (rx.id == id && rx.buf[0] == want) return true;
      }
    return false;
  }

  static void print(const char *dir, const CAN_message_t &f) {
    Serial.printf("%s %03X [%d]", dir, (unsigned)f.id, f.len);
    for (int i = 0; i < f.len; i++) Serial.printf(" %02X", f.buf[i]);
    Serial.println();
  }

  void parse(const CAN_message_t &f) {
    if (dump && f.id != 0) print("RX", f);
    if (f.id > 0xFF) return;
    GDZMotor &s = m[f.id];
    const uint8_t *b = f.buf;
    s.online = true;
    switch (b[0]) {
      case 0xA0: s.fw = get<uint16_t>(b + 3); break;
      case 0xA1: case 0xC0: s.amps = get<int32_t>(b + 1) * 0.001f; break;
      case 0xA2: case 0xC1: s.rpm = get<int32_t>(b + 1) * 0.01f; break;
      case 0xA3: case 0xC2: case 0xC3: case 0xC4:
        s.singleDeg = get<uint16_t>(b + 1) / CNT;
        s.deg = get<int32_t>(b + 3) / CNT;
        break;
      case 0xA4:
        s.tempC = b[1];
        s.amps = get<int16_t>(b + 2) * 0.001f;
        s.rpm = get<int16_t>(b + 4) * 0.01f;
        s.singleDeg = get<uint16_t>(b + 6) / CNT;
        break;
      case 0xAE: case 0xCF:
        s.volts = get<uint16_t>(b + 1) * 0.01f;
        s.busAmps = get<uint16_t>(b + 3) * 0.01f;
        s.tempC = b[5]; s.mode = b[6]; s.fault = b[7];
        break;
      case 0xAF: s.fault = b[1]; break;
      case 0xB0: s.poles = b[1]; s.kt = get<float>(b + 2); s.gear = b[6]; break;
      case 0xF0:
        s.pMax = get<uint16_t>(b + 1) * 0.1f;
        s.vMax = get<uint16_t>(b + 3) * 0.01f;
        s.tMax = get<uint16_t>(b + 5) * 0.01f;
        break;
      case 0xF1:
        s.mitPos = toF(b[1] << 8 | b[2], -s.pMax, s.pMax, 16);
        s.mitVel = toF(b[3] << 4 | b[4] >> 4, -s.vMax, s.vMax, 12);
        s.mitTorque = toF((b[4] & 0x0F) << 8 | b[5], -s.tMax, s.tMax, 12);
        s.mitStatus = b[6];
        break;
    }
  }
};
