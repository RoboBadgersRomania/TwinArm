// TwinArm master arm on a LightGateway 1.0 (ESP32-S3): reads the arm, sends it to the follower's gateway over the
// encrypted, frequency-hopping TwinLink radio (TwinLink.h), and is the app's radio link over USB.
// Sensors: a TCA9548A at 0x70 on either I2C header (SDA IO3, SCL IO4, 3V3, GND; A0..A2 to GND), one MT6701 per
//          channel: SD0 / SC0 joint 1, SD1 / SC1 joint 2, SD2 / SC2 joint 3.
//          Potentiometers, outer legs on 3V3 and GND: wiper on GPIO1 = gripper, wiper on GPIO2 = turret left / right.
// USB serial 115200, where the app connects when its gateway field says COMxx:
//   out  M,<j1>,<j2>,<j3>,<gripper pot %>,<turret pot %>,<ok bits>   20 times a second; joints 0..360 deg, -1 = no reading
//        L,m,...   the radio link once a second (see TwinLink.h);  every line the follower's gateway sends back
//   in   #key [<32 hex digits>]   shows / sets the radio key (it never leaves this board);  #radio  frame counts;
//        #test   radio self-test
//        any other line goes over the radio to the follower's gateway, as if the app had sent it there over WiFi
// Board: "ESP32S3 Dev Module", USB CDC On Boot enabled. Libraries: RadioLib, TwinLink (ArduinoCode/libraries).
#ifndef ARDUINO_ARCH_ESP32
#error "Wrong board: choose Tools > Board > ESP32S3 Dev Module. This sketch runs on the LightGateway, not the Teensy."
#endif
#include <Wire.h>
#include <TwinLink.h>

const int SDA_PIN = 3, SCL_PIN = 4;     // the LightGateway's I2C header
const uint8_t MUX = 0x70;               // TCA9548A with A0..A2 low
const uint8_t MT6701 = 0x06;            // MT6701 factory I2C address
const uint8_t JOINT_CH[] = {0, 1, 2};   // mux channel of joint 1, 2, 3
const int POT_PIN[] = {1, 2};           // gripper, turret (ADC1 pins)
const int NJ = sizeof JOINT_CH, NP = sizeof POT_PIN / sizeof POT_PIN[0];

TwinLink lora;
char line[256];
size_t len = 0;

bool present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool selectChannel(uint8_t ch) {
  Wire.beginTransmission(MUX);
  Wire.write(1 << ch);                  // one bit per channel; only this one connected
  return Wire.endTransmission() == 0;
}

// 14-bit angle, 0..16383 per turn: register 0x03 holds bits 13..6, register 0x04 bits 5..0 in its top 6 bits.
// Both bytes in one read so they belong to the same sample. -1 if the sensor did not answer.
int readRaw() {
  Wire.beginTransmission(MT6701);
  Wire.write(0x03);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(MT6701, (uint8_t)2) != 2) return -1;
  uint8_t hi = Wire.read(), lo = Wire.read();
  return hi << 6 | lo >> 2;
}

// The arm into the radio's state bytes, the layout masterLine() reads: per joint its raw angle, per pot 0..4095.
void readArm() {
  for (int j = 0; j < NJ; j++) {
    int raw = selectChannel(JOINT_CH[j]) ? readRaw() : -1;
    put16(lora.state + 2 * j, raw < 0 ? 0xFFFF : raw);
  }
  for (int p = 0; p < NP; p++) put16(lora.state + 2 * NJ + 2 * p, analogRead(POT_PIN[p]));
}

void setup() {
  Serial.begin(115200);
  delay(1000);                          // let the USB serial monitor attach
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  Serial.printf("TCA9548A at 0x%02X: %s\n", MUX, present(MUX) ? "ok" : "NOT FOUND (check SDA/SCL/3V3, A0..A2 to GND)");
  for (int j = 0; j < NJ; j++) {
    selectChannel(JOINT_CH[j]);
    Serial.printf("joint %d MT6701 on channel %d: %s\n", j + 1, JOINT_CH[j],
                  present(MT6701) ? "ok" : "NOT FOUND (check its SD/SC wiring and the sensor's power)");
  }
  lora.begin(true);
}

void loop() {
  lora.poll();
  static uint32_t readAt = 0, statAt = 0;
  char out[200];
  if (millis() - readAt >= 50) {
    readAt = millis();
    readArm();
    masterLine(lora.state, out, sizeof out);
    Serial.println(out);
  }
  if (millis() - statAt >= 1000) {
    statAt = millis();
    lora.status(out, sizeof out);
    Serial.println(out);
  }
  while (lora.read(out, sizeof out)) Serial.println(out);  // from the follower's gateway, for the app

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (len < sizeof line - 1) line[len++] = c;  // an overlong line is cut
      continue;
    }
    line[len] = 0;
    len = 0;
    if (line[0] && !lora.command(line, Serial)) lora.write(line);  // dropped while the radio link is down
  }
}
