// TwinArm master arm on a LightGateway 1.0 (ESP32-S3): reads the arm, sends it to the follower's gateway over the
// encrypted, frequency-hopping TwinLink radio (TwinLink.h), and is the app's radio link over USB.
// Sensors: a TCA9548A at 0x70 on either I2C header (SDA IO3, SCL IO4, 3V3, GND; A0..A2 to GND), one MT6701 per
//          channel: SD0 / SC0 turret (base left / right), SD1 / SC1 shoulder, SD2 / SC2 elbow, SD4 / SC4 wrist pitch.
//          Wrist-roll potentiometer (the gripper turning on its axis): outer legs on 3V3 and GND, wiper on GPIO1.
//          Gripper button between GPIO7 and GND: a toggle, each press closes the gripper, the next opens it.
// USB serial 115200, where the app connects when its gateway field says COMxx:
//   out  M,<turret>,<shoulder>,<elbow>,<wrist pitch>,<roll pot %>,<gripper 1 = closed>,<ok bits>   20 times a second;
//        joints 0..360 deg, -1 = no reading
//        L,m,...   the radio link once a second (see TwinLink.h);  every line the follower's gateway sends back
//   in   #key [<32 hex digits>]   shows / sets the radio key (it never leaves this board);  #radio  frame counts;
//        #test   radio self-test;  #scan  which mux channels have an MT6701
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
const uint8_t JOINT_CH[] = {0, 1, 2, 4};  // mux channel of the turret, shoulder, elbow, wrist pitch
const char *JOINT_NAME[] = {"turret", "shoulder", "elbow", "wrist pitch"};
const int NJ = sizeof JOINT_CH;
const int ROLL_POT = 1;                   // wrist-roll pot wiper (ADC1 pin)
const int GRIP_BUTTON = 7;                // gripper button to GND
const uint32_t DEBOUNCE_MS = 30;

TwinLink lora;
bool gripClosed = false;
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

// The arm into the radio's state bytes, the layout masterLine() reads: per joint its raw angle, the pot 0..4095.
void readArm() {
  for (int j = 0; j < NJ; j++) {
    int raw = selectChannel(JOINT_CH[j]) ? readRaw() : -1;
    put16(lora.state + 2 * j, raw < 0 ? 0xFFFF : raw);
  }
  put16(lora.state + 2 * NJ, analogRead(ROLL_POT));
}

void scan() {  // #scan: which mux channels have an MT6701, for checking the wiring
  for (int ch = 0; ch < 8; ch++) {
    selectChannel(ch);
    int raw = present(MT6701) ? readRaw() : -1;
    if (raw < 0) Serial.printf("scan: SD%d / SC%d  -\n", ch, ch);
    else Serial.printf("scan: SD%d / SC%d  MT6701, %.2f deg\n", ch, ch, raw * 360.0f / 16384);
  }
}

void readButton() {  // a press (to GND) toggles the gripper, straight into the next radio frame
  static bool was = HIGH;
  static uint32_t changedAt = 0;
  bool now = digitalRead(GRIP_BUTTON);
  if (now == was || millis() - changedAt < DEBOUNCE_MS) return;
  changedAt = millis();
  was = now;
  if (now == LOW) gripClosed = !gripClosed;
  lora.state[2 * NJ + 2] = gripClosed;
}

void setup() {
  Serial.begin(115200);
  delay(1000);                          // let the USB serial monitor attach
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  pinMode(GRIP_BUTTON, INPUT_PULLUP);
  Serial.printf("TCA9548A at 0x%02X: %s\n", MUX, present(MUX) ? "ok" : "NOT FOUND (check SDA/SCL/3V3, A0..A2 to GND)");
  for (int j = 0; j < NJ; j++) {
    selectChannel(JOINT_CH[j]);
    Serial.printf("%s MT6701 on channel %d: %s\n", JOINT_NAME[j], JOINT_CH[j],
                  present(MT6701) ? "ok" : "NOT FOUND (check its SD/SC wiring and the sensor's power)");
  }
  lora.begin(true);
}

void loop() {
  lora.poll();
  readButton();
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
    if (!strcmp(line, "#scan")) scan();
    else if (line[0] && !lora.command(line, Serial)) lora.write(line);  // dropped while the radio link is down
  }
}
