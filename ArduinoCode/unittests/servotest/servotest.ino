// Gripper servo test: type a pulse width in us in the serial monitor (send with newline), the servo on pin 6 goes there.
// Nothing else (no CAN, no homing). Starts at 1500. Servo on its own supply, ground shared with the Teensy.
#include <Servo.h>

Servo gripper;
const int GRIPPER_PIN = 6;
const int MIN_US = 500, MAX_US = 2500;  // typed values are clamped to this; the gripper may bind well inside it

void setup() {
  Serial.begin(115200);
  gripper.attach(GRIPPER_PIN);
  gripper.writeMicroseconds(1500);
}

void loop() {
  if (!Serial.available()) return;
  int us = Serial.parseInt();
  while (Serial.available()) Serial.read();  // drop the newline
  if (us == 0) return;
  us = constrain(us, MIN_US, MAX_US);
  gripper.writeMicroseconds(us);
  Serial.printf("%d us\n", us);
}
