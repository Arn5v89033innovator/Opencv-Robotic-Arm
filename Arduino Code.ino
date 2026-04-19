#include <Servo.h>

Servo shoulder; // Pin 10
Servo elbow;    // Pin 9
Servo gripper;  // Pin 11

void setup() {
  Serial.begin(9600);
  shoulder.attach(10);
  elbow.attach(9);
  gripper.attach(11);
}

void loop() {
  if (Serial.available() > 0) {
    // Expecting data in format: SxxxExxxGxxx
    String data = Serial.readStringUntil('\n');

    int sIndex = data.indexOf('S');
    int eIndex = data.indexOf('E');
    int gIndex = data.indexOf('G');

    if (sIndex != -1 && eIndex != -1 && gIndex != -1) {
      int shoulderVal = data.substring(sIndex + 1, eIndex).toInt();
      int elbowVal = data.substring(eIndex + 1, gIndex).toInt();
      int gripperVal = data.substring(gIndex + 1).toInt();

      shoulder.write(shoulderVal);
      elbow.write(elbowVal);
      gripper.write(gripperVal);
    }
  }
}
