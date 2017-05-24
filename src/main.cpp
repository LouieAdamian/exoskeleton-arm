#include "arduino.h"
#include <EEPROM.h>
#include <RunningMedian.h>
#define emgPin A0
#define motorPin1 2
#define motorPin2 3
#define encoderPinA 4
#define encoderPinB 5
int offset, err, lastErr, integral, derivative, speed, preset, read, emg;
float Kp, Ki,  Kd, move, newZero, emgA, lastemg;
RunningMedian samples = RunningMedian(5);
void setup() {
  Serial.begin(115200);
  Serial.print("init");
  preset = EEPROM.read(0);
}
void runMotor(int speed) {
analogWrite(motorPin2, 0);
analogWrite(motorPin1, speed);
}
void loop() {
  emg = analogRead(emgPin);
  samples.add(emg);
  emgA = samples.getAverage();
  Serial.println(emgA);
  if (lastemg - emgA > 150){
  err = offset - emgA;
  derivative = err - lastErr;
  move = Kp * err + Ki * integral + Kd *derivative;
  runMotor(move);
  lastErr = err;
  lastemg = emgA;
}
}
