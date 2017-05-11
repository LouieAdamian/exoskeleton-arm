#include "arduino.h"
#include <RunningMedian.h>
#define emgPin A0
#define encoderPinA 4
#define encoderPinB 5
int tp, offset, err, lasterr, integral, derivative, speed;
float Kp, Ki, Kd, move;
offset = null;
Kp = null;
Ki = null;
Kd = null;
steer = null;
RunningMedian samples =RunningMedian(9);
long emg;
void setup() {
Serial.begin(115200);
Serial.print("init")
}
void loop() {
  // put your main code here, to run repeatedly:
emg = analogRead(emgPin);
serial.println(emg);
samples.add(emg);
long emgM = samples.getMedian();
Serial.println(emgM);
err = offset - emg;
derivative = err - lasterr;
move = Kp * err + ki * intergral + Kd *derivative;
// move motor
lasterr = err
}
