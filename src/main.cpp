#include "arduino.h"
#include <RunningMedian.h>
#define emgPin A0
#define encoderPinA 4
#define encoderPinB 5
int offset;
int err;
int lasterr;
int integral;
int derivative;
int speed;
float Kp;
float Ki =0;
float Kd =0;
float move;
RunningMedian samples =RunningMedian(9);
long emg;
void setup() {
  Serial.begin(115200);
  Serial.print("init");
}
void loop() {
  // put your main code here, to run repeatedly:
  emg = analogRead(emgPin);
  Serial.println(emg);
  samples.add(emg);
  long emgM = samples.getMedian();
  Serial.println(emgM);
  err = offset - emg;
  derivative = err - lasterr;
  move = Kp * err + Ki * integral + Kd *derivative;
  // move motor(move)
  lasterr = err;
}
