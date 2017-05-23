#include "arduino.h"
#include <EEPROM.h>
#include <RunningMedian.h>
#define emgPin A0
#define motorPin1 2
#define motorPin2 3
#define encoderPinA 4
#define encoderPinB 5
int offset;
int err;
int lasterr;
int integral;
int derivative;
int speed;
bool preset;
float Kp ;
float Ki ;
float Kd ;
float move;
float newZero;
RunningMedian samples = RunningMedian(5);
RunningMedian samples2= RunningMedian(20);
long emg;
void tune(){
for (int i = 0; i<20; i++){
  int read = analogRead(A0);
  samples2.add(read);
}
newZero = samples.getMedian();
}
void setup() {
  Serial.begin(115200);
  Serial.print("init");
  preset = EEPROM.read(0);
  tune();
}
void runMotor(int speed) {
analogWrite(motorPin2, 0);
analogWrite(motorPin1, speed);
}
void loop() {
  emg = analogRead(emgPin);
  samples.add(emg);
  long emgM = samples.getMedian();
  Serial.println(emgM);
  err = offset - emgM;
  derivative = err - lasterr;
  move = Kp * err + Ki * integral + Kd *derivative;
  runMotor(move);
  lasterr = err;
}
