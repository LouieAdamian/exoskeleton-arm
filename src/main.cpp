#include "arduino.h"
#include <RunningMedian.h>
#include <arm_math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <TimerOne.h>

#define emgPin A0
#define motorPin1 12
#define motorPin2 11
#define encoderPinA 4
#define encoderPinB 5
#define FFT_SIZE 256
#define ANALOG_READ_RESOLUTION 10
#define ANALOG_READ_AVERAGING 16
#define MAX_CHARS 65
//volatile int
int offset, err, lastErr, integral, derivative, speed, preset, read, emg, SAMPLE_RATE_HZ, SPECTRUM_MIN_DB, SPECTRUM_MAX_DB;
float Kp, Ki,  Kd, move, newZero, emgA, lastemg, emgf;

//Band pass butterworth filter order=3 alpha1=0.025 alpha2=0.15
class  FilterBuBp3{
public:
		FilterBuBp3()
		{
			for(int i=0; i <= 6; i++)
				v[i]=0.0;
		}
	private:
		float v[7];
	public:
		float step(float x) //class II
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = v[3];
			v[3] = v[4];
			v[4] = v[5];
			v[5] = v[6];
			v[6] = (3.170787110733085806e-2 * x)
				 + (-0.19782518726431930212 * v[0])
				 + (1.38788470731904034494 * v[1])
				 + (-4.20931786595035717369 * v[2])
				 + (7.17741585647423896432 * v[3])
				 + (-7.27499106118624716544 * v[4])
				 + (4.11519807365291967471 * v[5]);
			return
				 (v[6] - v[0])
				+3 * (v[2] - v[4]);
		}
};

FilterBuBp3 filter;

void interrupt(){

}
RunningMedian samples = RunningMedian(5);
void setup() {
  pinMode(emgPin, INPUT);
	Timer1.initialize(100);
	Timer1.start();
	Timer1.attachInterrupt(interrupt);
  Serial.begin(9600);
  Serial.print("init");
}
void runMotor(int speed) {
  analogWrite(motorPin2, 0);
  analogWrite(motorPin1, speed);
}

unsigned long prevTime =0;
void loop() {
  unsigned long currTime = micros();
  //Serial.println(currTime - prevTime);
  prevTime = currTime;
  emg = analogRead(emgPin);
  samples.add(emg);
  emgf = filter.step(emg);

  emgA = samples.getAverage();
  Serial.println(emgf);
    err = offset - emgA;
    derivative = err - lastErr;
    move = Kp * err + Ki * integral + Kd *derivative;
    runMotor(move);
    lastErr = err;
    lastemg = emgA;
}
