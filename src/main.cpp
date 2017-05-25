#include "arduino.h"
#include <RunningMedian.h>
// #include <arm_math.h>

#define emgPin A0
#define motorPin1 12
#define motorPin2 11
#define encoderPinA 4
#define encoderPinB 5
#define FFT_SIZE 256
#define ANALOG_READ_RESOLUTION 10
#define ANALOG_READ_AVERAGING 16
#define MAX_CHARS 65
int offset, err, lastErr, integral, derivative, speed, preset, read, emg, SAMPLE_RATE_HZ, SPECTRUM_MIN_DB, SPECTRUM_MAX_DB;
float Kp, Ki,  Kd, move, newZero, emgA, lastemg, emgf;

//Low pass butterworth filter order=3 alpha1=0.1875

//Low pass butterworth filter order=3 alpha1=0.46875
class  FilterBuLp3
{
	public:
		FilterBuLp3()
		{
			for(int i=0; i <= 3; i++)
				v[i]=0.0;
		}
	private:
		float v[4];
	public:
		float step(float x) //class II
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = v[3];
			v[3] = (8.214636347864285870e-1 * x)
				 + (-0.67480188729649281942 * v[0])
				 + (-2.28899391497678816876 * v[1])
				 + (-2.60791327601814737491 * v[2]);
			return
				 (v[0] + v[3])
				+3 * (v[1] + v[2]);
		}
};



FilterBuLp3 filter;

RunningMedian samples = RunningMedian(5);
void setup() {
  pinMode(emgPin, INPUT);
  // analogReadResolution(ANALOG_READ_RESOLUTION);
  // analogReadAveraging(ANALOG_READ_AVERAGING0);

  // memset(commandBuffer, 0, sizeof(commandBuffer));

  Serial.begin(9600);
  Serial.print("init");
}
void runMotor(int speed) {
  analogWrite(motorPin2, 0);
  analogWrite(motorPin1, speed);
}
// void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
//     *windowMean = 0;
//     *otherMean = 0;
//     // Notice the first magnitude bin is skipped because it represents the
//     // average power of the signal.
//     for (int i = 1; i < FFT_SIZE/2; ++i) {
//       if (i >= lowBin && i <= highBin) {
//         *windowMean += magnitudes[i];
//       }
//       else {
//         *otherMean += magnitudes[i];
//       }
//     }
//     *windowMean /= (highBin - lowBin) + 1;
//     *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin));
// } void samplingCallback() {
//   // Read from the ADC and store the sample data
//   samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
//   // Complex FFT functions require a coefficient for the imaginary part of the input.
//   // Since we only have real data, set this coefficient to zero.
//   samples[sampleCounter+1] = 0.0;
//   // Update sample buffer position and stop after the buffer is filled
//   sampleCounter += 2;
//   if (sampleCounter >= FFT_SIZE*2) {
//     samplingTimer.end();
//   }
// } void samplingBegin() {
//   // Reset sample buffer position and start callback at necessary rate.
//   sampleCounter = 0;
//   samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
// }   boolean samplingIsDone() {
//   return sampleCounter >= FFT_SIZE*2;
// } void FFT() {
//   if (samplingIsDone()) {
//     // Run FFT on sample data.
//     arm_cfft_radix4_instance_f32 fft_inst;
//     arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
//     arm_cfft_radix4_f32(&fft_inst, samples);
//     // Calculate magnitude of complex numbers output by the FFT.
//     arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);
//
//     if (LEDS_ENABLED == 1)
//     {
//       spectrumLoop();
//     }
//
//     // Restart audio sampling.
//     samplingBegin();
//   }
//   parserLoop();
// }
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
