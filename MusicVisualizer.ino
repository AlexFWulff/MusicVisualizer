/*
  Written by Alex Wulff (www.AlexWulff.com), with code from:
  - http://wiki.openmusiclabs.com/wiki/ArduinoFFT
  - arduinoFFT example file FFT_03.ino
  - https://create.arduino.cc/projecthub/Shajeeb/32-band-audio-spectrum-visualizer-analyzer-902f51
*/

#include <arduinoFFT.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

const uint8_t LED_PIN = 2;
const uint8_t ENV_PIN = A1;

const uint16_t samples = 64;
const uint16_t num_pixels = 60;
const uint8_t loop_num = 1;
const uint16_t sound_thresh = 100;

arduinoFFT FFT = arduinoFFT();
Adafruit_NeoPixel strip = Adafruit_NeoPixel(num_pixels, LED_PIN, NEO_GRB + NEO_KHZ800);

double vReal[samples];
double vImag[samples];

void setup() {
  //TIMSK0 = 0; // turn off timer0 for lower jitter
  ADCSRA = 0xe5; // set the adc to free running mode
  ADMUX = 0x40; // use adc0
  DIDR0 = 0x01; // turn off the digital input for adc0

  strip.begin();
  strip.setBrightness(64);
  strip.show();

  pinMode(ENV_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  // collects the samples from the ADC and computes the average
  int freq_avg = 0;
  int max_avg = 0;

  // we sample and compute the FFT a few times to reduce noise
  for (int loop_count = 0; loop_count < loop_num; loop_count++) {
    double average = 0;
    for (int i = 0; i < samples; i++) {
      while (!(ADCSRA & 0x10)); // wait on ADIF bit
      ADCSRA = 0b11110101; // clear ADIF bit
      vReal[i] = (double)ADC; average += ADC;
      vImag[i] = 0;
    }

    average = average / samples;
    for (int i = 0; i < samples; i++) {
      vReal[i] = vReal[i] - average;
    }

    // windows the data and computes the FFT
    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, samples);

    // finds largest frequency component, and computes overall volume
    double max_val = 0;
    uint16_t frequency = 0;
    for (int i = 0; i < samples/2; i++) {
      if (vReal[i] > max_val) {
        max_val = vReal[i];
        frequency = (20000 / (samples/2)) * i;
      }
    }

    max_avg += max_val;
    freq_avg += frequency;
  }

  max_avg /= loop_num;
  freq_avg /= loop_num;
  
  const int range_max = 8000;
  if (freq_avg > range_max) freq_avg = range_max;
  int x = map(freq_avg,0,range_max,0,255);
  
  for (int i = 0; i < num_pixels; i++) {
    int red = -abs(2*(x-255))+255;
    int green = -abs(2*x)+255;
    int blue = -abs(2*(x-128))+255;
      
    if (red < 0) red = 0;
    if (green < 0) green = 0;
    if (blue < 0) blue = 0;

    if (max_avg < sound_thresh) strip.setPixelColor(i, strip.Color(10, 10, 10, 255));
    else strip.setPixelColor(i, strip.Color(red, green, blue, 255));
  }
  
  Serial.println(freq_avg);
  strip.show();
}
