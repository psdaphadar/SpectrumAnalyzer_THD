/*************************************************************
 Project : Spectrum Analyzer cum THD Meter using ATmega328P
 Author  : Partha Sarathi Daphadar
 Date    : 05-10-2025
 Copyright (c) 2025 Partha Sarathi Daphadar
 All Rights Reserved.

 LICENSE & TERMS OF USE:

 PERMISSION:
 - You may study, reference, and modify this code for personal,
   educational, or non-commercial research purposes only.
 - Proper credit must be given to the author when using this code.

 RESTRICTIONS:
 - Commercial use, redistribution, or inclusion in products or services
   is strictly prohibited without written permission from the author.
 - You may not sublicense, sell, or distribute this software, in whole
   or in part, without the author’s consent.

 DISCLAIMER OF WARRANTY:
 - This software is provided "as is," without any promises or guarantees.
 - The author does not guarantee that it will work correctly, be free of errors,
   or meet your specific needs.

 LIMITATION OF LIABILITY:
 - You use this software at your own risk.
 - The author is not responsible for any problems, damages, or losses
   that may happen from using it, including hardware damage, data loss,
   or personal injury.

*************************************************************/


#include "fix_fft.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

// ------------- OLED Settings --------------
#define OLED_RESET -1
#define OLED_ADDR 0x3C
Adafruit_SH1106 display(OLED_RESET);

// -------------- FFT Settings --------------
const int N = 128;               // FFT size (2^7)
int Fs = 2000;                   // Sampling rate (updated dynamically)
char data[N], im[N];
float samplesPerCycle = N / 7.0 + 1;

// ------------- Schmitt Trigger & Voltage Limits -------------
const int thresholdHigh = 530;
const int thresholdLow  = 510;
const int voltageMin    = 41;    // 0.2 V -> 0.2/5*1023
const int voltageMax    = 982;   // 4.8 V -> 4.8/5*1023

// ------------- Measurement Settings --------------
const int numEdges = 20;         // Edges to measure for frequency

// ------------- THD & Amplitude Buffers --------------
const int thdBufferSize = 5;
float thdBuffer[thdBufferSize] = {0};
int thdIndex = 0;
const int ampBufferSize = 3;
uint16_t ampBuffer[4][ampBufferSize]; // Stores A1–A4 harmonics
int ampIndex = 0;
int harmonicIndexArr[10]; // store fundamental + 9 harmonics

// -------------- Button Settings -------------
const int buttonPin = 8;             // Pushbutton pin
bool showOnlyHarmonics = false;      // Toggle flag
bool voltageOutOfRange = false;
bool errorStat = false;
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 1000; // 1 second lockout

// ------------- Frequency Smoothing -------------
static float freqSmooth = 40;
static int lastIndex = 0;

// -------------- Voltage Tracking ---------------
float minVoltA0 = 0, maxVoltA0 = 0;
float minVoltA1 = 0, maxVoltA1 = 0;
float mulFactor = 1;

// -------------- I2C Auto-Speed --------------
uint32_t bestI2CSpeed = 100000;

// Function to test if OLED ACKs at current I2C speed
bool testOledWrite()
{
 Wire.beginTransmission(OLED_ADDR);
 Wire.write(0x00);
 uint8_t err = Wire.endTransmission();
 return (err == 0);
}

// Automatically select the best I2C speed
void autoSetI2Cspeed()
{
 uint32_t speeds[] = {100000, 200000, 300000, 400000, 500000, 600000, 800000};
 bestI2CSpeed = speeds[0];
 for (int i = 0; i < sizeof(speeds)/sizeof(speeds[0]); i++)
 {
  Wire.setClock(speeds[i]);
  delay(5);
  if (testOledWrite()) bestI2CSpeed = speeds[i]; else break;
 }
 Wire.setClock(bestI2CSpeed);
}

// ---------------- Setup ----------------
void setup()
{
 pinMode(buttonPin, INPUT);
 Wire.begin();
 autoSetI2Cspeed();

 display.begin(SH1106_SWITCHCAPVCC, OLED_ADDR);
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(WHITE);
 display.setCursor(0,0);
 display.println(F("I2C Auto Speed Test"));
 display.setCursor(0,15);
 display.print(F("Max Speed: "));
 display.print(bestI2CSpeed / 1000);
 display.println(F(" kHz"));
 display.display();
 delay(3000);

 display.clearDisplay();
 display.setTextSize(2);
 display.setCursor(20,15);
 display.println(F("Spectrum   Analyzer"));
 display.display();
 delay(2000);
 display.clearDisplay();

 analogReference(DEFAULT);
}

// -------------- Frequency measurement ----------------
float measureFrequency()
{
 int edgeCount = 0;
 bool lastState = false;
 unsigned long startTime = micros();
 unsigned long timeout = 510000; // 0.51 seconds max
 while (edgeCount < numEdges)
 {
  if (micros() - startTime > timeout) return 0; // no signal detected
  int val = analogRead(A1);
  bool currentState = (val > thresholdHigh) ? true : (val < thresholdLow)  ? false : lastState;
  if (!lastState && currentState) edgeCount++;
  lastState = currentState;
 }
 unsigned long endTime = micros();
 float periodAvg = (endTime - startTime) / (float)numEdges;
 return 1000000.0 / periodAvg;
}

// ---------- Button handling ------------
void checkButton()
{
 if (digitalRead(buttonPin) == LOW)
 { // button pressed
  unsigned long now = millis();
  if (now - lastButtonPress > debounceDelay)
  {
   showOnlyHarmonics = !showOnlyHarmonics; // toggle mode
   lastButtonPress = now;
  }
 }
}

float correctedFreq(float fm)
{
 if (fm <= 0.0f) return fm;        // bail out
 // number of edges used in measureFrequency()
 const float Ns = (float) numEdges;
 // --- determine ADC prescaler from ADCSRA ADPS[2:0] bits ---
 uint8_t adps = ADCSRA & 0x07;
 int prescaler = 128; // default fallback
 switch (adps)
 {
  case 0: prescaler = 2; break;   // 000 -> /2 (note: 000 and 001 behave same on AVR core docs)
  case 1: prescaler = 2; break;   // 001 -> /2 (historic mapping)
  case 2: prescaler = 4; break;   // 010 -> /4
  case 3: prescaler = 8; break;   // 011 -> /8
  case 4: prescaler = 16; break;  // 100 -> /16
  case 5: prescaler = 32; break;  // 101 -> /32
  case 6: prescaler = 64; break;  // 110 -> /64
  case 7: prescaler = 128; break; // 111 -> /128 (Arduino default)
 }
 // --- ADC conversion time (seconds) from datasheet: 13 ADC cycles per conversion ---
 // t_conv = 13 / f_ADC = 13 * prescaler / F_CPU  (seconds)
 float t_conv = (13.0f * (float)prescaler) / (float)F_CPU; // seconds
 float s = t_conv * 0.5f; // average detection delay (seconds)
 // measured period per-edge (seconds)
 float Pm = 1.0f / fm;
 // guard: subtract the tiny s/Ns term (if it would go negative, skip correction)
 float denomPart = Pm - (s / Ns);
 if (denomPart <= 0.0f) return fm; // too small to correct safely
 // corrected true period Pt and frequency
 float Pt = (Ns / (Ns - 1.0f)) * denomPart;
 if (Pt <= 0.0f) return fm;
 float f_corr = 1.0f / Pt;
 return f_corr;
}

void loop()
{
 errorStat = false;
 voltageOutOfRange = false;

 checkButton();

 float freq = measureFrequency();
 // -------- Dynamically calculate sampling rate --------
 Fs = correctedFreq(measureFrequency()) * samplesPerCycle;
 if (Fs < 100) Fs = 100;

 freqSmooth = 0.8*freqSmooth + 0.2*freq; // exponential average
  
 // ---------------- Sampling -----------------
 analogReference(DEFAULT);
 ADCSRA = (ADCSRA & 0xF8) | 0x04; // prescaler = 16
 for (int i = 0; i < N; i++)
 {
  unsigned long start = micros();
  int val = analogRead(A1);
  data[i] = (val - 512) / 4; // scale to -128..127
  im[i] = 0;                 // imaginary part = 0
  while (micros() - start < 1000000UL / Fs);
 }

 // ---------------- DC Measurement -----------------
 float dc_val = analogRead(A2) * (5.0 / 1023.0) - 2.5;

 // ---- Compute min/max  ----
 maxVoltA0 = analogRead(A0) * (5.0 / 1023.0);
 int minValA1 = 1023;
 int maxValA1 = 0;
 for (int i = 0; i < N; i++)
 {
  int adcVal = data[i] * 4 + 512; // reconstruct original ADC value
  if (adcVal < voltageMin || adcVal > voltageMax)
  { 
   // below 0.2V or above 4.8V
   errorStat = true;
   voltageOutOfRange = true;
  }
  if (adcVal < minValA1) minValA1 = adcVal;
  if (adcVal > maxValA1) maxValA1 = adcVal;
 }
 minVoltA1 = minValA1 * (5.0 / 1023.0);
 maxVoltA1 = maxValA1 * (5.0 / 1023.0);
 if (maxVoltA1 > 0) mulFactor = maxVoltA0 / maxVoltA1;  else  mulFactor = 1;
 minVoltA0 = minVoltA1 * mulFactor;
 
 if (Fs < 200) errorStat = true;

 if (!errorStat)
 {
  // -------- Perform FFT --------
  fix_fft(data, im, 7, 0);

  // -------- Find fundamental --------
  long sumHarmonics = 0;
  int numHarmonics = 9;
  int maxIndex = 1;
  int maxMag = 0;
  for (int i = 1; i < N/2; i++)
  {
   int mag = sqrt((float)data[i]*(float)data[i] + (float)im[i]*(float)im[i]);
   if (mag > maxMag)
   {
    maxMag = mag;
    maxIndex = i;
   }
  }

  // ------ Start with FFT peak bin -------
  int bestIndex = maxIndex;
  float bestSide = 1e9;
  // Test maxIndex and its immediate neighbors
  for (int offset = -1; offset <= 1; offset++)
  {
   int testIndex = maxIndex + offset;
   if (testIndex < 1 || testIndex >= N/2-1) continue;
   int magL = sqrt((float)data[testIndex-1]*data[testIndex-1] + (float)im[testIndex-1]*im[testIndex-1]);
   int magR = sqrt((float)data[testIndex+1]*data[testIndex+1] + (float)im[testIndex+1]*im[testIndex+1]);
   float sideSum = (float)magL + (float)magR;
   if (sideSum < bestSide)
   {
    bestSide = sideSum;
    bestIndex = testIndex;
   }
  }
  // ------ Refined fundamental with bin-lock ------
  if (lastIndex == 0) lastIndex = bestIndex;  // initialize
  if (abs(bestIndex - lastIndex) <= 1)
  {
   lastIndex = bestIndex;   // accept small move
  }
  else
  {
   bestIndex = lastIndex;   // reject sudden jump
  }
  float fftFundamental = (float)bestIndex * Fs / N;
  freq = fftFundamental;

  int mag = sqrt((float)data[maxIndex * 2]*(float)data[maxIndex * 2] + (float)im[maxIndex * 2]*(float)im[maxIndex * 2]);
  sumHarmonics += (long)mag * mag;
  harmonicIndexArr[0] = maxIndex; // fundamental peak bin
  harmonicIndexArr[1] = maxIndex * 2; // 1st harmonic peak bin
  for (int k = 3; k <= numHarmonics+1; k++)
  {
   int harmonicIndex = maxIndex * k;
   // ------- find max bin within ±30% window around expected harmonic -------
   int bestHarmIndex = harmonicIndex; // start with expected harmonic
   float bestMag = sqrt((float)data[harmonicIndex]*data[harmonicIndex] + (float)im[harmonicIndex]*im[harmonicIndex]);
   for (int i = harmonicIndex - maxIndex/3; i <= harmonicIndex + maxIndex/3 && i < N/2; i++)
   {
    float mag = sqrt((float)data[i]*data[i] + (float)im[i]*im[i]);
    if (mag > bestMag)
    {
     bestMag = mag;
     bestHarmIndex = i;
    }
   }
   harmonicIndex = bestHarmIndex;
   if (harmonicIndex < N/2)
   {
    int mag = sqrt((float)data[harmonicIndex]*data[harmonicIndex] + (float)im[harmonicIndex]*im[harmonicIndex]);
    if (mag < 0.03 * maxMag) continue;  // ignore small harmonics
    sumHarmonics += (long)mag * mag;
    harmonicIndexArr[k-1] = harmonicIndex; // store exact peak bin
   }
  }
  float thd = 0;
  if (maxMag > 0) thd = sqrt(sumHarmonics) / maxMag * 100.0;

  // -------------- Store THD in buffer --------------
  thdBuffer[thdIndex] = thd;
  thdIndex = (thdIndex + 1) % thdBufferSize;

  // -------------- Compute smoothed THD -------------
  float thdAverage = 0;
  for (int i = 0; i < thdBufferSize; i++) thdAverage += thdBuffer[i];
  thdAverage /= thdBufferSize;

  // -------------- Display spectrum on OLED --------------
  display.clearDisplay();
  int ylim = 54;
  int maxBarHeight = ylim - 12;
  display.drawLine(0, ylim, 128, ylim, WHITE);
  for (int i = 1; i < N/2; i++)
  {
   if (i % maxIndex == 0 && i/maxIndex <= numHarmonics+1)
   {
    int harmonicNum = i / maxIndex; // 1=fundamental, 2=2nd harmonic, etc.
    display.setCursor(i*2-2, ylim+2);
    display.setTextSize(1);
    display.print(harmonicNum);
   }
   if (showOnlyHarmonics)
   {
    // show only fundamental and harmonics
    bool isHarmonic = false;
    for (int k = 0; k <= numHarmonics; k++)
    {
     if (i == harmonicIndexArr[k]) { isHarmonic = true; break; }
    }
    if (!isHarmonic) continue; // skip non-peak bins
   }
   int magnitude = sqrt((float)data[i]*(float)data[i] + (float)im[i]*(float)im[i]);
   int height = magnitude * 2;
   if (height > maxBarHeight) height = maxBarHeight;
   display.drawLine(i*2, ylim, i*2, ylim - height, WHITE);

   // ------- Label fundamental and harmonics -------
   int harmonicNum = -1;
   for (int k = 0; k <= numHarmonics; k++)
   {
    if (i == harmonicIndexArr[k])
    {
     harmonicNum = k + 1; // 1=fundamental, 2=2nd harmonic, etc.
     break;
    }
   }
   if (harmonicNum == -1) continue; // not a peak bin
   display.setTextSize(1);
   float voltsPeak = magnitude * (5.0 / 1023.0) * 4.0 * 2.0;
   if (harmonicNum >= 1 && harmonicNum <= 4)
    ampBuffer[harmonicNum-1][ampIndex] = (uint16_t)(voltsPeak * 100.0);
   if (harmonicNum == 4) ampIndex = (ampIndex + 1) % ampBufferSize;
   // ------- Display averaged A1–A4 -------
   for (int h = 1; h <= 4; h++)
   {
    uint16_t avgVal = 0;
    for (int j = 0; j < ampBufferSize; j++) avgVal += ampBuffer[h-1][j];
    avgVal /= ampBufferSize;
    display.fillRect(80, 2 + h*8, 48, 8, BLACK);
    display.setCursor(80, 2 + h*8);
    display.print(F("A"));
    display.print(h);
    display.print(F(":"));
    display.print((avgVal * mulFactor) / 100.0, 2);
    display.print(F("V"));
   }
  }

  // --------- Display Frequency & Smoothed THD ---------
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print(F("F:"));
  display.print(Fs / samplesPerCycle, 1);
  display.print(F("Hz"));

  display.setCursor(80, 0);
  display.print(F("THD:"));
  display.print(thdAverage, 0);
  display.print(F("%"));
  
  display.setCursor(25, 10);
  display.print(F("DC:"));
  display.print(dc_val, 1);
  display.println(F("V"));
  
  display.display();
 }
 else
 {
  if (voltageOutOfRange)
  {
   display.clearDisplay();
   display.setTextSize(2);
   display.setCursor(10, 0);
   display.print(F("Input Out Of Range"));
   display.setTextSize(1);
   display.setCursor(10, 39);
   if (maxVoltA0 < 4.8)
   {
    display.print(F("Vmax = "));
    display.print(maxVoltA0 - 2.5, 1);
   }
   else display.print(F("Vmax > 2.5"));
   display.print(F("V"));
   display.setCursor(10, 48);
   if (minVoltA0 > 0.2 )
   {
    display.print(F("Vmin = "));
    display.print(minVoltA0 - 2.5, 1);
   }
   else display.print(F("Vmin < -2.5"));
   display.print(F("V"));
   display.setCursor(10, 57);
   display.print(F("DC Avg = "));
   display.print(dc_val, 1);
   display.println(F("V"));
  }
  // -------- No Signal Display ---------
  if (freq < 1)
  {
   display.clearDisplay();
   display.setTextSize(2);
   display.setCursor(10, 28);
   display.println(F("No Signal"));
   if (maxVoltA0 < 0.2 || (maxVoltA0 > dc_val + 2.5 - 0.2 && maxVoltA0 < dc_val + 2.5 + 0.2))
   {
    display.setTextSize(1);
    display.setCursor(10, 57);
    display.print(F("DC Offset = "));
    display.print(dc_val, 1);
    display.println(F("V"));
   }
  }
  display.display();
 }
}