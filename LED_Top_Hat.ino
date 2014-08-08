//*************************************************************************************
//******** LED TOP HAT v1.2 (C)opyright 2014 Matt Camp (matt@noise.net.nz)   **********
//*************************************************************************************


#define ARM_MATH_CM4
#include <arm_math.h>
#include "FastLED.h"
#include "LowPassFilter.c"  // Low pass filter @ 150hz for beat detection. Designed at http://t-filter.appspot.com
#include "RunningAverage.h"

// Constants for LED Strip
#define DATA_PIN 11
#define CLOCK_PIN 12
#define NUM_LEDS 240
CRGB leds[NUM_LEDS];


// My matrix is 30 wide by 8 high. 
// The LED strips are laid out in an S type pattern with 0 being the top left corner, and 239 being the bottom right.
// The colData structure stores the values for each 'column' for drawing vertical lines

int colData[30][8] = { 
{239,180,179,120,119,60,59,0},
{238,181,178,121,118,61,58,1},
{237,182,177,122,117,62,57,2},
{236,183,176,123,116,63,56,3},
{235,184,175,124,115,64,55,4},
{234,185,174,125,114,65,54,5},
{233,186,173,126,113,66,53,6},
{232,187,172,127,112,67,52,7},
{231,188,171,128,111,68,51,8},
{230,189,170,129,110,69,50,9},
{229,190,169,130,109,70,49,10},
{228,191,168,131,108,71,48,11},
{227,192,167,132,107,72,47,12},
{226,193,166,133,106,73,46,13},
{225,194,165,134,105,74,45,14},
{224,195,164,135,104,75,44,15},
{223,196,163,136,103,76,43,16},
{222,197,162,137,102,77,42,17},
{221,198,161,138,101,78,41,18},
{220,199,160,139,100,79,40,19},
{219,200,159,140,99,80,39,20},
{218,201,158,141,98,81,38,21},
{217,202,157,142,97,82,37,22},
{216,203,156,143,96,83,36,23},
{215,204,155,144,95,84,35,24},
{214,205,154,145,94,85,34,25},
{213,206,153,146,93,86,33,26},
{212,207,152,147,92,87,32,27},
{211,208,151,148,91,88,31,28},
{210,209,150,149,90,89,30,29}};



// Constants for beat detection
int samplesN = 200;
int micPin = 14;

LowPassFilter* filter;
int index2 = 0;
int maxpeak = 0 ;
int minPeak = 1023;
RunningAverage myRA(10);
RunningAverage bpm(5);



// Constants for Spectrum Analyser
int SAMPLE_RATE_HZ = 9000;             // Sample rate of the audio in hertz.
float SPECTRUM_MIN_DB = 25.0;          // Audio intensity (in decibels) that maps to low LED brightness.
float SPECTRUM_MAX_DB = 60.0;          // Audio intensity (in decibels) that maps to high LED brightness.
const int FFT_SIZE = 256;              // Size of the FFT.  Realistically can only be at most 256 
                                       // without running out of memory for buffers and other state.
const int AUDIO_INPUT_PIN = 14;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading.
const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).
const int COLUMN_COUNT = 16;           // Number of columns on the spectrum analyser. Since we mirror the display this is half the real number of columns on the hat

float noise[16] = { 0.1, 0, 0, 0, 0, 0, 0, 0 };
uint8_t eq_raw[16] = { 2, 4, 8, 14, 22, 33, 47, 68, 99, 147, 198, 220, 225, 218, 175, 255 };
float eq[16] = { 1.0, 1.2, 1.3, 2.0, 2.39, 3.33, 3.48, 4.47, 4.59, 5.78, 5.58, 4.86, 5.88, 6.85, 7.69, 8.0 };
float LvlMax = 0;
float LvlMin = 0;

IntervalTimer samplingTimer;
float samples[FFT_SIZE*2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;

float frequencyWindow[COLUMN_COUNT+1];
float hues[COLUMN_COUNT];


// ******* BEGIN FUNCTIONS *****************************

void setup(){
  Serial.begin(115200);
  filter = new LowPassFilter();
  LowPassFilter_init(filter);
  myRA.clear();

  
  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);
  
  pinMode(10, INPUT_PULLUP); // For detecting button presses
  FastLED.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, BRG>(leds, NUM_LEDS);
  set_max_power_in_volts_and_milliamps( 5, 5000); //5v 5000mA
  pinMode(13, OUTPUT); // enable flashing of the on-board LED if FastLED hits the power limit
  set_max_power_indicator_LED( 13);
}

void loop() {
  spectrum();
  delay(500); // 500ms delay to avoid reading one button push as multiples
  beat_detect_solid();
  delay(500);
  beat_detect_random();
  delay(500);
  cylon(25);
  delay(500);
  colourRoll(25);
  delay(500);
  randomFlash(15);
  delay(500);
}


//// **** Spectrum Analyser Functions ****

void spectrum() {
  // Initialize spectrum display
  spectrumSetup();
  
  // Begin sampling audio
  samplingBegin();

  while(1) {
    if (digitalRead(10) != HIGH)  break; // button was pressed
    if (samplingIsDone()) {
      // Run FFT on sample data.
      arm_cfft_radix4_instance_f32 fft_inst;
      arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
      arm_cfft_radix4_f32(&fft_inst, samples);
      // Calculate magnitude of complex numbers output by the FFT.
      arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);

      spectrumLoop();

      // Restart audio sampling.
      samplingBegin();
    }    
  } 
 samplingTimer.end(); 
}

void spectrumSetup() {
  // Set the frequency window values by evenly dividing the possible frequency
  // spectrum across the number of neo pixels.
  float windowSize = (SAMPLE_RATE_HZ / 2.0) / float(COLUMN_COUNT);
  for (int i = 0; i < COLUMN_COUNT+1; ++i) {
    frequencyWindow[i] = i*windowSize;
  }
  // Evenly spread hues across all pixels.
  for (int i = 0; i < COLUMN_COUNT; ++i) {
    hues[i] = 360.0*(float(i)/float(COLUMN_COUNT-1));
  }
}

void spectrumLoop() {
  // Update each LED based on the intensity of the audio 
  // in the associated frequency window.
  float intensity, otherMean, nint;
  int leds2[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  int led_c[16];
  float intensities[16];
  float int_sum;
  LvlMax=0;
  LvlMin=0;
  for (int i = 0; i < COLUMN_COUNT; ++i) {
    windowMean(magnitudes, 
               frequencyToBin(frequencyWindow[i]),
               frequencyToBin(frequencyWindow[i+1]),
               &intensity,
               &otherMean);
    // Convert intensity to decibels.
    intensity = 20.0*log10(intensity);
    // Scale the intensity and clamp between 0 and 1.0.
    intensity -= SPECTRUM_MIN_DB;
    
    intensity /= (SPECTRUM_MAX_DB-SPECTRUM_MIN_DB);
    
    if(i==0) {
      intensity-=0.15;
    }
    if(i==1) {
      intensity-=0.04;
    }
    if(intensity<0) {
      intensity=0;
    }
    intensity=intensity*eq[i];
    intensity = intensity < 0.0 ? 0.0 : intensity;
    intensity = intensity > 1.0 ? 1.0 : intensity;
    intensities[i]=intensity;
    if(intensity>LvlMax) {
      LvlMax=intensity;
    }
    if(intensity<LvlMin) {
      LvlMin=intensity;
    }
    int_sum+=intensity;
  }
  
  // Attempt to scale the display. This only really works at making quiet stuff louder, too much noise will still result in a maxxed out display.
  float avg_int=int_sum/COLUMN_COUNT;
  float scaleFactor = (float)0.9/(LvlMax);
  //if(avg_int>0.75) scaleFactor=(float)1-(avg_int-0.75); // didn't really work well
  if(scaleFactor>4.0) scaleFactor=4.0; // Don't scale too far or things get weird
  
  for (int z=0, j = 15; z< 8; z++, j--) // Reverse the intensities array because we want the bass end of the spectrum in the middle/front of the hat
  {
    float temp = intensities[z];
    intensities[z] = intensities[j];
    intensities[j] = temp;
  }
  
  
  for (int i = 0; i < COLUMN_COUNT; i++) {
    //if(avg_int>0.75) intensities[i] = intensities[i]*scaleFactor; // this didn't work well either
    if(LvlMax<0.8) intensities[i] = intensities[i]*scaleFactor;
    int lc= 8*intensities[i];
    if(lc>leds2[i]) leds2[i]=lc;
    setCol_spectrum(i,leds2[i]); // Draw the first 15 columns
    setCol_spectrum(30-i,leds2[i]); // Draw the second 15 columns
  }
  
  show_at_max_brightness_for_power();
  
  for (int i = 0; i < COLUMN_COUNT; i++) { // Fade the columns back towards zero
     if(leds2[i]) leds2[i]--;
  }

}

////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter+1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE*2) {
    samplingTimer.end();
  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE*2;
}

////////////////////////////////////////////////////////////////////////////////
// SPECTRUM UTILITY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
    *windowMean = 0;
    *otherMean = 0;
    // Notice the first magnitude bin is skipped because it represents the
    // average power of the signal.
    for (int i = 1; i < FFT_SIZE/2; ++i) {
      if (i >= lowBin && i <= highBin) {
        *windowMean += magnitudes[i];
      }
      else {
        *otherMean += magnitudes[i];
      }
    }
    *windowMean /= (highBin - lowBin) + 1;
    *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin));
}

// Convert a frequency to the appropriate FFT bin it will fall within.
int frequencyToBin(float frequency) {
  float binFrequency = float(SAMPLE_RATE_HZ) / float(FFT_SIZE);
  return int(frequency / binFrequency);
}

void blank_strip() {
  int i;
  for (i=0; i < NUM_LEDS; i++) {
      leds[i]=CRGB::Black;    
  } 
  show_at_max_brightness_for_power();
}


// setCol_spectrum is used to set a specific column to a value (1-8) for the spectrum analyser. 
// different values get different colours to give the green-yellow-red effect.

void setCol_spectrum(int col, int val) {
    int pixel, row;
    int r,g,b;
    if(val>8) val=8;
    if(col>29) col=29;  
    
    for(row=0; row<8; row++) {
      pixel=colData[col][row];
      if(row<val) {
        switch(row) {
          case 0:
            r = 0;
            g = 255;
            b = 0;
            break;          
          case 1:
            r = 0;
            g = 255;
            b = 0;
            break;          
          case 2:
            r = 0;
            g = 255;
            b = 0;
            break;
          case 3:
            r = 100;
            g = 40;
            b = 0;
            break;
          case 4:
            r = 255;
            g = 30;
            b = 0;           
            break;
          case 5:
            r = 255;
            g = 20;
            b = 0;
            break;
          case 6:
            r = 255;
            g = 10;
            b = 0;
            break;
          case 7:
            r = 255;
            g = 0;
            b = 0;
            break;
        }
        
      } else {
        r = 0;
        g = 0;
        b = 0;
      }
      leds[pixel]=CRGB(r,g,b);
    }
}

// WHeel() returns a colour based on it's position in a 384-value rainbow cycle
CRGB Wheel(uint16_t WheelPos)
{
  byte r, g, b;
  int rgb[3] = {0,0,0};
  switch(WheelPos / 128)
  {
    case 0:
      r = 127 - WheelPos % 128;   //Red down
      g = WheelPos % 128;         // Green up
      b = 0;                      //blue off
      break; 
    case 1:
      g = 127 - WheelPos % 128;  //green down
      b = WheelPos % 128;        //blue up
      r = 0;                     //red off
      break; 
    case 2:
      b = 127 - WheelPos % 128;  //blue down 
      r = WheelPos % 128;        //red up
      g = 0;                     //green off
      break; 
  }
  rgb[0]=r;
  rgb[1]=g;
  rgb[2]=b;
  return CRGB(r,g,b);
}


// **** BEAT DETECT FUNCTIONS ******

// Whole matrix is one solid random colour, changes in time with the music
// This function requires a couple of beats before it will display. 
void beat_detect_solid() {
  unsigned long lastbeat = 0;
  while(1) {
    int peak = 0;
    if (digitalRead(10) != HIGH)  break; // button was pressed
    for(int k=0; k<samplesN; k++){
      int val = analogRead(micPin);
      LowPassFilter_put(filter, val);
  
      int filtered = LowPassFilter_get(filter);
      peak = max(peak, filtered);

    }  
    maxpeak = max(maxpeak, peak);
    minPeak = min(minPeak, peak);
  
    index2++;
    if(index2 == 1000){
      maxpeak = 0;
      minPeak = 1023;
    }
    int lvl = map(peak, minPeak, maxpeak, 0, 1023);
    myRA.addValue(lvl); // Keep a rolling average of the last 10 levels
    if(lvl-myRA.getAverage()>50) { // current level was higher than the average, is probably a beat.
      unsigned long time = millis();
      if(time-lastbeat>300) { // Limit to one beat every 300ms. Prevents multiple detection on what is actually a single beat. Adjust as necessary for your music.
        lastbeat=time;
        int c=random(384);
        for (int i=0;i<NUM_LEDS; i++) {
          leds[i]=Wheel(c);
        }
        show_at_max_brightness_for_power();
        //myRA.clear(); // Clear the rolling average. In theory this is using up ram unless cleared, but Teensy has enough it should run for days.
      }
    }
  }
  myRA.clear(); // clear the rolling average before moving on to next pattern
}

// Same as beat_detect_solid except that every pixel gets a different random colour, changed in time with the music.
void beat_detect_random() {
  unsigned long lastbeat = millis();
  while(1) {
    int peak = 0;
    if (digitalRead(10) != HIGH)  break; // button was pressed
    for(int k=0; k<samplesN; k++){
      int val = analogRead(micPin);
      LowPassFilter_put(filter, val);
  
      int filtered = LowPassFilter_get(filter);
      peak = max(peak, filtered);
    }  
    maxpeak = max(maxpeak, peak);
    minPeak = min(minPeak, peak);
  
    index2++;
    if(index2 == 1000){
      maxpeak = 0;
      minPeak = 1023;
    }
    int lvl = map(peak, minPeak, maxpeak, 0, 1023);
    myRA.addValue(lvl);
    if(lvl-myRA.getAverage()>50) { // beat detected
      unsigned long time = millis();
      if(time-lastbeat>300) { // see previous function for comments
        lastbeat=time;
        for (int i=0;i<NUM_LEDS; i++) {
          int c=random(384);
          leds[i]=Wheel(c);
        }
        show_at_max_brightness_for_power();
        //myRA.clear();
      }
    }
  }
  myRA.clear();
}

// **** OTHER PATTERNS ****

void rainbow(uint8_t wait) {
  int i, j;
   
  for (j=0; j < 384; j++) {     // 3 cycles of all 384 colors in the wheel
    for (i=0; i < NUM_LEDS; i++) {
      leds[i]=Wheel(i+j);
    }  
    show_at_max_brightness_for_power();   // write all the pixels out
    delay(wait);
  }
}

// Slightly different, this one makes the rainbow wheel equally distributed 
// along the chain
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;
  
  for (j=0; j < 384 * 5; j++) {     // 5 cycles of all 384 colors in the wheel
    for (i=0; i < NUM_LEDS; i++) {
      // tricky math! we use each pixel as a fraction of the full 384-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 384 is to make the wheel cycle around
      //strip.setPixelColor(i, Wheel( ((i * 384 / strip.numPixels()) + j) % 384) );
      leds[i]=Wheel( ((i * 384 / NUM_LEDS) + j) % 384);
    }  
    show_at_max_brightness_for_power();   // write all the pixels out
    delay(wait);
  }
}


// **** THESE FUNCTIONS BELOW ARE TAKEN FROM THE LPD8806 STRANDTEST EXAMPLE BUT HAVE NOT YET BEEN UPDATED FOR FASTLED ***
/*
// Fill the dots progressively along the strip.
void colorWipe(uint32_t c, uint8_t wait) {
  int i;

  for (i=0; i < NUM_LEDS; i++) {
    //strip.setPixelColor(i, c);
    leds[i]=CRGB(c,c,c);
      strip.show();
      //strip.showCompileTime<6, 7>(PORTD, PORTD);
      //delay(wait);
  }
}

// Chase one dot down the full strip.
void colorChase(uint32_t c, uint8_t wait) {
  int i;

  // Start by turning all pixels off:
  for(i=0; i<strip.numPixels(); i++) strip.setPixelColor(i, 0);

  // Then display one pixel at a time:
  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c); // Set new pixel 'on'
    strip.show();              // Refresh LED states
    //strip.showCompileTime<6, 7>(PORTD, PORTD);
    strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
    delay(wait);
  }

  strip.show(); // Refresh to turn off last pixel
}



//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<20; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();
      //strip.showCompileTime<6, 7>(PORTD, PORTD);
     
      delay(wait);
     
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}
*/

//Theatre-style crawling lights with rainbow effect
// This function needs to be updated to work with a serpentine matrix style... still kinda looks cool anyway.
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 384; j++) {     // cycle all 384 colors in the wheel
    for (int q=0; q < 3; q++) {
        for (int i=0; i < NUM_LEDS; i=i+3) { 
          leds[i+q]=Wheel(i+j); //turn every third pixel on
        }
        show_at_max_brightness_for_power();
       
        delay(wait);
       
        for (int i=0; i < NUM_LEDS; i=i+3) {       
          leds[i+q]=CRGB::Black; //turn every third pixel off
        }
    }
  }
}

void cylon(uint8_t wait) {
  int i;
  int direction=0; 
  while(1) {
    if (digitalRead(10) != HIGH)  return;
    switch(direction) {
      case 0:  // Left to Right
        for(i=0; i<29; i++) {
          setCol_HSV(i,8,CHSV(0,255,255)); // first column is solid red at max brightness
          if(i) {
            if(i>0) setCol_HSV(i-1,8,CHSV(0,255,128)); // fade out the few columns behind the main one to give a nice visual effect
            if(i>1) setCol_HSV(i-2,8,CHSV(0,255,80));
            if(i>2) setCol_HSV(i-3,8,CHSV(0,255,32));
            if(i>3) setCol_HSV(i-4,0,CHSV(0,255,0));
          }
          show_at_max_brightness_for_power();
          if (digitalRead(10) != HIGH)  return; // button was pressed
          delay(wait);
        }
        direction=1;
        break;
      case 1:  // Right to Left
        for(i=29; i>=0; i--) {
          setCol_HSV(i,8,CHSV(0,255,255));
          if(i<29) {
            setCol_HSV(i+1,8,CHSV(0,255,128));
            setCol_HSV(i+2,8,CHSV(0,255,80));
            setCol_HSV(i+3,8,CHSV(0,255,32));
            setCol_HSV(i+4,0,CHSV(0,255,0));
          }
          show_at_max_brightness_for_power();
          if (digitalRead(10) != HIGH)  return; // button was pressed
          delay(wait);
        }
        direction=0;
        break;        
    }
  }
  
}


// Roll a rainbow around the hat horizontally
void colourRoll(uint8_t wait) {
  int c=0;

  while(1) {
    if (digitalRead(10) != HIGH)  break;
    for(int i=0; i<30; i++) {
      c+=25; // adjust for how granular you want the rainbow
      if(c>384) c=0;
      setCol_RGB(i,8,Wheel(c));
    }
  
    //show_at_max_brightness_for_power();
    FastLED.show();
    delay(wait);
  } 
}


// Randomly flash LEDs full white then fade them back towards zero unevenly, which makes them turn coloured.
void randomFlash(uint8_t wait) {
  uint8_t r[240];
  uint8_t g[240];
  uint8_t b[240];
  uint8_t dim=20; // This is the max amount an LED can be randomly dimmed. Increase this if you want them to dim slower
  randomSeed(analogRead(0));
  
  blank_strip(); // ensure we're starting with a clear matrix
  
  while(1) {
    if (digitalRead(10) != HIGH)  break;
    for(int i=0; i<240; i++) { // cycle through all LEDs and dim them randomly towards zero if they are lit

      
      int c=random(3); // Decide if this LED is going towards blue, green, or red by setting one value to zero
      switch(c) {
        case 0:
           if(r[i] >250 ) r[i]=0;
           break;
        case 1:
           if(g[i] > 250) g[i]=0;
           break;
        case 2:
           if(b[i]> 250) b[i]=0;
           break;
      }
      
      // if this led has a red value, fade it out by a random amount
      int d=random(dim);
      if(r[i]) {
        if(r[i]>d) {
          r[i]-=d;
        } else {
          r[i]=0;
        }
      }  
      
      // if this led has a green value, fade it out by a random amount
      d=random(dim);
      if(g[i]) {
        if(g[i]>d) {
          g[i]-=d;
        } else {
          g[i]=0;
        }
      }
      
      // if this led has a blue value, fade it out by a random amount
      d=random(dim);
      if(b[i]) {
        if(b[i]>d) {
          b[i]-=d;
        } else {
          b[i]=0;
        }
      }  
      leds[i]=CRGB(r[i],g[i],b[i]);     
    }
    
    // Now pick one random pixel and set it to white
    int pixel=random(240);
    r[pixel]=255;
    g[pixel]=255;
    b[pixel]=255; 
    leds[pixel]=CRGB::White;
    
    show_at_max_brightness_for_power(); 
    delay(wait);  
  }
}

// setCol_HSV sets one column to a specified HSV value
void setCol_HSV(int col, int val, CHSV colour) {
    int pixel, row;
    int r,g,b;

    if(val>8) val=8;
    if(col>29) col=29;  
 
    for(row=0; row<8; row++) {
      pixel=colData[col][row];
      if(row<val) {
        leds[pixel]=colour;       
      } else {
        leds[pixel]=CRGB::Black;
      }
    }
}

// setCol_HSV sets one column to a specified HSV value
void setCol_RGB(int col, int val, CRGB colour) {
    int pixel, row;
    int r,g,b;
    if(val>8) val=8;
    if(col>29) col=29;
    if(col<0) col=0;
     
    for(row=0; row<8; row++) {
      pixel=colData[col][row];
      if(row<val) {
        leds[pixel]=colour;      
      } else {
        leds[pixel]=CRGB::Black;
      }
    }
}


