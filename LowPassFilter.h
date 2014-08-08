#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 2000 Hz

* 0 Hz - 100 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 3.930212852225667 dB

* 150 Hz - 1000 Hz
  gain = 0
  desired attenuation = -50 dB
  actual attenuation = -50.508162917032166 dB

*/

#define LOWPASSFILTER_TAP_NUM 57

typedef struct {
  double history[LOWPASSFILTER_TAP_NUM];
  unsigned int last_index;
} LowPassFilter;

void LowPassFilter_init(LowPassFilter* f);
void LowPassFilter_put(LowPassFilter* f, double input);
double LowPassFilter_get(LowPassFilter* f);

#endif
