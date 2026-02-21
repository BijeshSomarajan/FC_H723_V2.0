#ifndef _BIQUADFILTER_H_
#define _BIQUADFILTER_H_

#include <math.h>
enum {
	BIQUAD_LOWPASS = 0, BIQUAD_HIGHPASS, BIQUAD_BANDPASS, BIQUAD_NOTCH, BIQUAD_PEAK, BIQUAD_LOWSHELF, BIQUAD_HIGHSHELF
};
typedef struct _BIQUADFILTER BIQUADFILTER;
struct _BIQUADFILTER {
	int type;
	float a0, a1, a2, b0, b1, b2;
	float x1, x2, y, y1, y2;
	float center_freq, sample_rate, Q, gainDB;
	float lastCenterFreq;
};

#define BIQUAD_USE_FAST_SQRT       1
#define BIQUAD_USE_APPROX_TRIG       1

void biQuadFilterInit(BIQUADFILTER* self, int type, float center_freq, float sample_rate, float Q, float gainDB);
float biQuadFilterUpdate(BIQUADFILTER* self, float data);
void biQuadFilterReset(BIQUADFILTER* self);
void biQuadCalculateCoeffs(BIQUADFILTER* self);
void biQuadFilterSetCenterFreq(BIQUADFILTER* self,float center_freq);

#endif
