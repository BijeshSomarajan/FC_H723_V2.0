#ifndef SRC_FC_SENSORS_ATTITUDE_ADAPTIVENOTCHFILTER_H_
#define SRC_FC_SENSORS_ATTITUDE_ADAPTIVENOTCHFILTER_H_
#include "../../../dsp/FFT.h"
#include "../../../dsp/BiQuadFilter.h"

// Minimum notch Q (widest bandwidth) used for smeared or unstable noise.
#define ADAPTIVE_NOTCH_Q_MIN 0.75f
// Maximum notch Q (narrowest bandwidth) used for clean tonal peaks.
#define ADAPTIVE_NOTCH_Q_MAX 1.5f
// Gain (dB) below which the filter switches from PEAK to full NOTCH.
#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP -30.0f //25.0f
// SNR below which detected peaks are considered noisy or unreliable.
#define ADAPTIVE_NOTCH_SNR_LOW 2.5f
// SNR above which detected peaks are considered clean and well-defined.
#define ADAPTIVE_NOTCH_SNR_HIGH 4.0f
// FFT magnitude below which only shallow attenuation is applied.
#define ADAPTIVE_NOTCH_MAG_MIN 6.0f//20.0f
// FFT magnitude above which maximum attenuation is applied.
#define ADAPTIVE_NOTCH_MAG_MAX 15.0f//400.0f
// EMA smoothing factor for notch Q (lower = smoother bandwidth changes).
#define ADAPTIVE_NOTCH_ALPHA_Q 0.5f//0.10f
// EMA smoothing factor for notch depth/gain (lower = smoother depth changes).
#define ADAPTIVE_NOTCH_ALPHA_GAIN 0.2f//0.08f

typedef struct {
	float smoothedQ;
	float smoothedGain;
	float lastAppliedFreq;
	float lastAppliedQ;
	float lastAppliedGain;
	float snr;
	float peakMag;
} AdaptiveNotchFilterState;

void processAdaptiveNotchFilter(BIQUADFILTER *filter, AdaptiveNotchFilterState *state, float targetFreq, int peakBin, float *magnitudes) ;


#endif /* SRC_FC_SENSORS_ATTITUDE_ADAPTIVENOTCHFILTER_H_ */
