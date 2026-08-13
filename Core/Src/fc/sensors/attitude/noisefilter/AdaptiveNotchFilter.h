#ifndef SRC_FC_SENSORS_ATTITUDE_ADAPTIVENOTCHFILTER_H_
#define SRC_FC_SENSORS_ATTITUDE_ADAPTIVENOTCHFILTER_H_

#include "../../../dsp/FFT.h"
#include "../../../dsp/BiQuadFilter.h"

// Minimum notch Q (widest bandwidth)
#define ADAPTIVE_NOTCH_Q_MIN                  0.75f

// Maximum notch Q (narrowest bandwidth)
#define ADAPTIVE_NOTCH_Q_MAX                  4.0f

// Minimum / shallow attenuation
#define ADAPTIVE_NOTCH_GAIN_MIN              -10.0f

// Maximum / deep attenuation
#define ADAPTIVE_NOTCH_GAIN_MAX              -30.0f

// Absolute gain limits
#define ADAPTIVE_NOTCH_GAIN_ABSOLUTE_MIN     -35.0f
#define ADAPTIVE_NOTCH_GAIN_ABSOLUTE_MAX      -8.0f

// Peak detection
#define ADAPTIVE_NOTCH_SNR_LOW                 2.5f
#define ADAPTIVE_NOTCH_SNR_HIGH                4.0f

#define ADAPTIVE_NOTCH_MAG_MIN                 6.0f
#define ADAPTIVE_NOTCH_MAG_MAX                15.0f

// EMA smoothing
#define ADAPTIVE_NOTCH_ALPHA_Q                 0.25f
#define ADAPTIVE_NOTCH_ALPHA_GAIN              0.20f
#define ADAPTIVE_NOTCH_ALPHA_CF_GAIN           0.25f


// Gain below which PEAK becomes full NOTCH

// Maximum NOTCH
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -10.0f
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -12.0f
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -14.0f
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -16.0f
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -18.0f
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -20.0f
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -22.0f
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -24.0f
#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -25.0f //Old profile1
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -26.0f
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -28.0f
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -30.0f
// PEAK-only
//#define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -31.0f


/*
 * ============================================================================
 * ADAPTIVE NOTCH FILTER STATE
 * ============================================================================
 */
typedef struct {
	/*
	 * Smoothed target parameters.
	 */
	float smoothedQ;
	float smoothedGain;
	float smoothedCF;
	/*
	 * Last parameters actually applied to the biquad.
	 */
	float lastAppliedFreq;
	float lastAppliedQ;
	float lastAppliedGain;
	/*
	 * Detection information.
	 */
	float snr;
	float peakMag;

} AdaptiveNotchFilterState;

/*
 * ============================================================================
 * API
 * ============================================================================
 */
void processAdaptiveNotchFilter(BIQUADFILTER *filter, AdaptiveNotchFilterState *state, float targetFreq, int peakBin, float *magnitudes);

#endif /* SRC_FC_SENSORS_ATTITUDE_ADAPTIVENOTCHFILTER_H_ */
