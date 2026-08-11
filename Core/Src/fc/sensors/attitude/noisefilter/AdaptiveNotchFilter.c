#include <math.h>
#include "AdaptiveNotchFilter.h"
#include "../../../memory/Memory.h"

#define FFT_BIN_GUARD 3

/**
 * @brief Logic to update a single Filter state and its Biquad coefficients
 */
__ATTR_ITCM_TEXT
void processAdaptiveNotchFilter(BIQUADFILTER *filter, AdaptiveNotchFilterState *state, float targetFreq, int peakBin, float *magnitudes) {
	if (peakBin < FFT_BIN_GUARD || peakBin > (FFT_HALF_N - FFT_BIN_GUARD)) {
		return;
	}

	// 1. SNR Calculation
	float peakMag = magnitudes[peakBin];
	float localFloor = (magnitudes[peakBin - 2] + magnitudes[peakBin - 3] + magnitudes[peakBin + 2] + magnitudes[peakBin + 3]) * 0.25f;
	float snr = peakMag / (localFloor + 1e-6f);
	state->snr = snr;
	state->peakMag = peakMag;

	// 1.1. Smooth Center Frequency
	state->smoothedCF += ADAPTIVE_NOTCH_ALPHA_CF_GAIN * (targetFreq - state->smoothedCF);

	// 2. Calculate Target Q (Frequency Compensated)
	float freqComp = 1.0f + (state->smoothedCF / 1200.0f);
	float targetQ = ADAPTIVE_NOTCH_Q_MIN + ((snr * freqComp) - ADAPTIVE_NOTCH_SNR_LOW) * ((ADAPTIVE_NOTCH_Q_MAX - ADAPTIVE_NOTCH_Q_MIN) / (ADAPTIVE_NOTCH_SNR_HIGH - ADAPTIVE_NOTCH_SNR_LOW));
	targetQ = constrainToRangeF(targetQ, ADAPTIVE_NOTCH_Q_MIN, ADAPTIVE_NOTCH_Q_MAX);

	// 3. Calculate Target Gain (Depth) - Tied directly to Config Defines
	float gainRatio = (peakMag - ADAPTIVE_NOTCH_MAG_MIN) / (ADAPTIVE_NOTCH_MAG_MAX - ADAPTIVE_NOTCH_MAG_MIN);
    gainRatio = constrainToRangeF(gainRatio, 0.0f, 1.0f); // Ensure ratio stays bounded [0, 1]

	float targetGain = ADAPTIVE_NOTCH_GAIN_MIN + (gainRatio * (ADAPTIVE_NOTCH_GAIN_MAX - ADAPTIVE_NOTCH_GAIN_MIN));
	targetGain = constrainToRangeF(targetGain, ADAPTIVE_NOTCH_GAIN_ABSOLUTE_MIN, ADAPTIVE_NOTCH_GAIN_ABSOLUTE_MAX);

	// 4. Apply EMA Smoothing
	state->smoothedQ += ADAPTIVE_NOTCH_ALPHA_Q * (targetQ - state->smoothedQ);
	state->smoothedGain += ADAPTIVE_NOTCH_ALPHA_GAIN * (targetGain - state->smoothedGain);

	// 5. Thresholded Update (Save CPU)
	uint8_t freqChanged = fabsf(state->smoothedCF - state->lastAppliedFreq) > 2.0f;
	uint8_t qChanged = fabsf(state->smoothedQ - state->lastAppliedQ) > 0.05f;
	uint8_t gainChanged = fabsf(state->smoothedGain - state->lastAppliedGain) > 1.0f;

	if (freqChanged || qChanged || gainChanged) {
		filter->center_freq = state->smoothedCF;
		filter->Q = state->smoothedQ;
		filter->gainDB = state->smoothedGain;

		// Morphing Logic: Correctly splits the range between Peak and Notch
		if (state->smoothedGain > ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP) {
			filter->type = BIQUAD_PEAK;
		} else {
			filter->type = BIQUAD_NOTCH;
		}

		biQuadCalculateCoeffs(filter);

		state->lastAppliedFreq = state->smoothedCF;
		state->lastAppliedQ = state->smoothedQ;
		state->lastAppliedGain = state->smoothedGain;
	}
}
