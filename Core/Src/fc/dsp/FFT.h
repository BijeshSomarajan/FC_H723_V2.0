#ifndef FC_FCDSP_INCLUDE_FFT_H_
#define FC_FCDSP_INCLUDE_FFT_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "LowPassFilter.h"

// FFT size (number of samples per transform), must be a power of two
#define FFT_N             256

// Number of usable frequency bins (Nyquist limit)
#define FFT_HALF_N        (FFT_N / 2)

// Number of top dominant frequency bins tracked for noise analysis
#define FFT_TOP_FREQ_N     2

// FFT hop size between consecutive windows (50% overlap improves time resolution)
#define FFT_HOP_SIZE      (FFT_N / 2)   // M: 50% Overlap

// --- Feature Toggles ---
// Enable windowing (e.g. Hann) to reduce spectral leakage
#define FFT_ENABLE_WINDOWING    1

// Account FFT windowing cost in processing budget calculations
#define FFT_ENABLE_BUDGET_FOR_WINDOWING  0

// Enable pre-FFT low-pass filtering of input samples
#define FFT_ENABLE_SAMPLE_LPF   1

// Cutoff frequency (Hz) for pre-FFT sample low-pass filter
#define FFT_ENABLE_SAMPLE_LPF_CUTOFF_FREQUENCY   800.0f

// Use fast square-root approximation for magnitude computation
#define FFT_USE_FAST_SQRT  1

// FFT mode: detect harmonic structure rather than isolated peaks
#define FFT_MODE_HARMONICS   1

// FFT mode: detect strongest independent spectral peaks
#define FFT_MODE_PEAKS       2

// Selected FFT operating mode
#define FFT_MODE FFT_MODE_HARMONICS

// Enable computation of harmonic frequencies from detected fundamental
#define FFT_CALCULATE_HARMONIC_FREQUENCIES 1


/*
 //How fast the FFT noise floor rises when noise increases.
 #define FFT_NOISE_EMA_RISE_ALPHA   0.04f
 //How fast noise floor falls when motors quieten.
 #define FFT_NOISE_EMA_FALL_ALPHA   0.10f
 //Absolute lower bound on noise floor.
 #define FFT_NOISE_MIN_FLOOR        0.25f
 // How sharp a peak must be above neighbors to count.
 #define FFT_PEAK_REJECT_RATIO      0.28f
 //Peak must be XX noise floor to be “real”
 #define FFT_NOISE_MULTIPLIER       2.0f
 //Max allowed frequency movement per millisecond.
 #define FFT_MAX_FREQ_SLEW_HZ_PER_MS 2.0f
 //Ignore very low frequencies when computing noise floor.
 #define FFT_NOISE_FLOOR_MIN_HZ      40.0f
 //Use only lower XX% of spectrum to compute noise floor.
 #define FFT_MAX_BIN_FRACTION_FOR_NOISE  0.8f
 */

/* --- Noise model - Soft Mounting --- */
//How fast the FFT noise floor rises when noise increases.
#define FFT_NOISE_EMA_RISE_ALPHA   0.10f
//How fast noise floor falls when motors quieten.
#define FFT_NOISE_EMA_FALL_ALPHA   0.04f
//Absolute lower bound on noise floor.
#define FFT_NOISE_MIN_FLOOR        0.3f//0.28f
// How sharp a peak must be above neighbors to count.
#define FFT_PEAK_REJECT_RATIO      1.5f
//Peak must be XX noise floor to be “real”.
#define FFT_NOISE_MULTIPLIER      1.5f  // 5.0f
//Max allowed frequency movement per millisecond.
#define FFT_MAX_FREQ_SLEW_HZ_PER_MS 0.5f//0.25f
//Ignore very low frequencies when computing noise floor.
#define FFT_NOISE_FLOOR_MIN_HZ      50.0f
//Use only lower XX% of spectrum to compute noise floor.
#define FFT_MAX_BIN_FRACTION_FOR_NOISE  0.6f


typedef struct {
	float real;
	float imag;
} FFTData;

typedef enum {
	FFT_PHASE_IDLE,    // Waiting for new samples
	FFT_PHASE_WINDOW,  // Apply window to the new buffer
	FFT_PHASE_BITREV,  // Incremental bit reversal
	FFT_PHASE_STAGES,  // Incremental butterfly calculation
	FFT_PHASE_DONE     // Magnitude/peak calculation
} FFTPhase;

typedef struct {
	FFTData fftData[FFT_N];
	float fftMagnitudes[FFT_HALF_N + 1];

	float topFreqBin[FFT_TOP_FREQ_N];
	float topFreq[FFT_TOP_FREQ_N];
	float topMagn[FFT_TOP_FREQ_N];

	float fftSamplingFrequency;
	uint16_t fftSampleCount;

	FFTPhase phase;             // Current phase of processing
	uint8_t processing;         // 1 if currently performing a step, 0 otherwise

	int stage;                  // Current FFT stage (1 to log2(N))
	int start;                  // Current butterfly group start index
	int i;                      // Current butterfly index within the group
	int br_i;                   // Current index for bit reversal
	int br_reversed;            // Current bit-reversed index
	float windowBuffer[FFT_N];  // Temp buffer for overlap shift/storage
	float noiseFloor;
	float peakNoise;

	uint8_t hasProcessUpdate;

	LOWPASSFILTER sampleLPF;

} FFTContext;

void initFFT(float sampleFrequency);
void initFFTContext(FFTContext *ctx);

uint8_t updateFFT(FFTContext *ctx, float input, float dt);
uint8_t processFFT(FFTContext *ctx, uint16_t processingBudget);

#endif
