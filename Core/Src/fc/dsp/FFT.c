#include "FFT.h"

#include <math.h>
#include <string.h>
#include <stddef.h>
#include "../util/CommonUtil.h"
#include "../util/MathUtil.h"
#include "../memory/Memory.h"

/* ---------- Globals ---------- */
static int fftLog2InputLength;
static float fftSamplingFrequency;
static float fftBinWidth;

static FFTData fftTwiddleFactors[FFT_HALF_N];
static float fftHannWindowValues[FFT_N];
static float fftBinFrequencies[FFT_HALF_N + 1];

static const float FFT_HANN_GAIN = 0.5f;
static float fftWindowGain = FFT_ENABLE_WINDOWING ? FFT_HANN_GAIN : 1.0f;
static float fftScaleFull = 0;
static float fftScaleEdge = 0;

int fftNoiseFloorStartBin = 1;
int fftNoiseFloorEndBin = FFT_HALF_N;
float fftFrequencySlewMs;

/* ---------- Setup Helpers ---------- */
static void calculateFFTLog2InputLength(void) {
	fftLog2InputLength = 0;
	int n = FFT_N;
	while ( n > 1 ) {
		fftLog2InputLength++;
		n >>= 1;
	}
}

static void calculateFFTHannWindowValues(void) {
#if FFT_ENABLE_WINDOWING
	for (int i = 0; i < FFT_N; i++) {
		fftHannWindowValues[i] = 0.5f * (1.0f - cosf(2.0f * ONE_PI * i / (FFT_N - 1)));
	}
#else
    (void)fftHannWindowValues;
#endif
}

static void calculateFFTTwiddleFactors(void) {
	for (int i = 0; i < FFT_HALF_N; i++) {
		float angle = -2.0f * ONE_PI * i / FFT_N;
		fftTwiddleFactors[i].real = cosf(angle);
		fftTwiddleFactors[i].imag = sinf(angle);
	}
}

static void calculateFFTBinFrequencies(void) {
	for (int i = 0; i <= FFT_HALF_N; i++) {
		fftBinFrequencies[i] = (float) i * fftSamplingFrequency / FFT_N;
	}
}

static void caclulateFFTFScales(void) {
	float invN = 1.0f / (float) FFT_N;
	fftScaleFull = (2.0f * invN) / fftWindowGain;
	fftScaleEdge = invN / fftWindowGain;
}

/* ---------- Public API ---------- */
void initFFT(float sampleFrequency) {
	fftSamplingFrequency = sampleFrequency;
	fftBinWidth = sampleFrequency / FFT_N;
	calculateFFTLog2InputLength();
	calculateFFTTwiddleFactors();
	calculateFFTBinFrequencies();
	calculateFFTHannWindowValues();
	caclulateFFTFScales();
}

void initFFTContext(FFTContext *ctx) {
	if (!ctx) {
		return;
	}
	ctx->fftSampleCount = 0;
	ctx->fftSamplingFrequency = fftSamplingFrequency;
	for (int i = 0; i < FFT_N; i++) {
		ctx->fftData[i].real = 0.0f;
		ctx->fftData[i].imag = 0.0f;
	}
	for (int i = 0; i <= FFT_HALF_N; i++) {
		ctx->fftMagnitudes[i] = 0.0f;
	}
	for (int i = 0; i < FFT_TOP_FREQ_N; i++) {
		ctx->topFreq[i] = 0.0f;
		ctx->topMagn[i] = 0.0f;
	}
	ctx->processing = 0;
	ctx->phase = FFT_PHASE_IDLE;
	ctx->br_i = 0;
	ctx->br_reversed = 0;
	ctx->stage = 1;
	ctx->start = 0;
	ctx->i = 0;
	memset(ctx->windowBuffer, 0, sizeof(ctx->windowBuffer));

#if FFT_ENABLE_SAMPLE_LPF == 1
	lowPassFilterInit(&ctx->sampleLPF, FFT_ENABLE_SAMPLE_LPF_CUTOFF_FREQUENCY);
#endif

	fftNoiseFloorStartBin = (int) ((FFT_NOISE_FLOOR_MIN_HZ * FFT_N) / ctx->fftSamplingFrequency);
	fftNoiseFloorEndBin = (int) (FFT_MAX_BIN_FRACTION_FOR_NOISE * FFT_HALF_N);

	if (fftNoiseFloorStartBin < 1) {
		fftNoiseFloorStartBin = 1;
	}
	if (fftNoiseFloorEndBin > FFT_HALF_N) {
		fftNoiseFloorEndBin = FFT_HALF_N;
	}

	// 2. Apply Slew Rate Limiting
	// Max slew: e.g., 2.0Hz per ms (2000Hz/sec)
	// Hop time: (FFT_HOP_SIZE * 1000.0f) / ctx->fftSamplingFrequency
	fftFrequencySlewMs = (float) (FFT_HOP_SIZE * 1000.0f) / ctx->fftSamplingFrequency;

}

__ATTR_ITCM_TEXT
void startFFTProcess(FFTContext *ctx) {
	ctx->processing = 1;
	ctx->phase = FFT_PHASE_WINDOW; // start windowing
	ctx->br_i = 0;
	ctx->br_reversed = 0;
	ctx->stage = 1;
	ctx->start = 0;
	ctx->i = 0;
}

__ATTR_ITCM_TEXT
void updateAdaptiveNoiseFloor(FFTContext *ctx, float peak) {
	float sum = 0.0f;
	int count = 0;
	for (int bin = fftNoiseFloorStartBin; bin < fftNoiseFloorEndBin; bin++) {
		float m = ctx->fftMagnitudes[bin];
		// Reject bins close to dominant peak
		if (m > 0.0f && m < peak * FFT_PEAK_REJECT_RATIO) {
			sum += m;
			count++;
		}
	}
	float instantNoise;
	if (count > 0) {
		instantNoise = sum / (float) count;
	} else {
		instantNoise = FFT_NOISE_MIN_FLOOR;
	}
	// Apply sensitivity scaling
	instantNoise *= FFT_NOISE_MULTIPLIER;
	if (instantNoise < FFT_NOISE_MIN_FLOOR) {
		instantNoise = FFT_NOISE_MIN_FLOOR;
	}
	/* -------- Adaptive EMA -------- */
	if (ctx->noiseFloor <= 0.0f) {
		// First-time initialization
		ctx->noiseFloor = instantNoise;
	} else if (instantNoise > ctx->noiseFloor) {
		// Noise rising → slow update
		ctx->noiseFloor += FFT_NOISE_EMA_RISE_ALPHA * (instantNoise - ctx->noiseFloor);
	} else {
		// Noise falling → faster recovery
		ctx->noiseFloor += FFT_NOISE_EMA_FALL_ALPHA * (instantNoise - ctx->noiseFloor);
	}
	// Hard safety clamp
	if (ctx->noiseFloor < FFT_NOISE_MIN_FLOOR) {
		ctx->noiseFloor = FFT_NOISE_MIN_FLOOR;
	}
}

__ATTR_ITCM_TEXT
float getInterpolatedFrequency(const float *magnitudes, int peakBin, float binWidth) {
	// We need the neighbor bins to interpolate.
	// If the peak is at the very edge (bin 0 or bin N/2), we cannot interpolate.
	if (peakBin <= 0 || peakBin >= FFT_HALF_N) {
		return (float) peakBin * binWidth;
	}

	float alpha = magnitudes[peakBin - 1]; // Left neighbor
	float beta = magnitudes[peakBin];     // The peak
	float gamma = magnitudes[peakBin + 1]; // Right neighbor

	// Formula: k = 0.5 * (alpha - gamma) / (alpha - 2*beta + gamma)
	float denom = alpha - (2.0f * beta) + gamma;
	float k = 0.0f;

	// Standard epsilon check to avoid division by zero
	if (fabsf(denom) > 1e-6f) {
		k = 0.5f * (alpha - gamma) / denom;
	}

	// Constrain k to +/- 0.5 bins to ensure the peak stays within the bin's logical area
	if (k > 0.5f) k = 0.5f;
	if (k < -0.5f) k = -0.5f;

	return ((float) peakBin + k) * binWidth;
}

/**
 * @brief Limits the rate of change of the detected frequency to prevent jitter.
 * @param currentFreq The newly detected frequency from the FFT.
 * @param lastFreq The frequency used in the previous frame.
 * @param maxSlewHzPerMs Maximum allowed change in Hz per millisecond.
 * @param timeMs Time elapsed since the last update (Hop period).
 * @return The smoothed frequency.
 */
static float applyFrequencySlewLimit(float currentFreq, float lastFreq, float maxSlewHzPerMs, float timeMs) {
	if (lastFreq <= 0.0f) {
		return currentFreq; // Initial lock
	}
	float delta = currentFreq - lastFreq;
	float maxChange = maxSlewHzPerMs * timeMs;

	if (fabsf(delta) > maxChange) {
		if (delta > 0) {
			return lastFreq + maxChange;
		} else {
			return lastFreq - maxChange;
		}
	}
	return currentFreq;
}

__ATTR_ITCM_TEXT
uint8_t processFFT(FFTContext *ctx, uint16_t processingBudget) {
	if (!ctx || !ctx->processing) return 0;

// Incremental windowing
	if (ctx->phase == FFT_PHASE_WINDOW) {

#if FFT_ENABLE_BUDGET_FOR_WINDOWING == 1
		while ( processingBudget > 0 && ctx->i < FFT_N ) {
#else
		while ( ctx->i < FFT_N ) {
#endif

#if FFT_ENABLE_WINDOWING
			ctx->fftData[ctx->i].real *= fftHannWindowValues[ctx->i];
#endif

			ctx->i++;

#if FFT_ENABLE_BUDGET_FOR_WINDOWING == 1
			processingBudget--;
#endif
		}

		if (ctx->i < FFT_N) {
			return 0;
		}
		ctx->i = 0;
		ctx->phase = FFT_PHASE_BITREV;
	}

// Bit reversal
	if (ctx->phase == FFT_PHASE_BITREV) {
		while ( processingBudget > 0 && ctx->br_i < FFT_N ) {
			int i = ctx->br_i++;
			int reversed = ctx->br_reversed;
			if (i < reversed) {
				FFTData tmp = ctx->fftData[i];
				ctx->fftData[i] = ctx->fftData[reversed];
				ctx->fftData[reversed] = tmp;
			}
			int mask = FFT_HALF_N;
			while ( reversed & mask ) {
				reversed &= ~mask;
				mask >>= 1;
			}
			reversed |= mask;
			ctx->br_reversed = reversed;
			processingBudget--;
		}
		if (ctx->br_i < FFT_N) {
			return 0;
		}
		ctx->phase = FFT_PHASE_STAGES;
		ctx->stage = 1;
		ctx->start = 0;
		ctx->i = 0;
	}

// Radix-2 stages
	if (ctx->phase == FFT_PHASE_STAGES) {
		for (; ctx->stage <= fftLog2InputLength;) {
			int stage = ctx->stage;
			int step = 1 << stage;
			int half = step >> 1;
			int twStep = FFT_N / step;
			for (; ctx->start < FFT_N;) {
				int start = ctx->start;
				for (; ctx->i < half;) {
					if (processingBudget <= 0) {
						return 0;
					}
					FFTData *top = &ctx->fftData[start + ctx->i];
					FFTData *bot = &ctx->fftData[start + ctx->i + half];
					int twIndex = ctx->i * twStep;
					if ((unsigned) twIndex >= (unsigned) FFT_HALF_N) twIndex %= FFT_HALF_N;
					FFTData W = fftTwiddleFactors[twIndex];
					float t_re = W.real * bot->real - W.imag * bot->imag;
					float t_im = W.real * bot->imag + W.imag * bot->real;
					float newBotRe = top->real - t_re;
					float newBotIm = top->imag - t_im;
					top->real = top->real + t_re;
					top->imag = top->imag + t_im;
					bot->real = newBotRe;
					bot->imag = newBotIm;
					ctx->i++;
					processingBudget--;
				}
				ctx->i = 0;
				ctx->start += step;
			}
			ctx->start = 0;
			ctx->stage++;
		}
		ctx->phase = FFT_PHASE_DONE;
	}

// Finalize magnitudes
	if (ctx->phase == FFT_PHASE_DONE) {
		ctx->fftSampleCount = 0;
		float re0 = ctx->fftData[0].real;
		float im0 = ctx->fftData[0].imag;
#if FFT_USE_FAST_SQRT==1
		ctx->fftMagnitudes[0] = fftScaleEdge * fastSqrtf(re0 * re0 + im0 * im0);
#else
        ctx->fftMagnitudes[0] = fftScaleEdge * sqrtf(re0*re0+im0*im0);
#endif
		float reN2 = ctx->fftData[FFT_HALF_N].real;
		float imN2 = ctx->fftData[FFT_HALF_N].imag;
#if FFT_USE_FAST_SQRT==1
		ctx->fftMagnitudes[FFT_HALF_N] = fftScaleEdge * fastSqrtf(reN2 * reN2 + imN2 * imN2);
#else
        ctx->fftMagnitudes[FFT_HALF_N] = fftScaleEdge * sqrtf(reN2*reN2+imN2*imN2);
#endif
		float peak = 0.0f;
		int peakBin = 0;
		for (int bin = 1; bin < FFT_HALF_N; ++bin) {
#if FFT_USE_FAST_SQRT==1
			float raw = fastSqrtf(ctx->fftData[bin].real * ctx->fftData[bin].real + ctx->fftData[bin].imag * ctx->fftData[bin].imag);
#else
            float raw = sqrtf(ctx->fftData[bin].real*ctx->fftData[bin].real + ctx->fftData[bin].imag*ctx->fftData[bin].imag);
#endif
			float m = fftScaleFull * raw;
			ctx->fftMagnitudes[bin] = m;
			if (m > peak) {
				peak = m;
				peakBin = bin;
			}
		}

		updateAdaptiveNoiseFloor(ctx, peak);
		ctx->peakNoise = peak;
		if (peak < ctx->noiseFloor) {
			for (int i = 0; i < FFT_TOP_FREQ_N; ++i) {
				ctx->topFreq[i] = 0.0f;
				ctx->topMagn[i] = 0.0f;
				ctx->topFreqBin[i] = 1.0f;
			}
		} else {

#if FFT_MODE==FFT_MODE_HARMONICS
			float rawFreq = getInterpolatedFrequency(ctx->fftMagnitudes, peakBin, fftBinWidth);
			ctx->topFreq[0] = applyFrequencySlewLimit(rawFreq, ctx->topFreq[0], FFT_MAX_FREQ_SLEW_HZ_PER_MS, fftFrequencySlewMs);
			ctx->topMagn[0] = peak;
			ctx->topFreqBin[0] = peakBin;

#if FFT_CALCULATE_HARMONIC_FREQUENCIES == 1
			for (int h = 2; h <= FFT_TOP_FREQ_N; ++h) {
				int hb = h * peakBin;
				if (hb <= FFT_HALF_N) {
					ctx->topFreq[h - 1] = ctx->topFreq[0] * h;
					ctx->topMagn[h - 1] = ctx->fftMagnitudes[hb];
					ctx->topFreqBin[h - 1] = hb;
				}
			}
#else

			for (int h = 2; h <= FFT_TOP_FREQ_N; ++h) {
				int hb = h * peakBin;
				if (hb <= FFT_HALF_N) {
					ctx->topFreq[h - 1] = getInterpolatedFrequency(ctx->fftMagnitudes, hb, fftBinWidth);//fftBinFrequencies[hb];
					ctx->topMagn[h - 1] = ctx->fftMagnitudes[hb];
					ctx->topFreqBin[h - 1] = hb;
				}
			}
#endif

#else
			for (int i = 0; i < FFT_TOP_FREQ_N; i++) {
				ctx->topFreq[i] = 0.0f;
				ctx->topMagn[i] = 0.0f;
			}

			for (int b = 1; b < FFT_HALF_N; b++) {
				float m = ctx->fftMagnitudes[b];
				float f = fftBinFrequencies[b];

				if (m > ctx->topMagn[0]) {
					ctx->topMagn[2] = ctx->topMagn[1];
					ctx->topFreq[2] = ctx->topFreq[1];

					ctx->topMagn[1] = ctx->topMagn[0];
					ctx->topFreq[1] = ctx->topFreq[0];

					ctx->topMagn[0] = m;
					ctx->topFreq[0] = f;
				} else if (m > ctx->topMagn[1]) {
					ctx->topMagn[2] = ctx->topMagn[1];
					ctx->topFreq[2] = ctx->topFreq[1];

					ctx->topMagn[1] = m;
					ctx->topFreq[1] = f;
				} else if (m > ctx->topMagn[2]) {
					ctx->topMagn[2] = m;
					ctx->topFreq[2] = f;
				}
			}
#endif
		}
		ctx->processing = 0;
		ctx->phase = FFT_PHASE_IDLE;
		ctx->hasProcessUpdate = 1;
		return 1;
	}
	return 0;
}

/* ---------- updateFFT: feed samples ---------- */
__ATTR_ITCM_TEXT uint8_t updateFFT(FFTContext *ctx, float input, float dt) {

#if FFT_ENABLE_SAMPLE_LPF == 1
	input = lowPassFilterUpdate(&ctx->sampleLPF, input, dt);
#endif

	ctx->windowBuffer[FFT_N - FFT_HOP_SIZE + ctx->fftSampleCount] = input;
	ctx->fftSampleCount++;
	if (ctx->fftSampleCount < FFT_HOP_SIZE) {
		return 0;
	} else {
		memmove(&ctx->windowBuffer[0], &ctx->windowBuffer[FFT_HOP_SIZE], (FFT_N - FFT_HOP_SIZE) * sizeof(float));
		for (int k = 0; k < FFT_N; k++) {
			ctx->fftData[k].real = ctx->windowBuffer[k];
			ctx->fftData[k].imag = 0.0f;
		}
		ctx->fftSampleCount = 0;
		startFFTProcess(ctx);
	}
	return 1;
}
