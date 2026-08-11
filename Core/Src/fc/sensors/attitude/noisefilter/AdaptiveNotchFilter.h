#ifndef SRC_FC_SENSORS_ATTITUDE_ADAPTIVENOTCHFILTER_H_
#define SRC_FC_SENSORS_ATTITUDE_ADAPTIVENOTCHFILTER_H_

#include "../../../dsp/FFT.h"
#include "../../../dsp/BiQuadFilter.h"

/*
 * DYNAMIC NOTCH PROFILES — the two differ in only two things.
 *
 * 1) DEEP threshold (dominant). Computed gain is bounded to [-30,-10] dB, so
 *    whether GAIN_THRESHOLD_DEEP is reachable decides the filter TYPE:
 *      P1 (-25, reachable)   -> HYBRID: peaking cut normally, escalates to a
 *                               true NOTCH on strong peaks (sharp phase at CF).
 *      P2 (-35, unreachable) -> ALWAYS a bounded PEAK cut (-30 dB max); the
 *                               NOTCH branch is dead code.
 *    P2 flies "smoother" because it never deploys a true notch, so it keeps
 *    phase margin when the noise peak sits near the control band.
 *
 * 2) EMA alphas (secondary). P2's are HIGHER = faster tracking, slightly
 *    jitterier notch params. Helps when the peak moves with throttle.
 *
 * Note: GAIN_ABSOLUTE_MIN/MAX are inert (gain already sits inside [-30,-10]).
 *       P1's PEAK<->NOTCH morph has no hysteresis and can chatter near -25 dB.
 */
#define ADAPTIVE_NOTCH_PROFILE    1

#if ADAPTIVE_NOTCH_PROFILE == 1

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

    // Gain below which PEAK becomes full NOTCH
    #define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -25.0f

    // Peak detection
    #define ADAPTIVE_NOTCH_SNR_LOW                 2.5f
    #define ADAPTIVE_NOTCH_SNR_HIGH                4.0f

    #define ADAPTIVE_NOTCH_MAG_MIN                 6.0f
    #define ADAPTIVE_NOTCH_MAG_MAX                15.0f

    // EMA smoothing
    #define ADAPTIVE_NOTCH_ALPHA_Q                 0.25f
    #define ADAPTIVE_NOTCH_ALPHA_GAIN              0.20f
    #define ADAPTIVE_NOTCH_ALPHA_CF_GAIN           0.25f

/*
 * ============================================================================
 * PROFILE 2 - SMOOTHER
 * ============================================================================
 *
 */
#elif ADAPTIVE_NOTCH_PROFILE == 2

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


    // Gain below which PEAK becomes full NOTCH
    #define ADAPTIVE_NOTCH_GAIN_THRESHOLD_DEEP   -35.0f

    // Peak detection
    #define ADAPTIVE_NOTCH_SNR_LOW                 2.5f
    #define ADAPTIVE_NOTCH_SNR_HIGH                4.0f

    #define ADAPTIVE_NOTCH_MAG_MIN                 6.0f
    #define ADAPTIVE_NOTCH_MAG_MAX                15.0f

    // EMA smoothing
    #define ADAPTIVE_NOTCH_ALPHA_Q                 0.45f
    #define ADAPTIVE_NOTCH_ALPHA_GAIN              0.35f
    #define ADAPTIVE_NOTCH_ALPHA_CF_GAIN           0.45f

#endif


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
void processAdaptiveNotchFilter(
    BIQUADFILTER *filter,
    AdaptiveNotchFilterState *state,
    float targetFreq,
    int peakBin,
    float *magnitudes
);


#endif /* SRC_FC_SENSORS_ATTITUDE_ADAPTIVENOTCHFILTER_H_ */
