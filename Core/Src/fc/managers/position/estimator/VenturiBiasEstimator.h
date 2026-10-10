#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_H_
#include <sys/_stdint.h>

/* =============================================================================
 *  VENTURI BIAS ESTIMATOR
 * =============================================================================
 *  Airflow over the baro port in horizontal flight lowers local pressure, so
 *  baro reads HIGH -> EKF thinks we climbed -> controller descends -> dip.
 *  This module estimates that artifact [m] and feeds it to the EKF BP state.
 *
 *  Model chain:
 *    pitch -> lateral accel -> integrated model speed (drag, brake dwell,
 *    damping when level) -> bias = speed^2 * BIAS_GAIN (clamped) -> LPF
 *    -> venturiBias [m] -> EKF BP state
 *
 *  Known limits (accept, don't tune around):
 *  - Wind-blind: hover in wind has airspeed with little pitch. Gusts are
 *    handled by baro dynamic-R, not here.
 *  - Direction-symmetric: real artifact is asymmetric (port placement).
 *    Split fwd/bwd gains once the backward leg is calibrated.
 *  - Model speed != ground speed. Gains are calibrated against model speed;
 *    re-measure if a real speed source is ever used.
 *
 *  Tuning law for BIAS_GAIN: cruise compensation error becomes a real altitude
 *  offset, repaid at the next stop.
 *    Too low  -> flies LOW in cruise, RISES at stop.
 *    Too high -> flies HIGH in cruise, DIPS at stop.
 *  Fix the gain, not the transition.
 *
 *  Calibration: steady 5+ s cruise at 1.5-2 m, away from walls. Log baro, EKF z,
 *  model speed. BIAS_GAIN = mean(baro - EKF z) / speed^2. Repeat backward for
 *  GAIN_BWD (pending).
 * ============================================================================= */

typedef struct _VENTURI_ESTIMATE_DATA VENTURI_ESTIMATE_DATA;
struct _VENTURI_ESTIMATE_DATA {
	float venturiBias;        // output to EKF BP state [m]
	float lateralSpeedMag;    // sqrt(vPitch^2 + vRoll^2) [m/s]
};
extern VENTURI_ESTIMATE_DATA venturiEstimateData;

/* Model speed cap [m/s]. Runaway protection only, not a tuning knob. */
#define VENTURI_EST_SPEED_MAX                   50.0f
#define VENTURI_EST_SPEED_MIN                   0.1f
/* Output clamp [m]. Safety ceiling on claimed baro error. Raise only with
 * high-speed data showing the artifact exceeds it, never to fix a dip. */
#define VENTURI_EST_BIAS_VALUE_MAX              1.0f
/* Output LPF cutoff [Hz], tau = 1/(2*pi*f). Should match the pneumatic settling
 * of the real artifact. Tune from end-to-end logs (artifact vs BP), since EKF
 * BP fusion adds its own lag. */
#define VENTURI_EST_BIAS_LPF_FREQ               25.0f
/* bias[m] = speed^2 * GAIN. The calibrated core.
 * Measured: 0.32 m artifact at 3.6 m/s model speed -> 0.025.
 * Dips at stops -> too high. Rises at stops -> too low.
 * Re-measure after any airframe/port/canopy change or ACCEL/DRAG gain change. */
#define VENTURI_EST_BIAS_GAIN_DEFAULT     0.036f

uint8_t initVenturiBiasEstimator(void);
float getVenturiBiasEstimate(float dt, float speed);
void resetVenturiBiasEstimator(void);
#endif
