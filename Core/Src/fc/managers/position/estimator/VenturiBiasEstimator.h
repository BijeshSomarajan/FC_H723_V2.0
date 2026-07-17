#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_H_
#include <sys/_stdint.h>

/* =============================================================================
 *  VENTURI BIAS ESTIMATOR - TUNING GUIDE
 * =============================================================================
 *
 *  WHAT THIS MODELS
 *  ----------------
 *  Airflow over the barometer port during horizontal flight lowers the local
 *  pressure (venturi/dynamic-pressure effect), so the baro reads HIGH - the
 *  EKF thinks the drone climbed, the controller descends, and the drone dips.
 *  This module estimates that pressure artifact in meters and feeds it to the
 *  EKF as a pseudo-measurement of the baro bias state (BP), which the filter
 *  then subtracts from the baro reading.
 *
 *  THE MODEL CHAIN (each stage has its own constants below):
 *
 *    pitch --(deadband/clamp)--> lateralAccel = tan(pitch)*g*ACCEL_GAIN
 *          --(integrate, drag=v*DRAG_GAIN, zero-cross clamp, BRAKE_DWELL,
 *             DAMPING drain when level)--> lateralSpeed  [pitch-proxy, m/s]
 *          --> bias = lateralSpeed^2 * BIAS_GAIN, clamped to BIAS_VALUE_MAX
 *          --(LPF at BIAS_LPF_FREQ)--> venturiBias [m] --> EKF BP state
 *
 *  KNOWN LIMITS OF THE PITCH PROXY (accept, don't tune around):
 *  - Wind-blind: in a hover-in-wind, airspeed exists with little pitch until
 *    the model's slow integrator catches up. Gust artifacts are handled by
 *    the baro dynamic-R machinery, not here.
 *  - Direction-symmetric: bias = v^2 is the same fwd/bwd, but the real
 *    artifact is asymmetric (port placement). Direction-split gains are the
 *    planned fix once GAIN_BWD is measured (see BIAS_GAIN notes).
 *  - Model speed is NOT ground speed. All gains below are calibrated against
 *    the MODEL's speed, not GPS speed. If a real speed source (GNSS/flow) is
 *    ever fed in, every speed-referenced gain must be re-measured.
 *
 *  THE ONE LAW TO REMEMBER WHEN TUNING BIAS_GAIN:
 *    Any compensation error during cruise becomes a REAL altitude offset of
 *    the opposite sign, repaid as a transient at the next stop.
 *      Under-compensate -> flies LOW in cruise, RISES at the stop.
 *      Over-compensate  -> flies HIGH in cruise, DIPS at the stop.
 *    The cruise symptom and the stop symptom are the same error - fix the
 *    gain, not the transition.
 *
 *  HOW TO CALIBRATE (the only honest way to set BIAS_GAIN):
 *    Fly a steady 5+ s cruise leg at 1.5-2 m altitude, away from walls and
 *    people, logging baro(altitudeSLFiltered), EKF z, and lateralSpeed.
 *    artifact = (baro - EKF z) averaged over the steady segment.
 *    BIAS_GAIN = artifact / lateralSpeed^2.
 *    Repeat backward for the (pending) GAIN_BWD.
 * ============================================================================= */

typedef struct _VENTURI_ESTIMATE_DATA VENTURI_ESTIMATE_DATA;
struct _VENTURI_ESTIMATE_DATA {
	float pitchAngleAbsFiltered;
	float venturiBias;       // final output fed to EKF BP state [m]
	float lateralSpeed;      // pitch-proxy horizontal speed state [m/s], signed
	float brakeDwell;        // remaining hold time after a braking zero-cross [s]
	float effectiveThrottle;
};
extern VENTURI_ESTIMATE_DATA venturiEstimateData;

/* ---------------- Input conditioning ---------------- */

/* Pitch deadband, deg. Below this the model sees zero tilt and the DAMPING
 * drain runs instead of integration.
 * Raise (1-2 deg): hover trim / wind-hold tilt no longer builds phantom speed
 *   - use if the bias creeps up during a long leaning hover.
 * Lower: catches slow-cruise tilt, but a miscalibrated level trim then
 *   integrates forever. 0.5 assumes a well-trimmed horizon (which this
 *   airframe has, post-Mahony fixes). */
#define VENTURI_EST_PITCH_ANGLE_MIN             0.5f

/* Pitch clamp, deg. Caps the model's accel input during aggressive maneuvers
 * so a stunt doesn't slingshot the speed state. Matches the attitude
 * envelope; no reason to tune independently of it. */
#define VENTURI_EST_PITCH_ANGLE_MAX             30.0f

/* ---------------- Speed-model dynamics ---------------- */

/* Hard cap on the model speed state, m/s. Pure runaway protection - with
 * BIAS_GAIN 0.025 the bias clamp saturates at ~4.5 m/s anyway, so this
 * never binds in normal flight. Not a tuning knob. */
#define VENTURI_EST_SPEED_MAX                   25.0f

/* Accel mapping gain: lateralAccel = tan(pitch)*g*THIS. 1.0 would be ideal
 * drag-free physics; 1.75 makes the model speed RAMP faster than the real
 * airframe, deliberately, so compensation isn't late on cruise entry (the
 * measured lag chain: model + LPF + BP fusion).
 * Raise: compensation leads more on entry (risk: overshoots the transient).
 * Lower: compensation lags entry -> dip during acceleration returns.
 * Couples with DRAG_GAIN (together they set ramp time AND terminal speed) -
 * change them as a pair or re-calibrate BIAS_GAIN afterward. */
#define VENTURI_EST_ACCEL_GAIN                  1.75f

/* Model drag: decel = speed*THIS. Sets terminal model speed
 * (= tan(pitch)*g*ACCEL_GAIN/THIS: ~12 m/s at 10 deg) and the convergence
 * time constant (1/THIS = 4 s).
 * Raise: model settles faster and lower (snappier decay after leveling).
 * Lower: more momentum, slower everything.
 * WARNING: terminal-speed changes silently re-scale the bias at cruise -
 * re-run the BIAS_GAIN calibration after touching this. */
#define VENTURI_EST_DRAG_GAIN                   0.25f

/* Hold time after a braking zero-cross, s. THE fix for the brake-pitch
 * misfire: during a brake, pitch opposes velocity, and without this hold the
 * model reads the brake tail as flight in the opposite direction (measured:
 * -3.9 m/s phantom -> 0.45 m false bias -> the dip-then-rise at every stop).
 * Sized from logs: this airframe's brake-pitch tails run ~0.7 s, so 0.8
 * covers them (0.5 let a +1.2 m/s phantom through the tail).
 * Raise if a phantom bias still appears right after stops (log lateralSpeed
 * rebuilding during the brake tail). Cost of raising: a genuine direction
 * reversal waits this long before compensation resumes - negligible, since
 * the bias needs seconds to matter anyway. */
#define VENTURI_EST_BRAKE_DWELL                 0.8f

/* Level-flight drain rate, 1/s: speed *= (1 - THIS*dt) when |pitch| is inside
 * the deadband. tau = 1/2.5 = 0.4 s - how fast compensation bleeds off after
 * the sticks are released and the drone is level.
 * Raise: bias exits faster at stops (risk: exits before the real suction
 *   does -> momentary under-compensation blip).
 * Lower: bias lingers after stops -> the estimate reads low -> post-stop
 *   climb. Tune only from a log showing the bias/artifact decay mismatch. */
#define VENTURI_EST_DAMPING_GAIN                2.5f

/* ---------------- Speed -> bias translation ---------------- */

/* Quadratic gain: bias[m] = lateralSpeed^2 * THIS.  ** THE calibrated core **
 * MEASURED, not guessed: two independent logs gave artifact ~0.32 m at model
 * speed ~3.6 m/s -> 0.32/3.6^2 = 0.025. The previous 0.07 over-compensated
 * ~2.8x, which made the drone secretly fly ~0.15 m HIGH during cruise and
 * DIP at every stop (see THE ONE LAW above).
 * Symptoms: dips at stops / EKF sags below baro-consistent value in cruise
 *   -> gain too HIGH. Rises at stops / dips during cruise -> too LOW.
 * Re-measure (procedure in header) after ANY airframe/port/canopy change,
 * or after touching ACCEL_GAIN/DRAG_GAIN.
 * PENDING: direction split (GAIN_FWD/GAIN_BWD) - the artifact is measured
 * asymmetric with flight direction; backward-leg calibration not yet flown. */
#define VENTURI_EST_BIAS_GAIN                   0.025f//was 0.07f

/* Output clamp, m. Safety ceiling on how much altitude the model may claim
 * the baro is lying by. With GAIN 0.025 this engages at model speed
 * ~4.5 m/s. Raise toward 1.0 only with outdoor high-speed calibration data
 * showing the real artifact exceeds 0.5 m - never to "fix" a dip (that is
 * always the gain or the decay, not the clamp). */
#define VENTURI_EST_BIAS_VALUE_MAX              0.5f

/* Output LPF, Hz (tau ~0.45 s). Matches the pneumatic settling of the real
 * pressure field so the bias doesn't step. Part of the measured ~0.8 s total
 * compensation lag (model + this + BP fusion in the EKF).
 * Raise toward 0.7-1.0 Hz if logs show bias arriving late vs the artifact;
 * lower if the bias output is jittery. Note the EKF's BP fusion adds its own
 * ~0.5 s - tune this from end-to-end logs (artifact vs BP), not in isolation. */
#define VENTURI_EST_BIAS_LPF_FREQ               0.35f

uint8_t initVenturiBiasEstimator(void);
float getVenturiBiasEstimate(float dt);
void resetVenturiBiasEstimator(void);
#endif
