#ifndef _ALTITUDECONTROL_H_
#define _ALTITUDECONTROL_H_

#include <stdio.h>
#include <inttypes.h>

/* =============================================================================
 *  ALTITUDE CONTROL - CASCADE STRUCTURE & TUNING GUIDE
 * =============================================================================
 *
 *  CASCADE (outer -> inner):
 *    altPID      : altitude error -> velocity target    [P only]
 *    altRatePID  : velocity error -> accel target       [P I D]  <-- ONLY integrator
 *    altAccPID   : accel error    -> throttle correction[P D]
 *
 *  PLANT MODEL: thrust for vertical accel a is m*(g + a), so
 *      throttleDelta = hoverThrottle * (a / g)
 *      K = hoverThrottle / GRAVITY_MSS      [throttle units per m/s^2]
 *  K is used FORWARD as feedforward (accel target -> throttle) and INVERSE in
 *  the disturbance observer (throttle output -> expected accel).
 *  Reference point for this airframe: hoverThrottle ~500 -> K ~51.
 *
 *  WHY: previously the acc PID had to manufacture every throttle unit from
 *  acceleration ERROR alone. With Acc Kp 80 and K 51, the acc loop delivered
 *  only Kp/(K+Kp) = 61% of the requested acceleration - the outer loops were
 *  tuned against a plant that silently under-delivered by 39%. Feedforward
 *  restores delivery to ~1.0, which also IMPROVES outer-loop damping
 *  (zeta = 0.5*sqrt(Kv/Kp): 0.68 -> 0.87 at Vel P 3.0 / Master P 1.0).
 *  Do NOT reduce Vel P when enabling FF.
 *
 *  SYMPTOM -> KNOB
 *  ---------------
 *  Sluggish response to throttle/alt steps -> ACC_FF_GAIN up (toward 1.0)
 *  Overshoot on steps / slow bobbing       -> check Alt Vel LIMIT windup
 *                                             first, then ACC_FF_GAIN down
 *  Sinks/rises in wind, slow recovery      -> DOB_GAIN up (0.5 -> 0.8)
 *  Oscillation at a few Hz with DOB on     -> THRUST_TAU mismatched: measure
 *                                             real command->accel lag
 *  Fast throttle hunting / buzz (few Hz)   -> acc Kp too high for the thrust
 *                                             lag: 80 -> 50 (loop gain ~1.0).
 *                                             Verify THRUST_TAU by measurement.
 *  Soft response, DOB doing all the work   -> raise DOB_GAIN toward 0.8 before
 *                                             raising acc Kp (DOB has the model)
 *  Slow standing offset in steady wind     -> altRatePID Ki (calibration)
 *
 *  --------------------------------------------------------------------------
 *  RELATED CALIBRATION VALUES (flash, set via config app - NOT defined here)
 *  --------------------------------------------------------------------------
 *    Alt Master P 1.0, LIMIT 2.5 m/s   -> ~1 s altitude time constant
 *    Alt Vel  P 3.0, I 2.0, D 0
 *    Alt Vel  LIMIT 3.0 m/s^2  <- MUST NOT exceed OUTPUT_LIMIT/K (= 4.9), and
 *        is set below it so FF leaves room for correction + DOB. Was 16, i.e.
 *        3.3x beyond anything the plant can deliver: the integrator could wind
 *        into unreachable territory, and this loop has NO back-calculation
 *        anti-windup (clamp + resetAltitudeRIControl only), so it unwound only
 *        by accumulating opposite error over seconds -> post-transient balloon.
 *    Alt Acc  P 80 (loop gain 80/K = 1.57), D 0
 *    Alt Acc  LIMIT 100  <- correction budget only, since FF now carries the
 *        command. Leaving it at 250 lets the correction swallow the whole
 *        output budget and clamp the feedforward away.
 *  RE-DERIVE all of the above if hover throttle changes materially (payload,
 *  props, battery) - every relationship above is anchored to K.
 *
 *  NOTE: there is no D anywhere in this cascade. All damping comes from the
 *  cascade structure itself. If step response stays soft after FF, a small
 *  Alt Vel D is the lever - not more P.
 * ============================================================================= */

typedef struct _ALTITUDE_CONTROL_GAINS ALTITUDE_CONTROL_GAINS;
struct _ALTITUDE_CONTROL_GAINS {
	float masterPGain;
	float ratePGain;
	float rateIGain;
	float rateIBleed;
	float rateDGain;
	float accPGain;
	float accDGain;

};

uint8_t initAltitudeControl(void);
void resetAltitudeControl(uint8_t hard);
void resetAltitudeControlMaster(void);
void resetAltitudeControlRate(void);

void resetAltitudeRIControl(void);
void setAltitudeRIControl(float value);

void applyAltitudeControlMPMinLimitToValue(float value);
void applyAltitudeControlRIMinLimitToValue(float value);

void resetAltitudeRateControl(void);
void resetAltitudeMasterControl(void);

void controlAltitudeAltWithGains(float dt, float expectedAltitude, float currentAltitude, ALTITUDE_CONTROL_GAINS altControlGains);
void controlAltitudeVelWithGains(float dt, ALTITUDE_CONTROL_GAINS altControlGains);
void controlAltitudeAccWithGains(float dt, ALTITUDE_CONTROL_GAINS altControlGains);

#define ALT_CONTROL_RATE_PID_D_LPF_FREQ 32.0f
#define ALT_CONTROL_ACC_PID_D_LPF_FREQ  32.0f

#define ALT_CONTROL_RATE_PID_I_LIMIT_RATIO 1.0f
#define ALT_CONTROL_RATE_PID_D_LIMIT_RATIO 1.0f
#define ALT_CONTROL_ACC_PID_D_LIMIT_RATIO 1.0f

/* --------------------------------------------------------------------------
 * Hover-throttle learner
 * --------------------------------------------------------------------------
 * Liftoff throttle is measured in ground effect and does not track battery
 * sag, so it is used only as the SEED. In flight the true hover throttle is
 * learned from the actual mixed throttle whenever the vehicle is essentially
 * not climbing and not heavily tilted.
 *
 * IMPORTANT: the learner reads controlData.throttleControl, which INCLUDES
 * tiltCompThDelta and posBrakeCompThDelta. The lift-factor gate below is what
 * keeps tilt compensation out of the learned hover value - it is NOT
 * redundant with the velocity gate. Do not remove it.
 */
// [1] learn in flight | [0] stay on the liftoff seed forever
#define ALT_CONTROL_HOVER_LEARN_ENABLED        1
// Learner time constant, s. Long: this is a slow trim, not a tracker.
#define ALT_CONTROL_HOVER_LEARN_TAU            8.0f
// Only learn when |zVelocity| is below this (m/s) - i.e. actually hovering.
#define ALT_CONTROL_HOVER_LEARN_VEL_MAX        0.25f
// Only learn when tilt lift factor cos(pitch)*cos(roll) is above this
// (~cos(12deg)); tilted flight needs extra throttle that is NOT hover thrust.
#define ALT_CONTROL_HOVER_LEARN_LIFT_MIN       0.978f
// Sanity band around the liftoff seed - the learner may never wander outside.
#define ALT_CONTROL_HOVER_LEARN_MIN_RATIO      0.60f
#define ALT_CONTROL_HOVER_LEARN_MAX_RATIO      1.60f

/* --------------------------------------------------------------------------
 * Acceleration feedforward (replaces ALT_CONTROL_VEL_FEED_FWD)
 * --------------------------------------------------------------------------
 * output = accelTarget * K * ACC_FF_GAIN
 * 1.0 = full model authority (exact tracking, independent of acc Kp).
 * Start at 0.5, raise if response is sluggish. At Vel LIMIT 3.0 m/s^2 and
 * K = 51, FF alone can reach 3.0 * 51 * 1.0 = 153 throttle units.
 */
#define ALT_CONTROL_ACC_FF_ENABLED             1
#define ALT_CONTROL_ACC_FF_GAIN                1.0f

/* --------------------------------------------------------------------------
 * Disturbance observer (REAL: compares measured accel against the accel the
 * previous THROTTLE OUTPUT should have produced, through the inverse plant
 * model - NOT against the setpoint, which would merely re-add the acc PID's
 * own error as a lagged proportional term)
 * --------------------------------------------------------------------------
 * This is the wind / battery-sag / ground-effect rejection term. It catches
 * any external vertical force within ~TAU and cancels it directly, instead of
 * leaving the whole job to altRatePID.i working through two cascade layers.
 */
#define ALT_CONTROL_ACC_DISTURBANCE_EST_ENABLED  1
// Thrust response lag (command -> actual accel), s. MUST be modelled or the
// DOB reads its own lag as disturbance and amplifies its own commands (the
// same failure the position DOB had before DOB_ATT_TAU was added).
// 0.12 is an ESTIMATE - measure from a throttle step (command edge to accel
// response) and correct it. Acc Kp's safe ceiling scales inversely with this.
#define ALT_CONTROL_THRUST_TAU                 0.12f
// Disturbance estimate smoothing, s. Lower = more responsive, more noise.
#define ALT_CONTROL_ACC_DISTURBANCE_TAU        0.065f
// How much of the estimated disturbance to cancel. 0.5 is a safe start;
// 0.8-1.0 gives stronger wind rejection once THRUST_TAU is verified.
#define ALT_CONTROL_ACC_DISTURBANCE_FF_GAIN    0.3f
// Sanity clamp on the disturbance estimate, m/s^2. NOTE this is not merely a
// guard: at K = 51 and gain 0.5 it bounds DOB authority to ~102 throttle
// units, a meaningful share of the budget below.
#define ALT_CONTROL_DOB_ACC_LIMIT              4.0f

/* --------------------------------------------------------------------------
 * Total output ceiling, throttle units.
 * --------------------------------------------------------------------------
 * Bounds FF + acc-PID correction + DOB TOGETHER. The acc PID's own limit
 * (calibration, 100) bounds only its own term.
 * Budget at hoverThrottle 500, K = 51 units per m/s^2:
 *     FF        vel target 3.0 m/s^2  ->  153
 *     acc PID   limit 100             ->  100   (~2.0 m/s^2 correction)
 *     DOB       4.0 clamp x 0.5 gain  ->  102   (~2.0 m/s^2)
 * Peaks do not coincide; typical composition ~130. Mixer headroom above hover
 * is ~500 units less ~120 reserved for tilt comp, so 250 fits with margin.
 */
#define ALT_CONTROL_OUTPUT_LIMIT               250.0f


#endif
