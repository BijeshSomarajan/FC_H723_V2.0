#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORCONFIG_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORCONFIG_H_

/* =============================================================================
 *  POSITION ESTIMATOR TUNING GUIDE  (read before touching any value)
 * =============================================================================
 *
 *  MENTAL MODEL
 *  ------------
 *  Per axis the EKF blends two information sources:
 *    - PREDICTION: earth-frame accelerometer integrated at 1 kHz (fast, drifts)
 *    - MEASUREMENTS: GNSS / baro / rangefinder (slow, absolute)
 *  Q says how fast the prediction goes stale. R says how noisy a measurement
 *  is. THE FILTER ONLY CARES ABOUT THE RATIO Q/R:
 *    - Raise Q or lower R  -> output follows measurements faster (snappier,
 *      noisier, sensor artifacts print into the state)
 *    - Lower Q or raise R  -> output leans on the accelerometer (smoother,
 *      but accel bias / tilt error can drag the estimate between fixes)
 *
 *  UNITS & TIME BASE (IMPORTANT)
 *  -----------------------------
 *  All Q values are added PER PREDICT STEP at 1 kHz (NOT per second, NOT
 *  scaled by dt). Effective continuous rate = value x 1000.
 *    e.g. Q_VEL 0.001 per step  ->  1.0 (m/s)^2 per second.
 *  If the estimator task rate ever changes, every Q below silently changes
 *  meaning. // TODO: multiply by dt in positionEKFPredict, then re-state
 *  these as per-second values.
 *
 *  R values are variances: position m^2, velocity (m/s)^2.
 *  sigma = sqrt(R). R = 0.09 means "I believe this measurement to ~0.3 m (1σ)".
 *
 *  Gates are Mahalanobis distance SQUARED: 9.0 = 3σ, 6.0 ≈ 2.45σ, 16 = 4σ.
 *
 *  SYMPTOM -> KNOB (start here when something is wrong)
 *  ----------------------------------------------------
 *  Estimate lags real motion (feels late)          -> raise Q_VEL, or lower
 *                                                     the relevant R floor
 *  Estimate jitters / follows GPS noise            -> raise R floor (HACC/SACC
 *                                                     _MIN) or lower Q
 *  Slow drift-and-snap sawtooth vs raw GNSS        -> raise Q_BIAS (bias state
 *                                                     not absorbing tilt error)
 *  Altitude dips when translating / balloons at
 *  stops                                           -> raise BARO_RP_MIN (trust
 *                                                     baro less) or check
 *                                                     venturi estimator
 *  rejectCount climbing in normal flight           -> gate too tight, or the
 *                                                     latency constant is wrong
 *  5 Hz tick/ring in the controls                  -> SACC_MIN too low (GNSS
 *                                                     velocity steps too hard)
 *
 *  RULES OF THUMB
 *  --------------
 *  - Change ONE value per flight. Log innovation[] and rejectCount[].
 *  - Tune in factors of 2-3, never 10x, except to disable something.
 *  - R floors (_MIN) are the everyday knobs; Q is structural - retune Q only
 *    when the sensor set or airframe changes.
 *  - POS_ESTIMATOR_GNSS_LATENCY_S is a MEASUREMENT, not a tuning knob.
 * ============================================================================= */

/* =========================================================================
 * Group 1: EKF Core Process Noise & Validation Gates (Horizontal Axis - XY)
 * ========================================================================= */

/* Position random walk, m^2 per 1ms step (x1000 = per second).
 * Current: 1e-5 -> 0.01 m^2/s. Deliberately tiny: XY position confidence is
 * carried by the velocity+bias states; this only sets how fast raw position
 * uncertainty grows on top of them. Rarely needs touching.
 * Range: 1e-6 .. 1e-4. Too high: position wanders between fixes. */
#define POS_EKF_X_Q_POS                     0.0003f // was 0.00001f
#define POS_EKF_Y_Q_POS                     0.0003f // was 0.00001f

/* Velocity random walk, (m/s)^2 per step -> 1.0 (m/s)^2/s effective.
 * THE main Q knob for XY feel. Sets how quickly the filter admits "my
 * accel-integrated velocity may be wrong", i.e. how hard GNSS velocity can
 * steer the estimate (together with SACC_MIN below).
 * Raise (e.g. 0.002-0.005): snappier stops, less lag, more GNSS-step noise.
 * Lower (e.g. 0.0005): silkier velocity, but tilt/bias error lingers longer.
 * Tuned against SACC_MIN 0.1 + delay comp; retune if either changes. */
#define POS_EKF_X_Q_VEL                      0.001f //0.001f
#define POS_EKF_Y_Q_VEL                      0.001f //0.001f

/* Accel-bias random walk, (m/s^2)^2 per step -> 0.01/s effective.
 * Absorbs tilt-induced gravity leakage (1 deg attitude error = 0.17 m/s^2
 * phantom accel) and thermal drift. History: raised 1e-7 -> 1e-5 to kill the
 * wind-hold drift-and-snap sawtooth. NOTE: only observable through GNSS -
 * if GNSS trust is ever reduced drastically, this state stops learning.
 * Raise if: velocity estimate ramps between fixes when holding tilt in wind.
 * Lower if: bias trace oscillates at the position-loop frequency (chasing). */
#define POS_EKF_X_Q_BIAS                      0.00001f  // was 1e-7 (absorbs tilt-induced gravity leakage)
#define POS_EKF_Y_Q_BIAS                      0.00001f  // was 1e-7 (absorbs tilt-induced gravity leakage)

/* Innovation gate, Mahalanobis d^2. 9.0 = 3 sigma.
 * Rejects multipath jumps but must pass honest maneuver innovations.
 * History: was 6.0 (2.45σ) - too tight, caused reject cycles in turbulence.
 * With delay comp active, maneuver innovations are small; 9.0 has margin.
 * Widen to 12-16 only if rejectCount ticks during normal aggressive flight. */
#define POS_EKF_X_GATE                         9.0f
#define POS_EKF_Y_GATE                         9.0f

/* Consecutive rejections before covariance inflation (panic recovery).
 * Time-to-recover = value / GNSS rate: 8 rejects @ 5 Hz = 1.6 s of flying
 * blind on inertial before the gate is forced back open (P x PANIC_P_INFLATE).
 * Lower = faster recovery from real divergence, higher risk of swallowing a
 * multipath burst. 1-2 s of inertial-only XY is safe; keep in that band. */
#define POS_EKF_X_PANIC                        8
#define POS_EKF_Y_PANIC                        8

/* =========================================================================
 * Group 2: EKF Core Process Noise & Validation Gates (Vertical Axis - Z)
 * =========================================================================
 * PHILOSOPHY (differs from XY!): Z is BARO-ANCHORED. GNSS-Z is essentially
 * muted (Group 6). The huge Q values below mean "prediction goes stale
 * almost immediately - track the baro", and the baro's authority is then
 * throttled by BARO_RP_MIN 30 (Group 7). Net altitude time constant ~0.3 s.
 * These two groups form ONE tune: do not change Z_Q_* without re-checking
 * BARO_RP_MIN, and vice versa.  (Validated config: dips/balloon-free.) */

/* 0.01 per step -> 10 m^2/s. Huge on purpose - see philosophy above. */
#define POS_EKF_Z_Q_POS                        0.01f

/* 0.02 per step -> 20 (m/s)^2/s. Huge on purpose. Climb-rate follows
 * baro-derivative + accel. If climb feels spongey: raise; if altitude is
 * twitchy on throttle punches: lower toward 0.005 (and retest translation
 * dips - this trades against baro-artifact immunity). */
#define POS_EKF_Z_Q_VEL                        0.02f

/* Z accel-bias walk -> 0.01/s. Same role as XY bias; observable through
 * baro here, so it keeps learning even with GNSS muted. */
#define POS_EKF_Z_Q_BIAS                       0.00001f

/* Baro reference (weather/pressure-datum) bias walk -> 0.01/s.
 * Lets the filter track slow ambient pressure change without calling it a
 * climb. Raise if long hovers show altitude creeping with weather. */
#define POS_EKF_Z_Q_POS_BIAS                   0.00001f

/* 3-sigma gate, same semantics as XY. */
#define POS_EKF_Z_GATE                         9.0f

/* 25 rejects. Baro updates fast (~50 Hz), so this is only ~0.5 s to panic -
 * intentionally quicker than XY: a wedged Z estimate is an altitude runaway. */
#define POS_EKF_Z_PANIC                        25

/* =========================================================================
 * Group 3: Adaptive Q Tuning Engine (Structural Strain Scaling)
 * =========================================================================
 * Inflates Q when |earth-frame linear accel| is high (hard maneuvers /
 * vibration bursts), letting the state slip rather than fight. stress =
 * accel / THRESH, clamped 0..1; Q_eff = Q * (1 + stress * GAIN * ...).
 * These thresholds are in m/s^2 of LINEAR (gravity-removed) acceleration. */
#define POS_EKF_DYNAMIC_Q_ENABLED              1

/* 45 m/s^2 (~4.5 g lateral) before XY Q starts inflating - i.e. only truly
 * violent events. Normal gusts (2-4 m/s^2) never touch it. Lower toward
 * 15-20 only if hard braking visibly upsets the XY estimate. */
#define POS_EKF_ACC_THRESH_XY                  15.0f// 45.0f

/* 60 m/s^2 vertical - hard landings / prop strikes territory. */
#define POS_EKF_ACC_THRESH_Z                   60.0f

/* Ceiling on total Q inflation (x15). Safety bound, not a tuning knob. */
#define POS_EKF_Q_MAX_SCALE                    15.0f

/* Per-state stress gains: how much of the inflation each state receives.
 * Velocity gets 2.5x (it's what slips in a hard maneuver), position 1x,
 * bias 0 (bias must NOT learn from momentary structural transients -
 * keep 0 unless you have a specific frame-flex problem). */
#define POS_EKF_Q_POS_STRESS_GAIN              1.0f
#define POS_EKF_Q_VEL_STRESS_GAIN              2.5f
#define POS_EKF_Q_BIAS_STRESS_GAIN             0.0f

/* Covariance multiplier applied on panic (see *_PANIC above). 10x reopens
 * a 3-sigma gate after one inflation in most divergence cases. */
#define POS_EKF_PANIC_P_INFLATE   10.0f

/* =========================================================================
 * Group 4: Dynamic Sensor Variance Scaling - GNSS Horizontal (XY) Position
 * =========================================================================
 * R = HACC_SCALE * max(hAcc, HACC_MIN)^2, capped at RP_MAX.
 * hAcc is the receiver's own 1-sigma horizontal accuracy report. */

/* Multiplier on receiver-reported accuracy. 1.0 = take the receiver at its
 * word. Raise to 2-4 if the receiver is optimistic (estimate follows
 * multipath excursions the receiver didn't flag). */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_HACC_SCALE        1.0f

/* Trust floor: never believe better than 0.25 m 1-sigma (R floor 0.0625 m^2).
 * THE everyday XY-position-tightness knob.
 * Lower -> tighter hold, more GPS-noise chasing. 0.25 is near the physical
 * floor for standard (non-RTK) GNSS - do not go below ~0.2 without RTK.
 * History: 0.3 -> 0.25 after delay comp made fixes honest during motion. */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_HACC_MIN          0.3f// was 0.3f

/* Variance ceiling 16 m^2 (sigma 4 m): even a terrible fix is processed at
 * this weight rather than ignored (gate handles true outliers). */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RP_MAX            16.0f

/* =========================================================================
 * Group 5: Dynamic Sensor Variance Scaling - GNSS Horizontal (XY) Velocity
 * =========================================================================
 * R = SACC_SCALE * max(sAcc, SACC_MIN)^2, capped at RV_MAX.
 * This channel ANCHORS the XY velocity estimate and makes the accel-bias
 * state observable - it is the backbone of wind-hold performance. */

/* Multiplier on receiver-reported speed accuracy. 1.0 = as reported. */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_SCALE        1.0f

/* Trust floor: never believe better than 0.1 m/s 1-sigma (R floor 0.01).
 * THE most consequential single value in this file.
 * Too low (0.05): each 5 Hz fix yanks the velocity state -> the 5 Hz
 *   tick/ring in the tilt command (seen in flight; that is why it was raised).
 * Too high (0.3+): velocity rides integrated accel for seconds -> tilt-error
 *   phantom velocity in wind -> slow to-and-fro hunting.
 * 0.1 is tuned WITH delay comp ON (latency-honest innovations). If delay
 * comp is disabled, expect to need ~0.15 again. */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_MIN         0.15f // was 0.05f

/* Standstill deadband on the measurement itself, m/s. 0.0001 = effectively
 * off (kept live so micro-drift is still corrected). */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_VEL_DEADBAND      0.0f

/* UNUSED IN EFFECT: additive base is dominated by the SACC_MIN floor
 * (0.001 << 0.01). Kept for formula symmetry; do not tune. */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_BASE           0.001f

/* Ceiling 10 (m/s)^2 - numerical-stability cap during GNSS degradation. */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_MAX            10.0f

/* =========================================================================
 * Group 6: Dynamic Sensor Variance Scaling - GNSS Vertical (Z)
 * =========================================================================
 * PHILOSOPHY: GNSS-Z is deliberately (near-)MUTED. Altitude truth = baro
 * (+ rangefinder low). These values keep GNSS-Z mathematically present so
 * it can catch gross baro failure, without letting its multipath jumps
 * warp altitude. Not a tuning area - change philosophy, not numbers. */

/* R_pos = 7000 + 500*vAcc^2, capped 70000; sigma ~84 m at base. Muted. */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VACC_SCALE        500.00f
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VACC_MIN          0.5f
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_BASE           7000.0f
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MUTED          700000.0f
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MAX            70000.0f

/* Z velocity: R_vel = 100 + 0.5*sAcc^2, capped 1000; sigma >= 10 m/s.
 * Effectively muted ("Z vel is very twitchy" - u-blox vertical velocity is
 * far noisier than horizontal). Climb rate comes from baro-derivative+accel. */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_SCALE        1.0f
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_MIN          0.1f // was 0.05f
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VEL_DEADBAND      0.0001f
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_BASE           3000.0f //Z vel is very twicthy
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_MAX            30000.0f

/* Used when no valid nav fix: fully detached. */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_MUTED          10000.0f

/* =========================================================================
 * Group 7: Terrain Rangefinder & Baro/Venturi (Z)
 * ========================================================================= */

/* Rangefinder: R scales from BASE (1 sigma = 10 cm) to MAX (1 m) on the
 * worse of distance-fraction and (1 - quality). MUTED when invalid. */
#define POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_BASE        0.01f
#define POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MAX         10.0f
#define POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MUTED       80000.0f

/* Venturi bias pseudo-measurement (models dynamic-pressure baro suction
 * when translating). Trusted when smooth (R 0.1), discounted 20x when
 * motionScale saturates. If altitude dips return ONLY while translating
 * fast, tune the VenturiBiasEstimator model first, these R's second. */
#define POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_BASE        0.1f
#define POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_MAX         2.0f

/* ---- Dynamic baro R:  R = RP_MIN + GAIN*residual^2/(Pzz+RP_MIN+eps),
 *      then blended toward RP_MAX by motionScale, then LPF'd by ALPHA ---- */

/* Residual self-inflation gain. At 0.005 with clamp 0.75 the residual term
 * maxes at ~0.0001 - i.e. NEGLIGIBLE against RP_MIN 30. Effectively off in
 * the current baro-skeptical tune; becomes live only if RP_MIN is ever
 * lowered drastically (it was load-bearing in the RP_MIN 0.3 experiment). */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_GAIN           200.0f//0.005f

/* LPF on dynamic R (per baro sample). 0.2 -> settles in ~5 samples. */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_ALPHA          0.20f

/* Baro trust floor, m^2 (sigma ~5.5 m per-sample... but fused at ~50 Hz the
 * EFFECTIVE altitude tracking time constant vs the huge Z_Q is ~0.3 s).
 * THE Z-axis master knob, paired with Z_Q_POS/VEL above:
 * Lower -> altitude glued to baro (crisper, but prop-wash/venturi artifacts
 *   print into altitude: the historical dips-and-balloon failure at 0.3).
 * Raise -> smoother, more accel-reliant (needs clean Z accel).
 * History: 0.3 (dips!) -> 30 (validated). */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MIN            4000.0f
/* Ceiling reached under high motionScale - lets rangefinder dominate and
 * shrugs off maneuver-induced pressure noise. */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MAX            10000.0f
/* Numerical guards - never tune. */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_EPS            0.000001f
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_SCALE_EPS      0.001f

/* Residual clamp, m, feeding the (currently negligible) residual term. */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RESIDUAL_CLAMP    0.75f

/* motionScale thresholds, m/s^2 of linear accel: baro R starts inflating
 * toward RP_MAX as maneuvers exceed these. 6/8 = moderate maneuvers already
 * discount baro (tuned low deliberately - baro lies most when moving).
 * Symptom if too low: altitude goes accel-only during gentle cruising.
 * Symptom if too high: throttle punches / fast cruise wobble the altitude. */
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_XY_THRESH          3.0f//6.0f     // was 24.0f
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_Z_THRESH           8.0f     // was 28.0f

/* =========================================================================
 * Group 8: GNSS Delay Compensation (XY only)
 * ========================================================================= */
/* [1] Fuse XY GNSS against the state at measurement time | [0] baseline.
 * Mechanism lives in the estimator (always compiled); this flag is pure
 * GNSS fusion policy in PositionEstimatorHelper. Flag 0 = exact baseline. */
#define POS_ESTIMATOR_GNSS_DELAY_ENABLED                1

/* End-to-end GNSS latency: receiver solution time + half nav period + UART
 * + processing. THIS IS A MEASUREMENT, NOT A KNOB - set by swing test /
 * flight bracketing (0.22 measured for this receiver chain). If the GNSS
 * module, nav rate, or baud ever changes, RE-MEASURE. Symptoms of error:
 * too small -> residual backtrack after stick release (settles behind);
 * too large -> creeps FORWARD past the release point. */
#define POS_ESTIMATOR_GNSS_LATENCY_S                 0.12f //10Hz GPS update

// PositionEstimatorConfig.h — Group 9
#define POS_ESTIMATOR_Z_CRUISE_ADAPT_ENABLED    1
#define POS_ESTIMATOR_Z_CRUISE_SPEED_LO        2.0f   // m/s — below this: pure hover trust profile
#define POS_ESTIMATOR_Z_CRUISE_SPEED_HI        4.0f   // m/s — above this: full cruise profile
#define POS_ESTIMATOR_Z_CRUISE_TAU_RISE        0.3f   // s — engage quickly as translation starts
#define POS_ESTIMATOR_Z_CRUISE_TAU_FALL        1.0f   // s — release slowly: the pressure field
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_BASE_CRUISE      1.0f   // (m/s)^2 — GNSS-Z vel trust during cruise (σ 1 m/s)

#endif
