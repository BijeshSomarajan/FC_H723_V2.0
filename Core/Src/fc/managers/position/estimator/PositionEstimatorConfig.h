#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORCONFIG_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORCONFIG_H_

/* =============================================================================
 *  POSITION ESTIMATOR TUNING GUIDE  (read before touching any value)
 * =============================================================================
 *
 *  MENTAL MODEL
 *  ------------
 *  Per axis the EKF combines:
 *    - PREDICTION: earth-frame accelerometer integrated at 1 kHz (fast, drifts)
 *    - MEASUREMENTS: GNSS / baro / rangefinder (slower, absolute references)
 *
 *  Q controls how quickly prediction uncertainty is allowed to grow.
 *  R controls how much measurement uncertainty is assumed.
 *
 *  In general:
 *    - Raise Q or lower R -> measurements correct the state more strongly
 *      (snappier, but more sensor noise/artifacts can enter the state).
 *    - Lower Q or raise R -> prediction has more influence
 *      (smoother, but accel bias / attitude error can persist between fixes).
 *
 *  The practical balance depends on Q, R, covariance, and cross-state coupling;
 *  Q/R is a useful tuning guide, not the only quantity that determines the
 *  final filter response.
 *
 *  UNITS & TIME BASE (IMPORTANT)
 *  -----------------------------
 *  All Q values are added PER PREDICT STEP at 1 kHz (NOT per second and NOT
 *  scaled by dt). Effective continuous rate = value x 1000.
 *
 *    Q_POS  0.001 per step -> 1.0 m^2/s effective
 *    Q_VEL  0.001 per step -> 1.0 (m/s)^2/s effective
 *    Q_BIAS 0.001 per step -> 1.0 (m/s^2)^2/s effective
 *
 *  If the estimator task rate changes, every Q below silently changes meaning.
 *
 *  TODO: multiply process-noise increments by dt in positionEKFPredict(),
 *        then restate these constants as per-second process-noise values.
 *
 *  R values are variances:
 *    - position: m^2
 *    - velocity: (m/s)^2
 *
 *  sigma = sqrt(R).
 *  Example: R = 0.09 -> sigma = 0.3 m.
 *
 *  Gates are Mahalanobis distance SQUARED:
 *    9.0 = 3 sigma
 *    6.0 ~= 2.45 sigma
 *    16.0 = 4 sigma
 *
 *  SYMPTOM -> KNOB (start here when something is wrong)
 *  ----------------------------------------------------
 *  Estimate lags real motion (feels late)
 *      -> raise Q_VEL, or lower the relevant measurement R.
 *
 *  Estimate jitters / follows GNSS noise
 *      -> raise the relevant R floor (HACC/SACC_MIN), or lower Q.
 *
 *  Slow drift-and-snap sawtooth versus raw GNSS
 *      -> check Q_BIAS and measurement authority; a bias state that is too
 *         constrained may retain attitude/accel error for too long.
 *
 *  Altitude dips when translating / balloons at stops
 *      -> check BARO_RP_MIN, dynamic baro R scaling, and the Venturi estimator.
 *
 *  rejectCount climbing during normal flight
 *      -> check the gate, measurement R, and GNSS latency compensation.
 *
 *  5 Hz tick/ring in controls
 *      -> check SACC_MIN and GNSS velocity measurement authority.
 *
 *  RULES OF THUMB
 *  --------------
 *  - Change ONE value per flight when possible. Log innovation[] and
 *    rejectCount[].
 *  - Tune in factors of 2-3; avoid large changes unless intentionally
 *    disabling a function.
 *  - R floors (_MIN) are the everyday measurement-authority knobs.
 *  - Q is more structural; retune it when the sensor set, estimator structure,
 *    or airframe dynamics materially change.
 *  - POS_ESTIMATOR_GNSS_LATENCY_S is a measured system characteristic,
 *    NOT a general tuning knob.
 * ============================================================================= */


/* =========================================================================
 * Group 1: EKF Core Process Noise & Validation Gates (Horizontal Axis - XY)
 * ========================================================================= */

/* Position random walk, m^2 per 1 ms predict step.
 *
 * Q_POS controls growth of position uncertainty itself. It does not directly
 * make the position estimate "faster". With Q_POS = 0.00005, the effective
 * continuous growth rate is 0.05 m^2/s.
 *
 * Keep relatively small: most XY motion uncertainty is represented through
 * velocity and accel-bias states. Raise only if position covariance becomes
 * unrealistically constrained between GNSS updates.
 */
#define POS_EKF_X_Q_POS                    0.00005f         // 0.000003f // 0.00001f
#define POS_EKF_Y_Q_POS                    0.00005f         // 0.000003f // 0.00001f

/* Velocity random walk, (m/s)^2 per 1 ms predict step.
 *
 * Effective continuous rate = 5.0 (m/s)^2/s.
 *
 * This is the main structural Q knob for XY velocity authority. Increasing it
 * allows the filter to admit that accel-integrated velocity may be wrong,
 * increasing the ability of GNSS velocity to steer the estimate.
 *
 * Raise -> faster correction / less lag, but more GNSS velocity noise.
 * Lower -> smoother velocity, but tilt/bias errors persist longer.
 *
 * Tuned together with GNSS SACC_MIN and delay compensation.
 */
#define POS_EKF_X_Q_VEL                     0.005f
#define POS_EKF_Y_Q_VEL                     0.005f

/* Accel-bias random walk, (m/s^2)^2 per 1 ms predict step.
 *
 * Effective continuous rate = 0.01 (m/s^2)^2/s.
 *
 * Allows the EKF bias state to absorb persistent acceleration errors such as
 * gravity leakage from small attitude errors and thermal drift.
 *
 * 1 degree of attitude error produces approximately 0.17 m/s^2 of horizontal
 * gravity leakage.
 *
 * Raise -> bias can adapt faster to persistent error.
 * Lower -> bias is more stable but persistent tilt/bias error takes longer to
 *          be absorbed.
 *
 * The bias is observable primarily through GNSS velocity/position updates.
 */
#define POS_EKF_X_Q_BIAS                    0.00001f        // was 1e-7
#define POS_EKF_Y_Q_BIAS                    0.00001f        // was 1e-7

/* Innovation gate, Mahalanobis distance squared.
 *
 * 9.0 = approximately 3 sigma.
 *
 * Rejects large GNSS innovations such as multipath jumps while allowing honest
 * maneuver innovations. Delay compensation reduces the innovation caused by
 * normal GNSS measurement age.
 */
#define POS_EKF_X_GATE                      9.0f
#define POS_EKF_Y_GATE                      9.0f

/* Consecutive rejected measurements before panic covariance inflation.
 *
 * At 5 Hz GNSS update rate, 8 consecutive rejects correspond to about 1.6 s.
 * Lower -> faster recovery from genuine divergence, but greater risk of
 * reopening the filter during a short multipath burst.
 */
#define POS_EKF_X_PANIC                     8
#define POS_EKF_Y_PANIC                     8


/* =========================================================================
 * Group 2: EKF Core Process Noise & Validation Gates (Vertical Axis - Z)
 * =========================================================================
 *
 * Z is intentionally treated differently from XY.
 *
 * Primary altitude reference:
 *    barometer + low-altitude rangefinder
 *
 * GNSS-Z is deliberately very weak and is not intended to determine normal
 * altitude behavior.
 *
 * The Z Q values and dynamic baro R therefore form one tuning system:
 * changing one changes the prediction/measurement balance of the other.
 */


/* Position random walk, m^2 per 1 ms predict step.
 *
 * Effective continuous rate = 1.0 m^2/s.
 *
 * This controls growth of Z position uncertainty during prediction. It does
 * not directly specify an altitude response time.
 */
#define POS_EKF_Z_Q_POS                    0.01f// 0.001f

/* Vertical-velocity random walk, (m/s)^2 per 1 ms predict step.
 *
 * Effective continuous rate = 1.0 (m/s)^2/s.
 *
 * Allows predicted vertical velocity uncertainty to grow so measurements can
 * correct accumulated acceleration error. Its effect must be considered
 * together with BARO_RP_MIN and the dynamic baro R scaling.
 */
#define POS_EKF_Z_Q_VEL                    0.02f //0.001f

/* Z accel-bias random walk, (m/s^2)^2 per 1 ms predict step.
 *
 * Effective continuous rate = 0.01 (m/s^2)^2/s.
 *
 * Allows the vertical acceleration-bias state to adapt to persistent sensor
 * bias. Keep conservative because an excessively mobile bias can absorb real
 * vertical disturbances.
 */
#define POS_EKF_Z_Q_BIAS                    0.00001f

/* Barometric position-reference bias random walk.
 *
 * Models slow movement of the barometric altitude datum caused by effects such
 * as ambient pressure change. This is intentionally slow and is not intended
 * to absorb short maneuver-induced pressure artifacts.
 */
#define POS_EKF_Z_Q_POS_BIAS                0.00001f

/* Innovation gate, Mahalanobis distance squared.
 *
 * 9.0 = approximately 3 sigma.
 */
#define POS_EKF_Z_GATE                      9.0f

/* Consecutive rejected Z measurements before panic covariance inflation.
 *
 * Baro updates much faster than GNSS, so 25 rejects represents a relatively
 * short period of Z measurement loss. This intentionally recovers faster than
 * the XY panic threshold.
 */
#define POS_EKF_Z_PANIC                     25


/* =========================================================================
 * Group 3: Adaptive Q Tuning Engine (Structural Strain Scaling)
 * =========================================================================
 *
 * Q is inflated when earth-frame linear acceleration indicates a sufficiently
 * strong maneuver or structural disturbance.
 *
 * stress = accel / threshold, clamped to 0..1
 *
 * Q_eff = Q * (1 + stress * gain ...)
 *
 * Thresholds are linear acceleration after gravity removal, in m/s^2.
 */


/* Enable dynamic Q scaling.
 *
 * When enabled, sufficiently strong linear acceleration temporarily increases
 * prediction uncertainty so the filter is less likely to become overconfident
 * during violent maneuvers or vibration bursts.
 */
#define POS_EKF_DYNAMIC_Q_ENABLED           1

/* XY linear-acceleration threshold, m/s^2.
 *
 * Dynamic Q scaling begins when XY linear acceleration reaches this threshold.
 * The current value of 15 m/s^2 is deliberately much lower than the previous
 * 45 m/s^2 setting, allowing significant maneuver-induced uncertainty to be
 * acknowledged before the estimator becomes overconfident.
 */
#define POS_EKF_ACC_THRESH_XY               15.0f            // 45.0f

/* Z linear-acceleration threshold, m/s^2.
 *
 * High threshold intended primarily for strong vertical disturbances such as
 * hard landings or severe prop/airframe events.
 */
#define POS_EKF_ACC_THRESH_Z                60.0f

/* Maximum total Q inflation factor.
 *
 * Safety ceiling on dynamic process-noise scaling. This is a protection bound,
 * not an everyday tuning knob.
 */
#define POS_EKF_Q_MAX_SCALE                 15.0f

/* Per-state stress gains.
 *
 * Velocity receives additional process-noise inflation because it is the state
 * most directly affected by maneuver-induced acceleration uncertainty.
 *
 * Position receives the nominal scaling.
 * Bias receives no dynamic stress injection: short structural transients should
 * not be learned as persistent sensor bias.
 */
#define POS_EKF_Q_POS_STRESS_GAIN           1.0f
#define POS_EKF_Q_VEL_STRESS_GAIN           2.5f
#define POS_EKF_Q_BIAS_STRESS_GAIN          0.0f

/* Covariance multiplier applied during panic recovery.
 *
 * A 10x inflation rapidly reopens the measurement gate when the state has
 * clearly diverged.
 */
#define POS_EKF_PANIC_P_INFLATE             10.0f


/* =========================================================================
 * Group 4: Dynamic Sensor Variance Scaling - GNSS Horizontal (XY) Position
 * =========================================================================
 *
 * R = HACC_SCALE * max(hAcc, HACC_MIN)^2
 *
 * R is capped at RP_MAX.
 *
 * hAcc is the receiver-reported horizontal 1-sigma accuracy.
 */


/* Multiplier on receiver-reported horizontal accuracy.
 *
 * 1.0 = use the receiver's reported accuracy without additional scaling.
 * Increase if the receiver is consistently optimistic in the local RF
 * environment.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_HACC_SCALE        1.0f

/* Minimum accepted horizontal 1-sigma accuracy, m.
 *
 * R cannot become smaller than approximately 0.09 m^2.
 *
 * This is the everyday XY-position measurement-authority knob:
 * lower -> tighter GNSS position tracking but more GNSS-noise following.
 * higher -> smoother GNSS contribution but more reliance on prediction.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_HACC_MIN          0.3f // was 0.3f

/* Maximum GNSS horizontal-position variance, m^2.
 *
 * sqrt(16) = 4 m 1-sigma.
 *
 * Poor GNSS accuracy is therefore strongly de-weighted but not mathematically
 * removed. Genuine outliers are handled separately by innovation gating.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RP_MAX            16.0f


/* =========================================================================
 * Group 5: Dynamic Sensor Variance Scaling - GNSS Horizontal (XY) Velocity
 * =========================================================================
 *
 * R = SACC_SCALE * max(sAcc, SACC_MIN)^2
 *
 * R is capped at RV_MAX.
 *
 * GNSS velocity is the main external anchor for XY velocity and therefore also
 * makes the XY acceleration-bias states observable.
 */


/* Multiplier on receiver-reported speed accuracy.
 *
 * 1.0 = use the receiver's reported sAcc directly.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_SCALE        1.0f

/* Minimum accepted GNSS horizontal-velocity 1-sigma accuracy, m/s.
 *
 * R cannot become smaller than approximately 0.0225 (m/s)^2.
 *
 * This is a highly consequential tuning value because GNSS velocity updates
 * occur at the navigation rate and directly influence the XY velocity state.
 *
 * Lower -> stronger per-fix correction and potentially visible GNSS-rate
 *          stepping/ringing.
 * Higher -> smoother velocity, but accel/tilt error persists longer.
 *
 * The current 0.15 m/s setting was chosen after the lower 0.05 m/s setting
 * produced excessive GNSS-rate structure in the control response.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_MIN         0.15f // was 0.05f

/* Velocity measurement deadband, m/s.
 *
 * 0.0 = disabled. Small GNSS velocity changes are therefore still available
 * to correct accumulated estimator drift.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_VEL_DEADBAND      0.0f

/* Additive base variance term.
 *
 * With the current SACC_MIN floor, this term is small compared with the
 * minimum SACC-derived variance and therefore has little practical influence.
 * Kept for formula symmetry and future tuning.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_BASE           0.001f

/* Maximum GNSS horizontal-velocity variance, (m/s)^2.
 *
 * sqrt(10) ~= 3.16 m/s 1-sigma.
 *
 * This is a degradation ceiling: even when the receiver reports very poor
 * velocity accuracy, the measurement is prevented from becoming arbitrarily
 * noisy. At this ceiling GNSS velocity has very weak, but non-zero, influence.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_MAX            10.0f


/* =========================================================================
 * Group 6: Dynamic Sensor Variance Scaling - GNSS Vertical (Z)
 * =========================================================================
 *
 * PHILOSOPHY:
 *
 * GNSS-Z is deliberately near-muted.
 *
 * Normal altitude truth comes from:
 *    - barometer
 *    - rangefinder when valid and appropriate
 *
 * GNSS-Z remains mathematically present so it can provide weak protection
 * against gross barometric failure without allowing normal GNSS-Z noise or
 * multipath to drive altitude.
 */


/* GNSS vertical-position variance scaling from receiver vAcc.
 *
 * vAcc is a reported 1-sigma vertical accuracy. The large scale/base values
 * deliberately make GNSS-Z very weak during normal operation.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VACC_SCALE        500.00f

/* Minimum receiver-reported vertical accuracy used by the dynamic R formula.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VACC_MIN          0.5f

/* Base GNSS-Z position variance.
 *
 * sqrt(7000) ~= 83.7 m 1-sigma.
 *
 * This is intentionally extremely weak for normal altitude estimation.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_BASE           7000.0f

/* Very large GNSS-Z position variance used when the measurement is intentionally
 * muted.
 *
 * sqrt(700000) ~= 837 m 1-sigma.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MUTED          700000.0f

/* Maximum dynamically calculated GNSS-Z position variance.
 *
 * sqrt(70000) ~= 265 m 1-sigma.
 *
 * The cap prevents the dynamic variance calculation from growing without
 * bound while still leaving GNSS-Z extremely weak.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MAX            70000.0f

/* GNSS vertical-velocity accuracy scaling.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_SCALE        1.0f

/* Minimum reported vertical-velocity accuracy, m/s.
 *
 * This prevents an unrealistically optimistic receiver sAcc value from making
 * GNSS-Z velocity overly authoritative.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_MIN          0.1f // was 0.05f

/* GNSS-Z velocity deadband, m/s.
 *
 * Small changes are effectively ignored only if they fall inside this value.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VEL_DEADBAND      0.0001f

/* Base GNSS-Z velocity variance.
 *
 * sqrt(3000) ~= 54.8 m/s 1-sigma.
 *
 * Deliberately extremely weak: GNSS vertical velocity is considerably less
 * useful for this estimator than horizontal GNSS velocity.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_BASE            3000.0f // Z vel is very twitchy

/* Maximum dynamically calculated GNSS-Z velocity variance.
 *
 * sqrt(30000) ~= 173.2 m/s 1-sigma.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_MAX            30000.0f

/* GNSS-Z velocity variance used when no valid navigation fix is available.
 *
 * This strongly de-weights the measurement rather than treating it as a
 * normal GNSS-Z velocity observation.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_MUTED          10000.0f


/* =========================================================================
 * Group 7: Terrain Rangefinder & Baro/Venturi (Z)
 * ========================================================================= */

/* Rangefinder position variance.
 *
 * BASE = 0.01 m^2 -> 0.1 m 1-sigma.
 * MAX  = 10.0 m^2 -> 3.16 m 1-sigma.
 *
 * Dynamic scaling de-weights the rangefinder as distance/quality degrades.
 * MUTED is used when the measurement is invalid or intentionally unavailable.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_BASE        0.01f
#define POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MAX         10.0f
#define POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MUTED       90000.0f


/* ---- Dynamic baro R ------------------------------------------------------
 *
 * R starts from RP_MIN, receives a residual-dependent term, is blended toward
 * RP_MAX according to motionScale, and is then low-pass filtered.
 *
 * Exact behavior therefore depends on both the residual and motion-scaling
 * implementation in the estimator.
 */


/* Residual self-inflation gain.
 *
 * The current RP_MIN is deliberately large, so the residual term is not the
 * primary mechanism controlling baro authority. Its importance increases if
 * RP_MIN is later reduced substantially.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_GAIN           500.0f // 0.005f

/* Low-pass coefficient applied to the dynamic baro variance.
 *
 * This is a per-baro-update smoothing coefficient, not a time constant by
 * itself. The exact response time depends on the baro update rate.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_ALPHA          0.20f

/* Minimum barometric position variance, m^2.
 *
 * sqrt(4000) ~= 63.2 m 1-sigma.
 *
 * This deliberately makes individual baro measurements very low-authority.
 * The purpose is to prevent propwash/Venturi pressure artifacts from being
 * printed directly into the altitude state.
 *
 * This is the main baro trust-floor knob and must be considered together with
 * the Z Q values above.
 *
 * Lower -> stronger baro tracking, but more pressure-artifact coupling.
 * Higher -> smoother/more inertial altitude estimate, but greater reliance on
 *           clean acceleration and bias estimation.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MIN            4000.0f

/* Maximum dynamic barometric position variance, m^2.
 *
 * sqrt(10000) = 100 m 1-sigma.
 *
 * Reached as motionScale increases, further reducing baro authority during
 * strong maneuver-induced pressure disturbances.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MAX            10000.0f

/* Numerical guards. Do not tune.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_EPS            0.000001f
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_SCALE_EPS      0.001f

/* Maximum residual used by the dynamic baro-R calculation, m.
 *
 * Limits the influence of a large baro innovation on the residual-dependent
 * variance term.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RESIDUAL_CLAMP    0.75f

/* Linear-acceleration thresholds for baro motion scaling, m/s^2.
 *
 * As maneuver acceleration exceeds these thresholds, baro R is progressively
 * increased toward RP_MAX.
 *
 * The XY threshold is intentionally low because lateral translation is where
 * pressure-field artifacts can become significant. The Z threshold is higher
 * because vertical acceleration is treated separately.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_XY_THRESH          3.0f // 6.0f // was 24.0f
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_Z_THRESH           8.0f // was 28.0f


/* =========================================================================
 * Group 8: GNSS Delay Compensation (XY only)
 * ========================================================================= */

/* [1] Fuse XY GNSS against the estimator state corresponding to measurement
 * time.
 *
 * This compensates the known end-to-end GNSS measurement delay and prevents
 * normal motion from appearing as an innovation simply because the GNSS fix
 * describes an earlier state.
 *
 * The mechanism is implemented in the estimator; this flag selects whether
 * the delay-compensated fusion policy is enabled.
 */
#define POS_ESTIMATOR_GNSS_DELAY_ENABLED                1

/* Current configured end-to-end GNSS latency, seconds.
 *
 * 0.05 s = 50 ms.
 *
 * This should represent the measured/validated system latency including the
 * receiver solution timing, navigation update timing, UART transport and
 * processing path as applicable to this implementation.
 *
 * This is a MEASUREMENT, not a normal tuning knob.
 *
 * Re-measure if the GNSS navigation rate, receiver configuration, UART baud
 * rate, message handling, or estimator processing path changes.
 */
#define POS_ESTIMATOR_GNSS_LATENCY_S                   0.05f


/* =========================================================================
 * Group 9: Cruise-Adaptive Z Estimator Profile
 * ========================================================================= */

/* Enable transition between hover and cruise Z estimator profiles based on
 * horizontal speed.
 */
#define POS_ESTIMATOR_Z_CRUISE_ADAPT_ENABLED           1

/* Below this horizontal speed, use the hover-side Z profile.
 */
#define POS_ESTIMATOR_Z_CRUISE_SPEED_LO                2.0f   // m/s

/* Above this horizontal speed, use the full cruise-side Z profile.
 */
#define POS_ESTIMATOR_Z_CRUISE_SPEED_HI                4.0f   // m/s

/* Time constant for entering the cruise profile.
 *
 * Faster transition as sustained translation begins.
 */
#define POS_ESTIMATOR_Z_CRUISE_TAU_RISE                0.3f   // s

/* Time constant for returning toward the hover profile.
 *
 * Deliberately slower release avoids an abrupt change in Z measurement
 * authority immediately after braking or leveling.
 */
#define POS_ESTIMATOR_Z_CRUISE_TAU_FALL                1.0f   // s

/* GNSS-Z velocity variance used by the cruise profile.
 *
 * R = 1.0 (m/s)^2 -> sigma = 1.0 m/s.
 *
 * This is intentionally much more authoritative than the normal GNSS-Z
 * velocity variance and is therefore a deliberate cruise exception.
 *
 * The value should be monitored for transition-induced velocity steps or
 * ringing, since GNSS-Z velocity is otherwise intentionally near-muted.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_BASE_CRUISE    1.0f   // (m/s)^2

#endif

