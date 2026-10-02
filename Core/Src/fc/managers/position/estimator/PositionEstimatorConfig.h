#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORCONFIG_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORCONFIG_H_

/* =============================================================================
 * POSITION ESTIMATOR TUNING GUIDE
 * =============================================================================
 *
 * Mental model
 * ------------
 * Each EKF axis combines:
 *
 *   Prediction:
 *     Earth-frame acceleration integrated at 1 kHz.
 *     Fast response, but subject to acceleration/attitude error and drift.
 *
 *   Measurements:
 *     GNSS / barometer / rangefinder.
 *     Slower absolute references used to correct the prediction.
 *
 * Q controls how quickly prediction uncertainty grows.
 * R controls how much measurement uncertainty is assumed.
 *
 * In general:
 *
 *   Higher Q / lower R
 *     -> measurements have more authority.
 *     -> faster correction, but more sensor noise/artifacts can enter.
 *
 *   Lower Q / higher R
 *     -> prediction has more authority.
 *     -> smoother response, but accel-bias/attitude errors can persist longer.
 *
 * Q/R is only a tuning guide. The final response also depends on covariance,
 * cross-state coupling, measurement rate, and the estimator implementation.
 *
 *
 * Q UNITS AND TIME BASE
 * ---------------------
 * Q values are currently added once per prediction step at 1 kHz.
 * They are NOT multiplied by dt.
 *
 * Therefore, the effective continuous process-noise rate is:
 *
 *     Q_effective = Q_value * 1000
 *
 * Example:
 *
 *     Q_POS  = 0.001 per step -> 1.0 m^2/s
 *     Q_VEL  = 0.001 per step -> 1.0 (m/s)^2/s
 *     Q_BIAS = 0.001 per step -> 1.0 (m/s^2)^2/s
 *
 * If the estimator task rate changes, the effective meaning of every Q value
 * changes as well.
 *
 * TODO:
 *   Multiply process-noise increments by dt in positionEKFPredict(), then
 *   restate these constants as per-second process-noise values.
 *
 *
 * R UNITS
 * -------
 * R values are variances:
 *
 *   Position : m^2
 *   Velocity : (m/s)^2
 *
 * Measurement 1-sigma uncertainty is:
 *
 *     sigma = sqrt(R)
 *
 * Example:
 *
 *     R = 0.09 -> sigma = 0.3 m
 *
 *
 * INNOVATION GATES
 * ----------------
 * Gates are squared Mahalanobis distance:
 *
 *     6.0  ~= 2.45 sigma
 *     9.0  = 3 sigma
 *     16.0 = 4 sigma
 *
 *
 * SYMPTOM -> KNOB
 * ---------------
 *
 * Estimate lags real motion:
 *   -> Raise Q_VEL, or lower the relevant measurement R.
 *
 * Estimate jitters / follows GNSS noise:
 *   -> Raise the relevant R floor (HACC_MIN / SACC_MIN), or lower Q.
 *
 * Slow drift-and-snap sawtooth versus raw GNSS:
 *   -> Check Q_BIAS and measurement authority.
 *
 * Altitude dips during translation / balloons when stopping:
 *   -> Check baro R, dynamic baro-R scaling, and the Venturi estimator.
 *
 * Reject count rises during normal flight:
 *   -> Check the innovation gate, measurement R, and GNSS latency compensation.
 *
 * 5 Hz structure / ringing in controls:
 *   -> Check SACC_MIN and GNSS velocity measurement authority.
 *
 *
 * RULES OF THUMB
 * --------------
 *
 * - Change one value per flight when possible. Log innovation[] and
 *   rejectCount[].
 *
 * - Tune in factors of 2-3. Avoid large changes unless intentionally disabling
 *   a function.
 *
 * - R floors (_MIN) are the everyday measurement-authority knobs.
 *
 * - Q is more structural. Retune it when the sensor set, estimator structure,
 *   or airframe dynamics materially change.
 *
 * - POS_ESTIMATOR_GNSS_LATENCY_S is a measured system characteristic, not a
 *   general tuning knob.
 *
 * =============================================================================
 */

/* ============================================================================
 * Group 1: EKF Core Process Noise & Validation Gates - Horizontal (XY)
 * ========================================================================== */

/* Position process noise, m^2 per 1 ms prediction step.
 *
 * Controls growth of XY position uncertainty during prediction.
 * Keep relatively small; most horizontal uncertainty is represented through
 * velocity and acceleration-bias states.
 */
#define POS_EKF_X_Q_POS                    0.00005f * 1.2f
#define POS_EKF_Y_Q_POS                    0.00005f * 1.2f

/* Velocity process noise, (m/s)^2 per 1 ms prediction step.
 *
 * Main structural Q knob for XY velocity authority.
 *
 * Higher -> GNSS velocity can correct the estimate more aggressively.
 * Lower  -> smoother velocity, but accel/tilt errors persist longer.
 *
 * Tune together with GNSS SACC_MIN and GNSS delay compensation.
 */
#define POS_EKF_X_Q_VEL                    0.005f * 1.2f
#define POS_EKF_Y_Q_VEL                    0.005f * 1.2f

/* Acceleration-bias process noise, (m/s^2)^2 per 1 ms prediction step.
 *
 * Allows the EKF bias state to absorb persistent acceleration errors such as
 * gravity leakage from small attitude errors and thermal drift.
 *
 * Higher -> faster bias adaptation.
 * Lower  -> more stable bias, but slower absorption of persistent error.
 *
 * The bias is observable primarily through GNSS velocity/position updates.
 */
#define POS_EKF_X_Q_BIAS                   0.00001f
#define POS_EKF_Y_Q_BIAS                   0.00001f

/* Innovation gate, squared Mahalanobis distance.
 *
 * 9.0 corresponds to approximately 3 sigma.
 */
#define POS_EKF_X_GATE                     9.0f
#define POS_EKF_Y_GATE                     9.0f

/* Consecutive rejected measurements before panic covariance inflation.
 *
 * At 5 Hz GNSS update rate, 8 rejects correspond to approximately 1.6 s.
 */
#define POS_EKF_X_PANIC                    8
#define POS_EKF_Y_PANIC                    8

/* ============================================================================
 * Group 2: EKF Core Process Noise & Validation Gates - Vertical (Z)
 * ========================================================================== */

/* Z is intentionally tuned differently from XY.
 *
 * Primary altitude reference:
 *   - Barometer
 *   - Rangefinder when valid and appropriate
 *
 * GNSS-Z is deliberately weak and is not intended to determine normal
 * altitude behavior.
 *
 * The Z Q values and dynamic baro-R settings form one tuning system:
 * changing either changes the prediction/measurement balance.
 */

/* Z position process noise, m^2 per 1 ms prediction step.
 *
 * Controls growth of vertical position uncertainty during prediction.
 * It does not directly define an altitude response time.
 */
#define POS_EKF_Z_Q_POS                    0.0005f * 0.1f

/* Z velocity process noise, (m/s)^2 per 1 ms prediction step.
 *
 * Allows vertical-velocity uncertainty to grow so measurements can correct
 * accumulated acceleration error.
 */
#define POS_EKF_Z_Q_VEL                    0.0005f * 0.1f

/* Z acceleration-bias process noise, (m/s^2)^2 per 1 ms prediction step.
 *
 * Allows the vertical acceleration-bias state to adapt to persistent sensor
 * bias. Keep conservative so real vertical disturbances are not learned as
 * persistent bias.
 */
#define POS_EKF_Z_Q_BIAS                   0.00001f

/* Barometric position-datum bias process noise.
 *
 * Models slow movement of the barometric altitude datum, such as ambient
 * pressure changes. It is intentionally slow and should not absorb short,
 * maneuver-induced pressure artifacts.
 */
#define POS_EKF_Z_Q_POS_BIAS               0.00001f

/* Innovation gate, squared Mahalanobis distance.
 *
 * 9.0 corresponds to approximately 3 sigma.
 */
#define POS_EKF_Z_GATE                     9.0f

/* Consecutive rejected Z measurements before panic covariance inflation.
 *
 * Baro updates occur much faster than GNSS updates, so a larger reject count
 * is used here to represent a similar elapsed time.
 */
#define POS_EKF_Z_PANIC                    25

/* ============================================================================
 * Group 3: Adaptive Q Tuning - Structural Stress Scaling
 * ========================================================================== */

/* Dynamic Q scaling increases prediction uncertainty when earth-frame linear
 * acceleration indicates a sufficiently strong maneuver or disturbance.
 *
 * Conceptually:
 *
 *   stress = accel / threshold, clamped to 0..1
 *   Q_eff  = Q * (1 + stress * gain)
 *
 * The thresholds use linear acceleration after gravity removal.
 */
#define POS_EKF_DYNAMIC_Q_ENABLED          1

#if POS_EKF_DYNAMIC_Q_ENABLED == 1

/* XY linear-acceleration threshold, m/s^2.
 *
 * Dynamic Q scaling begins when XY linear acceleration reaches this value.
 */
#define POS_EKF_ACC_THRESH_XY              15.0f

/* Z linear-acceleration threshold, m/s^2.
 *
 * Higher threshold intended mainly for strong vertical disturbances.
 */
#define POS_EKF_ACC_THRESH_Z               60.0f

/* Maximum total Q inflation factor.
 *
 * Safety ceiling for dynamic process-noise scaling; not an everyday tuning
 * knob.
 */
#define POS_EKF_Q_MAX_SCALE                15.0f

/* Per-state stress gains.
 *
 * Velocity receives additional inflation because it is directly affected by
 * maneuver-induced acceleration uncertainty.
 *
 * Position receives nominal scaling.
 *
 * Bias receives no dynamic stress injection so short structural transients
 * are not learned as persistent sensor bias.
 */
#define POS_EKF_Q_POS_STRESS_GAIN          1.0f
#define POS_EKF_Q_VEL_STRESS_GAIN          2.5f
#define POS_EKF_Q_BIAS_STRESS_GAIN         0.0f

#endif

/* Covariance multiplier applied during panic recovery.
 *
 * A 10x inflation rapidly reopens the measurement gate after clear divergence.
 */
#define POS_EKF_PANIC_P_INFLATE            10.0f

/* ============================================================================
 * Group 4: Dynamic GNSS Variance - Horizontal Position (XY)
 * ========================================================================== */

/* Dynamic position variance:
 *
 *   R = HACC_SCALE * max(hAcc, HACC_MIN)^2
 *
 * R is capped at RP_MAX.
 * hAcc is the receiver-reported horizontal 1-sigma accuracy.
 */

/* Multiplier on receiver-reported horizontal accuracy.
 *
 * 1.0 = use the receiver's reported accuracy directly.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_HACC_SCALE       1.0f

/* Minimum accepted horizontal 1-sigma accuracy, m.
 *
 * This is the everyday XY-position measurement-authority knob.
 *
 * Lower -> tighter GNSS tracking, but more GNSS-noise following.
 * Higher -> smoother GNSS contribution, but more reliance on prediction.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_HACC_MIN         0.3f

/* Maximum GNSS horizontal-position variance, m^2.
 *
 * sqrt(16) = 4 m 1-sigma.
 *
 * Poor reported accuracy is therefore strongly de-weighted but not removed.
 * Genuine outliers are handled separately by innovation gating.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RP_MAX           16.0f

/* ============================================================================
 * Group 5: Dynamic GNSS Variance - Horizontal Velocity (XY)
 * ========================================================================== */

/* Dynamic velocity variance:
 *
 *   R = SACC_SCALE * max(sAcc, SACC_MIN)^2
 *
 * R is capped at RV_MAX.
 *
 * GNSS velocity is the main external anchor for XY velocity and also makes
 * the XY acceleration-bias states observable.
 */

/* Multiplier on receiver-reported speed accuracy.
 *
 * 1.0 = use the receiver's reported sAcc directly.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_SCALE       100.0f

/* Minimum GNSS horizontal-velocity 1-sigma accuracy, m/s.
 *
 * This is a high-impact tuning value because GNSS velocity directly influences
 * the XY velocity state at the navigation update rate.
 *
 * Lower -> stronger per-fix correction and potentially visible GNSS-rate
 *           stepping/ringing.
 * Higher -> smoother velocity, but accel/tilt error persists longer.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_MIN         0.15f

/* GNSS velocity deadband, m/s.
 *
 * 0.0 = disabled. Small GNSS velocity changes remain available to correct
 * accumulated estimator drift.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_VEL_DEADBAND     0.0f

/* Additive base variance term, (m/s)^2.
 *
 * Small compared with the current SACC_MIN-derived variance.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_BASE          0.001f

/* Maximum GNSS horizontal-velocity variance, (m/s)^2.
 *
 * sqrt(10) ~= 3.16 m/s 1-sigma.
 *
 * This is a degradation ceiling; poor receiver-reported accuracy cannot make
 * the measurement arbitrarily noisy.
 */
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_MAX           10.0f

/* ============================================================================
 * Group 6: Dynamic GNSS Variance - Vertical (Z)
 * ========================================================================== */

/* GNSS-Z philosophy:
 *
 * GNSS-Z is deliberately near-muted.
 *
 * Normal altitude estimation relies on:
 *   - Barometer
 *   - Rangefinder when valid and appropriate
 *
 * GNSS-Z remains mathematically present as a weak protection against gross
 * barometric failure, without allowing normal GNSS-Z noise or multipath to
 * drive altitude.
 */

/* Base GNSS-Z position variance, m^2.
 *
 * sqrt(10000) = 100 m 1-sigma.
 *
 * This is intentionally extremely weak for normal altitude estimation.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_BASE           12000.0f

/* Maximum dynamically calculated GNSS-Z position variance, m^2.
 *
 * sqrt(70000) ~= 264.6 m 1-sigma.
 *
 * The cap prevents the dynamic variance from growing without bound.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MAX            120000.0f

/* Base GNSS-Z velocity variance, (m/s)^2.
 *
 * sqrt(2000) ~= 44.7 m/s 1-sigma.
 *
 * Deliberately extremely weak because GNSS vertical velocity is much less
 * useful here than horizontal GNSS velocity.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_BASE           4000.0f

/* Maximum dynamically calculated GNSS-Z velocity variance, (m/s)^2.
 *
 * sqrt(30000) ~= 173.2 m/s 1-sigma.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_MAX            20000.0f

/* GNSS vertical-position accuracy scaling.
 *
 * vAcc is the receiver-reported vertical 1-sigma accuracy.
 * The large scale/base values deliberately keep GNSS-Z weak during normal use.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VACC_SCALE        500.0f

/* Minimum receiver-reported vertical accuracy used by the dynamic R formula. */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VACC_MIN          0.5f

/* GNSS vertical-velocity accuracy scaling. */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_SCALE        500.0f

/* Minimum reported vertical-velocity accuracy, m/s.
 *
 * Prevents an unrealistically optimistic receiver sAcc value from making
 * GNSS-Z velocity overly authoritative.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_MIN          0.1f

/* GNSS-Z velocity deadband, m/s. */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VEL_DEADBAND      0.0001f

/* ============================================================================
 * Group 7: Terrain Rangefinder & Barometer / Venturi (Z)
 * ========================================================================== */

/* Rangefinder position variance.
 *
 * BASE = 0.01 m^2 -> 0.1 m 1-sigma
 * MAX  = 10.0 m^2 -> 3.16 m 1-sigma
 *
 * Dynamic scaling de-weights the rangefinder as distance/quality degrades.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_BASE        0.01f
#define POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MAX         10.0f

/* ---- Dynamic baro R ------------------------------------------------------- *
 *
 * Baro R starts from RP_BASE, receives a residual-dependent term, is blended
 * toward RP_MAX according to motionScale, and is then low-pass filtered.
 *
 * Exact behavior therefore depends on the estimator's residual and motion
 * scaling implementation.
 */

/* Minimum barometric position variance, m^2.
 *
 * sqrt(4000) ~= 63.2 m 1-sigma.
 *
 * This deliberately makes individual baro measurements low-authority so
 * propwash/Venturi pressure artifacts are not transferred directly into the
 * altitude state.
 *
 * Lower -> stronger baro tracking, but more pressure-artifact coupling.
 * Higher -> smoother/more inertial altitude estimate, but greater reliance on
 *           acceleration and bias estimation.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_BASE           5000.0f

/* Maximum dynamic barometric position variance, m^2.
 *
 * sqrt(27000) ~= 164.3 m 1-sigma.
 *
 * Reached as motionScale increases, further reducing baro authority during
 * strong maneuver-induced pressure disturbances.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MAX            27000.0f

/* Residual self-inflation gain.
 *
 * The current RP_BASE is already large, so the residual term is not the
 * primary mechanism controlling baro authority. Its importance increases if
 * RP_BASE is later reduced.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_GAIN           100.0f

/* Low-pass coefficient applied to the dynamic baro variance.
 *
 * This is a per-baro-update smoothing coefficient, not a time constant by
 * itself. The effective response depends on the baro update rate.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_ALPHA          0.20f

/* Numerical guards. Do not tune. */
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
 * As maneuver acceleration increases above these thresholds, baro R is
 * progressively increased toward RP_MAX.
 *
 * XY is intentionally low because lateral translation is where pressure-field
 * artifacts can become significant.
 *
 * Z is higher because vertical acceleration is treated separately.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_XY_THRESH          3.0f
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_Z_THRESH           8.0f

/* ============================================================================
 * Group 8: GNSS Delay Compensation (XY)
 * ========================================================================== */

/* Fuse XY GNSS against the estimator state corresponding to the measurement
 * time rather than blindly against the current state.
 *
 * This compensates known end-to-end GNSS delay and prevents normal motion from
 * appearing as an innovation simply because the fix describes an earlier state.
 */
#define POS_ESTIMATOR_GNSS_DELAY_ENABLED               1

/* Configured end-to-end GNSS latency, seconds.
 *
 * This should represent measured/validated system latency including receiver
 * timing, navigation update timing, UART transport, and processing as applicable.
 *
 * This is a measured system characteristic, not a normal tuning knob.
 * Re-measure if the GNSS rate, receiver configuration, UART baud rate,
 * message handling, or estimator processing path changes.
 */
#define POS_ESTIMATOR_GNSS_LATENCY_S                   0.05f

/* ============================================================================
 * Group 9: Cruise-Adaptive Z Estimator Profile
 * ========================================================================== */

/* Disabled by default. */
#define POSITION_MGR_Z_ENABLE_DYNAMIC_R                0

#if POSITION_MGR_Z_ENABLE_DYNAMIC_R == 1

/* Enable transition between hover and cruise Z estimator profiles based on
 * horizontal speed.
 */
#define POS_ESTIMATOR_Z_CRUISE_ADAPT_ENABLED           0

/* Below this horizontal speed, use the hover-side Z profile. */
#define POS_ESTIMATOR_Z_CRUISE_SPEED_LO                2.0f   // m/s

/* Above this horizontal speed, use the full cruise-side Z profile. */
#define POS_ESTIMATOR_Z_CRUISE_SPEED_HI                10.0f  // m/s

/* Time constant for entering the cruise profile. */
#define POS_ESTIMATOR_Z_CRUISE_TAU_RISE                0.3f   // s

/* Time constant for returning toward the hover profile.
 *
 * Slower release avoids an abrupt change in Z measurement authority after
 * braking or leveling.
 */
#define POS_ESTIMATOR_Z_CRUISE_TAU_FALL                1.0f   // s

/* GNSS-Z velocity variance used by the cruise profile.
 *
 * R = 1.0 (m/s)^2 -> 1.0 m/s 1-sigma.
 *
 * This is intentionally much more authoritative than the normal GNSS-Z
 * velocity variance and is therefore a deliberate cruise exception.
 *
 * Monitor for transition-induced velocity steps or ringing.
 */
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_BASE_CRUISE    1.0f   // (m/s)^2

#endif

#endif
