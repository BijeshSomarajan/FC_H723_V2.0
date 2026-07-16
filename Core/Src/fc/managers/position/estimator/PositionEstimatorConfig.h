#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORCONFIG_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORCONFIG_H_

/* =========================================================================
 * Group 1: EKF Core Process Noise & Validation Gates (Horizontal Axis - XY)
 * ========================================================================= */
// [+] Faster tracking of position changes | [-] Smoother position state but risks random linear wander
#define POS_EKF_X_Q_POS                       0.00001f
#define POS_EKF_Y_Q_POS                       0.00001f

// [+] Eliminates velocity phase lag/snappier stops | [-] Smoother velocity state but risks maneuver overshoots
#define POS_EKF_X_Q_VEL                       0.001f
#define POS_EKF_Y_Q_VEL                       0.001f

// [+] Quicker tracking of thermal accelerometer drift | [-] Firmly locks bias down; prevents chasing high-vibration noise
#define POS_EKF_X_Q_BIAS                      0.00001f  // was 1e-7 (absorbs tilt-induced gravity leakage)
#define POS_EKF_Y_Q_BIAS                      0.00001f  // was 1e-7 (absorbs tilt-induced gravity leakage)

// [+] Accepts aggressive stick maneuvers without rejection | [-] Aggressively rejects multi-path/GPS jumps but risks lockout
#define POS_EKF_X_GATE                         9.0f
#define POS_EKF_Y_GATE                         9.0f

// [+] Rides through extended sensor outages safely before a reset | [-] Fast, safe hard-reset during sensor failure but risks premature panics
#define POS_EKF_X_PANIC                        8
#define POS_EKF_Y_PANIC                        8

/* =========================================================================
 * Group 2: EKF Core Process Noise & Validation Gates (Vertical Axis - Z)
 * ========================================================================= */
// [+] Faster vertical position tracking response | [-] Dampens high-frequency altitude jitter but adds tracking lag
#define POS_EKF_Z_Q_POS                       0.00002f   // was 0.01   (500x down)

// [+] Eliminates vertical velocity lag/prevents punch overshoots | [-] Smoother climb rate but introduces spongey altitude response
#define POS_EKF_Z_Q_VEL                       0.0002f    // was 0.02   (100x down)

// [+] Rapidly compensates for vertical IMU thermal bias shifts | [-] Holds Z-bias firm against high-frequency pressure noise
#define POS_EKF_Z_Q_BIAS                       0.00001f

// [+] Faster tracking of persistent barometric reference/weather shifts | [-] Keeps overall altitude state stable against wind-gust spikes
#define POS_EKF_Z_Q_POS_BIAS                   0.00001f

// [+] Tolerates rapid vertical steps/spikes without gate rejection | [-] Strictly rejects vertical measurement anomalies but risks state freezing
#define POS_EKF_Z_GATE                         9.0f

// [+] Safely rides through long pressure/lidar dropouts without falling out of position hold | [-] Forces rapid filter reset during vertical sensor failure to prevent flyaways
#define POS_EKF_Z_PANIC                        25

/* =========================================================================
 * Group 3: Adaptive Q Tuning Engine (Structural Strain Scaling)
 * ========================================================================= */
// [1] Dynamically inflates process noise matrices under structural stress | [0] Keeps filter stiffness completely constant
#define POS_EKF_DYNAMIC_Q_ENABLED              1

// [+] Requires more violent maneuvers to scale up horizontal process noise | [-] Triggers sensitive Q expansion under minor horizontal vibrations
#define POS_EKF_ACC_THRESH_XY                  45.0f

// [+] Protects vertical filter from scaling up during hard landings | [-] Triggers rapid vertical Q expansion during minor motor oscillations
#define POS_EKF_ACC_THRESH_Z                   60.0f

// [+] Allows massive covariance inflation under extreme structural stress to save filter | [-] Bounds maximum state freedom during extreme vibrations
#define POS_EKF_Q_MAX_SCALE                    15.0f

// [+] Position state matrix loosens up faster during high-G turns | [-] Position confidence remains rigid even during aggressive maneuvering
#define POS_EKF_Q_POS_STRESS_GAIN              1.0f

// [+] Velocity tracking adapts instantly to aggressive maneuvers | [-] Risks velocity state lagging/slipping during sudden high-G braking
#define POS_EKF_Q_VEL_STRESS_GAIN              2.5f

// [+] Accelerometer bias states adapt quickly to physical frame flex | [-] Keeps bias estimation stable against momentary high-G structural impulses
#define POS_EKF_Q_BIAS_STRESS_GAIN             0.0f

#define POS_EKF_PANIC_P_INFLATE   10.0f

/* =========================================================================
 * Group 4: Dynamic Sensor Variance Scaling - GNSS Horizontal (XY) Position
 * ========================================================================= */
// [+] Penalizes poor satellite geometry heavily by inflating variance | [-] Relies strictly on baseline trust regardless of reported accuracy
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_HACC_SCALE        1.0f

// [+] Enforces a strict minimum trust floor to prevent EKF overconfidence | [-] Allows filter to assume perfect sub-centimeter GPS tracking
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_HACC_MIN          0.3f

// [+] Increases baseline measurement noise/smoothes tracking paths | [-] Forces razor-sharp adherence to raw GPS coordinates (ideal for RTK)
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RP_BASE           0.005f

// [+] Allows high-noise packets to be processed with maximum discount | [-] Caps maximum variance penalty, risking noise leakage during heavy multi-path
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RP_MAX            16.0f

/* =========================================================================
 * Group 5: Dynamic Sensor Variance Scaling - GNSS Horizontal (XY) Velocity
 * ========================================================================= */
// [+] Multiplies velocity variance penalties when speed accuracy degrades | [-] Keeps velocity measurement trust highly aggressive despite receiver noise
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_SCALE        1.0f

// [+] Protects velocity channel from over-trusting clean telemetry by capping minimum noise | [-] Trusts raw velocity updates down to near-zero variance
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_MIN          0.15f // was 0.05f

// [+] Completely ignores minor GPS velocity drift at a standstill | [-] Keeps velocity innovations active even for microscopic, noise-driven vectors
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_VEL_DEADBAND      0.0001f

// [+] Broadly dampens GPS velocity authority in horizontal fusion | [-] Sharpens immediate responsiveness to real-world velocity changes
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_BASE           0.001f

// [+] Completely discounts highly corrupted speed updates before gate check | [-] Forces filter to digest moderately noisy speed data at a capped threshold
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_MAX            10.0f

// [+] Forces a highly conservative state variance initialization during re-arm | [-] Instantly resets to low variance, risking filter jumps
#define POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_RESET          POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_MAX

/* =========================================================================
 * Group 6: Dynamic Sensor Variance Scaling - GNSS Vertical (Z) Position & Velocity
 * ========================================================================= */
// [+] Multiplies vertical GPS variance heavily when geometry degrades | [-] Blindly trusts vertical GPS data even during satellite count drops
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VACC_SCALE        500.00f

// [+] Sets a high baseline noise floor to prevent vertical GPS overconfidence | [-] Allows the EKF to treat raw vertical GPS as highly precise
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VACC_MIN          0.5f

// [+] Heavily discounts vertical GPS to prevent multi-path jumping | [-] Drastically increases reliance on vertical GPS over barometer/lidar data
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_BASE           7000.0f

// [+] Completely detaches vertical GPS during major reception anomalies | [-] Limits maximum GPS penalty, letting bad vertical telemetry warp altitude
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MAX            70000.0f

// [+] Forces absolute mathematical isolation of vertical GPS when invalid | [-] Keeps invalid state dangerously close to processing ranges
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MUTED          700000.0f

// [+] Heavily penalizes vertical velocity estimation during poor signal states | [-] Maintains baseline trust in vertical speed packets regardless of noise
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_SCALE        0.5f

// [+] Enforces a minimum trust floor for vertical velocity updates | [-] Allows zero-variance assumption for vertical velocity packets
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_MIN          POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_MIN

// [+] Eliminates minor vertical velocity hunting during a flat hover | [-] Keeps vertical velocity updates fully live down to absolute zero
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_VEL_DEADBAND      POS_ESTIMATOR_DYNAMIC_XY_GNSS_VEL_DEADBAND

// [+] Softens vertical velocity measurement authority globally | [-] Forces hard lock on vertical speed updates, risking jumpy climbs
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_BASE           10.0f

// [+] Safely isolates wild vertical velocity steps from blowing up the matrix | [-] Limits maximum penalty, allowing heavy vertical tracking errors to bleed through
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_MAX            200.0f

// [+] Initializes vertical velocity variance conservatively on boot | [-] Aggressive initial trust that can cause an upward/downward state jump
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_RESET          POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_MAX

// [+] Guarantees vertical speed channel is ignored during blackouts | [-] Risks letting uninitialized velocity vectors corrupt the EKF
#define POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_MUTED          10000.0f

/* =========================================================================
 * Group 7: Dynamic Sensor Variance Scaling - Terrain Rangefinder & Baro/Venturi (Z)
 * ========================================================================= */
// [+] Softens baseline lidar/rangefinder authority over terrain altitude | [-] Sharpens surface altitude tracking accuracy down to millimeters
#define POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_BASE        0.01f

// [+] Increases tolerance window over complex or grassy terrain anomalies | [-] Tightens rangefinder isolation ceiling, risking early sensor dropouts
#define POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MAX         1.0f

// [+] Completely decouples lidar from altitude loop if ground lock breaks | [-] Keeps failed terrain ranges mathematically close to operational bounds
#define POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MUTED       1000.0f

// 1-sigma error of the venturi correction as a fraction of itself.
#define POS_ESTIMATOR_VENTURI_CORR_UNCERTAINTY   0.0f
// Soft zero-anchor for BP (baro-only observability guard)
#define POS_ESTIMATOR_Z_POS_BIAS_ANCHOR_RP    10.0f

//#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_GAIN           0.005f
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_GAIN           2.0f     // was 0.005f
// [+] Dynamic barometer variance tracks immediate pressure noise spikes | [-] Heavily filters dynamic baro variance, adding phase lag to noise detection
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_ALPHA          0.20f
// [+] Safeguards against baro overconfidence in perfect weather conditions | [-] Lets the EKF completely rely on raw baro pressure data down to absolute zero
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MIN            2.0f    // was 0.3 — baro is an anchor, not a tracker  // was 30.0f  <- key change
// [+] Allows baro variance to scale high enough to let rangefinder completely dominate | [-] Caps barometer discount, allowing pressure noise to fight lidar
//#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MAX            100.0f
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MAX            10.0f    // was 100.0f
// [+] Bounds the maximum innovation error allowed to scale up baro variance | [-] Lets massive altitude errors continuously inflate variance exponentially
//#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RESIDUAL_CLAMP    0.05f
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RESIDUAL_CLAMP    0.75f    // was 0.05f
// [+] Prevents floating-point underflow division during baseline delta calculations | [-] Pulls matrix inversion closer to numerical instability boundaries
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_EPS            0.000001f
// [+] Stabilizes dynamic scaling multipliers against sudden infinitesimal changes | [-] Opens up scaling loop to microsecond rounding errors
#define POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_SCALE_EPS      0.001f
// [+] Delays baro variance inflation until high horizontal velocity tilt occurs | [-] Artificially inflates baro variance during gentle horizontal cruising
//#define POS_ESTIMATOR_DYNAMIC_Z_ACC_XY_THRESH          24.0f
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_XY_THRESH          6.0f     // was 24.0f
// [+] Protects baro variance from inflating during fast vertical punch outs | [-] Discards stable barometer data prematurely during rapid vertical climbs
//#define POS_ESTIMATOR_DYNAMIC_Z_ACC_Z_THRESH           28.0f
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_Z_THRESH           8.0f     // was 28.0f

#endif
