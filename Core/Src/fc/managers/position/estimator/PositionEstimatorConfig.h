#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORCONFIG_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORCONFIG_H_

/* =========================================================================
 * Tuning Parameters: Horizontal Axis (X, Y) - GNSS & IMU Fusion
 * ========================================================================= */
// [+] Snappier tracking of trajectory steps | [-] Higher weight on short-term IMU dead-reckoning
#define POS_EKF_X_Q_POS           0.01f
#define POS_EKF_Y_Q_POS           0.01f
// [+] Zero-latency velocity response         | [-] Smoothes velocity state (adds phase lag)
#define POS_EKF_X_Q_VEL           0.4f/100.0f
#define POS_EKF_Y_Q_VEL           0.4f/100.0f
// [+] Fast adaptation to IMU thermal bias    | [-] Locks bias firmly down (slow drift tracking)
#define POS_EKF_X_Q_BIAS          0.001f  //0.001
#define POS_EKF_Y_Q_BIAS          0.001f  //0.001
// [+] Fast adaptation to IMU thermal bias    | [-] Locks bias firmly down (stops chasing GPS jitter)
#define POS_EKF_X_R_MEAS          1.0f/100.0f
#define POS_EKF_Y_R_MEAS          1.0f/100.0f
// [+] Accepts larger GPS steps/glitches      | [-] Rejects valid aggressive maneuvers as outliers
#define POS_EKF_X_GATE            4.0f
#define POS_EKF_Y_GATE            4.0f
// [+] Tolerant of brief GNSS signal drops    | [-] Rapidly panics and resets filter during glitches
#define POS_EKF_X_PANIC           10
#define POS_EKF_Y_PANIC           10

/* =========================================================================
 * Tuning Parameters: Vertical Axis (Z) - Barometer & IMU Fusion
 * ========================================================================= */
// [+] Snappier altitude tracking response    | [-] Smooth but noticeably delayed altitude estimation
#define POS_EKF_Z_Q_POS           0.0001f
// [+] Quicker vertical velocity updates     | [-] Heavy vertical lag (causes massive punch overshoots)
#define POS_EKF_Z_Q_VEL           0.0003f
// [+] Rapid adaptation to weather/baro shift| [-] Solid baseline calculation (slow drift tracking)
#define POS_EKF_Z_Q_BIAS          0.0000001f  //0.001
// [+] Smooth altitude (low trust in baro)   | [-] Razor-sharp hold (twitches in ground effect/wind)
#define POS_EKF_Z_R_MEAS          40.0f
// [+] Tolerates sudden wind/pressure spikes  | [-] Rejects fast vertical maneuvers (climb/descend lag)
#define POS_EKF_Z_GATE            4.0f
// [+] Tolerates longer pressure anomalies     | [-] Aggressively resets filter at minor baro glitches
#define POS_EKF_Z_PANIC           10

//Adaptive Q Tuning
#define POS_EKF_DYNAMIC_Q_ENABLED   1
#define POS_EKF_ACC_THRESH_XY       24.0f
#define POS_EKF_ACC_THRESH_Z        26.0f
#define POS_EKF_Q_MAX_SCALE         15.0f
#define POS_EKF_Q_POS_STRESS_GAIN   1.5f
#define POS_EKF_Q_VEL_STRESS_GAIN   1.5f
#define POS_EKF_Q_BIAS_STRESS_GAIN  0.5f

/* Dynamic Position Scaling XY Axis*/
#define POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_SCALE        1.5f//1.2f
#define POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_MIN          0.3f//0.6f
#define POS_ESTIMATOR_DYNAMIC_XY_RP_BASE               POS_EKF_X_R_MEAS
#define POS_ESTIMATOR_DYNAMIC_XY_RP_MAX                (POS_EKF_X_R_MEAS * 20.0f)
/* Dynamic Velocity Scaling */
#define POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_SCALE        0.35f
#define POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_MIN          0.02f
#define POS_ESTIMATOR_DYNAMIC_XY_VEL_DEADBAND          0.05f
#define POS_ESTIMATOR_DYNAMIC_XY_RV_BASE               0.05f
#define POS_ESTIMATOR_DYNAMIC_XY_RV_RESET              0.5f
#define POS_ESTIMATOR_DYNAMIC_XY_RV_MAX                (POS_ESTIMATOR_DYNAMIC_XY_RV_BASE * 20.0f) // Keep it around 3

/* Dynamic scaling Z */
#define POS_ESTIMATOR_DYNAMIC_Z_RP_GAIN              0.03f
#define POS_ESTIMATOR_DYNAMIC_Z_RP_ALPHA             0.10f
#define POS_ESTIMATOR_DYNAMIC_Z_RP_MIN               POS_EKF_Z_R_MEAS
#define POS_ESTIMATOR_DYNAMIC_Z_RP_MAX               POS_EKF_Z_R_MEAS * 30.0f
#define POS_ESTIMATOR_DYNAMIC_Z_RP_EPS               0.000001f
#define POS_ESTIMATOR_DYNAMIC_Z_RP_SCALE_EPS         0.001f
#define POS_ESTIMATOR_DYNAMIC_Z_RESIDUAL_CLAMP       0.05f
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_XY_THRESH        24.0f
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_Z_THRESH         28.0f


#define TERRAIN_BASE_R             0.01f
#define TERRAIN_MAX_R             500.0f
#define TERRAIN_OBSTACLE_GAIN      20.0f

#endif
