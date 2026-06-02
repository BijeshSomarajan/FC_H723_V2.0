#ifndef SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_
#define SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_

#include <stdint.h>

/* --- Dimensions --- */
#define POS_EKF_STATE_DIM       9
#define POS_EKF_AXIS_DIM        3  // States per axis: [P, V, B]
#define POS_EKF_SPACE_DIM       3  // Physical axes: [X, Y, Z]

/* --- Axis Indices --- */
#define POS_EKF_X_AXIS          0
#define POS_EKF_Y_AXIS          1
#define POS_EKF_Z_AXIS          2

/* --- State Offsets within an Axis Block --- */
#define POS_EKF_STATE_P         0  // Position index offset
#define POS_EKF_STATE_V         1  // Velocity index offset
#define POS_EKF_STATE_B         2  // Accelerometer Bias index offset

/* ---- Numerical Stability Limits --- */
#define POS_EKF_P_MIN           1e-9f
#define POS_EKF_P_MAX           500.0f

/*=======================================================================================================
 //HORIZONTAL AXIS (X,Y) - GNSS & IMU Fusion
 =======================================================================================================*/
// [+] Faster position updates but jittery | [-] Smoother track but introduces position lag
#define POS_EKF_X_Q_POS   0.0001f
#define POS_EKF_Y_Q_POS   0.0001f
// [+] Trusts raw GNSS velocity more | [-] Trusts IMU short-term (smoother, but drifts if IMU is bad)
#define POS_EKF_X_Q_VEL   0.0065f//0.0015f
#define POS_EKF_Y_Q_VEL   0.0065f//0.0015f
// [+] Fast adaptation to changing sensor bias | [-] Locks bias firmly down but fails to track drift
#define POS_EKF_X_Q_BIAS  0.002f//0.001f
#define POS_EKF_Y_Q_BIAS  0.002f//0.001f
// [+] Ignores GPS noise (smoother, high drift) | [-] Follows GPS precisely (twitchy if GPS has jitter)
#define POS_EKF_X_R_MEAS  0.01f
#define POS_EKF_Y_R_MEAS  0.01f
// [+] Accepts larger GPS glitches | [-] Rejects valid aggressive movements as anomalies (causes freeze)
#define POS_EKF_X_GATE    6.0f
#define POS_EKF_Y_GATE    6.0f
// [+] Patient with bad GNSS drops | [-] Panics and hard-resets EKF quickly during minor glitches
#define POS_EKF_X_PANIC   8
#define POS_EKF_Y_PANIC   8

/* Dynamic Position Scaling */
#define POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_SCALE        1.2f
#define POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_MIN          0.6f
#define POS_ESTIMATOR_DYNAMIC_XY_RP_BASE         POS_EKF_X_R_MEAS
#define POS_ESTIMATOR_DYNAMIC_XY_RP_MAX          POS_EKF_X_R_MEAS * 100

/* Dynamic Velocity Scaling */
#define POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_SCALE        0.1f
#define POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_MIN          0.1f
#define POS_ESTIMATOR_DYNAMIC_XY_VEL_DEADBAND          0.0f
#define POS_ESTIMATOR_DYNAMIC_XY_RV_BASE         0.12f
#define POS_ESTIMATOR_DYNAMIC_XY_RV_RESET        0.20f
#define POS_ESTIMATOR_DYNAMIC_XY_RV_MAX          POS_ESTIMATOR_DYNAMIC_XY_RV_BASE * 100

/*=======================================================================================================
 VERTICAL AXIS (Z) - Barometer & IMU Fusion
 =======================================================================================================*/
// [+] Snappier vertical altitude tracking | [-] Smoother but more delayed altitude estimation
#define POS_EKF_Z_Q_POS   0.0000001f// 0.00000005f
// [+] Quicker vertical velocity response (noisy) | [-] Smoother vertical transitions (delayed)
#define POS_EKF_Z_Q_VEL   0.000002f //0.0000005f
// [+] Fast adaptation to weather/pressure drift | [-] Steadier correction but slow to adapt to changes
#define POS_EKF_Z_Q_BIAS  0.0000001f// 0.00000006f
// [+] Trusts baro less (smoother altitude, drifts) | [-] Trusts baro more (sharp hold, twitches with wind)
#define POS_EKF_Z_R_MEAS  0.01f
// [+] Accepts sudden pressure/wind spikes | [-] Rejects rapid vertical shifts (causes climb/drop lag)
#define POS_EKF_Z_GATE    6.0f
// [+] Tolerates prolonged baro anomalies | [-] Aggressively resets EKF at the slightest sensor error
#define POS_EKF_Z_PANIC   8

/* Dynamic scaling */
// [+] De-weights baro aggressively during high acceleration | [-] Sluggish to distrust bad baro data during maneuvers
#define POS_ESTIMATOR_DYNAMIC_Z_RP_GAIN              1.5f
// [+] Faster response to noise spikes but jittery | [-] Smoother noise scaling transitions but introduces lag
#define POS_ESTIMATOR_DYNAMIC_Z_RP_ALPHA      0.15f //0.4f
// [+] Smoother baseline but increases lag when static | [-] Sharper tracking at hover but catches static sensor noise
#define POS_ESTIMATOR_DYNAMIC_Z_RP_MIN          POS_EKF_Z_R_MEAS
// [+] Disregards baro completely during heavy vibes (trusts IMU) | [-] Forces baro reliance even during high vibrations
#define POS_ESTIMATOR_DYNAMIC_Z_RP_MAX          POS_EKF_Z_R_MEAS * 100.0f
// [+] Safer against float underflow | [-] Risk of EKF mathematical collapse/matrix corruption
#define POS_ESTIMATOR_DYNAMIC_Z_RP_EPS               0.000001f
// [+] Safer against division-by-zero | [-] Risk of NaN/Inf code crash if denominator approaches zero
#define POS_ESTIMATOR_DYNAMIC_Z_RP_SCALE_EPS         0.001f
// [+] Massive baro deviations trigger maximum rejection | [-] Limits noise scaling; protects against runaway R values
#define POS_ESTIMATOR_DYNAMIC_Z_RESIDUAL_CLAMP              0.05f
// [+] Requires fast horizontal flight to trigger scaling | [-] Triggers scaling on slight drift (prematurely drops baro trust)
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_XY_THRESH               4.0f
// [+] Requires hard vertical punches to alter baro trust | [-] Drops baro trust during tiny, normal altitude adjustments
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_Z_THRESH                5.5f

/**
 * @brief EKF Structure for 3D Position, Velocity, and Accel Bias estimation.
 */
typedef struct {
	float x[POS_EKF_STATE_DIM];                         // State vector: [px, vx, bx, py, vy, by, pz, vz, bz]
	float P[POS_EKF_STATE_DIM][POS_EKF_STATE_DIM];      // Error Covariance Matrix
	float Q[POS_EKF_STATE_DIM][POS_EKF_STATE_DIM];      // Process Noise Matrix

	float R[POS_EKF_SPACE_DIM];                         // Measurement Noise per axis [X, Y, Z]
	float gateSize[POS_EKF_SPACE_DIM];                  // Mahalanobis distance thresholds
	uint8_t panicLimit[POS_EKF_SPACE_DIM];              // Max consecutive rejections before reset
	uint8_t rejectCount[POS_EKF_SPACE_DIM];             // Current rejection counters

	uint8_t initialized;                                // Filter initialization status

	float innovation[POS_EKF_SPACE_DIM];
	uint8_t axisInitialized[POS_EKF_SPACE_DIM];

	float prevZR;

} POSITION_EKF;

uint8_t positionEKFInit(POSITION_EKF *ekf);
void positionEKFInvalidateAxis(POSITION_EKF *ekf, uint8_t axis);
void positionEKFSetMode(POSITION_EKF *ekf, uint8_t stabilize);
void positionEKFPredict(POSITION_EKF *ekf, float ax, float ay, float az, float dt);
void positionEKFUpdateZMeasure(POSITION_EKF *ekf, float z_meas);
void positionEKFUpdateZMeasureWithBias(POSITION_EKF *ekf, float z_meas, float bias);
void positionEKFUpdateXYMeasure(POSITION_EKF *ekf, float x_meas, float y_meas);
void positionEKFUpdateXYVel(POSITION_EKF *ekf, float xVel, float yVel, float dampingStrength);
void positionEKFUpdateZVel(POSITION_EKF *ekf, float zVel, float dampingStrength);
void positionEKFSetDymamicRP(POSITION_EKF *ekf, uint8_t axis, float rValue);
float getEstimatedZDynamicRP(POSITION_EKF *ekf, float zMeas, float bias, float ax, float ay, float az);
void positionEKFResetXYVel(POSITION_EKF *ekf) ;
float getEstimatedXYDynamicRV(float sAcc);
float getEstimatedXYDynamicRP(float hAcc);


#endif /* SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_ */
