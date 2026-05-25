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

/*=======================================================================================================
 HORIZONTAL AXIS (X / Y) - GPS & IMU Fusion
 =======================================================================================================*/
// State variance for position. Lower values assume the drone's true position cannot teleport between steps.
#define POS_EKF_X_Q_POS   0.006f
// Velocity process noise. Lower = heavily trust IMU integration short-term; Higher = rely on raw GPS velocity steps.
#define POS_EKF_X_Q_VEL   0.15f
// Accelerometer bias tracking speed. Lower = locks bias down during hover; Higher = allows bias to warp rapidly.
#define POS_EKF_X_Q_BIAS  0.001f
// Default GPS measurement noise. (Dynamically overridden at runtime by your GPS Accuracy telemetry).
#define POS_EKF_X_R_MEAS  1.0f
// Anomaly shield. Rejects GPS steps greater than 6 standard deviations from the EKF's predicted position.
#define POS_EKF_X_GATE    6.0f
// Defcon limit. Number of consecutive rejected GPS steps before EKF panics and forces a hard reset to the sensor.
#define POS_EKF_X_PANIC   8

// State variance for position. Lower values assume the drone's true position cannot teleport between steps.
#define POS_EKF_Y_Q_POS   0.006f
// Velocity process noise. Lower = heavily trust IMU integration short-term; Higher = rely on raw GPS velocity steps.
#define POS_EKF_Y_Q_VEL   0.15f
// Accelerometer bias tracking speed. Lower = locks bias down during hover; Higher = allows bias to warp rapidly.
#define POS_EKF_Y_Q_BIAS  0.001f
// Default GPS measurement noise. (Dynamically overridden at runtime by your GPS Accuracy telemetry).
#define POS_EKF_Y_R_MEAS  1.0f
// Anomaly shield. Rejects GPS steps greater than 6 standard deviations from the EKF's predicted position.
#define POS_EKF_Y_GATE    6.0f
// Defcon limit. Number of consecutive rejected GPS steps before EKF panics and forces a hard reset to the sensor.
#define POS_EKF_Y_PANIC   8

/*=======================================================================================================
 VERTICAL AXIS (Z) - Barometer & IMU Fusion
 =======================================================================================================*/
// State variance for altitude. Controls how tightly the EKF clamps the absolute geometric vertical height.
#define POS_EKF_Z_Q_POS   0.0001f
// Vertical velocity process noise. Lower eliminates phase lag for instantaneous PID D-term; Higher dampens response.
#define POS_EKF_Z_Q_VEL   0.001f
// Barometer/Thermal bias tracking speed. Keeps the floor high enough to absorb structural environmental pressure drift.
#define POS_EKF_Z_Q_BIAS  0.0003f
// Barometer hardware noise covariance. High value tells the EKF that baro data is noisy and should be smoothed out.
#define POS_EKF_Z_R_MEAS  4000.0f
// Innovation gate. Rejects massive, sudden pressure spikes (like prop wash or wind gusts hitting the canopy).
#define POS_EKF_Z_GATE    6.0f
// Defcon limit. Number of consecutive rejected BARO reads before EKF panics and forces a hard reset to the sensor.
#define POS_EKF_Z_PANIC   8

/*=======================================================================================================
 Dynamic R configuration for Z-axis based on vertical velocity and acceleration
 =======================================================================================================*/
// Multiplier for how aggressively model-vs-baro disagreement inflates measurement noise covariance
#define POS_Z_DYNAMIC_R_GAIN              1.25f
// IIR low-pass filter smoothing factor (0.0 to 1.0) to prevent step jumps in the covariance matrix
#define POS_Z_DYNAMIC_R_SMOOTH_ALPHA      0.4f
// Absolute lower bound for measurement noise variance; defaults to clean, static sensor baseline
#define POS_Z_DYNAMIC_R_MIN               POS_EKF_Z_R_MEAS
// Absolute upper bound for measurement noise variance; heavily dampens baro reliance during high vibration
#define POS_Z_DYNAMIC_R_MAX               6500.0f
// Small epsilon floor for state variance (Pzz) to prevent float underflow and mathematical collapse
#define POS_Z_DYNAMIC_R_EPS               1e-6f
// Protection factor added to the denominator to strictly prevent division-by-zero errors
#define POS_Z_DYNAMIC_R_SCALE_EPS         1e-3f
// Hard ceiling (in cm) on the tracking error fed into the dynamic noise scaling curve
#define POS_Z_RESIDUAL_CLAMP              5.0f
// Horizontal acceleration limit (m/s²) used to detect transitions into high-speed forward/lateral cruise
#define POS_Z_ACC_XY_THRESH               1.2f
// Vertical acceleration limit (m/s²) used to distinguish purposeful altitude maneuvers from static hover
#define POS_Z_ACC_Z_THRESH                1.5f

/*=======================================================================================================
 Numerical Stability Limits
 =======================================================================================================*/
#define POS_EKF_P_MIN           1e-9f
#define POS_EKF_P_MAX           500.0f

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

	uint8_t initialized;                                // Filter operational status
	float innovation[POS_EKF_SPACE_DIM];
} POSITION_EKF;

uint8_t positionEKFInit(POSITION_EKF *ekf);
void positionEKFSetMode(POSITION_EKF *ekf, uint8_t stabilize);
void positionEKFPredict(POSITION_EKF *ekf, float ax, float ay, float az, float dt);
void positionEKFUpdateZMeasure(POSITION_EKF *ekf, float z_meas);
void positionEKFUpdateZMeasureWithBias(POSITION_EKF *ekf, float z_meas, float bias);
void positionEKFUpdateXYMeasure(POSITION_EKF *ekf, float x_meas, float y_meas);
void positionEKFUpdateXYVel(POSITION_EKF *ekf, float xVel, float yVel, float dampingStrength);
void positionEKFUpdateZVel(POSITION_EKF *ekf, float zVel, float dampingStrength);
void positionEKFReset(POSITION_EKF *ekf, float x_new, float y_new, float z_new);
void positionEKFResetAxis(POSITION_EKF *ekf, uint8_t axis, float pos_new);
void positionEKFSetDymamicPosR(POSITION_EKF *ekf, uint8_t axis, float rValue);
float positionEKFUpdateZR(POSITION_EKF *ekf, float zMeas, float bias, float ax, float ay, float az);

#endif /* SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_ */
