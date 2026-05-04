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

/*
 Q_POS: Uncertainty in the position state integration.
 High: Allows position to "jump" or teleport regardless of velocity.
 Low: Forces position to be a smooth, mathematical integral of velocity.
 */
/*
 Q_VEL: Uncertainty in the velocity state integration.
 High: Makes velocity reactive to every IMU vibration (causes "staircase" effect).
 Low: Acts as an internal low-pass filter for velocity, making movement smooth.
 */
/*
 Q_BIAS: How fast the filter tracks changes in Accelerometer Offset.
 High: Rapidly learns gravity misalignments or sensor tilt (clears static drift).
 Low: Assumes the accelerometer bias is constant. If too low, offsets never disappear.
 */
/*
 R_BARO: Trust level of the Barometer signal.
 High: Filter relies almost entirely on the IMU; results in vertical "drifting."
 Low: Filter "snaps" to barometer readings; makes the drone twitchy during wind gusts.
 */
/*
 GATE: The "Sanity Check" for sensor data.
 If (Baro_Measure - Estimated_Pos)^2 / Innovation_Covariance > GATE, data is ignored.
 Tuning: Tighten this to reject "wind blasts" from blowing on the sensor.
 */
/*
 PANIC: The threshold to force-reset the filter state.
 Number of consecutive rejected measurements before the filter "gives up" and snaps to sensor.
 */
#define POS_EKF_X_Q_POS   0.006f
#define POS_EKF_X_Q_VEL   0.12f //0.15f
#define POS_EKF_X_Q_BIAS  0.001f
#define POS_EKF_X_R_MEAS  2.0f//1.0f//Overridden by positionEKFSetDymamicPosR() function based on GPS SAcc
#define POS_EKF_X_GATE    4.0f
#define POS_EKF_X_PANIC   8

#define POS_EKF_Y_Q_POS   0.006f
#define POS_EKF_Y_Q_VEL   0.12f //0.15f
#define POS_EKF_Y_Q_BIAS  0.001f
#define POS_EKF_Y_R_MEAS  2.0f//1.0f//Overridden by positionEKFSetDymamicPosR() function based on GPS SAcc
#define POS_EKF_Y_GATE    4.0f
#define POS_EKF_Y_PANIC   8

/*----------------------------------------- Z Axis ---------------------------------------------*/
#define POS_EKF_Z_Q_POS   0.00005f
#define POS_EKF_Z_Q_VEL   0.001f
#define POS_EKF_Z_Q_BIAS  0.0005f
#define POS_EKF_Z_R_MEAS  2500.0f
#define POS_EKF_Z_GATE    5.0f
#define POS_EKF_Z_PANIC   10//100
/// Dynamic R configuration for Z-axis based on vertical velocity and acceleration
#define POS_Z_DYNAMIC_R_GAIN              2.5f
#define POS_Z_DYNAMIC_R_SMOOTH_ALPHA      0.4f
#define POS_Z_DYNAMIC_R_MIN               POS_EKF_Z_R_MEAS
#define POS_Z_DYNAMIC_R_MAX               7500.0f
#define POS_Z_DYNAMIC_R_EPS               1e-6f
#define POS_Z_DYNAMIC_R_SCALE_EPS         1e-3f
#define POS_Z_RESIDUAL_CLAMP              5.0f
#define POS_Z_ACC_XY_THRESH               1.2f   // m/s² (forward flight detection)
#define POS_Z_ACC_Z_THRESH                1.5f   // m/s² (vertical motion)


/* --- Numerical Stability Limits --- */
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
} POSITION_EKF;

uint8_t positionEKFInit(POSITION_EKF *ekf);
void positionEKFSetMode(POSITION_EKF *ekf, uint8_t stabilize);
void positionEKFPredict(POSITION_EKF *ekf, float ax, float ay, float az, float dt);
void positionEKFUpdateZMeasure(POSITION_EKF *ekf, float z_meas);
void positionEKFUpdateZMeasureWithBias(POSITION_EKF *ekf, float z_meas, float bias);
void positionEKFUpdateXYMeasure(POSITION_EKF *ekf, float x_meas, float y_meas);
void positionEKFUpdateXYVel(POSITION_EKF *ekf, float xVel , float yVel , float dampingStrength);
void positionEKFUpdateZVel(POSITION_EKF *ekf,float zVel , float dampingStrength);
void positionEKFReset(POSITION_EKF *ekf, float x_new, float y_new, float z_new);
void positionEKFResetAxis(POSITION_EKF *ekf, uint8_t axis, float pos_new);
void positionEKFSetDymamicPosR(POSITION_EKF *ekf, uint8_t axis, float rValue);
float positionEKFUpdateZR(POSITION_EKF *ekf, float zMeas, float bias, float ax, float ay, float az) ;

#endif /* SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_ */
