#ifndef SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_
#define SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_

#include <stdint.h>

/* =========================================================================
 * Core EKF Dimensions & Layout Indices
 * ========================================================================= */
#define POS_EKF_SPACE_DIM         3   // Physical spatial axes: [X, Y, Z]
#define POS_EKF_AXIS_DIM          3   // States tracked per axis: [P, V, B]
#define POS_EKF_STATE_DIM         9   // Total dimension of State vector

/* Spatial Axis Address Mappings */
#define POS_EKF_X_AXIS            0
#define POS_EKF_Y_AXIS            1
#define POS_EKF_Z_AXIS            2

/* Internal State Offsets Within an Axis Block */
#define POS_EKF_STATE_P           0   // Position tracking index offset
#define POS_EKF_STATE_V           1   // Velocity tracking index offset
#define POS_EKF_STATE_B           2   // Accelerometer Bias tracking index offset

/* Numerical Stability Constraints */
#define POS_EKF_P_MIN             1e-9f
#define POS_EKF_P_MAX             500.0f

//Adaptive Q Tuning
#define POS_EKF_DYNAMIC_Q_ENABLED   1
#define POS_EKF_ACC_THRESH_XY       15.0f //3.5f  //0.35g
#define POS_EKF_ACC_THRESH_Z        20.0f  //3.5f  //0.35g
#define POS_EKF_Q_MAX_SCALE         30.0f //1e3f
#define POS_EKF_Q_POS_STRESS_GAIN   6.0f   // Max 7x scale (1.0 + 1.0 * 6.0)
#define POS_EKF_Q_VEL_STRESS_GAIN   15.0f  // Max 16x scale (1.0 + 1.0 * 15.0)
#define POS_EKF_Q_BIAS_STRESS_GAIN  1.5f

/* =========================================================================
 * Tuning Parameters: Horizontal Axis (X, Y) - GNSS & IMU Fusion
 * ========================================================================= */
// [+] Faster position tracking response   | [-] Jittery position state estimates
#define POS_EKF_X_Q_POS           0.006f
#define POS_EKF_Y_Q_POS           0.006f
// [+] High trust in raw GNSS velocity    | [-] High trust in short-term IMU prediction
#define POS_EKF_X_Q_VEL           0.12f
#define POS_EKF_Y_Q_VEL           0.12f
// [+] Fast adaptation to IMU thermal bias | [-] Locks bias firmly down (slow drift tracking)
#define POS_EKF_X_Q_BIAS          0.001f
#define POS_EKF_Y_Q_BIAS          0.001f
// [+] Smooth track (ignores GPS jitter)   | [-] Aggressively snaps to raw GPS data (twitchy)
#define POS_EKF_X_R_MEAS          2.5f //0.01f
#define POS_EKF_Y_R_MEAS          2.5f //0.01f
// [+] Accepts larger GPS steps/glitches   | [-] Rejects valid aggressive maneuvers as outliers
#define POS_EKF_X_GATE            4.0f
#define POS_EKF_Y_GATE            4.0f
// [+] Tolerant of brief GNSS signal drops  | [-] Rapidly panics and resets filter during glitches
#define POS_EKF_X_PANIC           8
#define POS_EKF_Y_PANIC           8

/* =========================================================================
 * Tuning Parameters: Vertical Axis (Z) - Barometer & IMU Fusion
 * ========================================================================= */
// [+] Snappier altitude tracking response | [-] Smooth but noticeably delayed altitude estimation
#define POS_EKF_Z_Q_POS           0.000005f
// [+] Quicker vertical velocity updates  | [-] Smoother vertical transitions
#define POS_EKF_Z_Q_VEL           0.000002f
// [+] Rapid adaptation to weather shifts  | [-] Solid baseline calculation (slow drift tracking)
#define POS_EKF_Z_Q_BIAS          0.000005f
// [+] Smooth altitude (low trust in baro) | [-] Razor-sharp hold (twitches in ground effect/wind)
#define POS_EKF_Z_R_MEAS          5.0f
// [+] Tolerates sudden wind/pressure spikes| [-] Rejects fast vertical maneuvers (climb/descend lag)
#define POS_EKF_Z_GATE            3.0f
// [+] Tolerates longer pressure anomalies  | [-] Aggressively resets filter at minor baro glitches
#define POS_EKF_Z_PANIC           6

/* =========================================================================
 * Core State Structure Definition
 * ========================================================================= */
typedef struct {
	float x[POS_EKF_STATE_DIM];                         // State vector: [px, vx, bx, py, vy, by, pz, vz, bz]
	float P[POS_EKF_STATE_DIM][POS_EKF_STATE_DIM];      // State Error Covariance Matrix
	float Q[POS_EKF_STATE_DIM][POS_EKF_STATE_DIM];      // Process Noise Covariance Matrix

	float R[POS_EKF_SPACE_DIM];                         // Current Measurement Noise Covariance per axis
	float gateSize[POS_EKF_SPACE_DIM];                  // Mahalanobis distance rejection gates
	uint8_t panicLimit[POS_EKF_SPACE_DIM];              // Consecutive fault ceilings before a hard reset
	uint8_t rejectCount[POS_EKF_SPACE_DIM];             // Trackers for current consecutive outlier frames

	float innovation[POS_EKF_SPACE_DIM];               // Innovation history cache for telemetry logging
	uint8_t axisInitialized[POS_EKF_SPACE_DIM];         // Individual health track flags

	float prevZR;                                       // Context tracking for dynamic variance modifications

} POSITION_EKF;

extern POSITION_EKF positionEkf;

/* =========================================================================
 * Public Core Filter Execution API
 * ========================================================================= */
uint8_t positionEKFInit(POSITION_EKF *ekf);

void positionEKFInvalidateAxis(POSITION_EKF *ekf, uint8_t axis);
void positionEKFSetMode(POSITION_EKF *ekf, uint8_t stabilize);

void positionEKFPredict(POSITION_EKF *ekf, float ax, float ay, float az, float dt);

void positionEKFUpdateXYPosition(POSITION_EKF *ekf, float x_meas, float y_meas);
void positionEKFUpdateZPosition(POSITION_EKF *ekf, float z_meas, float bias);

void positionEKFUpdateXYVelocity(POSITION_EKF *ekf, float xVel, float yVel, float rValue);
void positionEKFUpdateZVelocity(POSITION_EKF *ekf, float zVel, float rValue);

void positionEKFUpdateRPosition(POSITION_EKF *ekf, uint8_t axis, float rValue);

#endif /* SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_ */
