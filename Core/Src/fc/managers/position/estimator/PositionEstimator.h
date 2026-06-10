#ifndef SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_
#define SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_

#include <stdint.h>
#include "PositionEstimatorConfig.h"

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
