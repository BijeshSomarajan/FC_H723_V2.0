#ifndef SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_
#define SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_

#include <stdint.h>
#include "PositionEstimatorConfig.h"

/* =========================================================================
 * Core EKF Dimensions & Layout Indices
 * ========================================================================= */
#define POS_EKF_SPACE_DIM         3   // Physical spatial axes: [X, Y, Z]
#define POS_EKF_AXIS_DIM          4   // States tracked per axis: [P, V, B, BP]
#define POS_EKF_STATE_DIM        (POS_EKF_SPACE_DIM * POS_EKF_AXIS_DIM)

/* Spatial Axis Address Mappings */
#define POS_EKF_X_AXIS            0
#define POS_EKF_Y_AXIS            1
#define POS_EKF_Z_AXIS            2

/* Internal State Offsets Within an Axis Block */
#define POS_EKF_STATE_P           0   // Position tracking index offset
#define POS_EKF_STATE_V           1   // Velocity tracking index offset
#define POS_EKF_STATE_B           2   // Accelerometer Bias tracking index offset
#define POS_EKF_STATE_BP          3   // position bias [m]

/* Numerical Stability Constraints */
#define POS_EKF_P_MIN             1e-9f
#define POS_EKF_P_MAX             500.0f

/* --- GNSS delay compensation: state history --- */
#define POS_EKF_HIST_LEN        48       // 48 x 10ms = 480 ms of history
#define POS_EKF_HIST_PERIOD     0.01f    // snapshot cadence (100 Hz)

typedef struct {
	float p[2];      // X, Y position
	float v[2];      // X, Y velocity
} POS_EKF_HIST_ENTRY;

/* =========================================================================
 * Core State Structure Definition
 * ========================================================================= */
typedef struct {
	float x[POS_EKF_STATE_DIM];                         // State Vector [px,vx,bx,bpx, py,vy,by,bpy, pz,vz,bz,bpz]
	float P[POS_EKF_STATE_DIM][POS_EKF_STATE_DIM];      // State Error Covariance Matrix
	float Q[POS_EKF_STATE_DIM][POS_EKF_STATE_DIM];      // Process Noise Covariance Matrix

	float gateSize[POS_EKF_SPACE_DIM];                  // Mahalanobis distance rejection gates
	uint8_t panicLimit[POS_EKF_SPACE_DIM];              // Consecutive fault ceilings before a hard reset
	uint8_t rejectCount[POS_EKF_SPACE_DIM];             // Trackers for current consecutive outlier frames

	float innovation[POS_EKF_SPACE_DIM];               // Innovation history cache for telemetry logging
	uint8_t axisInitialized[POS_EKF_SPACE_DIM];         // Individual health track flags

	/* GNSS delay compensation (XY) */
	POS_EKF_HIST_ENTRY hist[POS_EKF_HIST_LEN];
	uint8_t histHead;      // index of most recent entry
	uint8_t histCount;     // number of valid entries (0 = buffer invalid)
	float histDtAcc;     // snapshot cadence accumulator
	float predOverride;
	uint8_t predOverrideValid;

} POSITION_EKF;

extern POSITION_EKF positionEkf;

/* =========================================================================
 * Public Core Filter Execution API
 * ========================================================================= */
uint8_t positionEKFInit(POSITION_EKF *ekf);
void positionEKFPredict(POSITION_EKF *ekf, float ax, float ay, float az, float dt);
void positionEKFMeasurementUpdate(POSITION_EKF *ekf, uint8_t axis, float meas, float rValue, const float H[4]);
void positionEKFReset(POSITION_EKF *ekf, uint8_t axis, uint8_t keepBias);
void positionEKFInvalidate(POSITION_EKF *ekf, uint8_t axis);

void positionEKFMeasurementUpdateLagged(POSITION_EKF *ekf, uint8_t axis, float meas, float rValue, const float H[4], float predLagged);
uint8_t positionEKFGetLaggedPred(const POSITION_EKF *ekf, uint8_t axis, const float H[4], float lagSeconds, float *predOut);

#endif /* SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_ */
