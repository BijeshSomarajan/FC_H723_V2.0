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

/* --- Default Process Noise Q (Model Uncertainty) --- */

// Q_POS: Uncertainty in the position state integration.
// High: Allows position to "jump" or teleport regardless of velocity.
// Low: Forces position to be a smooth, mathematical integral of velocity.

// Q_VEL: Uncertainty in the velocity state integration.
// High: Makes velocity reactive to every IMU vibration (causes "staircase" effect).
// Low: Acts as an internal low-pass filter for velocity, making movement smooth.

// Q_BIAS: How fast the filter tracks changes in Accelerometer Offset.
// High: Rapidly learns gravity misalignments or sensor tilt (clears static drift).
// Low: Assumes the accelerometer bias is constant. If too low, offsets never disappear.

// R_BARO: Trust level of the Barometer signal.
// High: Filter relies almost entirely on the IMU; results in vertical "drifting."
// Low: Filter "snaps" to barometer readings; makes the drone twitchy during wind gusts.

// GATE: The "Sanity Check" for sensor data.
// If (Baro_Measure - Estimated_Pos)^2 / Innovation_Covariance > GATE, data is ignored.
// Tuning: Tighten this to reject "wind blasts" from blowing on the sensor.

// PANIC: The threshold to force-reset the filter state.
// Number of consecutive rejected measurements before the filter "gives up" and snaps to sensor.


#define POS_EKF_X_Q_POS         0.01f
#define POS_EKF_X_Q_VEL         0.1f
#define POS_EKF_X_Q_BIAS        0.001f
#define POS_EKF_X_R_GPS         1.5f
#define POS_EKF_X_GATE          30.0f
#define POS_EKF_X_PANIC         20

#define POS_EKF_Y_Q_POS         0.01f
#define POS_EKF_Y_Q_VEL         0.1f
#define POS_EKF_Y_Q_BIAS        0.001f
#define POS_EKF_Y_R_GPS         1.5f
#define POS_EKF_Y_GATE          30.0f
#define POS_EKF_Y_PANIC         20

#define POS_EKF_Z_Q_POS         0.00013f
#define POS_EKF_Z_Q_VEL         0.00128f
#define POS_EKF_Z_Q_BIAS        0.001f
#define POS_EKF_Z_R_BARO        5000.0f
#define POS_EKF_Z_GATE          6.0f
#define POS_EKF_Z_PANIC         100

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

/**
 * @brief Initializes the Position EKF with default parameters.
 * Sets initial state to zero and configures noise matrices.
 * * @param ekf Pointer to the EKF instance to initialize.
 * @return 1 on success, 0 on failure.
 */
uint8_t positionEKFInit(POSITION_EKF *ekf);

/**
 * @brief Prediction Step (Time Update).
 * Integrates Earth-frame acceleration into position and velocity.
 * * @param ekf Pointer to the EKF instance.
 * @param ax Earth-frame X acceleration (m/s^2).
 * @param ay Earth-frame Y acceleration (m/s^2).
 * @param az Earth-frame Z acceleration (m/s^2).
 * @param dt Delta time since last prediction (s).
 */
void positionEKFPredict(POSITION_EKF *ekf, float ax, float ay, float az, float dt);

/**
 * @brief Vertical Measurement Update.
 * Corrects altitude and vertical velocity using Barometer or ToF data.
 * * @param ekf Pointer to the EKF instance.
 * @param z_meas Measured altitude (m).
 */
void positionEKFUpdateZMeasure(POSITION_EKF *ekf, float z_meas);

/**
 * @brief Horizontal Measurement Update.
 * Corrects horizontal position and velocity using GPS or Flow data.
 * * @param ekf Pointer to the EKF instance.
 * @param x_meas Measured X position (m).
 * @param y_meas Measured Y position (m).
 */
void positionEKFUpdateXYMeasure(POSITION_EKF *ekf, float x_meas, float y_meas);

/**
 * @brief Apply damping to Vertical axis.
 * Useful during landing detection to prevent "bounce" estimates.
 * * @param ekf Pointer to the EKF instance.
 * @param dampingStrength Damping factor (0.1 = Aggressive, 2.0 = Loose).
 */
void positionEKFApplyXYDamping(POSITION_EKF *ekf, float dampingStrength);

/**
 * @brief Apply damping to Vertical axis.
 * Useful during landing detection to prevent "bounce" estimates.
 * * @param ekf Pointer to the EKF instance.
 * @param dampingStrength Damping factor (0.1 = Aggressive, 2.0 = Loose).
 */
void positionEKFApplyZDamping(POSITION_EKF *ekf, float dampingStrength);

/**
 * @brief Resets Position and Velocity while preserving learned biases.
 * Call this when GPS is regained or during takeoff to prevent large jumps.
 * * @param ekf Pointer to the EKF instance.
 * @param x_new New X position (usually 0.0f).
 * @param y_new New Y position (usually 0.0f).
 * @param z_new New Z position (usually current baro height).
 */
void positionEKFReset(POSITION_EKF *ekf, float x_new, float y_new, float z_new) ;

#endif /* SRC_FC_SENSORS_ALTITUDE_ESTIMATION_EKFALTITUDEESTIMATOR_H_ */
