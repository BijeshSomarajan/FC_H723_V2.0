#include "PositionEstimator.h"
#include <string.h>
#include <math.h>

/* --- Initialization --- */
uint8_t positionEKFInit(POSITION_EKF *ekf) {
	memset(ekf, 0, sizeof(POSITION_EKF));
	float q_p[3] = { POS_EKF_X_Q_POS, POS_EKF_Y_Q_POS, POS_EKF_Z_Q_POS };
	float q_v[3] = { POS_EKF_X_Q_VEL, POS_EKF_Y_Q_VEL, POS_EKF_Z_Q_VEL };
	float q_b[3] = { POS_EKF_X_Q_BIAS, POS_EKF_Y_Q_BIAS, POS_EKF_Z_Q_BIAS };
	float r_v[3] = { POS_EKF_X_R_MEAS, POS_EKF_Y_R_MEAS, POS_EKF_Z_R_MEAS };
	float g_s[3] = { POS_EKF_X_GATE, POS_EKF_Y_GATE, POS_EKF_Z_GATE };
	uint8_t p_l[3] = { POS_EKF_X_PANIC, POS_EKF_Y_PANIC, POS_EKF_Z_PANIC };

	for (int axis = 0; axis < POS_EKF_SPACE_DIM; axis++) {
		int i = axis * POS_EKF_AXIS_DIM;

		ekf->Q[i + 0][i + 0] = q_p[axis];
		ekf->Q[i + 1][i + 1] = q_v[axis];
		ekf->Q[i + 2][i + 2] = q_b[axis];

		ekf->R[axis] = r_v[axis];
		ekf->gateSize[axis] = g_s[axis];
		ekf->panicLimit[axis] = p_l[axis];
		ekf->rejectCount[axis] = 0;

		ekf->P[i + 0][i + 0] = 10.0f;
		ekf->P[i + 1][i + 1] = 10.0f;
		ekf->P[i + 2][i + 2] = 1.0f;
	}
	ekf->initialized = 0;
	return 1;
}

/* --- Prediction Step --- */
void positionEKFPredict(POSITION_EKF *ekf, float ax, float ay, float az, float dt) {
	float acc[3] = { ax, ay, az };
	float hdt2 = 0.5f * dt * dt;

	for (int axis = 0; axis < POS_EKF_SPACE_DIM; axis++) {
		int i = axis * POS_EKF_AXIS_DIM;
		float a = acc[axis] - ekf->x[i + POS_EKF_STATE_B];

		// State update
		ekf->x[i + POS_EKF_STATE_P] += (ekf->x[i + POS_EKF_STATE_V] * dt) + (hdt2 * a);
		ekf->x[i + POS_EKF_STATE_V] += (a * dt);

		// Covariance prediction
		float p00 = ekf->P[i + 0][i + 0], p01 = ekf->P[i + 0][i + 1], p02 = ekf->P[i + 0][i + 2];
		float p10 = ekf->P[i + 1][i + 0], p11 = ekf->P[i + 1][i + 1], p12 = ekf->P[i + 1][i + 2];
		float p20 = ekf->P[i + 2][i + 0], p21 = ekf->P[i + 2][i + 1], p22 = ekf->P[i + 2][i + 2];

		float fp00 = p00 + dt * p10 - hdt2 * p20;
		float fp01 = p01 + dt * p11 - hdt2 * p21;
		float fp02 = p02 + dt * p12 - hdt2 * p22;
		float fp10 = p10 - dt * p20;
		float fp11 = p11 - dt * p21;
		float fp12 = p12 - dt * p22;

		ekf->P[i + 0][i + 0] = fp00 + dt * fp01 - hdt2 * fp02 + ekf->Q[i + 0][i + 0];
		ekf->P[i + 0][i + 1] = fp01 - dt * fp02;
		ekf->P[i + 0][i + 2] = fp02;
		ekf->P[i + 1][i + 0] = fp10 + dt * fp11 - dt * fp12;
		ekf->P[i + 1][i + 1] = fp11 - dt * fp12 + ekf->Q[i + 1][i + 1];
		ekf->P[i + 1][i + 2] = fp12;
		ekf->P[i + 2][i + 0] = p20 + dt * p21 - hdt2 * p22;
		ekf->P[i + 2][i + 1] = p21 - dt * p22;
		ekf->P[i + 2][i + 2] = p22 + ekf->Q[i + 2][i + 2];
	}

	// Symmetry & positivity
	for (int r = 0; r < POS_EKF_STATE_DIM; r++) {
		for (int c = r; c < POS_EKF_STATE_DIM; c++) {
			if (r == c) {
				if (ekf->P[r][r] < POS_EKF_P_MIN) ekf->P[r][r] = POS_EKF_P_MIN;
			} else {
				float avg = 0.5f * (ekf->P[r][c] + ekf->P[c][r]);
				ekf->P[r][c] = ekf->P[c][r] = avg;
			}
		}
	}
}

/**
 * @brief Resets Position and Velocity while preserving learned biases.
 * @param ekf Pointer to the EKF structure.
 * @param x_new New X position (usually 0.0f).
 * @param y_new New Y position (usually 0.0f).
 * @param z_new New Z position (usually current baro height).
 */
void positionEKFReset(POSITION_EKF *ekf, float x_new, float y_new, float z_new) {
	float new_pos[3] = { x_new, y_new, z_new };

	for (int axis = 0; axis < POS_EKF_SPACE_DIM; axis++) {
		int i = axis * POS_EKF_AXIS_DIM;

		// 1. Reset State: Position to new value, Velocity to zero.
		// We leave x[i + 2] (Bias) untouched so it remains calibrated.
		ekf->x[i + POS_EKF_STATE_P] = new_pos[axis];
		ekf->x[i + POS_EKF_STATE_V] = 0.0f;

		// 2. Reset Covariance: Re-initialize uncertainty for the axis block.
		// We set high uncertainty for P and V, but keep Bias uncertainty low.
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				ekf->P[i + r][i + c] = 0.0f;
			}
		}
		ekf->P[i + 0][i + 0] = 10.0f; // Position uncertainty
		ekf->P[i + 1][i + 1] = 10.0f; // Velocity uncertainty
		// Bias uncertainty remains at current or a small stable value
		ekf->P[i + 2][i + 2] = 0.01f;

		// 3. Reset Gating: Clear the rejection counter to allow immediate updates
		ekf->rejectCount[axis] = 0;
	}
}

/* --- Axis Update --- */
static void _axisPositionUpdate(POSITION_EKF *ekf, int axis, float meas) {
	const int i = axis * POS_EKF_AXIS_DIM;

	// Initial vertical alignment
	if (!ekf->initialized && axis == POS_EKF_Z_AXIS) {
		ekf->x[i + POS_EKF_STATE_P] = meas;
		ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P] = 2.0f;
		return;
	}

	float y = meas - ekf->x[i + POS_EKF_STATE_P];
	float S = ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P] + ekf->R[axis];
	float d2 = (y * y) / S;

	// Gating logic
	if (d2 > ekf->gateSize[axis]) {
		if (ekf->rejectCount[axis] < ekf->panicLimit[axis]) {
			ekf->rejectCount[axis]++;
			return;
		}
	}
	ekf->rejectCount[axis] = 0;

	// Kalman Gain
	float K[3] = { ekf->P[i + 0][i + 0] / S, ekf->P[i + 1][i + 0] / S, ekf->P[i + 2][i + 0] / S };

	// State update
	ekf->x[i + 0] += K[0] * y;
	ekf->x[i + 1] += K[1] * y;
	ekf->x[i + 2] += K[2] * y;

	// Covariance update
	float p0[3] = { ekf->P[i + 0][i + 0], ekf->P[i + 0][i + 1], ekf->P[i + 0][i + 2] };
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			ekf->P[i + r][i + c] -= K[r] * p0[c];

	// Stabilize block
	for (int r = 0; r < 3; r++) {
		for (int c = r; c < 3; c++) {
			int row = i + r;
			int col = i + c;
			if (row == col) {
				if (ekf->P[row][col] < POS_EKF_P_MIN) ekf->P[row][col] = POS_EKF_P_MIN;
			} else {
				float avg = 0.5f * (ekf->P[row][col] + ekf->P[col][row]);
				ekf->P[row][col] = ekf->P[col][row] = avg;
			}
		}
	}
}

/**
 * @brief Performs a Kalman update using a velocity measurement (Inertial Damping).
 * @param ekf   Pointer to the EKF structure.
 * @param axis  The axis index (0=X, 1=Y, 2=Z).
 * @param meas_v The reference velocity (usually 0.0f for damping).
 * @param R_v   The noise/uncertainty of this velocity measurement.
 */
static void _axisVelocityUpdate(POSITION_EKF *ekf, int axis, float meas_v, float R_v) {
	const int i = axis * POS_EKF_AXIS_DIM;

	// 1. Innovation (y)
	// For this update, H = [0, 1, 0]. We are observing the Velocity state (index i + 1).
	float y = meas_v - ekf->x[i + 1];

	// 2. Innovation Covariance (S)
	// S = HPH' + R. Since H = [0, 1, 0], S is simply the Velocity Variance + R.
	float S = ekf->P[i + 1][i + 1] + R_v;

	// 3. Kalman Gain (K)
	// K = PH' / S. With H' as [0; 1; 0], we pull the second column of the axis block.
	float K[3] = { ekf->P[i + 0][i + 1] / S, // Cross-covariance: Pos/Vel
	ekf->P[i + 1][i + 1] / S, // Velocity Variance
	ekf->P[i + 2][i + 1] / S  // Cross-covariance: Bias/Vel
	};

	// 4. State Update
	ekf->x[i + 0] += K[0] * y; // Adjusts Position based on velocity error
	ekf->x[i + 1] += K[1] * y; // Adjusts Velocity
	ekf->x[i + 2] += K[2] * y; // Adjusts Bias (Learns bias from persistent velocity error)

	// 5. Covariance Update (P = (I - KH)P)
	// Since H = [0, 1, 0], we must cache the Velocity Row (row 1 of the block)
	float p1[3] = { ekf->P[i + 1][i + 0], ekf->P[i + 1][i + 1], ekf->P[i + 1][i + 2] };

	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			// P_new[r][c] = P_old[r][c] - K[r] * H * P_old
			ekf->P[i + r][i + c] -= K[r] * p1[c];
		}
	}

	// 6. Full Resymmetrization and Positivity pass for the axis block
	for (int r = 0; r < 3; r++) {
		for (int c = r; c < 3; c++) {
			int row = i + r;
			int col = i + c;
			if (row == col) {
				if (ekf->P[row][col] < POS_EKF_P_MIN) ekf->P[row][col] = POS_EKF_P_MIN;
			} else {
				float avg = 0.5f * (ekf->P[row][col] + ekf->P[col][row]);
				ekf->P[row][col] = ekf->P[col][row] = avg;
			}
		}
	}
}

/* --- External Update Functions --- */
void positionEKFUpdateZMeasure(POSITION_EKF *ekf, float z_meas) {
	_axisPositionUpdate(ekf, POS_EKF_Z_AXIS, z_meas);
	ekf->initialized = 1;
}

void positionEKFUpdateXYMeasure(POSITION_EKF *ekf, float x_meas, float y_meas) {
	_axisPositionUpdate(ekf, POS_EKF_X_AXIS, x_meas);
	_axisPositionUpdate(ekf, POS_EKF_Y_AXIS, y_meas);
}

/**
 * @brief Apply damping to Horizontal axes only.
 * Call this when GPS is lost or sticks are centered.
 */
void positionEKFApplyXYDamping(POSITION_EKF *ekf, float dampingStrength) {
	// damping_strength: 0.1 (Aggressive) to 2.0 (Loose/Soft)
	_axisVelocityUpdate(ekf, POS_EKF_X_AXIS, 0.0f, dampingStrength);
	_axisVelocityUpdate(ekf, POS_EKF_Y_AXIS, 0.0f, dampingStrength);
}

/**
 * @brief Apply damping to Vertical axis.
 * Useful during landing detection to prevent "bounce" estimates.
 */
void positionEKFApplyZDamping(POSITION_EKF *ekf, float dampingStrength) {
	_axisVelocityUpdate(ekf, POS_EKF_Z_AXIS, 0.0f, dampingStrength);
}
