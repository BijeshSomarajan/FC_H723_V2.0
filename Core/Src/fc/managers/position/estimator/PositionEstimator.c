#include "PositionEstimator.h"

#include <string.h>
#include <sys/_stdint.h>

#include "../../../memory/Memory.h"
#include "../../../util/MathUtil.h"

float positionEKFPrevZR;

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

	positionEKFPrevZR = POS_EKF_Z_R_MEAS;

	return 1;
}

void positionEKFSetMode(POSITION_EKF *ekf, uint8_t stabilize) {
	float baseR[3] = { POS_EKF_X_R_MEAS, POS_EKF_Y_R_MEAS, POS_EKF_Z_R_MEAS };
	for (int axis = 0; axis < POS_EKF_SPACE_DIM; axis++) {
		ekf->R[axis] = stabilize ? (baseR[axis] * 0.1f) : baseR[axis];
	}

}

__ATTR_ITCM_TEXT
void positionEKFSetDymamicPosR(POSITION_EKF *ekf, uint8_t axis, float rValue) {
	ekf->R[axis] = rValue;
}

/* --- Prediction Step --- */

__ATTR_ITCM_TEXT
void positionEKFPredict(POSITION_EKF *ekf, float ax, float ay, float az, float dt) {
	float acc[3] = { ax, ay, az };
	const float dt2 = dt * dt;
	const float hdt2 = 0.5f * dt2;
	for (int axis = 0; axis < POS_EKF_SPACE_DIM; axis++) {
		const int i = axis * POS_EKF_AXIS_DIM;
		/* =========================
		 * State Prediction
		 * ========================= */
		float a = acc[axis] - ekf->x[i + POS_EKF_STATE_B];
		ekf->x[i + POS_EKF_STATE_P] += (ekf->x[i + POS_EKF_STATE_V] * dt) + (hdt2 * a);
		ekf->x[i + POS_EKF_STATE_V] += (a * dt);
		/* =========================
		 * Covariance Prediction
		 * Exact: P = FPFᵀ + Q
		 * ========================= */
		float p00 = ekf->P[i + 0][i + 0];
		float p01 = ekf->P[i + 0][i + 1];
		float p02 = ekf->P[i + 0][i + 2];

		float p10 = ekf->P[i + 1][i + 0];
		float p11 = ekf->P[i + 1][i + 1];
		float p12 = ekf->P[i + 1][i + 2];

		float p20 = ekf->P[i + 2][i + 0];
		float p21 = ekf->P[i + 2][i + 1];
		float p22 = ekf->P[i + 2][i + 2];

		float P00 = p00 + dt * (p01 + p10) + dt2 * p11 - hdt2 * (p02 + p20) - dt * hdt2 * (p12 + p21) + hdt2 * hdt2 * p22;
		float P01 = p01 + dt * p11 - hdt2 * p21 - dt * p02 - dt2 * p12 + dt * hdt2 * p22;
		float P02 = p02 + dt * p12 - hdt2 * p22;
		float P10 = p10 + dt * p11 - hdt2 * p20 - dt * p21 + dt * hdt2 * p22;
		float P11 = p11 - dt * (p12 + p21) + dt2 * p22;
		float P12 = p12 - dt * p22;
		float P20 = p20 + dt * p21 - hdt2 * p22;
		float P21 = p21 - dt * p22;
		float P22 = p22;
		/* =========================
		 * Add Process Noise
		 * ========================= */
		P00 += ekf->Q[i + 0][i + 0];
		P11 += ekf->Q[i + 1][i + 1];
		P22 += ekf->Q[i + 2][i + 2];
		/* =========================
		 * Store Back
		 * ========================= */
		ekf->P[i + 0][i + 0] = P00;
		ekf->P[i + 0][i + 1] = P01;
		ekf->P[i + 0][i + 2] = P02;

		ekf->P[i + 1][i + 0] = P10;
		ekf->P[i + 1][i + 1] = P11;
		ekf->P[i + 1][i + 2] = P12;

		ekf->P[i + 2][i + 0] = P20;
		ekf->P[i + 2][i + 1] = P21;
		ekf->P[i + 2][i + 2] = P22;
	}
	/* =========================
	 * Symmetry & Positivity
	 * ========================= */
	for (int r = 0; r < POS_EKF_STATE_DIM; r++) {
		for (int c = r; c < POS_EKF_STATE_DIM; c++) {
			if (r == c) {
				if (ekf->P[r][r] < POS_EKF_P_MIN) {
					ekf->P[r][r] = POS_EKF_P_MIN;
				}
			} else {
				float avg = 0.5f * (ekf->P[r][c] + ekf->P[c][r]);
				ekf->P[r][c] = ekf->P[c][r] = avg;
			}
		}
	}
}

/**
 * @brief Resets Position and Velocity for a single axis while preserving learned bias.
 * @param ekf Pointer to the EKF structure.
 * @param axis Axis index (0 = X, 1 = Y, 2 = Z).
 * @param pos_new New position for the selected axis.
 */
void positionEKFResetAxis(POSITION_EKF *ekf, uint8_t axis, float pos_new) {

	if (axis >= POS_EKF_SPACE_DIM) return;

	int i = axis * POS_EKF_AXIS_DIM;

	// 1. Reset State: Position to new value, Velocity to zero.
	// Bias (x[i + 2]) is preserved.
	ekf->x[i + POS_EKF_STATE_P] = pos_new;
	ekf->x[i + POS_EKF_STATE_V] = 0.0f;

	// 2. Reset Covariance: Re-initialize uncertainty for this axis block.
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			ekf->P[i + r][i + c] = 0.0f;
		}
	}

	ekf->P[i + 0][i + 0] = 10.0f; // Position uncertainty
	ekf->P[i + 1][i + 1] = 10.0f; // Velocity uncertainty
	ekf->P[i + 2][i + 2] = 0.01f; // Bias uncertainty (kept low)

	// 3. Reset Gating
	ekf->rejectCount[axis] = 0;
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
__ATTR_ITCM_TEXT
void _axisPositionUpdate(POSITION_EKF *ekf, int axis, float meas) {
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
	//Exporting the innovation for potential external monitoring or adaptive logic
	ekf->innovation[axis] = y;

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

__ATTR_ITCM_TEXT
void _axisPositionUpdateWithBias(POSITION_EKF *ekf, int axis, float meas, float bias) {
	const int i = axis * POS_EKF_AXIS_DIM;

	// Initial vertical alignment
	if (!ekf->initialized && axis == POS_EKF_Z_AXIS) {
		ekf->x[i + POS_EKF_STATE_P] = meas;
		ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P] = 2.0f;
		return;
	}

	// Calculate innovation (y) including the bias term
	// For Z-axis, this accounts for the Venturi effect
	float y = meas - (ekf->x[i + POS_EKF_STATE_P] + bias);
	float S = ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P] + ekf->R[axis];
	float d2 = (y * y) / S;
	//Exporting the innovation for potential external monitoring or adaptive logic
	ekf->innovation[axis] = y;

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

__ATTR_ITCM_TEXT
void _axisVelocityUpdate(POSITION_EKF *ekf, int axis, float meas_v, float R_v) {
	const int i = (axis * POS_EKF_AXIS_DIM);

	float y = (meas_v - ekf->x[i + 1]);
	float S = (ekf->P[i + 1][i + 1] + R_v);

	if (S < 1e-6f) {
		return;
	}

	float d2 = ((y * y) / S);

	if (d2 > ekf->gateSize[axis]) {
		if (ekf->rejectCount[axis] < ekf->panicLimit[axis]) {
			ekf->rejectCount[axis]++;
			return;
		}
	}
	ekf->rejectCount[axis] = 0;

	float K[3] = { (ekf->P[i + 0][i + 1] / S), (ekf->P[i + 1][i + 1] / S), (ekf->P[i + 2][i + 1] / S) };

	ekf->x[i + 0] += (K[0] * y);
	ekf->x[i + 1] += (K[1] * y);
	ekf->x[i + 2] += (K[2] * y);

	float p1[3] = { ekf->P[i + 1][i + 0], ekf->P[i + 1][i + 1], ekf->P[i + 1][i + 2] };

	for (int r = 0; (r < 3); r++) {
		for (int c = 0; (c < 3); c++) {
			ekf->P[i + r][i + c] -= (K[r] * p1[c]);
		}
	}

	for (int r = 0; (r < 3); r++) {
		for (int c = r; (c < 3); c++) {
			int row = (i + r);
			int col = (i + c);
			if (row == col) {
				if (ekf->P[row][col] < POS_EKF_P_MIN) {
					ekf->P[row][col] = POS_EKF_P_MIN;
				}
			} else {
				float avg = (0.5f * (ekf->P[row][col] + ekf->P[col][row]));
				ekf->P[row][col] = ekf->P[col][row] = avg;
			}
		}
	}
}

__ATTR_ITCM_TEXT
float positionEKFUpdateZR(POSITION_EKF *ekf, float zMeas, float bias, float ax, float ay, float az) {
	float zPred = ekf->x[6];
	float residual = zMeas - (zPred + bias);
	residual = constrainToRangeF(residual, -POS_Z_RESIDUAL_CLAMP, POS_Z_RESIDUAL_CLAMP);
	float Pzz = ekf->P[6][6];
	if (Pzz < POS_Z_DYNAMIC_R_EPS) {
		Pzz = POS_Z_DYNAMIC_R_EPS;
	}
	float denom = Pzz + POS_EKF_Z_R_MEAS + POS_Z_DYNAMIC_R_SCALE_EPS;
	if (denom < 1e-3f) {
		denom = 1e-3f;
	}
	float scale = 1.0f / denom;
	float residualTerm = POS_Z_DYNAMIC_R_GAIN * residual * residual * scale;
	float accXY = fabsf(ax) + fabsf(ay);
	float accZ = fabsf(az);
	float accXYScale = constrainToRangeF(accXY / POS_Z_ACC_XY_THRESH, 0.0f, 1.0f);
	float accZScale = constrainToRangeF(accZ / POS_Z_ACC_Z_THRESH, 0.0f, 1.0f);
	float motionScale = constrainToRangeF(accXYScale + accZScale, 0.0f, 1.0f);
	float dynamicR;
	if (motionScale > 0.15f) {
		dynamicR = POS_Z_DYNAMIC_R_MAX;
	} else {
		dynamicR = POS_EKF_Z_R_MEAS + residualTerm;
	}
	dynamicR = constrainToRangeF(dynamicR, POS_Z_DYNAMIC_R_MIN, POS_Z_DYNAMIC_R_MAX);
	dynamicR = positionEKFPrevZR + POS_Z_DYNAMIC_R_SMOOTH_ALPHA * (dynamicR - positionEKFPrevZR);
	positionEKFPrevZR = dynamicR;
	return dynamicR;
}

__ATTR_ITCM_TEXT
void positionEKFUpdateZMeasure(POSITION_EKF *ekf, float z_meas) {
	_axisPositionUpdate(ekf, POS_EKF_Z_AXIS, z_meas);
	ekf->initialized = 1;
}

__ATTR_ITCM_TEXT
void positionEKFUpdateZMeasureWithBias(POSITION_EKF *ekf, float z_meas, float bias) {
	_axisPositionUpdateWithBias(ekf, POS_EKF_Z_AXIS, z_meas, bias);
	ekf->initialized = 1;
}

__ATTR_ITCM_TEXT
void positionEKFUpdateXYMeasure(POSITION_EKF *ekf, float x_meas, float y_meas) {
	_axisPositionUpdate(ekf, POS_EKF_X_AXIS, x_meas);
	_axisPositionUpdate(ekf, POS_EKF_Y_AXIS, y_meas);
}

/**
 * @brief Apply damping to Horizontal axes only.
 * Call this when GPS is lost or sticks are centered.
 */
__ATTR_ITCM_TEXT
void positionEKFUpdateXYVel(POSITION_EKF *ekf, float xVel, float yVel, float dampingStrength) {
	// damping_strength: 0.1 (Aggressive) to 2.0 (Loose/Soft)
	_axisVelocityUpdate(ekf, POS_EKF_X_AXIS, xVel, dampingStrength);
	_axisVelocityUpdate(ekf, POS_EKF_Y_AXIS, yVel, dampingStrength);
}

/**
 * @brief Apply damping to Vertical axis.
 * Useful during landing detection to prevent "bounce" estimates.
 */
void positionEKFUpdateZVel(POSITION_EKF *ekf, float zVel, float dampingStrength) {
	_axisVelocityUpdate(ekf, POS_EKF_Z_AXIS, zVel, dampingStrength);
}
