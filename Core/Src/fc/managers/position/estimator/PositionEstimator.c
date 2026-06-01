#include "PositionEstimator.h"
#include <string.h>
#include "../../../memory/Memory.h"
#include "../../../util/MathUtil.h"

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
		ekf->axisInitialized[axis] = 0;

		ekf->P[i + 0][i + 0] = 5.0f;
		ekf->P[i + 1][i + 1] = 2.0f;
		ekf->P[i + 2][i + 2] = 0.1f;
	}
	ekf->initialized = 0;

	// PATCH: Thread Safety/Encapsulation (Ensure 'prevZR' is added to your POSITION_EKF struct definition in header)
	ekf->prevZR = POS_EKF_Z_R_MEAS;

	return 1;
}

void positionEKFSetMode(POSITION_EKF *ekf, uint8_t stabilize) {
	float baseR[3] = { POS_EKF_X_R_MEAS, POS_EKF_Y_R_MEAS, POS_EKF_Z_R_MEAS };
	for (int axis = 0; axis < POS_EKF_SPACE_DIM; axis++) {
		ekf->R[axis] = stabilize ? (baseR[axis] * 0.1f) : baseR[axis];
	}
}

__ATTR_ITCM_TEXT
void positionEKFSetDymamicRP(POSITION_EKF *ekf, uint8_t axis, float rValue) {
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
		 * Covariance Prediction (Optimized & Symmetrized)
		 * ========================= */
		float p00 = ekf->P[i + 0][i + 0];
		float p01 = ekf->P[i + 0][i + 1];
		float p02 = ekf->P[i + 0][i + 2];
		float p11 = ekf->P[i + 1][i + 1];
		float p12 = ekf->P[i + 1][i + 2];
		float p22 = ekf->P[i + 2][i + 2];

		float P00 = p00 + 2.0f * dt * p01 - 2.0f * hdt2 * p02 + dt2 * p11 - 2.0f * dt * hdt2 * p12 + hdt2 * hdt2 * p22 + ekf->Q[i + 0][i + 0];
		float P01 = p01 + dt * p11 - hdt2 * p12 - dt * p02 - dt2 * p12 + dt * hdt2 * p22;
		float P02 = p02 + dt * p12 - hdt2 * p22;
		float P11 = p11 - 2.0f * dt * p12 + dt2 * p22 + ekf->Q[i + 1][i + 1];
		float P12 = p12 - dt * p22;
		float P22 = p22 + ekf->Q[i + 2][i + 2];

		/* =========================
		 * Storage & Localized Stabilization
		 * ========================= */
		ekf->P[i + 0][i + 0] = (P00 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : P00;
		ekf->P[i + 1][i + 1] = (P11 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : P11;
		ekf->P[i + 2][i + 2] = (P22 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : P22;

		ekf->P[i + 0][i + 1] = P01;
		ekf->P[i + 1][i + 0] = P01;
		ekf->P[i + 0][i + 2] = P02;
		ekf->P[i + 2][i + 0] = P02;
		ekf->P[i + 1][i + 2] = P12;
		ekf->P[i + 2][i + 1] = P12;
	}
}

/* --- Axis Update Engine --- */
__ATTR_ITCM_TEXT
void _axisPositionUpdateInternal(POSITION_EKF *ekf, int axis, float meas, float externalBias) {
	const int i = axis * POS_EKF_AXIS_DIM;

	// Absolute Cold-Start Alignment Guard per individual axis
	if (!ekf->axisInitialized[axis]) {
		ekf->x[i + POS_EKF_STATE_P] = meas - externalBias;
		ekf->x[i + POS_EKF_STATE_V] = 0.0f;
		ekf->x[i + POS_EKF_STATE_B] = 0.0f;

		ekf->P[i + 0][i + 0] = 2.0f;
		ekf->P[i + 1][i + 1] = 1.0f;
		ekf->P[i + 2][i + 2] = 0.1f;
		ekf->P[i + 0][i + 1] = 0.0f;
		ekf->P[i + 1][i + 0] = 0.0f;
		ekf->P[i + 0][i + 2] = 0.0f;
		ekf->P[i + 2][i + 0] = 0.0f;
		ekf->P[i + 1][i + 2] = 0.0f;
		ekf->P[i + 2][i + 1] = 0.0f;

		ekf->rejectCount[axis] = 0;
		ekf->axisInitialized[axis] = 1;
		return;
	}

	float y = meas - (ekf->x[i + POS_EKF_STATE_P] + externalBias);
	float S = ekf->P[i + 0][i + 0] + ekf->R[axis];

	// FIX: Guard against pathological or negative variance collapse
	if (S < POS_EKF_P_MIN) {
		S = POS_EKF_P_MIN;
	}
	ekf->innovation[axis] = y;

	// Gating Outlier Rejection (Safe from division-by-zero)
	float d2 = (y * y) / S;
	if (d2 > ekf->gateSize[axis]) {
		if (ekf->rejectCount[axis] < ekf->panicLimit[axis]) {
			ekf->rejectCount[axis]++;
			return;
		}
		ekf->axisInitialized[axis] = 0;
		return;
	}
	ekf->rejectCount[axis] = 0;

	// Compute Kalman Gains
	float S_inv = 1.0f / S;
	float K[3] = { ekf->P[i + 0][i + 0] * S_inv, ekf->P[i + 1][i + 0] * S_inv, ekf->P[i + 2][i + 0] * S_inv };

	// Execute State Vector Updates
	ekf->x[i + 0] += K[0] * y;
	ekf->x[i + 1] += K[1] * y;
	ekf->x[i + 2] += K[2] * y;

	// Cache Row 0 of current covariance block before destructive write
	float p00 = ekf->P[i + 0][i + 0];
	float p01 = ekf->P[i + 0][i + 1];
	float p02 = ekf->P[i + 0][i + 2];

	// Unrolled updates enforcing mathematical symmetry directly
	float P00 = p00 - K[0] * p00;
	float P01 = p01 - K[0] * p01;
	float P02 = p02 - K[0] * p02;
	float P11 = ekf->P[i + 1][i + 1] - K[1] * p01;
	float P12 = ekf->P[i + 1][i + 2] - K[1] * p02;
	float P22 = ekf->P[i + 2][i + 2] - K[2] * p02;

	// Clamping and reflective mapping back to structures
	ekf->P[i + 0][i + 0] = (P00 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : P00;
	ekf->P[i + 1][i + 1] = (P11 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : P11;
	ekf->P[i + 2][i + 2] = (P22 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : P22;

	ekf->P[i + 0][i + 1] = P01;
	ekf->P[i + 1][i + 0] = P01;
	ekf->P[i + 0][i + 2] = P02;
	ekf->P[i + 2][i + 0] = P02;
	ekf->P[i + 1][i + 2] = P12;
	ekf->P[i + 2][i + 1] = P12;
}

__ATTR_ITCM_TEXT
void _axisPositionUpdate(POSITION_EKF *ekf, int axis, float meas) {
	_axisPositionUpdateInternal(ekf, axis, meas, 0.0f);
}

__ATTR_ITCM_TEXT
void _axisPositionUpdateWithBias(POSITION_EKF *ekf, int axis, float meas, float bias) {
	_axisPositionUpdateInternal(ekf, axis, meas, bias);
}

__ATTR_ITCM_TEXT
void _axisVelocityUpdate(POSITION_EKF *ekf, int axis, float meas_v, float R_v) {
	const int i = axis * POS_EKF_AXIS_DIM;

	if (!ekf->axisInitialized[axis])
		return;

	float y = meas_v - ekf->x[i + 1];
	float S = ekf->P[i + 1][i + 1] + R_v;

	// FIX: Guard against pathological or negative variance collapse instead of dropping frame
	if (S < POS_EKF_P_MIN) {
		S = POS_EKF_P_MIN;
	}

	// Gating Outlier Rejection (Safe from division-by-zero)
	float d2 = (y * y) / S;
	if (d2 > ekf->gateSize[axis]) {
		if (ekf->rejectCount[axis] < ekf->panicLimit[axis]) {
			ekf->rejectCount[axis]++;
			return;
		}
		ekf->axisInitialized[axis] = 0;
		ekf->rejectCount[axis] = 0;
		return;
	}
	ekf->rejectCount[axis] = 0;

	float S_inv = 1.0f / S;
	float K[3] = { ekf->P[i + 0][i + 1] * S_inv, ekf->P[i + 1][i + 1] * S_inv, ekf->P[i + 2][i + 1] * S_inv };

	ekf->x[i + 0] += K[0] * y;
	ekf->x[i + 1] += K[1] * y;
	ekf->x[i + 2] += K[2] * y;

	// Cache Row 1 parameters before modifications
	float p10 = ekf->P[i + 1][i + 0];
	float p11 = ekf->P[i + 1][i + 1];
	float p12 = ekf->P[i + 1][i + 2];

	float P00 = ekf->P[i + 0][i + 0] - K[0] * p10;
	float P01 = ekf->P[i + 0][i + 1] - K[0] * p11;
	float P02 = ekf->P[i + 0][i + 2] - K[0] * p12;
	float P11 = p11 - K[1] * p11;
	float P12 = p12 - K[1] * p12;
	float P22 = ekf->P[i + 2][i + 2] - K[2] * p12;

	ekf->P[i + 0][i + 0] = (P00 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : P00;
	ekf->P[i + 1][i + 1] = (P11 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : P11;
	ekf->P[i + 2][i + 2] = (P22 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : P22;

	ekf->P[i + 0][i + 1] = P01;
	ekf->P[i + 1][i + 0] = P01;
	ekf->P[i + 0][i + 2] = P02;
	ekf->P[i + 2][i + 0] = P02;
	ekf->P[i + 1][i + 2] = P12;
	ekf->P[i + 2][i + 1] = P12;
}

/* --- Public API Hooks --- */
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

__ATTR_ITCM_TEXT
void positionEKFUpdateXYVel(POSITION_EKF *ekf, float xVel, float yVel, float velR) {
	_axisVelocityUpdate(ekf, POS_EKF_X_AXIS, xVel, velR);
	_axisVelocityUpdate(ekf, POS_EKF_Y_AXIS, yVel, velR);
}

__ATTR_ITCM_TEXT
void positionEKFResetXYVel(POSITION_EKF *ekf) {
	positionEKFUpdateXYVel(ekf, 0.0f, 0.0f, POS_ESTIMATOR_DYNAMIC_XY_RV_RESET);
}

void positionEKFUpdateZVel(POSITION_EKF *ekf, float zVel, float velR) {
	_axisVelocityUpdate(ekf, POS_EKF_Z_AXIS, zVel, velR);
}

__ATTR_ITCM_TEXT
float getEstimatedZDynamicRP(POSITION_EKF *ekf, float zMeas, float bias, float ax, float ay, float az) {
	float zPred = ekf->x[6];
	float residual = zMeas - (zPred + bias);
	residual = constrainToRangeF(residual, -POS_ESTIMATOR_DYNAMIC_Z_RESIDUAL_CLAMP, POS_ESTIMATOR_DYNAMIC_Z_RESIDUAL_CLAMP);

	float Pzz = ekf->P[6][6];
	if (Pzz < POS_ESTIMATOR_DYNAMIC_Z_RP_EPS) {
		Pzz = POS_ESTIMATOR_DYNAMIC_Z_RP_EPS;
	}
	float denom = Pzz + POS_EKF_Z_R_MEAS + POS_ESTIMATOR_DYNAMIC_Z_RP_SCALE_EPS;
	if (denom < 1e-3f) {
		denom = 1e-3f;
	}
	float scale = 1.0f / denom;
	float residualTerm = POS_ESTIMATOR_DYNAMIC_Z_RP_GAIN * residual * residual * scale;

	float accXY = fastSqrtf(ax * ax + ay * ay);
	float accZ = fabsf(az);

	float accXYScale = constrainToRangeF(accXY / POS_ESTIMATOR_DYNAMIC_Z_ACC_XY_THRESH, 0.0f, 1.0f);
	float accZScale = constrainToRangeF(accZ / POS_ESTIMATOR_DYNAMIC_Z_ACC_Z_THRESH, 0.0f, 1.0f);
	float motionScale = constrainToRangeF(accXYScale + accZScale, 0.0f, 1.0f);

	float baseDynamicR = POS_EKF_Z_R_MEAS + residualTerm;
	float targetDynamicR = baseDynamicR + (motionScale * (POS_ESTIMATOR_DYNAMIC_Z_RP_MAX - baseDynamicR));
	targetDynamicR = constrainToRangeF(targetDynamicR, POS_ESTIMATOR_DYNAMIC_Z_RP_MIN, POS_ESTIMATOR_DYNAMIC_Z_RP_MAX);

	// PATCH: Encapsulate within structure variable to isolate thread execution contexts
	float dynamicR = ekf->prevZR + POS_ESTIMATOR_DYNAMIC_Z_RP_ALPHA * (targetDynamicR - ekf->prevZR);
	ekf->prevZR = dynamicR;

	return dynamicR;
}

float getEstimatedXYDynamicRP(float hAcc) {
	if (hAcc < POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_MIN) {
		hAcc = POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_MIN;
	}
	float dynamicR = (POS_ESTIMATOR_DYNAMIC_XY_RP_BASE + (POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_SCALE * (hAcc * hAcc)));
	if (dynamicR > POS_ESTIMATOR_DYNAMIC_XY_RP_MAX) {
		dynamicR = POS_ESTIMATOR_DYNAMIC_XY_RP_MAX;
	}
	return dynamicR;
}

float getEstimatedXYDynamicRV(float sAcc) {
	if (sAcc < POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_MIN) {
		sAcc = POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_MIN;
	}
	float dynamicRv = (POS_ESTIMATOR_DYNAMIC_XY_RV_BASE + (POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_SCALE * (sAcc * sAcc)));
	if (dynamicRv > POS_ESTIMATOR_DYNAMIC_XY_RV_MAX) {
		dynamicRv = POS_ESTIMATOR_DYNAMIC_XY_RV_MAX;
	}
	return dynamicRv;
}

void positionEKFInvalidateAxis(POSITION_EKF *ekf, uint8_t axis) {
	if (axis <= POS_EKF_Z_AXIS) {
		ekf->axisInitialized[axis] = 0;
		ekf->rejectCount[axis] = 0;
		int i = axis * POS_EKF_AXIS_DIM;
		ekf->x[i + POS_EKF_STATE_V] = 0.0f;
	}
}
