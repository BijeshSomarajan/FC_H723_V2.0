#include "PositionEstimator.h"

#include <string.h>
#include <sys/_stdint.h>

#include "../../../memory/Memory.h"
#include "../../../util/MathUtil.h"

POSITION_EKF positionEkf;

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

		ekf->P[i + 0][i + 0] = 0.1f;
		ekf->P[i + 1][i + 1] = 0.1f;
		ekf->P[i + 2][i + 2] = 0.01f;
	}

	// PATCH: Thread Safety/Encapsulation (Ensure 'prevZR' is added to your POSITION_EKF struct definition in header)
	ekf->prevZR = POS_EKF_Z_R_MEAS;

	return 1;
}

void positionEKFInvalidateAxis(POSITION_EKF *ekf, uint8_t axis) {
	if (axis <= POS_EKF_Z_AXIS) {
		ekf->axisInitialized[axis] = 0;
		ekf->rejectCount[axis] = 0;
		int i = axis * POS_EKF_AXIS_DIM;
		ekf->x[i + POS_EKF_STATE_V] = 0.0f;
	}
}

void positionEKFSetMode(POSITION_EKF *ekf, uint8_t stabilize) {
	float baseR[3] = { POS_EKF_X_R_MEAS, POS_EKF_Y_R_MEAS, POS_EKF_Z_R_MEAS };
	for (int axis = 0; axis < POS_EKF_SPACE_DIM; axis++) {
		ekf->R[axis] = stabilize ? (baseR[axis] * 0.1f) : baseR[axis];
	}
}

__ATTR_ITCM_TEXT
void calculateDynamicProcessNoise(const POSITION_EKF *ekf, int axis, float ax, float ay, float az, float *out_q00, float *out_q11, float *out_q22) {
	const int i = axis * POS_EKF_AXIS_DIM;

	// Cache baselines from pristine configuration matrix
	const float base_q00 = ekf->Q[i + 0][i + 0];
	const float base_q11 = ekf->Q[i + 1][i + 1];
	const float base_q22 = ekf->Q[i + 2][i + 2];

	// --- IMU stress tracking ---
	float accXY = fastSqrtf(ax * ax + ay * ay);
	float accZ = fabsf(az); // Assumes linear acceleration (gravity-stripped)

	float imuStressXY = accXY / POS_EKF_ACC_THRESH_XY;
	if (imuStressXY > 1.0f) {
		imuStressXY = 1.0f;
	}

	float imuStressZ = accZ / POS_EKF_ACC_THRESH_Z;
	if (imuStressZ > 1.0f) {
		imuStressZ = 1.0f;
	}

	// Peak-hold risk selection. If either plane goes crazy, protect the filter.
	float imuStress = (imuStressXY > imuStressZ) ? imuStressXY : imuStressZ;

	// --- Smooth parabolic response curves ---
	float stress2 = imuStress * imuStress;

	float velScale = 1.0f + stress2 * POS_EKF_Q_VEL_STRESS_GAIN;  // Up to 16x scale
	float posScale = 1.0f + stress2 * POS_EKF_Q_POS_STRESS_GAIN;   // Up to 7x scale
	float biasScale = 1.0f + imuStress * POS_EKF_Q_BIAS_STRESS_GAIN; // Up to 2.5x scale

	float q00 = base_q00 * posScale;
	float q11 = base_q11 * velScale;
	float q22 = base_q22 * biasScale;

	// --- Relative upper safety boundary clamping ---
	const float q00_max = base_q00 * POS_EKF_Q_MAX_SCALE;
	const float q11_max = base_q11 * POS_EKF_Q_MAX_SCALE;
	const float q22_max = base_q22 * POS_EKF_Q_MAX_SCALE;

	if (q00 > q00_max)
		q00 = q00_max;
	if (q11 > q11_max)
		q11 = q11_max;
	if (q22 > q22_max)
		q22 = q22_max;

	// Ship directly back to stack variables
	*out_q00 = q00;
	*out_q11 = q11;
	*out_q22 = q22;

}

__ATTR_ITCM_TEXT
void positionEKFPredict(POSITION_EKF *ekf, float ax, float ay, float az, float dt) {
	float acc[3] = { ax, ay, az };
	const float dt2 = dt * dt;
	const float hdt2 = 0.5f * dt2;

	for (int axis = 0; axis < POS_EKF_SPACE_DIM; axis++) {
		const int i = axis * POS_EKF_AXIS_DIM;

		/* =========================================================================
		 * 1. Non-Linear State Integration
		 * ========================================================================= */
		float a = acc[axis] - ekf->x[i + POS_EKF_STATE_B];
		ekf->x[i + POS_EKF_STATE_P] += (ekf->x[i + POS_EKF_STATE_V] * dt) + (hdt2 * a);
		ekf->x[i + POS_EKF_STATE_V] += (a * dt);
		// ekf->x[i + POS_EKF_STATE_B] handles as a random walk (unchanged here)

		/* =========================================================================
		 * 2. Explicit Discrete Jacobian Construction: F = df/dx
		 * Directly evaluated from partial derivatives of the state models above.
		 * ========================================================================= */
		const float F00 = 1.0f;   // dp/dp
		const float F01 = dt;     // dp/dv
		const float F02 = -hdt2;  // dp/db

		const float F10 = 0.0f;   // dv/dp
		const float F11 = 1.0f;   // dv/dv
		const float F12 = -dt;    // dv/db

		const float F20 = 0.0f;   // db/dp
		const float F21 = 0.0f;   // db/dv
		const float F22 = 1.0f;   // db/db

		/* =========================================================================
		 * 3. Covariance Propagation: P = F * P * F^T + Q
		 * Cached to maintain high execution speed without structural cross-talk.
		 * ========================================================================= */
		float p00 = ekf->P[i + 0][i + 0];
		float p01 = ekf->P[i + 0][i + 1];
		float p02 = ekf->P[i + 0][i + 2];
		float p11 = ekf->P[i + 1][i + 1];
		float p12 = ekf->P[i + 1][i + 2];
		float p22 = ekf->P[i + 2][i + 2];

		// Intermediate Matrix Transformation: M = F * P
		float m00 = F00 * p00 + F01 * p01 + F02 * p02;
		float m01 = F00 * p01 + F01 * p11 + F02 * p12;
		float m02 = F00 * p02 + F01 * p12 + F02 * p22;

		float m10 = F11 * p01 + F12 * p02; // F10 component is 0
		float m11 = F11 * p11 + F12 * p12; // F10 component is 0
		float m12 = F11 * p12 + F12 * p22; // F10 component is 0

		float m20 = p02;                   // F20=0, F21=0, F22=1
		float m21 = p12;
		float m22 = p22;

#if POS_EKF_DYNAMIC_Q_ENABLED == 1
		float q00, q11, q22;
		calculateDynamicProcessNoise(ekf, axis, ax, ay, az, &q00, &q11, &q22);

		// Final Multiplicative Transformation: P_new = M * F^T + Q
		float P00 = (m00 * F00 + m01 * F01 + m02 * F02) + q00;
		float P01 = (m00 * F10 + m01 * F11 + m02 * F12) + ekf->Q[i + 0][i + 1];
		float P02 = (m00 * F20 + m01 * F21 + m02 * F22) + ekf->Q[i + 0][i + 2];

		float P11 = (m10 * F10 + m11 * F11 + m12 * F12) + q11;
		float P12 = (m10 * F20 + m11 * F21 + m12 * F22) + ekf->Q[i + 1][i + 2];

		float P22 = (m20 * F20 + m21 * F21 + m22 * F22) + q22;
#else
		// Final Multiplicative Transformation: P_new = M * F^T + Q
		float P00 = (m00 * F00 + m01 * F01 + m02 * F02) + ekf->Q[i + 0][i + 0];
		float P01 = (m00 * F10 + m01 * F11 + m02 * F12) + ekf->Q[i + 0][i + 1];
		float P02 = (m00 * F20 + m01 * F21 + m02 * F22) + ekf->Q[i + 0][i + 2];

		float P11 = (m10 * F10 + m11 * F11 + m12 * F12) + ekf->Q[i + 1][i + 1];
		float P12 = (m10 * F20 + m11 * F21 + m12 * F22) + ekf->Q[i + 1][i + 2];

		float P22 = (m20 * F20 + m21 * F21 + m22 * F22) + ekf->Q[i + 2][i + 2];
#endif

		/* =========================================================================
		 * 4. Symmetric Mapping and Bound Protection
		 * ========================================================================= */
		ekf->P[i + 0][i + 0] = (P00 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : ((P00 > POS_EKF_P_MAX) ? POS_EKF_P_MAX : P00);
		ekf->P[i + 1][i + 1] = (P11 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : ((P11 > POS_EKF_P_MAX) ? POS_EKF_P_MAX : P11);
		ekf->P[i + 2][i + 2] = (P22 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : ((P22 > POS_EKF_P_MAX) ? POS_EKF_P_MAX : P22);

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
void _axisPositionUpdate(POSITION_EKF *ekf, int axis, float meas, float bias) {
	const int i = axis * POS_EKF_AXIS_DIM;

	/* Cold-Start Initialization Guard */
	if (!ekf->axisInitialized[axis]) {
		ekf->x[i + POS_EKF_STATE_P] = meas - bias;
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

	/* =========================================================================
	 * 1. Innovation and Residual Setup (H = [1, 0, 0])
	 * ========================================================================= */
	const float H0 = 1.0f;
	const float H1 = 0.0f;
	const float H2 = 0.0f;

	float y = meas - (ekf->x[i + POS_EKF_STATE_P] + bias);
	float rValue = ekf->R[axis];

	// S = H * P * H^T + R
	//float S = (H0 * ekf->P[i + 0][i + 0] + H1 * ekf->P[i + 1][0] + H2 * ekf->P[i + 2][0]) * H0 + (H0 * ekf->P[i + 0][i + 1] + H1 * ekf->P[i + 1][1] + H2 * ekf->P[i + 2][1]) * H1 + (H0 * ekf->P[i + 0][i + 2] + H1 * ekf->P[i + 1][2] + H2 * ekf->P[i + 2][2]) * H2 + rValue;
	float S = (H0 * ekf->P[i + 0][i + 0] + H1 * ekf->P[i + 1][i + 0] + H2 * ekf->P[i + 2][i + 0]) * H0 + (H0 * ekf->P[i + 0][i + 1] + H1 * ekf->P[i + 1][i + 1] + H2 * ekf->P[i + 2][i + 1]) * H1 + (H0 * ekf->P[i + 0][i + 2] + H1 * ekf->P[i + 1][i + 2] + H2 * ekf->P[i + 2][i + 2]) * H2 + rValue;

	if (S < POS_EKF_P_MIN) {
		S = POS_EKF_P_MIN;
	}
	ekf->innovation[axis] = y;

	/* Innovation Gating / Outlier Rejection */
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

	/* =========================================================================
	 * 2. Kalman Gain Computation: K = P * H^T * S^-1
	 * ========================================================================= */
	float S_inv = 1.0f / S;
	float K[3];
	K[0] = (ekf->P[i + 0][i + 0] * H0 + ekf->P[i + 0][i + 1] * H1 + ekf->P[i + 0][i + 2] * H2) * S_inv;
	K[1] = (ekf->P[i + 1][i + 0] * H0 + ekf->P[i + 1][i + 1] * H1 + ekf->P[i + 1][i + 2] * H2) * S_inv;
	K[2] = (ekf->P[i + 2][i + 0] * H0 + ekf->P[i + 2][i + 1] * H1 + ekf->P[i + 2][i + 2] * H2) * S_inv;

	/* State Update */
	ekf->x[i + POS_EKF_STATE_P] += K[0] * y;
	ekf->x[i + POS_EKF_STATE_V] += K[1] * y;
	ekf->x[i + POS_EKF_STATE_B] += K[2] * y;

	/* Cache Upper-Triangle Covariance Elements */
	float p00 = ekf->P[i + 0][i + 0];
	float p01 = ekf->P[i + 0][i + 1];
	float p02 = ekf->P[i + 0][i + 2];
	float p11 = ekf->P[i + 1][i + 1];
	float p12 = ekf->P[i + 1][i + 2];
	float p22 = ekf->P[i + 2][i + 2];

	/* =========================================================================
	 * 3. Pure Matrix Joseph Formulation: P = (I - KH)*P*(I - KH)^T + K*R*K^T
	 * ========================================================================= */
	// Intermediate: W = I - K * H
	float W00 = 1.0f - K[0] * H0;
	float W01 = -K[0] * H1;
	float W02 = -K[0] * H2;
	float W10 = -K[1] * H0;
	float W11 = 1.0f - K[1] * H1;
	float W12 = -K[1] * H2;
	float W20 = -K[2] * H0;
	float W21 = -K[2] * H1;
	float W22 = 1.0f - K[2] * H2;

	// Intermediate Matrix Product: M = W * P
	float m00 = W00 * p00 + W01 * p01 + W02 * p02;
	float m01 = W00 * p01 + W01 * p11 + W02 * p12;
	float m02 = W00 * p02 + W01 * p12 + W02 * p22;

	float m10 = W10 * p00 + W11 * p01 + W12 * p02;
	float m11 = W10 * p01 + W11 * p11 + W12 * p12;
	float m12 = W10 * p02 + W11 * p12 + W12 * p22;

	float m20 = W20 * p00 + W21 * p01 + W22 * p02;
	float m21 = W20 * p01 + W21 * p11 + W22 * p12;
	float m22 = W20 * p02 + W21 * p12 + W22 * p22;

	// Intermediate Matrix Product: N = M * W^T
	float N00 = m00 * W00 + m01 * W01 + m02 * W02;
	float N01 = m00 * W10 + m01 * W11 + m02 * W12;
	float N02 = m00 * W20 + m01 * W21 + m02 * W22;

	float N11 = m10 * W10 + m11 * W11 + m12 * W12;
	float N12 = m10 * W20 + m11 * W21 + m12 * W22;

	float N22 = m20 * W20 + m21 * W21 + m22 * W22;

	// Final Summation: P_new = N + K * R * K^T
	float P00 = N00 + K[0] * K[0] * rValue;
	float P01 = N01 + K[0] * K[1] * rValue;
	float P02 = N02 + K[0] * K[2] * rValue;
	float P11 = N11 + K[1] * K[1] * rValue;
	float P12 = N12 + K[1] * K[2] * rValue;
	float P22 = N22 + K[2] * K[2] * rValue;

	/* =========================================================================
	 * 4. Bound Protection and Structural Reflection
	 * ========================================================================= */
	ekf->P[i + 0][i + 0] = (P00 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : ((P00 > POS_EKF_P_MAX) ? POS_EKF_P_MAX : P00);
	ekf->P[i + 1][i + 1] = (P11 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : ((P11 > POS_EKF_P_MAX) ? POS_EKF_P_MAX : P11);
	ekf->P[i + 2][i + 2] = (P22 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : ((P22 > POS_EKF_P_MAX) ? POS_EKF_P_MAX : P22);

	ekf->P[i + 0][i + 1] = P01;
	ekf->P[i + 1][i + 0] = P01;
	ekf->P[i + 0][i + 2] = P02;
	ekf->P[i + 2][i + 0] = P02;
	ekf->P[i + 1][i + 2] = P12;
	ekf->P[i + 2][i + 1] = P12;
}

__ATTR_ITCM_TEXT
void _axisVelocityUpdate(POSITION_EKF *ekf, int axis, float meas, float rValue) {
	const int i = axis * POS_EKF_AXIS_DIM;

	if (!ekf->axisInitialized[axis])
		return;

	/* =========================================================================
	 * 1. Innovation and Residual Setup (H = [0, 1, 0])
	 * ========================================================================= */
	const float H0 = 0.0f;
	const float H1 = 1.0f;
	const float H2 = 0.0f;

	float y = meas - ekf->x[i + POS_EKF_STATE_V];

	// S = H * P * H^T + R
	//float S = (H0 * ekf->P[i + 0][i + 0] + H1 * ekf->P[i + 1][0] + H2 * ekf->P[i + 2][0]) * H0 + (H0 * ekf->P[i + 0][i + 1] + H1 * ekf->P[i + 1][1] + H2 * ekf->P[i + 2][1]) * H1 + (H0 * ekf->P[i + 0][i + 2] + H1 * ekf->P[i + 1][2] + H2 * ekf->P[i + 2][2]) * H2 + rValue;
	float S = (H0 * ekf->P[i + 0][i + 0] + H1 * ekf->P[i + 1][i + 0] + H2 * ekf->P[i + 2][i + 0]) * H0 + (H0 * ekf->P[i + 0][i + 1] + H1 * ekf->P[i + 1][i + 1] + H2 * ekf->P[i + 2][i + 1]) * H1 + (H0 * ekf->P[i + 0][i + 2] + H1 * ekf->P[i + 1][i + 2] + H2 * ekf->P[i + 2][i + 2]) * H2 + rValue;

	if (S < POS_EKF_P_MIN) {
		S = POS_EKF_P_MIN;
	}

	/* Innovation Gating / Outlier Rejection */
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

	/* =========================================================================
	 * 2. Kalman Gain Computation: K = P * H^T * S^-1
	 * ========================================================================= */
	float S_inv = 1.0f / S;
	float K[3];
	K[0] = (ekf->P[i + 0][i + 0] * H0 + ekf->P[i + 0][i + 1] * H1 + ekf->P[i + 0][i + 2] * H2) * S_inv;
	K[1] = (ekf->P[i + 1][i + 0] * H0 + ekf->P[i + 1][i + 1] * H1 + ekf->P[i + 1][i + 2] * H2) * S_inv;
	K[2] = (ekf->P[i + 2][i + 0] * H0 + ekf->P[i + 2][i + 1] * H1 + ekf->P[i + 2][i + 2] * H2) * S_inv;

	/* State Update */
	ekf->x[i + POS_EKF_STATE_P] += K[0] * y;
	ekf->x[i + POS_EKF_STATE_V] += K[1] * y;
	ekf->x[i + POS_EKF_STATE_B] += K[2] * y;

	/* Cache Upper-Triangle Covariance Elements */
	float p00 = ekf->P[i + 0][i + 0];
	float p01 = ekf->P[i + 0][i + 1];
	float p02 = ekf->P[i + 0][i + 2];
	float p11 = ekf->P[i + 1][i + 1];
	float p12 = ekf->P[i + 1][i + 2];
	float p22 = ekf->P[i + 2][i + 2];

	/* =========================================================================
	 * 3. Pure Matrix Joseph Formulation: P = (I - KH)*P*(I - KH)^T + K*R*K^T
	 * ========================================================================= */
	// Intermediate: W = I - K * H
	float W00 = 1.0f - K[0] * H0;
	float W01 = -K[0] * H1;
	float W02 = -K[0] * H2;
	float W10 = -K[1] * H0;
	float W11 = 1.0f - K[1] * H1;
	float W12 = -K[1] * H2;
	float W20 = -K[2] * H0;
	float W21 = -K[2] * H1;
	float W22 = 1.0f - K[2] * H2;

	// Intermediate Matrix Product: M = W * P
	float m00 = W00 * p00 + W01 * p01 + W02 * p02;
	float m01 = W00 * p01 + W01 * p11 + W02 * p12;
	float m02 = W00 * p02 + W01 * p12 + W02 * p22;

	float m10 = W10 * p00 + W11 * p01 + W12 * p02;
	float m11 = W10 * p01 + W11 * p11 + W12 * p12;
	float m12 = W10 * p02 + W11 * p12 + W12 * p22;

	float m20 = W20 * p00 + W21 * p01 + W22 * p02;
	float m21 = W20 * p01 + W21 * p11 + W22 * p12;
	float m22 = W20 * p02 + W21 * p12 + W22 * p22;

	// Intermediate Matrix Product: N = M * W^T
	float N00 = m00 * W00 + m01 * W01 + m02 * W02;
	float N01 = m00 * W10 + m01 * W11 + m02 * W12;
	float N02 = m00 * W20 + m01 * W21 + m02 * W22;

	float N11 = m10 * W10 + m11 * W11 + m12 * W12;
	float N12 = m10 * W20 + m11 * W21 + m12 * W22;

	float N22 = m20 * W20 + m21 * W21 + m22 * W22;

	// Final Summation: P_new = N + K * R * K^T
	float P00 = N00 + K[0] * K[0] * rValue;
	float P01 = N01 + K[0] * K[1] * rValue;
	float P02 = N02 + K[0] * K[2] * rValue;
	float P11 = N11 + K[1] * K[1] * rValue;
	float P12 = N12 + K[1] * K[2] * rValue;
	float P22 = N22 + K[2] * K[2] * rValue;

	/* =========================================================================
	 * 4. Bound Protection and Structural Reflection
	 * ========================================================================= */
	ekf->P[i + 0][i + 0] = (P00 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : ((P00 > POS_EKF_P_MAX) ? POS_EKF_P_MAX : P00);
	ekf->P[i + 1][i + 1] = (P11 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : ((P11 > POS_EKF_P_MAX) ? POS_EKF_P_MAX : P11);
	ekf->P[i + 2][i + 2] = (P22 < POS_EKF_P_MIN) ? POS_EKF_P_MIN : ((P22 > POS_EKF_P_MAX) ? POS_EKF_P_MAX : P22);

	ekf->P[i + 0][i + 1] = P01;
	ekf->P[i + 1][i + 0] = P01;
	ekf->P[i + 0][i + 2] = P02;
	ekf->P[i + 2][i + 0] = P02;
	ekf->P[i + 1][i + 2] = P12;
	ekf->P[i + 2][i + 1] = P12;
}

__ATTR_ITCM_TEXT
void positionEKFUpdateRPosition(POSITION_EKF *ekf, uint8_t axis, float rValue) {
	ekf->R[axis] = rValue;
}

__ATTR_ITCM_TEXT
void positionEKFUpdateXYPosition(POSITION_EKF *ekf, float x_meas, float y_meas) {
	_axisPositionUpdate(ekf, POS_EKF_X_AXIS, x_meas, 0);
	_axisPositionUpdate(ekf, POS_EKF_Y_AXIS, y_meas, 0);
}

__ATTR_ITCM_TEXT
void positionEKFUpdateXYVelocity(POSITION_EKF *ekf, float xVel, float yVel, float velR) {
	_axisVelocityUpdate(ekf, POS_EKF_X_AXIS, xVel, velR);
	_axisVelocityUpdate(ekf, POS_EKF_Y_AXIS, yVel, velR);
}

__ATTR_ITCM_TEXT
void positionEKFUpdateZPosition(POSITION_EKF *ekf, float z_meas, float bias) {
	_axisPositionUpdate(ekf, POS_EKF_Z_AXIS, z_meas, bias);
}

__ATTR_ITCM_TEXT
void positionEKFUpdateZVelocity(POSITION_EKF *ekf, float zVel, float velR) {
	_axisVelocityUpdate(ekf, POS_EKF_Z_AXIS, zVel, velR);
}
