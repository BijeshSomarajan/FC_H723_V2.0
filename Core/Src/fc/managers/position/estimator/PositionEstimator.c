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
	float g_s[3] = { POS_EKF_X_GATE, POS_EKF_Y_GATE, POS_EKF_Z_GATE };
	uint8_t p_l[3] = { POS_EKF_X_PANIC, POS_EKF_Y_PANIC, POS_EKF_Z_PANIC };

	for (int axis = 0; axis < POS_EKF_SPACE_DIM; axis++) {
		int i = axis * POS_EKF_AXIS_DIM;

		ekf->Q[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P] = q_p[axis];
		ekf->Q[i + POS_EKF_STATE_V][i + POS_EKF_STATE_V] = q_v[axis];
		ekf->Q[i + POS_EKF_STATE_B][i + POS_EKF_STATE_B] = q_b[axis];

		ekf->gateSize[axis] = g_s[axis];
		ekf->panicLimit[axis] = p_l[axis];
		ekf->rejectCount[axis] = 0;
		ekf->axisInitialized[axis] = 0;

		ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P] = 0.1f;
		ekf->P[i + POS_EKF_STATE_V][i + POS_EKF_STATE_V] = 0.1f;
		ekf->P[i + POS_EKF_STATE_B][i + POS_EKF_STATE_B] = 0.01f;

		ekf->x[i + POS_EKF_STATE_BP] = 0.0f;

		// Position bias only meaningful for Z
		if (axis == POS_EKF_Z_AXIS) {
			ekf->Q[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_BP] = POS_EKF_Z_Q_POS_BIAS;
			ekf->P[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_BP] = 4.0f;
		} else {
			ekf->P[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_BP] = POS_EKF_P_MIN;
		}
	}

	return 1;
}

__ATTR_ITCM_TEXT
void positionEKFReset(POSITION_EKF *ekf, uint8_t axis, uint8_t keepBias) {
	if (axis <= POS_EKF_Z_AXIS) {
		int i = axis * POS_EKF_AXIS_DIM;
		ekf->axisInitialized[axis] = 0;
		ekf->rejectCount[axis] = 0;

		/* Reset state estimates */
		ekf->x[i + POS_EKF_STATE_P] = 0.0f; // Explicitly zero position now
		ekf->x[i + POS_EKF_STATE_V] = 0.0f;
		ekf->x[i + POS_EKF_STATE_BP] = 0.0f;

		if (!keepBias) {
			ekf->x[i + POS_EKF_STATE_B] = 0.0f;
		}

		/* Reset covariance block */
		for (int r = 0; r < POS_EKF_AXIS_DIM; r++) {
			for (int c = 0; c < POS_EKF_AXIS_DIM; c++) {
				ekf->P[i + r][i + c] = 0.0f;
			}
		}
		ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P] = 0.1f;
		ekf->P[i + POS_EKF_STATE_V][i + POS_EKF_STATE_V] = 0.1f;

		// If keeping bias, preserve its existing uncertainty variance
		if (!keepBias) {
			ekf->P[i + POS_EKF_STATE_B][i + POS_EKF_STATE_B] = 0.01f;
		}

		if (axis == POS_EKF_Z_AXIS) {
			ekf->P[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_BP] = 4.0f;
		} else {
			ekf->P[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_BP] = POS_EKF_P_MIN;
		}
	}
}

__ATTR_ITCM_TEXT
void positionEKFInvalidate(POSITION_EKF *ekf, uint8_t axis) {
	if (axis <= POS_EKF_Z_AXIS) {
		int i = axis * POS_EKF_AXIS_DIM;
		ekf->axisInitialized[axis] = 0;
		ekf->rejectCount[axis] = 0;
		ekf->x[i + POS_EKF_STATE_V] = 0.0f;
	}
}

__ATTR_ITCM_TEXT
void calculateDynamicProcessNoise(const POSITION_EKF *ekf, int axis, float ax, float ay, float az, float *out_q00, float *out_q11, float *out_q22, float *out_q33) {
	const int i = axis * POS_EKF_AXIS_DIM;
	const float base_q00 = ekf->Q[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P];
	const float base_q11 = ekf->Q[i + POS_EKF_STATE_V][i + POS_EKF_STATE_V];
	const float base_q22 = ekf->Q[i + POS_EKF_STATE_B][i + POS_EKF_STATE_B];
	const float base_q33 = ekf->Q[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_BP];

	float accXY = fastSqrtf(ax * ax + ay * ay);
	float accZ = fabsf(az);

	float imuStressXY = constrainToRangeF(accXY / POS_EKF_ACC_THRESH_XY, 0.0f, 1.0f);
	float imuStressZ = constrainToRangeF(accZ / POS_EKF_ACC_THRESH_Z, 0.0f, 1.0f);
	float imuStress = (imuStressXY > imuStressZ) ? imuStressXY : imuStressZ;
	float stress2 = imuStress * imuStress;

	float q00 = base_q00 * (1.0f + stress2 * POS_EKF_Q_POS_STRESS_GAIN);
	float q11 = base_q11 * (1.0f + stress2 * POS_EKF_Q_VEL_STRESS_GAIN);
	float q22 = base_q22 * (1.0f + imuStress * POS_EKF_Q_BIAS_STRESS_GAIN);
	// Position bias process noise remains static
	float q33 = base_q33;

	*out_q00 = q00;
	*out_q11 = q11;
	*out_q22 = q22;
	*out_q33 = q33;
}

__ATTR_ITCM_TEXT
void positionEKFPredict(POSITION_EKF *ekf, float ax, float ay, float az, float dt) {

	float acc[3] = { ax, ay, az };

	const float dt2 = dt * dt;
	const float hdt2 = 0.5f * dt2;

	for (int axis = 0; axis < POS_EKF_SPACE_DIM; axis++) {

		const int i = axis * POS_EKF_AXIS_DIM;

		/* =========================================================================
		 * 1. State Prediction
		 * ========================================================================= */

		float a = acc[axis] - ekf->x[i + POS_EKF_STATE_B];

		ekf->x[i + POS_EKF_STATE_P] += (ekf->x[i + POS_EKF_STATE_V] * dt) + (hdt2 * a);

		ekf->x[i + POS_EKF_STATE_V] += (a * dt);

		// Accelerometer bias and Position bias are random walks
		// x[B]  = x[B]
		// x[BP] = x[BP]

		/* =========================================================================
		 * 2. Jacobian
		 * ========================================================================= */

		const float F00 = 1.0f;
		const float F01 = dt;
		const float F02 = -hdt2;
		const float F03 = 0.0f;

		const float F10 = 0.0f;
		const float F11 = 1.0f;
		const float F12 = -dt;
		const float F13 = 0.0f;

		const float F20 = 0.0f;
		const float F21 = 0.0f;
		const float F22 = 1.0f;
		const float F23 = 0.0f;

		const float F30 = 0.0f;
		const float F31 = 0.0f;
		const float F32 = 0.0f;
		const float F33 = 1.0f;

		/* =========================================================================
		 * 3. Load Covariance Matrix
		 * ========================================================================= */

		float p00 = ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P];

		float p01 = ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_V];

		float p02 = ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_B];

		float p03 = ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_BP];

		float p11 = ekf->P[i + POS_EKF_STATE_V][i + POS_EKF_STATE_V];

		float p12 = ekf->P[i + POS_EKF_STATE_V][i + POS_EKF_STATE_B];

		float p13 = ekf->P[i + POS_EKF_STATE_V][i + POS_EKF_STATE_BP];

		float p22 = ekf->P[i + POS_EKF_STATE_B][i + POS_EKF_STATE_B];

		float p23 = ekf->P[i + POS_EKF_STATE_B][i + POS_EKF_STATE_BP];

		float p33 = ekf->P[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_BP];

		/* =========================================================================
		 * 4. M = F * P
		 * ========================================================================= */

		float m00 = F00 * p00 + F01 * p01 + F02 * p02 + F03 * p03;
		float m01 = F00 * p01 + F01 * p11 + F02 * p12 + F03 * p13;
		float m02 = F00 * p02 + F01 * p12 + F02 * p22 + F03 * p23;
		float m03 = F00 * p03 + F01 * p13 + F02 * p23 + F03 * p33;

		float m10 = F10 * p00 + F11 * p01 + F12 * p02 + F13 * p03;
		float m11 = F10 * p01 + F11 * p11 + F12 * p12 + F13 * p13;
		float m12 = F10 * p02 + F11 * p12 + F12 * p22 + F13 * p23;
		float m13 = F10 * p03 + F11 * p13 + F12 * p23 + F13 * p33;

		float m20 = F20 * p00 + F21 * p01 + F22 * p02 + F23 * p03;
		float m21 = F20 * p01 + F21 * p11 + F22 * p12 + F23 * p13;
		float m22 = F20 * p02 + F21 * p12 + F22 * p22 + F23 * p23;
		float m23 = F20 * p03 + F21 * p13 + F22 * p23 + F23 * p33;

		float m30 = F30 * p00 + F31 * p01 + F32 * p02 + F33 * p03;
		float m31 = F30 * p01 + F31 * p11 + F32 * p12 + F33 * p13;
		float m32 = F30 * p02 + F31 * p12 + F32 * p22 + F33 * p23;
		float m33 = F30 * p03 + F31 * p13 + F32 * p23 + F33 * p33;

		/* =========================================================================
		 * 5. Dynamic Process Noise
		 * ========================================================================= */

#if POS_EKF_DYNAMIC_Q_ENABLED == 1
		float q00, q11, q22, q33;
		calculateDynamicProcessNoise(ekf, axis, ax, ay, az, &q00, &q11, &q22, &q33);
#else
		float q00 = ekf->Q[i + POS_EKF_STATE_P] [i + POS_EKF_STATE_P];
		float q11 = ekf->Q[i + POS_EKF_STATE_V] [i + POS_EKF_STATE_V];
		float q22 = ekf->Q[i + POS_EKF_STATE_B] [i + POS_EKF_STATE_B];
		float q33 = ekf->Q[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_BP];
#endif

		/* =========================================================================
		 * 6. P = M * F^T + Q
		 * ========================================================================= */

		float P00 = m00 * F00 + m01 * F01 + m02 * F02 + m03 * F03 + q00;
		float P01 = m00 * F10 + m01 * F11 + m02 * F12 + m03 * F13;
		float P02 = m00 * F20 + m01 * F21 + m02 * F22 + m03 * F23;
		float P03 = m00 * F30 + m01 * F31 + m02 * F32 + m03 * F33;

		float P11 = m10 * F10 + m11 * F11 + m12 * F12 + m13 * F13 + q11;
		float P12 = m10 * F20 + m11 * F21 + m12 * F22 + m13 * F23;
		float P13 = m10 * F30 + m11 * F31 + m12 * F32 + m13 * F33;

		float P22 = m20 * F20 + m21 * F21 + m22 * F22 + m23 * F23 + q22;
		float P23 = m20 * F30 + m21 * F31 + m22 * F32 + m23 * F33;

		float P33 = m30 * F30 + m31 * F31 + m32 * F32 + m33 * F33 + q33;

		/* =========================================================================
		 * 7. Bound Protection
		 * ========================================================================= */

		ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P] = constrainToRangeF(P00, POS_EKF_P_MIN, POS_EKF_P_MAX);
		ekf->P[i + POS_EKF_STATE_V][i + POS_EKF_STATE_V] = constrainToRangeF(P11, POS_EKF_P_MIN, POS_EKF_P_MAX);
		ekf->P[i + POS_EKF_STATE_B][i + POS_EKF_STATE_B] = constrainToRangeF(P22, POS_EKF_P_MIN, POS_EKF_P_MAX);
		ekf->P[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_BP] = constrainToRangeF(P33, POS_EKF_P_MIN, POS_EKF_P_MAX);

		/* =========================================================================
		 * 8. Symmetric Covariance Mapping
		 * ========================================================================= */

		ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_V] = P01;
		ekf->P[i + POS_EKF_STATE_V][i + POS_EKF_STATE_P] = P01;

		ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_B] = P02;
		ekf->P[i + POS_EKF_STATE_B][i + POS_EKF_STATE_P] = P02;

		ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_BP] = P03;
		ekf->P[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_P] = P03;

		ekf->P[i + POS_EKF_STATE_V][i + POS_EKF_STATE_B] = P12;
		ekf->P[i + POS_EKF_STATE_B][i + POS_EKF_STATE_V] = P12;

		ekf->P[i + POS_EKF_STATE_V][i + POS_EKF_STATE_BP] = P13;
		ekf->P[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_V] = P13;

		ekf->P[i + POS_EKF_STATE_B][i + POS_EKF_STATE_BP] = P23;
		ekf->P[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_B] = P23;
	}
}

__ATTR_ITCM_TEXT
void positionEKFMeasurementUpdate(POSITION_EKF *ekf, uint8_t axis, float meas, float rValue, const float H[4]) {
	const int i = axis * POS_EKF_AXIS_DIM;
	if (!ekf->axisInitialized[axis]) {
		if (H[POS_EKF_STATE_P] > 0.5f) {
			ekf->x[i + POS_EKF_STATE_P] = meas;
		}
		ekf->x[i + POS_EKF_STATE_V] = 0.0f;
		ekf->x[i + POS_EKF_STATE_B] = 0.0f;
		ekf->x[i + POS_EKF_STATE_BP] = 0.0f;

		ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P] = 2.0f;
		ekf->P[i + POS_EKF_STATE_V][i + POS_EKF_STATE_V] = 1.0f;
		ekf->P[i + POS_EKF_STATE_B][i + POS_EKF_STATE_B] = 0.1f;
		if (axis == POS_EKF_Z_AXIS) {
			ekf->P[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_BP] = 4.0f;
		} else {
			ekf->P[i + POS_EKF_STATE_BP][i + POS_EKF_STATE_BP] = POS_EKF_P_MIN;
		}

		ekf->rejectCount[axis] = 0;
		ekf->axisInitialized[axis] = 1;
		return;
	}

	/* ============================================================
	 * 1. Innovation
	 * ============================================================ */
	float pred = H[0] * ekf->x[i + 0] + H[1] * ekf->x[i + 1] + H[2] * ekf->x[i + 2] + H[3] * ekf->x[i + 3];
	float y = meas - pred;
	ekf->innovation[axis] = y;

	/* ============================================================
	 * 2. Load covariance
	 * ============================================================ */

	float P[4][4];

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			P[r][c] = ekf->P[i + r][i + c];
		}
	}

	/* ============================================================
	 * 3. Innovation covariance
	 * S = HPH' + R
	 * ============================================================ */

	float HP[4];

	for (int c = 0; c < 4; c++) {
		HP[c] = H[0] * P[0][c] + H[1] * P[1][c] + H[2] * P[2][c] + H[3] * P[3][c];
	}

	float S = HP[0] * H[0] + HP[1] * H[1] + HP[2] * H[2] + HP[3] * H[3] + rValue;

	if (S < POS_EKF_P_MIN) {
		S = POS_EKF_P_MIN;
	}

	/* ============================================================
	 * 4. Gating
	 * ============================================================ */
	float d2 = (y * y) / S;
	if (d2 > ekf->gateSize[axis]) {
		ekf->rejectCount[axis]++;
		if (ekf->rejectCount[axis] < ekf->panicLimit[axis]) {
			return;
		}
		/* PANIC: filter has diverged from the sensor. Inflate covariance so the
		 * gate reopens and the next measurements pull the state back, instead of
		 * rejecting forever and flying blind on inertial prediction. */
		ekf->rejectCount[axis] = 0;
		for (int r = 0; r < POS_EKF_AXIS_DIM; r++) {
			ekf->P[i + r][i + r] = constrainToRangeF(ekf->P[i + r][i + r] * POS_EKF_PANIC_P_INFLATE,
			POS_EKF_P_MIN, POS_EKF_P_MAX);
		}
		return;  // next measurement will pass the (now wider) gate
	}
	ekf->rejectCount[axis] = 0;

	/* ============================================================
	 * 5. Kalman Gain
	 * K = PH'/S
	 * ============================================================ */

	float S_inv = 1.0f / S;
	float K[4];
	for (int r = 0; r < 4; r++) {
		float PHt = P[r][0] * H[0] + P[r][1] * H[1] + P[r][2] * H[2] + P[r][3] * H[3];
		K[r] = PHt * S_inv;
	}

	/* ============================================================
	 * 6. State Update
	 * ============================================================ */

	for (int r = 0; r < 4; r++) {
		ekf->x[i + r] += K[r] * y;
	}

	/* ============================================================
	 * 7. W = I - K H
	 * ============================================================ */

	float W[4][4];
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			W[r][c] = -K[r] * H[c];
		}
		W[r][r] += 1.0f;
	}

	/* ============================================================
	 * 8. M = W P
	 * ============================================================ */

	float M[4][4];
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			M[r][c] = 0.0f;
			for (int k = 0; k < 4; k++) {
				M[r][c] += W[r][k] * P[k][c];
			}
		}
	}

	/* ============================================================
	 * 9. N = M W'
	 * ============================================================ */

	float N[4][4];
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			N[r][c] = 0.0f;
			for (int k = 0; k < 4; k++) {
				N[r][c] += M[r][k] * W[c][k];
			}
		}
	}

	/* ============================================================
	 * 10. Joseph Form
	 * P = N + K R K'
	 * ============================================================ */

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			float Pnew = N[r][c] + K[r] * K[c] * rValue;
			if (r == c) {
				Pnew = constrainToRangeF(Pnew, POS_EKF_P_MIN, POS_EKF_P_MAX);
			}
			ekf->P[i + r][i + c] = Pnew;
		}
	}

	/* ============================================================
	 * 11. Force covariance symmetry
	 * ============================================================ */

	for (int r = 0; r < 4; r++) {
		for (int c = r + 1; c < 4; c++) {
			float sym = 0.5f * (ekf->P[i + r][i + c] + ekf->P[i + c][i + r]);

			ekf->P[i + r][i + c] = sym;
			ekf->P[i + c][i + r] = sym;
		}
	}
}
