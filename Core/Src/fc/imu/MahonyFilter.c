#include "IMU.h"

#if IMU_FILTER_SELECTED == IMU_FILTER_MANHONY
#include "../sensors/attitude/AttitudeSensor.h"
#include "MahonyFilter.h"
#include "../logger/Logger.h"
#include <stdio.h>

float __ATTR_DTCM_BSS mahonyFilterKP = MAHONY_FILTER_KP;
float __ATTR_DTCM_BSS mahonyFilterKI = MAHONY_FILTER_KI;
float __ATTR_DTCM_BSS mahonyFilterMaxSpinRateRad = 0;

// Integral error terms
float __ATTR_DTCM_BSS mahonyFilterIBx = 0.0f;
float __ATTR_DTCM_BSS mahonyFilterIBy = 0.0f;
float __ATTR_DTCM_BSS mahonyFilterIBz = 0.0f;
float __ATTR_DTCM_BSS mahonyFilterMagGainRatio = MAHONY_FILTER_MAG_GAIN_RATIO;
float __ATTR_DTCM_BSS mahonyFilterMagRefNorm = 0.0f;   // 0 = unlearned; seeds on first sample
uint8_t __ATTR_DTCM_BSS mahonyFilterStabilizeMode = 1;

uint16_t imuFilterGetStabilizationCount() {
	return MAHONY_FILTER_STAB_COUNT;
}

/**
 * Extract Euler Angles (Roll, Pitch, Yaw) from the Rotation Matrix
 * Uses fast trigonometric approximations for high loop frequency.
 */
__ATTR_ITCM_TEXT
void imuFilterUpdateAngles(void) {
	// Roll: atan2(R21, R22)
	imuData.roll = convertRadToDegF(atan2Approx(imuData.rMatrix[2][1], imuData.rMatrix[2][2]));
	// Pitch: asin(-R20)
	float r20 = -imuData.rMatrix[2][0];
	if (r20 > 1.0f) {
		r20 = 1.0f;
	}
	if (r20 < -1.0f) {
		r20 = -1.0f;
	}
	imuData.pitch = convertRadToDegF(asinApproxFast(r20));
	// Yaw: -atan2(R10, R00)
	imuData.yaw = convertRadToDegF(-atan2Approx(imuData.rMatrix[1][0], imuData.rMatrix[0][0]));
}

/**
 * Updates compass heading based on magnetic inclination and external error offsets
 */
__ATTR_ITCM_TEXT
void imuFilterUpdateHeading() {
	float heading = imuData.yaw;
	// Normalize to [0, 360]
	if (heading < 0.0f) {
		heading += 360.0f;
	}
	if (heading > 360.0f) {
		heading -= 360.0f;
	}
	imuData.heading = heading;
}

/**
 * Build 3x3 Direction Cosine Matrix (DCM) from current Quaternion
 */
__ATTR_ITCM_TEXT
void mahonyFilterUpdateRotationMatrix(void) {
	float q0 = imuData.q0, q1 = imuData.q1, q2 = imuData.q2, q3 = imuData.q3;
	float q1_sq = q1 * q1, q2_sq = q2 * q2, q3_sq = q3 * q3;
	float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
	float q1q2 = q1 * q2, q1q3 = q1 * q3, q2q3 = q2 * q3;
	// Row 1: Earth-X (North) in Body Frame
	imuData.rMatrix[0][0] = 1.0f - 2.0f * (q2_sq + q3_sq);
	imuData.rMatrix[0][1] = 2.0f * (q1q2 - q0q3);
	imuData.rMatrix[0][2] = 2.0f * (q1q3 + q0q2);
	// Row 2: Earth-Y (East) in Body Frame
	imuData.rMatrix[1][0] = 2.0f * (q1q2 + q0q3);
	imuData.rMatrix[1][1] = 1.0f - 2.0f * (q1_sq + q3_sq);
	imuData.rMatrix[1][2] = 2.0f * (q2q3 - q0q1);
	// Row 3: Earth-Z (Down) in Body Frame
	imuData.rMatrix[2][0] = 2.0f * (q1q3 - q0q2);
	imuData.rMatrix[2][1] = 2.0f * (q2q3 + q0q1);
	imuData.rMatrix[2][2] = 1.0f - 2.0f * (q1_sq + q2_sq);
}

/**
 * Core Mahony Filter Update Loop
 * Performs sensor fusion of Gyro, Accel, and Mag
 */
__ATTR_ITCM_TEXT
void imuFilterUpdate(float dt) {
	float gx, gy, gz, ax, ay, az, mx, my, mz;
	float halfDt, ex = 0, ey = 0, ez = 0;
	float recipNorm, spin_rate;
	// 1. Prepare sensor data
	gx = convertDegToRadF(sensorAttitudeData.gxDSFiltered);
	gy = convertDegToRadF(sensorAttitudeData.gyDSFiltered);
	gz = convertDegToRadF(sensorAttitudeData.gzDSFiltered);
	ax = sensorAttitudeData.axGFilteredImu;
	ay = sensorAttitudeData.ayGFilteredImu;
	az = sensorAttitudeData.azGFilteredImu;
	mx = sensorAttitudeData.mxFiltered;
	my = sensorAttitudeData.myFiltered;
	mz = sensorAttitudeData.mzFiltered;
	halfDt = 0.5f * dt;

	// 2. Magnetometer Correction (Heading Only) — norm-gated, slow channel
	float magSq = (mx * mx) + (my * my) + (mz * mz);
	if (magSq > MAHONY_FILTER_MIN_MAG_MAGNITUDE) {
		recipNorm = fastInvSqrtf(magSq);
		float magNorm = magSq * recipNorm;              // = sqrt(magSq), reuses invsqrt
		/* ---- [C] Reference norm: seed once, then learn slowly when clean ---- */
		if (mahonyFilterMagRefNorm <= 0.0f) {
			mahonyFilterMagRefNorm = magNorm;          // first valid sample seeds it
		}
		/* ---- [A] Trust weight from norm deviation (distortion detector) ---- */
		if (mahonyFilterStabilizeMode) {
			/* Ground stabilization: motors off, field is trustworthy.
			 * Track the norm directly (fast) so the ref is valid before flight. */
			if (mahonyFilterMagRefNorm <= 0.0f) {
				mahonyFilterMagRefNorm = magNorm;
			} else {
				mahonyFilterMagRefNorm += 0.01f * (magNorm - mahonyFilterMagRefNorm); // tau ~ 31ms at 3.2k
			}
		}
		float normDev = fabsf(magNorm - mahonyFilterMagRefNorm) / mahonyFilterMagRefNorm;
		float magW = 1.0f;
		if (normDev > MAHONY_FILTER_MAG_NORM_GATE_START) {
			magW = 1.0f - ((normDev - MAHONY_FILTER_MAG_NORM_GATE_START) * MAHONY_FILTER_MAG_NORM_GATE_INV_W);
			if (magW < 0.0f)
				magW = 0.0f;
		} else if (!mahonyFilterStabilizeMode) {
			/* In flight and clean: slow environmental tracking only */
			float alphaRef = dt / (MAHONY_FILTER_MAG_REF_LEARN_TAU + dt);
			mahonyFilterMagRefNorm += alphaRef * (magNorm - mahonyFilterMagRefNorm);
		}
		if (magW > 0.0f) {
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;
			// Project measured mag field into Earth Frame (EF)
			float hx = imuData.rMatrix[0][0] * mx + imuData.rMatrix[0][1] * my + imuData.rMatrix[0][2] * mz;
			float hy = imuData.rMatrix[1][0] * mx + imuData.rMatrix[1][1] * my + imuData.rMatrix[1][2] * mz;
			// Calculate EF reference magnitude
			float bx = fastSqrtf(hx * hx + hy * hy);
			// Calculate Earth-Frame Yaw error (Planar Assumption)
			// [B] scaled by trust weight and the slow mag-channel ratio
			float ez_ef = -(hy * bx) * magW * mahonyFilterMagGainRatio;
			// [D] Project EF yaw error to Body Frame: earth-Z in body frame = row 2
			ex += imuData.rMatrix[2][0] * ez_ef;
			ey += imuData.rMatrix[2][1] * ez_ef;
			ez += imuData.rMatrix[2][2] * ez_ef;
		}
	}

	// 3. Accelerometer Correction (Tilt/Horizon) — norm-gated (unchanged)
	float accSq = (ax * ax) + (ay * ay) + (az * az);
	if ((accSq > MAHONY_FILTER_ACC_GATE_MIN_SQ) && (accSq < MAHONY_FILTER_ACC_GATE_MAX_SQ)) {
		recipNorm = fastInvSqrtf(accSq);
		float accNorm = accSq * recipNorm;   // = sqrt(accSq), reuses invsqrt
		// Trapezoid weight: 1.0 inside full-trust band, linear fade to hard edges
		float w = 1.0f;
		if (accNorm < MAHONY_FILTER_ACC_GATE_FULL_LO) {
			w = (accNorm - MAHONY_FILTER_ACC_GATE_MIN) * MAHONY_FILTER_ACC_GATE_INV_W_LO;
			if (w < 0.0f)
				w = 0.0f;
		} else if (accNorm > MAHONY_FILTER_ACC_GATE_FULL_HI) {
			w = (MAHONY_FILTER_ACC_GATE_MAX - accNorm) * MAHONY_FILTER_ACC_GATE_INV_W_HI;
			if (w < 0.0f)
				w = 0.0f;
		}
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		ex += w * ((ay * imuData.rMatrix[2][2]) - (az * imuData.rMatrix[2][1]));
		ey += w * ((az * imuData.rMatrix[2][0]) - (ax * imuData.rMatrix[2][2]));
		ez += w * ((ax * imuData.rMatrix[2][1]) - (ay * imuData.rMatrix[2][0]));
	}

	// 4. Error Integration (Integral Feedback for Gyro Bias) — unchanged
	// (mag portion of ex/ey/ez is already gated & scaled, so the bias
	//  integrator is automatically protected from distortion)
	if (mahonyFilterKI > 0.0f) {
		spin_rate = fastSqrtf((gx * gx) + (gy * gy) + (gz * gz));
		// Anti-windup: Stop integration during high-rate spins
		if (spin_rate <= mahonyFilterMaxSpinRateRad) {
			mahonyFilterIBx += (mahonyFilterKI * ex * dt);
			mahonyFilterIBy += (mahonyFilterKI * ey * dt);
			mahonyFilterIBz += (mahonyFilterKI * ez * dt);
		}
	} else {
		mahonyFilterIBx = 0;
		mahonyFilterIBy = 0;
		mahonyFilterIBz = 0;
	}
	// 5. Apply Feedback to Gyro Rates
	gx += (mahonyFilterKP * ex) + mahonyFilterIBx;
	gy += (mahonyFilterKP * ey) + mahonyFilterIBy;
	gz += (mahonyFilterKP * ez) + mahonyFilterIBz;
	// 6. Quaternion Integration (Atomic Update)
	gx *= halfDt;
	gy *= halfDt;
	gz *= halfDt;
	float q0_old = imuData.q0, q1_old = imuData.q1, q2_old = imuData.q2, q3_old = imuData.q3;
	imuData.q0 += (-(q1_old * gx) - (q2_old * gy) - (q3_old * gz));
	imuData.q1 += ((q0_old * gx) + (q2_old * gz) - (q3_old * gy));
	imuData.q2 += ((q0_old * gy) - (q1_old * gz) + (q3_old * gx));
	imuData.q3 += ((q0_old * gz) + (q1_old * gy) - (q2_old * gx));
	// 7. Re-normalize Quaternion
	float qSq = (imuData.q0 * imuData.q0) + (imuData.q1 * imuData.q1) + (imuData.q2 * imuData.q2) + (imuData.q3 * imuData.q3);
	recipNorm = fastInvSqrtf(qSq);
	imuData.q0 *= recipNorm;
	imuData.q1 *= recipNorm;
	imuData.q2 *= recipNorm;
	imuData.q3 *= recipNorm;
	// 8. Finalize state
	mahonyFilterUpdateRotationMatrix();
}

/* --- Initialization and Utility Functions --- */
void imuFilterSetMode(uint8_t stabilize) {
	if (stabilize) {
		mahonyFilterKP = MAHONY_FILTER_STABILIZE_KP;
		mahonyFilterKI = MAHONY_FILTER_STABILIZE_KI;
		mahonyFilterMagGainRatio = MAHONY_FILTER_STABILIZE_MAG_GAIN_RATIO;
	} else {
		mahonyFilterKP = MAHONY_FILTER_KP;
		mahonyFilterKI = MAHONY_FILTER_KI;
		mahonyFilterMagGainRatio = MAHONY_FILTER_MAG_GAIN_RATIO;
		mahonyFilterIBx = 0.0f;
		mahonyFilterIBy = 0.0f;
		mahonyFilterIBz = 0.0f;
	}
	mahonyFilterStabilizeMode = stabilize;
}

uint8_t imuFilterInit(uint8_t stabilize) {
	imuFilterSetMode(stabilize);
	imuFilterReset();
	mahonyFilterUpdateRotationMatrix();
	mahonyFilterMaxSpinRateRad = convertDegToRadF(MAHONY_FILTER_SPIN_RATE_LIMIT);
	return 1;
}

void mahonyFilterResetRotationMatrix() {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			imuData.rMatrix[i][j] = (i == j) ? 1.0f : 0.0f;
		}
	}
}

void imuFilterReset() {
	imuData.q0 = 1.0f;
	imuData.q1 = 0.0f;
	imuData.q2 = 0.0f;
	imuData.q3 = 0.0f;
	mahonyFilterResetRotationMatrix();
}
#endif
