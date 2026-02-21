#include "IMU.h"

#if IMU_FILTER_SELECTED == IMU_FILTER_MANHONY_BF
#include "../sensors/attitude/AttitudeSensor.h"
#include "MahonyFilter.h"

float __ATTR_DTCM_BSS mahonyFilter_BF_KP = MAHONY_FILTER_KP;
float __ATTR_DTCM_BSS mahonyFilter_BF_KI = MAHONY_FILTER_KI;
float __ATTR_DTCM_BSS mahonyFilter_MaxSpinRateRad = 0;

// Integral error terms
float __ATTR_DTCM_BSS mahonyFilter_BF_IBx = 0.0f;
float __ATTR_DTCM_BSS mahonyFilter_BF_IBy = 0.0f;
float __ATTR_DTCM_BSS mahonyFilter_BF_IBz = 0.0f;

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
	imuData.roll = convertRadToDeg(atan2Approx(imuData.rMatrix[2][1], imuData.rMatrix[2][2]));
	// Pitch: asin(-R20)
	float r20 = -imuData.rMatrix[2][0];
	if (r20 > 1.0f)  {r20 = 1.0f;}
	if (r20 < -1.0f) {r20 = -1.0f;}
	imuData.pitch = convertRadToDeg(asinApproxFast(r20));
	// Yaw: -atan2(R10, R00)
	imuData.yaw = convertRadToDeg(-atan2Approx(imuData.rMatrix[1][0], imuData.rMatrix[0][0]));
}

/**
 * Updates compass heading based on magnetic inclination and external error offsets
 */
__ATTR_ITCM_TEXT
void imuFilterUpdateHeading(float magIncl, float error) {
	imuData.heading = -imuData.yaw;
	// Normalize to [0, 360]
	if (imuData.heading < 0.0f) {
		imuData.heading += 360.0f;
	}
	if (imuData.heading > 360.0f) {
		imuData.heading -= 360.0f;
	}

	imuData.heading += (magIncl + error);

	// Final Wrap-around
	if (imuData.heading > 360.0f) {
		imuData.heading -= 360.0f;
	}
	if (imuData.heading < 0.0f) {
		imuData.heading += 360.0f;
	}
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
	gx = convertDegToRad(sensorAttitudeData.gxDSFiltered);
	gy = convertDegToRad(sensorAttitudeData.gyDSFiltered);
	gz = convertDegToRad(sensorAttitudeData.gzDSFiltered);
	ax = sensorAttitudeData.axGFiltered;
	ay = sensorAttitudeData.ayGFiltered;
	az = sensorAttitudeData.azGFiltered;
	mx = sensorAttitudeData.mxFiltered;
	my = sensorAttitudeData.myFiltered;
	mz = sensorAttitudeData.mzFiltered;
	halfDt = 0.5f * dt;
	// 2. Magnetometer Correction (Heading Only)
	float magSq = (mx * mx) + (my * my) + (mz * mz);
	if (magSq > MAHONY_FILTER_MIN_MAG_MAGNITUDE) {
		recipNorm = fastInvSqrtf(magSq);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;
		// Project measured mag field into Earth Frame (EF)
		float hx = imuData.rMatrix[0][0] * mx + imuData.rMatrix[0][1] * my + imuData.rMatrix[0][2] * mz;
		float hy = imuData.rMatrix[1][0] * mx + imuData.rMatrix[1][1] * my + imuData.rMatrix[1][2] * mz;
		// Calculate EF reference magnitude
		float bx = fastSqrtf(hx * hx + hy * hy);
		// Calculate Earth-Frame Yaw error (Planar Assumption)
		float ez_ef = -(hy * bx);
		// Project EF error back to Body Frame using Matrix Column 2 (Transpose rotation)
		// Corrects yaw while maintaining roll/pitch stability during tilt
		ex += imuData.rMatrix[0][2] * ez_ef;
		ey += imuData.rMatrix[1][2] * ez_ef;
		ez += imuData.rMatrix[2][2] * ez_ef;
	}
	// 3. Accelerometer Correction (Tilt/Horizon)
	float accSq = (ax * ax) + (ay * ay) + (az * az);
	if (accSq > MAHONY_FILTER_MIN_ACC_MAGNITUDE) {
		recipNorm = fastInvSqrtf(accSq);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		// Cross product: Measured Gravity (ax,ay,az) x Estimated Gravity (DCM Row 3)
		ex += ((ay * imuData.rMatrix[2][2]) - (az * imuData.rMatrix[2][1]));
		ey += ((az * imuData.rMatrix[2][0]) - (ax * imuData.rMatrix[2][2]));
		ez += ((ax * imuData.rMatrix[2][1]) - (ay * imuData.rMatrix[2][0]));
	}
	// 4. Error Integration (Integral Feedback for Gyro Bias)
	if (mahonyFilter_BF_KI > 0.0f) {
		spin_rate = fastSqrtf((gx * gx) + (gy * gy) + (gz * gz));
		// Anti-windup: Stop integration during high-rate spins
		if (spin_rate <= mahonyFilter_MaxSpinRateRad) {
			mahonyFilter_BF_IBx += (mahonyFilter_BF_KI * ex * dt);
			mahonyFilter_BF_IBy += (mahonyFilter_BF_KI * ey * dt);
			mahonyFilter_BF_IBz += (mahonyFilter_BF_KI * ez * dt);
		}
	} else {
		mahonyFilter_BF_IBx = 0;
		mahonyFilter_BF_IBy = 0;
		mahonyFilter_BF_IBz = 0;
	}
	// 5. Apply Feedback to Gyro Rates
	gx += (mahonyFilter_BF_KP * ex) + mahonyFilter_BF_IBx;
	gy += (mahonyFilter_BF_KP * ey) + mahonyFilter_BF_IBy;
	gz += (mahonyFilter_BF_KP * ez) + mahonyFilter_BF_IBz;
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
		mahonyFilter_BF_KP = MAHONY_FILTER_STABILIZE_KP;
		mahonyFilter_BF_KI = MAHONY_FILTER_STABILIZE_KI;
	} else {
		mahonyFilter_BF_KP = MAHONY_FILTER_KP;
		mahonyFilter_BF_KI = MAHONY_FILTER_KI;
	}
}

uint8_t imuFilterInit(uint8_t stabilize) {
	imuFilterSetMode(stabilize);
	imuFilterReset();
	mahonyFilterUpdateRotationMatrix();
	mahonyFilter_MaxSpinRateRad = convertDegToRad(MAHONY_FILTER_SPIN_RATE_LIMIT);
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
