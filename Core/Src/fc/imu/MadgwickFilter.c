#include "IMU.h"

#if IMU_FILTER_SELECTED == IMU_FILTER_MADGWICK

#include "esp_attr.h"
#include "madgwickFilter.h"
 
DRAM_ATTR float madgwickFilterBeta = 0.06f; // 0.110f;
void madgwickMARGUpdate(float dt);
void madgwickARGUpdate(float dt);
void madgwickUpdateRotationMatrix(void); 
void madgwickResetRotationMatrix(void);

/** 
 * Initializes the Madgwick Filter
 */
uint8_t imuFilterInit(uint8_t stabilize) {
	imuFilterSetMode(stabilize);
	imuFilterReset();
	madgwickUpdateRotationMatrix();
	return 1;
}

/**
 * Sets the filter mode
 */
void imuFilterSetMode(uint8_t stabilize) {
	if (stabilize) {
		madgwickFilterBeta = MADGWICK_FILTER_STAB_BETA;
	} else {
		madgwickFilterBeta = MADGWICK_FILTER_BETA;
	}
}

IRAM_ATTR void imuFilterUpdate(float dt) {
	madgwickMARGUpdate(dt);
	madgwickUpdateRotationMatrix();
}

/**
 * Resets the Madgwick filter state
 */
void imuFilterReset() {
	imuData.q0 = 1.0f;
	imuData.q1 = 0.0f;
	imuData.q2 = 0.0f;
	imuData.q3 = 0.0f;
	madgwickResetRotationMatrix();
}

uint16_t imuFilterGetStabilizationCount() { return MADGWICK_FILTER_STAB_COUNT; }

IRAM_ATTR void imuFilterUpdateAngles() {
#if MADGWICK_FILTER_USE_TRIG_APPROX == 1
	imuData.roll = convertRadToDeg(atan2Approx(imuData.rMatrix[2][1], imuData.rMatrix[2][2]));
	imuData.pitch = convertRadToDeg(HalfPI - acosApprox(-imuData.rMatrix[2][0]));
	imuData.yaw = convertRadToDeg(-atan2Approx(imuData.rMatrix[1][0], imuData.rMatrix[0][0]));
#else
	imuData.roll = convertRadToDeg(atan2f(imuData.rMatrix[2][1], imuData.rMatrix[2][2]));
	imuData.pitch = convertRadToDeg(HalfPI - acosf(-imuData.rMatrix[2][0]));
	imuData.yaw = convertRadToDeg(-atan2f(imuData.rMatrix[1][0], imuData.rMatrix[0][0]));
#endif
}

/**
 * Converts yaw to heading
 */
IRAM_ATTR void imuFilterUpdateHeading(float magIncl, float error) {
	imuData.heading = -imuData.yaw;
	if (imuData.heading < 0.0f) {
		imuData.heading += 360.0f;
	} else if (imuData.heading > 360.0f) {
		imuData.heading -= 360.0f;
	}
	imuData.heading += (magIncl + error);
	if (imuData.heading > 360) {
		imuData.heading -= 360;
	} else if (imuData.heading < 0) {
		imuData.heading += 360;
	}
}

IRAM_ATTR void madgwickUpdateRotationMatrix() {
	static float q0, q1, q2, q3, q1_sq, q2_sq, q3_sq, q0q1, q0q2, q0q3, q1q2, q1q3, q2q3;
	// Access quaternion components directly using the dot operator
	q0 = imuData.q0;
	q1 = imuData.q1;
	q2 = imuData.q2;
	q3 = imuData.q3;

	// Pre-calculate squares and products for efficiency
	// float q0_sq = q0 * q0;
	q1_sq = q1 * q1;
	q2_sq = q2 * q2;
	q3_sq = q3 * q3;

	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q3 = q2 * q3;

	// Build and store the rotation matrix directly into the local copy
	// Row 1
	imuData.rMatrix[0][0] = 1.0f - 2.0f * (q2_sq + q3_sq);
	imuData.rMatrix[0][1] = 2.0f * (q1q2 - q0q3);
	imuData.rMatrix[0][2] = 2.0f * (q1q3 + q0q2);

	// Row 2
	imuData.rMatrix[1][0] = 2.0f * (q1q2 + q0q3);
	imuData.rMatrix[1][1] = 1.0f - 2.0f * (q1_sq + q3_sq);
	imuData.rMatrix[1][2] = 2.0f * (q2q3 - q0q1);

	// Row 3
	imuData.rMatrix[2][0] = 2.0f * (q1q3 - q0q2);
	imuData.rMatrix[2][1] = 2.0f * (q2q3 + q0q1);
	imuData.rMatrix[2][2] = 1.0f - 2.0f * (q1_sq + q2_sq);
}

void madgwickResetRotationMatrix() {
	// Reset all elements to 0
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			imuData.rMatrix[i][j] = 0.0f;
		}
	}
	// Set the diagonal elements to 1
	imuData.rMatrix[0][0] = 1.0f;
	imuData.rMatrix[1][1] = 1.0f;
	imuData.rMatrix[2][2] = 1.0f;
}

IRAM_ATTR void madgwickMARGUpdate(float dt) {
	static float recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4, hx, hy, _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
		_2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3, gx, gy, gz, ax, ay, az, mx, my, mz;

	// Copy the variables locally
	gx = convertDegToRad(sensorAttitudeData.gxDSFiltered);
	gy = convertDegToRad(sensorAttitudeData.gyDSFiltered);
	gz = convertDegToRad(sensorAttitudeData.gzDSFiltered);
	ax = sensorAttitudeData.axGFiltered;
	ay = sensorAttitudeData.ayGFiltered;
	az = sensorAttitudeData.azGFiltered; 
	mx = sensorAttitudeData.mxFiltered;
	my = sensorAttitudeData.myFiltered;
	mz = sensorAttitudeData.mzFiltered;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalization)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		madgwickARGUpdate(dt);
	} else {
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-imuData.q1 * gx - imuData.q2 * gy - imuData.q3 * gz);
		qDot2 = 0.5f * (imuData.q0 * gx + imuData.q2 * gz - imuData.q3 * gy);
		qDot3 = 0.5f * (imuData.q0 * gy - imuData.q1 * gz + imuData.q3 * gx);
		qDot4 = 0.5f * (imuData.q0 * gz + imuData.q1 * gy - imuData.q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
		if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalize accelerometer measurement
#if MADGWICK_FILTER_USE_INV_SQRT_APPROX == 1
			recipNorm = fastInvSqrtf(power2f(ax) + power2f(ay) + power2f(az));
#else
			recipNorm = invSqrtf(power2f(ax) + power2f(ay) + power2f(az));
#endif
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Normalize magnetometer measurement
#if MADGWICK_FILTER_USE_INV_SQRT_APPROX == 1
			recipNorm = fastInvSqrtf(power2f(mx) + power2f(my) + power2f(mz));
#else
			recipNorm = invSqrtf(power2f(mx) + power2f(my) + power2f(mz));
#endif
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0f * imuData.q0 * mx;
			_2q0my = 2.0f * imuData.q0 * my;
			_2q0mz = 2.0f * imuData.q0 * mz;
			_2q1mx = 2.0f * imuData.q1 * mx;
			_2q0 = 2.0f * imuData.q0;
			_2q1 = 2.0f * imuData.q1;
			_2q2 = 2.0f * imuData.q2;
			_2q3 = 2.0f * imuData.q3;
			_2q0q2 = 2.0f * imuData.q0 * imuData.q2;
			_2q2q3 = 2.0f * imuData.q2 * imuData.q3;

			q0q0 = imuData.q0 * imuData.q0;
			q0q1 = imuData.q0 * imuData.q1;
			q0q2 = imuData.q0 * imuData.q2;
			q0q3 = imuData.q0 * imuData.q3;
			q1q1 = imuData.q1 * imuData.q1;
			q1q2 = imuData.q1 * imuData.q2;
			q1q3 = imuData.q1 * imuData.q3;
			q2q2 = imuData.q2 * imuData.q2;
			q2q3 = imuData.q2 * imuData.q3;
			q3q3 = imuData.q3 * imuData.q3;

			// Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * imuData.q3 + _2q0mz * imuData.q2 + mx * q1q1 + _2q1 * my * imuData.q2 + _2q1 * mz * imuData.q3 - mx * q2q2 -
				 mx * q3q3;
			hy = _2q0mx * imuData.q3 + my * q0q0 - _2q0mz * imuData.q1 + _2q1mx * imuData.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * imuData.q3 -
				 my * q3q3;

#if MADGWICK_FILTER_USE_SQRT_APPROX == 1
			_2bx = fastSqrtf(hx * hx + hy * hy);
#else
			_2bx = sqrtf(hx * hx + hy * hy);
#endif
			_2bz = -_2q0mx * imuData.q2 + _2q0my * imuData.q1 + mz * q0q0 + _2q1mx * imuData.q3 - mz * q1q1 + _2q2 * my * imuData.q3 - mz * q2q2 +
				   mz * q3q3;
			_4bx = 2.0f * _2bx;
			_4bz = 2.0f * _2bz;

			// Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
				 _2bz * imuData.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
				 (-_2bx * imuData.q3 + _2bz * imuData.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
				 _2bx * imuData.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * imuData.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
				 _2bz * imuData.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
				 (_2bx * imuData.q2 + _2bz * imuData.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
				 (_2bx * imuData.q3 - _4bz * imuData.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * imuData.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
				 (-_4bx * imuData.q2 - _2bz * imuData.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
				 (_2bx * imuData.q1 + _2bz * imuData.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
				 (_2bx * imuData.q0 - _4bz * imuData.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
				 (-_4bx * imuData.q3 + _2bz * imuData.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
				 (-_2bx * imuData.q0 + _2bz * imuData.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
				 _2bx * imuData.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

// normalize step magnitude
#if MADGWICK_FILTER_USE_INV_SQRT_APPROX == 1
			recipNorm = fastInvSqrtf(power2f(s0) + power2f(s1) + power2f(s2) + power2f(s3));
#else
			recipNorm = invSqrtf(power2f(s0) + power2f(s1) + power2f(s2) + power2f(s3));
#endif
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= madgwickFilterBeta * s0;
			qDot2 -= madgwickFilterBeta * s1;
			qDot3 -= madgwickFilterBeta * s2;
			qDot4 -= madgwickFilterBeta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		imuData.q0 += qDot1 * dt;
		imuData.q1 += qDot2 * dt;
		imuData.q2 += qDot3 * dt;
		imuData.q3 += qDot4 * dt;

		// Normalize quaternion
#if MADGWICK_FILTER_USE_INV_SQRT_APPROX == 1
		recipNorm = fastInvSqrtf(power2f(imuData.q0) + power2f(imuData.q1) + power2f(imuData.q2) + power2f(imuData.q3));
#else
		recipNorm = invSqrtf(power2f(imuData.q0) + power2f(imuData.q1) + power2f(imuData.q2) + power2f(imuData.q3));
#endif
		imuData.q0 *= recipNorm;
		imuData.q1 *= recipNorm;
		imuData.q2 *= recipNorm;
		imuData.q3 *= recipNorm;
	}
}

IRAM_ATTR void madgwickARGUpdate(float dt) {
	float recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4, _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3, gx, gy,
		gz, ax, ay, az;

	// Convert gyroscope degrees/sec to radians/sec
	gx = convertDegToRad(sensorAttitudeData.gxDSFiltered);
	gy = convertDegToRad(sensorAttitudeData.gyDSFiltered);
	gz = convertDegToRad(sensorAttitudeData.gzDSFiltered);
	ax = sensorAttitudeData.axGFiltered;
	ay = sensorAttitudeData.ayGFiltered; 
	az = sensorAttitudeData.azGFiltered;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-imuData.q1 * gx - imuData.q2 * gy - imuData.q3 * gz);
	qDot2 = 0.5f * (imuData.q0 * gx + imuData.q2 * gz - imuData.q3 * gy);
	qDot3 = 0.5f * (imuData.q0 * gy - imuData.q1 * gz + imuData.q3 * gx);
	qDot4 = 0.5f * (imuData.q0 * gz + imuData.q1 * gy - imuData.q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

// Normalize accelerometer measurement
#if MADGWICK_FILTER_USE_INV_SQRT_APPROX == 1
		recipNorm = fastInvSqrtf(power2f(ax) + power2f(ay) + power2f(az));
#else
		recipNorm = invSqrtf(power2f(ax) + power2f(ay) + power2f(az));
#endif
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * imuData.q0;
		_2q1 = 2.0f * imuData.q1;
		_2q2 = 2.0f * imuData.q2;
		_2q3 = 2.0f * imuData.q3;
		_4q0 = 4.0f * imuData.q0;
		_4q1 = 4.0f * imuData.q1;
		_4q2 = 4.0f * imuData.q2;
		_8q1 = 8.0f * imuData.q1;
		_8q2 = 8.0f * imuData.q2;
		q0q0 = imuData.q0 * imuData.q0;
		q1q1 = imuData.q1 * imuData.q1;
		q2q2 = imuData.q2 * imuData.q2;
		q3q3 = imuData.q3 * imuData.q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * imuData.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * imuData.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * imuData.q3 - _2q1 * ax + 4.0f * q2q2 * imuData.q3 - _2q2 * ay;

// normalize step magnitude
#if MADGWICK_FILTER_USE_INV_SQRT_APPROX == 1
		recipNorm = fastInvSqrtf(power2f(s0) + power2f(s1) + power2f(s2) + power2f(s3));
#else
		recipNorm = invSqrtf(power2f(s0) + power2f(s1) + power2f(s2) + power2f(s3));
#endif
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= madgwickFilterBeta * s0;
		qDot2 -= madgwickFilterBeta * s1;
		qDot3 -= madgwickFilterBeta * s2;
		qDot4 -= madgwickFilterBeta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	imuData.q0 += qDot1 * dt;
	imuData.q1 += qDot2 * dt;
	imuData.q2 += qDot3 * dt;
	imuData.q3 += qDot4 * dt;

	// Normalize quaternion
#if MADGWICK_FILTER_USE_INV_SQRT_APPROX == 1
	recipNorm = fastInvSqrtf(power2f(imuData.q0) + power2f(imuData.q1) + power2f(imuData.q2) + power2f(imuData.q3));
#else
	recipNorm = invSqrtf(power2f(imuData.q0) + power2f(imuData.q1) + power2f(imuData.q2) + power2f(imuData.q3));
#endif

	imuData.q0 *= recipNorm;
	imuData.q1 *= recipNorm;
	imuData.q2 *= recipNorm;
	imuData.q3 *= recipNorm;
}

#endif