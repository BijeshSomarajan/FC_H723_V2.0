#include "MagCalibrationHelper.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "../../../logger/Logger.h"
#include "../../../timers/DelayTimer.h"
#include "../devices/AttitudeDevice.h"

void magFitAdd(MagFitAccum *acc, float x, float y, float z) {
	double u = x / MAG_CAL_FIT_NORM_UT;
	double v = y / MAG_CAL_FIT_NORM_UT;
	double w = z / MAG_CAL_FIT_NORM_UT;
	double f[6] = { u * u, v * v, w * w, u, v, w };
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			acc->s[i][j] += f[i] * f[j];
		}
		acc->b[i] += f[i];
	}
	acc->n++;
}

/* Returns false if the system is singular or the result is not an ellipsoid. */
uint8_t magFitSolve(const MagFitAccum *acc, float bias[3], float radius[3], double *rms) {
	double m[6][7];
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			m[i][j] = acc->s[i][j];
		}
		m[i][6] = acc->b[i];
	}
	double pivotMin = 1.0e-9 * (double) acc->n;
	for (int col = 0; col < 6; col++) {
		int piv = col;
		for (int r = col + 1; r < 6; r++) {
			if (fabs(m[r][col]) > fabs(m[piv][col])) {
				piv = r;
			}
		}
		if (fabs(m[piv][col]) < pivotMin) {
			return false;
		}
		if (piv != col) {
			for (int c = 0; c < 7; c++) {
				double t = m[piv][c];
				m[piv][c] = m[col][c];
				m[col][c] = t;
			}
		}
		for (int r = col + 1; r < 6; r++) {
			double f = m[r][col] / m[col][col];
			for (int c = col; c < 7; c++) {
				m[r][c] -= f * m[col][c];
			}
		}
	}
	double t[6];
	for (int i = 5; i >= 0; i--) {
		double sum = m[i][6];
		for (int j = i + 1; j < 6; j++) {
			sum -= m[i][j] * t[j];
		}
		t[i] = sum / m[i][i];
	}

	if (t[0] <= 0.0 || t[1] <= 0.0 || t[2] <= 0.0) {
		return false;
	}
	double cx = -t[3] / (2.0 * t[0]);
	double cy = -t[4] / (2.0 * t[1]);
	double cz = -t[5] / (2.0 * t[2]);
	double k = 1.0 + t[3] * t[3] / (4.0 * t[0]) + t[4] * t[4] / (4.0 * t[1]) + t[5] * t[5] / (4.0 * t[2]);
	if (k <= 0.0) {
		return false;
	}
	bias[0] = (float) (cx * MAG_CAL_FIT_NORM_UT);
	bias[1] = (float) (cy * MAG_CAL_FIT_NORM_UT);
	bias[2] = (float) (cz * MAG_CAL_FIT_NORM_UT);
	radius[0] = (float) (sqrt(k / t[0]) * MAG_CAL_FIT_NORM_UT);
	radius[1] = (float) (sqrt(k / t[1]) * MAG_CAL_FIT_NORM_UT);
	radius[2] = (float) (sqrt(k / t[2]) * MAG_CAL_FIT_NORM_UT);
	/* Residual of the fit, computed from the sums: sum (1 - f.t)^2 */
	double sse = (double) acc->n;
	for (int i = 0; i < 6; i++) {
		sse -= 2.0 * t[i] * acc->b[i];
		for (int j = 0; j < 6; j++) {
			sse += t[i] * acc->s[i][j] * t[j];
		}
	}
	if (sse < 0.0) {
		sse = 0.0;
	}
	*rms = sqrt(sse / (double) acc->n);
	return true;
}

static float median3(float a, float b, float c) {
	float lo = a < b ? a : b;
	float hi = a < b ? b : a;
	float m = hi < c ? hi : c;
	return lo > m ? lo : m;
}

/*
 * Returns true and updates deviceAttitudeData bias/scale only on success.
 * On failure nothing is modified.
 */
uint8_t doMagCalibration(void) {
	deviceMagReadOffset();
	float mn[3] = { FLT_MAX, FLT_MAX, FLT_MAX };
	float mx[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
	MagFitAccum fit;
	memset(&fit, 0, sizeof(fit));
	float win[3][3]; /* sliding window of last 3 samples, in uT */
	uint32_t count = 0;
	char buf[64];
	for (int indx = 0; indx < SENSOR_MAG_CALIB_SAMPLE_COUNT; indx++) {
		deviceMagRead();
		delayMs(2);
		deviceMagLoadData();
		float *slot = win[indx % 3];
		slot[0] = (float) deviceAttitudeData.rawMx * deviceAttitudeData.magSensitivity;
		slot[1] = (float) deviceAttitudeData.rawMy * deviceAttitudeData.magSensitivity;
		slot[2] = (float) deviceAttitudeData.rawMz * deviceAttitudeData.magSensitivity;
		sprintf(buf, "%0.4f,%0.4f,%0.4f\n", slot[0], slot[1], slot[2]);
		logString(buf);
		if (indx >= 2) {
			/* Median-of-3 per axis removes single-sample spikes. */
			float s[3];
			for (int a = 0; a < 3; a++) {
				s[a] = median3(win[0][a], win[1][a], win[2][a]);
				if (s[a] > mx[a])
					mx[a] = s[a];
				if (s[a] < mn[a])
					mn[a] = s[a];
			}
			magFitAdd(&fit, s[0], s[1], s[2]);
			count++;
		}
		delayMs(SENSOR_MAG_CALIB_SAMPLE_DELAY);
	}
	/* ---- Validate coverage: every axis must have been swept ------------- */
	if (count < 3) {
		return 0;
	}
	for (int a = 0; a < 3; a++) {
		if ((mx[a] - mn[a]) < MAG_CAL_MIN_SPAN_UT) {
			return 0;
		}
	}
	/* ---- Baseline: min/max estimate ------------------------------------ */
	float bias[3];
	float half[3];
	for (int a = 0; a < 3; a++) {
		bias[a] = 0.5f * (mx[a] + mn[a]);
		half[a] = 0.5f * (mx[a] - mn[a]);
	}
	float avg = (half[0] + half[1] + half[2]) / 3.0f;
	float scale[3];
	for (int a = 0; a < 3; a++) {
		scale[a] = (half[a] > 0.0f) ? avg / half[a] : 1.0f;
	}
	/* ---- Refinement: least-squares ellipsoid, if it looks sane ----------- */
	float fBias[3];
	float fRadius[3];
	double rms;
	if (magFitSolve(&fit, fBias, fRadius, &rms) && rms <= MAG_CAL_MAX_FIT_RMS) {
		uint8_t ok = 1;
		float rMin = fRadius[0], rMax = fRadius[0];
		for (int a = 0; a < 3; a++) {
			if (fRadius[a] < MAG_CAL_FIT_MIN_RADIUS || fRadius[a] > MAG_CAL_FIT_MAX_RADIUS) {
				ok = 0;
			}
			if (fabsf(fBias[a] - bias[a]) > MAG_CAL_MAX_FIT_SHIFT_UT) {
				ok = 0;
			}
			if (fRadius[a] < rMin)
				rMin = fRadius[a];
			if (fRadius[a] > rMax)
				rMax = fRadius[a];
		}
		if (rMax / rMin > MAG_CAL_MAX_RADIUS_RATIO) {
			ok = 0;
		}
		if (ok) {
			float fAvg = (fRadius[0] + fRadius[1] + fRadius[2]) / 3.0f;
			for (int a = 0; a < 3; a++) {
				bias[a] = fBias[a];
				scale[a] = fAvg / fRadius[a];
			}
		}
	}
	deviceAttitudeData.biasMx = bias[0];
	deviceAttitudeData.biasMy = bias[1];
	deviceAttitudeData.biasMz = bias[2];
	deviceAttitudeData.scaleMx = scale[0];
	deviceAttitudeData.scaleMy = scale[1];
	deviceAttitudeData.scaleMz = scale[2];
	return 1;
}
