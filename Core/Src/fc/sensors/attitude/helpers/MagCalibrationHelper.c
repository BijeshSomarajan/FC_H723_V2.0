#include "MagCalibrationHelper.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "../../../logger/Logger.h"
#include "../../../timers/DelayTimer.h"
#include "../devices/AttitudeDevice.h"

#define MAG_BINS 27 /* 3x3x3 direction grid; centre cell (13) cannot occur -> 26 usable */

MagCalResult magCalResult;

/*
 * Running sums for a least-squares fit of a general (rotated) ellipsoid
 *
 *     v'Mv + 2 n'v = 1      (M symmetric 3x3, 6 terms; n 3 terms)
 *
 * No sample buffer needed: 9x9 normal equations are accumulated on the fly.
 */
typedef struct {
	double ata[9][9];
	double atb[9];
	uint32_t n;
	uint8_t bin[MAG_BINS];
	float mn[3], mx[3];
} MagEllipsoid;

/* ~750 B;  keeps it off the stack. Calibration is therefore not re-entrant. */
MagEllipsoid fitAcc;

void ellInit(MagEllipsoid *e) {
	memset(e, 0, sizeof(*e));
	for (int a = 0; a < 3; a++) {
		e->mn[a] = FLT_MAX;
		e->mx[a] = -FLT_MAX;
	}
}

/* Direction bin of the sample relative to the running min/max midpoint, -1 if not binnable yet. */
int ellBinIndex(const MagEllipsoid *e, float x, float y, float z) {
	float v[3] = { x, y, z };
	float c[3];
	for (int a = 0; a < 3; a++) {
		if ((e->mx[a] - e->mn[a]) < MAG_CAL_BIN_START_SPAN_UT) {
			return -1;
		}
		c[a] = v[a] - 0.5f * (e->mx[a] + e->mn[a]);
	}
	float n = sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
	if (n < 1.0e-3f) {
		return -1;
	}
	int idx = 0;
	for (int a = 0; a < 3; a++) {
		float d = c[a] / n;
		int t = (d > 0.38f) ? 2 : (d < -0.38f ? 0 : 1);
		idx = idx * 3 + t;
	}
	return idx == 13 ? -1 : idx;
}

/* Returns 1 if the sample entered the fit, 0 if skipped (its direction bin is already full). */
uint8_t ellAdd(MagEllipsoid *e, float x, float y, float z) {
	float v[3] = { x, y, z };
	for (int a = 0; a < 3; a++) {
		if (v[a] < e->mn[a])
			e->mn[a] = v[a];
		if (v[a] > e->mx[a])
			e->mx[a] = v[a];
	}
	int bi = ellBinIndex(e, x, y, z);
	if (bi >= 0) {
		if (e->bin[bi] >= MAG_CAL_BIN_CAP) {
			return 0;
		}
		e->bin[bi]++;
	}
	double u = x / MAG_CAL_FIT_NORM_UT;
	double w = y / MAG_CAL_FIT_NORM_UT;
	double q = z / MAG_CAL_FIT_NORM_UT;
	double p[9] = { u * u, w * w, q * q, 2 * u * w, 2 * u * q, 2 * w * q, 2 * u, 2 * w, 2 * q };
	for (int i = 0; i < 9; i++) {
		for (int j = i; j < 9; j++) {
			e->ata[i][j] += p[i] * p[j];
		}
		e->atb[i] += p[i];
	}
	e->n++;
	return 1;
}

int ellBinsFilled(const MagEllipsoid *e) {
	int c = 0;
	for (int i = 0; i < MAG_BINS; i++) {
		if (i != 13 && e->bin[i] >= MAG_CAL_BIN_MIN) {
			c++;
		}
	}
	return c;
}

/* Gaussian elimination with partial pivoting (destroys a and b). */
bool solve9(double a[9][9], double b[9], double x[9]) {
	for (int c = 0; c < 9; c++) {
		int p = c;
		for (int r = c + 1; r < 9; r++) {
			if (fabs(a[r][c]) > fabs(a[p][c]))
				p = r;
		}
		if (fabs(a[p][c]) < 1.0e-12) {
			return false;
		}
		if (p != c) {
			for (int k = 0; k < 9; k++) {
				double t = a[c][k];
				a[c][k] = a[p][k];
				a[p][k] = t;
			}
			double t = b[c];
			b[c] = b[p];
			b[p] = t;
		}
		for (int r = c + 1; r < 9; r++) {
			double f = a[r][c] / a[c][c];
			for (int k = c; k < 9; k++) {
				a[r][k] -= f * a[c][k];
			}
			b[r] -= f * b[c];
		}
	}
	for (int r = 8; r >= 0; r--) {
		double s = b[r];
		for (int k = r + 1; k < 9; k++) {
			s -= a[r][k] * x[k];
		}
		x[r] = s / a[r][r];
	}
	return true;
}

bool inv3(const double m[3][3], double o[3][3]) {
	double c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
	double c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
	double c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
	double det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
	if (fabs(det) < 1.0e-18) {
		return false;
	}
	double id = 1.0 / det;
	o[0][0] = c00 * id;
	o[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * id;
	o[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id;
	o[1][0] = c01 * id;
	o[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id;
	o[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * id;
	o[2][0] = c02 * id;
	o[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * id;
	o[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id;
	return true;
}

/* Jacobi eigen-decomposition of a symmetric 3x3: a = V diag(w) V^T (destroys a). */
void jacobi3(double a[3][3], double w[3], double v[3][3]) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			v[i][j] = (i == j) ? 1.0 : 0.0;
		}
	}
	for (int sweep = 0; sweep < 30; sweep++) {
		double off = fabs(a[0][1]) + fabs(a[0][2]) + fabs(a[1][2]);
		if (off < 1.0e-15) {
			break;
		}
		for (int p = 0; p < 2; p++) {
			for (int q = p + 1; q < 3; q++) {
				if (fabs(a[p][q]) < 1.0e-18) {
					continue;
				}
				double th = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
				double t = (th >= 0 ? 1.0 : -1.0) / (fabs(th) + sqrt(th * th + 1.0));
				double c = 1.0 / sqrt(t * t + 1.0);
				double s = t * c;
				for (int k = 0; k < 3; k++) {
					double akp = a[k][p], akq = a[k][q];
					a[k][p] = c * akp - s * akq;
					a[k][q] = s * akp + c * akq;
				}
				for (int k = 0; k < 3; k++) {
					double apk = a[p][k], aqk = a[q][k];
					a[p][k] = c * apk - s * aqk;
					a[q][k] = s * apk + c * aqk;
				}
				for (int k = 0; k < 3; k++) {
					double vkp = v[k][p], vkq = v[k][q];
					v[k][p] = c * vkp - s * vkq;
					v[k][q] = s * vkp + c * vkq;
				}
			}
		}
	}
	for (int i = 0; i < 3; i++) {
		w[i] = a[i][i];
	}
}

/*
 * Solves for b and A. Returns false only on numerical failure / not an ellipsoid.
 * Quality limits (radius, ratio, spread) are applied by the caller.
 * A is scaled so the corrected field has the same mean radius as the raw data (stays in uT).
 */
bool ellSolve(const MagEllipsoid *e, MagCalResult *out) {
	if (e->n < 9) {
		return false;
	}
	double ata[9][9], ata0[9][9], atb[9], atb0[9], th[9];
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 9; j++) {
			ata[i][j] = (j >= i) ? e->ata[i][j] : e->ata[j][i];
		}
		atb[i] = e->atb[i];
	}
	memcpy(ata0, ata, sizeof(ata));
	memcpy(atb0, atb, sizeof(atb));
	if (!solve9(ata, atb, th)) {
		return false;
	}

	/* Algebraic residual from the normal equations: sum (v'Mv + 2n'v - 1)^2 */
	double ssr = (double) e->n;
	for (int i = 0; i < 9; i++) {
		double t = 0.0;
		for (int j = 0; j < 9; j++) {
			t += ata0[i][j] * th[j];
		}
		ssr += th[i] * t - 2.0 * th[i] * atb0[i];
	}
	double rmsAlg = sqrt((ssr > 0.0 ? ssr : 0.0) / (double) e->n);

	double M[3][3] = { { th[0], th[3], th[4] }, { th[3], th[1], th[5] }, { th[4], th[5], th[2] } };
	double n[3] = { th[6], th[7], th[8] };
	double Mi[3][3];
	if (!inv3(M, Mi)) {
		return false;
	}
	double c[3];
	for (int i = 0; i < 3; i++) {
		c[i] = -(Mi[i][0] * n[0] + Mi[i][1] * n[1] + Mi[i][2] * n[2]);
	}
	double k = 1.0 - (n[0] * c[0] + n[1] * c[1] + n[2] * c[2]); /* 1 + n'M^-1 n */
	if (k <= 1.0e-9) {
		return false;
	}
	double Mk[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Mk[i][j] = M[i][j] / k;
		}
	}
	double w[3], V[3][3];
	jacobi3(Mk, w, V);
	if (w[0] <= 0.0 || w[1] <= 0.0 || w[2] <= 0.0) {
		return false; /* not an ellipsoid: too little coverage or bad data */
	}

	double r[3] = { 1.0 / sqrt(w[0]), 1.0 / sqrt(w[1]), 1.0 / sqrt(w[2]) };
	double rmin = r[0], rmax = r[0];
	for (int i = 1; i < 3; i++) {
		if (r[i] < rmin)
			rmin = r[i];
		if (r[i] > rmax)
			rmax = r[i];
	}
	double rg = pow(r[0] * r[1] * r[2], 1.0 / 3.0) * MAG_CAL_FIT_NORM_UT; /* geometric-mean radius, uT */

	/* Symmetric square root sq = V diag(sqrt w) V^T : corrects shape without rotating the frame. */
	double sq[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			double s = 0.0;
			for (int m = 0; m < 3; m++) {
				s += V[i][m] * sqrt(w[m]) * V[j][m];
			}
			sq[i][j] = s;
		}
	}
	for (int i = 0; i < 3; i++) {
		out->b[i] = (float) (c[i] * MAG_CAL_FIT_NORM_UT);
		for (int j = 0; j < 3; j++) {
			out->A[i][j] = (float) (rg * sq[i][j] / MAG_CAL_FIT_NORM_UT);
		}
	}
	out->radius = (float) rg;
	out->spread = (float) (rmsAlg / (2.0 * k));
	out->axisRatio = (float) (rmax / rmin);
	out->binsFilled = (uint8_t) ellBinsFilled(e);
	return true;
}

float median3(float a, float b, float c) {
	float lo = a < b ? a : b;
	float hi = a < b ? b : a;
	float m = hi < c ? hi : c;
	return lo > m ? lo : m;
}

void publishToAttitude(const MagCalResult *r) {
	deviceAttitudeData.biasMx = r->b[0];
	deviceAttitudeData.biasMy = r->b[1];
	deviceAttitudeData.biasMz = r->b[2];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			deviceAttitudeData.softIronM[i][j] = r->A[i][j];
		}
	}
}

/* Restore a saved calibration (e.g. from flash after CRC/finite checks). */
void magCalSet(const float b[3], const float A[3][3]) {
	for (int i = 0; i < 3; i++) {
		magCalResult.b[i] = b[i];
		for (int j = 0; j < 3; j++) {
			magCalResult.A[i][j] = A[i][j];
		}
	}
	publishToAttitude(&magCalResult);
}

/*
 * Returns true and updates magCalResult / deviceAttitudeData bias/scale only on success.
 * On failure nothing is modified.
 * Tumble the device through all orientations (3D, not a flat spin) while it runs;
 * it stops early once the sphere is covered.
 */
uint8_t doMagCalibration(void) {
	deviceMagReadOffset();
	MagEllipsoid *e = &fitAcc;
	ellInit(e);
	float win[3][3]; /* sliding window of last 3 samples, in uT */
	for (int indx = 0; indx < SENSOR_MAG_CALIB_SAMPLE_COUNT; indx++) {
		deviceMagRead();
		delayMs(2);
		deviceMagLoadData();
		float *slot = win[indx % 3];
		slot[0] = (float) deviceAttitudeData.rawMx * deviceAttitudeData.magSensitivity;
		slot[1] = (float) deviceAttitudeData.rawMy * deviceAttitudeData.magSensitivity;
		slot[2] = (float) deviceAttitudeData.rawMz * deviceAttitudeData.magSensitivity;
		if (indx >= 2) {
			/* Median-of-3 per axis removes single-sample spikes. */
			float s[3];
			for (int a = 0; a < 3; a++) {
				s[a] = median3(win[0][a], win[1][a], win[2][a]);
			}
			ellAdd(e, s[0], s[1], s[2]);
			/* Stop only after the minimum run time AND once the sphere is covered in depth. */
			if (indx >= MAG_CAL_MIN_RUN_SAMPLES && e->n >= MAG_CAL_MIN_SAMPLES
					&& ellBinsFilled(e) >= MAG_CAL_MIN_BINS) {
				break;
			}
		}
		delayMs(SENSOR_MAG_CALIB_SAMPLE_DELAY);
	}
	/* ---- Validate coverage: every axis must have been swept ------------- */
	if (e->n < MAG_CAL_MIN_SAMPLES || ellBinsFilled(e) < MAG_CAL_MIN_BINS) {
		return 0;
	}
	for (int a = 0; a < 3; a++) {
		if ((e->mx[a] - e->mn[a]) < MAG_CAL_MIN_SPAN_UT) {
			return 0;
		}
	}
	/* ---- Least-squares ellipsoid, then sanity limits --------------------- */
	MagCalResult r;
	if (!ellSolve(e, &r)) {
		return 0;
	}
	if (r.radius < MAG_CAL_FIT_MIN_RADIUS || r.radius > MAG_CAL_FIT_MAX_RADIUS) {
		return 0;
	}
	if (r.axisRatio > MAG_CAL_MAX_RADIUS_RATIO || r.spread > MAG_CAL_MAX_SPREAD) {
		return 0;
	}
	for (int a = 0; a < 3; a++) {
		float mid = 0.5f * (e->mx[a] + e->mn[a]);
		if (fabsf(r.b[a] - mid) > MAG_CAL_MAX_FIT_SHIFT_UT) {
			return 0;
		}
	}
	magCalResult = r; /* diagnostics only: radius / spread / axisRatio / binsFilled */
	publishToAttitude(&r);
	return 1;
}
