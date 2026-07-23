#include "PositionManagerHomeAquisitionHelper.h"

#include <math.h>
#include <sys/_stdint.h>
#include "../../../sensors/position/GNSS.h"
#include "../../../status/FCStatus.h"
#include "../../../util/MathUtil.h"

HOME_SAMPLE positionMgrHomeWindow[POSITION_MGR_HOME_WINDOW_LEN];
uint8_t positionMgrHomeCount;    // valid samples in window
uint8_t positionMgrHomeHead;     // next write index (circular)
float positionMgrHomeSampleAcc;  // sample-period accumulator [s]
float positionMgrHomeElapsed;    // total time trying [s]

void resetHomePositionAcquisition(void) {
	positionMgrHomeCount = 0;
	positionMgrHomeHead = 0;
	positionMgrHomeSampleAcc = 0.0f;
	positionMgrHomeElapsed = 0.0f;
}

/* Oldest-to-newest index i (0 = oldest) into the circular window. */
static const HOME_SAMPLE* homeSampleAt(uint8_t i) {
	uint8_t start = (uint8_t) ((positionMgrHomeHead + POSITION_MGR_HOME_WINDOW_LEN - positionMgrHomeCount) % POSITION_MGR_HOME_WINDOW_LEN);
	return &positionMgrHomeWindow[(start + i) % POSITION_MGR_HOME_WINDOW_LEN];
}

/* Mean over an oldest-relative index range [from, to). */
static void homeWindowMean(uint8_t from, uint8_t to, double *lat, double *lon, float *hMSL) {
	double sLat = 0.0, sLon = 0.0;
	float sZ = 0.0f;
	uint8_t n = (uint8_t) (to - from);
	for (uint8_t i = from; i < to; i++) {
		const HOME_SAMPLE *s = homeSampleAt(i);
		sLat += s->lat;
		sLon += s->lon;
		sZ += s->hMSL;
	}
	*lat = sLat / (double) n;
	*lon = sLon / (double) n;
	*hMSL = sZ / (float) n;
}

/* Degrees -> metres about a reference latitude. */
static void homeDegToMetres(double dLat, double dLon, double refLat, float *north, float *east) {
	*north = (float) (dLat * 111320.0);
	*east = (float) (dLon * 111320.0 * cos(refLat * (M_PI / 180.0)));
}

/*  Call once per GNSS cycle while home is not yet set; dt is the GNSS delta.
 *  Returns 1 on the cycle home is locked, 0 otherwise. */
uint8_t updateHomePositionAcquisition(float dt) {
	positionMgrHomeElapsed += dt;

	/* ---- gate 1: receiver confidence -------------------------------------
	 * A bad fix must not sit in the window contaminating the mean, so this
	 * CLEARS rather than slides. Consequence: if this keeps firing the window
	 * never fills and home is never set - deliberate, see header. */
	if (gnssData.hAcc > POSITION_MGR_HOME_MAX_HACC || gnssData.vAcc > POSITION_MGR_HOME_MAX_VACC) {
		positionMgrHomeCount = 0;
		positionMgrHomeHead = 0;
		positionMgrHomeSampleAcc = 0.0f;
		return 0;
	}

	/* ---- decimate to one sample per period -------------------------------
	 * The window must span real SETTLING time, not GNSS frames. At 10 Hz raw,
	 * 20 frames is 2 s - far too short to see a 20 s slew.
	 * Subtract the period rather than zeroing, so GNSS timing jitter does not
	 * slowly stretch the effective sample interval. */
	positionMgrHomeSampleAcc += dt;
	if (positionMgrHomeSampleAcc < POSITION_MGR_HOME_SAMPLE_PERIOD) {
		return 0;
	}
	positionMgrHomeSampleAcc -= POSITION_MGR_HOME_SAMPLE_PERIOD;

	positionMgrHomeWindow[positionMgrHomeHead].lat = gnssData.latitude;
	positionMgrHomeWindow[positionMgrHomeHead].lon = gnssData.longitude;
	positionMgrHomeWindow[positionMgrHomeHead].hMSL = gnssData.heightMSL;
	positionMgrHomeHead = (uint8_t) ((positionMgrHomeHead + 1) % POSITION_MGR_HOME_WINDOW_LEN);
	if (positionMgrHomeCount < POSITION_MGR_HOME_WINDOW_LEN) {
		positionMgrHomeCount++;
	}

	if (positionMgrHomeCount < POSITION_MGR_HOME_WINDOW_LEN) {
		return 0; /* window not full yet */
	}

	/* ---- statistics over the full window ---------------------------------- */
	double mLat, mLon;
	float mZ;
	homeWindowMean(0, POSITION_MGR_HOME_WINDOW_LEN, &mLat, &mLon, &mZ);

	uint8_t half = POSITION_MGR_HOME_WINDOW_LEN / 2;
	double oLat, oLon, nLat, nLon;
	float oZ, nZ;
	homeWindowMean(0, half, &oLat, &oLon, &oZ);                            /* older */
	homeWindowMean(half, POSITION_MGR_HOME_WINDOW_LEN, &nLat, &nLon, &nZ); /* newer */

	/* ---- gate 2: spread, RMS about the window mean -------------------------
	 * RMS not max: max grows with window length, so lengthening the window to
	 * catch a slower slew would silently tighten this gate. */
	float sumSq = 0.0f, sumSqZ = 0.0f;
	for (uint8_t i = 0; i < POSITION_MGR_HOME_WINDOW_LEN; i++) {
		const HOME_SAMPLE *s = homeSampleAt(i);
		float n, e;
		homeDegToMetres(s->lat - mLat, s->lon - mLon, mLat, &n, &e);
		sumSq += (n * n) + (e * e);
		float dz = s->hMSL - mZ;
		sumSqZ += (dz * dz);
	}
	float rmsDev = fastSqrtf(sumSq / (float) POSITION_MGR_HOME_WINDOW_LEN);
	float rmsDevZ = fastSqrtf(sumSqZ / (float) POSITION_MGR_HOME_WINDOW_LEN);

	/* ---- gate 3: drift between window halves - the slew detector ---------- */
	float dn, de;
	homeDegToMetres(nLat - oLat, nLon - oLon, mLat, &dn, &de);
	float drift = fastSqrtf((dn * dn) + (de * de));
	float driftZ = fabsf(nZ - oZ);

	uint8_t settled = (rmsDev <= POSITION_MGR_HOME_MAX_SPREAD_RMS) && (rmsDevZ <= POSITION_MGR_HOME_MAX_SPREAD_RMS_Z) && (drift <= POSITION_MGR_HOME_MAX_DRIFT) && (driftZ <= POSITION_MGR_HOME_MAX_DRIFT_Z);

	uint8_t timedOut = (POSITION_MGR_HOME_TIMEOUT > 0.0f) && (positionMgrHomeElapsed >= POSITION_MGR_HOME_TIMEOUT);

	if (!settled && !timedOut) {
		return 0; /* slide the window, try again next second */
	}

	/* ---- lock -------------------------------------------------------------
	 * Use the NEWER half only. If we got here on the timeout path the solution
	 * may still be slewing, and the newer half is closer to truth than the
	 * whole-window mean. If we got here settled, both halves agree anyway, so
	 * this costs nothing. */
	fcStatusData.positionLatHome = nLat;
	fcStatusData.positionLongHome = nLon;
	fcStatusData.positionZHome = nZ;
	fcStatusData.isPositionHomeSet = 1;

	resetHomePositionAcquisition();
	return 1;
}
