#include <sys/_stdint.h>
#include "../common/PositionCommon.h"
#include "../../../util/MathUtil.h"
#include "../../../util/CommonUtil.h"
#include "../../../status/FCStatus.h"
#include "../../../sensors/position/GNSS.h"
#include "../../../sensors/attitude/AttitudeSensor.h"
#include "PositionManagerHelper.h"

float posManagerGNSSStableTime = 0;

__ATTR_ITCM_TEXT
void updatePositionDataReliability(float dt) {
	// 1. Basic threshold check (Must be a 3D fix or higher)
	uint8_t valid = (gnssData.fixType >= POSITION_GNSS_MIN_FIX) && (gnssData.hAcc <= POSITION_GNSS_MIN_HACC) && (gnssData.sAcc <= POSITION_GNSS_MIN_SACC) && (gnssData.satCount >= POSITION_GNSS_MIN_NSAT);
	if (valid) {
		// Accumulate trust linearly (1.0s of real time = 1.0s of trust value)
		posManagerGNSSStableTime += dt;
	} else {
		// Decay trust aggressively (1.0s of real time = 2.0s of trust loss)
		posManagerGNSSStableTime -= POSITION_GNSS_STABILITY_INVALID_GAIN * dt;
	}

	// Clamp securely to [0.0, POSITION_GNSS_STABILITY_MAX_WINDOW]
	posManagerGNSSStableTime = constrainToRangeF(posManagerGNSSStableTime, 0.0f, POSITION_GNSS_STABILITY_MAX_WINDOW);

	// 2. Clear, Explicit Hysteresis Logic
	if (fcStatusData.isPositionDataReliable) {
		// If currently trusted, it must drop below 1.0s to lose trust.
		// Starting from max saturation (2.0s), a drop below 1.0s takes exactly 0.5 seconds of bad data.
		if (posManagerGNSSStableTime < POSITION_GNSS_TRUST_THRESHOLD) {
			fcStatusData.isPositionDataReliable = 0;
		}
	} else {
		// If not trusted, it must climb above 1.0s to gain trust.
		// Starting from 0.0s, this takes exactly 1.0 second of clean, uninterrupted good data.
		if (posManagerGNSSStableTime > POSITION_GNSS_TRUST_THRESHOLD) {
			fcStatusData.isPositionDataReliable = 1;
		}
	}
}

__ATTR_ITCM_TEXT
void convertGNSSToSICordinates(double latDeg, double lonDeg, double latRefDeg, double lonRefDeg, float *x, float *y) {
	// Convert to radians
	double latRad = convertDegToRad(latDeg);
	double lonRad = convertDegToRad(lonDeg);
	double latRefRad = convertDegToRad(latRefDeg);
	double lonRefRad = convertDegToRad(lonRefDeg);
	// Differences
	double dLat = latRad - latRefRad;
	double dLon = lonRad - lonRefRad;
	// Mean latitude (better accuracy than using current lat)
	double meanLat = 0.5 * (latRad + latRefRad);
	// Earth frame (NED)
	// X → North
	// Y → East
	*x = (float) (dLat * POSITION_GNSS_EARTH_RADIUS_METERS);
	*y = (float) (dLon * POSITION_GNSS_EARTH_RADIUS_METERS * cos(meanLat));
}

__ATTR_ITCM_TEXT
void convertEarthToBodyCordinates(float xEarth, float yEarth, float heading, float *xBody, float *yBody) {
	//heading = 0;
	float headingRad = convertDegToRadF(heading);
	float headingCosValue = cosApproxF(headingRad);
	float headingSinValue = sinApproxF(headingRad);

	*xBody = (xEarth * headingCosValue) + (yEarth * headingSinValue);
	*yBody = (-xEarth * headingSinValue) + (yEarth * headingCosValue);
}

__ATTR_ITCM_TEXT
float getGroundSpeed(void) {
	float vx = positionCordinateData.xVelocity;
	float vy = positionCordinateData.yVelocity;
	return fastSqrtf((vx * vx) + (vy * vy));
}

