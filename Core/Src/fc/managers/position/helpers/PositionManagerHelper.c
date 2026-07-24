#include <sys/_stdint.h>
#include "../common/PositionCommon.h"
#include "../../../util/MathUtil.h"
#include "../../../util/CommonUtil.h"
#include "../../../status/FCStatus.h"
#include "../../../sensors/position/GNSS.h"
#include "../../../sensors/attitude/AttitudeSensor.h"
#include "../../../sensors/altitude/AltitudeSensor.h"

#include "PositionManagerHelper.h"

float posManagerGNSSStableTime = 0;
float posManagerTerrainAltStableTime = 0;
float posManagerTerrainNavStableTime = 0;

__ATTR_ITCM_TEXT
void updateTerrainAltDataReliability(float dt) {
	uint8_t valid = fcStatusData.canFly && sensorAltitudeData.altitudeTerrainQual >= POSITION_TERRAIN_ALT_QUAL_MIN && sensorAltitudeData.altitudeTerrain >= POSITION_TERRAIN_ALT_DIST_MIN && sensorAltitudeData.altitudeTerrain <= POSITION_TERRAIN_ALT_DIST_MAX;
	if (valid) {
		// Accumulate trust linearly (1.0s of real time = 1.0s of trust value)
		posManagerTerrainAltStableTime += dt;
	} else {
		// Decay trust aggressively (1.0s of real time = 2.0s of trust loss)
		posManagerTerrainAltStableTime -= POSITION_TERRAIN_ALT_STABILITY_INVALID_GAIN * dt;
	}
	posManagerTerrainAltStableTime = constrainToRangeF(posManagerTerrainAltStableTime, 0.0f, POSITION_TERRAIN_ALT_STABILITY_MAX_WINDOW);
	// 2. Clear, Explicit Hysteresis Logic
	if (fcStatusData.isTerrainAltDataReliable) {
		// If currently trusted, it must drop below 1.0s to lose trust.
		// Starting from max saturation (2.0s), a drop below 1.0s takes exactly 0.5 seconds of bad data.
		if (posManagerTerrainAltStableTime < POSITION_TERRAIN_ALT_TRUST_THRESHOLD) {
			fcStatusData.isTerrainAltDataReliable = 0;
		}
	} else {
		// If not trusted, it must climb above 1.0s to gain trust.
		// Starting from 0.0s, this takes exactly 1.0 second of clean, uninterrupted good data.
		if (posManagerTerrainAltStableTime > POSITION_TERRAIN_ALT_TRUST_THRESHOLD) {
			fcStatusData.isTerrainAltDataReliable = 1;
		}
	}
}

__ATTR_ITCM_TEXT
void updateGNSSDataReliability(float dt) {
	// 1. Basic threshold check (Strictly requires 3D fix or higher)
	uint8_t valid = (gnssData.fixType >= POSITION_GNSS_MIN_FIX) && (gnssData.hAcc <= POSITION_GNSS_MIN_HACC) && (gnssData.vAcc <= POSITION_GNSS_MIN_VACC) && (gnssData.satCount >= POSITION_GNSS_MIN_NSAT);

	if (valid) {
		// Accumulate trust linearly (1.0s of clean data = 1.0s added to accumulator)
		posManagerGNSSStableTime += dt;
	} else {
		// Decay trust aggressively (1.0s of bad data = 2.0s removed from accumulator)
		posManagerGNSSStableTime -= (POSITION_GNSS_STABILITY_INVALID_GAIN * dt);
	}

	// Clamp securely to [0.0, POSITION_GNSS_STABILITY_MAX_WINDOW]
	posManagerGNSSStableTime = constrainToRangeF(posManagerGNSSStableTime, 0.0f, POSITION_GNSS_STABILITY_MAX_WINDOW);

	// 2. Clear, Explicit Hysteresis Logic
	if (fcStatusData.isNavDataReliable) {
		// Starting from max saturation (2.0s), a drop below 1.0s takes exactly 0.5 seconds of bad data.
		if (posManagerGNSSStableTime < POSITION_GNSS_TRUST_THRESHOLD) {
			fcStatusData.isNavDataReliable = 0;
		}
	} else {
		// Starting from 0.0s, this takes exactly 1.0 second of clean, uninterrupted good data.
		if (posManagerGNSSStableTime > POSITION_GNSS_TRUST_THRESHOLD) {
			fcStatusData.isNavDataReliable = 1;
		}
	}
}

__ATTR_ITCM_TEXT
uint8_t isNavModeActive() {
	return (fcStatusData.isNavRTHModeActive || fcStatusData.isNavModeActive);
}

__ATTR_ITCM_TEXT
void convertGNSSToXYCordinates(double latDeg, double lonDeg, double latRefDeg, double lonRefDeg, float *x, float *y) {
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
// Earth frame (NED) , X → North , Y → East
	*x = (float) (dLat * POSITION_GNSS_EARTH_RADIUS_METERS);
	*y = (float) (dLon * POSITION_GNSS_EARTH_RADIUS_METERS * cosApproxF(meanLat));
}

__ATTR_ITCM_TEXT
void convertEarthToBodyCordinates(float xEarth, float yEarth, float heading, float *xBody, float *yBody) {
	float headingRad = convertDegToRadF(heading);
	float headingCosValue = cosApproxF(headingRad);
	float headingSinValue = sinApproxF(headingRad);

	*xBody = (xEarth * headingCosValue) + (yEarth * headingSinValue);
	*yBody = (-xEarth * headingSinValue) + (yEarth * headingCosValue);
}

float calculateDistance(double homeLat, double homeLon, double lat, double lon) {
	float lat1 = convertDegToRadF((float) homeLat);
	float lat2 = convertDegToRadF((float) lat);
	float dLat = lat2 - lat1;
	float dLon = convertDegToRadF((float) (lon - homeLon));
	float x = dLon * cosApproxF((lat1 + lat2) * 0.5f);
	float y = dLat;
	return POSITION_GNSS_EARTH_RADIUS_METERS * fastSqrtf(x * x + y * y);
}

__ATTR_ITCM_TEXT
float calculateBearing(float xNorth, float yEast) {
	float bearing = convertRadToDegF(atan2Approx(-yEast, -xNorth));
	if (bearing < 0.0f) {
		bearing += 360.0f;
	}
	return bearing;
}



