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
	// 1. Basic threshold check
	uint8_t valid = (gnssData.fixStatus >= POSITION_GNSS_MIN_FIX) && (gnssData.hAccMts <= POSITION_GNSS_MIN_HACC) && (gnssData.satCount >= POSITION_GNSS_MIN_NSAT);
	if (valid) {
		posManagerGNSSStableTime += dt;
	} else {
		// Faster decay: 2 seconds of "lost" time for every 1 second of real time
		posManagerGNSSStableTime -= POSITION_GNSS_STABILITY_INVALID_GAIN * dt;
	}
	// Clamp to [0, 2.0]
	posManagerGNSSStableTime = constrainToRangeF(posManagerGNSSStableTime, 0.0f, POSITION_GNSS_STABILITY_INVALID_GAIN);
	// 2. Hysteresis Logic
	if (fcStatusData.isPositionDataReliable) {
		// If already reliable, requires more than 0.5s of bad data to drop
		if (posManagerGNSSStableTime < POSITION_GNSS_STABILITY_MIN_INVALID_DT) {
			fcStatusData.isPositionDataReliable = 0;
		}
	} else {
		// If unreliable, requires 1.0s of consistent good data to gain trust
		if (posManagerGNSSStableTime > POSITION_GNSS_STABILITY_MIN_VALID_DT) {
			fcStatusData.isPositionDataReliable = 1;
		}
	}
}

__ATTR_ITCM_TEXT
// WGS84 Earth radius (meters)
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

