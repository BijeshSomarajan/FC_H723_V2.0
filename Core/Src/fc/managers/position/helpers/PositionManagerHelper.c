#include <sys/_stdint.h>
#include "../common/PositionCommon.h"
#include "../../../util/MathUtil.h"
#include "../../../util/CommonUtil.h"
#include "../../../status/FCStatus.h"
#include "../../../sensors/position/GNSS.h"
#include "../../../sensors/attitude/AttitudeSensor.h"
#include "PositionManagerHelper.h"

static uint16_t posManagerGNSSStabCount = 0;

void updatePositionDataReliability(float dt) {
	uint8_t isCurrentPacketValid = (gnssData.fixStatus >= POSITION_GNSS_MIN_FIX) && (gnssData.hAccMts <= POSITION_GNSS_MIN_HACC) && (gnssData.satCount >= POSITION_GNSS_MIN_NSAT);
	if (isCurrentPacketValid) {
		if (posManagerGNSSStabCount < POSITION_GNSS_MIN_STABILITY_COUNT) {
			posManagerGNSSStabCount++;
		}
	} else {
		if (posManagerGNSSStabCount > 0) {
			posManagerGNSSStabCount--;
		}
	}
	if (posManagerGNSSStabCount >= POSITION_GNSS_RELIABILITY_THRESHOLD) {
		fcStatusData.isPositionDataReliable = 1;
	} else {
		fcStatusData.isPositionDataReliable = 0;
	}
}

void convertGNSSToSICordinates(double latDeg, double longDeg, double latRef, double longRef, float *xCordinate, float *yCordinate) {
	double dLat = latDeg - latRef;
	double dLon = longDeg - longRef;
	float curLatitudeRad = (float) convertDegToRad(latDeg);
	float cosLat = cosApprox(curLatitudeRad);
	// Optional safety clamp (prevents extreme edge cases)
	if (cosLat < 0.01f) {
		cosLat = 0.01f;
	}
	*xCordinate = (float) (dLat * POSITION_GNSS_CMS_PER_DEG_LAT) ;
	*yCordinate = (float) (dLon * POSITION_GNSS_CMS_PER_DEG_LAT * cosLat) ;
}

void convertEarthToBodyCordinates(float xEarth, float yEarth, float heading, float *xBody, float *yBody) {
	float headingRad = convertDegToRad(heading);

	float headingCosValue = cosApprox(headingRad);
	float headingSinValue = sinApprox(headingRad);

	*xBody = (xEarth * headingCosValue) + (yEarth * headingSinValue);
	*yBody = (-xEarth * headingSinValue) + (yEarth * headingCosValue);
}
