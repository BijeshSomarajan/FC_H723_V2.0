#include <sys/_stdint.h>
#include "../common/PositionCommon.h"
#include "../../../util/MathUtil.h"
#include "../../../util/CommonUtil.h"
#include "../../../status/FCStatus.h"
#include "../../../sensors/position/GNSS.h"
#include "../../../sensors/attitude/AttitudeSensor.h"
#include "PositionManagerHelper.h"

static uint16_t posManagerGNSSStabCount = 0;

void updateGNSSDataReliability(float dt) {
	// 1. Check current packet against your strict hardware constraints
	// Note: hAccMts should be LESS THAN the threshold for high accuracy
	uint8_t isCurrentPacketValid = (gnssData.fixStatus >= POSITION_GNSS_MIN_FIX) && (gnssData.hAccMts <= POSITION_GNSS_MIN_HACC) && (gnssData.satCount >= POSITION_GNSS_MIN_NSAT);

	if (isCurrentPacketValid) {
		// 2. Increment counter but cap it to avoid overflow
		if (posManagerGNSSStabCount < POSITION_GNSS_MIN_STABILITY_COUNT) {
			posManagerGNSSStabCount++;
		}
	} else {
		// 3. INSTANT INVALIDATION: If one packet is bad, we stop trusting immediately
		posManagerGNSSStabCount = 0;
	}
	// 4. Assert reliability only after passing the NSeconds duration
	if (posManagerGNSSStabCount >= POSITION_GNSS_MIN_STABILITY_COUNT) {
		fcStatusData.isGNSSDataReliable = 1;
	} else {
		fcStatusData.isGNSSDataReliable = 0;
	}
}

void convertGNSSToSICordinates(double latDeg, double longDeg, double latRef, double longRef, float *xCordinate, float *yCordinate) {
	double dLat = latDeg - latRef;
	double dLon = longDeg - longRef;
	float curLatitudeRad = (float) convertDegToRad(latDeg);
	*xCordinate = (float) (dLat * POSITION_GNSS_METERS_PER_DEG_LAT) * 100;
	*yCordinate = (float) (dLon * POSITION_GNSS_METERS_PER_DEG_LAT * cosApprox(curLatitudeRad)) * 100;
}

void convertEarthToBodyCordinates(float xEarth, float yEarth, float *xBody, float *yBody) {
	float headingRad = convertDegToRad(sensorAttitudeData.heading);

	float headingCosValue = cosApprox(headingRad);
	float headingSinValue = sinApprox(headingRad);

	*xBody = (xEarth * headingCosValue) + (yEarth * headingSinValue);
	*yBody = (-xEarth * headingSinValue) + (yEarth * headingCosValue);
}
