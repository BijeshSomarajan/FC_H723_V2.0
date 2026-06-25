#ifndef SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_
#define SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_

#include "../estimator/PositionEstimator.h"

#define POSITION_GNSS_EARTH_RADIUS_METERS 6378137.0
#define POSITION_GNSS_DEG_TO_RAD 0.017453292519943295

// GNSS Quality Gates
#define POSITION_GNSS_MIN_NSAT                  8       // Modern multi-GNSS easily gets 12+ sats out in the open
#define POSITION_GNSS_MIN_HACC                  3.0f    // Meters (Ceiling for acceptable horizontal position scatter)
#define POSITION_GNSS_MIN_VACC                  3.0f    // Meters (Ceiling for acceptable vertical position scatter)
#define POSITION_GNSS_MIN_FIX                   2       // MANDATORY: 3 = 3D Fix
#define POSITION_GNSS_STABILITY_MAX_WINDOW      2.0f    // Seconds (Maximum accumulator depth)
#define POSITION_GNSS_STABILITY_INVALID_GAIN    1.0f    // Decay rate multiplier for bad data
#define POSITION_GNSS_TRUST_THRESHOLD           1.0f    // Cross this boundary to change states

//Terrain quality gates
#define POSITION_TERRAIN_ALT_QUAL_MIN 0.02f
#define POSITION_TERRAIN_ALT_DIST_MIN     0.01f
#define POSITION_TERRAIN_ALT_DIST_MAX     12.0f
#define POSITION_TERRAIN_ALT_STABILITY_INVALID_GAIN    1.1f    // Decay rate multiplier for bad data
#define POSITION_TERRAIN_ALT_STABILITY_MAX_WINDOW      2.0f    // Seconds (Maximum accumulator depth)
#define POSITION_TERRAIN_ALT_TRUST_THRESHOLD 1.0f

#define POSITION_TERRAIN_NAV_STABILITY_INVALID_GAIN    1.1f    // Decay rate multiplier for bad data
#define POSITION_TERRAIN_NAV_STABILITY_MAX_WINDOW      2.0f    // Seconds (Maximum accumulator depth)
#define POSITION_TERRAIN_NAV_TRUST_THRESHOLD 1.0f
#define POSITION_TERRAIN_NAV_MIN_DIST    POS_ESTIMATOR_DYNAMIC_XY_FLOW_HEIGHT_MIN
#define POSITION_TERRAIN_NAV_MAX_DIST    POSITION_TERRAIN_ALT_DIST_MAX //Limited to the ramge of Terrain Alt Sensor
#define POSITION_TERRAIN_NAV_QUAL_MIN    POS_ESTIMATOR_DYNAMIC_XY_FLOW_QUAL_MIN

void updateGNSSDataReliability(float dt);
void updateTerrainAltDataReliability(float dt);
void updateTerrainNavDataReliability(float dt);
void convertGNSSToSICordinates(double latDeg, double longDeg, double latRef, double longRef, float *xCordinate, float *yCordinate);

void convertEarthToBodyCordinates(float xEarth, float yEarth, float heading, float *xBody, float *yBody);
void convertBodyToEarthCordinates(float xBody, float yBody, float heading, float *xEarth, float *yEarth);
uint8_t canEngageNavMode();
float getGroundSpeed(void);

#endif /* SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_ */
