#ifndef SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_
#define SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_

#define POSITION_GNSS_EARTH_RADIUS_METERS 6378137.0f
#define POSITION_GNSS_DEG_TO_RAD 0.017453292519943295

// GNSS Quality Gates (Tightened for safe autonomous flight)
#define POSITION_GNSS_MIN_NSAT                  8       // Modern multi-GNSS easily gets 12+ sats out in the open
#define POSITION_GNSS_MIN_HACC                  1.5f    // Meters (Ceiling for acceptable horizontal position scatter)
#define POSITION_GNSS_MIN_SACC                  0.5f    // Meter/Sec (Ceiling for speed estimation chatter)
#define POSITION_GNSS_MIN_FIX                   1       // MANDATORY: 3 = 3D Fix
// Hysteresis Configuration
#define POSITION_GNSS_STABILITY_MAX_WINDOW      2.0f    // Seconds (Maximum accumulator depth)
#define POSITION_GNSS_STABILITY_INVALID_GAIN    2.0f    // Decay rate multiplier for bad data
// Clear thresholds matching real-world time requirements
#define POSITION_GNSS_TRUST_THRESHOLD           1.0f    // Cross this boundary to change states

void updatePositionDataReliability(float dt);
void convertGNSSToSICordinates( double latDeg, double longDeg, double latRef, double longRef, float *xCordinate, float *yCordinate);
void convertEarthToBodyCordinates(float xEarth, float yEarth, float heading, float *xBody, float *yBody);
float getGroundSpeed(void) ;

#endif /* SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_ */
