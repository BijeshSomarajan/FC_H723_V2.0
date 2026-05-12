#ifndef SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_
#define SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_

#define POSITION_GNSS_EARTH_RADIUS_METERS 6378137.0f
#define POSITION_GNSS_DEG_TO_RAD 0.017453292519943295

#define POSITION_GNSS_MIN_NSAT 6
#define POSITION_GNSS_MIN_HACC 2.5f
#define POSITION_GNSS_MIN_FIX 1

#define POSITION_GNSS_STABILITY_MIN_VALID_DT 1.0f
#define POSITION_GNSS_STABILITY_MIN_INVALID_DT 0.5f
#define POSITION_GNSS_STABILITY_INVALID_GAIN 2.0f

void updatePositionDataReliability(float dt);
void convertGNSSToSICordinates( double latDeg, double longDeg, double latRef, double longRef, float *xCordinate, float *yCordinate);
void convertEarthToBodyCordinates(float xEarth, float yEarth, float heading, float *xBody, float *yBody);

#endif /* SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_ */
