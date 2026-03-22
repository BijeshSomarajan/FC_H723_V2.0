#ifndef SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_
#define SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_

#define POSITION_GNSS_CMS_PER_DEG_LAT 111320.0f * 100.0f

#define POSITION_GNSS_MIN_NSAT 8
#define POSITION_GNSS_MIN_HACC 1.5f
#define POSITION_GNSS_MIN_FIX 1
#define POSITION_GNSS_MIN_STABILITY_COUNT 25
#define POSITION_GNSS_RELIABILITY_THRESHOLD 5

void updatePositionDataReliability(float dt);
void convertGNSSToSICordinates( double latDeg, double longDeg, double latRef, double longRef, float *xCordinate, float *yCordinate);
void convertEarthToBodyCordinates(float xEarth, float yEarth, float heading, float *xBody, float *yBody);

#endif /* SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHELPER_H_ */
