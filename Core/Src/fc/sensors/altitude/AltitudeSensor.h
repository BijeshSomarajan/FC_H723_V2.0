#ifndef _BAROCOMMON_H_
#define _BAROCOMMON_H_

#include <sys/_stdint.h>


#define SENSOR_ALT_BARO_LPF_FREQUENCY  5.0f
#define SENSOR_ALT_BARO_LPF_SMOOTHEST_FREQUENCY  0.25f

#define SENSOR_ALT_BARO_ALTITUDE_GAIN  100.0f  //In CMS // Output in meters

typedef struct _SENSOR_ALTITUDE_DATA SENSOR_ALTITUDE_DATA;
struct _SENSOR_ALTITUDE_DATA {
	float altitudeSLGround;
	float altitudeSL;
	float altitudeSLScaled;
	float altitudeSLFiltered;
	float altitudeSLMaxFiltered;
	float altUpdateDt;
	float altProcessDt;
	float altVenturiBias;

};
extern SENSOR_ALTITUDE_DATA sensorAltitudeData;

uint8_t initAltitudeSensors(void);
uint8_t readAltitudeSensors(void);
uint8_t loadAltitudeSensorsData(void);
void updateAltitudeSensorData(float dt);
void resetAltitudeSensors(uint8_t hard);

#endif /* FC_FCDEVICES_INCLUDE_BAROCOMMON_H_ */
