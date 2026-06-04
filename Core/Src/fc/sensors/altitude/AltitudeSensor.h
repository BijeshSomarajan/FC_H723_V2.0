#ifndef _BAROCOMMON_H_
#define _BAROCOMMON_H_

#include <sys/_stdint.h>

#define SENSOR_ALT_BARO_LPF_FREQUENCY  5.0f
#define SENSOR_ALT_BARO_LPF_SMOOTHEST_FREQUENCY  0.65f

#define SENSOR_ALT_LIDAR_AVAILABLE 1
#define SENSOR_ALT_LIDAR_LPF_FREQUENCY  5.0f
#define SENSOR_ALT_LIDAR_LPF_SMOOTHEST_FREQUENCY  0.65f

#define SENSOR_BARO_READ_FREQUENCY DEVICE_BARO_READ_FREQUENCY
#define SENSOR_BARO_READ_PERIOD 1.0f/SENSOR_BARO_READ_FREQUENCY

#define SENSOR_LIDAR_READ_FREQUENCY DEVICE_LIDAR_READ_FREQUENCY * 2
#define SENSOR_LIDAR_READ_PERIOD 1.0f/SENSOR_LIDAR_READ_FREQUENCY

typedef struct _SENSOR_ALTITUDE_DATA SENSOR_ALTITUDE_DATA;
struct _SENSOR_ALTITUDE_DATA {
	float altitudeSLGround;
	float altitudeSL;
	float altitudeSLScaled;
	float altitudeSLFiltered;
	float altitudeSLMaxFiltered;
	float altUpdateDt;
	float altProcessDt;

	float altitudeTerrain;
	float altitudeTerrainQlty;

};
extern SENSOR_ALTITUDE_DATA sensorAltitudeData;

uint8_t initAltitudeSensors(void);
uint8_t readAltitudeSensors(float dt);
uint8_t loadAltitudeSensorsData(void);
void updateAltitudeSensorData(float dt);
void resetAltitudeSensors(uint8_t hard);

#endif /* FC_FCDEVICES_INCLUDE_BAROCOMMON_H_ */
