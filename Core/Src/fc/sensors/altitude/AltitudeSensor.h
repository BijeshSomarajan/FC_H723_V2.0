#ifndef _BAROCOMMON_H_
#define _BAROCOMMON_H_

#include <sys/_stdint.h>

#define SENSOR_ALT_BARO_LPF_FREQUENCY  5.0f

#define SENSOR_ALT_LIDAR_AVAILABLE 1
#define SENSOR_ALT_LIDAR_LPF_FREQUENCY  5.0f

#define SENSOR_BARO_READ_FREQUENCY DEVICE_BARO_READ_FREQUENCY
#define SENSOR_BARO_READ_PERIOD 1.0f/SENSOR_BARO_READ_FREQUENCY

#define SENSOR_LIDAR_READ_FREQUENCY DEVICE_LIDAR_READ_FREQUENCY * 2
#define SENSOR_LIDAR_READ_PERIOD 1.0f/SENSOR_LIDAR_READ_FREQUENCY

#define SENSOR_DATA_NONE  0x00
#define SENSOR_DATA_BARO  0x01
#define SENSOR_DATA_LIDAR 0x02

typedef struct _SENSOR_ALTITUDE_DATA SENSOR_ALTITUDE_DATA;
struct _SENSOR_ALTITUDE_DATA {
	float altitudeSLGround;
	float altitudeSL;
	float altitudeSLScaled;
	float altitudeSLFiltered;
	float altitudeSLZOffset;

	float altUpdateDt;
	float altProcessDt;

	float altitudeTerrain;
	float altitudeTerrainZOffset;
	float altitudeTerrainQual;

	float altitudeTerrainFiltered;
	float altitudeSLUpdateDt;

};
extern SENSOR_ALTITUDE_DATA sensorAltitudeData;

uint8_t initAltitudeSensors(void);
uint8_t readAltitudeSensors(float dt);
uint8_t loadAltitudeSensorsData(void);
void resetAltitudeSensors(uint8_t hard);

#endif /* FC_FCDEVICES_INCLUDE_BAROCOMMON_H_ */
