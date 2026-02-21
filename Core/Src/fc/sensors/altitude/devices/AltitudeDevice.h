#ifndef SRC_FC_SENSORS_ALTITUDE_DEVICES_ALTITUDEDEVICE_H_
#define SRC_FC_SENSORS_ALTITUDE_DEVICES_ALTITUDEDEVICE_H_
#include <sys/_stdint.h>

typedef struct _DEVICE_ALTITUDE_DATA DEVICE_ALTITUDE_DATA;
struct _DEVICE_ALTITUDE_DATA {
	uint8_t buffer[32];
	uint32_t rawPressure;
	int32_t rawTemperature;
	float pressure;
	float temperature;
	float altitude;
	float altitudeGround;
};
extern DEVICE_ALTITUDE_DATA deviceAltitudeData;

#define BARO_SENSOR_BMP581 1
#define BARO_SENSOR_BMP390 2

#define BARO_SENSOR_SELECTED BARO_SENSOR_BMP390
#define BARO_SENSOR_READ_FREQUENCY DEVICE_BARO_READ_FREQUENCY

#define DEVICE_BARO_READ_FREQUENCY 100

uint8_t deviceBaroInit(void);
uint8_t deviceBaroRead(void);
uint8_t deviceBaroLoadData(void);
uint8_t deviceBaroReset(void);

#endif
