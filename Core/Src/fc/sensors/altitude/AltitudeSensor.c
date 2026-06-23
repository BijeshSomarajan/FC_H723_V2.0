#include "AltitudeSensor.h"
#include "../../dsp/LowPassFilter.h"
#include "../../imu/IMU.h"
#include "../../logger/Logger.h"
#include "../../util/MathUtil.h"
#include "devices/AltitudeDevice.h"
#include "../../status/FCStatus.h"
#include "../rc/RCSensor.h"

SENSOR_ALTITUDE_DATA sensorAltitudeData;
extern DEVICE_ALTITUDE_DATA deviceAltitudeData;

LOWPASSFILTER sensorAltBaroLPF;
LOWPASSFILTER sensorAltTerrainLPF;

float sensorBaroReadDt = 0;
float sensorLidarReadDt = 0;

uint8_t initAltitudeSensors(void) {
	uint8_t status = 1;
	status = deviceBaroInit();
	if (status) {
		lowPassFilterInit(&sensorAltBaroLPF, SENSOR_ALT_BARO_LPF_FREQUENCY);
		lowPassFilterInit(&sensorAltTerrainLPF, SENSOR_ALT_LIDAR_LPF_FREQUENCY);

		logString("[Altitude Sensor] Baro Sensor Init > Success\n");
	} else {
		logString("[Altitude Sensor] Baro Sensor Init > Failed\n");
		return 0;
	}

#if SENSOR_ALT_LIDAR_AVAILABLE == 1
	status = deviceLidarInit();
	if (status) {
		logString("[Altitude Sensor] TF Mini Init > Success\n");
	} else {
		logString("[Altitude Sensor] TF Mini Init > Failed\n");
	}
#endif

	return status;
}

__ATTR_ITCM_TEXT
void scaleSeaLevelAlt() {
	sensorAltitudeData.altitudeSLScaled = sensorAltitudeData.altitudeSL;
}

__ATTR_ITCM_TEXT
void filterSeaLevelAlt(float dt) {
	sensorAltitudeData.altitudeSLFiltered = lowPassFilterUpdate(&sensorAltBaroLPF, sensorAltitudeData.altitudeSLScaled, dt);
}

__ATTR_ITCM_TEXT
void filterTerrainLevelAlt(float dt) {
	sensorAltitudeData.altitudeTerrainFiltered = lowPassFilterUpdate(&sensorAltTerrainLPF, sensorAltitudeData.altitudeTerrain, dt);
}

__ATTR_ITCM_TEXT
uint8_t loadAltitudeSensorsData(void) {
	uint8_t flags = SENSOR_DATA_NONE;
	if (deviceBaroLoadData()) {
		flags |= SENSOR_DATA_BARO;
		scaleSeaLevelAlt();
		filterSeaLevelAlt(SENSOR_BARO_READ_PERIOD);
		sensorAltitudeData.altitudeSL = deviceAltitudeData.altitudeSL;
	}
#if SENSOR_ALT_LIDAR_AVAILABLE == 1
	if (deviceLidarLoadData()) {
		flags |= SENSOR_DATA_LIDAR;
		sensorAltitudeData.altitudeTerrain = deviceAltitudeData.altitudeTerrain;
		filterTerrainLevelAlt(SENSOR_LIDAR_READ_PERIOD) ;
		sensorAltitudeData.altitudeTerrainQlty = deviceAltitudeData.altitudeTerrainQlty;
	}
#endif
	return flags;
}

__ATTR_ITCM_TEXT
uint8_t readAltitudeSensors(float dt) {
	uint8_t status = 0;
	sensorBaroReadDt += dt;
	if (sensorBaroReadDt >= SENSOR_BARO_READ_PERIOD) {
		sensorAltitudeData.altitudeSLUpdateDt = sensorBaroReadDt;
		sensorBaroReadDt = 0;
		status = deviceBaroRead();
	}

#if SENSOR_ALT_LIDAR_AVAILABLE == 1
	sensorLidarReadDt += dt;
	if (sensorLidarReadDt >= SENSOR_LIDAR_READ_PERIOD) {
		sensorLidarReadDt = 0;
		status |= deviceLidarRead();
	}
#endif

	return status;
}

void resetAltitudeSensors(uint8_t hard) {
	lowPassFilterResetToValue(&sensorAltBaroLPF, 0);
	lowPassFilterResetToValue(&sensorAltTerrainLPF, 0);
	sensorAltitudeData.altitudeSLFiltered = 0;
	sensorAltitudeData.altitudeSLScaled = 0;
	sensorAltitudeData.altitudeSL = 0;
	sensorAltitudeData.altitudeTerrain = 0;
	sensorAltitudeData.altitudeTerrainQlty = 0;
	sensorBaroReadDt = 0;
	sensorLidarReadDt = 0;
	if (hard) {
		deviceBaroReset(hard);
		deviceLidarReset(hard);
	}
}

