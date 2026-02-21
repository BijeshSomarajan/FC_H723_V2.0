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
LOWPASSFILTER sensorAltBaroLPFSmoothest;

uint8_t initAltitudeSensors(void) {
	uint8_t status = 1;
	status = deviceBaroInit();
	if (status) {
		lowPassFilterInit(&sensorAltBaroLPF, SENSOR_ALT_BARO_LPF_FREQUENCY);
		lowPassFilterInit(&sensorAltBaroLPFSmoothest, SENSOR_ALT_BARO_LPF_SMOOTHEST_FREQUENCY);
		logString("[Altitude Sensor] Baro Sensor Init > Success\n");
	} else {
		logString("[Altitude Sensor] Baro Sensor Init > Failed\n");
	}
	return status;
}

__ATTR_ITCM_TEXT
void scaleSeaLevelAlt() {
	sensorAltitudeData.altitudeSLScaled = sensorAltitudeData.altitudeSL * SENSOR_ALT_BARO_ALTITUDE_GAIN;
}

__ATTR_ITCM_TEXT
void filterSeaLevelAlt(float dt) {
	sensorAltitudeData.altitudeSLFiltered = lowPassFilterUpdate(&sensorAltBaroLPF, sensorAltitudeData.altitudeSLScaled, dt);
	sensorAltitudeData.altitudeSLMaxFiltered = lowPassFilterUpdate(&sensorAltBaroLPFSmoothest, sensorAltitudeData.altitudeSLScaled, dt);
}

__ATTR_ITCM_TEXT
void updateAltitudeSensorData(float dt) {
	scaleSeaLevelAlt();
	filterSeaLevelAlt(dt);
	sensorAltitudeData.altUpdateDt = dt;
}

__ATTR_ITCM_TEXT
uint8_t loadAltitudeSensorsData() {
	if (deviceBaroLoadData()) {
		sensorAltitudeData.altitudeSL = deviceAltitudeData.altitude;
		return 1;
	}
	return 0;
}

uint8_t readAltitudeSensors() {
	return deviceBaroRead();
}

void resetAltitudeSensors(uint8_t hard) {
	lowPassFilterResetToValue(&sensorAltBaroLPF, 0);
	lowPassFilterResetToValue(&sensorAltBaroLPFSmoothest, 0);
	sensorAltitudeData.altitudeSLFiltered = 0;
	sensorAltitudeData.altitudeSLScaled = 0;
	sensorAltitudeData.altitudeSLScaled = 0;
	if (hard) {
		deviceBaroReset();
	}
}

