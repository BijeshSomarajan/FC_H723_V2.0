#include "AltitudeSensor.h"

#include "../../dsp/LowPassFilter.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../status/FCStatus.h"
#include "../../util/MathUtil.h"
#include "../attitude/AttitudeSensor.h"
#include "devices/AltitudeDevice.h"

SENSOR_ALTITUDE_DATA sensorAltitudeData;
extern DEVICE_ALTITUDE_DATA deviceAltitudeData;

LOWPASSFILTER sensorAltBaroLPF;
LOWPASSFILTER sensorAltTerrainLPF;

float sensorBaroReadDt = 0;
float sensorLidarReadDt = 0;

static float lidarDataLastRange = 0.0f;
static uint8_t lidarDataRejectStreak = SENSOR_LIDAR_MAX_REJECT_STREAK;   // starts "unsynced": first good sample is accepted

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

	status = deviceLidarInit();
	fcStatusData.isTerrainSensorExist = status;
	if (status) {
		logString("[Altitude Sensor] TF Mini >> Exist & Init Success\n");
	} else {
		logString("[Altitude Sensor] TF Mini >> Does not Exist or Init Failed!\n");
		status = 1;
	}

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
float getLidarTiltGain() {
	float tiltGain = cosApproxF(convertDegToRadF(sensorAttitudeData.pitch)) * cosApproxF(convertDegToRadF(sensorAttitudeData.roll));
	tiltGain = constrainToRangeF(tiltGain, 0.0f, 1.0f);
	return tiltGain;
}

__ATTR_ITCM_TEXT
static uint8_t isLidarSampleValid(float range, float qual) {
	uint8_t lidarValid = (qual >= SENSOR_LIDAR_QUAL_MIN) && (range >= SENSOR_LIDAR_ALT_MIN) && (range <= SENSOR_LIDAR_ALT_MAX);
	if (lidarValid && lidarDataRejectStreak < SENSOR_LIDAR_MAX_REJECT_STREAK) {
		float maxStep = SENSOR_LIDAR_NOISE_MARGIN + SENSOR_LIDAR_MAX_RATE * SENSOR_LIDAR_READ_PERIOD * (1 + lidarDataRejectStreak);
		lidarValid = fabsf(range - lidarDataLastRange) <= maxStep;
	}
	if (!lidarValid) {
		if (lidarDataRejectStreak < SENSOR_LIDAR_MAX_REJECT_STREAK) {
			lidarDataRejectStreak++;
		}
		return 0;
	}
	lidarDataRejectStreak = 0;
	lidarDataLastRange = range;
	return 1;
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
	if (fcStatusData.isTerrainSensorExist == 1) {
		if (deviceLidarLoadData()) {
			flags |= SENSOR_DATA_LIDAR;
			float range = deviceAltitudeData.altitudeTerrain * getLidarTiltGain();   // your clamped tilt correction
			float qual = deviceAltitudeData.altitudeTerrainQlty;
			sensorAltitudeData.altitudeTerrainQual = qual;
			sensorAltitudeData.altitudeTerrainDataValid = isLidarSampleValid(range, qual);
			if (sensorAltitudeData.altitudeTerrainDataValid) {
				sensorAltitudeData.altitudeTerrain = range;
				filterTerrainLevelAlt(SENSOR_LIDAR_READ_PERIOD);
			}
		}
	}
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

	if (fcStatusData.isTerrainSensorExist == 1) {
		sensorLidarReadDt += dt;
		if (sensorLidarReadDt >= SENSOR_LIDAR_READ_PERIOD) {
			sensorAltitudeData.altitudeTerrainUpdateDt = sensorLidarReadDt;
			sensorLidarReadDt = 0;
			status |= deviceLidarRead();
		}
	}

	return status;
}

void resetAltitudeSensors(uint8_t hard) {
	lowPassFilterResetToValue(&sensorAltBaroLPF, 0);
	lowPassFilterResetToValue(&sensorAltTerrainLPF, 0);
	sensorAltitudeData.altitudeSLFiltered = 0;
	sensorAltitudeData.altitudeSLScaled = 0;
	sensorAltitudeData.altitudeSL = 0;
	sensorAltitudeData.altitudeTerrain = 0;
	sensorAltitudeData.altitudeTerrainQual = 0;
	sensorBaroReadDt = 0;
	sensorLidarReadDt = 0;
	if (hard) {
		deviceBaroReset(hard);
		deviceLidarReset(hard);
	}
}

