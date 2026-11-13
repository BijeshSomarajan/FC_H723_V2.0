#include "AttitudeSensor.h"

#include <math.h>
#include <sys/_stdint.h>

#include "../../calibration/Calibration.h"
#include "../../dsp/LowPassFilter.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../status/FCStatus.h"
#include "../../timers/DelayTimer.h"
#include "../../util/MathUtil.h"
#include "helpers/MagCalibrationHelper.h"

extern DEVICE_ATTITUDE_DATA deviceAttitudeData;
SENSOR_ATTITUDE_DATA __ATTR_DTCM_BSS sensorAttitudeData;

// Calibration related
LOWPASSFILTER sensorAccXCalibLPF, sensorAccYCalibLPF, sensorAccZCalibLPF;
LOWPASSFILTER sensorGyroXCalibLPF, sensorGyroYCalibLPF, sensorGyroZCalibLPF;

LOWPASSFILTER sensorTempCalibLPF;
int32_t sensorAttitudeTempCalibData[9];
float sensorAccXMaxG;
float sensorAccYMaxG;
float sensorAccZMaxG;

float sensorGyroXMaxDS;
float sensorGyroYMaxDS;
float sensorGyroZMaxDS;

// --- Start & Init ---
void startAttitudeMgmtTimers(void);
uint8_t initAttitudeSensors(void);

void loadAttitudeSensorConfig(void);

// --- Calibration Functions ---
void calculateAccAndGyroBias(void);
void calculateMagBias(void);
void sendAttitudeTempCalibData(void);
void calculateAccAndGyroTempCoeff(void);

void updateAccSensorData(float dt);
void updateGyroSensorData(float dt);
void updateTempSensorData(float dt);

uint8_t initAttitudeSensors() {
	uint8_t status = 1; // Start assuming success
	// Step 1: Initialize Accelerometer/Gyro (AG) Sensor
	if (status) {
		status = deviceAGInit();
		if (status) {
			logString("[Attitude Sensor] AG Sensor Init > Success\n");
		} else {
			logString("[Attitude Sensor] AG Sensor Init > Failed\n");
		}
	}
	// Step 2: Initialize Magnetometer (MAG) Sensor
	if (status) { // Only runs if Step 1 was successful
		status = deviceMAGInit();
		if (status) {
			logString("[Attitude Sensor] MAG Sensor Init > Success\n");
		} else {
			logString("[Attitude Sensor] MAG Sensor Init > Failed\n");
		}
	}
	// Step 3 & 4: Load Configuration and Initialize Filters
	if (status) { // Only runs if Step 1 and Step 2 were successful

		loadAttitudeSensorConfig();
		logString("[Attitude Sensor] Calibration > Loaded\n");
		// Initialize low pass filters
		lowPassFilterInit(&sensorAccXCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorAccYCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorAccZCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorGyroXCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorGyroYCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorGyroZCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorTempCalibLPF, SENSOR_AG_TEMP_CALIB_LOWPASS_FREQ);

		sensorAccXMaxG = getMaxValidG() * SENSOR_ACC_FLYABLE_VALUE_XY_GAIN;
		sensorAccYMaxG = getMaxValidG() * SENSOR_ACC_FLYABLE_VALUE_XY_GAIN;
		sensorAccZMaxG = getMaxValidG() * SENSOR_ACC_FLYABLE_VALUE_Z_GAIN;

		sensorGyroXMaxDS = getMaxValidDS() * SENSOR_GYRO_FLYABLE_VALUE_GAIN;
		sensorGyroYMaxDS = getMaxValidDS() * SENSOR_GYRO_FLYABLE_VALUE_GAIN;
		sensorGyroZMaxDS = getMaxValidDS() * SENSOR_GYRO_FLYABLE_VALUE_GAIN;

		logString("[Attitude Sensor] > Calibration Filters > Initialized\n");
	}
	// Final Status Report (Single exit point)
	if (status) {

		logString("[Attitude Sensor] > Init > Success\n");
	} else {
		logString("[Attitude Sensor] > Init > Failed\n");
	}
	return status;
}

void resetAttitudeSensors(uint8_t hard) {
	sensorAttitudeData.axG = 0;
	sensorAttitudeData.ayG = 0;
	sensorAttitudeData.azG = 0;
	sensorAttitudeData.gxDS = 0;
	sensorAttitudeData.gyDS = 0;
	sensorAttitudeData.gzDS = 0;
	sensorAttitudeData.axGRaw = 0;
	sensorAttitudeData.ayGRaw = 0;
	sensorAttitudeData.azGRaw = 0;
	sensorAttitudeData.gxDSRaw = 0;
	sensorAttitudeData.gyDSRaw = 0;
	sensorAttitudeData.gzDSRaw = 0;
	sensorAttitudeData.mx = 0;
	sensorAttitudeData.my = 0;
	sensorAttitudeData.mz = 0;
	sensorAttitudeData.temp = 0;
	sensorAttitudeData.tempRaw = 0;
}

void loadAttitudeSensorConfig() {
	deviceAttitudeData.offsetAx = getCalibrationValue(CALIB_PROP_AX_BIAS_ADDR);
	deviceAttitudeData.offsetAy = getCalibrationValue(CALIB_PROP_AY_BIAS_ADDR);
	deviceAttitudeData.offsetAz = getCalibrationValue(CALIB_PROP_AZ_BIAS_ADDR);

	deviceAttitudeData.offsetGx = getCalibrationValue(CALIB_PROP_GX_BIAS_ADDR);
	deviceAttitudeData.offsetGy = getCalibrationValue(CALIB_PROP_GY_BIAS_ADDR);
	deviceAttitudeData.offsetGz = getCalibrationValue(CALIB_PROP_GZ_BIAS_ADDR);


	/* Load: hard iron (uT) */
	deviceAttitudeData.biasMx = get1KXScaledCalibrationValue(CALIB_PROP_MX_BIAS_ADDR);
	deviceAttitudeData.biasMy = get1KXScaledCalibrationValue(CALIB_PROP_MY_BIAS_ADDR);
	deviceAttitudeData.biasMz = get1KXScaledCalibrationValue(CALIB_PROP_MZ_BIAS_ADDR);

	/* Load: soft iron matrix, row-major */
	deviceAttitudeData.softIronM[0][0] = get1KXScaledCalibrationValue(CALIB_PROP_M_SI_00_ADDR);
	deviceAttitudeData.softIronM[0][1] = get1KXScaledCalibrationValue(CALIB_PROP_M_SI_01_ADDR);
	deviceAttitudeData.softIronM[0][2] = get1KXScaledCalibrationValue(CALIB_PROP_M_SI_02_ADDR);
	deviceAttitudeData.softIronM[1][0] = get1KXScaledCalibrationValue(CALIB_PROP_M_SI_10_ADDR);
	deviceAttitudeData.softIronM[1][1] = get1KXScaledCalibrationValue(CALIB_PROP_M_SI_11_ADDR);
	deviceAttitudeData.softIronM[1][2] = get1KXScaledCalibrationValue(CALIB_PROP_M_SI_12_ADDR);
	deviceAttitudeData.softIronM[2][0] = get1KXScaledCalibrationValue(CALIB_PROP_M_SI_20_ADDR);
	deviceAttitudeData.softIronM[2][1] = get1KXScaledCalibrationValue(CALIB_PROP_M_SI_21_ADDR);
	deviceAttitudeData.softIronM[2][2] = get1KXScaledCalibrationValue(CALIB_PROP_M_SI_22_ADDR);

}


__ATTR_ITCM_TEXT
void updateAccSensorData(float dt) {

	deviceAccApplyOffsetCorrection();
	deviceAccApplyDataScaling();
	deviceAccApplyTempOffsetCorrection();

	sensorAttitudeData.axGRaw = deviceAttitudeData.axG;
	sensorAttitudeData.ayGRaw = deviceAttitudeData.ayG;
	sensorAttitudeData.azGRaw = deviceAttitudeData.azG;

	deviceAccApplyOrientationForImu();

// Limit the values
	sensorAttitudeData.axG = constrainToRangeF(deviceAttitudeData.axG, -sensorAccXMaxG, sensorAccXMaxG);
	sensorAttitudeData.ayG = constrainToRangeF(deviceAttitudeData.ayG, -sensorAccYMaxG, sensorAccYMaxG);
	sensorAttitudeData.azG = constrainToRangeF(deviceAttitudeData.azG, -sensorAccZMaxG, sensorAccZMaxG);
}

float getMaxValidG() {
	return deviceAccGetMaxValidG();
}

float getMaxValidDS() {
	return deviceGyroGetMaxValidDS();
}

void updateGyroSensorData(float dt) {
	deviceGyroApplyOffsetCorrection();
	deviceGyroApplyDataScaling();
	deviceGyroApplyTempOffsetCorrection();

	sensorAttitudeData.gxDSRaw = deviceAttitudeData.gxDS;
	sensorAttitudeData.gyDSRaw = deviceAttitudeData.gyDS;
	sensorAttitudeData.gzDSRaw = deviceAttitudeData.gzDS;

	deviceGyroApplyOrientationForImu();

	// Limit the values
	sensorAttitudeData.gxDS = constrainToRangeF(deviceAttitudeData.gxDS, -sensorGyroXMaxDS, sensorGyroXMaxDS);
	sensorAttitudeData.gyDS = constrainToRangeF(deviceAttitudeData.gyDS, -sensorGyroYMaxDS, sensorGyroYMaxDS);
	sensorAttitudeData.gzDS = constrainToRangeF(deviceAttitudeData.gzDS, -sensorGyroZMaxDS, sensorGyroZMaxDS);
}


void updateAGTSensorData(float dt) {
	updateGyroSensorData(dt);
	updateAccSensorData(dt);
	sensorAttitudeData.agtDataUpdateDt = dt;
}

void updateMagSensorData(float dt) {
	deviceMagApplyDataScaling();
	deviceMagApplyOffsetCorrection();

	if (fcStatusData.modelVersion == 1) {
		deviceMagApplyOrientationForImu(0);
	} else {
		deviceMagApplyOrientationForImu(180);
	}

	sensorAttitudeData.mx = deviceAttitudeData.mx;
	sensorAttitudeData.my = deviceAttitudeData.my;
	sensorAttitudeData.mz = deviceAttitudeData.mz;
	sensorAttitudeData.magDataUpdateDt = dt;
}

uint8_t loadAccGyroTempSensorData() {
	return deviceAccGyroTempLoadData();
}

__ATTR_ITCM_TEXT
uint8_t readAccAndGyroSensor() {
	if (deviceAccAndGyroRead()) {
		return 1;
	}
	return 0;
}

__ATTR_ITCM_TEXT
uint8_t readMagSensor() {
	if (deviceMagRead()) {
		return 1;
	}
	return 0;
}

uint8_t loadMagSensorData() {
	return deviceMagLoadData();
}

void calculateAccAndGyroBias() {
	// Calibrate mems Acc and Gyro
	deviceAttitudeData.offsetAx = 0;
	deviceAttitudeData.offsetAy = 0;
	deviceAttitudeData.offsetAz = 0;
	deviceAttitudeData.offsetGx = 0;
	deviceAttitudeData.offsetGy = 0;
	deviceAttitudeData.offsetGz = 0;
	uint8_t lpfInit = 0;
	// Take average of Acc and Gyro readings
	for (int16_t sampleCount = 0; sampleCount < SENSOR_AG_OFFSET_CALIB_SAMPLE_COUNT; sampleCount++) {
		deviceAccRead();
		deviceGyroRead();
		if (!lpfInit) {
			lowPassFilterResetToValue(&sensorAccXCalibLPF, deviceAttitudeData.rawAx);
			lowPassFilterResetToValue(&sensorAccYCalibLPF, deviceAttitudeData.rawAy);
			lowPassFilterResetToValue(&sensorAccZCalibLPF, deviceAttitudeData.rawAz);
			lowPassFilterResetToValue(&sensorGyroXCalibLPF, deviceAttitudeData.rawGx);
			lowPassFilterResetToValue(&sensorGyroYCalibLPF, deviceAttitudeData.rawGy);
			lowPassFilterResetToValue(&sensorGyroZCalibLPF, deviceAttitudeData.rawGz);
			lpfInit = 1;
		}
		float dt = SENSOR_AG_OFFSET_CALIB_SAMPLE_DELAY * 0.001;
		deviceAttitudeData.offsetAx = lowPassFilterUpdate(&sensorAccXCalibLPF, deviceAttitudeData.rawAx, dt);
		deviceAttitudeData.offsetAy = lowPassFilterUpdate(&sensorAccYCalibLPF, deviceAttitudeData.rawAy, dt);
		deviceAttitudeData.offsetAz = lowPassFilterUpdate(&sensorAccZCalibLPF, deviceAttitudeData.rawAz, dt);
		deviceAttitudeData.offsetGx = lowPassFilterUpdate(&sensorGyroXCalibLPF, deviceAttitudeData.rawGx, dt);
		deviceAttitudeData.offsetGy = lowPassFilterUpdate(&sensorGyroYCalibLPF, deviceAttitudeData.rawGy, dt);
		deviceAttitudeData.offsetGz = lowPassFilterUpdate(&sensorGyroZCalibLPF, deviceAttitudeData.rawGz, dt);
		delayMs(SENSOR_AG_OFFSET_CALIB_SAMPLE_DELAY);
	}
	// Back fill data for persistence
	setCalibrationValue(CALIB_PROP_AX_BIAS_ADDR, deviceAttitudeData.offsetAx);
	setCalibrationValue(CALIB_PROP_AY_BIAS_ADDR, deviceAttitudeData.offsetAy);
	setCalibrationValue(CALIB_PROP_AZ_BIAS_ADDR, deviceAttitudeData.offsetAz);
	setCalibrationValue(CALIB_PROP_GX_BIAS_ADDR, deviceAttitudeData.offsetGx);
	setCalibrationValue(CALIB_PROP_GY_BIAS_ADDR, deviceAttitudeData.offsetGy);
	setCalibrationValue(CALIB_PROP_GZ_BIAS_ADDR, deviceAttitudeData.offsetGz);
	// Persist the calibration
	saveCalibration();
}

void calculateMagBias() {
	doMagCalibration() ;

	// Back fill data for persistence
	/* Hard iron (uT) */
	setCalibrationValue(CALIB_PROP_MX_BIAS_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.biasMx));
	setCalibrationValue(CALIB_PROP_MY_BIAS_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.biasMy));
	setCalibrationValue(CALIB_PROP_MZ_BIAS_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.biasMz));

	/* Soft iron matrix, row-major */
	setCalibrationValue(CALIB_PROP_M_SI_00_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.softIronM[0][0]));
	setCalibrationValue(CALIB_PROP_M_SI_01_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.softIronM[0][1]));
	setCalibrationValue(CALIB_PROP_M_SI_02_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.softIronM[0][2]));
	setCalibrationValue(CALIB_PROP_M_SI_10_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.softIronM[1][0]));
	setCalibrationValue(CALIB_PROP_M_SI_11_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.softIronM[1][1]));
	setCalibrationValue(CALIB_PROP_M_SI_12_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.softIronM[1][2]));
	setCalibrationValue(CALIB_PROP_M_SI_20_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.softIronM[2][0]));
	setCalibrationValue(CALIB_PROP_M_SI_21_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.softIronM[2][1]));
	setCalibrationValue(CALIB_PROP_M_SI_22_ADDR, get1KXScalableCalibrationValue(deviceAttitudeData.softIronM[2][2]));

	// Persist the calibration
	saveCalibration();
}


