#include "AttitudeSensor.h"

#include "../../calibration/Calibration.h"
#include "../../dsp/BiQuadFilter.h"
#include "../../dsp/LowPassFilter.h"
#include "../../imu/IMU.h"
#include "../../timers/DelayTimer.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/GPTimer.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"

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
void startAttitudeSensorsRead(void);
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

	deviceAttitudeData.offsetMx = getScaledCalibrationValue(CALIB_PROP_MX_OFFSET_ADDR);
	deviceAttitudeData.offsetMy = getScaledCalibrationValue(CALIB_PROP_MY_OFFSET_ADDR);
	deviceAttitudeData.offsetMz = getScaledCalibrationValue(CALIB_PROP_MZ_OFFSET_ADDR);

	deviceAttitudeData.biasMx = getScaledCalibrationValue(CALIB_PROP_MX_BIAS_ADDR);
	deviceAttitudeData.biasMy = getScaledCalibrationValue(CALIB_PROP_MY_BIAS_ADDR);
	deviceAttitudeData.biasMz = getScaledCalibrationValue(CALIB_PROP_MZ_BIAS_ADDR);

	deviceAttitudeData.scaleMx = getScaledCalibrationValue(CALIB_PROP_MX_SCALE_ADDR);
	deviceAttitudeData.scaleMy = getScaledCalibrationValue(CALIB_PROP_MY_SCALE_ADDR);
	deviceAttitudeData.scaleMz = getScaledCalibrationValue(CALIB_PROP_MZ_SCALE_ADDR);

	deviceAttitudeData.offsetTemp = getScaledCalibrationValue(CALIB_PROP_IMU_TEMP_ADDR);
	deviceAttitudeData.tempC = deviceAttitudeData.offsetTemp;

	// Higher order coefficients are divided by higher powers of 10
	deviceAttitudeData.accXTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C_ADDR) / 10000.0f;	 // 3136.34291;
	deviceAttitudeData.accXTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C1_ADDR) / 100000.0f;	 //-66.95038;
	deviceAttitudeData.accXTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C2_ADDR) / 1000000.0f;	 //-3.85809;
	deviceAttitudeData.accXTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C3_ADDR) / 10000000.0f; // 0.04168;

	deviceAttitudeData.accYTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C_ADDR) / 10000.0f;	 // 3099.9199;
	deviceAttitudeData.accYTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C1_ADDR) / 100000.0f;	 //-111.3936;
	deviceAttitudeData.accYTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C2_ADDR) / 1000000.0f;	 //-1.23488;
	deviceAttitudeData.accYTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C3_ADDR) / 10000000.0f; // 0.00965;

	deviceAttitudeData.accZTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C_ADDR) / 10000.0f;	 //-14301.91832;
	deviceAttitudeData.accZTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C1_ADDR) / 100000.0f;	 // 6.38;
	deviceAttitudeData.accZTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C2_ADDR) / 1000000.0f;	 //-4.84;
	deviceAttitudeData.accZTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C3_ADDR) / 10000000.0f; // 0.03351;

	deviceAttitudeData.gyroXTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C_ADDR) / 10000.0f;	  // 12.8099;
	deviceAttitudeData.gyroXTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C1_ADDR) / 100000.0f;	  //-1.85993;
	deviceAttitudeData.gyroXTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C2_ADDR) / 1000000.0f;  // 0.4426;
	deviceAttitudeData.gyroXTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C3_ADDR) / 10000000.0f; //-0.00890;

	deviceAttitudeData.gyroYTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C_ADDR) / 10000.0f;	  // 10.94848;
	deviceAttitudeData.gyroYTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C1_ADDR) / 100000.0f;	  //-.46218;
	deviceAttitudeData.gyroYTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C2_ADDR) / 1000000.0f;  // 0.20199;
	deviceAttitudeData.gyroYTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C3_ADDR) / 10000000.0f; //-0.002;

	deviceAttitudeData.gyroZTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C_ADDR) / 10000.0f;	  // 181.0141
	deviceAttitudeData.gyroZTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C1_ADDR) / 100000.0f;	  //-6.10440;
	deviceAttitudeData.gyroZTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C2_ADDR) / 1000000.0f;  //-0.608;
	deviceAttitudeData.gyroZTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C3_ADDR) / 10000000.0f; // 0.00962;
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

void updateTempSensorData(float dt) {
	deviceTempApplyDataScaling();
	sensorAttitudeData.tempRaw = deviceAttitudeData.tempC;
	sensorAttitudeData.temp = deviceAttitudeData.tempC;
}

void updateAGTSensorData(float dt) {
	updateGyroSensorData(dt);
	updateAccSensorData(dt);
	updateTempSensorData(dt);
	sensorAttitudeData.agtDataUpdateDt = dt;
}

void updateMagSensorData(float dt) {
	deviceMagApplyDataScaling();
	deviceMagApplyOffsetCorrection();
	deviceMagApplyOrientationForImu();
	sensorAttitudeData.mx = deviceAttitudeData.mx;
	sensorAttitudeData.my = deviceAttitudeData.my;
	sensorAttitudeData.mz = deviceAttitudeData.mz;
	sensorAttitudeData.magDataUpdateDt = dt;
}

uint8_t loadAccGyroTempSensorData() {
	return deviceAccGyroTempLoadData();
}

__ATTR_ITCM_TEXT
uint8_t readAccGyroTempSensor() {
	if (deviceAccGyroTempRead()) {
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
		deviceTempRead();
		deviceTempApplyDataScaling();
		if (!lpfInit) {
			lowPassFilterResetToValue(&sensorAccXCalibLPF, deviceAttitudeData.rawAx);
			lowPassFilterResetToValue(&sensorAccYCalibLPF, deviceAttitudeData.rawAy);
			lowPassFilterResetToValue(&sensorAccZCalibLPF, deviceAttitudeData.rawAz);
			lowPassFilterResetToValue(&sensorGyroXCalibLPF, deviceAttitudeData.rawGx);
			lowPassFilterResetToValue(&sensorGyroYCalibLPF, deviceAttitudeData.rawGy);
			lowPassFilterResetToValue(&sensorGyroZCalibLPF, deviceAttitudeData.rawGz);
			lowPassFilterResetToValue(&sensorTempCalibLPF, deviceAttitudeData.tempC);
			lpfInit = 1;
		}
		float dt = SENSOR_AG_OFFSET_CALIB_SAMPLE_DELAY * 0.001;
		deviceAttitudeData.offsetAx = lowPassFilterUpdate(&sensorAccXCalibLPF, deviceAttitudeData.rawAx, dt);
		deviceAttitudeData.offsetAy = lowPassFilterUpdate(&sensorAccYCalibLPF, deviceAttitudeData.rawAy, dt);
		deviceAttitudeData.offsetAz = lowPassFilterUpdate(&sensorAccZCalibLPF, deviceAttitudeData.rawAz, dt);
		deviceAttitudeData.offsetGx = lowPassFilterUpdate(&sensorGyroXCalibLPF, deviceAttitudeData.rawGx, dt);
		deviceAttitudeData.offsetGy = lowPassFilterUpdate(&sensorGyroYCalibLPF, deviceAttitudeData.rawGy, dt);
		deviceAttitudeData.offsetGz = lowPassFilterUpdate(&sensorGyroZCalibLPF, deviceAttitudeData.rawGz, dt);
		deviceAttitudeData.offsetTemp = lowPassFilterUpdate(&sensorTempCalibLPF, deviceAttitudeData.tempC, dt);
		delayMs(SENSOR_AG_OFFSET_CALIB_SAMPLE_DELAY);
	}
	// Back fill data for persistence
	setCalibrationValue(CALIB_PROP_AX_BIAS_ADDR, deviceAttitudeData.offsetAx);
	setCalibrationValue(CALIB_PROP_AY_BIAS_ADDR, deviceAttitudeData.offsetAy);
	setCalibrationValue(CALIB_PROP_AZ_BIAS_ADDR, deviceAttitudeData.offsetAz);
	setCalibrationValue(CALIB_PROP_GX_BIAS_ADDR, deviceAttitudeData.offsetGx);
	setCalibrationValue(CALIB_PROP_GY_BIAS_ADDR, deviceAttitudeData.offsetGy);
	setCalibrationValue(CALIB_PROP_GZ_BIAS_ADDR, deviceAttitudeData.offsetGz);
	setCalibrationValue(CALIB_PROP_IMU_TEMP_ADDR, getCalibrationScalableValue(deviceAttitudeData.offsetTemp));
	// Persist the calibration
	saveCalibration();
}

void calculateMagBias() {
	deviceMagReadOffset();
	// Determining magnetometer bias , Move the device in 8 pattern
	int32_t mag_max[3] = { -2147483648, -2147483648, -2147483648 };
	int32_t mag_min[3] = { 2147483647, 2147483647, 2147483647 };
	for (int indx = 0; indx < SENSOR_MAG_CALIB_SAMPLE_COUNT; indx++) {
		// Read the mag data
		deviceMagRead();
		delayMs(2);
		deviceMagLoadData();
		// Check X
		if (deviceAttitudeData.rawMx > mag_max[0]) mag_max[0] = deviceAttitudeData.rawMx;
		if (deviceAttitudeData.rawMx < mag_min[0]) mag_min[0] = deviceAttitudeData.rawMx;

		// Check Y
		if (deviceAttitudeData.rawMy > mag_max[1]) mag_max[1] = deviceAttitudeData.rawMy;
		if (deviceAttitudeData.rawMy < mag_min[1]) mag_min[1] = deviceAttitudeData.rawMy;

		// Check Z
		if (deviceAttitudeData.rawMz > mag_max[2]) mag_max[2] = deviceAttitudeData.rawMz;
		if (deviceAttitudeData.rawMz < mag_min[2]) mag_min[2] = deviceAttitudeData.rawMz;

		delayMs(SENSOR_MAG_CALIB_SAMPLE_DELAY);
	}

	// Get hard iron correction , Bias
	deviceAttitudeData.biasMx = ((float) (mag_max[0] + mag_min[0]) / 2.0f) * deviceAttitudeData.magSensitivity;
	deviceAttitudeData.biasMy = ((float) (mag_max[1] + mag_min[1]) / 2.0f) * deviceAttitudeData.magSensitivity;
	deviceAttitudeData.biasMz = ((float) (mag_max[2] + mag_min[2]) / 2.0f) * deviceAttitudeData.magSensitivity;

	// Get soft iron correction estimate
	deviceAttitudeData.scaleMx = (float) (mag_max[0] - mag_min[0]) / 2.0f; // get average x axis max chord length in counts
	deviceAttitudeData.scaleMy = (float) (mag_max[1] - mag_min[1]) / 2.0f; // get average y axis max chord length in counts
	deviceAttitudeData.scaleMz = (float) (mag_max[2] - mag_min[2]) / 2.0f; // get average z axis max chord length in counts
	float avg_rad = (deviceAttitudeData.scaleMx + deviceAttitudeData.scaleMy + deviceAttitudeData.scaleMz) / 3.0f;

	if (deviceAttitudeData.scaleMx != 0) {
		deviceAttitudeData.scaleMx = avg_rad / deviceAttitudeData.scaleMx;
	} else {
		deviceAttitudeData.scaleMx = 1.0f;
	}

	if (deviceAttitudeData.scaleMy != 0) {
		deviceAttitudeData.scaleMy = avg_rad / deviceAttitudeData.scaleMy;
	} else {
		deviceAttitudeData.scaleMy = 1.0f;
	}

	if (deviceAttitudeData.scaleMz != 0) {
		deviceAttitudeData.scaleMz = avg_rad / deviceAttitudeData.scaleMz;
	} else {
		deviceAttitudeData.scaleMz = 1.0f;
	}

	// Back fill data for persistence
	setCalibrationValue(CALIB_PROP_MX_OFFSET_ADDR, getCalibrationScalableValue(deviceAttitudeData.offsetMx));
	setCalibrationValue(CALIB_PROP_MY_OFFSET_ADDR, getCalibrationScalableValue(deviceAttitudeData.offsetMy));
	setCalibrationValue(CALIB_PROP_MZ_OFFSET_ADDR, getCalibrationScalableValue(deviceAttitudeData.offsetMz));

	setCalibrationValue(CALIB_PROP_MX_BIAS_ADDR, getCalibrationScalableValue(deviceAttitudeData.biasMx));
	setCalibrationValue(CALIB_PROP_MY_BIAS_ADDR, getCalibrationScalableValue(deviceAttitudeData.biasMy));
	setCalibrationValue(CALIB_PROP_MZ_BIAS_ADDR, getCalibrationScalableValue(deviceAttitudeData.biasMz));

	setCalibrationValue(CALIB_PROP_MX_SCALE_ADDR, getCalibrationScalableValue(deviceAttitudeData.scaleMx));
	setCalibrationValue(CALIB_PROP_MY_SCALE_ADDR, getCalibrationScalableValue(deviceAttitudeData.scaleMy));
	setCalibrationValue(CALIB_PROP_MZ_SCALE_ADDR, getCalibrationScalableValue(deviceAttitudeData.scaleMz));

	// Persist the calibration
	saveCalibration();
}

/***********************************************************************/
/* Send the calibration data                                           */
/***********************************************************************/
void sendAttitudeTempCalibData() {
	sensorAttitudeTempCalibData[0] = deviceAttitudeData.tempC * 100000;
	sensorAttitudeTempCalibData[1] = deviceAttitudeData.axG * 100000;
	sensorAttitudeTempCalibData[2] = deviceAttitudeData.ayG * 100000;
	sensorAttitudeTempCalibData[3] = deviceAttitudeData.azG * 100000;
	sensorAttitudeTempCalibData[4] = deviceAttitudeData.gxDS * 100000;
	sensorAttitudeTempCalibData[5] = deviceAttitudeData.gyDS * 100000;
	sensorAttitudeTempCalibData[6] = deviceAttitudeData.gzDS * 100000;
	//sendConfigData(sensorAttitudeTempCalibData, 7, CMD_CALIBRATE_IMU_TEMP_DATA);
}

void calculateAccAndGyroTempCoeff() {
	float deltaTemp = 0;
	uint8_t lpfInit = 0;
	float previousTemp = 0;
	float dt = SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY * 0.001;
	// Wait till the temperature is close to calibration temp
	do {
		delayMs(SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY);
		deviceTempRead();
		deviceTempApplyDataScaling();
		if (!lpfInit) {
			lowPassFilterResetToValue(&sensorTempCalibLPF, deviceAttitudeData.tempC);
			lpfInit = 1;
		}
		deviceAttitudeData.tempC = lowPassFilterUpdate(&sensorTempCalibLPF, deviceAttitudeData.tempC, dt);
	} while ( fabs(deviceAttitudeData.tempC - deviceAttitudeData.offsetTemp) > SESNSOR_TEMP_CAL_PROXIMITY_DEAD_BAND );
	previousTemp = deviceAttitudeData.tempC;
	lpfInit = 0;
	lowPassFilterResetToValue(&sensorTempCalibLPF, deviceAttitudeData.tempC);
	// Take measurements
	while ( fabs(deltaTemp) <= SESNSOR_TEMP_CAL_RANGE ) {
		delayMs(SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY);
		deviceTempRead();
		deviceAccRead();
		deviceGyroRead();

		deviceAccApplyOffsetCorrection();
		deviceGyroApplyOffsetCorrection();
		deviceAccApplyDataScaling();
		deviceGyroApplyDataScaling();
		deviceTempApplyDataScaling();
		if (!lpfInit) {
			lowPassFilterResetToValue(&sensorAccXCalibLPF, deviceAttitudeData.axG);
			lowPassFilterResetToValue(&sensorAccYCalibLPF, deviceAttitudeData.ayG);
			lowPassFilterResetToValue(&sensorAccZCalibLPF, deviceAttitudeData.azG);
			lowPassFilterResetToValue(&sensorGyroXCalibLPF, deviceAttitudeData.gxDS);
			lowPassFilterResetToValue(&sensorGyroYCalibLPF, deviceAttitudeData.gyDS);
			lowPassFilterResetToValue(&sensorGyroZCalibLPF, deviceAttitudeData.gzDS);
			lowPassFilterResetToValue(&sensorTempCalibLPF, deviceAttitudeData.tempC);
			lpfInit = 1;
		} else {
			deviceAttitudeData.axG = lowPassFilterUpdate(&sensorAccXCalibLPF, deviceAttitudeData.axG, dt);
			deviceAttitudeData.ayG = lowPassFilterUpdate(&sensorAccYCalibLPF, deviceAttitudeData.ayG, dt);
			deviceAttitudeData.azG = lowPassFilterUpdate(&sensorAccZCalibLPF, deviceAttitudeData.azG, dt);
			deviceAttitudeData.gxDS = lowPassFilterUpdate(&sensorGyroXCalibLPF, deviceAttitudeData.gxDS, dt);
			deviceAttitudeData.gyDS = lowPassFilterUpdate(&sensorGyroYCalibLPF, deviceAttitudeData.gyDS, dt);
			deviceAttitudeData.gzDS = lowPassFilterUpdate(&sensorGyroZCalibLPF, deviceAttitudeData.gzDS, dt);
			deviceAttitudeData.tempC = lowPassFilterUpdate(&sensorTempCalibLPF, deviceAttitudeData.tempC, dt);
		}
		deltaTemp = deviceAttitudeData.tempC - deviceAttitudeData.offsetTemp;
		if (fabs(previousTemp - deviceAttitudeData.tempC) >= SENSOR_TEMP_CAL_TEMP_DELTA) {
			previousTemp = deviceAttitudeData.tempC;
			deviceAttitudeData.tempC = deltaTemp;
			sendAttitudeTempCalibData();
		}
	} // Delta while loop
	delayMs(SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY * 10);
}
