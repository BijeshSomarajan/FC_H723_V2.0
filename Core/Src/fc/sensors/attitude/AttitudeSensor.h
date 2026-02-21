#ifndef _ATTITUDESENSOR_H_
#define _ATTITUDESENSOR_H_

#include <stdio.h>
#include <inttypes.h>
#include "devices/AttitudeDevice.h"

/* -----------------------------------------------------------
 * 📌 Data Structures
 * -----------------------------------------------------------*/

typedef struct _SENSOR_ATTITUDE_DATA SENSOR_ATTITUDE_DATA;
struct _SENSOR_ATTITUDE_DATA {
	// Raw data (before scaling/calibration)
	float axGRaw, ayGRaw, azGRaw, gxDSRaw, gyDSRaw, gzDSRaw, tempRaw;

	// Scaled data (physical units: G, deg/s, uT, deg C)
	float axG, ayG, azG, gxDS, gyDS, gzDS, mx, my, mz, temp;

	// Filtered data
	float axGFiltered, ayGFiltered, azGFiltered, gxDSFiltered, gyDSFiltered, gzDSFiltered, mxFiltered, myFiltered, mzFiltered, tempFiltered;

	// Update Timings (Delta T)
	float agtDataUpdateDt;
	float magDataUpdateDt;
	float tempUpdateDt;

	float agtNoiseFilterationDt;
	float magNoiseFilterationDt;

	float noiseFilterProcessXDt;
	float noiseFilterProcessYDt;
	float noiseFilterProcessZDt;

	float noiseFilterDataUpdateDt;

	// Attitude Estimation (Pitch, Roll, Heading, Rates)
	float pitch;
	float pitchRate;
	float roll;
	float rollRate;
	float heading;
	float yawRate;

};

extern SENSOR_ATTITUDE_DATA sensorAttitudeData;

/* -----------------------------------------------------------
 * ⚙️ Sensor Sample Frequencies (Hz)
 * -----------------------------------------------------------*/

#define SENSOR_AGT_SAMPLE_FREQUENCY     DEVICE_GYRO_SAMPLE_FREQUENCY
#define SENSOR_ACC_SAMPLE_FREQUENCY     DEVICE_ACC_SAMPLE_FREQUENCY
#define SENSOR_GYRO_SAMPLE_FREQUENCY    DEVICE_GYRO_SAMPLE_FREQUENCY
#define SENSOR_MAG_SAMPLE_FREQUENCY     DEVICE_MAG_SAMPLE_FREQUENCY
#define SENSOR_TEMP_SAMPLE_FREQUENCY    DEVICE_TEMP_SAMPLE_FREQUENCY

/* -----------------------------------------------------------
 * ⚠️ Flyable Value Limits (Safety Thresholds)
 * -----------------------------------------------------------*/

#define SENSOR_ACC_FLYABLE_VALUE_XY_GAIN   0.9f
#define SENSOR_ACC_FLYABLE_VALUE_Z_GAIN    0.9f // G
#define SENSOR_GYRO_FLYABLE_VALUE_GAIN     0.9f // degrees/sec

/* -----------------------------------------------------------
 * 📐 Calibration Constants
 * -----------------------------------------------------------*/

// AG (Accelerometer/Gyroscope) Calibration
#define SENSOR_AG_OFFSET_CALIB_SAMPLE_COUNT     2000.0f
#define SENSOR_AG_OFFSET_CALIB_SAMPLE_DELAY     2
#define SENSOR_AG_CALIB_LOWPASS_FREQ            0.1f

// Temperature Calibration
#define SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY       2
#define SESNSOR_TEMP_CAL_PROXIMITY_DEAD_BAND    1.0f
#define SESNSOR_TEMP_CAL_RANGE                  25.0f // Degrees
#define SENSOR_TEMP_CAL_TEMP_DELTA              0.25f
#define SENSOR_AG_TEMP_CALIB_LOWPASS_FREQ       0.1f
#define SENSOR_TEMP_CORRECTION_ACC_ENABLED      DEVICE_APPLY_ACC_TEMP_OFFSET_CORRECTION
#define SENSOR_TEMP_CORRECTION_GYRO_ENABLED     DEVICE_APPLY_GYRO_TEMP_OFFSET_CORRECTION

// Magnetometer (MAG) Calibration
#define SENSOR_MAG_CALIB_SAMPLE_COUNT           5000
#define SENSOR_MAG_CALIB_SAMPLE_DELAY           10
#define SENSOR_MAG_CALIB_USE_SIMPLE_ALGO        1

/* -----------------------------------------------------------
 * 📞 Function Prototypes
 * -----------------------------------------------------------*/

uint8_t initAttitudeSensors();
void resetAttitudeSensors(uint8_t hard);
uint8_t readAccGyroTempSensor();
uint8_t loadAccGyroTempSensorData(void);

uint8_t readMagSensor();
uint8_t loadMagSensorData(void);

void updateAGTSensorData(float dt);
void updateMagSensorData(float dt);

void calculateAccAndGyroBias(void);
void calculateMagBias(void);
void calculateAccAndGyroTempCoeff(void);
float getMaxValidG(void);
float getMaxValidDS(void);

#endif // _ATTITUDESENSOR_H_
