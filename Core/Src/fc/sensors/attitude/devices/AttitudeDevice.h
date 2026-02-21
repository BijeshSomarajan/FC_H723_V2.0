#ifndef FC_FCDEVICES_INCLUDE_MEMSCOMMON_H_
#define FC_FCDEVICES_INCLUDE_MEMSCOMMON_H_
#include <stdio.h>
#include <inttypes.h>

#include "../../../memory/Memory.h"

#define DEVICE_NONE 0
#define DEVICE_ISM330 1
#define DEVICE_RM3100 2

#define DEVICE_AG_SELECTED DEVICE_ISM330
#define DEVICE_MAG_SELECTED DEVICE_RM3100

#if DEVICE_AG_SELECTED == DEVICE_ISM330

//Reading AGT in a single shot
#define DEVICE_ACC_SAMPLE_FREQUENCY  3200.0f //1000.0f
#define DEVICE_GYRO_SAMPLE_FREQUENCY 3200.0f //3200.0f
#define DEVICE_TEMP_SAMPLE_FREQUENCY 3200.0f //20.0f

#define DEVICE_ACC_TEMP_OFFSET_CORRECTION 0
#define DEVICE_GYRO_TEMP_OFFSET_CORRECTION 0
#endif

#if DEVICE_MAG_SELECTED == DEVICE_RM3100
#define DEVICE_MAG_SAMPLE_FREQUENCY  200.0f
#endif

typedef struct _DEVICE_ATTITUDE_DATA DEVICE_ATTITUDE_DATA;
struct _DEVICE_ATTITUDE_DATA {
	uint8_t bufferAccTx[4];
	uint8_t bufferGyroTx[4];
	uint8_t bufferMagTx[4];
	uint8_t bufferTempTx[4];

	uint8_t bufferAGTRx[14];
	uint8_t bufferAccRx[6];
	uint8_t bufferGyroRx[6];
	uint8_t bufferTempRx[2];
	uint8_t bufferMagRx[10];

	//Accelerometer , Gyroscope and Magnetometer Full Scale
	int16_t gyroFullScale, accFullScale, magFullScale;
	float maxG;
	float maxDS;

	//Temperature
	int16_t rawTemp;

	//Temperature in Celcius
	float tempC;

	//Temperature offsets and coefficients
	float offsetTemp;
	double gyroXTempCoeff[4];
	double gyroYTempCoeff[4];
	double gyroZTempCoeff[4];

	double accXTempCoeff[4];
	double accYTempCoeff[4];
	double accZTempCoeff[4];

	volatile float accXTempOffset;
	volatile float accYTempOffset;
	volatile float accZTempOffset;

	float gyroXTempOffset;
	float gyroYTempOffset;
	float gyroZTempOffset;

	//Accelerometer , Gyroscope and Magnetometer sensitivity
	float gyroSensitivity, accSensitivity, magSensitivity, tempSensitivity;

	//Accelerometer ,Gyroscope and Magnetometer Raw measurements
	int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
	int32_t rawMx, rawMy, rawMz;

	//Accelerometer , Gyroscope Raw offsets
	int16_t offsetAx, offsetAy, offsetAz, offsetGx, offsetGy, offsetGz;

	//Magnetometer factory offset , env bias and scale
	float offsetMx, offsetMy, offsetMz, biasMx, biasMy, biasMz, scaleMx, scaleMy, scaleMz;

	//Accelerometer , Gyroscope and Magnetometer Scaled measurements
	float axG, ayG, azG, gxDS, gyDS, gzDS, mx, my, mz;
};

extern DEVICE_ATTITUDE_DATA deviceAttitudeData;

uint8_t deviceAGInit(void);
uint8_t deviceMAGInit(void);

uint8_t deviceAccGyroTempRead(void);
uint8_t deviceAccGyroTempLoadData(void);
uint8_t deviceAccRead(void);
uint8_t deviceGyroRead(void);
uint8_t deviceTempRead(void);

uint8_t deviceMagReadOffset(void);
uint8_t deviceMagRead(void);
uint8_t deviceMagLoadData(void);

void deviceAccApplyOrientationForImu(void);
void deviceAccApplyOffsetCorrection(void);
void deviceGyroApplyOrientationForImu(void);
void deviceGyroApplyOffsetCorrection(void);
void deviceMagApplyOrientationForImu(void);
void deviceMagApplyOffsetCorrection(void);

void deviceGyroApplyTempOffsetCorrection(void);
void deviceAccApplyTempOffsetCorrection(void);

void deviceAccApplyDataScaling(void);
void deviceGyroApplyDataScaling(void);
void deviceMagApplyDataScaling(void);
void deviceTempApplyDataScaling(void);

float deviceAccGetMaxValidG(void);
float deviceGyroGetMaxValidDS(void);

#endif /* FC_FCDEVICES_INCLUDE_MEMSCOMMON_H_ */
