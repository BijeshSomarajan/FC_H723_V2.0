#include "../AttitudeDevice.h"

#if DEVICE_AG_SELECTED == DEVICE_ISM330
#include <string.h>
#include "../../../../logger/Logger.h"
#include "../../../../io/spi/SPI.h"
#include "ISM330.h"
#include "ISM330Registers.h"
#include "stm32h7xx_ll_utils.h"
#include "../../../../timers/DelayTimer.h"

#define ISM330_AG_DEVICE FC_SPI2_DEVICE1
#define ISM330_ACC_FULL_SCALE ISM_16g
#define ISM330_GYRO_FULL_SCALE ISM_2000dps
#define ISM330_APPLY_GYRO_TEMP_OFFSET_CORRECTION 0
#define ISM330_APPLY_ACC_TEMP_OFFSET_CORRECTION 0

#define ISM330_AGT_READ_ASYNC 1
#define ISM330_AGT_ASYNC_BUFFER_SIZE 8

void memsDeriveGyroSensitivity(void);
void memsDeriveAccSensitivity(void);
void ism330Reset(void);
void ism330ConfigureAcc(void);
void ism330ConfigureGyro(void);

volatile uint8_t ism330HasAGTData = 0;

uint8_t memsAGTCheckConnection() {
	uint8_t status = spi2ReadRegister(ISM330DHCX_WHO_AM_I, deviceAttitudeData.bufferAccRx, 1, ISM330_AG_DEVICE);
	delayMs(10);
	if (status) {
		char buf[50];
		sprintf(buf, "[ism330,WAI] : %X(%X) \n", deviceAttitudeData.bufferAccRx[0], ISM330DHCX_WHO_AM_I_VAL);
		logString(buf);

		return (deviceAttitudeData.bufferAccRx[0] == ISM330DHCX_WHO_AM_I_VAL);
	}
	return status;
}

uint8_t deviceAGInit() {
	uint8_t status = spi2Init();
	if (status) {
		logString("[ism330,IO:SPI] > Success\n");
	} else {
		logString("[ism330,IO:SPI] > Failed\n");
		return 0;
	}
	logString("[ism330,IO:CON] > Checking\n");

	status = memsAGTCheckConnection();

	if (status) {
		logString("[ism330,AGT:CON] > Success\n");
	} else {
		logString("[ism330,AGT:CON] > Failed\n");
		return 0;
	}

	delayMs(100);
	logString("[ism330,AGT:Configuring]\n");
	ism330Reset();
	logString("[ism330,AGT:Reset] > Success\n");

	delayMs(100);
	ism330ConfigureAcc();
	logString("[ism330,AGT:Acc config ] > Success\n");

	delayMs(100);
	ism330ConfigureGyro();
	logString("[ism330,AGT:Gyroc config ] > Success\n");

	delayMs(100);
	memsDeriveGyroSensitivity();
	memsDeriveAccSensitivity();

	return 1;
}

void ism330Reset() {
	/*
	 CTRL3_C (12h)
	 BOOT BDU  H_LACTIVE  PP_OD  SIM  IF_INC  0(1) SW_RESET
	 0   0       0        0     0     1      0      1
	 */
	deviceAttitudeData.bufferAccTx[0] = 0b00000101;
	spi2WriteRegister(ISM330DHCX_CTRL3_C, deviceAttitudeData.bufferAccTx, 1, ISM330_AG_DEVICE);
}

void ism330ConfigureAcc() {
	/*
	 CTRL1_XL (10h)
	 ODR_XL3 ODR_XL2 ODR_XL1 ODR_XL0 FS1_XL FS0_XL LPF2_XL_EN  0(1)
	 1      0       0       0       1    0      0           0         1.66 kHz , 4g , Ist LPF enabled'
	 1      0       0       0       0    1      0           0         1.66 kHz , 16g , Ist LPF enabled
	 */
	deviceAttitudeData.bufferAccTx[0] = 0b10000100;
	spi2WriteRegister(ISM330DHCX_CTRL1_XL, deviceAttitudeData.bufferAccTx, 1, ISM330_AG_DEVICE);
}

void ism330ConfigureGyro() {
	/*
	 CTRL4_C (13h)
	 0(1) SLEEP_G INT2_on_INT1 0(1) DRDY_MASK I2C_disable LPF1_SEL_G 0(1)
	 0      0       0           0      0         1           0        0    I2C Disable , LPF1 disable
	 */
	deviceAttitudeData.bufferGyroTx[0] = 0b00000100;
	spi2WriteRegister(ISM330DHCX_CTRL4_C, deviceAttitudeData.bufferGyroTx, 1, ISM330_AG_DEVICE);

	/*
	 CTRL2_G (11h)
	 ODR_G3   ODR_G2   ODR_G1   ODR_G0  FS1_G   FS0_G   FS_125   FS_4000
	 1        0       1         0       0       0        0        0       6.66 kHz , 00: ±250 dps
	 1        0       1         0       1       1        0        0       6.66 kHz , 11: ±2000 dps
	 */
	deviceAttitudeData.bufferGyroTx[0] = 0b10101100;
	spi2WriteRegister(ISM330DHCX_CTRL2_G, deviceAttitudeData.bufferGyroTx, 1, ISM330_AG_DEVICE);

}

void memsDeriveGyroSensitivity() {
	switch (ISM330_GYRO_FULL_SCALE) {
	case ISM_125dps:
		deviceAttitudeData.gyroSensitivity = 4.375f / 1000.0f;
		deviceAttitudeData.maxDS = 125.0f;
		break;
	case ISM_250dps:
		deviceAttitudeData.gyroSensitivity = 8.75f / 1000.0f;
		deviceAttitudeData.maxDS = 250.0f;
		break;
	case ISM_500dps:
		deviceAttitudeData.gyroSensitivity = 17.50f / 1000.0f;
		deviceAttitudeData.maxDS = 500.0f;
		break;
	case ISM_1000dps:
		deviceAttitudeData.gyroSensitivity = 35.0f / 1000.0f;
		deviceAttitudeData.maxDS = 1000.0f;
		break;
	case ISM_2000dps:
		deviceAttitudeData.gyroSensitivity = 70.0f / 1000.0f;
		deviceAttitudeData.maxDS = 2000.0f;
		break;
	case ISM_4000dps:
		deviceAttitudeData.gyroSensitivity = 140.0f / 1000.0f;
		deviceAttitudeData.maxDS = 4000.0f;
		break;
	}
}

void memsDeriveAccSensitivity() {
	switch (ISM330_ACC_FULL_SCALE) {
	case ISM_2g:
		deviceAttitudeData.accSensitivity = 0.061f / 1000.0f;
		deviceAttitudeData.maxG = 2.0f;
		break;
	case ISM_4g:
		deviceAttitudeData.accSensitivity = 0.122f / 1000.0f;
		deviceAttitudeData.maxG = 4.0f;
		break;
	case ISM_8g:
		deviceAttitudeData.accSensitivity = 0.244f / 1000.0f;
		deviceAttitudeData.maxG = 8.0f;
		break;
	case ISM_16g:
		deviceAttitudeData.accSensitivity = 0.488f / 1000.0f;
		deviceAttitudeData.maxG = 16.0f;
		break;
	}
}

void __deviceAGTCallback(uint8_t *buf, uint16_t len) {
	if (!ism330HasAGTData) {
		memcpy(deviceAttitudeData.bufferAGTRx, buf, len);
		ism330HasAGTData = 1;
	}
}

__ATTR_ITCM_TEXT
uint8_t deviceAccGyroTempLoadData() {
#if ISM330_AGT_READ_ASYNC ==1
	if (ism330HasAGTData) {
		ism330HasAGTData = 0;
		deviceAttitudeData.rawTemp = (deviceAttitudeData.bufferAGTRx[1] << 8) | deviceAttitudeData.bufferAGTRx[0];

		deviceAttitudeData.rawGx = (((int16_t) deviceAttitudeData.bufferAGTRx[3]) << 8) | deviceAttitudeData.bufferAGTRx[2];
		deviceAttitudeData.rawGy = (((int16_t) deviceAttitudeData.bufferAGTRx[5]) << 8) | deviceAttitudeData.bufferAGTRx[4];
		deviceAttitudeData.rawGz = (((int16_t) deviceAttitudeData.bufferAGTRx[7]) << 8) | deviceAttitudeData.bufferAGTRx[6];

		deviceAttitudeData.rawAx = (((int16_t) deviceAttitudeData.bufferAGTRx[9]) << 8) | deviceAttitudeData.bufferAGTRx[8];
		deviceAttitudeData.rawAy = (((int16_t) deviceAttitudeData.bufferAGTRx[11]) << 8) | deviceAttitudeData.bufferAGTRx[10];
		deviceAttitudeData.rawAz = (((int16_t) deviceAttitudeData.bufferAGTRx[13]) << 8) | deviceAttitudeData.bufferAGTRx[12];
		return 1;
	}
	return 0;
#else
	return 1;
#endif
}

__ATTR_ITCM_TEXT
uint8_t deviceAccGyroTempRead(void) {
#if ISM330_AGT_READ_ASYNC ==1
	if (spi2ReadRegisterAsync(ISM330DHCX_OUT_TEMP_L, 14, ISM330_AG_DEVICE, __deviceAGTCallback)) {
		return 1;
	}
#else
	if (spi2ReadRegister(ISM330DHCX_OUT_TEMP_L, deviceAttitudeData.bufferAGTRx, 14, ISM330_AG_DEVICE)) {
		deviceAttitudeData.rawTemp = (deviceAttitudeData.bufferAGTRx[1] << 8) | deviceAttitudeData.bufferAGTRx[0];

		deviceAttitudeData.rawGx = (((int16_t) deviceAttitudeData.bufferAGTRx[3]) << 8) | deviceAttitudeData.bufferAGTRx[2];
		deviceAttitudeData.rawGy = (((int16_t) deviceAttitudeData.bufferAGTRx[5]) << 8) | deviceAttitudeData.bufferAGTRx[4];
		deviceAttitudeData.rawGz = (((int16_t) deviceAttitudeData.bufferAGTRx[7]) << 8) | deviceAttitudeData.bufferAGTRx[6];

		deviceAttitudeData.rawAx = (((int16_t) deviceAttitudeData.bufferAGTRx[9]) << 8) | deviceAttitudeData.bufferAGTRx[8];
		deviceAttitudeData.rawAy = (((int16_t) deviceAttitudeData.bufferAGTRx[11]) << 8) | deviceAttitudeData.bufferAGTRx[10];
		deviceAttitudeData.rawAz = (((int16_t) deviceAttitudeData.bufferAGTRx[13]) << 8) | deviceAttitudeData.bufferAGTRx[12];
		return 1;
	}
#endif
	return 0;
}

uint8_t deviceAccRead(void) {
	if (spi2ReadRegister(ISM330DHCX_OUTX_L_A, deviceAttitudeData.bufferAccRx, 6, ISM330_AG_DEVICE)) {
		deviceAttitudeData.rawAx = (((int16_t) deviceAttitudeData.bufferAccRx[1]) << 8) | deviceAttitudeData.bufferAccRx[0];
		deviceAttitudeData.rawAy = (((int16_t) deviceAttitudeData.bufferAccRx[3]) << 8) | deviceAttitudeData.bufferAccRx[2];
		deviceAttitudeData.rawAz = (((int16_t) deviceAttitudeData.bufferAccRx[5]) << 8) | deviceAttitudeData.bufferAccRx[4];
		return 1;
	}
	return 0;
}

uint8_t deviceGyroRead(void) {
	if (spi2ReadRegister(ISM330DHCX_OUTX_L_G, deviceAttitudeData.bufferGyroRx, 6, ISM330_AG_DEVICE)) {
		deviceAttitudeData.rawGx = (((int16_t) deviceAttitudeData.bufferGyroRx[1]) << 8) | deviceAttitudeData.bufferGyroRx[0];
		deviceAttitudeData.rawGy = (((int16_t) deviceAttitudeData.bufferGyroRx[3]) << 8) | deviceAttitudeData.bufferGyroRx[2];
		deviceAttitudeData.rawGz = (((int16_t) deviceAttitudeData.bufferGyroRx[5]) << 8) | deviceAttitudeData.bufferGyroRx[4];
		return 1;
	}
	return 0;
}

uint8_t deviceTempRead(void) {
	if (spi2ReadRegister(ISM330DHCX_OUT_TEMP_L, deviceAttitudeData.bufferTempTx, 2, ISM330_AG_DEVICE)) {
		deviceAttitudeData.rawTemp = (deviceAttitudeData.bufferTempTx[1] << 8) | deviceAttitudeData.bufferTempTx[0];
		return 1;
	}
	return 0;
}

__ATTR_ITCM_TEXT
void deviceAccApplyOrientationForImu(void) {
	deviceAttitudeData.azG = (1 + deviceAttitudeData.azG);
}
__ATTR_ITCM_TEXT
void deviceGyroApplyOrientationForImu(void) {
}

__ATTR_ITCM_TEXT
void deviceGyroApplyTempOffsetCorrection(void) {
#if ISM330_APPLY_GYRO_TEMP_OFFSET_CORRECTION == 1
	deviceAttitudeData.gxDS -= deviceAttitudeData.gyroXTempOffset;
	deviceAttitudeData.gyDS -= deviceAttitudeData.gyroYTempOffset;
	deviceAttitudeData.gzDS -= deviceAttitudeData.gyroZTempOffset;
#endif
}

__ATTR_ITCM_TEXT
void deviceAccApplyTempOffsetCorrection(void) {
#if ISM330_APPLY_ACC_TEMP_OFFSET_CORRECTION == 1
	deviceAttitudeData.axG -= deviceAttitudeData.accXTempOffset;
	deviceAttitudeData.ayG -= deviceAttitudeData.accYTempOffset;
	deviceAttitudeData.azG -= deviceAttitudeData.accZTempOffset;
#endif
}

__ATTR_ITCM_TEXT
void deviceAccApplyOffsetCorrection() {
	deviceAttitudeData.rawAx -= deviceAttitudeData.offsetAx;
	deviceAttitudeData.rawAy -= deviceAttitudeData.offsetAy;
	deviceAttitudeData.rawAz -= deviceAttitudeData.offsetAz;
}

__ATTR_ITCM_TEXT
void deviceGyroApplyOffsetCorrection() {
	deviceAttitudeData.rawGx -= deviceAttitudeData.offsetGx;
	deviceAttitudeData.rawGy -= deviceAttitudeData.offsetGy;
	deviceAttitudeData.rawGz -= deviceAttitudeData.offsetGz;
}

__ATTR_ITCM_TEXT
void deviceAccApplyDataScaling(void) {
	deviceAttitudeData.axG = (float) deviceAttitudeData.rawAx * deviceAttitudeData.accSensitivity;
	deviceAttitudeData.ayG = (float) deviceAttitudeData.rawAy * deviceAttitudeData.accSensitivity;
	deviceAttitudeData.azG = (float) deviceAttitudeData.rawAz * deviceAttitudeData.accSensitivity;
}

__ATTR_ITCM_TEXT
void deviceGyroApplyDataScaling(void) {
	deviceAttitudeData.gxDS = (float) deviceAttitudeData.rawGx * deviceAttitudeData.gyroSensitivity;
	deviceAttitudeData.gyDS = (float) deviceAttitudeData.rawGy * deviceAttitudeData.gyroSensitivity;
	deviceAttitudeData.gzDS = (float) deviceAttitudeData.rawGz * deviceAttitudeData.gyroSensitivity;
}

void deviceTempApplyDataScaling(void) {
	deviceAttitudeData.tempC = 25.0f + (float) deviceAttitudeData.rawTemp / 256.0f;
}

float deviceAccGetMaxValidG(void) {
	return deviceAttitudeData.maxG * 0.9f;
}

float deviceGyroGetMaxValidDS(void) {
	return deviceAttitudeData.maxDS * 0.9f;
}

#endif
