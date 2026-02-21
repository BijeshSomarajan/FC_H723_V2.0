#include "../AttitudeDevice.h"

#if DEVICE_MAG_SELECTED == DEVICE_RM3100

#include <string.h>
#include "../../../../logger/Logger.h"
#include "../../../../io/spi/SPI.h"
#include "RM3100Registers.h"

#define RM3100_MAG_DEVICE FC_SPI6_DEVICE1
#define RM3100_MAG_CYCLE_COUNT 200

uint8_t rm3100SetCycleCount();
uint8_t rm3100SetMode(void);
uint8_t rm3100Read(void);
void rm3100ApplyScaling(void);

volatile uint8_t rm3100HasMagData = 0;

uint8_t memsMCheckConnection() {
	uint8_t status = spi6ReadRegister(RM3100_WHO_AM_I_REG, deviceAttitudeData.bufferMagRx, 1, RM3100_MAG_DEVICE);
	if (status) {
		char buf[50];
		sprintf(buf, "[rm3100,WAI] : %X(%X) \n", deviceAttitudeData.bufferMagRx[0], RM3100_WHO_AM_I_REG_VALUE);
		logString(buf);
		status = (deviceAttitudeData.bufferMagRx[0] == RM3100_WHO_AM_I_REG_VALUE);
	}
	return status;
}

void rm3100_reset() {
	// Write to the CMM register with 0x00 to stop all measurements
	deviceAttitudeData.bufferMagTx[0] = 0x00;
	spi6WriteRegister(RM3100_CCX1_REG, deviceAttitudeData.bufferMagTx, 1, RM3100_MAG_DEVICE);
}

uint8_t deviceMAGInit() {
	uint8_t status = spi6Init();
	if (status) {
		logString("[rm3100,IO:SPI] > Success\n");
	} else {
		logString("[rm3100,IO:SPI] > Failed\n");
		return 0;
	}

	logString("[rm3100,MAG:CON] > Checking\n");
	status = memsMCheckConnection();
	if (status) {
		logString("[rm3100,MAG:CON] > Success\n");
	} else {
		logString("[rm3100,MAG:CON] > Failed\n");
		return 0;
	}

	rm3100_reset();

	deviceAttitudeData.magSensitivity = 1.0f / ((0.3671f * (float) RM3100_MAG_CYCLE_COUNT) + 1.5f);

	status = rm3100SetCycleCount();
	if (status) {
		logString("[rm3100,MAG: Set CC] > Success\n");
	} else {
		logString("[rm3100,MAG: Set CC] > Failed\n");
		return 0;
	}

	status = rm3100SetMode();
	if (status) {
		logString("[rm3100,MAG: Set Mode] > Success\n");
	} else {
		logString("[rm3100,MAG: Set Mode] > Failed\n");
		return 0;
	}

	return 1;
}

uint8_t rm3100SetMode() {
	deviceAttitudeData.bufferMagTx[0] = 0x75;
	uint8_t status = spi6WriteRegister(RM3100_CMM_REG, deviceAttitudeData.bufferMagTx, 1, RM3100_MAG_DEVICE);
	return status;
}

void __deviceRM3100Callback(uint8_t *buf, uint16_t len) {
	if (!rm3100HasMagData) {
		memcpy(deviceAttitudeData.bufferMagRx, buf, len);
		rm3100HasMagData = 1;
	}
}

__ATTR_ITCM_TEXT
uint8_t deviceMagLoadData() {
#if RM3100_READ_ASYNC == 1
	if (rm3100HasMagData) {

		deviceAttitudeData.rawMx = ((signed char) deviceAttitudeData.bufferMagRx[0]) << 16;
		deviceAttitudeData.rawMx |= deviceAttitudeData.bufferMagRx[1] << 8;
		deviceAttitudeData.rawMx |= deviceAttitudeData.bufferMagRx[2];

		deviceAttitudeData.rawMy = ((signed char) deviceAttitudeData.bufferMagRx[3]) << 16;
		deviceAttitudeData.rawMy |= deviceAttitudeData.bufferMagRx[4] << 8;
		deviceAttitudeData.rawMy |= deviceAttitudeData.bufferMagRx[5];

		deviceAttitudeData.rawMz = ((signed char) deviceAttitudeData.bufferMagRx[6]) << 16;
		deviceAttitudeData.rawMz |= deviceAttitudeData.bufferMagRx[7] << 8;
		deviceAttitudeData.rawMz |= deviceAttitudeData.bufferMagRx[8];

		rm3100HasMagData = 0;

		return 1;
	}
	return 0;
#else
	return 1;
#endif
}

uint8_t rm3100Read() {
#if RM3100_READ_ASYNC == 1
	if (spi6ReadRegisterAsync(RM3100_MEASUREMENT_REG, 9, RM3100_MAG_DEVICE, __deviceRM3100Callback)) {
		return 1;
	}
#else
	if (spi6ReadRegister(RM3100_MEASUREMENT_REG, deviceAttitudeData.bufferMagRx, 9, RM3100_MAG_DEVICE)) {
		deviceAttitudeData.rawMx = ((signed char) deviceAttitudeData.bufferMagRx[0]) << 16;
		deviceAttitudeData.rawMx |= deviceAttitudeData.bufferMagRx[1] << 8;
		deviceAttitudeData.rawMx |= deviceAttitudeData.bufferMagRx[2];

		deviceAttitudeData.rawMy = ((signed char) deviceAttitudeData.bufferMagRx[3]) << 16;
		deviceAttitudeData.rawMy |= deviceAttitudeData.bufferMagRx[4] << 8;
		deviceAttitudeData.rawMy |= deviceAttitudeData.bufferMagRx[5];

		deviceAttitudeData.rawMz = ((signed char) deviceAttitudeData.bufferMagRx[6]) << 16;
		deviceAttitudeData.rawMz |= deviceAttitudeData.bufferMagRx[7] << 8;
		deviceAttitudeData.rawMz |= deviceAttitudeData.bufferMagRx[8];
		return 1;
	}
#endif
	return 0;
}

void rm3100ApplyScaling() {
	deviceAttitudeData.mx = deviceAttitudeData.rawMx * deviceAttitudeData.magSensitivity;
	deviceAttitudeData.my = deviceAttitudeData.rawMy * deviceAttitudeData.magSensitivity;
	deviceAttitudeData.mz = deviceAttitudeData.rawMz * deviceAttitudeData.magSensitivity;
}

uint8_t rm3100SetCycleCount() {
	// Default cycle count is 200
	if (RM3100_MAG_CYCLE_COUNT != 200) {
		uint8_t CCMSB = (RM3100_MAG_CYCLE_COUNT & 0xFF00) >> 8; // get the most significant byte
		uint8_t CCLSB = RM3100_MAG_CYCLE_COUNT & 0xFF;			// get the least significant byte

		deviceAttitudeData.bufferMagTx[0] = CCMSB;
		deviceAttitudeData.bufferMagTx[1] = CCLSB;

		deviceAttitudeData.bufferMagTx[2] = CCMSB;
		deviceAttitudeData.bufferMagTx[3] = CCLSB;

		deviceAttitudeData.bufferMagTx[4] = CCMSB;
		deviceAttitudeData.bufferMagTx[5] = CCLSB;

		uint8_t status = spi6WriteRegister(RM3100_CCX1_REG, deviceAttitudeData.bufferMagTx, 6, RM3100_MAG_DEVICE);
		return status;
	} else {
		return 1;
	}
}

uint8_t deviceMagReadOffset() {
	// No Implementation
	return 1;
}

uint8_t deviceMagRead() {
	return rm3100Read();
}

void deviceMagApplyOrientationForImu() {
	deviceAttitudeData.mx = -deviceAttitudeData.mx;
}

void deviceMagApplyOffsetCorrection(void) {
	/*
	 memsData.mx -= memsData.offsetMx;
	 memsData.my -= memsData.offsetMy;
	 memsData.mz -= memsData.offsetMz;
	 */

	// Apply the bias if already set
	deviceAttitudeData.mx -= deviceAttitudeData.biasMx;
	deviceAttitudeData.my -= deviceAttitudeData.biasMy;
	deviceAttitudeData.mz -= deviceAttitudeData.biasMz;

	// Apply the scales for each axis

	deviceAttitudeData.mx *= deviceAttitudeData.scaleMx;
	deviceAttitudeData.my *= deviceAttitudeData.scaleMy;
	deviceAttitudeData.mz *= deviceAttitudeData.scaleMz;
}

void deviceMagApplyDataScaling(void) {
	rm3100ApplyScaling();
}

#endif
