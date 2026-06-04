#include "TFMini.h"
#include "../AltitudeDevice.h"

TFMini tfMini;

volatile uint8_t tfMiniHasData = 0;
volatile uint8_t tfMiniDataRequestComplete = 0;

uint8_t tfMiniCmdRequestData[5] = {0x5A, 0x05, 0x00, 0x01, 0x60};

void __deviceLidarTFMiniDataReadCallback(uint8_t *buf, uint16_t len) {
	if (!tfMiniHasData) {
		memcpy(tfMini.buffer, buf, len);
		tfMiniHasData = 1;
	}
	tfMiniDataRequestComplete = 0;
}

uint8_t tfMiniReadDataAsync() {
	return i2c1ReadAsync(TFMINI_DEFAULT_ADDRESS, 9, __deviceLidarTFMiniDataReadCallback);
}

void __deviceLidarTFMiniDataRequestCallback(uint8_t *buf, uint16_t len) {
	tfMiniDataRequestComplete = 1;
}

uint8_t tfMiniRequestDataAsync() {
	tfMiniDataRequestComplete = 0;
	/*
	tfMini.buffer[0] = 0x5A;
	tfMini.buffer[1] = 0x05;
	tfMini.buffer[2] = 0x00;
	tfMini.buffer[3] = 0x01;
	tfMini.buffer[4] = 0x60;
	return i2c1WriteAsync(TFMINI_DEFAULT_ADDRESS, tfMini.buffer, 5, __deviceLidarTFMiniDataRequestCallback);
	*/
	return i2c1WriteAsync(TFMINI_DEFAULT_ADDRESS, tfMiniCmdRequestData, 5, __deviceLidarTFMiniDataRequestCallback);
}

uint8_t tfMiniRequestData() {
	tfMini.buffer[0] = 0x5A;
	tfMini.buffer[1] = 0x05;
	tfMini.buffer[2] = 0x00;
	tfMini.buffer[3] = 0x01;
	tfMini.buffer[4] = 0x60;
	return i2c1Write(TFMINI_DEFAULT_ADDRESS, tfMini.buffer, 5);
}

uint8_t tfMiniReadData() {
	return i2c1Read(TFMINI_DEFAULT_ADDRESS, tfMini.buffer, 9);
}

uint8_t tfMiniSaveSettings() {
	tfMini.buffer[0] = 0x5A;
	tfMini.buffer[1] = 0x04;
	tfMini.buffer[2] = 0x11;
	tfMini.buffer[3] = 0x67;
	return i2c1Write(TFMINI_DEFAULT_ADDRESS, tfMini.buffer, 4);
}

uint8_t tfMiniSet100FrameRate() {
	tfMini.buffer[0] = 0x5A;
	tfMini.buffer[1] = 0x06;
	tfMini.buffer[2] = 0x03;
	tfMini.buffer[3] = 0x64;
	tfMini.buffer[4] = 0x00;
	tfMini.buffer[5] = 0xC7;
	return i2c1Write(TFMINI_DEFAULT_ADDRESS, tfMini.buffer, 6);
}

uint8_t tfMiniResetDevice() {
	tfMini.buffer[0] = 0x5A;
	tfMini.buffer[1] = 0x04;
	tfMini.buffer[2] = 0x02;
	tfMini.buffer[3] = 0x60;
	return i2c1Write(TFMINI_DEFAULT_ADDRESS, tfMini.buffer, 4);
}

uint8_t deviceLidarInit() {
	uint8_t status = 0;
	status = initI2C1();
	if (!status) {
		logString("[TFMini] I2C Init Failed\n");
		return 0;
	} else {
		logString("[TFMini] I2C Init Success\n");
	}
	i2c1_ScanBus();
	status = i2c1CheckAddress(TFMINI_DEFAULT_ADDRESS);
	if (!status) {
		logString("[TFMini] I2C Check Failed\n");
		return 0;
	} else {
		logString("[TFMini] I2C Check Success\n");
	}

	status = tfMiniResetDevice();
	if (!status) {
		logString("[TFMini] Reset Failed\n");
		return 0;
	} else {
		logString("[TFMini] Reset Success\n");
	}
	delayMs(50);

	status = tfMiniSet100FrameRate();
	if (!status) {
		logString("[TFMini] Frame Rate Config Failed\n");
		return 0;
	} else {
		logString("[TFMini] Frame Rate Config Success\n");
	}

	status = tfMiniSaveSettings();
	if (!status) {
		logString("[TFMini] Config Save Failed\n");
		return 0;
	} else {
		logString("[TFMini] Config Save Success\n");
	}
	delayMs(10);
	return status;
}

void deviceLidarDataProcess(void) {
	uint8_t ckSum = tfMini.buffer[8];
	uint16_t ckSumCalc = 0;
	for (uint8_t indx = 0; indx < 8; indx++) {
		ckSumCalc += tfMini.buffer[indx];
	}
	if ((ckSumCalc & 0xFF) == ckSum) {
		tfMini.distance = ((float) ((uint16_t) (tfMini.buffer[2] + ((uint16_t) tfMini.buffer[3] << 8)))) / 100.0f;
		tfMini.strength = (uint16_t) (tfMini.buffer[4] + ((uint16_t) tfMini.buffer[5] << 8));
		deviceAltitudeData.altitudeTerrain = tfMini.distance;
		deviceAltitudeData.altitudeTerrainQlty = (float) tfMini.strength / (float) TFMINI_MAX_STRENGTH;
	} else {
		deviceAltitudeData.altitudeTerrain = 0;
		deviceAltitudeData.altitudeTerrainQlty = 0;
	}

}


uint8_t deviceLidarRead(void) {
#if TFMINI_READ_ASYNC == 1
	if(tfMiniDataRequestComplete){
		tfMiniReadDataAsync();
	}else{
		tfMiniRequestDataAsync();
	}
#else
	if (tfMiniRequestData()) {
		if (tfMiniReadData()) {
			deviceLidarDataProcess();
			tfMiniHasData = 1;
			return 1;
		} else {
			return 0;
		}
	} else {
		return 0;
	}
#endif
	return 1;
}

uint8_t deviceLidarLoadData(void) {
#if TFMINI_READ_ASYNC == 1
	if (!tfMiniHasData) {
		return 0;
	}
	deviceLidarDataProcess();
	tfMiniHasData = 0;
	return 1;
#else
	return 0;
#endif
}

uint8_t deviceLidarReset(uint8_t hard) {
	deviceAltitudeData.altitudeTerrain = 0;
	deviceAltitudeData.altitudeTerrainQlty = 0;
	return 1;
}

