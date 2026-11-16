#include "TFMini.h"
#if TFMINI_IO_TYPE == TFMINI_IO_TYPE_I2C
#include "../AltitudeDevice.h"

TFMini tfMini;

volatile uint8_t tfMiniHasData = 0;
volatile uint8_t tfMiniDataRequestComplete = 0;

uint8_t tfMiniCmdRequestData[5] = { 0x5A, 0x05, 0x00, 0x01, 0x60 };

void __deviceLidarTFMiniDataReadCallback(uint8_t *buf, uint16_t len) {
	/* buf == NULL: the transfer failed (NACK, bus error, DMA error). Do not set tfMiniHasData,
	 * or the old frame (valid checksum) would be reprocessed as fresh data. A length other
	 * than 9 means this was not our frame, so ignore it as well. */
	if (buf == NULL || len != 9) {
		return;
	}
	if (!tfMiniHasData) {
		memcpy(tfMini.buffer, buf, len);
		tfMiniHasData = 1;
	}
}

uint8_t tfMiniReadDataAsync() {
	return i2c1ReadAsync(TFMINI_DEFAULT_ADDRESS, 9, __deviceLidarTFMiniDataReadCallback);
}

void __deviceLidarTFMiniDataRequestCallback(uint8_t *buf, uint16_t len) {
	if (buf == NULL || len != 5) {
		return; /* request failed: leave the flag clear so the next call sends it again */
	}
	tfMiniDataRequestComplete = 1;
}

uint8_t tfMiniRequestDataAsync() {
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
	tfMini.buffer[3] = 0x6F;   // was 0x67
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

uint8_t tfMiniWaitForDevice(uint16_t timeoutMs) {
	for (uint16_t t = 0; t < timeoutMs; t += 10) {
		if (i2c1CheckAddress(TFMINI_DEFAULT_ADDRESS))
			return 1;
		delayMs(10);
	}
	return 0;
}


/* Switch the TFMini Plus from I2C to UART output (command ID 0x0A, mode 0 = UART).
 * Call it while the sensor still answers on I2C. Returns 1 if the commands were accepted,
 * 0 if the sensor did not answer on I2C (already in UART mode, or not connected) or a write failed. */
uint8_t tfMiniSwitchToUart() {
	uint8_t cmdSetUart[5] = { 0x5A, 0x05, 0x0A, 0x00, 0x69 }; /* interface = UART */
	uint8_t cmdSave[4]    = { 0x5A, 0x04, 0x11, 0x6F };       /* save settings */
	uint8_t cmdReset[4]   = { 0x5A, 0x04, 0x02, 0x60 };       /* system reset */

	if (!i2c1CheckAddress(TFMINI_DEFAULT_ADDRESS)) {
		logString("[TFMini] Not on I2C (already UART or not connected)\n");
		return 0;
	}
	if (!i2c1Write(TFMINI_DEFAULT_ADDRESS, cmdSetUart, 5)) {
		logString("[TFMini] UART mode command failed\n");
		return 0;
	}
	delayMs(100);               /* manual: wait ~100 ms after a configuration frame */
	if (!i2c1Write(TFMINI_DEFAULT_ADDRESS, cmdSave, 4)) {
		logString("[TFMini] Save after UART switch failed\n");
		return 0;
	}
	delayMs(100);
	i2c1Write(TFMINI_DEFAULT_ADDRESS, cmdReset, 4); /* best effort: ignored if the sensor already left I2C */
	delayMs(1000);              /* manual: keep power on and wait 1 s after a system reset */
	logString("[TFMini] Switched to UART mode\n");
	return 1;
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

#if TFMINI_SWITCH_TO_UART ==1
	tfMiniSwitchToUart();
	return 0;
#endif

	i2c1_ScanBus();
	status = i2c1CheckAddress(TFMINI_DEFAULT_ADDRESS);
	if (!status) {
		logString("[TFMini] I2C Check Failed\n");
		return 0;
	} else {
		logString("[TFMini] I2C Check Success\n");
	}
	delayMs(100);
	status = tfMiniResetDevice();
	if (!status) {
		logString("[TFMini] Reset Failed\n");
		return 0;
	} else {
		logString("[TFMini] Reset Success\n");
	}

	delayMs(50);

	if (!tfMiniWaitForDevice(1000)) {
		logString("[TFMini] Not back after reset\n");
		return 0;
	}
	delayMs(20);

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
		deviceAltitudeData.altitudeTerrainQlty = mapAndClampToRangeFloat(tfMini.strength == 65535 ? 0 : tfMini.strength, TFMINI_MIN_VALID_STRENGTH, TFMINI_MAX_VALID_STRENGTH, 0.0f, 1.0f);
		if (deviceAltitudeData.altitudeTerrainQlty != 0.0f) {
			deviceAltitudeData.altitudeTerrain = tfMini.distance;
		}
	} else {
		deviceAltitudeData.altitudeTerrainQlty = 0;
	}
}

uint8_t deviceLidarRead(void) {
	if (tfMiniDataRequestComplete) {
		tfMiniDataRequestComplete = 0;
		if (!tfMiniReadDataAsync()) {
			/* Bus busy (e.g. the INA260 has it): retry the read on the next call
			 * instead of dropping the request and wasting a whole frame */
			tfMiniDataRequestComplete = 1;
		}
	} else {
		tfMiniRequestDataAsync();
	}
	return 1;
}

uint8_t deviceLidarLoadData(void) {
	if (!tfMiniHasData) {
		return 0;
	}
	deviceLidarDataProcess();
	tfMiniHasData = 0;
	return 1;
}

uint8_t deviceLidarReset(uint8_t hard) {
	deviceAltitudeData.altitudeTerrain = 0;
	deviceAltitudeData.altitudeTerrainQlty = 0;
	return 1;
}

#endif
