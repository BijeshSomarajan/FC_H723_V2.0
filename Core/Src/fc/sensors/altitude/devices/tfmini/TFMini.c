#include "TFMini.h"
#include "../AltitudeDevice.h"

#include <string.h>
#include <stdio.h>

TFMini tfMini;

// Dual-Flag Asynchronous State Machine Pattern
volatile uint8_t tfMiniTxInFlight = 0; // Bus lock to prevent multi-stage write/read overlapping
volatile uint8_t tfMiniRequestComplete = 0; // Phase flag tracking the transition from Request to Read
volatile uint8_t tfMiniDataReady = 0; // Signal to consumer loop that data is safe to process

uint8_t tfMiniCmdRequestData[5] = { 0x5A, 0x05, 0x00, 0x01, 0x60 };

/* ------------------------------------------------------------------ */
/* Async Callbacks                                                    */
/* ------------------------------------------------------------------ */
void __deviceLidarTFMiniDataRequestCallback(uint8_t *buf, uint16_t len) {
	tfMiniRequestComplete = 1; // Write sequence complete, sensor ready for reading
	tfMiniTxInFlight = 0;      // Release bus lock for the read phase
}

void __deviceLidarTFMiniDataReadCallback(uint8_t *buf, uint16_t len) {
	memcpy(tfMini.buffer, buf, len);
	tfMiniDataReady = 1;       // Safe vector inside memory for data process execution
	tfMiniRequestComplete = 0; // Clear sequence state tracker for next frame loop
	tfMiniTxInFlight = 0;      // Release bus lock entirely
}

/* ------------------------------------------------------------------ */
/* Asynchronous Pipeline Actions                                      */
/* ------------------------------------------------------------------ */
uint8_t tfMiniRequestDataAsync() {
	return i2c1WriteAsync(TFMINI_DEFAULT_ADDRESS, tfMiniCmdRequestData, 5, __deviceLidarTFMiniDataRequestCallback);
}

uint8_t tfMiniReadDataAsync() {
	return i2c1ReadAsync(TFMINI_DEFAULT_ADDRESS, 9, __deviceLidarTFMiniDataReadCallback);
}

/* ------------------------------------------------------------------ */
/* Synchronous Operations                                             */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/* Initialization & Processing                                        */
/* ------------------------------------------------------------------ */
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

	tfMiniTxInFlight = 0;
	tfMiniRequestComplete = 0;
	tfMiniDataReady = 0;

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

/* ------------------------------------------------------------------ */
/* Core Loop Interface                                                */
/* ------------------------------------------------------------------ */
uint8_t deviceLidarRead(void) {
#if TFMINI_READ_ASYNC == 1
	// Guard execution if an explicit bus transaction is actively moving bytes or data is pending consume
	if (tfMiniTxInFlight || tfMiniDataReady) {
		return 0;
	}

	if (tfMiniRequestComplete) {
		// Stage 2: Write phase completed, trigger the data extraction read over I2C1
		tfMiniTxInFlight = 1;
		if (!tfMiniReadDataAsync()) {
			tfMiniTxInFlight = 0;
			return 0;
		}
	} else {
		// Stage 1: Pipeline clear, trigger the asynchronous data request command burst
		tfMiniTxInFlight = 1;
		if (!tfMiniRequestDataAsync()) {
			tfMiniTxInFlight = 0;
			return 0;
		}
	}
	return 1;
#else
	if (tfMiniRequestData()) {
		if (tfMiniReadData()) {
			deviceLidarDataProcess();
			return 1;
		} else {
			return 0;
		}
	} else {
		return 0;
	}
#endif
}

uint8_t deviceLidarLoadData(void) {
#if TFMINI_READ_ASYNC == 1
	if (tfMiniDataReady) {
		deviceLidarDataProcess();
		tfMiniDataReady = 0; // Clear data slot flag to restart request pipeline
		return 1;
	}
	return 0;
#else
	return 0;
#endif
}

uint8_t deviceLidarReset(uint8_t hard) {
	deviceAltitudeData.altitudeTerrain = 0;
	deviceAltitudeData.altitudeTerrainQlty = 0;
	tfMiniTxInFlight = 0;
	tfMiniRequestComplete = 0;
	tfMiniDataReady = 0;
	return 1;
}
