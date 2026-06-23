#include <sys/_stdint.h>

#include "../AltitudeDevice.h"

#if BARO_SENSOR_SELECTED == BARO_SENSOR_BMP581

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "stm32h7xx_ll_utils.h"
#include "../../../../io/spi/SPI.h"
#include "../../../../logger/Logger.h"
#include "../../../../timers/DelayTimer.h"
#include "../../../../util/CommonUtil.h"
#include "../../../../util/MathUtil.h"

#include "BMP581.h"
#include "BMP581Registers.h"

#define BMP581_DEVICE FC_SPI4_DEVICE1
#define CALIB_SAMPLES 100

/* ------------------------------------------------------------------ */
/* Local State                                                        */
/* ------------------------------------------------------------------ */
// Dual-Flag State Machine Pattern
static volatile uint8_t bmp581TxInFlight = 0; // Lockout to prevent overlapping SPI DMA requests
static volatile uint8_t bmp581DataReady  = 0; // Signals when buffer contains new, completely copied data

static float bmp581GroundPressure = 0.0f;
static uint8_t bmp581IsCalibrated = 0;

static float bmp581CalibPSum = 0.0f;
static int bmp581CalibCount = 0;

/* ------------------------------------------------------------------ */
/* Forward Declarations                                               */
/* ------------------------------------------------------------------ */
static uint8_t bmp581Configure(void);
static void bmp581CalculateAltitude(void);

/* ------------------------------------------------------------------ */
/* Connection Check                                                   */
/* ------------------------------------------------------------------ */
uint8_t baroCheckConnection(void) {
	if (!spi4ReadRegister(BMP581_REG_CHIP_ID, deviceAltitudeData.buffer, 1, BMP581_DEVICE)) {
		logString("[bmp581] SPI read failed\n");
		return 0;
	}
	uint8_t id = deviceAltitudeData.buffer[0];
	char buf[64];
	sprintf(buf, "[bmp581] CHIP_ID: %02X (exp %02X)\n", id, BMP581_CHIP_ID_VALUE);
	logString(buf);
	if (id == BMP581_CHIP_ID_VALUE) {
		return 1;
	} else {
		return 0;
	}
}

/* ------------------------------------------------------------------ */
/* Data Ready Check                                                   */
/* ------------------------------------------------------------------ */
uint8_t baroCheckDataReady(void) {
	uint8_t reg;
	delayMs(50);
	if (!spi4ReadRegister(BMP581_REG_STATUS, &reg, 1, BMP581_DEVICE)) {
		return 0;
	}
	if (reg & BMP581_STATUS_DRDY_DATA) {
		return 1;
	} else {
		return 0;
	}
}

/* ------------------------------------------------------------------ */
/* Reset                                                              */
/* ------------------------------------------------------------------ */
static void deviceBaroReadIntStatus(void) {
	spi4ReadRegister(BMP581_REG_INT_STATUS, deviceAltitudeData.buffer, 1, BMP581_DEVICE);
	delayMs(5);
}

uint8_t deviceBaroReset(uint8_t hard) {
	if (hard) {
		deviceAltitudeData.buffer[0] = BMP581_SOFT_RESET_CMD;
		if (!spi4WriteRegister(BMP581_REG_CMD, deviceAltitudeData.buffer, 1, BMP581_DEVICE)) {
			return 0;
		}
		delayMs(5);
		deviceBaroReadIntStatus();
	}
	bmp581CalibPSum = 0.0f;
	bmp581CalibCount = 0;
	bmp581IsCalibrated = 0;

	// Reset the async states
	bmp581TxInFlight = 0;
	bmp581DataReady = 0;
	return 1;
}

/* ------------------------------------------------------------------ */
/* Initialization                                                     */
/* ------------------------------------------------------------------ */
uint8_t deviceBaroInit(void) {
	if (!spi4Init()) {
		logString("[bmp581] SPI init failed\n");
		return 0;
	}
	if (!deviceBaroReset(1)) {
		logString("[bmp581] Reset failed\n");
		return 0;
	}
	delayMs(5);
	if (!baroCheckConnection()) {
		logString("[bmp581] Connection failed\n");
		return 0;
	}
	if (!bmp581Configure()) {
		logString("[bmp581] Config failed\n");
		return 0;
	}
	if (!baroCheckDataReady()) {
		logString("[bmp581] Data not ready\n");
		return 0;
	}
	bmp581IsCalibrated = 0;
	bmp581CalibPSum = 0.0f;
	bmp581CalibCount = 0;
	logString("[bmp581] Init success\n");
	return 1;
}

uint8_t bmp581Configure(void) {
	deviceAltitudeData.buffer[0] = 0b01011000;
	if (!spi4WriteRegister(BMP581_REG_OSR_CONFIG, deviceAltitudeData.buffer, 1, BMP581_DEVICE)) {
		logString("[bmp581] OSR_CONFIG write failed\n");
		return 0;
	}
	delayMs(5);

	deviceAltitudeData.buffer[0] = 0b10101001;
	if (!spi4WriteRegister(BMP581_REG_ODR_CONFIG, deviceAltitudeData.buffer, 1, BMP581_DEVICE)) {
		logString("[bmp581] ODR_CONFIG write failed\n");
		return 0;
	}
	delayMs(5);

	deviceAltitudeData.buffer[0] = 0b00010010;
	if (!spi4WriteRegister(BMP581_REG_DSP_IIR, deviceAltitudeData.buffer, 1, BMP581_DEVICE)) {
		logString("[bmp581] DSP_IIR write failed\n");
		return 0;
	}
	delayMs(5);
	return 1;
}

/* ------------------------------------------------------------------ */
/* Altitude Calculation                                               */
/* ------------------------------------------------------------------ */
void bmp581CalculateAltitude(void) {
	float currentPreassure = deviceAltitudeData.pressure;
	if (currentPreassure <= 0.0f) {
		deviceAltitudeData.altitudeSL = 0.0f;
		return;
	}
	if (!bmp581IsCalibrated) {
		bmp581CalibPSum += currentPreassure;
		bmp581CalibCount++;
		if (bmp581CalibCount >= CALIB_SAMPLES) {
			bmp581GroundPressure = bmp581CalibPSum / bmp581CalibCount;
			float ratio = bmp581GroundPressure / convertPascalToHectoPascal(BMP581_SEALEVEL_PRESSURE);
			deviceAltitudeData.altitudeSLGround = BMP581_PRESSURE_GAS_CONST * (1.0f - powf(ratio, BMP581_PRESSURE_PWR_CONST));
			bmp581CalibPSum = 0.0f;
			bmp581CalibCount = 0;
			bmp581IsCalibrated = 1;
		}
		deviceAltitudeData.altitudeSL = 0.0f;
		return;
	}
	float ratio = currentPreassure / bmp581GroundPressure;
	if (ratio > 0.9f && ratio < 1.2f) {
		deviceAltitudeData.altitudeSL = BMP581_PRESSURE_GAS_CONST * (1.0f - powf(ratio, BMP581_PRESSURE_PWR_CONST));
	}
}

/* ------------------------------------------------------------------ */
/* Data Processing                                                    */
/* ------------------------------------------------------------------ */
void deviceBaroDataProcess(void) {
	uint8_t *b = deviceAltitudeData.buffer;
	deviceAltitudeData.rawTemperature = (int32_t) ((((uint32_t) b[2] << 16) | ((uint16_t) b[1] << 8) | b[0]) << 8) >> 8;
	deviceAltitudeData.rawPressure = ((uint32_t) b[5] << 16) | ((uint16_t) b[4] << 8) | b[3];
	deviceAltitudeData.temperature = (float) deviceAltitudeData.rawTemperature / 65536.0f;
	deviceAltitudeData.pressure = convertPascalToHectoPascal((float) deviceAltitudeData.rawPressure / 64.0f);
	bmp581CalculateAltitude();
}

/* ------------------------------------------------------------------ */
/* Data Load                                                          */
/* ------------------------------------------------------------------ */
uint8_t deviceBaroLoadData(void) {
#if BMP581_READ_ASYNC == 1
	if (bmp581DataReady) {
		deviceBaroDataProcess();
		bmp581DataReady = 0; // Free the slot for the next transaction execution
		return 1;
	}
	return 0;
#else
    return 1;
#endif
}

/* ------------------------------------------------------------------ */
/* Async Callback                                                     */
/* ------------------------------------------------------------------ */
void __deviceBaroBMP581Callback(uint8_t *buf, uint16_t len) {
	memcpy(deviceAltitudeData.buffer, buf, len);
	bmp581DataReady = 1;   // Data is safely inside memory bounds
	bmp581TxInFlight = 0;  // Open the SPI4 bus line state again
}

/* ------------------------------------------------------------------ */
/* Read                                                               */
/* ------------------------------------------------------------------ */
uint8_t deviceBaroRead(void) {
#if BMP581_READ_ASYNC == 1
	// Decline execution if a DMA burst is active or unread data occupies the slot
	if (bmp581TxInFlight || bmp581DataReady) {
		return 0;
	}

	bmp581TxInFlight = 1; // Secure bus lock before executing async command

	if (!spi4ReadRegisterAsync(BMP581_REG_TEMP_DATA_XLSB, 6, BMP581_DEVICE, __deviceBaroBMP581Callback)) {
		bmp581TxInFlight = 0; // Rollback lock if initialization rejected
		return 0;
	}
	return 1;
#else
    if (!spi4ReadRegister(BMP581_REG_TEMP_DATA_XLSB, deviceAltitudeData.buffer, 6, BMP581_DEVICE)) {
        return 0;
    }
    deviceBaroDataProcess();
    return 1;
#endif
}

#endif
