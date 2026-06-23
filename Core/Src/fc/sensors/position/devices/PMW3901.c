#include "../OFlow.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pmw3901Registers.h"
#include "stm32h7xx_ll_utils.h"
#include "../../../io/spi/SPI.h"
#include "../../../logger/Logger.h"
#include "../../../timers/DelayTimer.h"
#include "../../../util/CommonUtil.h"
#include "../../../util/MathUtil.h"

#define PMW3901_DEVICE FC_SPI1_DEVICE1

#define PMW3901_FOV_DEG      42.0f
#define PMW3901_RES_PX       30.0f  // PMW3901 native imaging array size (30x30)
#define PMW3901_DRIVER_SCALER   14.54f// --- PMW3901 Driver Count Scaling Divisor ---

float pmw3901PxToRad = 1.0f;

/* ------------------------------------------------------------------ */
/* Local State                                                        */
/* ------------------------------------------------------------------ */
// Dual-Flag State Machine Pattern
static volatile uint8_t pmw3901TxInFlight = 0; // Lockout to prevent overlapping SPI DMA requests
static volatile uint8_t pmw3901DataReady = 0; // Signals when buffer contains new, completely copied data

//Global variable
OFlowData oFlowData;

uint8_t pmw3901OpenSPI() {
	return spi1Init();
}

void pmw3901RegisterWrite(uint8_t reg, uint8_t value) {
	oFlowData.buffer[0] = value;
	spi1WriteRegister(reg, oFlowData.buffer, 1, PMW3901_DEVICE);
}

uint8_t pmw3901RegisterRead(uint8_t reg) {
	spi1ReadRegister(reg, oFlowData.buffer, 1, PMW3901_DEVICE);
	return oFlowData.buffer[0];
}

/* ------------------------------------------------------------------ */
/* Connection Check                                                   */
/* ------------------------------------------------------------------ */
uint8_t pmw3901CheckConnection(void) {
	pmw3901RegisterWrite(PMW3901_REG_NAME_POWER_UP_RESET, 0x5A);
	delayMs(10);
	uint8_t wai = pmw3901RegisterRead(PMW3901_REG_NAME_PRODUCT_ID);
	char bufPrint[48];
	sprintf(bufPrint, "[pmw3901 WAI ] : %02X(%02X)\n", wai, PMW3901_REG_VALUE_PRODUCT_ID);
	logString(bufPrint);
	for (uint8_t mReg = 0x02; mReg <= 0x06; mReg++) {
		uint8_t mRegVal = pmw3901RegisterRead(mReg);
		sprintf(bufPrint, "[pmw3901 MReg] : %02X=%02X\n", mReg, mRegVal);
		logString(bufPrint);
	}
	delayMs(10);
	return wai == PMW3901_REG_VALUE_PRODUCT_ID;
}

void pmwSetLed(uint8_t ledOn) {
	delayMs(100);
	pmw3901RegisterWrite(0x7f, 0x14);
	pmw3901RegisterWrite(0x6f, ledOn ? 0x1c : 0x00);
	pmw3901RegisterWrite(0x7f, 0x00);
}

void pmwInitRegisters() {
	pmw3901RegisterWrite(0x7F, 0x00);
	pmw3901RegisterWrite(0x61, 0xAD);
	pmw3901RegisterWrite(0x7F, 0x03);
	pmw3901RegisterWrite(0x40, 0x00);
	pmw3901RegisterWrite(0x7F, 0x05);
	pmw3901RegisterWrite(0x41, 0xB3);
	pmw3901RegisterWrite(0x43, 0xF1);
	pmw3901RegisterWrite(0x45, 0x14);
	pmw3901RegisterWrite(0x5B, 0x32);
	pmw3901RegisterWrite(0x5F, 0x34);
	pmw3901RegisterWrite(0x7B, 0x08);
	pmw3901RegisterWrite(0x7F, 0x06);
	pmw3901RegisterWrite(0x44, 0x1B);
	pmw3901RegisterWrite(0x40, 0xBF);
	pmw3901RegisterWrite(0x4E, 0x3F);
	pmw3901RegisterWrite(0x7F, 0x08);
	pmw3901RegisterWrite(0x65, 0x20);
	pmw3901RegisterWrite(0x6A, 0x18);
	pmw3901RegisterWrite(0x7F, 0x09);
	pmw3901RegisterWrite(0x4F, 0xAF);
	pmw3901RegisterWrite(0x5F, 0x40);
	pmw3901RegisterWrite(0x48, 0x80);
	pmw3901RegisterWrite(0x49, 0x80);
	pmw3901RegisterWrite(0x57, 0x77);
	pmw3901RegisterWrite(0x60, 0x78);
	pmw3901RegisterWrite(0x61, 0x78);
	pmw3901RegisterWrite(0x62, 0x08);
	pmw3901RegisterWrite(0x63, 0x50);
	pmw3901RegisterWrite(0x7F, 0x0A);
	pmw3901RegisterWrite(0x45, 0x60);
	pmw3901RegisterWrite(0x7F, 0x00);
	pmw3901RegisterWrite(0x4D, 0x11);
	pmw3901RegisterWrite(0x55, 0x80);
	pmw3901RegisterWrite(0x74, 0x1F);
	pmw3901RegisterWrite(0x75, 0x1F);
	pmw3901RegisterWrite(0x4A, 0x78);
	pmw3901RegisterWrite(0x4B, 0x78);
	pmw3901RegisterWrite(0x44, 0x08);
	pmw3901RegisterWrite(0x45, 0x50);
	pmw3901RegisterWrite(0x64, 0xFF);
	pmw3901RegisterWrite(0x65, 0x1F);
	pmw3901RegisterWrite(0x7F, 0x14);
	pmw3901RegisterWrite(0x65, 0x60);
	pmw3901RegisterWrite(0x66, 0x08);
	pmw3901RegisterWrite(0x63, 0x78);
	pmw3901RegisterWrite(0x7F, 0x15);
	pmw3901RegisterWrite(0x48, 0x58);
	pmw3901RegisterWrite(0x7F, 0x07);
	pmw3901RegisterWrite(0x41, 0x0D);
	pmw3901RegisterWrite(0x43, 0x14);
	pmw3901RegisterWrite(0x4B, 0x0E);
	pmw3901RegisterWrite(0x45, 0x0F);
	pmw3901RegisterWrite(0x44, 0x42);
	pmw3901RegisterWrite(0x4C, 0x80);
	pmw3901RegisterWrite(0x7F, 0x10);
	pmw3901RegisterWrite(0x5B, 0x02);
	pmw3901RegisterWrite(0x7F, 0x07);
	pmw3901RegisterWrite(0x40, 0x41);
	pmw3901RegisterWrite(0x70, 0x00);

	delayMs(100);
	pmw3901RegisterWrite(0x32, 0x44);
	pmw3901RegisterWrite(0x7F, 0x07);
	pmw3901RegisterWrite(0x40, 0x40);
	pmw3901RegisterWrite(0x7F, 0x06);
	pmw3901RegisterWrite(0x62, 0xf0);
	pmw3901RegisterWrite(0x63, 0x00);
	pmw3901RegisterWrite(0x7F, 0x0D);
	pmw3901RegisterWrite(0x48, 0xC0);
	pmw3901RegisterWrite(0x6F, 0xd5);
	pmw3901RegisterWrite(0x7F, 0x00);
	pmw3901RegisterWrite(0x5B, 0xa0);
	pmw3901RegisterWrite(0x4E, 0xA8);
	pmw3901RegisterWrite(0x5A, 0x50);
	pmw3901RegisterWrite(0x40, 0x80);
}

uint8_t initOFlow(void) {
	if (!pmw3901OpenSPI()) {
		logString("[pmw3901] SPI init failed\n");
		return 0;
	} else {
		logString("[pmw3901] SPI init Success\n");
	}
	if (pmw3901CheckConnection()) {
		logString("[pmw3901] Init Success\n");
		pmwInitRegisters();
		pmwSetLed(1);
		// Evaluates to: (2 * tanf(21°)) / 30 = 0.02559093f rad/pixel
		float theoreticalPxToRad = ((2.0f * tanf((PMW3901_FOV_DEG * PI_BY_180) / 2.0f)) / PMW3901_RES_PX);
		pmw3901PxToRad = theoreticalPxToRad / PMW3901_DRIVER_SCALER;

		pmw3901TxInFlight = 0;
		pmw3901DataReady = 0;

		return 1;
	} else {
		logString("[pmw3901] init failed\n");
	}
	return 0;
}

void resetOFlow() {
	pmw3901TxInFlight = 0;
	pmw3901DataReady = 0;
}

/* ------------------------------------------------------------------ */
/* Data Load                                                          */
/* ------------------------------------------------------------------ */

uint8_t loadOFlowData(void) {
	if (pmw3901DataReady) {
		oFlowData.xRad = oFlowData.deltaRawX * pmw3901PxToRad;
		oFlowData.yRad = oFlowData.deltaRawY * pmw3901PxToRad;
		pmw3901DataReady = 0; // Clear flag to reopen data acquisition pipeline
		return 1;
	} else {
		return 0;
	}
}

/* ------------------------------------------------------------------ */
/* Async Callback                                                     */
/* ------------------------------------------------------------------ */
void __pmw3901Callback(uint8_t *rxData, uint16_t len) {
	oFlowData.motion = rxData[0];        // Register 0x02 data
	oFlowData.observation = rxData[1];   // Register 0x03 data

	// Combine 8-bit registers into signed 16-bit integers
	oFlowData.deltaRawX = (int16_t) (((uint16_t) rxData[3] << 8) | rxData[2]);  // Reg 0x04 & 0x05
	oFlowData.deltaRawY = (int16_t) (((uint16_t) rxData[5] << 8) | rxData[4]);  // Reg 0x06 & 0x07
	oFlowData.qual = rxData[6];                                                // Reg 0x08 (Surface Quality)

	pmw3901DataReady = 1;   // Signal that data parsing has fully completed
	pmw3901TxInFlight = 0;  // Open the SPI1 bus back up
}

/* ------------------------------------------------------------------ */
/* Read                                                               */
/* ------------------------------------------------------------------ */
uint8_t readOFlowData(void) {
	// Decline request if a DMA stream is running or data is waiting to be consumed
	if (pmw3901TxInFlight || pmw3901DataReady) {
		return 0;
	}

	pmw3901TxInFlight = 1; // Engage lock before calling the async driver

	if (!spi1ReadRegisterAsync(PMW3901_REG_NAME_MOTION_BURST, 7, PMW3901_DEVICE, __pmw3901Callback)) {
		pmw3901TxInFlight = 0; // Roll back lock if the peripheral rejected the request
		return 0;
	}
	return 1;
}
