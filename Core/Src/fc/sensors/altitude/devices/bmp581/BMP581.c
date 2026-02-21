
#include "../AltitudeDevice.h"
#if BARO_SENSOR_SELECTED == BARO_SENSOR_BMP581
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_ll_utils.h"
#include "../../../../io/spi/SPI.h"
#include "../../../../logger/Logger.h"
#include "../../../../timers/DelayTimer.h"
#include "BMP581.h"
#include "BMP581Registers.h"

#define BMP581_DEVICE FC_SPI4_DEVICE1

/* ------------------------------------------------------------------
 * Local state
 * ------------------------------------------------------------------ */
volatile uint8_t bmp581HasData = 0;

uint8_t bmp581Configure(void);

/* ------------------------------------------------------------------
 * Connection check
 * ------------------------------------------------------------------ */
uint8_t baroCheckConnection(void) {
	uint8_t status;
	status = spi4ReadRegister(BMP581_REG_CHIP_ID, deviceAltitudeData.buffer, 1, BMP581_DEVICE);
	if (!status) {
		logString("[bmp581,WAI] SPI read failed\n");
		return 0;
	}

	char buf[64];
	sprintf(buf, "[bmp581,WAI] : %02X (expected %02X)\n", deviceAltitudeData.buffer[0], BMP581_CHIP_ID_VALUE);
	logString(buf);

	return (deviceAltitudeData.buffer[0] == BMP581_CHIP_ID_VALUE);
}

/* ------------------------------------------------------------------
 * Data-ready check (safe)
 * ------------------------------------------------------------------ */
uint8_t baroCheckDataReady(void) {
	uint8_t reg;
	delayMs(100);
	if (!spi4ReadRegister(BMP581_REG_STATUS, &reg, 1, BMP581_DEVICE)) {
		return 0;
	}
	return (reg & BMP581_STATUS_DRDY_DATA) ? 1U : 0U;
}

void deviceBaroReadIntStatus() {
	deviceAltitudeData.buffer[0] = BMP581_SOFT_RESET_CMD;
	spi4ReadRegister(BMP581_REG_INT_STATUS, deviceAltitudeData.buffer, 1, BMP581_DEVICE);
	delayMs(10);
}

/* ------------------------------------------------------------------
 * Soft reset
 * ------------------------------------------------------------------ */
uint8_t deviceBaroReset(void) {
	deviceAltitudeData.buffer[0] = BMP581_SOFT_RESET_CMD;
	if (spi4WriteRegister(BMP581_REG_CMD, deviceAltitudeData.buffer, 1, BMP581_DEVICE)) {
		delayMs(300);
		deviceBaroReadIntStatus();
		return 1;
	}
	return 0;
}

/* ------------------------------------------------------------------
 * Initialization
 * ------------------------------------------------------------------ */
uint8_t deviceBaroInit(void) {
	if (!spi4Init()) {
		logString("[bmp581,IO:SPI] > Failed\n");
		return 0;
	}
	logString("[bmp581,IO : SPI] > Success\n");
	delayMs(10);

	if (!baroCheckConnection()) {
		logString("[bmp581,Baro:CON] > Failed\n");
		return 0;
	}
	logString("[bmp581,Baro:CON] > Success\n");

	if (!deviceBaroReset()) {
		logString("[bmp581,Baro:Reset] > Failed\n");
		return 0;
	}
	logString("[bmp581,Baro:Reset] > Success\n");


	if (!bmp581Configure()) {
		logString("[bmp581,CFG] > Failed\n");
		return 0;
	}
	logString("[bmp581,CFG] > Success\n");

	if (!baroCheckDataReady()) {
		logString("[bmp581,Baro : Ready Status] > Failed\n");
		return 0;
	}
	logString("[bmp581,Baro : Ready Status] > Success\n");

	delayMs(200);
	return 1;
}

/* ------------------------------------------------------------------
 * Sensor configuration
 * ------------------------------------------------------------------ */
uint8_t bmp581ConfigureNew(void) {

	/* OSR_CONFIG (0x36)
	 +-----+-------+-------+-------+-------+-------+---------+-------+-------+
	 | Bit     |   7 |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
	 +-----+-------+-- -----+-------+-------+-------+--------+-------+-------+
	 | Content |Resv | p_en  |        osr_p          |        osr_t          |
	 +-----------------------------------------------------------------------+
	 > osr_t , osr_p(bit offset: 0) OSR_T selection
	 +--------------+-------------------------+
	 | Value        | Description             |
	 +--------------+-------------------------+
	 | 0b000 (0x0)  | oversampling rate = 1x  |<< T
	 | 0b001 (0x1)  | oversampling rate = 2x  |
	 | 0b010 (0x2)  | oversampling rate = 4x  |
	 | 0b011 (0x3)  | oversampling rate = 8x  |
	 | 0b100 (0x4)  | oversampling rate = 16x | << P
	 | 0b101 (0x5)  | oversampling rate = 32x |
	 | 0b110 (0x6)  | oversampling rate = 64x |
	 | 0b111 (0x7)  | oversampling rate = 128x|
	 +--------------+-----------------------+
	 */
	deviceAltitudeData.buffer[0] = 0b01100000;
	if (!spi4WriteRegister(BMP581_REG_OSR_CONFIG, deviceAltitudeData.buffer, 1, BMP581_DEVICE)) {
		logString("[bmp581] OSR_CONFIG write failed\n");
		return 0;
	}
	delayMs(10);

	/* ODR_CONFIG (0x37)
	 +-----+-------+-------+-------+-------+-------+-------+-------+-------------+
	 | Bit     |   7     |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
	 +-----+-------+-------+-------+-------+-------+-------+-------+-------------+
	 | Content | deep_dis|        odr                            | pwr_mode      |
	 +---------------------------------------------------------------------------+
	 > pwr_mode (bit offset: 0) Power mode configuration
	 +------------+--------------------------------------------------- ----------+
	 | Value      | Description                                                  |
	 +------------+-------------------------------------------------------------+
	 | 0b00 (0x0) | Standby mode: no measurement ongoing                        |
	 | 0b01 (0x1) | Normal mode: measurement in configured ODR grid             |
	 | 0b10 (0x2) | Forced mode: forced single measurement                      |
	 | 0b11 (0x3) | Non-Stop mode: repetitive measurements without further      |
	 |            | duty-cycling                                              |
	 +------------+-----------------------------------------------------------+
	 > odr (bit offset: 2) ODR Selection
	 +--------+--------+-------------------------+
	 | Hex    | Binary | Frequency               |
	 +--------+--------+-------------------------+
	 | 0x0    | 0000   | 240.000 Hz (Error = 0.00)|
	 | 0x1    | 0001   | 218.537 Hz (Error = 0.87)|
	 | 0x2    | 0010   | 199.114 Hz (Error = 0.81)|
	 | 0x3    | 0011   | 180.000 Hz (Error = 0.44)|
	 | 0x4    | 0100   | 160.000 Hz (Error = 0.00)|
	 | 0x5    | 0101   | 150.000 Hz (Error = 0.00)|
	 | 0x6    | 0110   | 140.000 Hz (Error = 0.00)|
	 | 0x7    | 0111   | 129.855 Hz (Error = 0.11)|
	 | 0x8    | 1000   | 120.000 Hz (Error = 0.00)|
	 | 0x9    | 1001   | 110.164 Hz (Error = 0.15)|
	 | 0xA    | 1010   | 100.299 Hz (Error = 0.30)|<<
	 | 0xB    | 1011   |  90.000 Hz (Error = 0.00)|
	 | 0xC    | 1100   |  80.000 Hz (Error = 0.00)|
	 | 0xD    | 1101   |  70.000 Hz (Error = 0.00)|
	 | 0xE    | 1110   |  60.000 Hz (Error = 0.00)|
	 | 0xF    | 1111   |  50.056 Hz (Error = 0.11)|
	 +--------+--------+-------------------------+

	 * */
	deviceAltitudeData.buffer[0] = 0b10101001;
	if (!spi4WriteRegister(BMP581_REG_ODR_CONFIG, deviceAltitudeData.buffer, 1, BMP581_DEVICE)) {
		logString("[bmp581] ODR_CONFIG write failed\n");
		return 0;
	}
	delayMs(10);

	/* DSP_IIR Config
	 +-----+-------+-------+-------+-------+-------+-------+-------+-----------+
	 | Bit     |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
	 +-----+-------+-------+-------+-------+-------+-------+-----------+-------+
	 | Content |   reserved_3  |     iir_p             |          iir_t        |
	 +-------------------------------------------------------------------------+
	 > iir_t , iir_p (bit offset: 0) IIR_T selection
	 +--------+----------------------+
	 | Value  | Description          |
	 +--------+----------------------+
	 | 0b000  | Bypass               |
	 | 0b001  | Filter Coefficient 1 |
	 | 0b010  | Filter Coefficient 3 |
	 | 0b011  | Filter Coefficient 7 |
	 | 0b100  | Filter Coefficient 15|
	 | 0b101  | Filter Coefficient 31|
	 | 0b110  | Filter Coefficient 63|
	 | 0b111  | Filter Coefficient127|
	 +--------+----------------------+
	 */
	deviceAltitudeData.buffer[0] = 0b00001001;
	if (!spi4WriteRegister(BMP581_REG_DSP_IIR, deviceAltitudeData.buffer, 1, BMP581_DEVICE)) {
		logString("[bmp581] DSP_IIR write failed\n");
		return 0;
	}
	delayMs(10);

	return 1;
}

uint8_t bmp581Configure() {
	uint8_t data;
	/* ---------------------------------------------------------------
	 * OSR_CONFIG (0x36)
	 * Bit 7   : reserved = 0
	 * Bit 6   : press_en = 1 → enable pressure measurement
	 * Bits 5:3: osr_p = 0b010 (×4 oversampling)
	 * Bits 2:0: osr_t = 0b000 (×1 oversampling)
	 * => 0b0101_0000 = 0x50
	 */
	data = 0x50;
	spi4WriteRegister(BMP581_REG_OSR_CONFIG, &data, 1, BMP581_DEVICE);

	/* ---------------------------------------------------------------
	 * ODR_CONFIG (0x37)
	 * Bit 7   : deep_dis = 0 (normal deep standby)
	 * Bits 6:3: odr = 0b1001 (200 Hz)
	 * Bits 2:0: pwr_mode = 0b011 (NORMAL continuous mode)
	 * => 0b0100_1011 = 0x4B
	 */
	data =0x4B;
	spi4WriteRegister(BMP581_REG_ODR_CONFIG, &data, 1, BMP581_DEVICE);

	data = 0x0;
	//	spi4WriteRegister(BMP581_REG_DSP_CONFIG, &data, 1, BMP581_DEVICE);


	/* ---------------------------------------------------------------
	 * DSP_IIR (0x31)
	 * Bits 7:6: reserved = 0
	 * Bits 5:3: set_iir_p = 0b010 (Filter coefficient 4)
	 * Bits 2:0: set_iir_t = 0b010 (Filter coefficient 3)
	 * => 0b0001_0010 = 0x12
	 */
	data = 0x12;
	//spi4WriteRegister(BMP581_REG_DSP_IIR, &data, 1, BMP581_DEVICE);
	/* ---------------------------------------------------------------
	 * Wait for configuration to apply
	 * --------------------------------------------------------------- */
	delayMs(200);

	return 1;
}

void deviceBaroDataProcess() {
	deviceAltitudeData.rawTemperature = (int32_t) ((int32_t) ((uint32_t) (((uint32_t) deviceAltitudeData.buffer[2] << 16) | ((uint16_t) deviceAltitudeData.buffer[1] << 8) | deviceAltitudeData.buffer[0]) << 8) >> 8);
	deviceAltitudeData.rawPressure = (uint32_t) ((uint32_t) (deviceAltitudeData.buffer[5] << 16) | (uint16_t) (deviceAltitudeData.buffer[4] << 8) | deviceAltitudeData.buffer[3]);

	deviceAltitudeData.temperature = (float) deviceAltitudeData.rawTemperature / 65536.0f;
	deviceAltitudeData.pressure = (float) deviceAltitudeData.rawPressure / 64.0f;

	if (deviceAltitudeData.pressure > 0.0f) {
		deviceAltitudeData.altitude = 44330.0f * (1.0f - powf(deviceAltitudeData.pressure / SEA_LEVEL_PRESSURE_PA, 0.1903f));
	} else {
		deviceAltitudeData.altitude = 0.0f;
	}

}

/* ------------------------------------------------------------------
 * Raw-to-physical conversion
 * ------------------------------------------------------------------ */
uint8_t deviceBaroLoadData(void) {
#if BMP_READ_ASYNC == 1
	if (bmp581HasData) {
		deviceBaroDataProcess();
		bmp581HasData = 0;
		return 1;
	} else {
		return 0;
	}
#else
	return 1;
#endif
}

/* ------------------------------------------------------------------
 * Async SPI callback
 * ------------------------------------------------------------------ */
void __deviceBaroBMP581Callback(uint8_t *buf, uint16_t len) {
	if (!bmp581HasData) {
		memcpy(deviceAltitudeData.buffer, buf, len);
		bmp581HasData = 1;
	}
}

/* ------------------------------------------------------------------
 * Read sensor data
 * ------------------------------------------------------------------ */
uint8_t deviceBaroRead(void) {
#if BMP_READ_ASYNC == 1
	if (spi4ReadRegisterAsync(BMP581_REG_TEMP_DATA_XLSB, 6, BMP581_DEVICE, __deviceBaroBMP581Callback)) {
		return 1;
	}
#else
	if (spi4ReadRegister(BMP581_REG_TEMP_DATA_XLSB, deviceAltitudeData.buffer, 6, BMP581_DEVICE)) {
		deviceBaroDataProcess();
		return 1;
	}
#endif
	return 0;
}

#endif
