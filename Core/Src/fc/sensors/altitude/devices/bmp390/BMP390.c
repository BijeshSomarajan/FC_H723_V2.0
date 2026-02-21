#include "../AltitudeDevice.h"

#if BARO_SENSOR_SELECTED == BARO_SENSOR_BMP390
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_ll_utils.h"
#include "../../../../io/spi/SPI.h"
#include "../../../../logger/Logger.h"
#include "../../../../timers/DelayTimer.h"
#include "BMP390.h"
#include "BMP390Registers.h"

#define BMP390_SPI4_DEVICE FC_SPI4_DEVICE1
BMP390 bmp390;
volatile uint8_t bmp390HasData = 0;

float bmp390GroundPressure = 0;
float bmp390PressureToMeterFactor = 0;
uint8_t bmp390IsBaroCalibrated = 0;

uint8_t bmpReadCalib(void);
uint8_t bmp390SoftReset(void);
uint8_t bmp390Configure(void);

uint8_t baroCheckConnection() {
	uint8_t status = spi4ReadRegister(BMP390_CHIP_ID_ADDR, deviceAltitudeData.buffer, 2, BMP390_SPI4_DEVICE);
	if (status) {
		char buf[50];
		sprintf(buf, "[bmp390,WAI] : %X(%X)\n", deviceAltitudeData.buffer[1], BMP390_CHIP_ID);
		logString(buf);
		//First byte is dummy
		status = (deviceAltitudeData.buffer[1] == BMP390_CHIP_ID);
	}
	return status;
}

uint8_t deviceBaroInit(void) {
	uint8_t status = spi4Init();
	if (status) {
		logString("[bmp390,IO:SPI] > Success\n");
	} else {
		logString("[bmp390,IO:SPI] > Failed\n");
		return 0;
	}
	status = baroCheckConnection();
	if (status) {
		logString("[bmp390,CON] > Success\n");
	} else {
		logString("[bmp390,CON] > Failed\n");
		return 0;
	}
	if (bmp390SoftReset()) {
		logString("[bmp390,Reset] > Success\n");
		if (bmpReadCalib()) {
			logString("[bmp390, Calib Read] > Success\n");
			if (bmp390Configure()) {
				logString("[bmp390, Settings] > Success\n");
				status = 1;
			} else {
				logString("[bmp390, Settings] > Failed\n");
				return 0;
			}
		} else {
			logString("[bmp390, Calib Read] > Failed\n");
			return 0;
		}
	} else {
		logString("[bmp390, Reset] > Failed\n");
		return 0;
	}
	return status;
}

uint8_t bmpReadCalib() {
	uint8_t status = spi4ReadRegister(BMP390_CALIB_DATA_ADDR, deviceAltitudeData.buffer, BMP390_CALIB_DATA_LEN + 1, BMP390_SPI4_DEVICE);
	if (status) {
		bmp390.t1 = BMP390_CONCAT_BYTES(deviceAltitudeData.buffer[2], deviceAltitudeData.buffer[1]);
		double temp_var = 0.00390625f; //1/2^8
		bmp390.dt1 = ((double) bmp390.t1 / temp_var);

		bmp390.t2 = BMP390_CONCAT_BYTES(deviceAltitudeData.buffer[4], deviceAltitudeData.buffer[3]);
		temp_var = 1073741824.0; //2^30
		bmp390.dt2 = ((double) bmp390.t2 / temp_var);

		bmp390.t3 = (int8_t) deviceAltitudeData.buffer[5];
		temp_var = 281474976710656.0; //2^48
		bmp390.dt3 = ((double) bmp390.t3 / temp_var);

		bmp390.p1 = (int16_t) BMP390_CONCAT_BYTES(deviceAltitudeData.buffer[7], deviceAltitudeData.buffer[6]);
		temp_var = 1048576.0; //2^20
		bmp390.dp1 = ((double) (bmp390.p1 - (16384)) / temp_var);

		bmp390.p2 = (int16_t) BMP390_CONCAT_BYTES(deviceAltitudeData.buffer[9], deviceAltitudeData.buffer[8]);
		temp_var = 536870912.0; //2^29
		bmp390.dp2 = ((double) (bmp390.p2 - (16384)) / temp_var);

		bmp390.p3 = (int8_t) deviceAltitudeData.buffer[10];
		temp_var = 4294967296.0; //2^32
		bmp390.dp3 = ((double) bmp390.p3 / temp_var);

		bmp390.p4 = (int8_t) deviceAltitudeData.buffer[11];
		temp_var = 137438953472.0; //2^37
		bmp390.dp4 = ((double) bmp390.p4 / temp_var);

		bmp390.p5 = BMP390_CONCAT_BYTES(deviceAltitudeData.buffer[13], deviceAltitudeData.buffer[12]);
		temp_var = 0.125; //1/2^3
		bmp390.dp5 = ((double) bmp390.p5 / temp_var);

		bmp390.p6 = BMP390_CONCAT_BYTES(deviceAltitudeData.buffer[15], deviceAltitudeData.buffer[14]);
		temp_var = 64.0; //2^6
		bmp390.dp6 = ((double) bmp390.p6 / temp_var);

		bmp390.p7 = (int8_t) deviceAltitudeData.buffer[16];
		temp_var = 256.0; //2^8
		bmp390.dp7 = ((double) bmp390.p7 / temp_var);

		bmp390.p8 = (int8_t) deviceAltitudeData.buffer[17];
		temp_var = 32768.0; //2^15
		bmp390.dp8 = ((double) bmp390.p8 / temp_var);

		bmp390.p9 = (int16_t) BMP390_CONCAT_BYTES(deviceAltitudeData.buffer[19], deviceAltitudeData.buffer[18]);
		temp_var = 281474976710656.0; //2^48
		bmp390.dp9 = ((double) bmp390.p9 / temp_var);

		bmp390.p10 = (int8_t) deviceAltitudeData.buffer[20];
		temp_var = 281474976710656.0; // 2^48
		bmp390.dp10 = ((double) bmp390.p10 / temp_var);

		bmp390.p11 = (int8_t) deviceAltitudeData.buffer[21];
		temp_var = 36893488147419103232.0; //2^65
		bmp390.dp11 = ((double) bmp390.p11 / temp_var);
	}
	return status;
}

uint8_t bmp390SoftReset() {
	bmp390IsBaroCalibrated = 0;
	deviceAltitudeData.buffer[1] = BMP390_PWR_SOFT_RESET;
	uint8_t status = spi4WriteRegister(BMP390_CMD_ADDR, deviceAltitudeData.buffer, 1, BMP390_SPI4_DEVICE);
	delayMs(10);
	return status;
}

uint8_t bmp390SetOSR() {
	uint8_t status = 0;
	uint8_t osr_t, osr_p;
	if (BMP390_RESOLUTION_TYPE == BMP390_RESOLUTION_TYPE_CUSTOM) {
		osr_t = BMP390_OVERSAMPLING_1X;
		osr_p = BMP390_OVERSAMPLING_8X;
	} else if (BMP390_RESOLUTION_TYPE == BMP390_RESOLUTION_TYPE_LOW_POWER) {
		osr_t = BMP390_OVERSAMPLING_1X;
		osr_p = BMP390_OVERSAMPLING_2X;
	} else if (BMP390_RESOLUTION_TYPE == BMP390_RESOLUTION_TYPE_STD) {
		osr_t = BMP390_OVERSAMPLING_2X;
		osr_p = BMP390_OVERSAMPLING_4X;
	} else if (BMP390_RESOLUTION_TYPE == BMP390_RESOLUTION_TYPE_HIGH) {
		osr_t = BMP390_OVERSAMPLING_4X;
		osr_p = BMP390_OVERSAMPLING_8X;
	} else if (BMP390_RESOLUTION_TYPE == BMP390_RESOLUTION_TYPE_ULTRA_HIGH) {
		osr_t = BMP390_OVERSAMPLING_4X;
		osr_p = BMP390_OVERSAMPLING_16X;
	} else {
		osr_t = BMP390_OVERSAMPLING_4X;
		osr_p = BMP390_OVERSAMPLING_32X;
	}
	deviceAltitudeData.buffer[0] = ((osr_t & 0x07) << 3) | (osr_p & 0x07);
	if (spi4WriteRegister(BMP390_OSR_ADDR, deviceAltitudeData.buffer, 1, BMP390_SPI4_DEVICE)) {
		delayMs(10);
		status = 1;
	}
	return status;
}

uint8_t bmp390SetIIRFilter() {
	uint8_t status = 0;
	uint8_t coeff = 0;
	if (BMP390_FILTER_TYPE == BMP390_FILTER_TYPE_CUSTOM) {
		coeff = BMP390_IIR_FILTER_COEFF_3;
	} else if (BMP390_FILTER_TYPE == BMP390_FILTER_TYPE_NONE) {
		coeff = BMP390_IIR_FILTER_DISABLE;
	} else if (BMP390_FILTER_TYPE == BMP390_FILTER_TYPE_STD) {
		coeff = BMP390_IIR_FILTER_COEFF_1;
	} else if (BMP390_FILTER_TYPE == BMP390_FILTER_TYPE_HIGH) {
		coeff = BMP390_IIR_FILTER_COEFF_3;
	} else if (BMP390_FILTER_TYPE == BMP390_FILTER_TYPE_ULTRA_HIGH) {
		coeff = BMP390_IIR_FILTER_COEFF_7;
	} else {
		coeff = BMP390_IIR_FILTER_COEFF_15;
	}
	deviceAltitudeData.buffer[0] = (coeff & 0x07) << 1;
	if (spi4WriteRegister(BMP390_CONFIG_ADDR, deviceAltitudeData.buffer, 1, BMP390_SPI4_DEVICE)) {
		delayMs(10);
		status = 1;
	}
	return status;
}

uint8_t bmp390SetODR() {
	uint8_t status = 0;
	if (BMP390_RESOLUTION_TYPE == BMP390_RESOLUTION_TYPE_CUSTOM) {
		deviceAltitudeData.buffer[0] = BMP390_ODR_50_HZ;
	} else if (BMP390_RESOLUTION_TYPE == BMP390_RESOLUTION_TYPE_LOW_POWER) {
		deviceAltitudeData.buffer[0] = BMP390_ODR_100_HZ;
	} else if (BMP390_RESOLUTION_TYPE == BMP390_RESOLUTION_TYPE_STD) {
		deviceAltitudeData.buffer[0] = BMP390_ODR_50_HZ;
	} else if (BMP390_RESOLUTION_TYPE == BMP390_RESOLUTION_TYPE_HIGH) {
		deviceAltitudeData.buffer[0] = BMP390_ODR_25_HZ;
	} else if (BMP390_RESOLUTION_TYPE == BMP390_RESOLUTION_TYPE_ULTRA_HIGH) {
		deviceAltitudeData.buffer[0] = BMP390_ODR_12_5_HZ;
	} else if (BMP390_RESOLUTION_TYPE == BMP390_RESOLUTION_TYPE_HIGHEST) {
		deviceAltitudeData.buffer[0] = BMP390_ODR_6_25_HZ;
	}
	if (spi4WriteRegister(BMP390_ODR_ADDR, deviceAltitudeData.buffer, 1, BMP390_SPI4_DEVICE)) {
		delayMs(10);
		status = 1;
	}
	return status;
}

uint8_t bmp390SetPWRControl() {
	uint8_t status = 0;
	deviceAltitudeData.buffer[0] = (uint8_t) BMP390_NORMAL_MODE | (uint8_t) BMP390_PRESSURE_ENABLE | (uint8_t) BMP390_TEMPERATURE_ENABLE;
	if (spi4WriteRegister(BMP390_PWR_CTRL_ADDR, deviceAltitudeData.buffer, 1, BMP390_SPI4_DEVICE)) {
		delayMs(10);
		status = 1;
	}
	return status;
}

uint8_t bmp390Configure() {
	uint8_t status = 0;
	if (bmp390SetOSR()) {
		if (bmp390SetIIRFilter()) {
			if (bmp390SetODR()) {
				if (bmp390SetPWRControl()) {
					status = 1;
				}
			}
		}
	}
	return status;
}

void bmp390CalculateTemp() {
	float tempData1 = (float) (bmp390.uTemperature - bmp390.dt1);
	float tempData2 = (float) (tempData1 * bmp390.dt2);
	deviceAltitudeData.temperature = tempData2 + (tempData1 * tempData1) * bmp390.dt3;
}

void bmp390CalculatePressure() {
	double partial_data1 = bmp390.dp6 * deviceAltitudeData.temperature;

	double tP2 = (double) deviceAltitudeData.temperature * (double) deviceAltitudeData.temperature;
	double tP3 = (double) deviceAltitudeData.temperature * tP2;

	double partial_data2 = bmp390.dp7 * tP2;
	double partial_data3 = bmp390.dp8 * tP3;
	double partial_out1 = bmp390.dp5 + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = bmp390.dp2 * deviceAltitudeData.temperature;
	partial_data2 = bmp390.dp3 * tP2;
	partial_data3 = bmp390.dp4 * tP3;

	double partial_out2 = bmp390.uPressure * (bmp390.dp1 + partial_data1 + partial_data2 + partial_data3);

	double uP2 = (double) bmp390.uPressure * (double) bmp390.uPressure;
	double uP3 = (double) bmp390.uPressure * uP2;

	partial_data1 = uP2;
	partial_data2 = bmp390.dp9 + bmp390.dp10 * deviceAltitudeData.temperature;
	partial_data3 = partial_data1 * partial_data2;

	double partial_data4 = partial_data3 + uP3 * bmp390.dp11;

	deviceAltitudeData.pressure = (partial_out1 + partial_out2 + partial_data4) * BMP390_PRESSURE_OUTPUT_SCALE;

}

void bmp390Caliberate() {
	bmp390GroundPressure = deviceAltitudeData.pressure;
	float baseSlope = (BMP390_PRESSURE_GAS_CONST * BMP390_PRESSURE_PWR_CONST) / bmp390GroundPressure;
	float localTempKelvin = deviceAltitudeData.temperature + BMP390_KELVIN_OFFSET;
	float tempCorrection = localTempKelvin / BMP390_STANDARD_TEMP_K;
	bmp390PressureToMeterFactor = baseSlope * tempCorrection;
	bmp390IsBaroCalibrated = 1;
}

void bmp390CalculateAltitude() {
	float currentPressure = deviceAltitudeData.pressure;
	if (!bmp390IsBaroCalibrated) {
		bmp390Caliberate();
		deviceAltitudeData.altitudeGround = (1.0f - powf(bmp390GroundPressure / BMP390_SEALEVEL_PRESSURE, BMP390_PRESSURE_PWR_CONST)) * BMP390_PRESSURE_GAS_CONST;
	}
	float pressureDiff = bmp390GroundPressure - currentPressure;
	deviceAltitudeData.altitude = (pressureDiff * bmp390PressureToMeterFactor);
}

void deviceBaroBMP390DataProcess() {
	bmp390CalculateTemp();
	bmp390CalculatePressure();
	bmp390CalculateAltitude();
}

uint8_t deviceBaroLoadData(void) {
#if BMP390_READ_ASYNC == 1
	if (bmp390HasData) {
		deviceBaroBMP390DataProcess();
		bmp390HasData = 0;
		return 1;
	} else {
		return 0;
	}
#else
	return 1;
#endif
}

void __deviceBaroBMP390Callback(uint8_t *buf, uint16_t len) {
	if (!bmp390HasData) {
		bmp390.uPressure = ((uint32_t) (buf[3] << 16)) | (uint32_t) (buf[2] << 8) | (uint32_t) buf[1];
		bmp390.uTemperature = ((uint32_t) (buf[6] << 16)) | (uint32_t) (buf[5] << 8) | (uint32_t) buf[4];
		bmp390HasData = 1;
	}
}

uint8_t deviceBaroRead() {
#if BMP390_READ_ASYNC == 1
	if (spi4ReadRegisterAsync(BMP390_DATA_ADDR, 7, BMP390_SPI4_DEVICE, __deviceBaroBMP390Callback)) {
		return 1;
	}
#else
	if (spi4ReadRegister(BMP390_DATA_ADDR, deviceAltitudeData.buffer, 7, BMP390_SPI4_DEVICE) == 1) {
		bmp390.uPressure = ((uint32_t) (deviceAltitudeData.buffer[3] << 16)) | (uint32_t) (deviceAltitudeData.buffer[2] << 8) | (uint32_t) deviceAltitudeData.buffer[1];
		bmp390.uTemperature = ((uint32_t) (deviceAltitudeData.buffer[6] << 16)) | (uint32_t) (deviceAltitudeData.buffer[5] << 8) | (uint32_t) deviceAltitudeData.buffer[4];
		deviceBaroBMP390DataProcess();
		return 1;
	}
#endif
	return 0;
}

uint8_t deviceBaroReset() {
	return bmp390SoftReset();
}

#endif
