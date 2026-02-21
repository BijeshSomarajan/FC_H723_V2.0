#ifndef _BMP390_H_
#define _BMP390_H_

#if BARO_SENSOR_SELECTED == BARO_SENSOR_BMP390

#include "BMP390Registers.h"

#include <stdio.h>
#include <inttypes.h>

typedef struct _BMP390 BMP390;
struct _BMP390 {
	uint32_t uPressure;
	uint32_t uTemperature;

	int32_t dT;
	int64_t off;
	int64_t sens;

	//Configs
	uint16_t t1;
	uint16_t t2;
	int8_t t3;
	int16_t p1;
	int16_t p2;
	int8_t p3;
	int8_t p4;
	uint16_t p5;
	uint16_t p6;
	int8_t p7;
	int8_t p8;
	int16_t p9;
	int8_t p10;
	int8_t p11;

	double dt1;
	double dt2;
	double dt3;
	double dp1;
	double dp2;
	double dp3;
	double dp4;
	double dp5;
	double dp6;
	double dp7;
	double dp8;
	double dp9;
	double dp10;
	double dp11;
};

#define  BMP390_PRESSURE_CALC_SCALE 1.0f
#define  BMP390_PRESSURE_OUTPUT_SCALE 0.01f

#define  BMP390_SEALEVEL_PRESSURE  1013.25f * BMP390_PRESSURE_CALC_SCALE
#define  BMP390_PRESSURE_PWR_CONST 0.1902225603956629f//0.190295f //0.1902225603956629f
#define  BMP390_PRESSURE_GAS_CONST 44330.0f * BMP390_PRESSURE_CALC_SCALE

#define BMP390_KELVIN_OFFSET 273.15f
#define BMP390_STANDARD_TEMP_K 288.15f // 15 degrees Celsius

#define BMP390_RESOLUTION_TYPE_LOW_POWER 0
#define BMP390_RESOLUTION_TYPE_STD 1
#define BMP390_RESOLUTION_TYPE_HIGH 2
#define BMP390_RESOLUTION_TYPE_ULTRA_HIGH 3
#define BMP390_RESOLUTION_TYPE_HIGHEST 4
#define BMP390_RESOLUTION_TYPE_CUSTOM 5

#define BMP390_FILTER_TYPE_NONE 0
#define BMP390_FILTER_TYPE_STD 1
#define BMP390_FILTER_TYPE_HIGH 2
#define BMP390_FILTER_TYPE_ULTRA_HIGH 3
#define BMP390_FILTER_TYPE_HIGHEST 4
#define BMP390_FILTER_TYPE_CUSTOM 5

#define BMP390_RESOLUTION_TYPE BMP390_RESOLUTION_TYPE_CUSTOM
#define BMP390_FILTER_TYPE BMP390_FILTER_TYPE_CUSTOM

#define BMP390_READ_ASYNC 1
//Global variable
extern BMP390 bmp390;

#endif

#endif
