#ifndef SENSOR_TFMINI_H_
#define SENSOR_TFMINI_H_

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../../../../io/i2c/I2C.h"
#include "../../../../timers/DelayTimer.h"
#include "../../../../logger/Logger.h"
#include "../../../../util/CommonUtil.h"
#include "../../../../util/MathUtil.h"

#define TFMINI_DEFAULT_ADDRESS   (0x10)

#define TFMINI_MAX_VALID_DISTANCE 12.0f //12 mts
#define TFMINI_MIN_VALID_DISTANCE 0.12f //12 cms
#define TFMINI_MIN_QUAL 0.02f

#define TFMINI_MIN_VALID_STRENGTH 80
#define TFMINI_MAX_VALID_STRENGTH 20000

#define TFMINI_SWITCH_TO_UART 0
#define TFMINI_SWITCH_TO_I2C 0

#define TFMINI_IO_TYPE_UART 1
#define TFMINI_IO_TYPE_I2C 2

#define TFMINI_IO_TYPE TFMINI_IO_TYPE_UART

typedef struct _TFMini TFMini;
struct _TFMini {
	float distance;
	float rate;
	uint16_t strength;
	uint8_t buffer[32];
};

//Global variable
extern TFMini tfMini;

#endif
