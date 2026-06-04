/*
 * i2c.h
 *
 *  Created on: Jun 3, 2026
 *      Author: bijes
 */

#ifndef SRC_FC_IO_I2C_I2C_H_
#define SRC_FC_IO_I2C_I2C_H_

#include "stm32h7xx_ll_i2c.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dmamux.h"
#include "../../memory/Memory.h"

typedef void (*i2c_callback_t)(uint8_t *pData, uint16_t len);
#define I2C_IO_TIMEOUT_COUNT 50000U

/* I2C1 methods */
void i2c1_ScanBus(void);
uint8_t initI2C1(void);
uint8_t i2c1Write(uint8_t address, uint8_t *data, uint16_t length);
uint8_t i2c1Read(uint8_t address, uint8_t *data, uint16_t length);
uint8_t i2c1CheckAddress(uint8_t address);
uint8_t i2c1WriteAsync(uint8_t address, uint8_t *data, uint16_t length, i2c_callback_t callback);
uint8_t i2c1ReadAsync(uint8_t address, uint16_t length, i2c_callback_t callback);

#endif /* SRC_FC_IO_I2C_I2C_H_ */
