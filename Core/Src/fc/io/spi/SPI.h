#ifndef SRC_FC_IO_SPI_SPI_H_
#define SRC_FC_IO_SPI_SPI_H_
#include <stdint.h>
#include "stm32h7xx_ll_spi.h"

#include "../../memory/Memory.h"

#define FC_SPI_READ_MASK  (0x80)
#define FC_SPI_WRITE_MASK (0x7F)

#define SPI_IO_TIMEOUT_COUNT 10000U

#define FC_SPI2_DEVICE1  1
#define FC_SPI4_DEVICE1  2
#define FC_SPI6_DEVICE1  3

typedef void (*spi_callback_t)(uint8_t *buf, uint16_t len);

uint8_t spi2Init(void);
uint8_t spi2ReadRegister(uint8_t regAddr, uint8_t *rxData, uint16_t rxLen, uint8_t device);
uint8_t spi2ReadRegisterAsync(uint8_t reg, uint16_t rxLen, uint8_t device, spi_callback_t callback);
uint8_t spi2WriteRegister(uint8_t regAddr, uint8_t *txData, uint16_t txLen, uint8_t device);

uint8_t spi4Init(void);
uint8_t spi4ReadRegister(uint8_t regAddr, uint8_t *rxData, uint16_t rxLen, uint8_t device);
uint8_t spi4ReadRegisterAsync(uint8_t reg, uint16_t rxLen, uint8_t device, spi_callback_t callback);
uint8_t spi4WriteRegister(uint8_t regAddr, uint8_t *txData, uint16_t txLen, uint8_t device);

uint8_t spi6Init(void);
uint8_t spi6ReadRegister(uint8_t regAddr, uint8_t *rxData, uint16_t rxLen, uint8_t device);
uint8_t spi6ReadRegisterAsync(uint8_t regAddr, uint16_t rxLen, uint8_t device, spi_callback_t callback);
uint8_t spi6WriteRegister(uint8_t regAddr, uint8_t *txData, uint16_t txLen, uint8_t device);

#endif
