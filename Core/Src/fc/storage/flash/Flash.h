#ifndef STORAGE_FLASH_H_
#define STORAGE_FLASH_H_

#include <stdint.h>
#include "stm32h7xx.h"
#include "stm32h7xx_hal_flash.h"



uint8_t initFlash(void);
uint8_t writeWordsToFlash(uint32_t *data, uint32_t length);
uint8_t readWordsFromFlash(uint32_t *data, uint32_t length);
uint32_t readWordFromFlash(uint32_t indx);

#endif
