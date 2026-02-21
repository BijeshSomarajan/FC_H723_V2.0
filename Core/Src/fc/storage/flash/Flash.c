#include "flash.h"
#include "stm32h7xx_hal.h"

/* STM32H723VGT6 Flash info:
 * - 1MB Flash, single bank
 * - 8KB per sector (128 sectors total)
 * - Last sector (127): 0x080FE000 – 0x080FFFFF
 */
#define ADDR_FLASH_SECTOR_127     0x080FE000UL   // Sector 127 start address
#define FLASH_SECTOR_NUM_127      127
#define FLASH_BANK_USED           FLASH_BANK_1

uint8_t initFlash(void) {
    return 1;
}

/*
 * 256 bits = 32 bytes = 8 words is the flash programming unit.
 * You must always write in 32-byte aligned blocks.
 */
uint8_t writeWordsToFlash(uint32_t *data, uint32_t length) {
    uint8_t status = 0;

    if (HAL_FLASH_Unlock() == HAL_OK) {
        status = 1;
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

        // Prepare erase
        FLASH_EraseInitTypeDef eraseInit;
        uint32_t sectorError = 0;

        eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
        eraseInit.Banks        = FLASH_BANK_USED;
        eraseInit.Sector       = FLASH_SECTOR_NUM_127;
        eraseInit.NbSectors    = 1;
        eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
            HAL_FLASH_Lock();
            return 0;
        }

        uint32_t address = ADDR_FLASH_SECTOR_127;

        // Write full 8-word (32-byte) blocks
        while (length >= 8) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)data) == HAL_OK) {
                address += 32;  // 8 words × 4 bytes
                data    += 8;
                length  -= 8;
            } else {
                status = 0;
                break;
            }
        }

        // Pad and write remaining (<8 words)
        if (status && (length > 0)) {
            uint32_t paddedData[8] = {0};
            for (uint8_t i = 0; i < length; i++) {
                paddedData[i] = data[i];
            }

            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)paddedData) != HAL_OK) {
                status = 0;
            }
        }
    }

    HAL_FLASH_Lock();
    return status;
}

uint8_t readWordsFromFlash(uint32_t *data, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        uint32_t address = ADDR_FLASH_SECTOR_127 + (i * 4);
        data[i] = *(__IO uint32_t*)address;
    }
    return 1;
}

uint32_t readWordFromFlash(uint32_t index) {
    uint32_t address = ADDR_FLASH_SECTOR_127 + (index * 4);
    return *(__IO uint32_t*)address;
}
