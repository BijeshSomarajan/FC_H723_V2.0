#include "blackbox.h"

#include <main.h>
#include <stm32_hal_legacy.h>
#include <stm32h723xx.h>
#include <stm32h7xx_hal_cortex.h>
#include <stm32h7xx_hal_def.h>
#include <stm32h7xx_hal_gpio.h>
#include <stm32h7xx_hal_mdma.h>
#include <stm32h7xx_hal_rcc.h>
#include <stm32h7xx_hal_rcc_ex.h>
#include <stm32h7xx_hal_sd.h>
#include <stm32h7xx_ll_sdmmc.h>
#include <sys/_stdint.h>

#include "../../../sd/fatfs/ff.h"
#include "../../../sd/fatfs.h"
#include "../../logger/Logger.h"

SD_HandleTypeDef sdHandle;
MDMA_HandleTypeDef mdmaHandle;

char blackBoxSessionFileName[10];
uint8_t blackboxSessionActive = 0;
uint32_t blackboxByteswritten = 0;

uint8_t isBlackBoxAvailable() {
	return BLACK_BOX_ENABLED && blackboxSessionActive;
}

void SDMMC1_IRQHandler(void) {
	HAL_SD_IRQHandler(&sdHandle);
}

void MDMA_IRQHandler(void) {
	HAL_MDMA_IRQHandler(&mdmaHandle);
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd) {
	if (hsd->Instance == SDMMC1) {
		__HAL_RCC_SDMMC1_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
		HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
	}
}

void HAL_SD_MspInit(SD_HandleTypeDef *hsd) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
	if (hsd->Instance == SDMMC1) {
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
		PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
			Error_Handler();
		}
		__HAL_RCC_SDMMC1_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
		/*
		 PC8     ------> SDMMC1_D0
		 PC9     ------> SDMMC1_D1
		 PC10     ------> SDMMC1_D2
		 PC11     ------> SDMMC1_D3
		 PC12     ------> SDMMC1_CK
		 PD2     ------> SDMMC1_CMD
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* SDMMC1 interrupt Init */
		HAL_NVIC_SetPriority(SDMMC1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
		__HAL_RCC_SDMMC1_FORCE_RESET();
		__HAL_RCC_SDMMC1_RELEASE_RESET();
	}
}

uint8_t initSDMMC1() {
	sdHandle.Instance = SDMMC1;
	sdHandle.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	sdHandle.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	sdHandle.Init.BusWide = SDMMC_BUS_WIDE_4B;
	sdHandle.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	sdHandle.Init.ClockDiv = 2;
	return 1;
}

uint8_t initMDMA() {
	__HAL_RCC_MDMA_CLK_ENABLE();

	mdmaHandle.Instance = MDMA_Channel0;
	mdmaHandle.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
	mdmaHandle.Init.Priority = MDMA_PRIORITY_LOW;
	mdmaHandle.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
	mdmaHandle.Init.SourceInc = MDMA_SRC_INC_BYTE;
	mdmaHandle.Init.DestinationInc = MDMA_DEST_INC_BYTE;
	mdmaHandle.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
	mdmaHandle.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
	mdmaHandle.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
	mdmaHandle.Init.BufferTransferLength = 1;
	mdmaHandle.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
	mdmaHandle.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
	mdmaHandle.Init.SourceBlockAddressOffset = 0;
	mdmaHandle.Init.DestBlockAddressOffset = 0;
	if (HAL_MDMA_Init(&mdmaHandle) != HAL_OK) {
		return 0;
	}
	HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MDMA_IRQn);
	return 1;
}

uint8_t mountSDCard() {
	FRESULT fMount = f_mount(&SDFatFS, (TCHAR const*) SDPath, 0);
	return (fMount == FR_OK);
}

uint8_t unMountSDCard() {
	FRESULT fUMount = f_mount(&SDFatFS, (TCHAR const*) SDPath, 0);
	return (fUMount == FR_OK);
}

uint8_t initBlackbox(void) {
	if (BLACK_BOX_ENABLED) {
		uint8_t status = 0;
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOH_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
		status = initSDMMC1();
		if (status) {
			status = initMDMA();
		}
		if (status) {
			MX_FATFS_Init();
		}
		return status;
	} else {
		return 1;
	}
}

uint8_t createAndOpenSessionFile() {
	uint8_t status = 0;
	DIR dir;
	uint16_t totalFiles = 0;
	FRESULT dirRes = f_opendir(&dir, "");
	if (dirRes == FR_OK) {
		status = 1;
		while (1) {
			FILINFO fno;
			FRESULT readDirRes = f_readdir(&dir, &fno);
			if (readDirRes != FR_OK || (fno.fname[0] == 0)) {
				break;
			} else if (!(fno.fattrib & AM_DIR)) {
				totalFiles++;
			}
		}
	} else {
		char printBuf[64];
		sprintf(printBuf, "BlackBox -> Unable to open SD : %d\n", dirRes);
		logString(printBuf);
	}

	if (status) {
		totalFiles++;
		if (totalFiles > BLACK_BOX_MAX_FILES) {
			status = 0;
			logString("BlackBox -> Too many files , Clean up SD!!\n");
		} else {
			sprintf(blackBoxSessionFileName, "%d.log", totalFiles);
			FRESULT fMOpen = f_open(&SDFile, blackBoxSessionFileName, FA_CREATE_ALWAYS | FA_WRITE);
			if (fMOpen != FR_OK) {
				status = 0;
				logString("BlackBox -> Unable to open the session file\n");
			} else {
				char buf[100];
				sprintf(buf, "BlackBox -> Session File : %s\n", blackBoxSessionFileName);
				logString(buf);
			}
		}
	}

	return status;
}

uint8_t startBlackboxSession() {
	if (BLACK_BOX_ENABLED) {
		uint8_t status = 0;
		endBlackboxSession();
		status = mountSDCard();
		if (status) {
			status = createAndOpenSessionFile();
			blackboxSessionActive = status;
		}
		return status;
	} else {
		return 1;
	}
}

uint8_t endBlackboxSession() {
	if (BLACK_BOX_ENABLED) {
		if (blackboxSessionActive) {
			f_close(&SDFile);
			unMountSDCard();
		}
	}
	blackboxSessionActive = 0;
	return 1;
}

uint32_t recordToBlackbox(char *data, uint16_t len) {
	if (BLACK_BOX_ENABLED && blackboxSessionActive) {
		FRESULT resWrite = f_write(&SDFile, data, len, (void*) &blackboxByteswritten);
		if (resWrite == FR_OK) {

			return blackboxByteswritten;
		}
	}
	return 0;
}

