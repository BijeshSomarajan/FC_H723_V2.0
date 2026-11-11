#include "blackbox.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <cmsis_gcc.h>
#include <main.h>
#include <stm32_hal_legacy.h>
#include <stm32h723xx.h>
#include <stm32h7xx_hal_cortex.h>
#include <stm32h7xx_hal_def.h>
#include <stm32h7xx_hal_gpio.h>
#include <stm32h7xx_hal_rcc.h>
#include <stm32h7xx_hal_rcc_ex.h>
#include <stm32h7xx_hal_sd.h>
#include <stm32h7xx_ll_sdmmc.h>

#include "../../logger/Logger.h"
#include "../../sensors/blackbox/asyncfatfs.h"
#include "../../sensors/blackbox/bsp_driver_sd.h"
#include "../../sensors/blackbox/fat_standard.h"
#include "../../sensors/blackbox/sdcard.h"

SD_HandleTypeDef sdHandle;

char blackBoxSessionFileName[10];
uint8_t blackboxSessionActive = 0;

static uint8_t blackboxRing[BLACK_BOX_RING_SIZE];
static volatile uint32_t blackboxRingHead = 0;
static volatile uint32_t blackboxRingTail = 0;
static uint32_t blackboxDroppedBytes = 0;
static uint8_t blackboxDropLogged = 0;
static uint8_t blackboxWriteChunk[BLACK_BOX_WRITE_CHUNK] __attribute__((aligned(32)));

typedef enum {
	BLACKBOX_STATE_IDLE,
	BLACKBOX_STATE_WAIT_FILESYSTEM,
	BLACKBOX_STATE_OPEN_DIRECTORY,
	BLACKBOX_STATE_OPEN_DIRECTORY_PENDING,
	BLACKBOX_STATE_SCAN_DIRECTORY,
	BLACKBOX_STATE_CLOSE_DIRECTORY,
	BLACKBOX_STATE_CLOSE_DIRECTORY_PENDING,
	BLACKBOX_STATE_OPEN_FILE,
	BLACKBOX_STATE_OPEN_FILE_PENDING,
	BLACKBOX_STATE_RECORDING,
	BLACKBOX_STATE_DRAINING,
	BLACKBOX_STATE_DESTROYING,
	BLACKBOX_STATE_ERROR
} blackboxState_e;

static blackboxState_e blackboxState = BLACKBOX_STATE_IDLE;
static afatfsFilePtr_t blackboxDirectory = NULL;
static afatfsFilePtr_t blackboxFile = NULL;
static afatfsFinder_t blackboxFinder;
static uint32_t blackboxHighestFileIndex = 0;
static uint8_t blackboxFinderStarted = 0;
static uint8_t blackboxFilesystemStarted = 0;
static uint8_t blackboxShutdownRequested = 0;
static uint8_t blackboxSyncRequested = 0;
static uint8_t blackboxSyncComplete = 0;
static uint8_t blackboxSyncMetadataSaved = 0;
static uint8_t blackboxFatalLogged = 0;

/* -------------------------------------------------------------------------- */
/* SDMMC1                                                                      */
/* -------------------------------------------------------------------------- */

void SDMMC1_IRQHandler(void) {
	HAL_SD_IRQHandler(&sdHandle);
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

		HAL_NVIC_SetPriority(SDMMC1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

		__HAL_RCC_SDMMC1_FORCE_RESET();
		__HAL_RCC_SDMMC1_RELEASE_RESET();
	}
}

static uint8_t initSDMMC1(void) {
	sdHandle.Instance = SDMMC1;
	sdHandle.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	sdHandle.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	sdHandle.Init.BusWide = SDMMC_BUS_WIDE_4B;
	sdHandle.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
	sdHandle.Init.ClockDiv = 4;
	return 1;
}

/* -------------------------------------------------------------------------- */
/* Ring buffer                                                                */
/* -------------------------------------------------------------------------- */

static uint32_t blackboxRingBufferUsed(void) {
	return blackboxRingHead - blackboxRingTail;
}

static uint8_t blackboxRingBufferPush(const uint8_t *data, uint16_t len) {
	uint32_t used = blackboxRingBufferUsed();
	if (len > (BLACK_BOX_RING_SIZE - used)) {
		return 0;
	}

	uint32_t head = blackboxRingHead;
	for (uint32_t i = 0; i < len; i++) {
		blackboxRing[(head + i) & (BLACK_BOX_RING_SIZE - 1)] = data[i];
	}
	__DMB();
	blackboxRingHead = head + len;
	return 1;
}

static void blackboxRingBufferPeek(uint8_t *dst, uint32_t len) {
	uint32_t tail = blackboxRingTail;
	for (uint32_t i = 0; i < len; i++) {
		dst[i] = blackboxRing[(tail + i) & (BLACK_BOX_RING_SIZE - 1)];
	}
}

static void blackboxRingBufferAdvance(uint32_t len) {
	__DMB();
	blackboxRingTail += len;
}

static void blackboxDiscardPendingRingData(void) {
	uint32_t pending = blackboxRingBufferUsed();
	blackboxDroppedBytes += pending;
	__DMB();
	blackboxRingTail = blackboxRingHead;
}

/* -------------------------------------------------------------------------- */
/* Session filename handling                                                  */
/* -------------------------------------------------------------------------- */

static uint32_t blackboxParseSessionFileIndex(const fatDirectoryEntry_t *entry) {
	uint32_t index = 0;
	uint8_t digitCount = 0;

	if (entry == NULL) {
		return 0;
	}
	if ((entry->attrib & 0x18U) != 0U) {
		return 0;
	}
	if (entry->filename[0] == 0x00 || entry->filename[0] == (char) 0xE5) {
		return 0;
	}
	if ((entry->filename[8] != 'L' && entry->filename[8] != 'l') || (entry->filename[9] != 'O' && entry->filename[9] != 'o') || (entry->filename[10] != 'G' && entry->filename[10] != 'g')) {
		return 0;
	}

	for (uint8_t i = 0; i < 8; i++) {
		char c = entry->filename[i];
		if (c >= '0' && c <= '9') {
			index = (index * 10U) + (uint32_t) (c - '0');
			digitCount++;
		} else if (c == ' ') {
			break;
		} else {
			return 0;
		}
	}

	if (digitCount == 0U || index == 0U) {
		return 0;
	}

	return index;
}

/* -------------------------------------------------------------------------- */
/* AsyncFatFS callbacks                                                       */
/* -------------------------------------------------------------------------- */

static void blackboxDirectoryOpenComplete(afatfsFilePtr_t file) {
	blackboxDirectory = file;
	if (file == NULL) {
		blackboxState = BLACKBOX_STATE_ERROR;
		return;
	}
	blackboxFinderStarted = 0;
	blackboxState = BLACKBOX_STATE_SCAN_DIRECTORY;
}

static void blackboxDirectoryCloseComplete(void) {
	blackboxDirectory = NULL;
	blackboxState = BLACKBOX_STATE_OPEN_FILE;
}

static void blackboxFileOpenComplete(afatfsFilePtr_t file) {
	blackboxFile = file;
	if (file == NULL) {
		blackboxState = BLACKBOX_STATE_ERROR;
	} else if (blackboxShutdownRequested) {
		blackboxSessionActive = 0;
		blackboxState = BLACKBOX_STATE_DRAINING;
	} else {
		blackboxSessionActive = 1;
		blackboxState = BLACKBOX_STATE_RECORDING;
	}
}

/* -------------------------------------------------------------------------- */
/* Session creation                                                           */
/* -------------------------------------------------------------------------- */

static void blackboxOpenSessionFile(void) {
	if ((blackboxHighestFileIndex + 1U) > BLACK_BOX_MAX_FILES) {
		logString("BlackBox -> Maximum log file count reached\n");
		blackboxState = BLACKBOX_STATE_ERROR;
		return;
	}

	blackboxHighestFileIndex++;
	snprintf(blackBoxSessionFileName, sizeof(blackBoxSessionFileName), "%lu.log", (unsigned long) blackboxHighestFileIndex);

	blackboxState = BLACKBOX_STATE_OPEN_FILE_PENDING;
	if (!afatfs_fopen(blackBoxSessionFileName, "w", blackboxFileOpenComplete)) {
		blackboxState = BLACKBOX_STATE_ERROR;
	}
}

static void blackboxScanSessionDirectory(void) {
	fatDirectoryEntry_t *entry = NULL;

	if (!blackboxFinderStarted) {
		afatfs_findFirst(blackboxDirectory, &blackboxFinder);
		blackboxFinderStarted = 1;
	}

	afatfsOperationStatus_e status = afatfs_findNext(blackboxDirectory, &blackboxFinder, &entry);
	if (status == AFATFS_OPERATION_IN_PROGRESS) {
		return;
	}
	if (status == AFATFS_OPERATION_FAILURE) {
		blackboxState = BLACKBOX_STATE_ERROR;
		return;
	}
	if (entry == NULL) {
		blackboxState = BLACKBOX_STATE_CLOSE_DIRECTORY;
		return;
	}

	uint32_t index = blackboxParseSessionFileIndex(entry);
	if (index > blackboxHighestFileIndex) {
		blackboxHighestFileIndex = index;
	}
}

/* -------------------------------------------------------------------------- */
/* Filesystem initialization                                                  */
/* -------------------------------------------------------------------------- */

uint8_t initBlackbox(void) {
	if (!BLACK_BOX_ENABLED) {
		return 1;
	}
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	if (!initSDMMC1()) {
		logString("BlackBox -> initSDMMC1 failed\n");
		return 0;
	}
	if (BSP_SD_Init() != 0) {
		logString("BlackBox -> BSP_SD_Init failed\n");
		return 0;
	}
	return 1;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

uint8_t isBlackBoxAvailable(void) {
	return BLACK_BOX_ENABLED && blackboxSessionActive;
}

uint8_t startBlackboxSession(void) {
	if (!BLACK_BOX_ENABLED || blackboxState != BLACKBOX_STATE_IDLE) {
		logString("BlackBox -> Not in idle state\n");
		return 0;
	}

	blackboxRingHead = 0;
	blackboxRingTail = 0;
	blackboxDroppedBytes = 0;
	blackboxDropLogged = 0;
	blackboxHighestFileIndex = 0;
	blackboxDirectory = NULL;
	blackboxFile = NULL;
	blackboxFinderStarted = 0;
	blackboxShutdownRequested = 0;
	blackboxSyncRequested = 0;
	blackboxSyncComplete = 0;
	blackboxSyncMetadataSaved = 0;
	blackboxFatalLogged = 0;
	blackboxSessionActive = 0;

	afatfs_init();
	blackboxFilesystemStarted = 1;
	blackboxState = BLACKBOX_STATE_WAIT_FILESYSTEM;
	return 1;
}

uint32_t recordToBlackbox(const void *data, uint16_t len) {
	if (!blackboxSessionActive || data == NULL || len == 0U) {
		return 0;
	}

	if (!blackboxRingBufferPush((const uint8_t*) data, len)) {
		blackboxDroppedBytes += len;
		return 0;
	}

	return len;
}

/* -------------------------------------------------------------------------- */
/* Async write processing                                                     */
/* -------------------------------------------------------------------------- */

static void writeRingChunk(void) {
	uint32_t available = blackboxRingBufferUsed();
	if (available == 0U || blackboxFile == NULL) {
		return;
	}

	uint32_t len = available;
	if (len > BLACK_BOX_WRITE_CHUNK) {
		len = BLACK_BOX_WRITE_CHUNK;
	}

	blackboxRingBufferPeek(blackboxWriteChunk, len);

	uint32_t written = afatfs_fwrite(blackboxFile, blackboxWriteChunk, len);
	if (written > 0U) {
		blackboxRingBufferAdvance(written);
	}

	if (written == 0U && afatfs_isFull()) {
		logString("BlackBox: Filesystem full - shutdown\n");
		blackboxSessionActive = 0;
		blackboxShutdownRequested = 1;
		blackboxDiscardPendingRingData();
		blackboxState = BLACKBOX_STATE_DESTROYING;
	}
}

/* -------------------------------------------------------------------------- */
/* Main-loop processing                                                       */
/* -------------------------------------------------------------------------- */

void processBlackbox(void) {
	if (!BLACK_BOX_ENABLED || !blackboxFilesystemStarted) {
		return;
	}

	afatfs_poll();

	if (sdcard_isFaulted() && blackboxState != BLACKBOX_STATE_ERROR) {
		logString("BlackBox -> SD card faulted, stopping logging\n");
		blackboxSessionActive = 0;
		blackboxState = BLACKBOX_STATE_ERROR;
	}

	if (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_FATAL) {
		if (!blackboxFatalLogged) {
			logString("BlackBox -> AsyncFatFS fatal error\n");
			blackboxFatalLogged = 1;
		}
		blackboxSessionActive = 0;
		blackboxState = BLACKBOX_STATE_ERROR;
	}

	switch (blackboxState) {
	case BLACKBOX_STATE_IDLE:
		break;

	case BLACKBOX_STATE_WAIT_FILESYSTEM:
		if (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_READY) {
			blackboxState = BLACKBOX_STATE_OPEN_DIRECTORY;
		}
		break;

	case BLACKBOX_STATE_OPEN_DIRECTORY:
		blackboxState = BLACKBOX_STATE_OPEN_DIRECTORY_PENDING;
		if (!afatfs_fopen(".", "r", blackboxDirectoryOpenComplete)) {
			blackboxState = BLACKBOX_STATE_ERROR;
		}
		break;

	case BLACKBOX_STATE_OPEN_DIRECTORY_PENDING:
		break;

	case BLACKBOX_STATE_SCAN_DIRECTORY:
		blackboxScanSessionDirectory();
		break;

	case BLACKBOX_STATE_CLOSE_DIRECTORY:
		blackboxState = BLACKBOX_STATE_CLOSE_DIRECTORY_PENDING;
		if (!afatfs_fclose(blackboxDirectory, blackboxDirectoryCloseComplete)) {
			if (blackboxState == BLACKBOX_STATE_CLOSE_DIRECTORY_PENDING) {
				blackboxState = BLACKBOX_STATE_CLOSE_DIRECTORY;
			}
		}
		break;

	case BLACKBOX_STATE_CLOSE_DIRECTORY_PENDING:
		break;

	case BLACKBOX_STATE_OPEN_FILE:
		blackboxOpenSessionFile();
		break;

	case BLACKBOX_STATE_OPEN_FILE_PENDING:
		break;

	case BLACKBOX_STATE_RECORDING:
		// Drain all accepted ring-buffer data before synchronizing.
		if (blackboxRingBufferUsed() > 0U) {
			if (blackboxRingBufferUsed() >= BLACK_BOX_WRITE_CHUNK || blackboxSyncRequested) {
				writeRingChunk();
			}
			break;
		}
		if (blackboxSyncRequested) {
			// Update the directory entry and release the current data-sector lock.
			if (!blackboxSyncMetadataSaved) {
				afatfsOperationStatus_e status = afatfs_fsync(blackboxFile);
				if (status == AFATFS_OPERATION_SUCCESS) {
					blackboxSyncMetadataSaved = 1;
				} else if (status == AFATFS_OPERATION_FAILURE) {
					logString("BlackBox: Metadata sync failed\n");
					blackboxSessionActive = 0;
					blackboxState = BLACKBOX_STATE_ERROR;
				}
				break;
			}
			// Submit dirty sectors.
			uint8_t flushReady = afatfs_flush();
			// Wait for the card to finish its current transfer.
			uint8_t cardReady = sdcard_isOperationIdle();
			if (flushReady && cardReady && blackboxRingBufferUsed() == 0U) {
				blackboxSyncComplete = 1;
				blackboxSyncRequested = 0;
				blackboxSyncMetadataSaved = 0;
			}
		}
		break;
	case BLACKBOX_STATE_DRAINING:
		if (blackboxRingBufferUsed() > 0U) {
			writeRingChunk();
		} else {
			blackboxState = BLACKBOX_STATE_DESTROYING;
		}
		break;
	case BLACKBOX_STATE_DESTROYING: {
		uint8_t destroyed = afatfs_destroy(false);
		if (destroyed) {
			blackboxFilesystemStarted = 0;
			blackboxDirectory = NULL;
			blackboxFile = NULL;
			blackboxSessionActive = 0;
			blackboxShutdownRequested = 0;
			blackboxSyncRequested = 0;
			blackboxSyncComplete = 0;
			blackboxSyncMetadataSaved = 0;
			blackboxState = BLACKBOX_STATE_IDLE;
		}
		break;
	}
	case BLACKBOX_STATE_ERROR:
		blackboxSessionActive = 0;
		break;
	default:
		blackboxState = BLACKBOX_STATE_ERROR;
		break;
	}
	if (blackboxDroppedBytes > 0U && !blackboxDropLogged) {
		char printBuf[80];
		snprintf(printBuf, sizeof(printBuf), "BlackBox -> Dropped %lu bytes\n", (unsigned long) blackboxDroppedBytes);
		logString(printBuf);
		blackboxDropLogged = 1;
	}
}

/* -------------------------------------------------------------------------- */
/* Synchronization and shutdown                                               */
/* -------------------------------------------------------------------------- */

uint8_t syncBlackbox(void) {
	if (!blackboxSessionActive || blackboxState != BLACKBOX_STATE_RECORDING) {
		return 0;
	}

	if (blackboxSyncComplete) {
		blackboxSyncComplete = 0;
		return 1;
	}

	blackboxSyncRequested = 1;
	return 0;
}

uint8_t endBlackboxSession(void) {
	if (blackboxState == BLACKBOX_STATE_IDLE) {
		return 1;
	}

	blackboxSessionActive = 0;
	blackboxShutdownRequested = 1;

	if (blackboxState == BLACKBOX_STATE_RECORDING) {
		blackboxState = BLACKBOX_STATE_DRAINING;
	}

	return 1;
}
