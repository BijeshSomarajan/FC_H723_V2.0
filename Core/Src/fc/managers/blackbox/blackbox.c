#include "blackbox.h"

#include <cmsis_gcc.h>
#include <main.h>
#include <stdbool.h>
#include <stdio.h>
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
#include "../../FCConfig.h"
#include "../../memory/Memory.h"

SD_HandleTypeDef sdHandle;

/* Shutdown must never hang: if draining or destroying takes longer, the filesystem is abandoned. */
#define BLACK_BOX_DRAIN_TIMEOUT_MS     2000U
#define BLACK_BOX_DESTROY_TIMEOUT_MS   3000U

/* A sync that does not finish in this time is abandoned (logging continues, the next sync tries again). */
#define BLACK_BOX_SYNC_TIMEOUT_MS      1000U

/* Directory entries examined per processBlackbox() call while looking for the highest log number. */
#define BLACK_BOX_SCAN_ENTRIES_PER_CALL  16U

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
static uint32_t blackboxShutdownStartTick = 0;
static uint32_t blackboxSyncTargetHead = 0;   /* ring head when the sync was requested */
static uint32_t blackboxSyncStartTick = 0;
static uint32_t blackboxSyncTimeouts = 0;

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

		HAL_NVIC_SetPriority(SDMMC1_IRQn, BLACK_BOX_TASK_PRIORITY, 0);
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
		blackboxShutdownStartTick = HAL_GetTick();
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
		blackboxState = BLACKBOX_STATE_ERROR;
		return;
	}
	blackboxHighestFileIndex++;
	snprintf(blackBoxSessionFileName, sizeof(blackBoxSessionFileName), "%lu.log", (unsigned long) blackboxHighestFileIndex);

#if BLACK_BOX_DEBUG_ENABLED == 1
	logPrintf("[Blackbox]:New Session File : %s\n",blackBoxSessionFileName);
#endif

	blackboxState = BLACKBOX_STATE_OPEN_FILE_PENDING;
	if (!afatfs_fopen(blackBoxSessionFileName, "w", blackboxFileOpenComplete)) {
		blackboxState = BLACKBOX_STATE_ERROR;
	}
}

static void blackboxScanSessionDirectory(void) {
	if (!blackboxFinderStarted) {
		afatfs_findFirst(blackboxDirectory, &blackboxFinder);
		blackboxFinderStarted = 1;
	}

	for (uint32_t n = 0; n < BLACK_BOX_SCAN_ENTRIES_PER_CALL; n++) {
		fatDirectoryEntry_t *entry = NULL;

		afatfsOperationStatus_e status = afatfs_findNext(blackboxDirectory, &blackboxFinder, &entry);
		if (status == AFATFS_OPERATION_IN_PROGRESS) {
			return;
		}
		if (status == AFATFS_OPERATION_FAILURE) {
			blackboxState = BLACKBOX_STATE_ERROR;
			return;
		}

		/* End of the directory, or the terminator entry: nothing valid follows it. */
		if (entry == NULL || fat_isDirectoryEntryTerminator(entry)) {
			blackboxState = BLACKBOX_STATE_CLOSE_DIRECTORY;
			return;
		}

		uint32_t index = blackboxParseSessionFileIndex(entry);
		if (index > blackboxHighestFileIndex) {
			blackboxHighestFileIndex = index;
		}
	}
}

/* -------------------------------------------------------------------------- */
/* Filesystem initialization                                                  */
/* -------------------------------------------------------------------------- */

uint8_t initBlackbox(void) {
	if (!BLACK_BOX_ENABLED) {
		return 0;
	}
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	if (!initSDMMC1()) {
		logString("[Blackbox]:SDMMC1 init failed\n");
		return 0;
	}
	logString("[Blackbox]:SDMMC1 init Success\n");
	if (BSP_SD_Init() != 0) {
		logString("[Blackbox]:BSP_SD init failed\n");
		return 0;
	}
	logString("[Blackbox]:BSP_SD init Success\n");
	return 1;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

uint8_t isBlackBoxAvailable(void) {
	return BLACK_BOX_ENABLED && blackboxSessionActive;
}

uint8_t isBlackboxIdle(void) {
    return blackboxState == BLACKBOX_STATE_IDLE;
}

char* getBlackBoxSessionFileName(){
	return blackBoxSessionFileName;
}

uint8_t startBlackboxSession(void) {
	if (!BLACK_BOX_ENABLED || blackboxState != BLACKBOX_STATE_IDLE) {
		return 0;
	}

	if (sdcard_isFaulted()) {
		/* Blocking re-init: only start a session while disarmed. */
		if (!sdcard_recover()) {
			return 0;
		}
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

__ATTR_ITCM_TEXT
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
__ATTR_ITCM_TEXT
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
		blackboxSessionActive = 0;
		blackboxShutdownRequested = 1;
		blackboxDiscardPendingRingData();
		blackboxShutdownStartTick = HAL_GetTick();
		blackboxState = BLACKBOX_STATE_DESTROYING;
	}
}

/* -------------------------------------------------------------------------- */
/* Main-loop processing                                                       */
/* -------------------------------------------------------------------------- */
__ATTR_ITCM_TEXT
void processBlackbox(void) {
	if (!BLACK_BOX_ENABLED || !blackboxFilesystemStarted) {
		return;
	}

	afatfs_poll();

	if (sdcard_isFaulted() && blackboxState != BLACKBOX_STATE_ERROR) {
		blackboxSessionActive = 0;
		blackboxState = BLACKBOX_STATE_ERROR;
	}

	if (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_FATAL) {
		if (!blackboxFatalLogged) {
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
		if (!blackboxSyncRequested) {
			if (blackboxRingBufferUsed() >= BLACK_BOX_WRITE_CHUNK) {
				writeRingChunk();
			}
			break;
		}

		/* A stuck sync must never stop the data flow: give up and carry on logging. */
		if ((uint32_t) (HAL_GetTick() - blackboxSyncStartTick) > BLACK_BOX_SYNC_TIMEOUT_MS) {
			blackboxSyncTimeouts++;
			blackboxSyncRequested = 0;
			blackboxSyncMetadataSaved = 0;
			break;
		}

		/*
		 * Sync: write out only what was queued when the sync was requested. Waiting for the
		 * ring to become completely empty would never finish if records keep arriving every pass.
		 */
		if ((int32_t) (blackboxSyncTargetHead - blackboxRingTail) > 0) {
			writeRingChunk();
			break;
		}

		// Update the directory entry (true file size) and release the current data-sector lock.
		if (!blackboxSyncMetadataSaved) {
			/* Keep the ring draining while the metadata save is pending, so a slow sync cannot fill it. */
			if (blackboxRingBufferUsed() >= BLACK_BOX_WRITE_CHUNK) {
				writeRingChunk();
			}

			afatfsOperationStatus_e status = afatfs_fsync(blackboxFile);
			if (status == AFATFS_OPERATION_SUCCESS) {
				blackboxSyncMetadataSaved = 1;
			} else if (status == AFATFS_OPERATION_FAILURE) {
				blackboxSessionActive = 0;
				blackboxState = BLACKBOX_STATE_ERROR;
			}
			break;
		}

		// Submit dirty sectors, then wait for the driver to finish the transfers (no CMD13).
		if (afatfs_flush() && sdcard_isOperationIdle()) {
			blackboxSyncComplete = 1;
			blackboxSyncRequested = 0;
			blackboxSyncMetadataSaved = 0;
		}
		break;

	case BLACKBOX_STATE_DRAINING:
		if (blackboxRingBufferUsed() > 0U) {
			if ((uint32_t) (HAL_GetTick() - blackboxShutdownStartTick) > BLACK_BOX_DRAIN_TIMEOUT_MS) {
				blackboxDiscardPendingRingData();
			} else {
				writeRingChunk();
			}
		} else {
			blackboxShutdownStartTick = HAL_GetTick();
			blackboxState = BLACKBOX_STATE_DESTROYING;
		}
		break;

	case BLACKBOX_STATE_DESTROYING: {
		/*
		 * A faulted card can never finish a flush, so abandon the filesystem at once
		 * (dirty destroy). The same applies if a normal destroy takes too long.
		 */
		bool timedOut = (uint32_t) (HAL_GetTick() - blackboxShutdownStartTick) > BLACK_BOX_DESTROY_TIMEOUT_MS;
		uint8_t destroyed = afatfs_destroy(sdcard_isFaulted() || timedOut);
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
		if (blackboxShutdownRequested) { /* set by endBlackboxSession() */
			afatfs_destroy(true); /* dirty: no flushing, no waiting; also clears the whole afatfs state */
			blackboxFilesystemStarted = 0;
			blackboxDirectory = NULL;
			blackboxFile = NULL;
			blackboxShutdownRequested = 0;
			blackboxSyncRequested = 0;
			blackboxSyncComplete = 0;
			blackboxSyncMetadataSaved = 0;
			blackboxState = BLACKBOX_STATE_IDLE;
		}
		break;

	default:
		blackboxState = BLACKBOX_STATE_ERROR;
		break;
	}
	if (blackboxDroppedBytes > 0U && !blackboxDropLogged) {
		blackboxDropLogged = 1;
	}
}

/* -------------------------------------------------------------------------- */
/* Synchronization and shutdown                                               */
/* -------------------------------------------------------------------------- */
__ATTR_ITCM_TEXT
uint8_t syncBlackbox(void) {
	if (!blackboxSessionActive || blackboxState != BLACKBOX_STATE_RECORDING) {
		return 0;
	}

	if (blackboxSyncComplete) {
		blackboxSyncComplete = 0;
		return 1;
	}

	if (!blackboxSyncRequested) {
		blackboxSyncTargetHead = blackboxRingHead;
		blackboxSyncStartTick = HAL_GetTick();
		blackboxSyncRequested = 1;
	}
	return 0;
}

uint8_t endBlackboxSession(void) {
	blackboxSessionActive = 0;
	if (blackboxState == BLACKBOX_STATE_IDLE) {
		return 1;
	}
	blackboxShutdownRequested = 1;
	if (blackboxState == BLACKBOX_STATE_RECORDING) {
		blackboxShutdownStartTick = HAL_GetTick();
		blackboxState = BLACKBOX_STATE_DRAINING;
	}
	return 1;
}

uint32_t getBlackBoxMaxOPsMs(){
	return sdcard_getMaxOpMs();
}

uint32_t getBlackBoxSyncTimeouts(void) {
	return blackboxSyncTimeouts;
}

/* Diagnostics: call at most once per second, and only when a blocking UART print is acceptable. */
void printBlackboxDebug(void) {
#if BLACK_BOX_DEBUG_ENABLED == 1
	logPrintf("BB st=%u ring=%lu drop=%lu syncReq=%u meta=%u syncTmo=%lu fs=%u err=%u free=%lu sdIdle=%u sdFault=%u\r\n", (unsigned int) blackboxState, (unsigned long) blackboxRingBufferUsed(), (unsigned long) blackboxDroppedBytes, (unsigned int) blackboxSyncRequested,
			(unsigned int) blackboxSyncMetadataSaved, (unsigned long) blackboxSyncTimeouts, (unsigned int) afatfs_getFilesystemState(), (unsigned int) afatfs_getLastError(), (unsigned long) afatfs_getFreeBufferSpace(), (unsigned int) sdcard_isOperationIdle(),
			(unsigned int) sdcard_isFaulted());
#endif

}
