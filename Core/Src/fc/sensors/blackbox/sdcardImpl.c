#include "sdcard.h"

#include "bsp_driver_sd.h"
#include "stm32h7xx_hal.h"
#include "core_cm7.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define SDCARD_BLOCK_SIZE           512U
#define SDCARD_CACHE_LINE_SIZE       32U

#define SDCARD_OP_TIMEOUT_MS        500U  /* one 512-byte block never takes this long */
#define SDCARD_MAX_RETRIES            3U
#define SDCARD_FAULT_AFTER_FAILURES   3U  /* consecutive failed operations before the card is latched as faulted */

extern SD_HandleTypeDef sdHandle;

/* -------------------------------------------------------------------------- */
/* Operation state                                                            */
/* -------------------------------------------------------------------------- */

typedef struct {
	sdcardBlockOperation_e operation;

	uint32_t blockIndex;
	uint8_t *buffer;
	uint32_t callbackData;

	sdcard_operationCompleteCallback_c callback;

	uint32_t startTime;
	uint8_t retries;

	volatile uint8_t active;
	volatile uint8_t dmaComplete;
	volatile uint8_t dmaError;

	volatile uint8_t timedOut;

	/* set after HAL_SD_Abort(): waiting (without blocking) for the card to return to TRANSFER state */
	uint8_t retryPending;
} sdcardOperation_t;

static sdcardOperation_t sdcardOperation;

static sdcard_profilerCallback_c sdcardProfilerCallback;

/* -------------------------------------------------------------------------- */
/* Diagnostics (counters only)                                                */
/* -------------------------------------------------------------------------- */

volatile uint32_t sdWriteCompleteCount = 0;
volatile uint32_t sdReadCompleteCount = 0;
volatile uint32_t sdAbortCount = 0;
volatile uint32_t sdErrorCount = 0;
volatile uint32_t sdLastHalError = 0;
volatile uint32_t sdRetryCount = 0;
volatile uint32_t sdTimeoutCount = 0;
volatile uint32_t sdFailedOpCount = 0;
volatile uint32_t sdMaxOpMs = 0;      /* longest operation start -> completion, for tuning the timeouts */

/*
 * Once an operation has failed for good (timeout, retries exhausted) the card is
 * considered faulted. From then on the driver sends NO commands to the card:
 * every HAL status command to an unresponsive card blocks the main loop for
 * about 5 s (SDMMC_CMDTIMEOUT in the LL driver), which is what makes the whole
 * MCU "dead slow". The application must end the session (see sdcard_isFaulted())
 * and may call sdcard_recover() later, when a blocking re-init is acceptable.
 */
static volatile bool sdcardFaulted = false;
static uint8_t sdcardConsecutiveFailures = 0U;
static uint32_t sdcardLastReadyCheckTick = 0U;

/* -------------------------------------------------------------------------- */
/* Cache maintenance                                                          */
/* -------------------------------------------------------------------------- */

static bool sdcard_isBufferAligned(const uint8_t *buffer) {
	return (((uintptr_t) buffer & (SDCARD_CACHE_LINE_SIZE - 1U)) == 0U);
}

static void sdcard_invalidateCache(uint8_t *buffer) {
	SCB_InvalidateDCache_by_Addr((uint32_t*) buffer, SDCARD_BLOCK_SIZE);
	__DSB();
}

static void sdcard_cleanCache(uint8_t *buffer) {
	SCB_CleanDCache_by_Addr((uint32_t*) buffer, SDCARD_BLOCK_SIZE);
	__DSB();
}

/* -------------------------------------------------------------------------- */
/* Operation handling                                                         */
/* -------------------------------------------------------------------------- */

static bool sdcard_isReady(void) {
	if (sdcardFaulted) {
		return false; /* never talk to a faulted card: each command may block ~5 s */
	}

	return BSP_SD_GetCardState() == SD_TRANSFER_OK;
}

/* No CMD13: true when the driver has no transfer in flight. */
bool sdcard_isOperationIdle(void) {
	return !sdcardOperation.active;
}

bool sdcard_isFaulted(void) {
	return sdcardFaulted;
}

static void sdcard_resetOperation(void) {
	sdcardOperation.active = 0U;

	__DMB();

	sdcardOperation.dmaComplete = 0U;
	sdcardOperation.dmaError = 0U;

	sdcardOperation.callback = NULL;
	sdcardOperation.buffer = NULL;
	sdcardOperation.callbackData = 0U;
	sdcardOperation.retries = 0U;
	sdcardOperation.timedOut = 0U;
	sdcardOperation.retryPending = 0U;
}

/*
 * Blocking re-initialisation of the card (can take hundreds of ms, up to several
 * seconds if the card is dead). Call only when that is acceptable, e.g. at the
 * start of a new logging session while disarmed. Returns true if the card works again.
 */
bool sdcard_recover(void) {
	sdcard_resetOperation();

	HAL_SD_DeInit(&sdHandle);

	if (BSP_SD_Init() != 0) {
		return false;
	}

	sdcardFaulted = false;
	sdcardConsecutiveFailures = 0U;
	sdcardLastReadyCheckTick = 0U;

	return sdcard_isReady();
}

static void sdcard_completeOperation(bool success) {
	sdcard_operationCompleteCallback_c callback = sdcardOperation.callback;
	sdcardBlockOperation_e operation = sdcardOperation.operation;
	uint32_t blockIndex = sdcardOperation.blockIndex;
	uint8_t *buffer = success ? sdcardOperation.buffer : NULL;
	uint32_t callbackData = sdcardOperation.callbackData;
	uint32_t duration = HAL_GetTick() - sdcardOperation.startTime;

	if (duration > sdMaxOpMs) {
		sdMaxOpMs = duration;
	}

	if (success) {
		sdcardConsecutiveFailures = 0U;
	} else {
		sdFailedOpCount++;

		/*
		 * A timeout means the card stopped answering: latch the fault at once.
		 * Other failures (e.g. HAL errors that survived the retries) are handed back to
		 * AsyncFatFS, which re-marks the sector dirty and tries again; only a run of
		 * consecutive failures latches the fault.
		 */
		if (sdcardOperation.timedOut || ++sdcardConsecutiveFailures >= SDCARD_FAULT_AFTER_FAILURES) {
			sdcardFaulted = true;
		}
	}

	/*
	 * Clear operation state before invoking the callback.
	 * This permits AsyncFatFS to queue another operation.
	 */
	sdcard_resetOperation();

	if (sdcardProfilerCallback != NULL) {
		sdcardProfilerCallback(operation, blockIndex, duration);
	}

	if (callback != NULL) {
		callback(operation, blockIndex, buffer, callbackData);
	}
}

/*
 * Start (or restart) the transfer described by sdcardOperation.
 * Does the cache maintenance and issues the HAL DMA call.
 */
static HAL_StatusTypeDef sdcard_startTransfer(void) {
	sdcardOperation.dmaComplete = 0U;
	sdcardOperation.dmaError = 0U;
	sdcardOperation.startTime = HAL_GetTick();

	__DMB();

	if (sdcardOperation.operation == SDCARD_BLOCK_OPERATION_READ) {
		/* Invalidate before DMA so stale/dirty lines cannot overwrite received data. */
		sdcard_invalidateCache(sdcardOperation.buffer);

		return (HAL_StatusTypeDef) BSP_SD_ReadBlocks_DMA((uint32_t*) sdcardOperation.buffer, sdcardOperation.blockIndex, 1U);
	}

	/* Make sure the DMA engine sees the latest CPU-written data. */
	sdcard_cleanCache(sdcardOperation.buffer);

	return (HAL_StatusTypeDef) BSP_SD_WriteBlocks_DMA((uint32_t*) sdcardOperation.buffer, sdcardOperation.blockIndex, 1U);
}

/* -------------------------------------------------------------------------- */
/* Read                                                                       */
/* -------------------------------------------------------------------------- */

bool sdcard_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData) {
	if (buffer == NULL || callback == NULL) {
		return false;
	}

	if (sdcardOperation.active) {
		return false;
	}

	if (!sdcard_isBufferAligned(buffer)) {
		return false;
	}

	if (sdcardFaulted) {
		return false; /* no CMD13 here: the previous operation completed only after TRANSFER state was seen */
	}

	sdcardOperation.operation = SDCARD_BLOCK_OPERATION_READ;
	sdcardOperation.blockIndex = blockIndex;
	sdcardOperation.buffer = buffer;
	sdcardOperation.callbackData = callbackData;
	sdcardOperation.callback = callback;
	sdcardOperation.retries = 0U;

	sdcardOperation.active = 1U;

	if (sdcard_startTransfer() != HAL_OK) {
		sdcard_resetOperation();
		return false;
	}

	return true;
}

/* -------------------------------------------------------------------------- */
/* Write                                                                      */
/* -------------------------------------------------------------------------- */

sdcardOperationStatus_e sdcard_writeBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData) {
	if (buffer == NULL || callback == NULL) {
		return SDCARD_OPERATION_FAILURE;
	}

	if (sdcardOperation.active) {
		return SDCARD_OPERATION_BUSY;
	}

	if (!sdcard_isBufferAligned(buffer)) {
		return SDCARD_OPERATION_FAILURE;
	}

	if (sdcardFaulted) {
		return SDCARD_OPERATION_BUSY; /* no CMD13 here, see sdcard_readBlock */
	}

	sdcardOperation.operation = SDCARD_BLOCK_OPERATION_WRITE;
	sdcardOperation.blockIndex = blockIndex;
	sdcardOperation.buffer = buffer;
	sdcardOperation.callbackData = callbackData;
	sdcardOperation.callback = callback;
	sdcardOperation.retries = 0U;

	sdcardOperation.active = 1U;

	if (sdcard_startTransfer() != HAL_OK) {
		sdcard_resetOperation();
		return SDCARD_OPERATION_FAILURE;
	}

	return SDCARD_OPERATION_IN_PROGRESS;
}

/* -------------------------------------------------------------------------- */
/* HAL / BSP completion callbacks (interrupt context: keep them tiny)         */
/* -------------------------------------------------------------------------- */

void BSP_SD_ReadCpltCallback(void) {
	sdReadCompleteCount++;

	if (sdcardOperation.active && sdcardOperation.operation == SDCARD_BLOCK_OPERATION_READ) {
		sdcardOperation.dmaComplete = 1U;
		__DMB();
	}
}

void BSP_SD_WriteCpltCallback(void) {
	sdWriteCompleteCount++;

	if (sdcardOperation.active && sdcardOperation.operation == SDCARD_BLOCK_OPERATION_WRITE) {
		sdcardOperation.dmaComplete = 1U;
		__DMB();
	}
}

void BSP_SD_AbortCallback(void) {
	sdAbortCount++;

	if (sdcardOperation.active) {
		sdcardOperation.dmaError = 1U;
		sdcardOperation.dmaComplete = 1U;
		__DMB();
	}
}

/*
 * Called by the HAL on TX underrun, RX overrun, CRC error, data timeout...
 * Without this a failed DMA never completes and the operation hangs forever.
 */
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
	sdErrorCount++;
	sdLastHalError = hsd->ErrorCode;

	if (sdcardOperation.active) {
		sdcardOperation.dmaError = 1U;
		sdcardOperation.dmaComplete = 1U;
		__DMB();
	}
}

/* -------------------------------------------------------------------------- */
/* Poll                                                                       */
/* -------------------------------------------------------------------------- */

/* Returns true if a retry was scheduled, false if the operation must be failed. */
static bool sdcard_retryOperation(void) {
	if (sdcardOperation.retries >= SDCARD_MAX_RETRIES) {
		return false;
	}

	sdcardOperation.retries++;
	sdRetryCount++;

	/* Bring the card and the SDMMC peripheral back to a clean transfer state. */
	HAL_SD_Abort(&sdHandle);

	/*
	 * Do not wait for the card here: sdcard_poll() checks (at most once per ms)
	 * whether the card is back in TRANSFER state and then restarts the transfer.
	 */
	sdcardOperation.dmaComplete = 0U;
	sdcardOperation.dmaError = 0U;
	sdcardOperation.startTime = HAL_GetTick();
	sdcardOperation.retryPending = 1U;

	return true;
}

bool sdcard_poll(void) {
	uint32_t now = HAL_GetTick();

	if (sdcardFaulted) {
		return false;
	}

	if (!sdcardOperation.active) {
		return true;
	}

	/* A retry is scheduled: wait (polling at most once per ms) for the card, then restart. */
	if (sdcardOperation.retryPending) {
		if ((uint32_t) (now - sdcardOperation.startTime) > SDCARD_OP_TIMEOUT_MS) {
			sdTimeoutCount++;
			sdcardOperation.timedOut = 1U;
			sdcard_completeOperation(false);
			return false;
		}

		if (now == sdcardLastReadyCheckTick) {
			return false;
		}
		sdcardLastReadyCheckTick = now;

		if (!sdcard_isReady()) {
			return false;
		}

		sdcardOperation.retryPending = 0U;

		if (sdcard_startTransfer() != HAL_OK) {
			sdcard_completeOperation(false);
		}

		return false;
	}

	/* Detect a transfer that never completes. */
	if (!sdcardOperation.dmaComplete && (uint32_t) (now - sdcardOperation.startTime) > SDCARD_OP_TIMEOUT_MS) {
		sdTimeoutCount++;
		sdcardOperation.timedOut = 1U;
		sdcardOperation.dmaError = 1U;
	}

	/* Handle DMA errors and timeouts. */
	if (sdcardOperation.dmaError) {
		/*
		 * Retry HAL-reported errors.
		 * Do not retry a transfer that timed out.
		 */
		if (!sdcardOperation.timedOut && sdcard_retryOperation()) {
			return false;
		}
		sdcard_completeOperation(false);
		return false;
	}

	if (!sdcardOperation.dmaComplete) {
		return false;
	}

	/* A finished single-block read leaves the card in TRANSFER state: no CMD13 needed. */
	if (sdcardOperation.operation == SDCARD_BLOCK_OPERATION_READ) {
		sdcard_invalidateCache(sdcardOperation.buffer);
		sdcard_completeOperation(true);
		return !sdcardFaulted;
	}

	/*
	 * DMA completion does not necessarily mean
	 * the card has finished internal programming (writes only).
	 */
	if ((uint32_t) (now - sdcardOperation.startTime) > (SDCARD_OP_TIMEOUT_MS * 4U)) {
		sdTimeoutCount++;
		sdcardOperation.timedOut = 1U;
		sdcard_completeOperation(false);
		return false;
	}

	/*
	 * Ask the card at most once per millisecond (each CMD13 is a short busy-wait on the
	 * SDMMC response). A 512-byte block programs in about 1-5 ms, so this costs
	 * almost no throughput but keeps the main loop from spending its time in CMD13.
	 */
	if (now == sdcardLastReadyCheckTick) {
		return false;
	}
	sdcardLastReadyCheckTick = now;

	if (!sdcard_isReady()) {
		return false;
	}

	sdcard_completeOperation(true);
	return !sdcardFaulted; /* the card was in TRANSFER state a moment ago: no extra CMD13 */
}

/* -------------------------------------------------------------------------- */
/* Optional multi-block write optimization                                    */
/* -------------------------------------------------------------------------- */

sdcardOperationStatus_e sdcard_beginWriteBlocks(uint32_t blockIndex, uint32_t blockCount) {
	(void) blockIndex;
	(void) blockCount;
	return SDCARD_OPERATION_SUCCESS;
}

sdcardOperationStatus_e sdcard_endWriteBlocks(void) {
	return SDCARD_OPERATION_SUCCESS;
}

uint32_t sdcard_getMaxOpMs(){
	return sdMaxOpMs;
}
