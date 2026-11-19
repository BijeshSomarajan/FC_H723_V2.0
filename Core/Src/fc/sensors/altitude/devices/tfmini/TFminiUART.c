#include "TFMini.h"
#if TFMINI_IO_TYPE == TFMINI_IO_TYPE_UART
#include "../AltitudeDevice.h"
#include "../../../../io/uart/UART.h"
#include "../../../../memory/Memory.h"

#define TFMINI_UART_FRAME_LEN        9
/* DMA length: two halves of one frame each, so the callback fires about once per frame.
 * The buffer array is padded to a full 32-byte cache line so the cache invalidation done by
 * UART8.c can never touch a neighbouring variable. */
#define TFMINI_UART_RX_DMA_LEN       (2 * TFMINI_UART_FRAME_LEN)
#define TFMINI_UART_RX_BUFFER_SIZE   32
#define TFMINI_UART_STARTUP_WAIT_MS  1000  /* wait this long for the first frame at init */
#define TFMINI_UART_RESET_WAIT_MS    1000  /* manual: keep power on and wait 1 s after a system reset */
#define TFMINI_UART_STALE_POLLS      20    /* deviceLidarRead() calls without a new frame before the sensor counts as silent (about 200 ms at 100 Hz) */

TFMini tfMini;

volatile uint8_t tfMiniHasData = 0;

__ATTR_RAM_D2 static uint8_t tfMiniUartRxBuffer[TFMINI_UART_RX_BUFFER_SIZE] __attribute__((aligned(32)));

/* ---------- Parser state (touched only from the DMA callback) ---------- */
static uint8_t tfMiniUartWin[TFMINI_UART_FRAME_LEN]; /* the last 9 bytes received, newest last */

/* ---------- Hand-off from the callback to the main context ---------- */
/* Everything shared is volatile and copied byte by byte, so the compiler cannot reorder the
 * data accesses against the counters. No separate compiler barrier is needed. */

/* Newest valid data frame. A sequence counter (odd while a write is in progress) lets the
 * reader detect a torn copy and retry. */
static volatile uint8_t tfMiniUartLatest[TFMINI_UART_FRAME_LEN];
static volatile uint32_t tfMiniUartSeq = 0;
static volatile uint32_t tfMiniUartFrameCount = 0; /* valid data frames received since boot */

/* Silent-sensor watchdog (main context only) */
static uint32_t tfMiniUartLastCount = 0;
static uint32_t tfMiniUartStalePolls = 0;

/* Sliding-window parser. Every byte shifts into a window of the last 9 bytes; a data frame is
 * accepted when the window holds 59 59 ... with a valid checksum. There is no sync state: garbage,
 * a dropped byte or a header-like pair inside a payload simply never validates, and the next real
 * frame is found as soon as its last byte arrives. Command responses (5A ...) never match. */
static void tfMiniUartParseByte(uint8_t b) {
	memmove(tfMiniUartWin, tfMiniUartWin + 1, TFMINI_UART_FRAME_LEN - 1);
	tfMiniUartWin[TFMINI_UART_FRAME_LEN - 1] = b;

	if (tfMiniUartWin[0] != 0x59 || tfMiniUartWin[1] != 0x59) {
		return;
	}
	uint8_t sum = 0;
	for (uint8_t i = 0; i < TFMINI_UART_FRAME_LEN - 1; i++) {
		sum += tfMiniUartWin[i];
	}
	if (sum != tfMiniUartWin[TFMINI_UART_FRAME_LEN - 1]) {
		return;
	}

	tfMiniUartSeq++; /* odd: write in progress */
	for (uint8_t i = 0; i < TFMINI_UART_FRAME_LEN; i++) {
		tfMiniUartLatest[i] = tfMiniUartWin[i];
	}
	tfMiniUartSeq++; /* even: stable */
	tfMiniUartFrameCount++;
	tfMiniHasData = 1;
	memset(tfMiniUartWin, 0, sizeof(tfMiniUartWin)); /* consumed: the next frame needs 9 fresh bytes */
}

/* Called by UART8.c from the DMA interrupt with each received half buffer */
static void __deviceLidarTFMiniUartRxCallback(uint8_t *data, uint16_t len) {
	if (data == NULL) {
		return;
	}
	for (uint16_t i = 0; i < len; i++) {
		tfMiniUartParseByte(data[i]);
	}
}

/* (Re)start circular DMA reception. UART8.c also clears the UART error flags and flushes
 * the receiver here, which recovers from an overrun or framing error. */
static uint8_t tfMiniUartStartRx(void) {
	memset(tfMiniUartWin, 0, sizeof(tfMiniUartWin));
	return uart8ReadStart(tfMiniUartRxBuffer, TFMINI_UART_RX_DMA_LEN, __deviceLidarTFMiniUartRxCallback);
}

/* ---------- Configuration commands ---------- */
uint8_t tfMiniSaveSettings() {
	uint8_t cmd[4] = { 0x5A, 0x04, 0x11, 0x6F };
	uart8Write(cmd, 4);
	delayMs(100); /* no response is read, so give the sensor time to apply it */
	return 1;
}

uint8_t tfMiniSet100FrameRate() {
	uint8_t cmd[6] = { 0x5A, 0x06, 0x03, 0x64, 0x00, 0xC7 };
	uart8Write(cmd, 6);
	delayMs(100); /* no response is read, so give the sensor time to apply it */
	return 1;
}

/* System reset. The manual's reset response bytes are inconsistent with its own checksum rule, so the
 * response is not used. Success means the data stream is back after the reset. */
uint8_t tfMiniResetDevice() {
	uint8_t cmd[4] = { 0x5A, 0x04, 0x02, 0x60 };
	uart8Write(cmd, 4);
	delayMs(TFMINI_UART_RESET_WAIT_MS);
	/* The sensor restarted its UART: restart reception to clear any error it caused */
	tfMiniUartStartRx();
	uint32_t base = tfMiniUartFrameCount;
	uint16_t waited = 0;
	while (tfMiniUartFrameCount == base && waited < TFMINI_UART_STARTUP_WAIT_MS) {
		delayMs(10);
		waited += 10;
	}
	return tfMiniUartFrameCount != base;
}

/* Switch the sensor from UART to I2C output (command ID 0x0A, mode 1 = I2C), the counterpart of
 * tfMiniSwitchToUart() in the I2C build. Same bytes for TFMini Plus and TFMini-S. Returns 1 if the
 * save was acknowledged. Afterwards the sensor answers on I2C (address 0x10); if it still streams on
 * UART, power-cycle it. Call it once: the save writes the sensor's flash. */
uint8_t tfMiniSwitchToI2C() {
	uint8_t cmdSetI2C[5] = { 0x5A, 0x05, 0x0A, 0x01, 0x6A }; /* interface = I2C (no response documented) */
	uint8_t cmdSave[4] = { 0x5A, 0x04, 0x11, 0x6F };
	uint8_t cmdReset[4] = { 0x5A, 0x04, 0x02, 0x60 };
	uart8Write(cmdSetI2C, sizeof(cmdSetI2C));
	delayMs(100);
	uart8Write(cmdSave, sizeof(cmdSave));
	delayMs(100);
	uart8Write(cmdReset, sizeof(cmdReset)); /* best effort */
	delayMs(TFMINI_UART_RESET_WAIT_MS);
	logString("[TFMini] Switched to I2C mode (power-cycle the sensor if it still streams on UART)\n");
	return 1;
}

uint8_t deviceLidarInit() {
	uint8_t status = uart8Init();
	if (!status) {
		logString("[TFMini] UART Init Failed\n");
		return 0;
	}
	logString("[TFMini] UART Init Success\n");

	status = tfMiniUartStartRx();
	if (!status) {
		logString("[TFMini] UART Read Start Failed\n");
		return 0;
	}
	logString("[TFMini] UART Read Start Success\n");

#if TFMINI_UART_SWITCH_TO_I2C == 1
	tfMiniSwitchToI2C();
	return 0;
#endif
	/* The sensor streams on its own: wait for the first valid frame */
	uint16_t waited = 0;
	while (tfMiniUartFrameCount == 0 && waited < TFMINI_UART_STARTUP_WAIT_MS) {
		delayMs(10);
		waited += 10;
	}
	if (tfMiniUartFrameCount == 0) {
		logString("[TFMini] No UART frames (check wiring and that the sensor is in UART mode)\n");
		return 0;
	}
	logString("[TFMini] UART Frames Received\n");

	status = tfMiniResetDevice();
	if (!status) {
		logString("[TFMini] Not streaming after reset\n");
		return 0;
	}
	logString("[TFMini] Reset Success\n");

	/* The sensor is streaming, so it is usable even if a configuration step is not confirmed:
	 * these two are logged but do not fail the init. */
	status = tfMiniSet100FrameRate();
	logString(status ? "[TFMini] Frame Rate Config Success\n" : "[TFMini] Frame Rate Config Failed\n");

	status = tfMiniSaveSettings();
	logString(status ? "[TFMini] Config Save Success\n" : "[TFMini] Config Save Failed\n");
	return 1;
}

void deviceLidarDataProcess(void) {
	uint8_t ckSum = tfMini.buffer[8];
	uint16_t ckSumCalc = 0;
	for (uint8_t indx = 0; indx < 8; indx++) {
		ckSumCalc += tfMini.buffer[indx];
	}
	if ((ckSumCalc & 0xFF) == ckSum) {
		tfMini.distance = ((float) ((uint16_t) (tfMini.buffer[2] + ((uint16_t) tfMini.buffer[3] << 8)))) / 100.0f;
		tfMini.strength = (uint16_t) (tfMini.buffer[4] + ((uint16_t) tfMini.buffer[5] << 8));
		deviceAltitudeData.altitudeTerrainQlty = mapAndClampToRangeFloat(tfMini.strength == 65535 ? 0 : tfMini.strength, TFMINI_MIN_VALID_STRENGTH, TFMINI_MAX_VALID_STRENGTH, 0.0f, 1.0f);
		if (deviceAltitudeData.altitudeTerrainQlty != 0.0f) {
			deviceAltitudeData.altitudeTerrain = tfMini.distance;
		}
	} else {
		deviceAltitudeData.altitudeTerrainQlty = 0;
	}
}

/*
 * Frames arrive by DMA, so there is nothing to start here. This is only a watchdog: if no new
 * data frame arrives for TFMINI_UART_STALE_POLLS calls (sensor unplugged, powered down, or the
 * UART stopped after an error), the altitude quality is set to 0 so an old reading is not reported
 * as valid, and reception is restarted.
 */
uint8_t deviceLidarRead(void) {
	uint32_t count = tfMiniUartFrameCount;
	if (count != tfMiniUartLastCount) {
		tfMiniUartLastCount = count;
		tfMiniUartStalePolls = 0;
	} else if (++tfMiniUartStalePolls >= TFMINI_UART_STALE_POLLS) {
		tfMiniUartStalePolls = 0;
		deviceAltitudeData.altitudeTerrainQlty = 0;
		tfMiniUartStartRx();
	}
	return 1;
}

uint8_t deviceLidarLoadData(void) {
	if (!tfMiniHasData) {
		return 0;
	}
	/* Clear the flag BEFORE copying: if a newer frame lands during the copy, the flag is set
	 * again and that frame is processed on the next call. */
	tfMiniHasData = 0;

	uint8_t frame[TFMINI_UART_FRAME_LEN];
	uint32_t seq;
	uint8_t tries = 0;
	do {
		seq = tfMiniUartSeq;
		for (uint8_t i = 0; i < TFMINI_UART_FRAME_LEN; i++) {
			frame[i] = tfMiniUartLatest[i];
		}
	} while (((seq & 1U) || seq != tfMiniUartSeq) && ++tries < 4);

	if ((seq & 1U) || seq != tfMiniUartSeq) {
		tfMiniHasData = 1; /* could not get a stable copy: try again on the next call */
		return 0;
	}

	memcpy(tfMini.buffer, frame, TFMINI_UART_FRAME_LEN);
	deviceLidarDataProcess();
	return 1;
}

uint8_t deviceLidarReset(uint8_t hard) {
	deviceAltitudeData.altitudeTerrain = 0;
	deviceAltitudeData.altitudeTerrainQlty = 0;
	return 1;
}

#endif
