#include "FSIA.h"

#include "../../../logger/Logger.h"
#include "../../../io/uart/UART.h"
#include "../../../dsp/CircularQueue.h"
#include "../../../memory/Memory.h"


#define FSIA_BUFFSIZE 32
#define FSIA_CKSUM_LSB_INDX FSIA_BUFFSIZE-2
#define FSIA_CKSUM_HSB_INDX FSIA_BUFFSIZE-1

uint16_t fsiaChannelValue[FSIA_CHANNEL_COUNT] = { 0 };
uint8_t fsiaFrameDataIndex = 0;
uint8_t fsiaFrameData[FSIA_BUFFSIZE] = { 0 };

uint8_t fsiaPrevDataByte = 0;
uint16_t fsiaChksum = 0xFF9F; //0xFFFF-0x40-0x20
volatile uint8_t fsiaHasData = 0;

//This buffer need to be in DMA accessible memory
__ATTR_RAM_D2 uint8_t fsiaIOReadBuffer[FSIA_BUFFSIZE];
uint8_t fsiaIOReadSize = FSIA_BUFFSIZE;

//Circular queue configurations
CircularQueue fsiaIOQueue;
#define CIRCULAR_QUEUE_SIZE FSIA_BUFFSIZE * 2
#define CIRCULAR_QUEUE_READ_SIZE CIRCULAR_QUEUE_SIZE/4
uint8_t fsiaCircularQueueReadBuffer[CIRCULAR_QUEUE_READ_SIZE];
uint8_t updateFSIAData(uint8_t *dataBytes, uint16_t length);

uint16_t getFSIAFrameRate(void) {
	return FSIA_DATA_FRAME_FREQUENCY;
}

void _processFSAIData(uint8_t *data, uint16_t len) {
	fsiaHasData = 1;
	circularQueueWrite(&fsiaIOQueue, data, len);
}

/**
 * Initializes the iBUS protocol
 **/
uint8_t initFSIA() {
	if (uart4Init()) {
		logString("[FSAI] : IO:UART > Success\n");
		if (uart4ReadStart(fsiaIOReadBuffer, fsiaIOReadSize, _processFSAIData)) {
			circularQueueInit(&fsiaIOQueue, CIRCULAR_QUEUE_SIZE);
			logString("[FSAI] : IO , UART Read start > Success\n");
			return 1;
		} else {
			logString("[FSAI] : IO , UART Read start > Failed\n");
		}
	} else {
		logString("[FSAI] IO:UART > Failure\n");
	}
	return 0;
}

uint8_t readFSIA(void) {
	if (circularQueueAvailableData(&fsiaIOQueue) > 0) {
		uint8_t queuReadLength = circularQueueRead(&fsiaIOQueue, fsiaCircularQueueReadBuffer, CIRCULAR_QUEUE_READ_SIZE);
		return updateFSIAData(fsiaCircularQueueReadBuffer, queuReadLength);
	}
	return 0;
}

/**
 * Frame example:
 * x20x40   = Header (length of 32 bytes + command)
 * xDCx05  = 1500 ch 1
 * xDBx05  = 1499 ch 2
 * xEFx03  = 1007 ch 3
 * xDDx05  = 1501 ch 4
 * xD0x07  = 2000 ch 5
 * xD0x07  = 2000 ch 6
 * xDCx05  = 1500 ch 7
 * xDCx05  = 1500 ch 8
 * xDCx05  = 1500 ch 9
 * xDCx05  = 1500 ch 10
 * xDCx05  = 1500 ch 11
 * xDCx05  = 1500 ch 12
 * xDCx05  = 1500 ch 13
 * xDCx05  = 1500 ch 14
 * x54xF3  = Checksum:  0xFFFF - (0x20 + 0x40 ... sum of all !)
 **/

uint8_t updateFSIAData(uint8_t *dataBytes, uint16_t length) {
	uint8_t frameComplete = 0;
	for (uint16_t indx = 0; indx < length; indx++) {
		uint8_t data = dataBytes[indx];
		if (fsiaFrameDataIndex == 0) {
			// State 1: Looking for Header Byte 1 (0x20)
			if (data == 0x20) {
				fsiaFrameData[0] = data;
				fsiaFrameDataIndex = 1;
			}
		} else if (fsiaFrameDataIndex == 1) {
			// State 2: Looking for Header Byte 2 (0x40)
			if (data == 0x40) {
				fsiaFrameData[1] = data;
				fsiaFrameDataIndex = 2; // Move to Payload state
				fsiaChksum = 0xFF9F; // Reset checksum
			} else {
				// Unexpected byte (e.g., 0x20 0x1A), reset alignment
				fsiaFrameDataIndex = 0;
			}
		} else if (fsiaFrameDataIndex > 1 && fsiaFrameDataIndex < FSIA_BUFFSIZE) {
			// State 3: Reading Payload and Checksum
			fsiaFrameData[fsiaFrameDataIndex] = data;
			// 3A: Checksum Calculation (Bytes 2 to 29)
			if (fsiaFrameDataIndex < FSIA_CKSUM_LSB_INDX) {
				fsiaChksum -= data;
			}
			// 3B: Checksum Verification (Last byte arrived)
			if (fsiaFrameDataIndex == FSIA_CKSUM_HSB_INDX) {
				uint16_t rxChksum = fsiaFrameData[FSIA_CKSUM_LSB_INDX] | (fsiaFrameData[FSIA_CKSUM_HSB_INDX] << 8);
				if (fsiaChksum == rxChksum) {
					// Success: Extract channel data
					for (int i = 0; i < FSIA_CHANNEL_COUNT; i++) {
						fsiaChannelValue[i] = fsiaFrameData[2 + 2 * i] | (fsiaFrameData[3 + 2 * i] << 8);
					}
					frameComplete = 1;
				}
				// Reset regardless of success/fail
				fsiaFrameDataIndex = 0;
			} else {
				fsiaFrameDataIndex++;
			}
		}
		// Store for checking header start in the next iteration
		fsiaPrevDataByte = data;
	}
	return frameComplete;
}

/**
 * Gets the value of RC Channel
 **/
uint16_t getFSIAChannelValue(uint8_t channel) {
	return fsiaChannelValue[channel];
}

void resetFSIAState() {
	fsiaHasData = 0;
}

uint8_t isFSIAActive() {
	if (fsiaHasData) {
		fsiaHasData = 0;
		return 1;
	}
	return 0;
}
