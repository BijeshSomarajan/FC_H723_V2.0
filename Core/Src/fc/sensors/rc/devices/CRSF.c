#include "CRSF.h"
#include "../../../logger/Logger.h"
#include "../../../io/uart/UART.h"
#include "../../../dsp/CircularQueue.h"
#include "../../../memory/Memory.h"

// Internal state & channel storage
static uint16_t crsfChannelValue[16] = { 1500 }; // Default to mid-point stick PWM
static uint8_t crsfFrameData[CRSF_MAX_FRAME_SIZE] = { 0 };
static uint8_t crsfFrameDataIndex = 0;
static uint8_t crsfFrameLength = 0;

volatile uint8_t crsfHasData = 0;
static uint32_t crsfCalculatedFrameRate = 0;
static uint32_t crsfFrameCounter = 0;
static uint32_t crsfRateTimer = 0;

// This buffer must reside in DMA accessible memory (D2 Domain) and be 32-byte aligned for H7 Cache
__ATTR_RAM_D2 uint8_t crsfIOReadBuffer[CRSF_BUFFSIZE];
uint8_t crsfIOReadSize = CRSF_BUFFSIZE;

// Circular queue configurations
CircularQueue crsfIOQueue;
#define CRSF_CIRCULAR_QUEUE_SIZE (CRSF_BUFFSIZE * 2)
#define CRSF_CIRCULAR_QUEUE_READ_SIZE (CRSF_CIRCULAR_QUEUE_SIZE / 4)
uint8_t crsfCircularQueueReadBuffer[CRSF_CIRCULAR_QUEUE_READ_SIZE];
CRSF_CHANNELS channels;
// Forward declaration of internal parser functions
uint8_t updateCRSFData(uint8_t *dataBytes, uint16_t length);
void processCRSFFrame(uint8_t frameType, const uint8_t *payload);

/**
 * @brief DMA RX ISR Callback - routes raw byte bursts directly into the circular queue
 */
void _processCRSFData(uint8_t *data, uint16_t len) {
	crsfHasData = 1;
	circularQueueWrite(&crsfIOQueue, data, len);
}

/**
 * @brief Initializes the UART4 peripheral and internal processing queues
 */
uint8_t initCRSF(void) {
	// ExpressLRS baseline operates at 420000 baud
	if (uart4Init(UART_BAUD_RATE_420000)) {
		logString("[CRSF] : IO:UART > Success\n");
		if (uart4ReadStart(crsfIOReadBuffer, crsfIOReadSize, _processCRSFData)) {
			circularQueueInit(&crsfIOQueue, CRSF_CIRCULAR_QUEUE_SIZE);
			logString("[CRSF] : IO , UART Read start > Success\n");
			return 1;
		} else {
			logString("[CRSF] : IO , UART Read start > Failed\n");
		}
	} else {
		logString("[CRSF] IO:UART > Failure\n");
	}
	return 0;
}

/**
 * @brief Pulls data out of the thread-safe circular queue and executes the parser
 */
uint8_t readCRSF(void) {
	if (circularQueueAvailableData(&crsfIOQueue) > 0) {
		uint8_t queueReadLength = circularQueueRead(&crsfIOQueue, crsfCircularQueueReadBuffer, CRSF_CIRCULAR_QUEUE_READ_SIZE);
		return updateCRSFData(crsfCircularQueueReadBuffer, queueReadLength);
	}
	return 0;
}

uint16_t convert_to_pwm(uint16_t raw_value) {
	return (uint16_t) (((raw_value - 992) * 5) >> 3) + 1500;
}

/**
 * @brief DVB-S2 variant CRC8 validation function
 */
uint8_t crsf_calculate_crc(const uint8_t *data, uint8_t length) {
	uint8_t crc = 0;
	while (length--) {
		crc ^= *data++;
		for (int i = 0; i < 8; i++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0xD5;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

/**
 * @brief Variable-length Frame State Machine Parser
 */
uint8_t updateCRSFData(uint8_t *dataBytes, uint16_t length) {
	uint8_t frameComplete = 0;

	for (uint16_t indx = 0; indx < length; indx++) {
		uint8_t data = dataBytes[indx];

		if (crsfFrameDataIndex == 0) {
			// State 1: Look for Sync Byte (0xC8)
			if (data == CRSF_SYNC_BYTE) {
				crsfFrameData[0] = data;
				crsfFrameDataIndex = 1;
			}
		} else if (crsfFrameDataIndex == 1) {
			// State 2: Read Length Byte
			// Valid length includes Type byte + Payload + CRC byte (Min 2)
			if (data >= 2 && data <= (CRSF_MAX_FRAME_SIZE - 2)) {
				crsfFrameLength = data;
				crsfFrameData[1] = data;
				crsfFrameDataIndex = 2;
			} else {
				crsfFrameDataIndex = 0; // Malformed length, drop allocation and reset sync
			}
		} else {
			// State 3: Collect Frame Type, Payload, and Checksum
			crsfFrameData[crsfFrameDataIndex] = data;
			crsfFrameDataIndex++;

			// Total frame size expected = 1 (Addr) + 1 (Len) + crsfFrameLength
			if (crsfFrameDataIndex == (crsfFrameLength + 2)) {
				uint8_t rxCrc = data;

				// CRC calculation covers Type up to the end of the Payload (Length - 1)
				uint8_t calculatedCrc = crsf_calculate_crc(&crsfFrameData[2], crsfFrameLength - 1);

				if (calculatedCrc == rxCrc) {
					uint8_t frameType = crsfFrameData[2];

					// Core structural change: Routes directly to our type-safe handler
					processCRSFFrame(frameType, &crsfFrameData[3]);

					if (frameType == CRSF_FRAME_RC_CHANNELS) {
						frameComplete = 1;
					}
				}
				// Frame finished processing (or failed validation), reset parser layout
				crsfFrameDataIndex = 0;
			}
		}
	}
	return frameComplete;
}

/**
 * @brief Evaluates incoming validated frames based on structural type macros
 */
void processCRSFFrame(uint8_t frameType, const uint8_t *payload) {
	switch (frameType) {
	case CRSF_FRAME_RC_CHANNELS: {
		// Cast raw sequence memory directly to our packed 11-bit structural type
		const CRSF_CHANNELS *channels = (const CRSF_CHANNELS*) payload;
		// Extract data without shifting loops
		crsfChannelValue[0] = convert_to_pwm(channels->ch0);
		crsfChannelValue[1] = convert_to_pwm(channels->ch1);
		crsfChannelValue[2] = convert_to_pwm(channels->ch2);
		crsfChannelValue[3] = convert_to_pwm(channels->ch3);
		crsfChannelValue[4] = convert_to_pwm(channels->ch4);
		crsfChannelValue[5] = convert_to_pwm(channels->ch5);
		crsfChannelValue[6] = convert_to_pwm(channels->ch6);
		crsfChannelValue[7] = convert_to_pwm(channels->ch7);
		crsfChannelValue[8] = convert_to_pwm(channels->ch8);
		crsfChannelValue[9] = convert_to_pwm(channels->ch9);
		crsfChannelValue[10] = convert_to_pwm(channels->ch10);
		crsfChannelValue[11] = convert_to_pwm(channels->ch11);
		crsfChannelValue[12] = convert_to_pwm(channels->ch12);
		crsfChannelValue[13] = convert_to_pwm(channels->ch13);
		crsfChannelValue[14] = convert_to_pwm(channels->ch14);
		crsfChannelValue[15] = convert_to_pwm(channels->ch15);
		// Track internal frame statistics
		crsfFrameCounter++;
		// Note: Replace '0' with system millisecond tracker (e.g., HAL_GetTick()) if available
		uint32_t currentTime = 0;
		if (currentTime - crsfRateTimer >= 1000) {
			crsfCalculatedFrameRate = crsfFrameCounter;
			crsfFrameCounter = 0;
			crsfRateTimer = currentTime;
		}
		break;
	}
		// Placeholder for adding future frame types (e.g., Link Statistics 0x14)
	default:
		break;
	}
}

/**
 * @brief Scale 11-bit values directly to standard 1000us-2000us PWM limits
 */

/**
 * @brief External Getter for processed channel outputs (returns 1000-2000 PWM)
 */
uint16_t getCRSFChannelValue(uint8_t channel) {
	if (channel < 16) {
		return crsfChannelValue[channel];
	}
	return 1500;
}

/**
 * @brief External Setter to manually override internal channel registers
 */
void setCRSFChannelValue(uint8_t channel, uint16_t value) {
	if (channel < 16) {
		crsfChannelValue[channel] = value;
	}
}

/**
 * @brief Resets the active hardware data status flags
 */
void resetCRSFState(void) {
	crsfHasData = 0;
}

/**
 * @brief Checks if fresh data blocks have been pushed by DMA since the last evaluation
 */
uint8_t isCRSFActive(void) {
	if (crsfHasData) {
		crsfHasData = 0;
		return 1;
	}
	return 0;
}

/**
 * @brief Returns the calculated link frame frequency
 */
uint16_t getCRSFFrameRate(void) {
	return crsfCalculatedFrameRate;
}



