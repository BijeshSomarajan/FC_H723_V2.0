#include <string.h>
#include <sys/_stdint.h>
#include <math.h>

#include "../../../dsp/CircularQueue.h"
#include "../../../io/uart/UART.h"
#include "../../../logger/Logger.h"
#include "../../../memory/Memory.h"
#include "../../../timers/DelayTimer.h"
#include "../GNSS.h"

#define UBX_RCV_BUFFER_SIZE       100
#define UBX_CIRCULAR_QUEUE_SIZE   (UBX_RCV_BUFFER_SIZE * 2)
#define UBX_READ_BUFFER_SIZE      UBX_RCV_BUFFER_SIZE
#define UBX_NAV_PVT_LEN           92

// Memory section for D2 Domain (DMA compatible)
__ATTR_RAM_D2 static uint8_t ubxDataBuffer[UBX_RCV_BUFFER_SIZE];
uint8_t ubxIOReadSize = UBX_RCV_BUFFER_SIZE;

CircularQueue ubxIOQueue;
uint8_t ubxCircularQueueReadBuffer[UBX_READ_BUFFER_SIZE];

// The Global Data accessed by PositionManager
GNSS_DATA gnssData;

typedef enum {
	IDLE, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B
} UBX_STATE;

typedef struct {
	UBX_STATE state;
	uint8_t classId;
	uint8_t msgId;
	uint16_t payloadIdx;
	uint16_t targetPayloadLen;
	uint8_t ua, ub;
	uint8_t fixType;
	uint8_t numSV;
	int32_t lon;
	int32_t lat;
	int32_t height;
	uint32_t hAcc;
	uint32_t sAcc;
	int32_t velN;
	int32_t velE;
	int32_t velD;
	uint8_t temp[4];
} UBX_CONTEXT;

static UBX_CONTEXT ubxContext = { .state = IDLE };

#define UBX_BUILD_U32(p) ((uint32_t)(p)[0] | ((uint32_t)(p)[1] << 8) | ((uint32_t)(p)[2] << 16) | ((uint32_t)(p)[3] << 24))
#define UBX_BUILD_I32(p) ((int32_t)UBX_BUILD_U32(p))

/**
 * @brief Updates UBX data and prevents partial-read "tearing"
 */
uint8_t updateUBXData(UBX_CONTEXT *p, uint8_t *buffer, uint16_t len) {
	uint8_t packet_ready = 0;
	for (uint16_t i = 0; i < len; i++) {
		uint8_t byte = buffer[i];

		switch (p->state) {
		case IDLE:
			if (byte == 0xB5) {
				p->state = SYNC2;
			}
			break;

		case SYNC2:
			if (byte == 0x62) {
				p->state = CLASS;
				p->ua = 0;
				p->ub = 0;
			} else if (byte == 0xB5) {
				p->state = SYNC2;
			} else {
				p->state = IDLE;
			}
			break;

		case CLASS:
			p->ua += byte;
			p->ub += p->ua;
			p->classId = byte;
			p->state = ID;
			break;

		case ID:
			p->ua += byte;
			p->ub += p->ua;
			p->msgId = byte;
			if (p->classId != 0x01 || p->msgId != 0x07) {
				p->state = IDLE;
				break;
			}
			p->state = LEN1;
			break;

		case LEN1:
			p->ua += byte;
			p->ub += p->ua;
			p->targetPayloadLen = byte;
			p->state = LEN2;
			break;

		case LEN2:
			p->ua += byte;
			p->ub += p->ua;
			p->targetPayloadLen |= ((uint16_t) byte << 8);
			p->payloadIdx = 0;
			if (p->targetPayloadLen > 0 && p->targetPayloadLen <= 256) {
				p->state = PAYLOAD;
			} else if (p->targetPayloadLen == 0) {
				p->state = CK_A;
			} else {
				p->state = IDLE;
			}
			break;

		case PAYLOAD:
			p->ua += byte;
			p->ub += p->ua;
			if (p->classId == 0x01 && p->msgId == 0x07) {
				if (p->payloadIdx == 20) p->fixType = byte;
				else if (p->payloadIdx == 23) p->numSV = byte;
				else if (p->payloadIdx >= 24 && p->payloadIdx <= 27) {
					p->temp[p->payloadIdx - 24] = byte;
					if (p->payloadIdx == 27) p->lon = UBX_BUILD_I32(p->temp);
				} else if (p->payloadIdx >= 28 && p->payloadIdx <= 31) {
					p->temp[p->payloadIdx - 28] = byte;
					if (p->payloadIdx == 31) p->lat = UBX_BUILD_I32(p->temp);
				} else if (p->payloadIdx >= 32 && p->payloadIdx <= 35) {
					p->temp[p->payloadIdx - 32] = byte;
					if (p->payloadIdx == 35) p->height = UBX_BUILD_I32(p->temp);
				} else if (p->payloadIdx >= 40 && p->payloadIdx <= 43) {
					p->temp[p->payloadIdx - 40] = byte;
					if (p->payloadIdx == 43) p->hAcc = UBX_BUILD_U32(p->temp);
				} else if (p->payloadIdx >= 48 && p->payloadIdx <= 51) {
					p->temp[p->payloadIdx - 48] = byte;
					if (p->payloadIdx == 51) p->velN = UBX_BUILD_I32(p->temp);
				} else if (p->payloadIdx >= 52 && p->payloadIdx <= 55) {
					p->temp[p->payloadIdx - 52] = byte;
					if (p->payloadIdx == 55) p->velE = UBX_BUILD_I32(p->temp);
				} else if (p->payloadIdx >= 56 && p->payloadIdx <= 59) {
					p->temp[p->payloadIdx - 56] = byte;
					if (p->payloadIdx == 59) p->velD = UBX_BUILD_I32(p->temp);
				}else if (p->payloadIdx >= 68 && p->payloadIdx <= 71) {
				    p->temp[p->payloadIdx - 68] = byte;
				    if (p->payloadIdx == 71) p->sAcc = UBX_BUILD_U32(p->temp);
				}
			}
			p->payloadIdx++;
			if (p->payloadIdx >= p->targetPayloadLen) {
				p->state = CK_A;
			}
			break;

		case CK_A:
			if (byte == p->ua) {
				p->state = CK_B;
			} else p->state = IDLE;
			break;

		case CK_B:
			if (byte == p->ub) {
				if (p->classId == 0x01 && p->msgId == 0x07) {
					gnssData.fixStatus = (p->fixType >= 3);
					gnssData.satCount  = p->numSV;
					gnssData.latitude  = (double)p->lat * 1e-7;
					gnssData.longitude = (double)p->lon * 1e-7;
					gnssData.altMts    = p->height * 1e-3f;
					gnssData.hAccMts   = p->hAcc * 1e-3f;
					gnssData.velN      = p->velN * 1e-3f;
					gnssData.velE      = p->velE * 1e-3f;
					gnssData.velD      = p->velD * 1e-3f;
					gnssData.sAcc      = p->sAcc * 1e-3f;
					gnssData.msgCount  = gnssData.msgCount + 1;
					if (gnssData.msgCount > 10000) {
						gnssData.msgCount = 0;
					}
					packet_ready = 1;
				}
			}
			p->state = IDLE;
			break;

		default:
			p->state = IDLE;
			break;
		}
	}
	return packet_ready;
}

/**
 * @brief STM32H7 DMA Callback
 * Handles Cache Incoherency before passing to queue.
 */
void _processUBXData(uint8_t *data, uint16_t len) {
	SCB_InvalidateDCache_by_Addr((uint32_t*)data, len);
	circularQueueWrite(&ubxIOQueue, data, len);
}

uint8_t readGNSSData(void) {
	uint16_t available = circularQueueAvailableData(&ubxIOQueue);
	if (available > 0) {
		uint16_t readLen = (available > UBX_READ_BUFFER_SIZE) ? UBX_READ_BUFFER_SIZE : available;
		readLen = circularQueueRead(&ubxIOQueue, ubxCircularQueueReadBuffer, readLen);
		return updateUBXData(&ubxContext, ubxCircularQueueReadBuffer, readLen);
	}
	return 0;
}

uint8_t initGNSS(void) {
	if (uart7Init()) {
		logString("[UBX] UART Init OK\n");
		circularQueueInit(&ubxIOQueue, UBX_CIRCULAR_QUEUE_SIZE);
		// Start DMA RX
		if (uart7ReadStart(ubxDataBuffer, ubxIOReadSize, _processUBXData)) {
			delayMs(100);
			resetGNSS();
			logString("[UBX] RX Start OK\n");
			return 1;
		}
	}
	return 0;
}

void resetGNSS(void) {
	memset((void*) &gnssData, 0, sizeof(GNSS_DATA));
	memset(&ubxContext, 0, sizeof(UBX_CONTEXT));
	ubxContext.state = IDLE;
}
